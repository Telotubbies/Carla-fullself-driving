import gymnasium as gym
from gymnasium import spaces
import numpy as np
import carla
from collections import deque
import cv2
import time
import logging
import threading
from typing import Dict, Tuple, Optional, Any, List, Union
try:
    from .lane_detector import LaneDetector
    LANE_DETECTOR_AVAILABLE = True
except ImportError:
    LANE_DETECTOR_AVAILABLE = False
    logging.warning("LaneDetector not available, will use CARLA API for waypoints")
try:
    import sys
    import os
    sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
    from utils.data_augmentation import DataAugmentation
    DATA_AUGMENTATION_AVAILABLE = True
except ImportError:
    DATA_AUGMENTATION_AVAILABLE = False
    logging.warning("DataAugmentation not available")
class ThreadedOperation:
    
    @staticmethod
    def run(
        func: Any, timeout: float, description: str = "Operation", accept_timeout: bool = False
    ) -> Tuple[bool, Optional[Exception]]:
        
        result = {"success": False, "error": None}
        def wrapper():
            try:
                func()
                result["success"] = True
            except Exception as e:
                result["error"] = e
        thread = threading.Thread(target=wrapper, daemon=True)
        thread.start()
        thread.join(timeout=timeout)
        if thread.is_alive():
            if accept_timeout:
                logging.debug(f"{description} timed out (accepted/ignored).")
                return True, None
            else:
                return False, TimeoutError(f"{description} timed out after {timeout}s")
        return result["success"], result["error"]
class CarlaRLEnv(gym.Env):
    
    metadata = {"render_modes": ["human", "rgb_array"]}
    def __init__(self, config: Dict[str, Any], port: int = None, rank: int = 0):
        super().__init__()
        self.config = config
        self.env_config = config.get("environment", {})
        self.sensor_config = config.get("sensors", {})
        self.obs_config = config.get("observations", {})
        self.action_config = config.get("actions", {})
        self.reward_config = config.get("rewards", {})
        self.curriculum_config = self.env_config.get("curriculum_learning", {})
        self.rank = rank
        self._configure_port(port)
        self.client: Optional[carla.Client] = None
        self.world: Optional[carla.World] = None
        self.client_timeout: float = 60.0
        self.vehicle: Optional[carla.Actor] = None
        self.actors: List[carla.Actor] = []
        self.sensors: Dict[str, Optional[carla.Actor]] = {"rgb": None, "depth": None, "collision": None, "gps": None}
        self.collision_occurred = False
        self.episode_start_time: Optional[float] = None
        self.last_location: Optional[carla.Location] = None
        self.last_steering = 0.0
        self.step_count = 0
        self.current_gps_location: Optional[np.ndarray] = None
        self.goal_location: Optional[carla.Location] = None
        self.distance_to_goal: Optional[float] = None
        self.goal_reached = False
        self.last_lane_id = None
        self.sequence_length = self.obs_config.get("stack_frames", 4)
        self.rgb_buffer = deque(maxlen=self.sequence_length)
        self.depth_buffer = deque(maxlen=self.sequence_length)
        self.current_rgb: Optional[np.ndarray] = None
        self.current_depth: Optional[np.ndarray] = None
        self._init_curriculum()
        self._init_metrics()
        self._init_augmentations()
        self._init_lane_detector()
        self._setup_spaces()
        self._clear_buffers()
    def _configure_port(self, port: Optional[int]):
        
        if port is not None:
            self.env_config["carla_port"] = port
        elif self.rank > 0:
            base_port = self.env_config.get("carla_port", 2000)
            self.env_config["carla_port"] = base_port + (self.rank * 2)
    def _init_curriculum(self):
        
        self.curriculum_enabled = self.curriculum_config.get("enabled", False)
        self.current_difficulty = (
            self.curriculum_config.get("initial_difficulty", 0.3) if self.curriculum_enabled else 1.0
        )
        self.difficulty_increase_rate = self.curriculum_config.get("difficulty_increase_rate", 0.01)
        self.max_difficulty = self.curriculum_config.get("max_difficulty", 1.0)
        self.reward_based_curriculum = self.curriculum_config.get("reward_based", False)
        self.reward_threshold = self.curriculum_config.get("reward_threshold", 0.0)
        self.reward_window_size = self.curriculum_config.get("reward_window_size", 50)
        self.episode_rewards = deque(maxlen=self.reward_window_size)
        self.current_episode_reward = 0.0
        prod_reward_conf = self.reward_config.get("progressive_rewards", {})
        self.progressive_rewards_enabled = prod_reward_conf.get("enabled", False)
        self.reward_scale = (
            prod_reward_conf.get("initial_scale", 1.0)
            if not self.progressive_rewards_enabled
            else prod_reward_conf.get("initial_scale", 0.5)
        )
        self.final_reward_scale = prod_reward_conf.get("final_scale", 1.0)
        self.reward_scale_increase_rate = prod_reward_conf.get("scale_increase_rate", 0.01)
    def _init_metrics(self):
        
        self.episode_metrics = {
            "total_distance": 0.0,
            "lane_keeping_time": 0.0,
            "total_steps": 0,
            "route_completion": 0.0,
            "jerk_sum": 0.0,
            "speed_variance": 0.0,
            "infractions": 0,
            "speeds": [],
        }
    def _init_augmentations(self):
        
        aug_config = self.obs_config.get("augmentation", {})
        self.data_augmentation = None
        if DATA_AUGMENTATION_AVAILABLE and aug_config.get("enabled", False):
            self.data_augmentation = DataAugmentation(aug_config)
            logging.info(
                f"✅ Data augmentation enabled: {
        aug_config.get(
            'methods', [])}"
            )
    def _init_lane_detector(self):
        
        self.use_vision_waypoint = self.obs_config.get("use_vision_waypoint", False)
        self.lane_detector = None
        if self.use_vision_waypoint:
            if LANE_DETECTOR_AVAILABLE:
                method = self.obs_config.get("edge_detection_method", "multiscale_canny")
                img_size = self.obs_config.get("image_size", [160, 90])
                self.lane_detector = LaneDetector(img_size[0], img_size[1], edge_detection_method=method)
                logging.info(f"✅ Using vision-based lane detection: {method}")
            else:
                logging.warning("⚠️ Vision waypoint requested but LaneDetector missing. Fallback to CARLA API.")
            self.use_vision_waypoint = False
    def _setup_spaces(self):
        
        image_size = self.obs_config.get("image_size", [160, 90])
        use_depth = self.obs_config.get("use_depth", True)
        channels = 4 if use_depth else 3
        vision_shape = (self.sequence_length, image_size[1], image_size[0], channels)
        self.use_gps = self.obs_config.get("use_gps", False)
        self.use_goal = self.obs_config.get("use_goal", False)
        self.use_waypoint = self.obs_config.get("use_waypoint", True)
        self.use_velocity = self.obs_config.get("use_velocity", True)
        is_dict_obs = any([self.use_gps, self.use_goal, self.use_waypoint, self.use_velocity])
        if is_dict_obs:
            space_dict = {"vision": spaces.Box(low=0.0, high=1.0, shape=vision_shape, dtype=np.float32)}
            if self.use_gps:
                space_dict["gps"] = spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32)
            if self.use_goal:
                space_dict["goal"] = spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32)
                space_dict["distance_to_goal"] = spaces.Box(low=0.0, high=np.inf, shape=(1,), dtype=np.float32)
            if self.use_waypoint:
                space_dict["waypoint"] = spaces.Box(low=-1.0, high=1.0, shape=(8,), dtype=np.float32)
            if self.use_velocity:
                space_dict["velocity"] = spaces.Box(low=-1.0, high=1.0, shape=(5,), dtype=np.float32)
            self.observation_space = spaces.Dict(space_dict)
        else:
            self.observation_space = spaces.Box(low=0.0, high=1.0, shape=vision_shape, dtype=np.float32)
        self.action_space = spaces.Box(
            low=np.array([-1.0, 0.0, 0.0], dtype=np.float32),
            high=np.array([1.0, 1.0, 1.0], dtype=np.float32),
            dtype=np.float32,
        )
    def _clear_buffers(self):
        
        self.rgb_buffer.clear()
        self.depth_buffer.clear()
        img_size = self.obs_config.get("image_size", [160, 90])
        zero_rgb = np.zeros((img_size[1], img_size[0], 3), dtype=np.float32)
        zero_depth = np.zeros((img_size[1], img_size[0], 1), dtype=np.float32)
        for _ in range(self.sequence_length):
            self.rgb_buffer.append(zero_rgb)
            self.depth_buffer.append(zero_depth)
    def _connect_to_carla(self) -> bool:
        
        host = self.env_config.get("carla_host", "localhost")
        port = self.env_config.get("carla_port", 2000)
        timeout = self.env_config.get("timeout", 120.0)
        town = self.env_config.get("town", "Town01")
        max_retries = 20
        logging.info(f"Connecting to CARLA at {host}:{port}...")
        for attempt in range(max_retries):
            try:
                self.client = carla.Client(host, port)
                self.client_timeout = 120.0 if self.rank > 0 else max(timeout, 60.0)
                self.client.set_timeout(self.client_timeout)
                if not self._load_world(town):
                    raise RuntimeError("Failed to load world")
                break
            except (RuntimeError, Exception) as e:
                if attempt == max_retries - 1:
                    logging.error(f"Failed to connect after {max_retries} attempts: {e}")
                    return False
                wait_time = min(5 + (attempt * 2), 15)
                logging.warning(f"Connection failed ({e}). Retrying in {wait_time}s...")
                time.sleep(wait_time)
        try:
            self._apply_world_settings()
            logging.info(f"✅ Connected to CARLA at {host}:{port}")
            return True
        except Exception as e:
            logging.error(f"Failed to apply world settings: {e}")
            return False
    def _load_world(self, requested_town: str) -> bool:
        
        try:
            if not self.client:
                logging.error("Cannot load world: client not initialized")
                return False
            self.world = self.client.load_world(requested_town, reset_settings=False)
            logging.info(f"✅ Loaded town: {requested_town}")
            return True
        except RuntimeError as e:
            try:
                self.world = self.client.get_world()
                current_map = self.world.get_map().name
                if current_map != requested_town:
                    logging.info(f"Loading requested town {requested_town}...")
                    self.world = self.client.load_world(requested_town, reset_settings=False)
                    logging.info(f"✅ Loaded town: {requested_town}")
                else:
                    logging.info(f"Using current town: {requested_town}")
                return True
            except Exception as e2:
                logging.error(f"Failed to load world: {e2}")
                return False
        except Exception as e:
            logging.error(f"Error loading world: {e}")
            return False
    def _apply_world_settings(self):
        
        if not self.world:
            return
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        settings.no_rendering_mode = self.env_config.get("no_rendering_mode", False)
        logging.info(
            f"🎨 No Rendering Mode: {
        'ENABLED' if settings.no_rendering_mode else 'DISABLED'}"
        )
        self.world.apply_settings(settings)
        self._setup_weather()
    def _setup_weather(self):
        
        domain_rand = self.env_config.get("domain_randomization", {})
        if domain_rand.get("enabled", False) and domain_rand.get("weather_randomization", False):
            import random
            ranges = domain_rand.get("weather", {})
            weather = carla.WeatherParameters(
                cloudiness=random.uniform(*ranges.get("cloudiness_range", [0, 0])),
                precipitation=random.uniform(*ranges.get("precipitation_range", [0, 0])),
                sun_altitude_angle=random.uniform(*ranges.get("sun_altitude_range", [45, 45])),
                fog_density=random.uniform(*ranges.get("fog_density_range", [0, 0])),
            )
        else:
            w_conf = self.env_config.get("weather", {})
            weather = carla.WeatherParameters(
                cloudiness=w_conf.get("cloudiness", 0.0),
                precipitation=w_conf.get("precipitation", 0.0),
                sun_altitude_angle=w_conf.get("sun_altitude_angle", 45.0),
            )
        self.world.set_weather(weather)
    def _spawn_traffic_manager(self):
        
        self.actors = []
        if self.curriculum_enabled:
            v_range = self.curriculum_config.get("num_vehicles", [0, 10])
            p_range = self.curriculum_config.get("num_pedestrians", [0, 5])
            num_vehicles = int(v_range[0] + (v_range[1] - v_range[0]) * self.current_difficulty)
            num_pedestrians = int(p_range[0] + (p_range[1] - p_range[0]) * self.current_difficulty)
            enable_traffic = self.curriculum_config.get("enable_traffic", False) and (self.current_difficulty > 0.5)
        else:
            num_vehicles = self.env_config.get("num_vehicles", 0)
            num_pedestrians = self.env_config.get("num_pedestrians", 0)
            enable_traffic = self.env_config.get("enable_traffic", False)
        if num_vehicles > 0 or num_pedestrians > 0:
            self._spawn_obstacles(num_vehicles, num_pedestrians, enable_traffic)
    def _spawn_obstacles(self, n_vehicles: int, n_walkers: int, enable_autopilot: bool):
        
        import random
        spawn_points = self.world.get_map().get_spawn_points()
        random.shuffle(spawn_points)
        bp_lib = self.world.get_blueprint_library()
        vehicle_bps = [bp for bp in bp_lib.filter("vehicle.*") if int(bp.get_attribute("number_of_wheels")) == 4]
        for sp in spawn_points[:n_vehicles]:
            try:
                bp = random.choice(vehicle_bps)
                actor = self.world.try_spawn_actor(bp, sp)
                if actor:
                    self.actors.append(actor)
                    if enable_autopilot and self.rank == 0:
                        try:
                            actor.set_autopilot(True)
                        except RuntimeError:
                            pass
            except Exception:
                pass
        walker_bps = bp_lib.filter("walker.pedestrian.*")
        for sp in self.world.get_map().get_spawn_points()[: n_walkers * 3]:
            if len([a for a in self.actors if a.type_id.startswith("walker")]) >= n_walkers:
                break
            try:
                bp = random.choice(walker_bps)
                trans = carla.Transform(location=sp.location, rotation=sp.rotation)
                actor = self.world.try_spawn_actor(bp, trans)
                if actor:
                    self.actors.append(actor)
                    try:
                        con_bp = bp_lib.find("controller.ai.walker")
                        controller = self.world.spawn_actor(con_bp, carla.Transform(), actor)
                        controller.start()
                        controller.go_to_location(random.choice(spawn_points).location)
                        self.actors.append(controller)
                    except:
                        pass
            except:
                pass
    def _spawn_agent(self) -> bool:
        max_retries = 3
        for retry in range(max_retries):
            try:
                if not self.world:
                    logging.error("Cannot spawn agent: world not initialized")
                    return False
                if not self.client:
                    logging.error("Cannot spawn agent: client not initialized")
                    return False
                self.client.set_timeout(self.client_timeout)
                bp_lib = self.world.get_blueprint_library()
                domain_rand = self.env_config.get("domain_randomization", {})
                if domain_rand.get("enabled", False) and domain_rand.get("vehicle_randomization", False):
                    import random
                    bp_name = random.choice(domain_rand.get("vehicle_blueprints", ["vehicle.tesla.model3"]))
                else:
                    bp_name = self.env_config.get("vehicle", {}).get("blueprint", "vehicle.tesla.model3")
                vehicle_bp = bp_lib.find(bp_name)
                if not vehicle_bp:
                    logging.error(f"Vehicle blueprint '{bp_name}' not found")
                    return False
                spawn_points = self.world.get_map().get_spawn_points()
                if not spawn_points:
                    logging.error("No spawn points available")
                    return False
                import random
                random.shuffle(spawn_points)
                for sp in spawn_points[:10]:
                    try:
                        self.vehicle = self.world.try_spawn_actor(vehicle_bp, sp)
                        if self.vehicle:
                            self.actors.append(self.vehicle)
                            logging.debug(f"Vehicle spawned successfully at {sp.location}")
                            break
                    except Exception as e:
                        logging.debug(f"Failed to spawn at {sp.location}: {e}")
                        continue
                if not self.vehicle:
                    logging.error("Failed to spawn vehicle after trying multiple spawn points")
                    return False
                try:
                    self.vehicle.set_simulate_physics(True)
                    self.vehicle.set_autopilot(False)
                except Exception as e:
                    logging.warning(f"Vehicle config warning: {e}")
                return True
            except Exception as e:
                logging.warning(f"Error in _spawn_agent (attempt {retry+1}/{max_retries}): {e}")
                if retry < max_retries - 1:
                    time.sleep(2.0)
                    try:
                        self.client.set_timeout(self.client_timeout)
                        self.world = self.client.get_world()
                    except Exception as reconnect_error:
                        logging.warning(f"Reconnection attempt failed: {reconnect_error}")
                else:
                    logging.error(f"Error in _spawn_agent after {max_retries} retries: {e}", exc_info=True)
                    return False
        return False
    def _reset_goal(self):
        
        if not self.vehicle:
            return
        import random
        spawn_points = self.world.get_map().get_spawn_points()
        current_loc = self.vehicle.get_location()
        valid_goals = [sp.location for sp in spawn_points if sp.location.distance(current_loc) > 50.0]
        if valid_goals:
            self.goal_location = random.choice(valid_goals)
        else:
            fwd = self.vehicle.get_transform().get_forward_vector()
            self.goal_location = current_loc + carla.Location(fwd.x * 100, fwd.y * 100, 0)
        self.distance_to_goal = current_loc.distance(self.goal_location)
        logging.info(
            f"🎯 Goal set at {
        self.goal_location}, distance: {
            self.distance_to_goal:.1f}m"
        )
    def _setup_sensors(self) -> bool:
        
        try:
            if not self.world or not self.vehicle:
                logging.error("Cannot setup sensors: world or vehicle not initialized")
                return False
            bp_lib = self.world.get_blueprint_library()
            sensors_attached = 0
            def attach(type_id, transform, callback, config_key):
                nonlocal sensors_attached
                if not self.sensor_config.get(config_key, {}).get("enabled", True):
                    return None
                try:
                    bp = bp_lib.find(type_id)
                    if not bp:
                        logging.warning(f"Sensor blueprint '{type_id}' not found")
                        return None
                    conf = self.sensor_config[config_key]
                    if "camera" in type_id:
                        bp.set_attribute("image_size_x", str(conf["width"]))
                        bp.set_attribute("image_size_y", str(conf["height"]))
                        bp.set_attribute("fov", str(conf["fov"]))
                    loc = carla.Location(*conf.get("location", [0, 0, 0]))
                    rot = carla.Rotation(*conf.get("rotation", [0, 0, 0]))
                    sensor = self.world.spawn_actor(bp, carla.Transform(loc, rot), attach_to=self.vehicle)
                    sensor.listen(callback)
                    self.actors.append(sensor)
                    sensors_attached += 1
                    return sensor
                except Exception as e:
                    logging.warning(f"Failed to attach sensor {type_id}: {e}")
                    return None
            self.sensors["rgb"] = attach("sensor.camera.rgb", None, self._process_rgb_image, "rgb_camera")
            if not self.sensors["rgb"]:
                logging.error("Failed to attach RGB camera (required)")
                return False
            self.sensors["depth"] = attach("sensor.camera.depth", None, self._process_depth_image, "depth_camera")
            gps_conf = self.sensor_config.get("gps", {})
            if gps_conf.get("enabled", False):
                try:
                    bp = bp_lib.find("sensor.other.gnss")
                    if bp:
                        loc = carla.Location(*gps_conf.get("location", [1.0, 0.0, 2.8]))
                        s = self.world.spawn_actor(bp, carla.Transform(loc), attach_to=self.vehicle)
                        s.listen(lambda d: self._process_gps(d))
                        self.sensors["gps"] = s
                        self.actors.append(s)
                        sensors_attached += 1
                except Exception as e:
                    logging.warning(f"Failed to attach GPS sensor: {e}")
            try:
                bp = bp_lib.find("sensor.other.collision")
                if bp:
                    s = self.world.spawn_actor(bp, carla.Transform(), attach_to=self.vehicle)
                    s.listen(lambda e: self._on_collision(e))
                    self.sensors["collision"] = s
                    self.actors.append(s)
                    sensors_attached += 1
            except Exception as e:
                logging.warning(f"Failed to attach collision sensor: {e}")
            logging.debug(f"✅ Sensors setup complete: {sensors_attached} sensors attached")
            return sensors_attached > 0
        except Exception as e:
            logging.error(f"Error in _setup_sensors: {e}", exc_info=True)
            return False
    def _process_rgb_image(self, image):
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))[:, :, :3]
        target_size = tuple(self.obs_config.get("image_size", [160, 90]))
        if (array.shape[1], array.shape[0]) != target_size:
            array = cv2.resize(array, target_size)
        if self.obs_config.get("normalize", True):
            array = array.astype(np.float32) / 255.0
        self.current_rgb = array
    def _process_depth_image(self, image):
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))[:, :, :3]
        depth = (array[:, :, 0] + array[:, :, 1] * 256.0 + array[:, :, 2] * 65536.0) / (16777215.0)
        depth = depth * 1000.0
        target_size = tuple(self.obs_config.get("image_size", [160, 90]))
        if (depth.shape[1], depth.shape[0]) != target_size:
            depth = cv2.resize(depth, target_size)
        depth = np.clip(depth / 100.0, 0.0, 1.0)
        self.current_depth = np.expand_dims(depth, axis=-1).astype(np.float32)
    def _process_gps(self, data):
        if self.vehicle:
            loc = self.vehicle.get_location()
            self.current_gps_location = np.array([loc.x, loc.y, loc.z], dtype=np.float32)
    def _on_collision(self, event):
        self.collision_occurred = True
    def reset(self, seed: Optional[int] = None, options: Optional[Dict] = None) -> Tuple[np.ndarray, Dict]:
        
        max_retries = 3
        retry_count = 0
        while retry_count < max_retries:
            try:
                logging.debug(
                    f"Environment reset attempt {
        retry_count + 1}/{max_retries}..."
                )
                super().reset(seed=seed)
                self.step_count = 0
                try:
                    self._destroy_all_actors()
                except Exception as e:
                    logging.warning(f"Error during actor cleanup (non-fatal): {e}")
                if not self.client:
                    success = self._connect_to_carla()
                    if not success:
                        raise ConnectionError("Failed to connect to CARLA")
                if not self.world:
                    success = self._load_world(self.env_config.get("town", "Town01"))
                    if not success:
                        raise RuntimeError("Failed to load world")
                self._spawn_traffic_manager()
                if not self._spawn_agent():
                    raise RuntimeError("Failed to spawn agent")
                if not self._setup_sensors():
                    raise RuntimeError("Failed to setup sensors")
                self.collision_occurred = False
                self.episode_start_time = time.time()
                self.last_location = None
                self.last_steering = 0.0
                self.goal_reached = False
                self.distance_to_goal = None
                self.current_episode_reward = 0.0
                self._clear_buffers()
                self._init_metrics()
                if self.use_goal:
                    self._reset_goal()
                for _ in range(5):
                    if not self._tick_world_safely(retries=3, timeout=5.0):
                        raise RuntimeError("Failed to tick world during warm-up")
                obs = self._compute_observation()
                if obs is None:
                    raise ValueError("Failed to compute observation")
                logging.debug("Environment reset successful")
                return obs, {}
            except Exception as e:
                retry_count += 1
                logging.warning(f"Environment reset failed (attempt {retry_count}/{max_retries}): {e}")
                if retry_count >= max_retries:
                    logging.error(f"Environment reset failed after {max_retries} attempts")
                    return self._get_zero_observation(), {"error": str(e), "reset_failed": True}
                time.sleep(0.5 * retry_count)
        return self._get_zero_observation(), {"error": "reset_failed", "reset_failed": True}
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        
        self.step_count += 1
        step_start_time = time.time()
        info = {}
        try:
            if action is None:
                logging.warning("Received None action, using zero action")
                action = np.zeros(3, dtype=np.float32)
            if not self._apply_vehicle_control(action):
                error_info = {"error": "control_failed", "step": self.step_count}
                logging.warning(f"Control failed at step {self.step_count}")
                return self._get_zero_observation(), -10.0, True, False, error_info
            repeat = self.action_config.get("action_repeat", 4)
            tick_failures = 0
            for i in range(repeat):
                if not self._tick_world_safely(retries=3, timeout=10.0):
                    tick_failures += 1
                    if tick_failures >= 3:
                        error_info = {
                            "error": "world_tick_failed",
                            "step": self.step_count,
                            "tick_failures": tick_failures,
                        }
                        logging.warning(
                            f"World tick failed {tick_failures} times at step {self.step_count}"
                        )
                        return self._get_zero_observation(), -10.0, True, False, error_info
                else:
                    tick_failures = 0
            obs = self._compute_observation()
            if obs is None:
                error_info = {"error": "observation_failed", "step": self.step_count}
                logging.warning(
                    f"Observation computation failed at step {
        self.step_count}"
                )
                return self._get_zero_observation(), -5.0, True, False, error_info
            reward = self._compute_reward_safe()
            done = self._check_done()
            step_duration = time.time() - step_start_time
            self._update_metrics(action, reward)
            if self.reward_based_curriculum:
                self.current_episode_reward += reward
                if done:
                    self.episode_rewards.append(self.current_episode_reward)
            info = self._compile_info(reward, done)
            info["step_duration"] = step_duration
            info["step_count"] = self.step_count
            if step_duration > 0.1:
                info["performance_warning"] = True
                logging.debug(
                    f"Slow step detected: {
        step_duration:.3f}s at step {
            self.step_count}"
                )
            return obs, reward, done, False, info
        except Exception as e:
            logging.error(
                f"Error in step() at step {
        self.step_count}: {e}",
                exc_info=True,
            )
            error_info = {"error": str(e), "step": self.step_count, "exception": type(e).__name__}
            return self._get_zero_observation(), -10.0, True, False, error_info
    def _apply_vehicle_control(self, action) -> bool:
        
        try:
            if not isinstance(action, np.ndarray):
                action = np.array(action)
            if action.ndim == 0:
                action = np.array([float(action)])
            elif action.ndim > 1:
                action = action.flatten()
            if len(action) < 3:
                action = np.pad(action, (0, 3 - len(action)), mode='constant', constant_values=0.0)
            else:
                action = action[:3]
            if not self.vehicle or not self.vehicle.is_alive:
                return False
            ctrl = carla.VehicleControl(
                steer=float(np.clip(action[0], -1.0, 1.0)),
                throttle=float(np.clip(action[1], 0.0, 1.0)),
                brake=float(np.clip(action[2], 0.0, 1.0)),
            )
            self.vehicle.apply_control(ctrl)
            return True
        except Exception as e:
            logging.warning(f"Control error: {e}")
            return False
    def _tick_world_safely(self, retries: int = 3, timeout: float = 10.0) -> bool:
        if not self.world:
            return False
        for i in range(retries + 1):
            success, error = ThreadedOperation.run(self.world.tick, timeout, f"Tick-{self.step_count}-{i}")
            if success:
                return True
            if i < retries:
                timeout *= 1.5
                logging.debug(f"World tick retry {i+1}/{retries} with timeout {timeout:.1f}s")
        else:
            logging.error(f"World tick failed after {retries} retries: {error}")
            return False
        return False
    def _compute_observation(self) -> Any:
        
        start = time.time()
        while (self.current_rgb is None or self.current_depth is None) and (time.time() - start < 0.1):
            pass
        if self.current_rgb is None:
            s = self.obs_config.get("image_size", [160, 90])
            self.current_rgb = np.zeros((s[1], s[0], 3), dtype=np.float32)
        if self.current_depth is None:
            s = self.obs_config.get("image_size", [160, 90])
            self.current_depth = np.zeros((s[1], s[0], 1), dtype=np.float32)
        rgb = self.current_rgb
        if self.data_augmentation and rgb is not None:
            rgb = self.data_augmentation.augment_image(rgb)
        self.rgb_buffer.append(rgb)
        self.depth_buffer.append(self.current_depth)
        rgb_stack = np.stack(self.rgb_buffer, axis=0)
        depth_stack = np.stack(self.depth_buffer, axis=0)
        vision = rgb_stack
        if self.obs_config.get("use_depth", True):
            vision = np.concatenate([rgb_stack, depth_stack], axis=-1)
        if isinstance(self.observation_space, spaces.Dict):
            obs = {"vision": vision.astype(np.float32)}
            if self.use_gps:
                obs["gps"] = (
                    self.current_gps_location
                    if self.current_gps_location is not None
                    else np.zeros((3,), dtype=np.float32)
                )
            if self.use_goal:
                obs["goal"] = (
                    np.array([self.goal_location.x, self.goal_location.y, self.goal_location.z])
                    if self.goal_location
                    else np.zeros(3)
                )
                obs["distance_to_goal"] = (
                    np.array([self.distance_to_goal]) if self.distance_to_goal else np.array([0.0])
                )
            if self.use_waypoint:
                obs["waypoint"] = self._get_waypoint_features()
            if self.use_velocity:
                obs["velocity"] = self._get_velocity_features()
            if self.step_count % 50 == 0 and rgb is not None:
                self._save_snapshot(rgb)
            return obs
        return vision.astype(np.float32)
    def _save_snapshot(self, rgb_array):
        
        try:
            if rgb_array.dtype == np.float32:
                img = (rgb_array * 255).astype(np.uint8)
            else:
                img = rgb_array.astype(np.uint8)
            img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            snapshot_path = os.path.join(self.config.get("log_dir", "logs"), "live_camera.jpg")
            temp_path = snapshot_path + ".tmp"
            cv2.imwrite(temp_path, img_bgr)
            os.replace(temp_path, snapshot_path)
        except Exception:
            pass
    def _get_waypoint_features(self):
        if not self.vehicle:
            return np.zeros(8, dtype=np.float32)
        if self.use_vision_waypoint and self.lane_detector and self.current_rgb is not None:
            try:
                lf = self.lane_detector.process_image(self.current_rgb)
                return np.array(
                    [
                        lf["waypoint_dx"],
                        lf["waypoint_dy"],
                        0.0,
                        abs(lf["waypoint_dy"]),
                        lf["curvature"],
                        lf["lane_change_left"],
                        lf["lane_change_right"],
                        lf["lane_center_offset"],
                    ],
                    dtype=np.float32,
                )
            except:
                pass
        try:
            t = self.vehicle.get_transform()
            wp = self.world.get_map().get_waypoint(t.location)
            nwp = wp.next(5.0)[0] if wp.next(5.0) else wp
            v_loc = t.location
            w_loc = nwp.transform.location
            dx, dy, dz = w_loc.x - v_loc.x, w_loc.y - v_loc.y, w_loc.z - v_loc.z
            yaw = np.radians(t.rotation.yaw)
            rx = dx * np.cos(yaw) + dy * np.sin(yaw)
            ry = -dx * np.sin(yaw) + dy * np.cos(yaw)
            dist = np.sqrt(dx**2 + dy**2 + dz**2)
            curv = np.radians(nwp.transform.rotation.yaw) - yaw
            curv = np.arctan2(np.sin(curv), np.cos(curv))
            lc = wp.lane_change
            lc_l = 1.0 if (lc == carla.LaneChange.Left or lc == carla.LaneChange.Both) else 0.0
            lc_r = 1.0 if (lc == carla.LaneChange.Right or lc == carla.LaneChange.Both) else 0.0
            return np.array(
                [
                    np.clip(rx / 10, -1, 1),
                    np.clip(ry / 10, -1, 1),
                    np.clip(dz / 5, -1, 1),
                    np.clip(dist / 20, 0, 1),
                    np.clip(curv / np.pi, -1, 1),
                    lc_l,
                    lc_r,
                    np.clip(curv / np.pi, -1, 1),
                ],
                dtype=np.float32,
            )
        except:
            return np.zeros(8, dtype=np.float32)
    def _get_velocity_features(self):
        if not self.vehicle:
            return np.zeros(5, dtype=np.float32)
        try:
            v = self.vehicle.get_velocity()
            speed_ms = np.sqrt(v.x**2 + v.y**2 + v.z**2)
            return np.array(
                [
                    np.clip((speed_ms * 3.6) / 100, 0, 1),
                    np.clip(v.x / 15, -1, 1),
                    np.clip(v.y / 15, -1, 1),
                    np.clip(v.z / 5, -1, 1),
                    np.clip(speed_ms / 15, 0, 1),
                ],
                dtype=np.float32,
            )
        except:
            return np.zeros(5, dtype=np.float32)
    def _compute_safety_potential(self) -> float:
        """
        Compute Safety Potential Function based on MDPI paper approach.
        This function provides proactive risk assessment by evaluating:
        - Distance to nearby vehicles/obstacles
        - Relative velocity (closing speed)
        - Time-to-collision (TTC) prediction
        
        Returns:
            float: Safety potential value (higher = more risk, lower = safer)
        """
        if not self.vehicle or not self.world:
            return 0.0
        
        try:
            # Get ego vehicle state
            ego_transform = self.vehicle.get_transform()
            ego_location = ego_transform.location
            ego_velocity = self.vehicle.get_velocity()
            ego_speed = np.sqrt(ego_velocity.x**2 + ego_velocity.y**2 + ego_velocity.z**2)
            
            # Get configuration parameters
            max_detection_range = self.reward_config.get("safety_potential_max_range", 50.0)
            safe_distance = self.reward_config.get("safe_distance", 10.0)
            critical_distance = self.reward_config.get("critical_distance", 5.0)
            alpha = self.reward_config.get("safety_potential_alpha", 2.0)  # Distance decay factor
            beta = self.reward_config.get("safety_potential_beta", 1.5)  # Velocity factor
            min_ttc = self.reward_config.get("safety_potential_min_ttc", 2.0)  # Minimum TTC threshold (seconds)
            
            total_potential = 0.0
            nearby_count = 0
            
            # Get all vehicles in the world
            all_actors = self.world.get_actors()
            vehicles = [actor for actor in all_actors if actor.type_id.startswith("vehicle.") and actor.id != self.vehicle.id]
            
            for vehicle in vehicles:
                try:
                    # Get vehicle state
                    vehicle_location = vehicle.get_location()
                    vehicle_velocity = vehicle.get_velocity()
                    vehicle_speed = np.sqrt(vehicle_velocity.x**2 + vehicle_velocity.y**2 + vehicle_velocity.z**2)
                    
                    # Calculate distance
                    distance = ego_location.distance(vehicle_location)
                    
                    if distance > max_detection_range:
                        continue
                    
                    # Calculate relative position vector (from ego to other vehicle)
                    relative_pos = carla.Location(
                        vehicle_location.x - ego_location.x,
                        vehicle_location.y - ego_location.y,
                        vehicle_location.z - ego_location.z
                    )
                    relative_distance = np.sqrt(relative_pos.x**2 + relative_pos.y**2 + relative_pos.z**2)
                    
                    # Calculate relative velocity vector
                    relative_vel = carla.Vector3D(
                        vehicle_velocity.x - ego_velocity.x,
                        vehicle_velocity.y - ego_velocity.y,
                        vehicle_velocity.z - ego_velocity.z
                    )
                    relative_speed = np.sqrt(relative_vel.x**2 + relative_vel.y**2 + relative_vel.z**2)
                    
                    # Calculate direction vector (normalized)
                    if relative_distance > 0.1:  # Avoid division by zero
                        direction = carla.Vector3D(
                            relative_pos.x / relative_distance,
                            relative_pos.y / relative_distance,
                            relative_pos.z / relative_distance
                        )
                        
                        # Calculate closing speed (negative = approaching, positive = receding)
                        closing_speed = -(
                            direction.x * relative_vel.x +
                            direction.y * relative_vel.y +
                            direction.z * relative_vel.z
                        )
                        
                        # Calculate Time-to-Collision (TTC)
                        if closing_speed > 0.1:  # Only if approaching
                            ttc = relative_distance / closing_speed
                        else:
                            ttc = float('inf')  # Receding or stationary
                        
                        # Safety Potential Function (based on MDPI paper approach)
                        # SP = exp(-distance/safe_distance) * (1 + relative_speed_factor) * (1 + ttc_factor)
                        
                        # Distance component: exponential decay with distance
                        distance_factor = np.exp(-relative_distance / safe_distance)
                        
                        # Velocity component: higher relative speed = higher risk
                        # Normalize relative speed (0-1 range, assuming max 100 km/h = ~27.8 m/s)
                        max_relative_speed = 30.0  # m/s
                        normalized_relative_speed = min(relative_speed / max_relative_speed, 1.0)
                        velocity_factor = 1.0 + beta * normalized_relative_speed
                        
                        # TTC component: lower TTC = higher risk
                        if ttc < min_ttc:
                            ttc_factor = 1.0 + (min_ttc - ttc) / min_ttc  # Exponential increase as TTC decreases
                        else:
                            ttc_factor = 1.0 / (1.0 + ttc - min_ttc)  # Decrease as TTC increases
                        
                        # Combine components
                        potential = distance_factor * velocity_factor * ttc_factor
                        
                        # Weight by distance (closer = more important)
                        weight = np.exp(-relative_distance / (alpha * safe_distance))
                        total_potential += potential * weight
                        nearby_count += 1
                        
                except Exception as e:
                    logging.debug(f"Error computing safety potential for vehicle {vehicle.id}: {e}")
                    continue
            
            # Normalize by number of nearby vehicles (avoid scaling issues)
            if nearby_count > 0:
                total_potential = total_potential / nearby_count
            
            return total_potential
            
        except Exception as e:
            logging.warning(f"Error in _compute_safety_potential: {e}")
            return 0.0
    
    def _compute_reward_safe(self) -> float:
        try:
            return self._compute_reward_impl()
        except Exception as e:
            logging.error(f"Reward error: {e}")
            return 0.0
    def _compute_reward_impl(self) -> float:
        if not self.vehicle:
            return 0.0
        t = self.vehicle.get_transform()
        v = self.vehicle.get_velocity()
        speed_kmh = 3.6 * np.sqrt(v.x**2 + v.y**2 + v.z**2)
        wp = self.world.get_map().get_waypoint(t.location)
        dist_center = t.location.distance(wp.transform.location)
        tolerance = self.reward_config.get("lane_center_tolerance", 0.5)
        reward = self.reward_config.get("lane_center_reward", 1.0) * max(0, 1.0 - dist_center / tolerance)
        t_speed = self.reward_config.get("target_speed", 70.0)
        w_speed = self.reward_config.get("speed_reward", 0.5)
        if 40 <= speed_kmh <= 100:
            if 50 <= speed_kmh <= 90:
                reward += w_speed
            elif 45 <= speed_kmh < 50 or 90 < speed_kmh <= 95:
                reward += w_speed * 0.8
            else:
                reward += w_speed * 0.5
        elif speed_kmh < 40:
            reward -= self.reward_config.get("low_speed_penalty", 0.1) * (40 - speed_kmh) / 40
        else:
            reward -= self.reward_config.get("high_speed_penalty", 0.2) * (speed_kmh - 100) / 100
        if self.last_location:
            prog = t.location.distance(self.last_location)
            max_progress_per_step = 2.0
            normalized_prog = min(prog / max_progress_per_step, 1.0)
            progress_coeff = self.reward_config.get("progress_reward", 2.0)
            reward += progress_coeff * normalized_prog
        steer = self.vehicle.get_control().steer
        reward += self.reward_config.get("smooth_steering_reward", 0.3) * (1.0 - abs(steer - self.last_steering))
        if self.collision_occurred:
            reward += self.reward_config.get("collision_penalty", -50.0)
        if self.use_goal and self.goal_location:
            dist = t.location.distance(self.goal_location)
            max_goal_penalty_distance = 50.0
            normalized_dist = min(dist, max_goal_penalty_distance) / max_goal_penalty_distance
            reward += -self.reward_config.get("goal_distance_reward", 0.1) * normalized_dist
            if dist < self.reward_config.get("goal_reached_threshold", 5.0) and not self.goal_reached:
                reward += self.reward_config.get("goal_reached_reward", 100.0)
                self.goal_reached = True
        off_lane_threshold = self.reward_config.get("off_lane_threshold", tolerance * 3)
        if dist_center > off_lane_threshold:
            reward += self.reward_config.get("off_lane_penalty", -10.0)
        
        # Safety Potential Function (MDPI Paper Approach)
        # Proactive risk assessment - penalize risky situations before collision occurs
        safety_potential_enabled = self.reward_config.get("safety_potential_enabled", True)
        if safety_potential_enabled:
            safety_potential = self._compute_safety_potential()
            safety_penalty_weight = self.reward_config.get("safety_potential_penalty_weight", 5.0)
            # Higher potential = higher risk = larger penalty
            reward -= safety_potential * safety_penalty_weight
        
        if self.progressive_rewards_enabled:
            reward *= self.reward_scale
        return reward
    def _check_done(self) -> bool:
        if self.collision_occurred:
            return True
        if self.episode_start_time and (time.time() - self.episode_start_time > self.env_config.get("timeout", 20.0)):
            return True
        return False
    def _compile_info(self, reward, done) -> Dict:
        speed = 0.0
        if self.vehicle:
            v = self.vehicle.get_velocity()
            speed = 3.6 * np.sqrt(v.x**2 + v.y**2 + v.z**2)
        return {
            "collision": self.collision_occurred,
            "speed": speed,
            "total_distance": self.episode_metrics["total_distance"],
            "lane_keeping_ratio": self.episode_metrics["lane_keeping_time"]
            / max(self.episode_metrics["total_steps"], 1),
        }
    def _update_metrics(self, action, reward):
        self.episode_metrics["total_steps"] += 1
        if not self.vehicle:
            return
        try:
            curr = self.vehicle.get_transform().location
            if self.last_location:
                self.episode_metrics["total_distance"] += curr.distance(self.last_location)
            self.last_location = curr
            self.last_steering = self.vehicle.get_control().steer
            wp = self.world.get_map().get_waypoint(curr)
            if curr.distance(wp.transform.location) < self.reward_config.get("lane_center_tolerance", 0.5):
                self.episode_metrics["lane_keeping_time"] += 1
        except:
            pass
    def _get_zero_observation(self):
        
        if isinstance(self.observation_space, spaces.Dict):
            d = {}
            for k, s in self.observation_space.spaces.items():
                if isinstance(s, spaces.Box):
                    d[k] = np.zeros(s.shape, dtype=s.dtype)
            return d
        return np.zeros(self.observation_space.shape, dtype=self.observation_space.dtype)
    def _destroy_all_actors(self):
        
        snapshot = list(self.actors) + [self.vehicle] + list(self.sensors.values())
        snapshot = [a for a in snapshot if a is not None and hasattr(a, "id")]
        seen = set()
        unique = []
        for a in snapshot:
            if a.id not in seen:
                unique.append(a)
                seen.add(a.id)
        for a in unique:
            if hasattr(a, "stop"):
                try:
                    a.stop()
                    time.sleep(0.01)
                except Exception as e:
                    logging.debug(f"Error stopping sensor {a.id if hasattr(a, 'id') else 'unknown'}: {e}")
                    pass
        for a in unique:
            try:
                if hasattr(a, "destroy"):
                    a.destroy()
                elif self.client:
                    cmd = carla.command.DestroyActor(a)
                    self.client.apply_batch([cmd])
            except Exception as e:
                logging.debug(f"Error destroying actor {a.id if hasattr(a, 'id') else 'unknown'}: {e}")
                pass
        if self.client and unique:
            try:
                remaining = [a for a in unique if (hasattr(a, 'is_alive') and a.is_alive) or not hasattr(a, 'is_alive')]
                if remaining:
                    cmd = carla.command.DestroyActor
                    batch = [cmd(a) for a in remaining]
                    self.client.apply_batch(batch)
            except Exception as e:
                logging.debug(f"Batch destroy failed: {e}")
        self.actors = []
        self.vehicle = None
        for k in self.sensors:
            self.sensors[k] = None
        time.sleep(0.05)
    def close(self):
        logging.info("Closing environment...")
        self._destroy_all_actors()
        for sensor_name, sensor in self.sensors.items():
            if sensor is not None:
                try:
                    if hasattr(sensor, "stop"):
                        sensor.stop()
                    if hasattr(sensor, "destroy"):
                        sensor.destroy()
                except Exception as e:
                    logging.debug(f"Error cleaning up sensor {sensor_name}: {e}")
        if self.world:
            ThreadedOperation.run(
                lambda: self.world.apply_settings(carla.WorldSettings(synchronous_mode=False)),
                timeout=2.0,
                description="Reset World Settings",
                accept_timeout=True,
            )
        if self.client:
            try:
                self.client = None
            except:
                pass
        self.world = None
        time.sleep(0.1)