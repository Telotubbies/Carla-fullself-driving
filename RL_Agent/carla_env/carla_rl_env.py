"""
CARLA Reinforcement Learning Environment
Vision-only environment wrapper for training RL agents
"""

import numpy as np
import gymnasium as gym
from gymnasium import spaces
import carla
from collections import deque
import cv2
from typing import Dict, Tuple, Optional, Any
import time
import logging

# Real-world lane detection (optional)
try:
    from .lane_detector import LaneDetector
    LANE_DETECTOR_AVAILABLE = True
except ImportError:
    LANE_DETECTOR_AVAILABLE = False
    logging.warning("LaneDetector not available, will use CARLA API for waypoints")

# Data augmentation (optional)
try:
    import sys
    import os
    sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
    from utils.data_augmentation import DataAugmentation
    DATA_AUGMENTATION_AVAILABLE = True
except ImportError:
    DATA_AUGMENTATION_AVAILABLE = False
    logging.warning("DataAugmentation not available")


class CarlaRLEnv(gym.Env):
    """
    CARLA Environment for Vision-Only RL Training
    
    Phase 1: Empty roads, lane keeping focus
    """
    
    metadata = {'render_modes': ['human', 'rgb_array']}
    
    def __init__(self, config: Dict[str, Any], port: int = None, rank: int = 0):
        super().__init__()
        
        self.config = config
        self.env_config = config.get('environment', {})
        self.sensor_config = config.get('sensors', {})
        self.obs_config = config.get('observations', {})
        self.action_config = config.get('actions', {})
        self.reward_config = config.get('rewards', {})
        
        # Parallel environment support: use different port for each environment
        self.rank = rank
        if port is not None:
            self.env_config['carla_port'] = port
        elif rank > 0:
            # Auto-assign port: base_port + rank
            base_port = self.env_config.get('carla_port', 2000)
            self.env_config['carla_port'] = base_port + (rank * 2)  # +2 per rank (2000, 2002, 2004, ...)
        
        # CARLA connection
        self.client = None
        self.world = None
        self.vehicle = None
        self.actors = []
        
        # Sensors
        self.rgb_camera = None
        self.depth_camera = None
        self.collision_sensor = None
        self.gps_sensor = None
        
        # GPS and Goal tracking
        self.current_gps_location = None
        self.goal_location = None
        self.distance_to_goal = None
        self.goal_reached = False
        
        # Observation buffers
        self.sequence_length = self.obs_config.get('stack_frames', 4)
        self.rgb_buffer = deque(maxlen=self.sequence_length)
        self.depth_buffer = deque(maxlen=self.sequence_length)
        
        # Observation Buffer Cache (Option A: เพิ่ม buffer cache)
        self.cache_enabled = self.obs_config.get('buffer_cache', {}).get('enabled', False)
        self.cache_size = self.obs_config.get('buffer_cache', {}).get('size', 32)  # Cache 32 observations
        self.observation_cache = deque(maxlen=self.cache_size) if self.cache_enabled else None
        self.processed_cache = deque(maxlen=self.cache_size) if self.cache_enabled else None
        
        # State tracking
        self.current_rgb = None
        self.current_depth = None
        self.collision_occurred = False
        self.episode_start_time = None
        self.last_location = None
        self.last_steering = 0.0
        
        # Curriculum Learning
        self.curriculum_config = self.env_config.get('curriculum_learning', {})
        self.curriculum_enabled = self.curriculum_config.get('enabled', False)
        self.current_difficulty = self.curriculum_config.get('initial_difficulty', 0.3) if self.curriculum_enabled else 1.0
        self.difficulty_increase_rate = self.curriculum_config.get('difficulty_increase_rate', 0.01)
        self.max_difficulty = self.curriculum_config.get('max_difficulty', 1.0)
        
        # Reward-based curriculum learning
        self.reward_based_curriculum = self.curriculum_config.get('reward_based', False)
        self.reward_threshold = self.curriculum_config.get('reward_threshold', 0.0)  # Mean reward threshold to advance
        self.reward_window_size = self.curriculum_config.get('reward_window_size', 50)  # Number of episodes to average
        self.episode_rewards = deque(maxlen=self.reward_window_size)  # Track recent episode rewards
        self.current_episode_reward = 0.0  # Current episode total reward
        
        # Progressive Reward Scaling
        self.progressive_rewards_config = self.reward_config.get('progressive_rewards', {})
        self.progressive_rewards_enabled = self.progressive_rewards_config.get('enabled', False)
        self.reward_scale = self.progressive_rewards_config.get('initial_scale', 1.0) if not self.progressive_rewards_enabled else self.progressive_rewards_config.get('initial_scale', 0.5)
        self.final_reward_scale = self.progressive_rewards_config.get('final_scale', 1.0)
        self.reward_scale_increase_rate = self.progressive_rewards_config.get('scale_increase_rate', 0.01)
        
        # Evaluation metrics tracking (Research Recommendations)
        self.episode_metrics = {
            'total_distance': 0.0,
            'lane_keeping_time': 0.0,
            'total_steps': 0,
            'route_completion': 0.0,
            'jerk_sum': 0.0,  # For smoothness metric
            'speed_variance': 0.0,
            'infractions': 0
        }
        
        # Define observation space (vision + GPS + goal + waypoint + velocity)
        image_size = self.obs_config.get('image_size', [160, 90])
        use_depth = self.obs_config.get('use_depth', True)
        use_gps = self.obs_config.get('use_gps', False)
        use_goal = self.obs_config.get('use_goal', False)
        use_waypoint = self.obs_config.get('use_waypoint', True)  # NEW: Waypoint Following
        use_velocity = self.obs_config.get('use_velocity', True)  # NEW: Velocity Information
        
        # Real-world lane detection (vision-based instead of CARLA API)
        self.use_vision_waypoint = self.obs_config.get('use_vision_waypoint', False)  # NEW: Vision-based waypoint
        edge_detection_method = self.obs_config.get('edge_detection_method', 'multiscale_canny')  # 'canny', 'multiscale_canny', 'enhanced_canny'
        self.lane_detector = None
        if self.use_vision_waypoint and LANE_DETECTOR_AVAILABLE:
            self.lane_detector = LaneDetector(
                image_width=image_size[0],
                image_height=image_size[1],
                edge_detection_method=edge_detection_method
            )
            logging.info(f"✅ Using vision-based lane detection (real-world approach) with {edge_detection_method}")
        elif self.use_vision_waypoint and not LANE_DETECTOR_AVAILABLE:
            logging.warning("⚠️  Vision waypoint requested but LaneDetector not available, falling back to CARLA API")
            self.use_vision_waypoint = False
        
        # Data augmentation
        augmentation_config = self.obs_config.get('augmentation', {})
        self.data_augmentation = None
        if DATA_AUGMENTATION_AVAILABLE and augmentation_config.get('enabled', False):
            self.data_augmentation = DataAugmentation(augmentation_config)
            logging.info(f"✅ Data augmentation enabled: {augmentation_config.get('methods', [])}")
        elif augmentation_config.get('enabled', False) and not DATA_AUGMENTATION_AVAILABLE:
            logging.warning("⚠️  Data augmentation requested but DataAugmentation not available")
        
        if use_depth:
            # RGB + Depth stacked
            vision_shape = (self.sequence_length, image_size[1], image_size[0], 4)  # 3 RGB + 1 Depth
        else:
            vision_shape = (self.sequence_length, image_size[1], image_size[0], 3)  # RGB only
        
        # If using GPS/goal/waypoint/velocity, use Dict observation space
        if use_gps or use_goal or use_waypoint or use_velocity:
            obs_dict = {'vision': spaces.Box(low=0.0, high=1.0, shape=vision_shape, dtype=np.float32)}
            if use_gps:
                obs_dict['gps'] = spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32)  # x, y, z
            if use_goal:
                obs_dict['goal'] = spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32)  # x, y, z
                obs_dict['distance_to_goal'] = spaces.Box(low=0.0, high=np.inf, shape=(1,), dtype=np.float32)
            if use_waypoint:
                # Waypoint features: direction (3D), distance, curvature, lane change (2), angle
                obs_dict['waypoint'] = spaces.Box(low=-1.0, high=1.0, shape=(8,), dtype=np.float32)
            if use_velocity:
                # Velocity features: speed, velocity vector (3D), speed normalized
                obs_dict['velocity'] = spaces.Box(low=-1.0, high=1.0, shape=(5,), dtype=np.float32)
            self.observation_space = spaces.Dict(obs_dict)
        else:
            # Vision-only (backward compatible)
            self.observation_space = spaces.Box(
                low=0.0,
                high=1.0,
                shape=vision_shape,
                dtype=np.float32
            )
        
        # Define action space (continuous: steering, throttle, brake)
        self.action_space = spaces.Box(
            low=np.array([-1.0, 0.0, 0.0], dtype=np.float32),
            high=np.array([1.0, 1.0, 1.0], dtype=np.float32),
            dtype=np.float32
        )
        
        # Initialize buffers with zeros
        self._initialize_buffers()
        
    def _initialize_buffers(self):
        """Initialize observation buffers with zeros"""
        image_size = self.obs_config.get('image_size', [160, 90])
        for _ in range(self.sequence_length):
            self.rgb_buffer.append(np.zeros((image_size[1], image_size[0], 3), dtype=np.float32))
            self.depth_buffer.append(np.zeros((image_size[1], image_size[0], 1), dtype=np.float32))
    
    def _connect_to_carla(self):
        """Connect to CARLA simulator with improved retry logic"""
        logging.info("Connecting to CARLA...")
        host = self.env_config.get('carla_host', 'localhost')
        port = self.env_config.get('carla_port', 2000)
        timeout = self.env_config.get('timeout', 60.0)  # Increased timeout
        town = self.env_config.get('town', 'Town01')  # Get town from config
        
        # Improved retry connection with exponential backoff
        max_retries = 20  # Increased from 5 to 20
        import time
        for attempt in range(max_retries):
            logging.info(f"   Attempt {attempt + 1}/{max_retries}...")
            try:
                self.client = carla.Client(host, port)
                self.client.set_timeout(timeout)
                
                # Load the specified town (or get current world if town matches)
                try:
                    self.world = self.client.load_world(town, reset_settings=False)
                    logging.info(f"Loaded town: {town}")
                except RuntimeError:
                    # If town is already loaded or load fails, get current world
                    self.world = self.client.get_world()
                    current_map = self.world.get_map().name
                    if current_map != town:
                        logging.info(f"Current map is {current_map}, requested {town}. Loading...")
                        self.world = self.client.load_world(town, reset_settings=False)
                    else:
                        logging.info(f"Using current town: {town}")
                
                break
            except RuntimeError as e:
                if attempt < max_retries - 1:
                    # Exponential backoff: 5s, 10s, 15s, 15s, 15s...
                    wait_time = min(5 + (attempt * 2), 15)
                    logging.warning(f"Connection attempt {attempt + 1}/{max_retries} failed, retrying in {wait_time}s...")
                    logging.warning(f"   Error: {str(e)}")
                    time.sleep(wait_time)
                else:
                    logging.error(f"Failed to connect after {max_retries} attempts")
                    raise
        
        # Set synchronous mode for deterministic simulation
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05  # 20 FPS
        
        # Enable no_rendering_mode to reduce CARLA server load (2D topview mode)
        # This significantly reduces GPU/CPU usage on CARLA server
        no_rendering = self.env_config.get('no_rendering_mode', False)
        settings.no_rendering_mode = no_rendering
        if no_rendering:
            logging.info("🎨 No Rendering Mode: ENABLED (2D topview, reduced server load)")
        else:
            logging.info("🎨 No Rendering Mode: DISABLED (3D rendering, higher server load)")
        
        self.world.apply_settings(settings)
        
        logging.info(f"Connected to CARLA at {host}:{port}")
    
    def _setup_world(self):
        """Configure world settings (weather, traffic, etc.)"""
        logging.info("Setting up world...")
        
        # Domain Randomization (Research Recommendation)
        domain_rand = self.env_config.get('domain_randomization', {})
        if domain_rand.get('enabled', False) and domain_rand.get('weather_randomization', False):
            # Randomize weather for domain randomization
            import random
            weather_ranges = domain_rand.get('weather', {})
            weather = carla.WeatherParameters(
                cloudiness=random.uniform(
                    weather_ranges.get('cloudiness_range', [0.0, 0.0])[0],
                    weather_ranges.get('cloudiness_range', [0.0, 0.0])[1]
                ),
                precipitation=random.uniform(
                    weather_ranges.get('precipitation_range', [0.0, 0.0])[0],
                    weather_ranges.get('precipitation_range', [0.0, 0.0])[1]
                ),
                sun_altitude_angle=random.uniform(
                    weather_ranges.get('sun_altitude_range', [45.0, 45.0])[0],
                    weather_ranges.get('sun_altitude_range', [45.0, 45.0])[1]
                ),
                fog_density=random.uniform(
                    weather_ranges.get('fog_density_range', [0.0, 0.0])[0],
                    weather_ranges.get('fog_density_range', [0.0, 0.0])[1]
                )
            )
            logging.info(f"Randomized weather: cloudiness={weather.cloudiness:.1f}, precipitation={weather.precipitation:.1f}")
        else:
            # Use fixed weather from config
            weather_config = self.env_config.get('weather', {})
            weather = carla.WeatherParameters(
                cloudiness=weather_config.get('cloudiness', 0.0),
                precipitation=weather_config.get('precipitation', 0.0),
                sun_altitude_angle=weather_config.get('sun_altitude_angle', 45.0)
            )
        self.world.set_weather(weather)
        
        # Spawn vehicles and pedestrians as obstacles
        # Apply curriculum learning if enabled
        if self.curriculum_enabled:
            # Calculate difficulty-based spawn counts
            vehicle_range = self.curriculum_config.get('num_vehicles', [0, 10])
            pedestrian_range = self.curriculum_config.get('num_pedestrians', [0, 5])
            
            num_vehicles = int(vehicle_range[0] + (vehicle_range[1] - vehicle_range[0]) * self.current_difficulty)
            num_pedestrians = int(pedestrian_range[0] + (pedestrian_range[1] - pedestrian_range[0]) * self.current_difficulty)
            enable_traffic = self.curriculum_config.get('enable_traffic', False) and (self.current_difficulty > 0.5)
            
            logging.info(f"Curriculum Learning: difficulty={self.current_difficulty:.2f}, vehicles={num_vehicles}, pedestrians={num_pedestrians}")
        else:
            num_vehicles = self.env_config.get('num_vehicles', 0)
            num_pedestrians = self.env_config.get('num_pedestrians', 0)
            enable_traffic = self.env_config.get('enable_traffic', False)
        
        if num_vehicles > 0 or num_pedestrians > 0:
            self._spawn_traffic(num_vehicles, num_pedestrians, enable_traffic)
            logging.info(f"World configured with {num_vehicles} vehicles and {num_pedestrians} pedestrians")
        else:
            logging.info("World configured for Phase 1 (empty roads)")
    
    def _spawn_vehicle(self):
        """Spawn the agent vehicle"""
        logging.info("Spawning agent vehicle...")
        
        # Domain Randomization: Random vehicle type
        domain_rand = self.env_config.get('domain_randomization', {})
        vehicle_config = self.env_config.get('vehicle', {})
        
        if domain_rand.get('enabled', False) and domain_rand.get('vehicle_randomization', False):
            # Randomize vehicle blueprint
            import random
            vehicle_blueprints = domain_rand.get('vehicle_blueprints', ['vehicle.tesla.model3'])
            blueprint_name = random.choice(vehicle_blueprints)
            logging.info(f"Randomized vehicle: {blueprint_name}")
        else:
            # Use fixed blueprint from config
            blueprint_name = vehicle_config.get('blueprint', 'vehicle.tesla.model3')
        
        blueprint_library = self.world.get_blueprint_library()
        vehicle_bp = blueprint_library.find(blueprint_name)
        
        # Random spawn point with retry
        spawn_points = self.world.get_map().get_spawn_points()
        vehicle_spawned = False
        
        # Try multiple spawn points
        import random
        random.shuffle(spawn_points)
        
        for spawn_point in spawn_points[:10]:  # Try first 10 spawn points
            try:
                self.vehicle = self.world.try_spawn_actor(vehicle_bp, spawn_point)
                if self.vehicle is not None:
                    self.actors.append(self.vehicle)
                    vehicle_spawned = True
                    break
            except Exception as e:
                continue
        
        if not vehicle_spawned:
            raise RuntimeError("Failed to spawn vehicle at any spawn point")
        
        # Enable physics for new vehicle (was disabled on disconnect)
        try:
            self.vehicle.set_simulate_physics(True)
        except:
            pass  # May not be available in all CARLA versions
        
        # Disable autopilot
        self.vehicle.set_autopilot(False)
        
        logging.info(f"Agent vehicle spawned: {self.vehicle.type_id}")
    
    def _set_random_goal(self):
        """Set a random goal location for navigation"""
        if self.vehicle is None or self.world is None:
            return
        
        try:
            # Get spawn points from map
            spawn_points = self.world.get_map().get_spawn_points()
            if len(spawn_points) < 2:
                # Fallback: set goal 100m ahead
                vehicle_loc = self.vehicle.get_location()
                vehicle_transform = self.vehicle.get_transform()
                forward_vector = vehicle_transform.get_forward_vector()
                self.goal_location = carla.Location(
                    x=vehicle_loc.x + forward_vector.x * 100.0,
                    y=vehicle_loc.y + forward_vector.y * 100.0,
                    z=vehicle_loc.z
                )
            else:
                # Pick a random spawn point that's not too close
                import random
                vehicle_loc = self.vehicle.get_location()
                valid_goals = []
                for sp in spawn_points:
                    distance = np.sqrt(
                        (sp.location.x - vehicle_loc.x)**2 +
                        (sp.location.y - vehicle_loc.y)**2
                    )
                    if distance > 50.0:  # At least 50m away
                        valid_goals.append(sp.location)
                
                if valid_goals:
                    self.goal_location = random.choice(valid_goals)
                else:
                    # Fallback: set goal 100m ahead
                    vehicle_transform = self.vehicle.get_transform()
                    forward_vector = vehicle_transform.get_forward_vector()
                    self.goal_location = carla.Location(
                        x=vehicle_loc.x + forward_vector.x * 100.0,
                        y=vehicle_loc.y + forward_vector.y * 100.0,
                        z=vehicle_loc.z
                    )
            
            # Calculate initial distance
            if self.vehicle is not None:
                current_loc = self.vehicle.get_location()
                self.distance_to_goal = np.sqrt(
                    (current_loc.x - self.goal_location.x)**2 +
                    (current_loc.y - self.goal_location.y)**2 +
                    (current_loc.z - self.goal_location.z)**2
                )
                logging.info(f"🎯 Goal set at ({self.goal_location.x:.1f}, {self.goal_location.y:.1f}, {self.goal_location.z:.1f}), distance: {self.distance_to_goal:.1f}m")
        except Exception as e:
            logging.error(f"Failed to set goal: {e}")
            self.goal_location = None
    
    def _spawn_traffic(self, num_vehicles: int, num_pedestrians: int, enable_traffic: bool):
        """Spawn traffic vehicles and pedestrians as obstacles"""
        import random
        
        logging.info(f"Starting traffic spawn: {num_vehicles} vehicles, {num_pedestrians} pedestrians")
        
        # Spawn vehicles
        if num_vehicles > 0:
            logging.info(f"   Spawning {num_vehicles} vehicles...")
            blueprint_library = self.world.get_blueprint_library()
            vehicle_blueprints = blueprint_library.filter('vehicle.*')
            
            # Filter out bicycles and motorcycles (too small)
            vehicle_blueprints = [bp for bp in vehicle_blueprints if int(bp.get_attribute('number_of_wheels')) == 4]
            
            spawn_points = self.world.get_map().get_spawn_points()
            random.shuffle(spawn_points)
            
            vehicles_spawned = 0
            for spawn_point in spawn_points:
                if vehicles_spawned >= num_vehicles:
                    break
                
                try:
                    vehicle_bp = random.choice(vehicle_blueprints)
                    vehicle = self.world.try_spawn_actor(vehicle_bp, spawn_point)
                    
                    if vehicle is not None:
                        self.actors.append(vehicle)
                        
                        # Enable traffic manager for moving vehicles
                        if enable_traffic:
                            vehicle.set_autopilot(True)
                        
                        vehicles_spawned += 1
                except Exception as e:
                    continue
            
            logging.info(f"Spawned {vehicles_spawned} traffic vehicles")
        
        # Spawn pedestrians
        if num_pedestrians > 0:
            logging.info(f"   Spawning {num_pedestrians} pedestrians...")
            blueprint_library = self.world.get_blueprint_library()
            walker_blueprints = blueprint_library.filter('walker.pedestrian.*')
            
            # Use spawn points from map instead of random navigation (faster, more reliable)
            spawn_points = []
            map_spawn_points = self.world.get_map().get_spawn_points()
            for sp in map_spawn_points[:num_pedestrians * 3]:  # Use nearby spawn points
                spawn_point = carla.Transform()
                spawn_point.location = sp.location
                spawn_point.rotation = sp.rotation
                spawn_points.append(spawn_point)
            
            pedestrians_spawned = 0
            for spawn_point in spawn_points:
                if pedestrians_spawned >= num_pedestrians:
                    break
                
                try:
                    walker_bp = random.choice(walker_blueprints)
                    walker = self.world.try_spawn_actor(walker_bp, spawn_point)
                    
                    if walker is not None:
                        self.actors.append(walker)
                        
                        # Make pedestrian walk randomly (with timeout protection)
                        try:
                            walker_controller_bp = blueprint_library.find('controller.ai.walker')
                            controller = self.world.spawn_actor(walker_controller_bp, carla.Transform(), walker)
                            controller.start()
                            # Use nearby location instead of random navigation
                            nearby_location = random.choice(map_spawn_points).location
                            controller.go_to_location(nearby_location)
                            self.actors.append(controller)
                        except Exception as e:
                            logging.warning(f"Warning: Could not setup pedestrian controller: {e}")
                            # Continue without controller - pedestrian will still spawn
                        
                        pedestrians_spawned += 1
                except Exception as e:
                    continue
            
            logging.info(f"Spawned {pedestrians_spawned} pedestrians")
    
    def _setup_sensors(self):
        """Setup RGB and Depth cameras"""
        logging.info("Setting up sensors...")
        blueprint_library = self.world.get_blueprint_library()
        
        # RGB Camera
        if self.sensor_config.get('rgb_camera', {}).get('enabled', True):
            try:
                rgb_config = self.sensor_config['rgb_camera']
                rgb_bp = blueprint_library.find('sensor.camera.rgb')
                rgb_bp.set_attribute('image_size_x', str(rgb_config['width']))
                rgb_bp.set_attribute('image_size_y', str(rgb_config['height']))
                rgb_bp.set_attribute('fov', str(rgb_config['fov']))
                
                rgb_transform = carla.Transform(
                    carla.Location(*rgb_config['location']),
                    carla.Rotation(*rgb_config['rotation'])
                )
                
                self.rgb_camera = self.world.spawn_actor(
                    rgb_bp, rgb_transform, attach_to=self.vehicle
                )
                self.rgb_camera.listen(lambda image: self._process_rgb_image(image))
                self.actors.append(self.rgb_camera)
                logging.info("RGB Camera setup")
            except Exception as e:
                logging.error(f"Failed to setup RGB camera: {e}")
                raise
        
        # Depth Camera
        if self.sensor_config.get('depth_camera', {}).get('enabled', True):
            try:
                depth_config = self.sensor_config['depth_camera']
                depth_bp = blueprint_library.find('sensor.camera.depth')
                depth_bp.set_attribute('image_size_x', str(depth_config['width']))
                depth_bp.set_attribute('image_size_y', str(depth_config['height']))
                depth_bp.set_attribute('fov', str(depth_config['fov']))
                
                depth_transform = carla.Transform(
                    carla.Location(*depth_config['location']),
                    carla.Rotation(*depth_config['rotation'])
                )
                
                self.depth_camera = self.world.spawn_actor(
                    depth_bp, depth_transform, attach_to=self.vehicle
                )
                self.depth_camera.listen(lambda image: self._process_depth_image(image))
                self.actors.append(self.depth_camera)
                logging.info("Depth Camera setup")
            except Exception as e:
                logging.error(f"Failed to setup Depth camera: {e}")
                raise
        
        # GPS Sensor
        if self.sensor_config.get('gps', {}).get('enabled', False):
            try:
                gps_config = self.sensor_config['gps']
                gps_bp = blueprint_library.find('sensor.other.gnss')
                gps_transform = carla.Transform(
                    carla.Location(*gps_config.get('location', [1.0, 0.0, 2.8]))
                )
                self.gps_sensor = self.world.spawn_actor(
                    gps_bp, gps_transform, attach_to=self.vehicle
                )
                self.gps_sensor.listen(lambda data: self._process_gps(data))
                self.actors.append(self.gps_sensor)
                logging.info("GPS Sensor setup")
            except Exception as e:
                logging.error(f"Failed to setup GPS sensor: {e}")
                raise
        
        # Collision Sensor
        try:
            collision_bp = blueprint_library.find('sensor.other.collision')
            collision_transform = carla.Transform(carla.Location(x=0.0, z=0.0))
            self.collision_sensor = self.world.spawn_actor(
                collision_bp, collision_transform, attach_to=self.vehicle
            )
            self.collision_sensor.listen(lambda event: self._on_collision(event))
            self.actors.append(self.collision_sensor)
            logging.info("Collision Sensor setup")
        except Exception as e:
            logging.error(f"Failed to setup Collision sensor: {e}")
            raise
    
    def _process_rgb_image(self, image):
        """Process RGB camera image"""
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]  # Remove alpha channel
        
        # Resize if needed
        image_size = self.obs_config.get('image_size', [160, 90])
        if array.shape[:2] != tuple(image_size[::-1]):  # [height, width]
            array = cv2.resize(array, tuple(image_size), interpolation=cv2.INTER_LINEAR)
        
        # Normalize to [0, 1]
        if self.obs_config.get('normalize', True):
            array = array.astype(np.float32) / 255.0
        
        self.current_rgb = array
    
    def _process_depth_image(self, image):
        """Process depth camera image"""
        # Convert to depth array
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]  # RGB depth visualization
        
        # Convert to actual depth (meters)
        # CARLA depth is encoded in RGB, decode it
        depth = (array[:, :, 0] + array[:, :, 1] * 256.0 + array[:, :, 2] * 256.0 * 256.0) / (256.0 * 256.0 * 256.0 - 1.0)
        depth = depth * 1000.0  # Convert to meters
        
        # Resize if needed
        image_size = self.obs_config.get('image_size', [160, 90])
        if depth.shape != tuple(image_size[::-1]):
            depth = cv2.resize(depth, tuple(image_size), interpolation=cv2.INTER_LINEAR)
        
        # Normalize depth (0-100m range)
        depth = np.clip(depth / 100.0, 0.0, 1.0)
        depth = np.expand_dims(depth, axis=-1)  # Add channel dimension
        
        self.current_depth = depth.astype(np.float32)
    
    def _process_gps(self, data):
        """Process GPS sensor data"""
        # Convert GPS (lat, lon) to CARLA coordinates (x, y, z)
        # CARLA uses a local coordinate system, so we use vehicle location directly
        if self.vehicle is not None:
            location = self.vehicle.get_location()
            self.current_gps_location = np.array([location.x, location.y, location.z], dtype=np.float32)
        else:
            self.current_gps_location = np.array([0.0, 0.0, 0.0], dtype=np.float32)
    
    def _on_collision(self, event):
        """Handle collision events"""
        self.collision_occurred = True
    
    def _get_observation(self) -> np.ndarray:
        """Get current observation (stacked frames)"""
        obs_start = time.time()
        step_count = getattr(self, '_step_count', 0)
        
        # Wait for sensor data with timeout (max 0.5 seconds)
        max_wait = 0.5
        wait_start = time.time()
        wait_iterations = 0
        while (self.current_rgb is None or self.current_depth is None) and (time.time() - wait_start) < max_wait:
            time.sleep(0.01)  # Small sleep to avoid busy waiting
            wait_iterations += 1
        
        wait_duration = time.time() - wait_start
        if wait_iterations > 0:
            logging.info(f"Step {step_count}: Waited {wait_duration:.3f}s for sensor data ({wait_iterations} iterations)")
        
        if self.current_rgb is None or self.current_depth is None:
            # If still no data, use zeros (will trigger reset)
            logging.warning(f"Step {step_count}: Sensor data not available after {wait_duration:.3f}s, using zero observation")
            if self.current_rgb is None:
                image_size = self.obs_config.get('image_size', [160, 90])
                self.current_rgb = np.zeros((image_size[1], image_size[0], 3), dtype=np.float32)
            if self.current_depth is None:
                image_size = self.obs_config.get('image_size', [160, 90])
                self.current_depth = np.zeros((image_size[1], image_size[0], 1), dtype=np.float32)
        
        # Apply data augmentation if enabled
        rgb_frame = self.current_rgb.copy() if self.current_rgb is not None else None
        depth_frame = self.current_depth.copy() if self.current_depth is not None else None
        
        if self.data_augmentation is not None and rgb_frame is not None:
            rgb_frame = self.data_augmentation.augment_image(rgb_frame)
        
        # Update buffers with latest frames (augmented if enabled)
        if rgb_frame is not None:
            self.rgb_buffer.append(rgb_frame)
        if depth_frame is not None:
            self.depth_buffer.append(depth_frame)
        
        # Ensure buffers are filled (repeat last frame if needed)
        while len(self.rgb_buffer) < self.sequence_length:
            if len(self.rgb_buffer) > 0:
                self.rgb_buffer.append(self.rgb_buffer[-1].copy())
            else:
                # Fallback: create zero frame
                image_size = self.obs_config.get('image_size', [160, 90])
                self.rgb_buffer.append(np.zeros((image_size[1], image_size[0], 3), dtype=np.float32))
        
        while len(self.depth_buffer) < self.sequence_length:
            if len(self.depth_buffer) > 0:
                self.depth_buffer.append(self.depth_buffer[-1].copy())
            else:
                # Fallback: create zero frame
                image_size = self.obs_config.get('image_size', [160, 90])
                self.depth_buffer.append(np.zeros((image_size[1], image_size[0], 1), dtype=np.float32))
        
        # Stack frames
        rgb_stack = np.stack(list(self.rgb_buffer), axis=0)
        depth_stack = np.stack(list(self.depth_buffer), axis=0)
        
        # Cache processed observation (Option A: Buffer Cache)
        if self.cache_enabled and self.observation_cache is not None:
            # Cache raw observation data for faster access
            cache_entry = {
                'rgb': self.current_rgb.copy() if self.current_rgb is not None else None,
                'depth': self.current_depth.copy() if self.current_depth is not None else None,
                'gps': self.current_gps_location.copy() if self.current_gps_location is not None else None,
                'goal': carla.Location(self.goal_location.x, self.goal_location.y, self.goal_location.z) if self.goal_location is not None else None,
                'distance': self.distance_to_goal
            }
            self.observation_cache.append(cache_entry)
            
            # Cache processed observation (stacked frames)
            if self.processed_cache is not None:
                processed_entry = {
                    'rgb_stack': rgb_stack.copy(),
                    'depth_stack': depth_stack.copy()
                }
                self.processed_cache.append(processed_entry)
        
        # Concatenate RGB and Depth
        if self.obs_config.get('use_depth', True):
            vision_obs = np.concatenate([rgb_stack, depth_stack], axis=-1)
        else:
            vision_obs = rgb_stack
        
        # Check if using GPS/goal/waypoint/velocity
        use_gps = self.obs_config.get('use_gps', False)
        use_goal = self.obs_config.get('use_goal', False)
        use_waypoint = self.obs_config.get('use_waypoint', True)  # NEW: Default True
        use_velocity = self.obs_config.get('use_velocity', True)  # NEW: Default True
        
        if use_gps or use_goal or use_waypoint or use_velocity:
            # Build dict observation
            obs_dict = {'vision': vision_obs.astype(np.float32)}
            
            # Add GPS
            if use_gps:
                if self.current_gps_location is not None:
                    obs_dict['gps'] = self.current_gps_location.copy()
                else:
                    obs_dict['gps'] = np.array([0.0, 0.0, 0.0], dtype=np.float32)
            
            # Add Goal and distance
            if use_goal:
                if self.goal_location is not None:
                    goal_array = np.array([self.goal_location.x, self.goal_location.y, self.goal_location.z], dtype=np.float32)
                    obs_dict['goal'] = goal_array
                    
                    # Calculate distance to goal
                    if self.vehicle is not None and self.current_gps_location is not None:
                        current_loc = self.vehicle.get_location()
                        distance = np.sqrt(
                            (current_loc.x - self.goal_location.x)**2 +
                            (current_loc.y - self.goal_location.y)**2 +
                            (current_loc.z - self.goal_location.z)**2
                        )
                        self.distance_to_goal = distance
                        obs_dict['distance_to_goal'] = np.array([distance], dtype=np.float32)
                    else:
                        obs_dict['distance_to_goal'] = np.array([999.0], dtype=np.float32)
                else:
                    obs_dict['goal'] = np.array([0.0, 0.0, 0.0], dtype=np.float32)
                    obs_dict['distance_to_goal'] = np.array([999.0], dtype=np.float32)
            
            # Add Waypoint features (NEW: Priority 2)
            if use_waypoint:
                waypoint_features = self._get_waypoint_features()
                obs_dict['waypoint'] = waypoint_features
            
            # Add Velocity features (NEW: Priority 3)
            if use_velocity:
                velocity_features = self._get_velocity_features()
                obs_dict['velocity'] = velocity_features
            
            obs_duration = time.time() - obs_start
            step_count = getattr(self, '_step_count', 0)
            logging.info(f"Step {step_count}: Observation (dict) took {obs_duration:.3f}s, vision={vision_obs.shape}, gps={use_gps}, goal={use_goal}, waypoint={use_waypoint}, velocity={use_velocity}")
            return obs_dict
        else:
            # Vision-only (backward compatible)
            obs_duration = time.time() - obs_start
            step_count = getattr(self, '_step_count', 0)
            logging.info(f"Step {step_count}: Observation shape={vision_obs.shape}, took {obs_duration:.3f}s, rgb={self.current_rgb is not None}, depth={self.current_depth is not None}")
            return vision_obs.astype(np.float32)
    
    def _get_waypoint_features(self) -> np.ndarray:
        """
        Extract waypoint features for observation (Priority 2: Waypoint Following)
        
        Supports two modes:
        1. Vision-based (real-world): Uses computer vision to detect lanes
        2. CARLA API: Uses CARLA waypoint API (default)
        """
        if self.vehicle is None or self.world is None:
            return np.zeros(8, dtype=np.float32)
        
        # Use vision-based detection if enabled
        if self.use_vision_waypoint and self.lane_detector is not None and self.current_rgb is not None:
            try:
                # Process current RGB image through lane detector
                lane_features = self.lane_detector.process_image(self.current_rgb)
                
                # Convert to same format as CARLA API features
                return np.array([
                    lane_features['waypoint_dx'],      # Waypoint direction x
                    lane_features['waypoint_dy'],      # Waypoint direction y
                    0.0,                               # Z component (not available from vision)
                    abs(lane_features['waypoint_dy']), # Distance estimate (from dy)
                    lane_features['curvature'],        # Road curvature
                    lane_features['lane_change_left'], # Lane change left
                    lane_features['lane_change_right'], # Lane change right
                    lane_features['lane_center_offset'] # Lane center offset (as angle)
                ], dtype=np.float32)
            except Exception as e:
                logging.warning(f"Error in vision-based waypoint detection: {e}, falling back to CARLA API")
                # Fall through to CARLA API
        
        # CARLA API approach (default or fallback)
        try:
            # Get current waypoint
            transform = self.vehicle.get_transform()
            waypoint = self.world.get_map().get_waypoint(transform.location)
            
            # Get next waypoint (5 meters ahead)
            next_waypoints = waypoint.next(5.0)
            if next_waypoints and len(next_waypoints) > 0:
                next_waypoint = next_waypoints[0]
            else:
                # Fallback: use current waypoint
                next_waypoint = waypoint
            
            # 1. Next waypoint direction (relative to vehicle)
            vehicle_location = transform.location
            waypoint_location = next_waypoint.transform.location
            
            # Convert to vehicle-relative coordinates
            dx = waypoint_location.x - vehicle_location.x
            dy = waypoint_location.y - vehicle_location.y
            dz = waypoint_location.z - vehicle_location.z
            
            # Rotate to vehicle frame
            vehicle_yaw = np.radians(transform.rotation.yaw)
            cos_yaw = np.cos(vehicle_yaw)
            sin_yaw = np.sin(vehicle_yaw)
            
            # Relative direction in vehicle frame
            rel_x = dx * cos_yaw + dy * sin_yaw
            rel_y = -dx * sin_yaw + dy * cos_yaw
            rel_z = dz
            
            # 2. Distance to next waypoint
            distance = np.sqrt(dx**2 + dy**2 + dz**2)
            
            # 3. Road curvature (angle difference)
            waypoint_yaw = np.radians(next_waypoint.transform.rotation.yaw)
            curvature = waypoint_yaw - vehicle_yaw
            # Normalize to [-pi, pi]
            curvature = np.arctan2(np.sin(curvature), np.cos(curvature))
            
            # 4. Lane change availability
            lane_change = waypoint.lane_change
            lane_change_left = 1.0 if (lane_change == carla.LaneChange.Left or lane_change == carla.LaneChange.Both) else 0.0
            lane_change_right = 1.0 if (lane_change == carla.LaneChange.Right or lane_change == carla.LaneChange.Both) else 0.0
            
            # 5. Waypoint angle (relative to vehicle heading)
            angle_diff = curvature  # Same as curvature
            
            # Normalize features
            return np.array([
                np.clip(rel_x / 10.0, -1.0, 1.0),      # Normalize (typical waypoint distance ~5-10m)
                np.clip(rel_y / 10.0, -1.0, 1.0),
                np.clip(rel_z / 5.0, -1.0, 1.0),
                np.clip(distance / 20.0, 0.0, 1.0),   # Normalize (typical distance ~5-20m)
                np.clip(curvature / np.pi, -1.0, 1.0), # Normalize to [-1, 1]
                lane_change_left,
                lane_change_right,
                np.clip(angle_diff / np.pi, -1.0, 1.0) # Normalize to [-1, 1]
            ], dtype=np.float32)
        except Exception as e:
            logging.warning(f"Error extracting waypoint features: {e}")
            return np.zeros(8, dtype=np.float32)
    
    def _get_velocity_features(self) -> np.ndarray:
        """Extract velocity features for observation (Priority 3: Velocity Information)"""
        if self.vehicle is None:
            return np.zeros(5, dtype=np.float32)
        
        try:
            # Get velocity
            velocity = self.vehicle.get_velocity()
            
            # Calculate speed (km/h)
            speed_ms = np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            speed_kmh = 3.6 * speed_ms
            
            # Normalize features
            # Speed: 0-100 km/h -> 0-1
            speed_normalized = np.clip(speed_kmh / 100.0, 0.0, 1.0)
            
            # Velocity components (normalize to typical range)
            # Typical max speed: ~15 m/s (54 km/h)
            vx_normalized = np.clip(velocity.x / 15.0, -1.0, 1.0)
            vy_normalized = np.clip(velocity.y / 15.0, -1.0, 1.0)
            vz_normalized = np.clip(velocity.z / 5.0, -1.0, 1.0)  # Vertical is usually small
            
            return np.array([
                speed_normalized,      # Normalized speed (0-1)
                vx_normalized,        # Normalized velocity x
                vy_normalized,        # Normalized velocity y
                vz_normalized,        # Normalized velocity z
                np.clip(speed_ms / 15.0, 0.0, 1.0)  # Speed in m/s (normalized)
            ], dtype=np.float32)
        except Exception as e:
            logging.warning(f"Error extracting velocity features: {e}")
            return np.zeros(5, dtype=np.float32)
    
    def _get_zero_observation(self):
        """Get zero observation matching the observation space"""
        if isinstance(self.observation_space, gym.spaces.Dict):
            # Dict observation: return zero dict
            zero_obs = {}
            for key, space in self.observation_space.spaces.items():
                if isinstance(space, gym.spaces.Box):
                    zero_obs[key] = np.zeros(space.shape, dtype=space.dtype)
                else:
                    zero_obs[key] = np.zeros(space.shape, dtype=np.float32)
            return zero_obs
        else:
            # Box observation: return zero array
            return np.zeros(self.observation_space.shape, dtype=self.observation_space.dtype)
    
    def _compute_reward(self) -> float:
        """Compute reward based on current state"""
        if self.vehicle is None:
            return 0.0
        
        reward = 0.0
        
        # Get vehicle state
        transform = self.vehicle.get_transform()
        velocity = self.vehicle.get_velocity()
        speed_kmh = 3.6 * np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        
        # Get waypoint and lane information
        waypoint = self.world.get_map().get_waypoint(transform.location)
        vehicle_location = transform.location
        
        # 1. Lane center reward
        lane_center = waypoint.transform.location
        distance_from_center = np.sqrt(
            (vehicle_location.x - lane_center.x)**2 + 
            (vehicle_location.y - lane_center.y)**2
        )
        lane_center_reward = self.reward_config.get('lane_center_reward', 1.0)
        lane_tolerance = self.reward_config.get('lane_center_tolerance', 0.5)
        reward += lane_center_reward * max(0, 1.0 - distance_from_center / lane_tolerance)
        
        # 2. Speed reward (ปรับให้รองรับหลายความเร็ว: 40-100 km/h)
        target_speed = self.reward_config.get('target_speed', 70.0)  # กลางช่วง 40-100
        speed_tolerance = self.reward_config.get('speed_tolerance', 5.0)
        speed_reward_weight = self.reward_config.get('speed_reward', 0.5)
        min_speed = self.reward_config.get('min_speed', 40.0)
        max_speed = self.reward_config.get('max_speed', 100.0)
        
        # รองรับหลายความเร็ว: 40-100 km/h
        if min_speed <= speed_kmh <= max_speed:
            # Reward สำหรับความเร็วที่เหมาะสม (40-100 km/h)
            if 50.0 <= speed_kmh <= 90.0:
                reward += speed_reward_weight * 1.0  # Full reward สำหรับช่วงกลาง
            elif 45.0 <= speed_kmh < 50.0 or 90.0 < speed_kmh <= 95.0:
                reward += speed_reward_weight * 0.8  # High reward
            else:
                # Partial reward สำหรับความเร็วใกล้ขอบ
                reward += speed_reward_weight * 0.5
        elif speed_kmh < min_speed:
            # Penalty สำหรับความเร็วช้าเกินไป (< 40 km/h)
            reward -= self.reward_config.get('low_speed_penalty', 0.1) * (min_speed - speed_kmh) / min_speed
        elif speed_kmh > max_speed:
            # Penalty สำหรับความเร็วเร็วเกินไป (> 100 km/h) - เพื่อความปลอดภัย
            reward -= self.reward_config.get('high_speed_penalty', 0.2) * (speed_kmh - max_speed) / max_speed
        
        # 3. Progress reward
        if self.last_location is not None:
            progress = np.sqrt(
                (vehicle_location.x - self.last_location.x)**2 +
                (vehicle_location.y - self.last_location.y)**2
            )
            reward += self.reward_config.get('progress_reward', 2.0) * progress
        self.last_location = vehicle_location
        
        # 4. Smooth steering reward
        current_steering = self.vehicle.get_control().steer
        steering_change = abs(current_steering - self.last_steering)
        reward += self.reward_config.get('smooth_steering_reward', 0.3) * (1.0 - steering_change)
        self.last_steering = current_steering
        
        # 5. Collision penalty
        if self.collision_occurred:
            reward += self.reward_config.get('collision_penalty', -50.0)
        
        # 6. Goal-based reward (NEW - GPS Navigation)
        use_goal = self.obs_config.get('use_goal', False)
        if use_goal and self.goal_location is not None and self.vehicle is not None:
            current_loc = self.vehicle.get_location()
            distance_to_goal = np.sqrt(
                (current_loc.x - self.goal_location.x)**2 +
                (current_loc.y - self.goal_location.y)**2 +
                (current_loc.z - self.goal_location.z)**2
            )
            self.distance_to_goal = distance_to_goal
            
            # Goal distance reward (negative distance = closer is better)
            goal_distance_reward_weight = self.reward_config.get('goal_distance_reward', 0.1)
            reward += -goal_distance_reward_weight * distance_to_goal
            
            # Goal reached reward
            goal_threshold = self.reward_config.get('goal_reached_threshold', 5.0)
            if distance_to_goal < goal_threshold and not self.goal_reached:
                goal_reached_reward = self.reward_config.get('goal_reached_reward', 100.0)
                reward += goal_reached_reward
                self.goal_reached = True
                logging.info(f"🎯 Goal reached! Distance: {distance_to_goal:.2f}m")
        
        # 7. Off-lane penalty (ลดลงเพื่อให้เปลี่ยนเลนได้)
        if distance_from_center > lane_tolerance * 2:
            # Penalty เฉพาะเมื่อออกจากเลนมากเกินไป (ไม่ใช่แค่เปลี่ยนเลน)
            if distance_from_center > lane_tolerance * 3:
                reward += self.reward_config.get('off_lane_penalty', -10.0)
        
        # 8. Progressive Reward Scaling
        if self.progressive_rewards_enabled:
            reward = reward * self.reward_scale
        
        return reward
        obstacle_avoidance_reward = self.reward_config.get('obstacle_avoidance_reward', 0.0)
        if obstacle_avoidance_reward > 0:
            min_distance = self._get_min_distance_to_obstacles(vehicle_location)
            if min_distance is not None:
                # Reward สำหรับการรักษาระยะห่างจาก obstacles
                safe_distance = self.reward_config.get('safe_distance', 10.0)  # meters
                critical_distance = self.reward_config.get('critical_distance', 5.0)  # meters - ต้องหยุด
                
                if min_distance >= safe_distance:
                    reward += obstacle_avoidance_reward
                elif min_distance < critical_distance:
                    # ใกล้เกินไป - ควรหยุดหรือหลบ
                    # Reward สำหรับการหยุดเมื่อหลบไม่ได้
                    current_brake = self.vehicle.get_control().brake
                    if current_brake > 0.5:  # หยุด (brake > 0.5)
                        # Reward สำหรับการหยุดเมื่อมีสิ่งกีดขวาง
                        stop_reward = self.reward_config.get('safe_stop_reward', 0.5)
                        reward += stop_reward * (1.0 - min_distance / critical_distance)  # ยิ่งใกล้ยิ่งได้ reward มาก
                    else:
                        # Penalty สำหรับไม่หยุดเมื่อใกล้เกินไป
                        reward -= obstacle_avoidance_reward * (critical_distance - min_distance) / critical_distance
                else:
                    # ระยะกลาง - ควรระวัง
                    reward += obstacle_avoidance_reward * 0.5
        
        # 8. Lane change reward (เปลี่ยนเลน)
        lane_change_reward = self.reward_config.get('lane_change_reward', 0.0)
        if lane_change_reward > 0:
            # Track lane changes (simplified: detect significant lateral movement)
            if hasattr(self, 'last_lane_id'):
                current_lane_id = waypoint.lane_id
                if current_lane_id != self.last_lane_id:
                    # Reward สำหรับการเปลี่ยนเลนสำเร็จ (ถ้าไม่ชน)
                    if not self.collision_occurred:
                        reward += lane_change_reward
                self.last_lane_id = current_lane_id
            else:
                self.last_lane_id = waypoint.lane_id
        
        return reward
    
    def _get_min_distance_to_obstacles(self, vehicle_location: carla.Location) -> Optional[float]:
        """Get minimum distance to nearby vehicles/pedestrians"""
        try:
            min_distance = float('inf')
            
            # Get all vehicles (excluding self)
            vehicles = self.world.get_actors().filter("*vehicle*")
            for vehicle in vehicles:
                if vehicle.id != self.vehicle.id and vehicle.is_alive:
                    distance = vehicle_location.distance(vehicle.get_location())
                    if distance < min_distance:
                        min_distance = distance
            
            # Get all pedestrians
            pedestrians = self.world.get_actors().filter("*walker*")
            for pedestrian in pedestrians:
                if pedestrian.is_alive:
                    distance = vehicle_location.distance(pedestrian.get_location())
                    if distance < min_distance:
                        min_distance = distance
            
            return min_distance if min_distance != float('inf') else None
        except Exception as e:
            logging.debug(f"Error computing obstacle distance: {e}")
            return None
    
    def _is_done(self) -> bool:
        """Check if episode is done"""
        # Collision
        if self.collision_occurred:
            return True
        
        # Timeout
        if self.episode_start_time is not None:
            elapsed = time.time() - self.episode_start_time
            if elapsed > self.env_config.get('timeout', 20.0):
                return True
        
        return False
    
    def _disconnect_vehicle(self):
        """Disconnect from vehicle - FIXED: Check if actor is alive before operations"""
        if self.vehicle is not None:
            try:
                # FIXED: Check if vehicle is alive before trying to control it
                if not hasattr(self.vehicle, 'is_alive') or not self.vehicle.is_alive:
                    logging.debug("Vehicle is not alive, skipping disconnect")
                    return
                
                # Stop vehicle control
                control = carla.VehicleControl()
                control.steer = 0.0
                control.throttle = 0.0
                control.brake = 1.0  # Full brake
                
                # FIXED: Add timeout to prevent hanging
                import threading
                control_completed = threading.Event()
                control_error = [None]
                
                def do_control():
                    try:
                        # Double-check vehicle is alive before operations
                        if not hasattr(self.vehicle, 'is_alive') or not self.vehicle.is_alive:
                            control_error[0] = RuntimeError("Vehicle is not alive")
                            control_completed.set()
                            return
                        
                self.vehicle.apply_control(control)
                self.vehicle.set_autopilot(False)
                        control_completed.set()
                    except Exception as e:
                        control_error[0] = e
                        control_completed.set()
                
                control_thread = threading.Thread(target=do_control, daemon=True)
                control_thread.start()
                control_thread.join(timeout=1.0)
                
                if not control_completed.is_set():
                    logging.warning("Vehicle control timeout")
                elif control_error[0] is not None:
                    # Only log if it's not the "actor not found" error (expected during reset)
                    error_msg = str(control_error[0])
                    if "Actor could not be found" not in error_msg and "not alive" not in error_msg:
                        logging.warning(f"Error controlling vehicle: {control_error[0]}")
                
                # Note: Vehicle will be destroyed in close() to prevent resource leaks
                logging.debug("Vehicle control stopped (will be destroyed in close())")
            except Exception as e:
                # Only log if it's not the "actor not found" error (expected during reset)
                error_msg = str(e)
                if "Actor could not be found" not in error_msg:
                logging.warning(f"Error disconnecting vehicle: {e}")
            # Don't set vehicle to None here - let close() handle destruction
    
    def _disconnect_sensors(self):
        """Disconnect sensors - FIXED: Will be properly destroyed in close()"""
        # Stop listening to sensors to reduce CPU load
        # Note: Sensors will be properly destroyed in close() to prevent resource leaks
        if self.rgb_camera is not None:
            try:
                self.rgb_camera.stop()
            except:
                pass
        
        if self.depth_camera is not None:
            try:
                self.depth_camera.stop()
            except:
                pass
        
        if self.collision_sensor is not None:
            try:
                self.collision_sensor.stop()
            except:
                pass
        
        if self.gps_sensor is not None:
            try:
                self.gps_sensor.stop()
            except:
                pass
        
        # Don't destroy here - let close() handle it properly with timeout protection
    
    def reset(self, seed: Optional[int] = None, options: Optional[Dict] = None) -> Tuple[np.ndarray, Dict]:
        """Reset environment"""
        logging.info("Environment reset: Starting reset...")
        super().reset(seed=seed)
        self._step_count = 0  # Reset step counter
        logging.info("Environment reset: Step counter reset to 0")
        
        # Disconnect from previous episode (don't destroy to prevent crashes)
        self._disconnect_sensors()
        self._disconnect_vehicle()
        
        # Reconnect if needed
        if self.client is None:
            self._connect_to_carla()
            self._setup_world()
        
        # Spawn vehicle and sensors
        self._spawn_vehicle()
        self._setup_sensors()
        
        # Reset state
        self.collision_occurred = False
        self.episode_start_time = time.time()
        self.last_location = None
        self.last_steering = 0.0
        self.goal_reached = False
        self.distance_to_goal = None
        self._initialize_buffers()
        # Reset episode reward for reward-based curriculum
        if self.reward_based_curriculum:
            self.current_episode_reward = 0.0
        
        # Set new goal location (if using goal navigation)
        use_goal = self.obs_config.get('use_goal', False)
        if use_goal:
            self._set_random_goal()
        
        # Reset evaluation metrics
        self.episode_metrics = {
            'total_distance': 0.0,
            'lane_keeping_time': 0.0,
            'total_steps': 0,
            'route_completion': 0.0,
            'jerk_sum': 0.0,
            'speed_variance': 0.0,
            'infractions': 0,
            'speeds': []  # Track speeds for variance calculation
        }
        
        # Wait for sensors to initialize (with timeout protection)
        max_wait_ticks = 10
        for i in range(max_wait_ticks):
            try:
                self.world.tick()
                time.sleep(0.05)
            except Exception as e:
                logging.warning(f"World tick failed during reset (attempt {i+1}/{max_wait_ticks}): {e}")
                if i == max_wait_ticks - 1:
                    raise RuntimeError(f"Failed to initialize sensors after {max_wait_ticks} attempts")
                time.sleep(0.1)
        
        # Get initial observation
        obs = self._get_observation()
        info = {}
        
        return obs, info
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        """Execute one step"""
        try:
            step_start_time = time.time()
            step_count = getattr(self, '_step_count', 0) + 1
            self._step_count = step_count
            logging.info(f"Step {step_count}: Starting step execution")
            try:
                handlers = logging.getLogger().handlers
                if handlers:
                    handlers[0].flush()
            except:
                pass
        except Exception as e:
            logging.error(f"Error at start of step(): {e}", exc_info=True)
            # Return error state
            obs = self._get_zero_observation()
            return obs, -10.0, True, False, {'error': 'step_start_failed'}
        
        # Convert action to numpy array if needed
        if not isinstance(action, np.ndarray):
            action = np.array(action)
        
        # Ensure action has correct shape (3 elements: steer, throttle, brake)
        if action.ndim == 0:
            # Scalar - expand to array
            action = np.array([action, 0.0, 0.0])
        elif action.shape[0] == 1:
            # Single element array - pad with zeros
            action = np.concatenate([action, np.zeros(2)])
        elif action.shape[0] < 3:
            # Less than 3 elements - pad with zeros
            action = np.pad(action, (0, 3 - action.shape[0]), mode='constant', constant_values=0.0)
        elif action.shape[0] > 3:
            # More than 3 elements - take first 3
            action = action[:3]
        
        # Apply action to vehicle
        control = carla.VehicleControl()
        control.steer = float(np.clip(action[0], -1.0, 1.0))
        control.throttle = float(np.clip(action[1], 0.0, 1.0))
        control.brake = float(np.clip(action[2], 0.0, 1.0))
        
        # Action repeat
        action_repeat = self.action_config.get('action_repeat', 4)
        logging.info(f"Step {step_count}: Applying action, action_repeat={action_repeat}, action={action}")
        for i in range(action_repeat):
            if self.vehicle is not None:
                try:
                    # FIXED: Check if vehicle is alive before applying control
                    if hasattr(self.vehicle, 'is_alive') and not self.vehicle.is_alive:
                        logging.warning(f"Step {step_count}: Vehicle is not alive at repeat {i+1}/{action_repeat}")
                        obs = self._get_zero_observation()
                        return obs, -10.0, True, False, {'error': 'vehicle_not_alive'}
                    self.vehicle.apply_control(control)
                except Exception as e:
                    logging.warning(f"Step {step_count}: Failed to apply control (repeat {i+1}/{action_repeat}): {e}")
                    # Vehicle might be destroyed, try to recover
                    if self.vehicle and hasattr(self.vehicle, 'is_alive') and not self.vehicle.is_alive:
                        logging.warning(f"Step {step_count}: Vehicle is not alive, marking as done")
                        # Return done=True to trigger reset in next step
                        obs = self._get_observation() if self.current_rgb is not None else self._get_zero_observation()
                        return obs, -10.0, True, False, {'error': 'vehicle_not_alive'}
            
            # Tick world to advance simulation (critical for synchronous mode)
            # FIXED: Add timeout protection to prevent hanging
            tick_start = time.time()
            tick_timeout = 2.0  # Maximum 2 seconds per tick
            logging.info(f"Step {step_count}: About to call world.tick() (repeat {i+1}/{action_repeat})")
            try:
                if self.world is None:
                    logging.error(f"Step {step_count}: World is None! Cannot tick.")
                    obs = self._get_zero_observation()
                    return obs, -10.0, True, False, {'error': 'world_is_none'}
                
                # FIXED: Use threading with timeout to prevent blocking
                import threading
                tick_completed = threading.Event()
                tick_error = [None]
                
                def do_tick():
                    try:
                        logging.info(f"Step {step_count}: Inside do_tick() thread, calling world.tick()...")
                self.world.tick()
                        logging.info(f"Step {step_count}: world.tick() returned successfully")
                        tick_completed.set()
                    except Exception as e:
                        logging.error(f"Step {step_count}: Error in world.tick(): {e}")
                        tick_error[0] = e
                        tick_completed.set()
                
                tick_thread = threading.Thread(target=do_tick, daemon=True, name=f"WorldTick-{step_count}-{i}")
                logging.info(f"Step {step_count}: Starting tick thread for repeat {i+1}/{action_repeat}")
                tick_thread.start()
                logging.info(f"Step {step_count}: Waiting for tick thread (timeout={tick_timeout}s)...")
                tick_thread.join(timeout=tick_timeout)
                
                if not tick_completed.is_set():
                    # Tick timed out - CARLA server is hanging
                    logging.error(f"Step {step_count}: World tick TIMEOUT after {tick_timeout}s (repeat {i+1}/{action_repeat}) - CARLA server may be hung")
                    obs = self._get_zero_observation()
                    return obs, -10.0, True, False, {'error': 'world_tick_timeout'}
                
                if tick_error[0] is not None:
                    logging.error(f"Step {step_count}: World tick error: {tick_error[0]}")
                    raise tick_error[0]
                
                tick_duration = time.time() - tick_start
                logging.info(f"Step {step_count}: world.tick() completed in {tick_duration:.3f}s (repeat {i+1}/{action_repeat})")
                if tick_duration > 0.1:  # Log if tick takes too long
                    logging.warning(f"Step {step_count}: World tick took {tick_duration:.3f}s (step {i+1}/{action_repeat})")
            except RuntimeError as e:
                logging.warning(f"World tick failed (step {i+1}/{action_repeat}): {e}")
                # Try to recover by waiting a bit
                time.sleep(0.05)
                try:
                    if self.world is not None:
                        # FIXED: Add timeout to recovery tick too
                        import threading
                        tick_completed = threading.Event()
                        tick_error = [None]
                        
                        def do_tick():
                            try:
                        self.world.tick()
                                tick_completed.set()
                            except Exception as e:
                                tick_error[0] = e
                                tick_completed.set()
                        
                        tick_thread = threading.Thread(target=do_tick, daemon=True)
                        tick_thread.start()
                        tick_thread.join(timeout=1.0)  # Shorter timeout for recovery
                        
                        if not tick_completed.is_set():
                            raise RuntimeError("World tick recovery timeout")
                        if tick_error[0] is not None:
                            raise tick_error[0]
                    else:
                        raise RuntimeError("World is None after recovery attempt")
                except Exception as e2:
                    logging.error(f"World tick recovery failed: {e2}")
                    # If we can't tick, the simulation is broken - mark as done
                    obs = self._get_observation() if self.current_rgb is not None else self._get_zero_observation()
                    return obs, -10.0, True, False, {'error': 'world_tick_failed'}
            except Exception as e:
                logging.error(f"Unexpected error in world.tick(): {e}", exc_info=True)
                obs = self._get_zero_observation()
                return obs, -10.0, True, False, {'error': 'world_tick_unexpected_error'}
            # Small delay for stability (reduced from 0.005 to 0.001 for faster collection)
            time.sleep(0.001)
        
        # Get observation (with error handling and timeout)
        obs_start = time.time()
        try:
            obs = self._get_observation()
            obs_duration = time.time() - obs_start
            if obs_duration > 0.1:  # Log if observation takes too long
                logging.warning(f"Getting observation took {obs_duration:.3f}s")
            
            # Validate observation shape
            # Handle both Box and Dict observation spaces
            if isinstance(self.observation_space, gym.spaces.Dict):
                # Dict observation: check each key
                if not isinstance(obs, dict):
                    logging.error(f"Expected Dict observation but got {type(obs)}")
                    obs = self._get_zero_observation()
            else:
                # Box observation: check shape
                expected_shape = self.observation_space.shape
                if obs.shape != expected_shape:
                    logging.error(f"Observation shape mismatch! Got {obs.shape}, expected {expected_shape}")
                    # Return zero observation and mark as done to trigger reset
                    obs = np.zeros(expected_shape, dtype=np.float32)
                return obs, -10.0, True, False, {'error': 'observation_shape_mismatch'}
        except Exception as e:
            logging.error(f"Failed to get observation: {e}")
            # Return zero observation and mark as done to trigger reset
            obs = self._get_zero_observation()
            return obs, -10.0, True, False, {'error': 'observation_failed'}
        
        # Compute reward
        try:
            reward = self._compute_reward()
            # Track episode reward for reward-based curriculum
            if self.reward_based_curriculum:
                self.current_episode_reward += reward
        except Exception as e:
            logging.warning(f"Failed to compute reward: {e}")
            reward = 0.0
        
        # Check if done
        try:
            done = self._is_done()
            # Store episode reward when done
            if done and self.reward_based_curriculum:
                self.episode_rewards.append(self.current_episode_reward)
                self.current_episode_reward = 0.0
        except Exception as e:
            logging.warning(f"Failed to check done: {e}")
            done = False
        
        truncated = False  # Not using time limits for now
        
        step_duration = time.time() - step_start_time
        logging.info(f"Step {step_count}: Completed in {step_duration:.3f}s, reward={reward:.2f}, done={done}, truncated={truncated}")
        if step_duration > 0.5:  # Log if entire step takes too long
            logging.warning(f"Step {step_count} took {step_duration:.3f}s (may be slow)")
        
        # Get vehicle info safely
        try:
            if self.vehicle and self.vehicle.is_alive:
                velocity = self.vehicle.get_velocity()
                speed = 3.6 * np.linalg.norm([velocity.x, velocity.y, velocity.z])
            else:
                speed = 0.0
        except:
            speed = 0.0
        
        # Update evaluation metrics (Research Recommendations)
        if self.vehicle and self.vehicle.is_alive:
            # Track distance traveled
            if self.last_location is not None:
                current_location = self.vehicle.get_transform().location
                distance = np.sqrt(
                    (current_location.x - self.last_location.x)**2 +
                    (current_location.y - self.last_location.y)**2
                )
                self.episode_metrics['total_distance'] += distance
                self.last_location = current_location
            else:
                self.last_location = self.vehicle.get_transform().location
            
            # Track lane keeping
            try:
                waypoint = self.world.get_map().get_waypoint(self.vehicle.get_transform().location)
                vehicle_location = self.vehicle.get_transform().location
                lane_center = waypoint.transform.location
                distance_from_center = np.sqrt(
                    (vehicle_location.x - lane_center.x)**2 +
                    (vehicle_location.y - lane_center.y)**2
                )
                if distance_from_center < self.reward_config.get('lane_center_tolerance', 0.5):
                    self.episode_metrics['lane_keeping_time'] += 1
            except:
                pass
            
            # Track speeds for variance calculation
            self.episode_metrics['speeds'].append(speed)
            if len(self.episode_metrics['speeds']) > 100:
                self.episode_metrics['speeds'] = self.episode_metrics['speeds'][-100:]  # Keep last 100
            
            # Track jerk (smoothness)
            if hasattr(self, 'last_steering'):
                current_steering = control.steer
                jerk = abs(current_steering - self.last_steering)
                self.episode_metrics['jerk_sum'] += jerk
                self.last_steering = current_steering
        
        self.episode_metrics['total_steps'] += 1
        
        # Comprehensive info dict (Research Recommendations)
        info = {
            'collision': self.collision_occurred,
            'speed': speed,
            # Evaluation metrics
            'total_distance': self.episode_metrics['total_distance'],
            'lane_keeping_ratio': self.episode_metrics['lane_keeping_time'] / max(self.episode_metrics['total_steps'], 1),
            'smoothness': 1.0 / (1.0 + self.episode_metrics['jerk_sum'] / max(self.episode_metrics['total_steps'], 1)),  # Inverse jerk
            'speed_consistency': 1.0 / (1.0 + np.std(self.episode_metrics['speeds']) if len(self.episode_metrics['speeds']) > 1 else 1.0)
        }
        
        return obs, reward, done, truncated, info
    
    def update_curriculum(self, timestep: int):
        """Update curriculum difficulty based on timestep or reward"""
        if not self.curriculum_enabled:
            return
        
        if self.reward_based_curriculum:
            # Reward-based: advance if mean reward exceeds threshold
            if len(self.episode_rewards) >= self.reward_window_size:
                mean_reward = sum(self.episode_rewards) / len(self.episode_rewards)
                if mean_reward >= self.reward_threshold:
                    # Advance to next phase
                    old_difficulty = self.current_difficulty
                    self.current_difficulty = min(
                        self.current_difficulty + self.difficulty_increase_rate * 10,  # Faster advance when reward is good
                        self.max_difficulty
                    )
                    if self.current_difficulty > old_difficulty:
                        logging.info(f"🎯 Reward-based curriculum: Mean reward {mean_reward:.2f} >= {self.reward_threshold:.2f}, "
                                   f"advancing difficulty {old_difficulty:.3f} → {self.current_difficulty:.3f}")
        else:
            # Time-based: increase difficulty gradually
            update_freq = 10000  # Update every 10k steps
            if timestep % update_freq == 0:
                old_difficulty = self.current_difficulty
                self.current_difficulty = min(
                    self.current_difficulty + self.difficulty_increase_rate,
                    self.max_difficulty
                )
                if self.current_difficulty > old_difficulty:
                    logging.info(f"📈 Time-based curriculum: Difficulty {old_difficulty:.3f} → {self.current_difficulty:.3f} at step {timestep}")
    
    def update_reward_scale(self, timestep: int):
        """Update reward scaling progressively"""
        if not self.progressive_rewards_enabled:
            return
        
        update_freq = 10000
        if timestep % update_freq == 0:
            old_scale = self.reward_scale
            self.reward_scale = min(
                self.reward_scale + self.reward_scale_increase_rate,
                self.final_reward_scale
            )
            if self.reward_scale > old_scale:
                logging.info(f"📊 Reward scale: {old_scale:.3f} → {self.reward_scale:.3f} at step {timestep}")
    
    def close(self):
        """Clean up environment - FIXED: Properly destroy all actors to prevent resource leaks"""
        try:
            logging.info("🧹 Starting environment cleanup...")
            
            # FIXED: Stop all sensors first (before destroying)
            if self.rgb_camera is not None:
                try:
                    self.rgb_camera.stop()
                    logging.debug("RGB camera stopped")
                except Exception as e:
                    logging.warning(f"Error stopping RGB camera: {e}")
            
            if self.depth_camera is not None:
                try:
                    self.depth_camera.stop()
                    logging.debug("Depth camera stopped")
                except Exception as e:
                    logging.warning(f"Error stopping depth camera: {e}")
            
            if self.collision_sensor is not None:
                try:
                    self.collision_sensor.stop()
                    logging.debug("Collision sensor stopped")
                except Exception as e:
                    logging.warning(f"Error stopping collision sensor: {e}")
            
            if self.gps_sensor is not None:
                try:
                    self.gps_sensor.stop()
                    logging.debug("GPS sensor stopped")
                except Exception as e:
                    logging.warning(f"Error stopping GPS sensor: {e}")
            
            # FIXED: Destroy all actors properly (including vehicle)
            # This is necessary to prevent resource leaks that cause hanging
            actors_to_destroy = []
            
            # Collect all actors
            if self.vehicle is not None:
                actors_to_destroy.append(self.vehicle)
            if self.rgb_camera is not None:
                actors_to_destroy.append(self.rgb_camera)
            if self.depth_camera is not None:
                actors_to_destroy.append(self.depth_camera)
            if self.collision_sensor is not None:
                actors_to_destroy.append(self.collision_sensor)
            if self.gps_sensor is not None:
                actors_to_destroy.append(self.gps_sensor)
            
            # Also destroy any other actors we spawned
            for actor in self.actors:
                if actor not in actors_to_destroy:
                    actors_to_destroy.append(actor)
            
            # Destroy all actors with timeout protection
            for actor in actors_to_destroy:
                try:
                    if actor is not None and hasattr(actor, 'is_alive') and actor.is_alive:
                        # FIXED: Use timeout to prevent hanging on destroy
                        import threading
                        destroy_completed = threading.Event()
                        destroy_error = [None]
                        
                        def do_destroy():
                            try:
                                actor.destroy()
                                destroy_completed.set()
                            except Exception as e:
                                destroy_error[0] = e
                                destroy_completed.set()
                        
                        destroy_thread = threading.Thread(target=do_destroy, daemon=True)
                        destroy_thread.start()
                        destroy_thread.join(timeout=1.0)  # 1 second timeout per actor
                        
                        if not destroy_completed.is_set():
                            logging.warning(f"Actor destroy timeout: {actor.type_id if hasattr(actor, 'type_id') else 'unknown'}")
                        elif destroy_error[0] is not None:
                            logging.warning(f"Error destroying actor: {destroy_error[0]}")
                except Exception as e:
                    logging.warning(f"Error destroying actor: {e}")
            
            # Clear actor references
            self.actors = []
            self.vehicle = None
            self.rgb_camera = None
            self.depth_camera = None
            self.collision_sensor = None
            self.gps_sensor = None
            
            # FIXED: Reset world settings with timeout
            if self.world is not None:
                try:
                    import threading
                    settings_completed = threading.Event()
                    settings_error = [None]
                    
                    def do_settings():
                try:
                    settings = self.world.get_settings()
                    settings.synchronous_mode = False
                    self.world.apply_settings(settings)
                            settings_completed.set()
                        except Exception as e:
                            settings_error[0] = e
                            settings_completed.set()
                    
                    settings_thread = threading.Thread(target=do_settings, daemon=True)
                    settings_thread.start()
                    settings_thread.join(timeout=2.0)
                    
                    if not settings_completed.is_set():
                        logging.warning("World settings reset timeout")
                    elif settings_error[0] is None:
                    logging.info("✅ World settings reset (async mode)")
                    else:
                        logging.warning(f"Error resetting world settings: {settings_error[0]}")
                except Exception as e:
                    logging.warning(f"Error resetting world settings: {e}")
            
            # FIXED: Properly disconnect CARLA client
            if self.client is not None:
                try:
                    # CARLA client doesn't have explicit disconnect, but we should clear references
                    # and ensure no pending operations
                    self.client = None
                    self.world = None
                    logging.info("✅ CARLA client disconnected")
                except Exception as e:
                    logging.warning(f"Error disconnecting CARLA client: {e}")
            
            logging.info("✅ Environment closed (all actors destroyed)")
        except Exception as e:
            logging.error(f"Error during environment cleanup: {e}", exc_info=True)

