import gymnasium as gym
import numpy as np
import carla
import random
import time
from typing import Dict, Tuple, Optional, Any
from .sensors import SensorManager
from .rewards import RewardCalculator
from .utils import get_speed, get_transform_matrix


class CarlaEnv(gym.Env):
    """
    CARLA Gymnasium Environment for autonomous driving with SAC training.
    
    State Space:
        - LiDAR BEV occupancy grid (256x256, multi-channel)
        - Ego state: [speed, heading, steering, acceleration]
        - Waypoint information: [lateral_offset, heading_error]
    
    Action Space:
        - Continuous: [steering ∈ [-1,1], throttle ∈ [0,1], brake ∈ [0,1]]
    """
    
    metadata = {'render_modes': ['human', 'rgb_array']}
    
    def __init__(self, config: Dict[str, Any]):
        super().__init__()
        
        # Configuration
        self.config = config
        self.host = config.get('host', 'localhost')
        self.port = config.get('port', 2000)
        self.timeout = config.get('timeout', 10.0)
        self.map_name = config.get('map', 'Town01')
        self.delta_seconds = config.get('delta_seconds', 0.05)
        
        # CARLA connection
        self.client = None
        self.world = None
        self.map = None
        self.vehicle = None
        self.traffic_manager = None
        
        # Sensors
        self.sensor_manager = None
        
        # Reward calculator
        self.reward_calculator = RewardCalculator(config.get('reward_config', {}))
        
        # Episode tracking
        self.episode_step = 0
        self.max_episode_steps = config.get('max_episode_steps', 1000)
        self.collision_occurred = False
        
        # State tracking
        self.prev_location = None
        self.prev_speed = 0.0
        
        # Fixed spawn points for consistent training
        self.use_fixed_spawn = config.get('use_fixed_spawn', True)
        self.fixed_spawn_indices = config.get('fixed_spawn_indices', [0, 1, 2])
        self.current_spawn_index = 0
        
        # Camera support
        self.use_camera = config.get('use_camera', True)
        # Skip perception sensors entirely (used when an upstream wrapper
        # supplies ground-truth observations and does not need LiDAR/RGB).
        self.disable_perception_sensors = config.get(
            'disable_perception_sensors', False,
        )
        
        # Define observation space
        # LiDAR BEV grid (256x256x3) + ego state (6 values) + camera (optional)
        obs_spaces = {
            'lidar_bev': gym.spaces.Box(
                low=0, high=255, 
                shape=(256, 256, 3), 
                dtype=np.uint8
            ),
            'ego_state': gym.spaces.Box(
                low=np.array([-np.inf, -np.pi, -1.0, -10.0, -np.inf, -np.pi]),
                high=np.array([np.inf, np.pi, 1.0, 10.0, np.inf, np.pi]),
                shape=(6,),
                dtype=np.float32
            )
        }
        
        if self.use_camera:
            obs_spaces['camera'] = gym.spaces.Box(
                low=0, high=255,
                shape=(480, 640, 3),
                dtype=np.uint8
            )
        
        self.observation_space = gym.spaces.Dict(obs_spaces)
        
        # Define action space: [steering, throttle, brake]
        self.action_space = gym.spaces.Box(
            low=np.array([-1.0, 0.0, 0.0]),
            high=np.array([1.0, 1.0, 1.0]),
            dtype=np.float32
        )
        
        # Initialize CARLA connection
        self._connect_to_carla()
    
    def _connect_to_carla(self):
        """Connect to CARLA server and setup world."""
        try:
            self.client = carla.Client(self.host, self.port)
            self.client.set_timeout(self.timeout)
            
            # Load map
            if self.client.get_world().get_map().name != self.map_name:
                self.world = self.client.load_world(self.map_name)
            else:
                self.world = self.client.get_world()
            
            self.map = self.world.get_map()
            
            # Set synchronous mode
            settings = self.world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = self.delta_seconds
            self.world.apply_settings(settings)
            
            # NOTE: TrafficManager is NOT started here. CARLA 0.9.16's local TM
            # runs a background thread that calls world.get_actors() via RPC;
            # if the RPC session dies (e.g. when Ray restarts a worker) the TM
            # thread throws an uncaught C++ exception -> std::terminate ->
            # SIGABRT of the whole Python process. We don't need TM in this
            # pipeline (no NPC autopilot), so skip it entirely.
            self.traffic_manager = None
            
            print(f"Connected to CARLA server at {self.host}:{self.port}")
            print(f"Loaded map: {self.map_name}")
            
        except Exception as e:
            print(f"Failed to connect to CARLA: {e}")
            raise
    
    def _spawn_vehicle(self) -> carla.Actor:
        """Spawn ego vehicle; tries fixed points first then any random point.

        Robust to transient spawn collisions (e.g. wreckage from the previous
        episode not yet despawned). We try up to `MAX_ATTEMPTS` points before
        giving up.
        """
        MAX_ATTEMPTS = 30
        blueprint_library = self.world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]
        spawn_points = self.map.get_spawn_points()
        if not spawn_points:
            raise RuntimeError("Map has no spawn points")

        # Candidate order: fixed indices first (if enabled), then shuffled rest.
        candidates = []
        if self.use_fixed_spawn and self.fixed_spawn_indices:
            for i in self.fixed_spawn_indices:
                if 0 <= i < len(spawn_points):
                    candidates.append((i, spawn_points[i]))
        remaining = [
            (i, sp) for i, sp in enumerate(spawn_points)
            if i not in {c[0] for c in candidates}
        ]
        random.shuffle(remaining)
        candidates.extend(remaining)

        last_err = None
        for attempt, (idx, sp) in enumerate(candidates[:MAX_ATTEMPTS]):
            vehicle = self.world.try_spawn_actor(vehicle_bp, sp)
            if vehicle is not None:
                self.current_spawn_index = idx + 1
                print(
                    f"[CarlaEnv] Spawned ego id={vehicle.id} at sp_idx={idx} "
                    f"({sp.location.x:.1f},{sp.location.y:.1f}) "
                    f"(attempt {attempt + 1})"
                )
                return vehicle
            last_err = f"sp_idx={idx} blocked"

        raise RuntimeError(
            f"Failed to spawn vehicle after {MAX_ATTEMPTS} attempts ({last_err})",
        )
    
    def _setup_sensors(self):
        """Setup all sensors for the vehicle."""
        sensor_cfg = dict(self.config.get('sensor_config', {}))
        # Propagate the "no perception" flag so SensorManager can skip
        # LiDAR/RGB; we still need the collision sensor for termination.
        sensor_cfg['disable_perception_sensors'] = self.disable_perception_sensors
        self.sensor_manager = SensorManager(
            self.world, 
            self.vehicle,
            sensor_cfg,
        )
        self.sensor_manager.setup_sensors()
        
        # Register collision callback
        self.sensor_manager.register_collision_callback(self._on_collision)
    
    def _on_collision(self, event):
        """Callback for collision events."""
        self.collision_occurred = True
    
    def reset(self, seed: Optional[int] = None, options: Optional[Dict] = None) -> Tuple[Dict, Dict]:
        """Reset the environment."""
        super().reset(seed=seed)
        
        # Clean up previous episode
        if self.sensor_manager is not None:
            self.sensor_manager.destroy()
        if self.vehicle is not None:
            self.vehicle.destroy()
        
        # Spawn new vehicle
        self.vehicle = self._spawn_vehicle()
        
        # Setup sensors
        self._setup_sensors()
        
        # Reset episode tracking
        self.episode_step = 0
        self.collision_occurred = False
        self.prev_location = self.vehicle.get_location()
        self.prev_speed = 0.0
        
        # Tick world to initialize sensors
        for _ in range(5):
            self.world.tick()
        
        # Get initial observation
        obs = self._get_obs()
        info = self._get_info()
        
        return obs, info
    
    def step(self, action: np.ndarray) -> Tuple[Dict, float, bool, bool, Dict]:
        """Execute one step in the environment."""
        # Apply action to vehicle.
        # Throttle and brake are kept mutually exclusive: SAC's initial
        # policy samples ~N(0, 1) and squashes to [0,1], which makes throttle
        # and brake average ~0.5 simultaneously -- the car never moves. Use
        # the dominant control only so exploration can actually drive.
        steering, throttle, brake = action
        throttle = float(np.clip(throttle, 0.0, 1.0))
        brake = float(np.clip(brake, 0.0, 1.0))
        if throttle >= brake:
            brake = 0.0
        else:
            throttle = 0.0
        control = carla.VehicleControl(
            throttle=throttle,
            steer=float(np.clip(steering, -1.0, 1.0)),
            brake=brake,
        )
        self.vehicle.apply_control(control)
        
        # Tick world
        self.world.tick()
        self.episode_step += 1
        
        # Get observation
        obs = self._get_obs()
        
        # Calculate reward
        reward = self._compute_reward(action)
        
        # Check termination conditions
        terminated = self._is_done()
        truncated = self.episode_step >= self.max_episode_steps
        
        # Get info
        info = self._get_info()
        
        return obs, reward, terminated, truncated, info
    
    def _get_obs(self) -> Dict[str, np.ndarray]:
        """Get current observation."""
        # Get LiDAR BEV grid (zeros when perception sensors are disabled --
        # wrappers like GTStateWrapper replace the obs entirely, so we just
        # need a valid-shape tensor to satisfy the declared observation_space).
        if self.disable_perception_sensors:
            lidar_bev = np.zeros((256, 256, 3), dtype=np.uint8)
        else:
            lidar_bev = self.sensor_manager.get_lidar_bev()
        
        # Get ego state
        transform = self.vehicle.get_transform()
        velocity = self.vehicle.get_velocity()
        control = self.vehicle.get_control()
        acceleration = self.vehicle.get_acceleration()
        
        speed = get_speed(velocity)
        heading = np.radians(transform.rotation.yaw)
        steering = control.steer
        accel = np.sqrt(acceleration.x**2 + acceleration.y**2 + acceleration.z**2)
        
        # Get waypoint information
        location = transform.location
        waypoint = self.map.get_waypoint(location)
        waypoint_transform = waypoint.transform
        
        # Calculate lateral offset and heading error
        lateral_offset = self._calculate_lateral_offset(location, waypoint_transform)
        heading_error = self._calculate_heading_error(transform.rotation.yaw, waypoint_transform.rotation.yaw)
        
        ego_state = np.array([
            speed,
            heading,
            steering,
            accel,
            lateral_offset,
            heading_error
        ], dtype=np.float32)
        
        obs = {
            'lidar_bev': lidar_bev,
            'ego_state': ego_state
        }
        
        # Add camera image if enabled
        if self.use_camera:
            camera_img = self.sensor_manager.get_camera_image()
            if camera_img is not None:
                obs['camera'] = camera_img
            else:
                # Return black image if camera not ready
                obs['camera'] = np.zeros((480, 640, 3), dtype=np.uint8)
        
        return obs
    
    def _compute_reward(self, action: np.ndarray) -> float:
        """Compute reward for current step."""
        transform = self.vehicle.get_transform()
        velocity = self.vehicle.get_velocity()
        location = transform.location
        
        # Get current state
        speed = get_speed(velocity)
        waypoint = self.map.get_waypoint(location)
        
        # Calculate various reward components
        reward_data = {
            'speed': speed,
            'prev_speed': self.prev_speed,
            'location': location,
            'prev_location': self.prev_location,
            'waypoint': waypoint,
            'collision': self.collision_occurred,
            'action': action,
            'heading': transform.rotation.yaw,
            'waypoint_heading': waypoint.transform.rotation.yaw
        }
        
        reward = self.reward_calculator.calculate(reward_data)
        
        # Update previous state
        self.prev_location = location
        self.prev_speed = speed
        
        return reward
    
    def _is_done(self) -> bool:
        """Check if episode is done."""
        if self.collision_occurred:
            return True
        
        # Check if vehicle is off-road
        location = self.vehicle.get_location()
        waypoint = self.map.get_waypoint(location)
        
        if waypoint is None:
            return True
        
        # Check lateral deviation
        lateral_offset = self._calculate_lateral_offset(location, waypoint.transform)
        if abs(lateral_offset) > 3.0:  # 3 meters off center
            return True
        
        return False
    
    def _get_info(self) -> Dict[str, Any]:
        """Get additional info."""
        transform = self.vehicle.get_transform()
        velocity = self.vehicle.get_velocity()
        
        return {
            'episode_step': self.episode_step,
            'speed': get_speed(velocity),
            'location': (transform.location.x, transform.location.y, transform.location.z),
            'collision': self.collision_occurred
        }
    
    def _calculate_lateral_offset(self, location: carla.Location, waypoint_transform: carla.Transform) -> float:
        """Calculate lateral offset from waypoint center."""
        waypoint_loc = waypoint_transform.location
        waypoint_yaw = np.radians(waypoint_transform.rotation.yaw)
        
        # Vector from waypoint to vehicle
        dx = location.x - waypoint_loc.x
        dy = location.y - waypoint_loc.y
        
        # Project onto perpendicular to waypoint direction
        lateral_offset = -dx * np.sin(waypoint_yaw) + dy * np.cos(waypoint_yaw)
        
        return lateral_offset
    
    def _calculate_heading_error(self, vehicle_yaw: float, waypoint_yaw: float) -> float:
        """Calculate heading error relative to waypoint."""
        error = vehicle_yaw - waypoint_yaw
        
        # Normalize to [-180, 180]
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        
        return np.radians(error)
    
    def render(self, mode='human'):
        """Render the environment."""
        if mode == 'rgb_array':
            return self.sensor_manager.get_camera_image()
        return None
    
    def close(self):
        """Clean up resources."""
        if self.sensor_manager is not None:
            self.sensor_manager.destroy()
        if self.vehicle is not None:
            self.vehicle.destroy()
        
        # Restore asynchronous mode
        if self.world is not None:
            settings = self.world.get_settings()
            settings.synchronous_mode = False
            self.world.apply_settings(settings)
        
        print("Environment closed")
