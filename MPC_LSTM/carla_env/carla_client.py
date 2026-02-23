"""
CARLA Client for managing simulation environment.

Handles connection, world setup, vehicle spawning, and sensor management.
All spawned actors are tracked in an internal registry and destroyed
automatically on cleanup — no manual killing required between runs.
"""

import carla
import logging
from typing import Optional, Dict, Any, List
import time

logger = logging.getLogger(__name__)


class CarlaClient:
    """Manages CARLA simulation client and environment setup."""
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize CARLA client.
        
        Args:
            config: Configuration dictionary with CARLA settings
        """
        self.config = config
        self.client: Optional[carla.Client] = None
        self.world: Optional[carla.World] = None
        self.vehicle: Optional[carla.Vehicle] = None
        self.blueprint_library: Optional[carla.BlueprintLibrary] = None
        self.spawn_point: Optional[carla.Transform] = None

        # Registry of every actor spawned through this client.
        # cleanup() will destroy all of them so they never block other vehicles.
        self._actor_registry: List[carla.Actor] = []

        self._connected = False
        self._cleaned_up = False
        
    def connect(self, max_retries: int = 5) -> bool:
        """
        Connect to CARLA server with retry logic and port checking.
        
        Args:
            max_retries: Maximum number of retry attempts
            
        Returns:
            True if connection successful, False otherwise
        """
        host = self.config['host']
        port = self.config['port']
        timeout = self.config.get('timeout', 20.0)
        
        # First, check if port is accessible
        import socket
        port_ready = False
        for port_check in range(5):
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(2.0)
                result = sock.connect_ex((host, port))
                sock.close()
                if result == 0:
                    port_ready = True
                    logger.info(f"✅ Port {port} is accessible")
                    break
                else:
                    if port_check < 4:
                        logger.debug(f"Port {port} not ready, waiting...")
                        time.sleep(2.0)
            except Exception as e:
                logger.debug(f"Port check failed: {e}")
                time.sleep(2.0)
        
        if not port_ready:
            logger.warning(f"⚠️  Port {port} is not accessible, but will try to connect anyway...")
        
        for attempt in range(max_retries):
            try:
                if attempt == 0:
                    logger.info(f"Connecting to CARLA at {host}:{port}")
                else:
                    wait_time = min(5 + (attempt * 2), 15)
                    logger.info(f"Retrying connection (attempt {attempt + 1}/{max_retries}) in {wait_time}s...")
                    time.sleep(wait_time)
                
                # Create client with longer timeout for initial connection
                self.client = carla.Client(host, port)
                # Use longer timeout for initial connection
                self.client.set_timeout(max(timeout, 30.0))
                
                # Test connection with multiple attempts
                for test_attempt in range(5):
                    try:
                        # Try to get world
                        world = self.client.get_world()
                        # Verify world is accessible
                        _ = world.get_map()
                        logger.info("✅ Successfully connected to CARLA")
                        self._connected = True
                        return True
                    except Exception as test_e:
                        if test_attempt < 4:
                            wait = 2.0 * (test_attempt + 1)
                            logger.debug(f"Connection test {test_attempt + 1} failed, waiting {wait}s...")
                            time.sleep(wait)
                            continue
                        else:
                            raise test_e
                
            except Exception as e:
                if attempt < max_retries - 1:
                    logger.warning(f"Connection attempt {attempt + 1} failed: {e}")
                else:
                    logger.error(f"❌ Failed to connect to CARLA after {max_retries} attempts: {e}")
                    logger.info("💡 Suggestions:")
                    logger.info("   1. Make sure CARLA is fully started (may take 30-60 seconds)")
                    logger.info("   2. Check if CARLA is listening on port 2000")
                    logger.info("   3. Try restarting CARLA")
                    self._connected = False
                    return False
        
        self._connected = False
        return False
    
    def load_world(self, retries: int = 3) -> bool:
        """
        Load specified town with retry logic.
        
        Args:
            retries: Number of retry attempts
            
        Returns:
            True if world loaded successfully
        """
        town = self.config.get('town', 'Town04')
        
        for attempt in range(retries):
            try:
                logger.info(f"Loading world: {town} (attempt {attempt + 1}/{retries})")
                
                # Try to get existing world first
                try:
                    self.world = self.client.get_world()
                    current_map = self.world.get_map().name
                    if town in current_map:
                        logger.info(f"World {town} already loaded")
                    else:
                        # Load new world
                        self.world = self.client.load_world(town)
                except Exception:
                    # If get_world fails, try load_world
                    self.world = self.client.load_world(town)
                
                # Wait for world to be ready (longer wait)
                time.sleep(3.0)
                
                # Verify world is accessible
                _ = self.world.get_map()
                break  # Success
                
            except Exception as e:
                if attempt < retries - 1:
                    logger.warning(f"Attempt {attempt + 1} failed: {e}, retrying...")
                    time.sleep(2.0)
                else:
                    logger.error(f"❌ Failed to load world after {retries} attempts: {e}")
                    return False
        
        try:
            
            # Set synchronous mode if configured
            if self.config.get('synchronous_mode', True):
                settings = self.world.get_settings()
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = self.config.get('fixed_delta_seconds', 0.05)
                self.world.apply_settings(settings)
                logger.info("✅ Synchronous mode enabled")
            
            # Set weather
            self._set_weather()
            
            # Get blueprint library
            self.blueprint_library = self.world.get_blueprint_library()
            
            logger.info(f"✅ World {town} loaded successfully")
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to load world: {e}")
            return False
    
    def _set_weather(self) -> None:
        """Set weather conditions."""
        weather_name = self.config.get('weather', 'ClearNoon')
        weather_presets = {
            'ClearNoon': carla.WeatherParameters.ClearNoon,
            'ClearSunset': carla.WeatherParameters.ClearSunset,
            'CloudyNoon': carla.WeatherParameters.CloudyNoon,
            'CloudySunset': carla.WeatherParameters.CloudySunset,
            'WetNoon': carla.WeatherParameters.WetNoon,
            'WetSunset': carla.WeatherParameters.WetSunset,
            'MidRainyNoon': carla.WeatherParameters.MidRainyNoon,
            'MidRainSunset': carla.WeatherParameters.MidRainSunset,
            'HardRainNoon': carla.WeatherParameters.HardRainNoon,
            'HardRainSunset': carla.WeatherParameters.HardRainSunset,
            'SoftRainNoon': carla.WeatherParameters.SoftRainNoon,
            'SoftRainSunset': carla.WeatherParameters.SoftRainSunset,
        }
        
        if weather_name in weather_presets:
            self.world.set_weather(weather_presets[weather_name])
            logger.info(f"✅ Weather set to {weather_name}")
        else:
            logger.warning(f"Unknown weather preset: {weather_name}, using ClearNoon")
            self.world.set_weather(carla.WeatherParameters.ClearNoon)
    
    def spawn_vehicle(self, max_attempts: int = 10) -> bool:
        """
        Spawn vehicle at specified spawn point, trying multiple points if needed.
        
        Args:
            max_attempts: Maximum number of spawn points to try
        
        Returns:
            True if vehicle spawned successfully
        """
        try:
            vehicle_name = self.config.get('vehicle', 'vehicle.tesla.model3')
            logger.info(f"Spawning vehicle: {vehicle_name}")
            
            # Get vehicle blueprint
            vehicle_bp = self.blueprint_library.find(vehicle_name)
            if vehicle_bp is None:
                logger.error(f"❌ Vehicle blueprint not found: {vehicle_name}")
                return False
            
            # Get spawn points
            spawn_points = self.world.get_map().get_spawn_points()
            if len(spawn_points) == 0:
                logger.error("❌ No spawn points available")
                return False
            
            # Try preferred spawn point first
            preferred_index = self.config.get('spawn_point_index', 0)
            if preferred_index >= len(spawn_points):
                logger.warning(f"Spawn point index {preferred_index} out of range, using 0")
                preferred_index = 0
            
            # Try spawn points starting from preferred
            indices_to_try = [preferred_index] + [i for i in range(len(spawn_points)) if i != preferred_index][:max_attempts-1]
            
            for attempt, spawn_index in enumerate(indices_to_try[:max_attempts]):
                try:
                    self.spawn_point = spawn_points[spawn_index]
                    logger.info(f"Attempt {attempt+1}/{len(indices_to_try[:max_attempts])}: Trying spawn point {spawn_index}: {self.spawn_point.location}")
                    
                    # Spawn vehicle and register it for auto-cleanup
                    self.vehicle = self.world.spawn_actor(vehicle_bp, self.spawn_point)
                    self._register_actor(self.vehicle)
                    logger.info(f"✅ Vehicle spawned successfully at spawn point {spawn_index}: {self.vehicle.id}")

                    return True
                    
                except Exception as e:
                    if attempt < len(indices_to_try[:max_attempts]) - 1:
                        logger.warning(f"⚠️  Spawn failed at point {spawn_index}: {e}, trying next...")
                        continue
                    else:
                        logger.error(f"❌ Failed to spawn vehicle at all {max_attempts} attempted spawn points")
                        raise
            
            return False
            
        except Exception as e:
            logger.error(f"❌ Failed to spawn vehicle: {e}")
            return False
    
    def get_vehicle_state(self) -> Dict[str, Any]:
        """
        Get current vehicle state.
        
        Returns:
            Dictionary with vehicle state information
        """
        if self.vehicle is None:
            return {}
        
        transform = self.vehicle.get_transform()
        velocity = self.vehicle.get_velocity()
        angular_velocity = self.vehicle.get_angular_velocity()
        control = self.vehicle.get_control()
        
        # Calculate speed in m/s
        speed = (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5
        
        # Get yaw from rotation
        yaw = transform.rotation.yaw
        
        return {
            'x': transform.location.x,
            'y': transform.location.y,
            'z': transform.location.z,
            'yaw': yaw,
            'pitch': transform.rotation.pitch,
            'roll': transform.rotation.roll,
            'velocity': speed,
            'velocity_x': velocity.x,
            'velocity_y': velocity.y,
            'velocity_z': velocity.z,
            'angular_velocity': angular_velocity,
            'steering': control.steer,
            'throttle': control.throttle,
            'brake': control.brake,
        }
    
    def apply_control(self, control: carla.VehicleControl) -> None:
        """
        Apply control to vehicle.
        
        Args:
            control: CARLA VehicleControl object
        """
        if self.vehicle is not None:
            self.vehicle.apply_control(control)
    
    def tick(self) -> None:
        """Tick the world (for synchronous mode)."""
        if self.world is not None and self.config.get('synchronous_mode', True):
            self.world.tick()
    
    def register_actor(self, actor: carla.Actor) -> carla.Actor:
        """
        Register an external actor (sensor, NPC, etc.) for auto-cleanup.

        Any actor registered here will be destroyed when cleanup() is called,
        so nothing is left blocking the map between runs.

        Args:
            actor: A live CARLA actor returned by world.spawn_actor()

        Returns:
            The same actor (allows chaining: sensor = client.register_actor(world.spawn_actor(...)))
        """
        if actor is not None:
            self._actor_registry.append(actor)
        return actor

    def _register_actor(self, actor: carla.Actor) -> None:
        """Internal helper used by spawn_vehicle."""
        self.register_actor(actor)

    def cleanup(self) -> None:
        """
        Destroy every actor spawned through this client and restore world settings.

        Safe to call multiple times — subsequent calls are no-ops.
        This is invoked automatically via signal handlers and atexit in main.py,
        so the vehicle (and any sensors) are always removed even on Ctrl-C or crash.
        """
        if self._cleaned_up:
            return
        self._cleaned_up = True

        logger.info("Cleaning up CARLA resources...")

        # Destroy all tracked actors in reverse-spawn order (sensors before vehicle)
        destroyed = 0
        for actor in reversed(self._actor_registry):
            try:
                if actor is not None and actor.is_alive:
                    actor.destroy()
                    destroyed += 1
            except Exception as e:
                logger.warning(f"Error destroying actor {getattr(actor, 'id', '?')}: {e}")

        self._actor_registry.clear()
        self.vehicle = None

        if destroyed:
            logger.info(f"✅ {destroyed} actor(s) destroyed")
        else:
            logger.info("No actors to destroy")

        # Restore async mode so the next session starts clean
        if self.world is not None:
            try:
                settings = self.world.get_settings()
                settings.synchronous_mode = False
                self.world.apply_settings(settings)
                logger.info("✅ Synchronous mode disabled")
            except Exception as e:
                logger.warning(f"Error disabling synchronous mode: {e}")

        self._connected = False
        logger.info("✅ CARLA cleanup completed")
    
    def is_connected(self) -> bool:
        """Check if client is connected."""
        return self._connected
    
    def get_waypoints(self, distance: float = 50.0, num_waypoints: int = 50) -> Optional[list]:
        """
        Get waypoints from current vehicle location.
        
        Args:
            distance: Distance between waypoints in meters
            num_waypoints: Number of waypoints to return
            
        Returns:
            List of waypoints or None if vehicle not available
        """
        if self.vehicle is None or self.world is None:
            return None
        
        try:
            # Get current vehicle location
            vehicle_location = self.vehicle.get_location()
            
            # Get map
            carla_map = self.world.get_map()
            
            # Get nearest waypoint
            nearest_wp = carla_map.get_waypoint(vehicle_location)
            
            if nearest_wp is None:
                return None
            
            # Generate waypoints along the road
            waypoints = []
            current_wp = nearest_wp
            
            for i in range(num_waypoints):
                waypoints.append(current_wp)
                
                # Get next waypoint
                next_wps = current_wp.next(distance)
                if next_wps and len(next_wps) > 0:
                    # Choose the waypoint that continues straight (first one)
                    current_wp = next_wps[0]
                else:
                    # No more waypoints
                    break
            
            return waypoints if len(waypoints) > 0 else None
            
        except Exception as e:
            logger.warning(f"Error getting waypoints: {e}")
            return None

