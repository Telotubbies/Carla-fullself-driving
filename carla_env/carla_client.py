"""
CARLA Client for managing simulation environment.

Handles connection, world setup, vehicle spawning, and sensor management.
"""

import carla
import logging
from typing import Optional, Dict, Any
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
        
        self._connected = False
        
    def connect(self) -> bool:
        """
        Connect to CARLA server.
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            logger.info(f"Connecting to CARLA at {self.config['host']}:{self.config['port']}")
            self.client = carla.Client(
                self.config['host'],
                self.config['port']
            )
            self.client.set_timeout(self.config.get('timeout', 10.0))
            
            # Test connection
            world = self.client.get_world()
            logger.info("✅ Successfully connected to CARLA")
            
            self._connected = True
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to connect to CARLA: {e}")
            self._connected = False
            return False
    
    def load_world(self) -> bool:
        """
        Load specified town.
        
        Returns:
            True if world loaded successfully
        """
        try:
            town = self.config.get('town', 'Town04')
            logger.info(f"Loading world: {town}")
            self.world = self.client.load_world(town)
            
            # Wait for world to be ready
            time.sleep(2.0)
            
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
    
    def spawn_vehicle(self) -> bool:
        """
        Spawn vehicle at specified spawn point.
        
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
            spawn_index = self.config.get('spawn_point_index', 0)
            
            if spawn_index >= len(spawn_points):
                logger.warning(f"Spawn point index {spawn_index} out of range, using 0")
                spawn_index = 0
            
            self.spawn_point = spawn_points[spawn_index]
            logger.info(f"Using spawn point {spawn_index}: {self.spawn_point.location}")
            
            # Spawn vehicle
            self.vehicle = self.world.spawn_actor(vehicle_bp, self.spawn_point)
            logger.info(f"✅ Vehicle spawned: {self.vehicle.id}")
            
            # Set vehicle physics
            physics_control = self.vehicle.get_physics_control()
            # Keep default physics for now
            
            return True
            
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
    
    def cleanup(self) -> None:
        """Clean up CARLA resources."""
        logger.info("Cleaning up CARLA resources...")
        
        if self.vehicle is not None:
            try:
                self.vehicle.destroy()
                logger.info("✅ Vehicle destroyed")
            except Exception as e:
                logger.warning(f"Error destroying vehicle: {e}")
        
        # Disable synchronous mode
        if self.world is not None:
            try:
                settings = self.world.get_settings()
                settings.synchronous_mode = False
                self.world.apply_settings(settings)
            except Exception as e:
                logger.warning(f"Error disabling synchronous mode: {e}")
        
        self._connected = False
        logger.info("✅ Cleanup completed")
    
    def is_connected(self) -> bool:
        """Check if client is connected."""
        return self._connected

