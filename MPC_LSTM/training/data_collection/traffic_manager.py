"""
Traffic Manager Configuration for Diverse Data Collection.

Configures CARLA Traffic Manager for diverse driving behaviors.
"""

import logging
import random
import carla
from typing import Optional

logger = logging.getLogger(__name__)


class TrafficManagerConfig:
    """Configures Traffic Manager for diversity."""
    
    @staticmethod
    def configure_for_diversity(
        traffic_manager: carla.TrafficManager,
        vehicle: Optional[carla.Vehicle] = None,
        random_seed: Optional[int] = None
    ):
        """
        Configure Traffic Manager for diverse behaviors.
        
        Args:
            traffic_manager: CARLA Traffic Manager instance
            vehicle: Vehicle to configure (optional)
            random_seed: Random seed for reproducibility
        """
        if random_seed is not None:
            traffic_manager.set_random_device_seed(random_seed)
        
        # Set global distance to leading vehicle (more aggressive = more diverse)
        traffic_manager.set_global_distance_to_leading_vehicle(2.0)
        
        # Enable random lane changes
        traffic_manager.set_random_lane_changes(True)
        
        # Set vehicle-specific settings if provided
        if vehicle is not None:
            # Random speed variation (80-120% of speed limit)
            speed_percent = random.uniform(0.8, 1.2)
            traffic_manager.vehicle_percentage_speed_difference(vehicle.id, speed_percent * 100)
            
            # Random lane change probability
            lane_change_prob = random.uniform(0.3, 0.7)
            traffic_manager.set_lane_change_permission(vehicle.id, lane_change_prob > 0.5)
        
        logger.info("✅ Traffic Manager configured for diversity")
    
    @staticmethod
    def set_weather_diversity(world: carla.World, weather_preset: Optional[str] = None):
        """
        Set diverse weather conditions.
        
        Args:
            world: CARLA world instance
            weather_preset: Weather preset name (None = random)
        """
        weather_presets = [
            carla.WeatherParameters.ClearNoon,
            carla.WeatherParameters.CloudyNoon,
            carla.WeatherParameters.WetNoon,
            carla.WeatherParameters.WetCloudyNoon,
            carla.WeatherParameters.MidRainyNoon,
            carla.WeatherParameters.HardRainNoon,
            carla.WeatherParameters.SoftRainNoon,
            carla.WeatherParameters.ClearSunset,
            carla.WeatherParameters.CloudySunset,
            carla.WeatherParameters.WetSunset,
        ]
        
        if weather_preset is None:
            weather = random.choice(weather_presets)
        else:
            # Map preset name to weather
            weather = weather_presets[0]  # Default to ClearNoon
        
        world.set_weather(weather)
        logger.info(f"Weather set to: {weather}")


