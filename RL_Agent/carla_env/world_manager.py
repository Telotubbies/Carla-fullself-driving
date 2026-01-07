"""
World Manager

Manages CARLA world settings, weather, and synchronous mode.
Follows Single Responsibility Principle - only handles world configuration.
"""

import carla
import logging
import random
from typing import Dict, Any, Optional


class WorldManager:
    """Manages CARLA world settings and configuration."""
    
    def __init__(self, world: carla.World, config: Dict[str, Any]):
        """
        Initialize world manager.
        
        Args:
            world: CARLA world object
            config: Environment configuration dictionary
        """
        self.world = world
        self.config = config.get('environment', {})
    
    def apply_settings(self) -> None:
        """Apply world settings including synchronous mode and rendering."""
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05  # 20 FPS
        settings.no_rendering_mode = self.config.get('no_rendering_mode', False)
        
        logging.info(f"🎨 No Rendering Mode: {'ENABLED' if settings.no_rendering_mode else 'DISABLED'}")
        self.world.apply_settings(settings)
        
        self.setup_weather()
    
    def setup_weather(self) -> None:
        """Set weather parameters, optionally randomized."""
        domain_rand = self.config.get('domain_randomization', {})
        
        if domain_rand.get('enabled', False) and domain_rand.get('weather_randomization', False):
            weather = self._create_random_weather(domain_rand)
        else:
            weather = self._create_default_weather()
        
        self.world.set_weather(weather)
    
    def _create_random_weather(self, domain_rand: Dict[str, Any]) -> carla.WeatherParameters:
        """Create randomized weather parameters."""
        ranges = domain_rand.get('weather', {})
        return carla.WeatherParameters(
            cloudiness=random.uniform(*ranges.get('cloudiness_range', [0, 0])),
            precipitation=random.uniform(*ranges.get('precipitation_range', [0, 0])),
            sun_altitude_angle=random.uniform(*ranges.get('sun_altitude_range', [45, 45])),
            fog_density=random.uniform(*ranges.get('fog_density_range', [0, 0]))
        )
    
    def _create_default_weather(self) -> carla.WeatherParameters:
        """Create default weather parameters."""
        w_conf = self.config.get('weather', {})
        return carla.WeatherParameters(
            cloudiness=w_conf.get('cloudiness', 0.0),
            precipitation=w_conf.get('precipitation', 0.0),
            sun_altitude_angle=w_conf.get('sun_altitude_angle', 45.0)
        )
    
    def reset_settings(self) -> None:
        """Reset world settings to default (disable synchronous mode)."""
        try:
            settings = self.world.get_settings()
            settings.synchronous_mode = False
            self.world.apply_settings(settings)
        except Exception as e:
            logging.debug(f"Error resetting world settings: {e}")

