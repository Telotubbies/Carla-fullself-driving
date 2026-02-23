"""
Configuration management with validation.
"""

import yaml
from pathlib import Path
from typing import Dict, Any, Optional, List
from dataclasses import dataclass, field
from .exceptions import ConfigurationError


@dataclass
class ConfigSchema:
    """Configuration schema with validation."""
    
    # CARLA
    carla_host: str = "localhost"
    carla_port: int = 2000
    carla_timeout: float = 10.0
    carla_town: str = "Town04"
    carla_vehicle: str = "vehicle.tesla.model3"
    carla_weather: str = "ClearNoon"
    carla_spawn_point_index: int = 0
    carla_synchronous_mode: bool = True
    carla_fixed_delta_seconds: float = 0.05
    
    # Camera
    camera_width: int = 640
    camera_height: int = 480
    camera_fov: int = 90
    camera_fps: int = 20
    
    # Perception
    perception_model: str = "resnet18"
    perception_pretrained: bool = True
    perception_freeze_backbone: bool = False
    perception_feature_dim: int = 512
    
    # Temporal
    temporal_input_size: int = 512
    temporal_hidden_size: int = 256
    temporal_num_layers: int = 2
    temporal_sequence_length: int = 10
    temporal_dropout: float = 0.1
    
    # MPC
    mpc_horizon: int = 10
    mpc_dt: float = 0.05
    mpc_max_steer: float = 0.5
    mpc_max_accel: float = 3.0
    mpc_max_decel: float = -3.0
    
    # Safety
    safety_emergency_brake_enabled: bool = True
    safety_max_speed_kmh: float = 30.0
    
    def validate(self) -> List[str]:
        """
        Validate configuration values.
        
        Returns:
            List of validation errors (empty if valid)
        """
        errors = []
        
        # CARLA
        if not 0 < self.carla_port < 65536:
            errors.append(f"Invalid CARLA port: {self.carla_port}")
        if self.carla_timeout <= 0:
            errors.append(f"Invalid CARLA timeout: {self.carla_timeout}")
        
        # Camera
        if self.camera_width <= 0 or self.camera_height <= 0:
            errors.append(f"Invalid camera dimensions: {self.camera_width}x{self.camera_height}")
        if not 0 < self.camera_fov <= 180:
            errors.append(f"Invalid camera FOV: {self.camera_fov}")
        
        # Perception
        if self.perception_feature_dim <= 0:
            errors.append(f"Invalid feature dimension: {self.perception_feature_dim}")
        
        # Temporal
        if self.temporal_sequence_length <= 0:
            errors.append(f"Invalid sequence length: {self.temporal_sequence_length}")
        if not 0 <= self.temporal_dropout < 1:
            errors.append(f"Invalid dropout: {self.temporal_dropout}")
        
        # MPC
        if self.mpc_horizon <= 0:
            errors.append(f"Invalid MPC horizon: {self.mpc_horizon}")
        if self.mpc_dt <= 0:
            errors.append(f"Invalid MPC dt: {self.mpc_dt}")
        
        return errors


class ConfigManager:
    """Configuration manager with validation."""
    
    def __init__(self, config_path: str = "config.yaml", environment: str = "development"):
        """
        Initialize configuration manager.
        
        Args:
            config_path: Path to main config file
            environment: Environment name (development, production, testing)
        """
        self.config_path = Path(config_path)
        self.environment = environment
        self._config: Dict[str, Any] = {}
        self._schema: Optional[ConfigSchema] = None
        
        self._load_config()
        self._validate_config()
    
    def _load_config(self) -> None:
        """Load configuration from file."""
        if not self.config_path.exists():
            raise ConfigurationError(f"Config file not found: {self.config_path}")
        
        with open(self.config_path, 'r') as f:
            self._config = yaml.safe_load(f)
        
        # Load environment-specific overrides
        env_config_path = Path("configs") / self.environment / "config.yaml"
        if env_config_path.exists():
            with open(env_config_path, 'r') as f:
                env_config = yaml.safe_load(f)
                self._merge_config(self._config, env_config)
    
    def _merge_config(self, base: Dict[str, Any], override: Dict[str, Any]) -> None:
        """Recursively merge configuration dictionaries."""
        for key, value in override.items():
            if key in base and isinstance(base[key], dict) and isinstance(value, dict):
                self._merge_config(base[key], value)
            else:
                base[key] = value
    
    def _validate_config(self) -> None:
        """Validate configuration."""
        errors = []
        
        # Check required sections
        required_sections = ['carla', 'perception', 'temporal', 'mpc']
        for section in required_sections:
            if section not in self._config:
                errors.append(f"Missing required section: {section}")
        
        if errors:
            raise ConfigurationError(f"Configuration validation failed:\n" + "\n".join(errors))
    
    def get(self, key: str, default: Any = None) -> Any:
        """
        Get configuration value using dot notation.
        
        Args:
            key: Configuration key (e.g., 'carla.host')
            default: Default value if not found
        
        Returns:
            Configuration value
        """
        keys = key.split('.')
        value = self._config
        
        for k in keys:
            if isinstance(value, dict) and k in value:
                value = value[k]
            else:
                return default
        
        return value
    
    def get_section(self, section: str) -> Dict[str, Any]:
        """
        Get configuration section.
        
        Args:
            section: Section name
        
        Returns:
            Section dictionary
        """
        return self._config.get(section, {})
    
    def validate(self) -> bool:
        """
        Validate configuration against schema.
        
        Returns:
            True if valid
        """
        # Convert config to schema and validate
        try:
            schema = ConfigSchema(
                carla_host=self.get('carla.host', 'localhost'),
                carla_port=self.get('carla.port', 2000),
                # ... add all fields
            )
            errors = schema.validate()
            if errors:
                raise ConfigurationError(f"Schema validation failed:\n" + "\n".join(errors))
            return True
        except Exception as e:
            raise ConfigurationError(f"Validation error: {e}")

