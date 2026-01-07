"""
Configuration Loader

Handles loading and validation of configuration files.
Follows Single Responsibility Principle - only handles configuration management.
"""

import yaml
import logging
from pathlib import Path
from typing import Dict, Any, Optional


class ConfigLoader:
    """Loads and validates configuration from YAML files."""
    
    @staticmethod
    def load(config_path: str) -> Dict[str, Any]:
        """
        Load configuration from YAML file.
        
        Args:
            config_path: Path to configuration YAML file
            
        Returns:
            Configuration dictionary
            
        Raises:
            FileNotFoundError: If config file doesn't exist
            yaml.YAMLError: If config file is invalid YAML
        """
        config_file = Path(config_path)
        
        if not config_file.exists():
            raise FileNotFoundError(f"Configuration file not found: {config_path}")
        
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
            
            if config is None:
                raise ValueError(f"Configuration file is empty: {config_path}")
            
            logging.info(f"Loaded configuration from: {config_path}")
            return config
            
        except yaml.YAMLError as e:
            logging.error(f"Error parsing YAML configuration: {e}")
            raise
    
    @staticmethod
    def get_section(config: Dict[str, Any], section: str, default: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Get a configuration section with optional default.
        
        Args:
            config: Configuration dictionary
            section: Section name to retrieve
            default: Default value if section doesn't exist
            
        Returns:
            Section dictionary or default
        """
        return config.get(section, default or {})
    
    @staticmethod
    def get_value(config: Dict[str, Any], key_path: str, default: Any = None) -> Any:
        """
        Get a nested configuration value using dot notation.
        
        Args:
            config: Configuration dictionary
            key_path: Dot-separated key path (e.g., 'environment.carla_port')
            default: Default value if key doesn't exist
            
        Returns:
            Configuration value or default
        """
        keys = key_path.split('.')
        value = config
        
        for key in keys:
            if isinstance(value, dict) and key in value:
                value = value[key]
            else:
                return default
        
        return value

