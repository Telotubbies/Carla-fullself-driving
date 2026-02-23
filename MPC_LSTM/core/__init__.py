"""
Core module for autonomous driving system.

This module provides abstract interfaces and base classes for all major components.
"""

from .interfaces import (
    IPerceptionModule,
    ITemporalModule,
    IControlModule,
    IVisualizationModule,
    IConfigManager,
    ILogger
)

from .exceptions import (
    ProjectError,
    CARLAConnectionError,
    ModelLoadError,
    DataValidationError,
    TrainingError,
    ControlError
)

from .config import ConfigManager, ConfigSchema

__all__ = [
    'IPerceptionModule',
    'ITemporalModule',
    'IControlModule',
    'IVisualizationModule',
    'IConfigManager',
    'ILogger',
    'ProjectError',
    'CARLAConnectionError',
    'ModelLoadError',
    'DataValidationError',
    'TrainingError',
    'ControlError',
    'ConfigManager',
    'ConfigSchema',
]

