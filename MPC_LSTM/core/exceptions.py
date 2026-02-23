"""
Custom exception hierarchy for the project.
"""


class ProjectError(Exception):
    """Base exception for all project errors."""
    pass


class CARLAConnectionError(ProjectError):
    """Error connecting to or communicating with CARLA."""
    pass


class ModelLoadError(ProjectError):
    """Error loading model files."""
    pass


class DataValidationError(ProjectError):
    """Error validating input data."""
    pass


class TrainingError(ProjectError):
    """Error during training."""
    pass


class ControlError(ProjectError):
    """Error in control computation."""
    pass


class ConfigurationError(ProjectError):
    """Error in configuration."""
    pass

