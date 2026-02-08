# API Documentation

This directory contains API documentation for all modules in the project.

## Module Documentation

### Core Modules

- [CARLA Environment](carla_env.md) - CARLA client and sensor management
- [Perception](perception.md) - ResNet encoder and lane detection
- [Temporal](temporal.md) - LSTM predictor and sequence buffer
- [Control](control.md) - MPC controller and path planning
- [Visualization](visualization.md) - Pygame display and visualization

### Training Modules

- [Data Collection](training_data_collection.md) - Data collection scripts
- [Data Preprocessing](training_preprocessing.md) - Data cleaning and normalization
- [Model Training](training_models.md) - ResNet and LSTM training

### Utilities

- [Device Utils](utils_device.md) - GPU/CPU device management
- [Status Logger](utils_status.md) - System status logging
- [Error Handler](utils_error.md) - Error handling utilities
- [Logging Config](utils_logging.md) - Logging configuration

## Generating Documentation

To generate API documentation:

```bash
# Install sphinx
pip install sphinx sphinx-rtd-theme

# Generate docs
cd docs/api
sphinx-build -b html . _build
```

## Documentation Standards

All modules should include:
- Module-level docstring
- Class docstrings with Args/Returns
- Function docstrings with Args/Returns/Raises
- Type hints for all functions
- Example usage where applicable

