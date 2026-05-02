"""Dual-mode bridge between the RL state builder and the world.

`WorldSource` is the abstraction. Two implementations:
    - DirectCarlaSource: CARLA PythonAPI (fast, used for training)
    - Ros2WorldSource  : subscribes to CARLA-ROS bridge topics (deploy/fine-tune)
"""

from .base import WorldSource

__all__ = ["WorldSource"]
