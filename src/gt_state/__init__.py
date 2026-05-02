"""Ground-truth state builder for CARLA RL.

Builds RL observations (vector + BEV) from CARLA ground-truth data
WITHOUT using camera/lidar/radar detection. All state comes from the
CARLA server directly via the `WorldSource` abstraction, so the same
schema can later be fed by a perception model.
"""

from .schema import StateSchema, DEFAULT_SCHEMA
from .state_builder import StateBuilder
from .noise import NoiseConfig, StateNoise

__all__ = [
    "StateSchema",
    "DEFAULT_SCHEMA",
    "StateBuilder",
    "NoiseConfig",
    "StateNoise",
]
