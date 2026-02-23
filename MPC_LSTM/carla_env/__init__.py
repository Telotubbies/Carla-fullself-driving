"""CARLA environment module for autonomous driving."""

from .carla_client import CarlaClient
from .sensors import CameraSensor

__all__ = ['CarlaClient', 'CameraSensor']

