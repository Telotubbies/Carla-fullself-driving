"""Ego-centric coordinate transforms.

Convention:
    ego frame: +x forward, +y left (right-hand, z up)
    yaw is in radians (CARLA yaw is degrees, convert at ingest time)
"""

from __future__ import annotations

import math
from typing import Tuple

import numpy as np


def wrap_angle(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def world_to_ego(
    px: float, py: float,
    ego_x: float, ego_y: float, ego_yaw: float,
) -> Tuple[float, float]:
    """Rotate+translate a world point into ego frame (+x fwd, +y left)."""
    dx = px - ego_x
    dy = py - ego_y
    c = math.cos(-ego_yaw)
    s = math.sin(-ego_yaw)
    fx = c * dx - s * dy
    fy = s * dx + c * dy
    return fx, fy


def world_vec_to_ego(
    vx: float, vy: float, ego_yaw: float,
) -> Tuple[float, float]:
    """Rotate a world velocity/acceleration into ego frame."""
    c = math.cos(-ego_yaw)
    s = math.sin(-ego_yaw)
    return c * vx - s * vy, s * vx + c * vy


def in_fov(fx: float, fy: float, half_fov_rad: float) -> bool:
    """Return True if (fx, fy) in ego frame lies within the forward FOV cone."""
    if fx <= 0.0:
        return False
    return abs(math.atan2(fy, fx)) <= half_fov_rad


def batch_world_to_ego(
    points: np.ndarray,  # (N, 2) world (x, y)
    ego_x: float, ego_y: float, ego_yaw: float,
) -> np.ndarray:
    """Vectorized world->ego transform. Returns (N, 2)."""
    if points.size == 0:
        return points.reshape(-1, 2).astype(np.float32)
    dx = points[:, 0] - ego_x
    dy = points[:, 1] - ego_y
    c = math.cos(-ego_yaw)
    s = math.sin(-ego_yaw)
    fx = c * dx - s * dy
    fy = s * dx + c * dy
    return np.stack([fx, fy], axis=1).astype(np.float32)
