"""Build the flat vector observation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

import numpy as np

from .actor_filter import ActorSnapshot
from .schema import (
    N_EGO_FEATURES,
    N_TRAFFIC_FEATURES,
    N_VEHICLE_FEATURES,
    N_WAYPOINT_FEATURES,
    StateSchema,
)


@dataclass
class EgoState:
    speed: float
    accel: float
    yaw: float
    yaw_rate: float
    steering: float
    lat_offset: float
    heading_err: float


@dataclass
class WaypointPoint:
    rel_x: float
    rel_y: float
    curvature: float


@dataclass
class TrafficInfo:
    # CARLA TrafficLightState: Red=0, Yellow=1, Green=2, Off/Unknown -> none
    state: str = "none"  # "red" | "yellow" | "green" | "none"
    distance_to_stop: float = -1.0  # meters; -1 if no TL ahead


_TL_INDEX = {"red": 0, "yellow": 1, "green": 2, "none": 3}


def build_vector(
    schema: StateSchema,
    ego: EgoState,
    actors: List[Optional[ActorSnapshot]],
    waypoints: List[Optional[WaypointPoint]],
    traffic: TrafficInfo,
    lane_offset_lookup=lambda a: 0.0,
) -> np.ndarray:
    """Assemble a fixed-size float32 vector following `StateSchema`."""
    dim = schema.vector_dim()
    out = np.zeros(dim, dtype=np.float32)
    idx = 0

    # Ego block (8 features; sin/cos yaw to keep continuity across +/-pi)
    out[idx + 0] = ego.speed / max(schema.max_speed, 1e-6)
    out[idx + 1] = ego.accel / max(schema.max_accel, 1e-6)
    out[idx + 2] = np.sin(ego.yaw)
    out[idx + 3] = np.cos(ego.yaw)
    out[idx + 4] = ego.yaw_rate
    out[idx + 5] = np.clip(ego.steering, -1.0, 1.0)
    out[idx + 6] = ego.lat_offset
    out[idx + 7] = ego.heading_err
    idx += N_EGO_FEATURES

    # Vehicles block
    for a in actors[: schema.n_vehicles]:
        if a is None:
            idx += N_VEHICLE_FEATURES
            continue
        out[idx + 0] = a.rel_x / schema.max_range
        out[idx + 1] = a.rel_y / schema.max_range
        out[idx + 2] = a.rel_vx / max(schema.max_speed, 1e-6)
        out[idx + 3] = a.rel_vy / max(schema.max_speed, 1e-6)
        out[idx + 4] = a.distance / schema.max_range
        out[idx + 5] = float(lane_offset_lookup(a))
        out[idx + 6] = 1.0  # valid mask
        idx += N_VEHICLE_FEATURES
    # Pad remaining vehicle slots if any (already zero-initialized)
    remaining = schema.n_vehicles - len(actors[: schema.n_vehicles])
    idx += max(0, remaining) * N_VEHICLE_FEATURES

    # Waypoints block
    for w in waypoints[: schema.k_waypoints]:
        if w is None:
            idx += N_WAYPOINT_FEATURES
            continue
        out[idx + 0] = w.rel_x / schema.max_range
        out[idx + 1] = w.rel_y / schema.max_range
        out[idx + 2] = np.clip(w.curvature, -1.0, 1.0)
        idx += N_WAYPOINT_FEATURES
    remaining = schema.k_waypoints - len(waypoints[: schema.k_waypoints])
    idx += max(0, remaining) * N_WAYPOINT_FEATURES

    # Traffic block
    tl_onehot = [0.0, 0.0, 0.0, 0.0]
    tl_onehot[_TL_INDEX.get(traffic.state, 3)] = 1.0
    out[idx + 0] = tl_onehot[0]
    out[idx + 1] = tl_onehot[1]
    out[idx + 2] = tl_onehot[2]
    out[idx + 3] = tl_onehot[3]
    d = traffic.distance_to_stop
    out[idx + 4] = -1.0 if d < 0 else np.clip(d / schema.max_range, 0.0, 1.0)
    idx += N_TRAFFIC_FEATURES

    assert idx == dim, f"Vector build size mismatch: {idx} vs {dim}"
    return out
