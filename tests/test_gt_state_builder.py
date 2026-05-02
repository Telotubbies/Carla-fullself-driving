"""Unit tests for the ground-truth state builder.

Runs WITHOUT CARLA by using a fake WorldSource.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from src.gt_state import DEFAULT_SCHEMA, NoiseConfig, StateBuilder, StateSchema
from src.gt_state.actor_filter import ActorSnapshot, filter_and_rank
from src.gt_state.ego_centric import wrap_angle, world_to_ego
from src.gt_state.schema import (
    N_BEV_CHANNELS,
    N_EGO_FEATURES,
    N_TRAFFIC_FEATURES,
    N_VEHICLE_FEATURES,
    N_WAYPOINT_FEATURES,
)
from src.gt_state.vector_builder import EgoState, TrafficInfo, WaypointPoint


class FakeSource:
    """Minimal WorldSource for tests."""

    def __init__(
        self,
        ego: EgoState,
        actors: List[ActorSnapshot],
        waypoints: List[Optional[WaypointPoint]],
        traffic: TrafficInfo,
        extent: Tuple[float, float] = (4.7, 2.0),
    ):
        self._ego = ego
        self._actors = actors
        self._waypoints = waypoints
        self._traffic = traffic
        self._extent = extent

    def get_ego_state(self) -> EgoState: return self._ego
    def get_ego_extent(self): return self._extent
    def get_nearby_actors(self, max_range): return self._actors
    def get_waypoints_ahead(self, k, spacing): return self._waypoints[:k]
    def get_traffic_light_ahead(self, max_range): return self._traffic
    def get_road_polylines(self): return None
    def get_lane_polylines(self): return None


def _ego():
    return EgoState(speed=5.0, accel=0.1, yaw=0.0, yaw_rate=0.0,
                    steering=0.0, lat_offset=0.0, heading_err=0.0)


def test_world_to_ego_identity_yaw():
    fx, fy = world_to_ego(10.0, 5.0, 0.0, 0.0, 0.0)
    assert fx == pytest.approx(10.0)
    assert fy == pytest.approx(5.0)


def test_world_to_ego_rotated_90():
    # Ego facing north (yaw=pi/2); world point at (10, 0) should be at (0, -10) in ego frame
    fx, fy = world_to_ego(10.0, 0.0, 0.0, 0.0, math.pi / 2)
    assert fx == pytest.approx(0.0, abs=1e-6)
    assert fy == pytest.approx(-10.0, abs=1e-6)


def test_wrap_angle():
    # 3*pi -> -pi (both valid; our implementation returns -pi)
    assert abs(abs(wrap_angle(3 * math.pi)) - math.pi) < 1e-6
    assert abs(abs(wrap_angle(-3 * math.pi)) - math.pi) < 1e-6
    assert wrap_angle(0.0) == pytest.approx(0.0)
    assert wrap_angle(math.pi / 2) == pytest.approx(math.pi / 2)


def test_filter_keeps_front_drops_back_and_far():
    a_front = ActorSnapshot(1,  10.0, 0.0, 0, 0, (4, 2), 0.0)
    a_back  = ActorSnapshot(2, -10.0, 0.0, 0, 0, (4, 2), 0.0)
    a_far   = ActorSnapshot(3,  60.0, 0.0, 0, 0, (4, 2), 0.0)
    a_side  = ActorSnapshot(4,   5.0, 5.0, 0, 0, (4, 2), 0.0)  # 45 deg, in 120 fov
    out = filter_and_rank([a_front, a_back, a_far, a_side],
                          max_range=50.0, fov_deg=120.0, top_n=5)
    ids = [a.id for a in out]
    assert 1 in ids and 4 in ids
    assert 2 not in ids and 3 not in ids
    # Sorted by distance: a_front(10) < a_side(~7.07)
    assert out[0].id == 4  # 7.07 < 10
    assert out[1].id == 1


def test_builder_shapes_and_vector_dim():
    schema = StateSchema(n_vehicles=3, k_waypoints=4, bev_size=64)
    actors = [ActorSnapshot(1, 10.0, 0.0, 1.0, 0.0, (4, 2), 0.0)]
    waypoints = [WaypointPoint(2.0 * i, 0.0, 0.0) for i in range(4)]
    src = FakeSource(_ego(), actors, waypoints,
                     TrafficInfo(state="red", distance_to_stop=8.0))
    builder = StateBuilder(schema=schema)
    obs = builder.build(src)

    assert set(obs.keys()) == {"vector", "bev"}
    assert obs["vector"].shape == (schema.vector_dim(),)
    assert obs["vector"].dtype == np.float32
    assert obs["bev"].shape == (N_BEV_CHANNELS, 64, 64)
    assert obs["bev"].dtype == np.uint8

    # expected dim consistency
    expected = (
        N_EGO_FEATURES
        + schema.n_vehicles * N_VEHICLE_FEATURES
        + schema.k_waypoints * N_WAYPOINT_FEATURES
        + N_TRAFFIC_FEATURES
    )
    assert schema.vector_dim() == expected


def test_valid_mask_padding():
    schema = StateSchema(n_vehicles=5, k_waypoints=2, bev_size=32)
    actors = [ActorSnapshot(1, 10.0, 0.0, 0, 0, (4, 2), 0.0)]
    waypoints = [WaypointPoint(1.0, 0.0, 0.0), WaypointPoint(2.0, 0.0, 0.0)]
    src = FakeSource(_ego(), actors, waypoints, TrafficInfo())
    obs = StateBuilder(schema=schema).build(src)
    v = obs["vector"]

    # vehicle block begins at index = N_EGO_FEATURES
    base = N_EGO_FEATURES
    # first vehicle valid_mask
    assert v[base + 6] == pytest.approx(1.0)
    # second vehicle slot should be all zeros including mask
    assert v[base + N_VEHICLE_FEATURES + 6] == pytest.approx(0.0)


def test_traffic_onehot_encoding():
    schema = StateSchema(n_vehicles=1, k_waypoints=1, bev_size=32)
    src = FakeSource(_ego(), [], [None],
                     TrafficInfo(state="green", distance_to_stop=10.0))
    v = StateBuilder(schema=schema).build(src)["vector"]
    # Traffic block is the last 5 entries
    tl = v[-N_TRAFFIC_FEATURES:]
    # one-hot: [red, yellow, green, none, dist_norm]
    assert tl[0] == 0.0
    assert tl[1] == 0.0
    assert tl[2] == 1.0
    assert tl[3] == 0.0
    assert 0.0 <= tl[4] <= 1.0


def test_noise_dropout_can_zero_actors():
    schema = StateSchema(n_vehicles=4, k_waypoints=1, bev_size=32)
    actors = [
        ActorSnapshot(i, 5.0 + i, 0.0, 0, 0, (4, 2), 0.0) for i in range(4)
    ]
    src = FakeSource(_ego(), actors, [None], TrafficInfo())
    noise = NoiseConfig(dropout_p=1.0, seed=0)  # force drop everything
    v = StateBuilder(schema=schema, noise=noise).build(src)["vector"]
    base = N_EGO_FEATURES
    for i in range(4):
        assert v[base + i * N_VEHICLE_FEATURES + 6] == pytest.approx(0.0)
