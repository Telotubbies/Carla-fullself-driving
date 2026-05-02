"""Ros2WorldSource: subscribe to CARLA-ROS bridge topics.

Intended for DEPLOY / fine-tune paths. Training uses `DirectCarlaSource`.
This implementation is intentionally light; you wire up the concrete
subscribers in your ROS2 node and feed snapshots via `update_*` methods.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from threading import Lock
from typing import List, Optional, Sequence, Tuple

import numpy as np

from src.gt_state.actor_filter import ActorSnapshot
from src.gt_state.vector_builder import EgoState, TrafficInfo, WaypointPoint


@dataclass
class _Ros2Cache:
    ego: Optional[EgoState] = None
    ego_extent: Tuple[float, float] = (4.7, 2.0)
    actors: List[ActorSnapshot] = field(default_factory=list)
    waypoints: List[Optional[WaypointPoint]] = field(default_factory=list)
    traffic: TrafficInfo = field(default_factory=lambda: TrafficInfo())
    road_polylines: Optional[Sequence[np.ndarray]] = None
    lane_polylines: Optional[Sequence[np.ndarray]] = None


class Ros2WorldSource:
    """Thread-safe cache fed by your ROS2 node's subscribers."""

    def __init__(self):
        self._lock = Lock()
        self._cache = _Ros2Cache()

    # ----- setters (called from ROS2 callbacks) -----
    def update_ego(self, ego: EgoState, extent: Tuple[float, float]) -> None:
        with self._lock:
            self._cache.ego = ego
            self._cache.ego_extent = extent

    def update_actors(self, actors: List[ActorSnapshot]) -> None:
        with self._lock:
            self._cache.actors = list(actors)

    def update_waypoints(self, waypoints: List[Optional[WaypointPoint]]) -> None:
        with self._lock:
            self._cache.waypoints = list(waypoints)

    def update_traffic(self, traffic: TrafficInfo) -> None:
        with self._lock:
            self._cache.traffic = traffic

    def update_polylines(
        self,
        road: Optional[Sequence[np.ndarray]] = None,
        lane: Optional[Sequence[np.ndarray]] = None,
    ) -> None:
        with self._lock:
            if road is not None:
                self._cache.road_polylines = road
            if lane is not None:
                self._cache.lane_polylines = lane

    # ----- WorldSource API -----
    def get_ego_state(self) -> EgoState:
        with self._lock:
            if self._cache.ego is None:
                return EgoState(0, 0, 0, 0, 0, 0, 0)
            return self._cache.ego

    def get_ego_extent(self) -> Tuple[float, float]:
        with self._lock:
            return self._cache.ego_extent

    def get_nearby_actors(self, max_range: float) -> List[ActorSnapshot]:
        with self._lock:
            return list(self._cache.actors)

    def get_waypoints_ahead(
        self, k: int, spacing: float,
    ) -> List[Optional[WaypointPoint]]:
        with self._lock:
            wps = list(self._cache.waypoints)
        # pad/truncate to k
        if len(wps) < k:
            wps = wps + [None] * (k - len(wps))
        return wps[:k]

    def get_traffic_light_ahead(self, max_range: float) -> TrafficInfo:
        with self._lock:
            return self._cache.traffic

    def get_road_polylines(self) -> Optional[Sequence[np.ndarray]]:
        with self._lock:
            return self._cache.road_polylines

    def get_lane_polylines(self) -> Optional[Sequence[np.ndarray]]:
        with self._lock:
            return self._cache.lane_polylines
