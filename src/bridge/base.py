"""WorldSource protocol: everything the StateBuilder needs from the world."""

from __future__ import annotations

from typing import List, Optional, Protocol, Sequence, Tuple

import numpy as np

from src.gt_state.actor_filter import ActorSnapshot
from src.gt_state.vector_builder import EgoState, TrafficInfo, WaypointPoint


class WorldSource(Protocol):
    def get_ego_state(self) -> EgoState: ...

    def get_ego_extent(self) -> Tuple[float, float]:
        """(length, width) in meters."""
        ...

    def get_nearby_actors(self, max_range: float) -> List[ActorSnapshot]:
        """All vehicles/walkers within `max_range`, already in ego frame."""
        ...

    def get_waypoints_ahead(
        self, k: int, spacing: float,
    ) -> List[Optional[WaypointPoint]]: ...

    def get_traffic_light_ahead(self, max_range: float) -> TrafficInfo: ...

    # BEV rasterization inputs; return None to let BEV builder use fallback
    def get_road_polylines(self) -> Optional[Sequence[np.ndarray]]: ...
    def get_lane_polylines(self) -> Optional[Sequence[np.ndarray]]: ...
