"""StateBuilder facade: WorldSource -> Dict{vector, bev}."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import numpy as np

from .actor_filter import ActorSnapshot, filter_and_rank, pad_to_n
from .bev_builder import build_bev
from .noise import NoiseConfig, StateNoise
from .schema import DEFAULT_SCHEMA, StateSchema
from .vector_builder import EgoState, TrafficInfo, WaypointPoint, build_vector


class StateBuilder:
    """Builds ground-truth RL state from a `WorldSource`.

    The builder never touches cameras/lidar; all data comes through
    `WorldSource` which exposes CARLA ground truth (or a ROS2 equivalent).
    """

    def __init__(
        self,
        schema: StateSchema = DEFAULT_SCHEMA,
        noise: Optional[NoiseConfig] = None,
    ):
        self.schema = schema
        self.noise = StateNoise(noise or NoiseConfig())

    def reset(self) -> None:
        self.noise.reset()

    def build(self, source: "WorldSource") -> Dict[str, np.ndarray]:
        """Return observation dict {'vector': (D,), 'bev': (C, H, W)}."""
        ego = source.get_ego_state()
        actors = source.get_nearby_actors(max_range=self.schema.max_range)
        waypoints = source.get_waypoints_ahead(
            k=self.schema.k_waypoints,
            spacing=self.schema.waypoint_spacing,
        )
        traffic = source.get_traffic_light_ahead(max_range=self.schema.max_range)

        # Filter + rank
        ranked = filter_and_rank(
            actors,
            max_range=self.schema.max_range,
            fov_deg=self.schema.fov_deg,
            top_n=self.schema.n_vehicles,
        )
        padded = pad_to_n(ranked, self.schema.n_vehicles)

        # Apply noise / latency / dropout
        noised = self.noise.apply_actors(padded)

        # Noisy ego scalars (speed only; pose remains ground truth origin)
        ego_noisy = EgoState(
            speed=self.noise.apply_ego_scalar(ego.speed, self.noise.cfg.ego_vel_sigma),
            accel=ego.accel,
            yaw=ego.yaw,
            yaw_rate=ego.yaw_rate,
            steering=ego.steering,
            lat_offset=self.noise.apply_ego_scalar(
                ego.lat_offset, self.noise.cfg.ego_pos_sigma,
            ),
            heading_err=ego.heading_err,
        )

        vector = build_vector(
            schema=self.schema,
            ego=ego_noisy,
            actors=noised,
            waypoints=waypoints,
            traffic=traffic,
        )

        bev = build_bev(
            schema=self.schema,
            actors=noised,
            waypoints=waypoints,
            traffic=traffic,
            ego_extent=source.get_ego_extent(),
            road_polylines_ego=source.get_road_polylines(),
            lane_polylines_ego=source.get_lane_polylines(),
        )

        return {"vector": vector.astype(np.float32), "bev": bev}
