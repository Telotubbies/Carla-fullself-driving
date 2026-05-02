"""State schema constants.

Single source of truth so perception models trained in the future can
produce tensors with identical shapes.
"""

from dataclasses import dataclass, field
from typing import List, Tuple


VEHICLE_FEATURES: List[str] = [
    "rel_x",       # ego-centric forward (m)
    "rel_y",       # ego-centric left (m)
    "rel_vx",      # ego-centric forward velocity (m/s)
    "rel_vy",      # ego-centric lateral velocity (m/s)
    "distance",    # euclidean distance (m)
    "lane_offset", # signed lateral offset from ego lane center (m)
    "valid_mask",  # 1.0 if slot filled, else 0.0
]
N_VEHICLE_FEATURES = len(VEHICLE_FEATURES)

EGO_FEATURES: List[str] = [
    "speed",        # m/s
    "accel",        # m/s^2 magnitude
    "sin_yaw",      # sin(yaw), continuous across wrap
    "cos_yaw",      # cos(yaw), continuous across wrap
    "yaw_rate",     # rad/s
    "steering",     # [-1, 1]
    "lat_offset",   # m (signed)
    "heading_err",  # rad (wrapped)
]
N_EGO_FEATURES = len(EGO_FEATURES)

WAYPOINT_FEATURES: List[str] = [
    "rel_x",     # ego-centric forward (m)
    "rel_y",     # ego-centric left (m)
    "curvature", # 1/m
]
N_WAYPOINT_FEATURES = len(WAYPOINT_FEATURES)

# Traffic block: [tl_red, tl_yellow, tl_green, tl_none, dist_to_stop_norm]
N_TRAFFIC_FEATURES = 5

BEV_CHANNELS: List[str] = ["road", "lane", "ego", "vehicles", "traffic"]
N_BEV_CHANNELS = len(BEV_CHANNELS)


@dataclass
class StateSchema:
    # Perception window
    max_range: float = 50.0          # meters
    fov_deg: float = 120.0           # total cone, centered on +x (forward)
    n_vehicles: int = 10
    k_waypoints: int = 10
    waypoint_spacing: float = 2.0    # meters between sampled waypoints

    # BEV
    bev_size: int = 128              # HxW pixels
    bev_meters_per_cell: float = 0.5 # so BEV spans bev_size * mpc meters
    bev_forward_bias: float = 0.5    # 0.5 = ego at image center; >0.5 shifts ego toward bottom (more forward view)

    # Normalization constants
    max_speed: float = 30.0          # m/s (~108 km/h)
    max_accel: float = 10.0          # m/s^2

    def vector_dim(self) -> int:
        return (
            N_EGO_FEATURES
            + self.n_vehicles * N_VEHICLE_FEATURES
            + self.k_waypoints * N_WAYPOINT_FEATURES
            + N_TRAFFIC_FEATURES
        )

    def bev_shape(self) -> Tuple[int, int, int]:
        return (N_BEV_CHANNELS, self.bev_size, self.bev_size)

    def bev_extent_m(self) -> float:
        return self.bev_size * self.bev_meters_per_cell


DEFAULT_SCHEMA = StateSchema()
