"""Rasterize ego-centric state into a multi-channel BEV image.

Channels: road, lane, ego, vehicles, traffic  (uint8, 0 or 255)

Ego is placed at (bev_size * bev_forward_bias, bev_size/2). Default bias=0.5
means ego is at the image center; increase to push ego toward the bottom
(shows more road ahead).
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

from .actor_filter import ActorSnapshot
from .schema import N_BEV_CHANNELS, StateSchema
from .vector_builder import TrafficInfo, WaypointPoint


def _ego_to_pixel(
    fx: float, fy: float, schema: StateSchema,
) -> Tuple[int, int]:
    """Ego frame (x fwd, y left) -> image (row, col)."""
    size = schema.bev_size
    mpc = schema.bev_meters_per_cell
    cx = int(size * 0.5)
    cy = int(size * schema.bev_forward_bias)
    # +x (forward) -> up (row decreases)
    row = cy - int(round(fx / mpc))
    # +y (left) -> col decreases
    col = cx - int(round(fy / mpc))
    return row, col


def _in_bounds(r: int, c: int, size: int) -> bool:
    return 0 <= r < size and 0 <= c < size


def build_bev(
    schema: StateSchema,
    actors: List[Optional[ActorSnapshot]],
    waypoints: Sequence[Optional[WaypointPoint]],
    traffic: TrafficInfo,
    ego_extent: Tuple[float, float] = (4.7, 2.0),  # Tesla Model 3 approx
    road_polylines_ego: Optional[Sequence[np.ndarray]] = None,
    lane_polylines_ego: Optional[Sequence[np.ndarray]] = None,
) -> np.ndarray:
    """Returns uint8 array of shape (C, H, W) with C = N_BEV_CHANNELS."""
    size = schema.bev_size
    bev = np.zeros((N_BEV_CHANNELS, size, size), dtype=np.uint8)
    road_ch, lane_ch, ego_ch, veh_ch, tl_ch = 0, 1, 2, 3, 4

    mpc = schema.bev_meters_per_cell

    # --- Road channel: if polylines provided, fill drivable lane corridor.
    # Otherwise, draw a coarse "forward lane corridor" as fallback so the
    # policy still gets a road prior at train time.
    if road_polylines_ego:
        for poly in road_polylines_ego:
            if poly is None or len(poly) < 2:
                continue
            pts = np.array(
                [_ego_to_pixel(float(p[0]), float(p[1]), schema) for p in poly],
                dtype=np.int32,
            )[:, [1, 0]]  # cv2 expects (x=col, y=row)
            cv2.fillPoly(bev[road_ch], [pts], 255)
    else:
        # Fallback: assume a 6m-wide straight lane corridor along +x
        half_w_cells = int(round(3.0 / mpc))
        cx = int(size * 0.5)
        cy = int(size * schema.bev_forward_bias)
        bev[road_ch, :cy, cx - half_w_cells: cx + half_w_cells] = 255

    # --- Lane channel
    if lane_polylines_ego:
        for poly in lane_polylines_ego:
            if poly is None or len(poly) < 2:
                continue
            pts = np.array(
                [_ego_to_pixel(float(p[0]), float(p[1]), schema) for p in poly],
                dtype=np.int32,
            )[:, [1, 0]]
            cv2.polylines(bev[lane_ch], [pts], False, 255, 1)

    # --- Waypoint polyline on lane channel (always)
    wp_pts = []
    for w in waypoints:
        if w is None:
            continue
        r, c = _ego_to_pixel(w.rel_x, w.rel_y, schema)
        if _in_bounds(r, c, size):
            wp_pts.append((c, r))
    if len(wp_pts) >= 2:
        cv2.polylines(
            bev[lane_ch], [np.array(wp_pts, dtype=np.int32)], False, 255, 1,
        )

    # --- Ego channel (rotated rectangle at image center, aligned to +x)
    _draw_rect(
        bev[ego_ch],
        (0.0, 0.0),
        yaw_rel=0.0,
        length=ego_extent[0],
        width=ego_extent[1],
        schema=schema,
    )

    # --- Vehicle channel
    for a in actors:
        if a is None:
            continue
        length, width = a.extent if a.extent else (4.5, 1.8)
        _draw_rect(
            bev[veh_ch],
            (a.rel_x, a.rel_y),
            yaw_rel=a.yaw_rel,
            length=length,
            width=width,
            schema=schema,
        )

    # --- Traffic channel: draw a colored disc at the stop line
    if traffic.state in ("red", "yellow", "green") and traffic.distance_to_stop >= 0:
        r, c = _ego_to_pixel(traffic.distance_to_stop, 0.0, schema)
        if _in_bounds(r, c, size):
            # Encode state via pixel value within channel
            val = {"red": 255, "yellow": 170, "green": 85}[traffic.state]
            cv2.circle(bev[tl_ch], (c, r), radius=3, color=val, thickness=-1)

    return bev


def _draw_rect(
    canvas: np.ndarray,
    center_ego: Tuple[float, float],
    yaw_rel: float,
    length: float,
    width: float,
    schema: StateSchema,
) -> None:
    size = schema.bev_size
    mpc = schema.bev_meters_per_cell
    # Build corners in local actor frame (length along +x, width along +y)
    hl = length * 0.5
    hw = width * 0.5
    corners_local = np.array(
        [[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]],
        dtype=np.float32,
    )
    c = math.cos(yaw_rel)
    s = math.sin(yaw_rel)
    rot = np.array([[c, -s], [s, c]], dtype=np.float32)
    corners_ego = corners_local @ rot.T + np.array(
        [center_ego[0], center_ego[1]], dtype=np.float32,
    )
    pts = []
    for cx_e, cy_e in corners_ego:
        r, col = _ego_to_pixel(float(cx_e), float(cy_e), schema)
        pts.append([col, r])
    pts_arr = np.array([pts], dtype=np.int32)
    cv2.fillPoly(canvas, pts_arr, 255)
