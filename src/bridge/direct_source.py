"""DirectCarlaSource: WorldSource backed by the CARLA PythonAPI.

Only ground-truth calls are used (no sensors).
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

try:
    import carla  # type: ignore
except Exception:  # pragma: no cover
    carla = None  # Allow import in tests without CARLA installed.

from src.gt_state.actor_filter import ActorSnapshot
from src.gt_state.ego_centric import (
    batch_world_to_ego,
    wrap_angle,
    world_to_ego,
    world_vec_to_ego,
)
from src.gt_state.vector_builder import EgoState, TrafficInfo, WaypointPoint


class DirectCarlaSource:
    """Ground-truth adapter over carla.World + carla.Vehicle."""

    def __init__(self, world: "carla.World", ego: "carla.Vehicle"):
        self.world = world
        self.ego = ego
        self.map = world.get_map()
        self._prev_yaw: Optional[float] = None
        self._prev_time: Optional[float] = None

    # ----- internals -----
    def _ego_pose(self) -> Tuple[float, float, float]:
        t = self.ego.get_transform()
        return t.location.x, t.location.y, math.radians(t.rotation.yaw)

    # ----- WorldSource API -----
    def get_ego_state(self) -> EgoState:
        tf = self.ego.get_transform()
        vel = self.ego.get_velocity()
        acc = self.ego.get_acceleration()
        ctl = self.ego.get_control()

        yaw = math.radians(tf.rotation.yaw)
        speed = math.hypot(vel.x, vel.y)
        accel = math.hypot(acc.x, acc.y)

        now = self.world.get_snapshot().timestamp.elapsed_seconds
        yaw_rate = 0.0
        if self._prev_yaw is not None and self._prev_time is not None:
            dt = max(1e-3, now - self._prev_time)
            yaw_rate = wrap_angle(yaw - self._prev_yaw) / dt
        self._prev_yaw = yaw
        self._prev_time = now

        wp = self.map.get_waypoint(tf.location, project_to_road=True)
        lat_offset = 0.0
        heading_err = 0.0
        if wp is not None:
            wx, wy = wp.transform.location.x, wp.transform.location.y
            wyaw = math.radians(wp.transform.rotation.yaw)
            # Lateral offset is the ego.y in the waypoint's frame
            dx = tf.location.x - wx
            dy = tf.location.y - wy
            lat_offset = -dx * math.sin(wyaw) + dy * math.cos(wyaw)
            heading_err = wrap_angle(yaw - wyaw)

        return EgoState(
            speed=speed,
            accel=accel,
            yaw=yaw,
            yaw_rate=yaw_rate,
            steering=float(ctl.steer),
            lat_offset=float(lat_offset),
            heading_err=float(heading_err),
        )

    def get_ego_extent(self) -> Tuple[float, float]:
        bb = self.ego.bounding_box
        return 2.0 * bb.extent.x, 2.0 * bb.extent.y

    def get_nearby_actors(self, max_range: float) -> List[ActorSnapshot]:
        ex, ey, eyaw = self._ego_pose()
        out: List[ActorSnapshot] = []
        actors = self.world.get_actors().filter("vehicle.*")
        for a in actors:
            if a.id == self.ego.id:
                continue
            t = a.get_transform()
            dx = t.location.x - ex
            dy = t.location.y - ey
            if dx * dx + dy * dy > max_range * max_range:
                continue
            fx, fy = world_to_ego(t.location.x, t.location.y, ex, ey, eyaw)
            v = a.get_velocity()
            vfx, vfy = world_vec_to_ego(v.x, v.y, eyaw)
            # subtract ego velocity to get relative velocity
            ev = self.ego.get_velocity()
            evfx, evfy = world_vec_to_ego(ev.x, ev.y, eyaw)
            rel_vx = vfx - evfx
            rel_vy = vfy - evfy
            yaw_rel = wrap_angle(math.radians(t.rotation.yaw) - eyaw)
            bb = a.bounding_box
            extent = (2.0 * bb.extent.x, 2.0 * bb.extent.y)
            out.append(ActorSnapshot(
                id=int(a.id),
                rel_x=float(fx), rel_y=float(fy),
                rel_vx=float(rel_vx), rel_vy=float(rel_vy),
                extent=extent, yaw_rel=float(yaw_rel),
            ))
        return out

    def get_waypoints_ahead(
        self, k: int, spacing: float,
    ) -> List[Optional[WaypointPoint]]:
        ex, ey, eyaw = self._ego_pose()
        wp = self.map.get_waypoint(
            self.ego.get_location(), project_to_road=True,
        )
        out: List[Optional[WaypointPoint]] = []
        prev_yaw: Optional[float] = None
        # Advance once so the FIRST returned waypoint is `spacing` meters ahead
        # of the ego (not coincident with it).
        cur = None
        if wp is not None:
            nxt = wp.next(spacing)
            cur = nxt[0] if nxt else None
        for _ in range(k):
            if cur is None:
                out.append(None)
                continue
            nxt = cur.next(spacing)
            cur2 = nxt[0] if nxt else None
            fx, fy = world_to_ego(
                cur.transform.location.x, cur.transform.location.y,
                ex, ey, eyaw,
            )
            cur_yaw = math.radians(cur.transform.rotation.yaw)
            if prev_yaw is None:
                curvature = 0.0
            else:
                dtheta = wrap_angle(cur_yaw - prev_yaw)
                curvature = dtheta / max(spacing, 1e-3)
            prev_yaw = cur_yaw
            out.append(WaypointPoint(
                rel_x=float(fx), rel_y=float(fy),
                curvature=float(np.clip(curvature, -1.0, 1.0)),
            ))
            cur = cur2
        return out

    def get_traffic_light_ahead(self, max_range: float) -> TrafficInfo:
        tl = None
        try:
            tl = self.ego.get_traffic_light()
        except Exception:
            tl = None
        if tl is None:
            return TrafficInfo(state="none", distance_to_stop=-1.0)

        state = "none"
        if carla is not None:
            s = tl.get_state()
            if s == carla.TrafficLightState.Red:
                state = "red"
            elif s == carla.TrafficLightState.Yellow:
                state = "yellow"
            elif s == carla.TrafficLightState.Green:
                state = "green"

        # Distance to the stop waypoint of the traffic light
        dist = -1.0
        try:
            stops = tl.get_stop_waypoints()
            if stops:
                ego_loc = self.ego.get_location()
                best = min(
                    stops,
                    key=lambda w: (w.transform.location.x - ego_loc.x) ** 2
                    + (w.transform.location.y - ego_loc.y) ** 2,
                )
                dist = math.hypot(
                    best.transform.location.x - ego_loc.x,
                    best.transform.location.y - ego_loc.y,
                )
                if dist > max_range:
                    dist = -1.0
        except Exception:
            dist = -1.0

        return TrafficInfo(state=state, distance_to_stop=float(dist))

    def get_road_polylines(self) -> Optional[Sequence[np.ndarray]]:
        """Lane corridor polygon (left+right edges) around ego for ~max range."""
        try:
            ex, ey, eyaw = self._ego_pose()
            wp = self.map.get_waypoint(
                self.ego.get_location(), project_to_road=True,
            )
            if wp is None:
                return None
            fwd: List[np.ndarray] = []
            cur = wp
            # Collect forward waypoints
            for _ in range(40):
                fwd.append(np.array(
                    [cur.transform.location.x, cur.transform.location.y],
                    dtype=np.float32,
                ))
                nxt = cur.next(1.0)
                if not nxt:
                    break
                cur = nxt[0]
            back: List[np.ndarray] = []
            cur = wp
            for _ in range(10):
                prv = cur.previous(1.0)
                if not prv:
                    break
                cur = prv[0]
                back.append(np.array(
                    [cur.transform.location.x, cur.transform.location.y],
                    dtype=np.float32,
                ))
            centers = np.array(list(reversed(back)) + fwd, dtype=np.float32)
            if len(centers) < 2:
                return None
            # Build left/right edges using lane_width
            lw = max(wp.lane_width, 1.0) * 0.5
            # tangents via finite diff
            diffs = np.diff(centers, axis=0)
            tang = np.vstack([diffs, diffs[-1:]])
            norm = np.linalg.norm(tang, axis=1, keepdims=True) + 1e-6
            tang = tang / norm
            # left normal = rotate tang 90 deg CCW
            left = np.stack([-tang[:, 1], tang[:, 0]], axis=1)
            left_edge = centers + left * lw
            right_edge = centers - left * lw
            poly_world = np.vstack([left_edge, right_edge[::-1]])
            poly_ego = batch_world_to_ego(poly_world, ex, ey, eyaw)
            return [poly_ego]
        except Exception:
            return None

    def get_lane_polylines(self) -> Optional[Sequence[np.ndarray]]:
        try:
            ex, ey, eyaw = self._ego_pose()
            wp = self.map.get_waypoint(
                self.ego.get_location(), project_to_road=True,
            )
            if wp is None:
                return None
            pts = []
            cur = wp
            for _ in range(40):
                pts.append([cur.transform.location.x, cur.transform.location.y])
                nxt = cur.next(1.0)
                if not nxt:
                    break
                cur = nxt[0]
            if len(pts) < 2:
                return None
            world = np.array(pts, dtype=np.float32)
            ego_pts = batch_world_to_ego(world, ex, ey, eyaw)
            return [ego_pts]
        except Exception:
            return None
