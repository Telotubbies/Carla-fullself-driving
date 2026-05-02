"""RvizPublisher: publish TF + markers for debugging RL rollouts.

Publishes:
  - TF:     map -> ego
  - Pose:   /rl_debug/ego_pose           (geometry_msgs/PoseStamped)
  - Path:   /rl_debug/waypoints          (nav_msgs/Path)
  - Markers:/rl_debug/vehicles           (visualization_msgs/MarkerArray)
  - Markers:/rl_debug/fov                (visualization_msgs/Marker, line strip cone)
  - Markers:/rl_debug/traffic            (visualization_msgs/Marker, sphere)

Import-safe: if rclpy / ROS2 is not installed the class becomes a no-op
stub so training on pure CARLA-PythonAPI does not require ROS2.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import (
        PoseStamped,
        Point,
        Quaternion,
        TransformStamped,
    )
    from nav_msgs.msg import Path
    from visualization_msgs.msg import Marker, MarkerArray
    from std_msgs.msg import ColorRGBA, Header
    from tf2_ros import TransformBroadcaster
    _ROS2_OK = True
except Exception:  # pragma: no cover
    _ROS2_OK = False

from src.gt_state.actor_filter import ActorSnapshot
from src.gt_state.schema import StateSchema
from src.gt_state.vector_builder import EgoState, TrafficInfo, WaypointPoint


@dataclass
class RvizPublisherConfig:
    node_name: str = "rl_debug_publisher"
    frame_map: str = "map"
    frame_ego: str = "ego"


def _yaw_to_quat(yaw: float):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class _NoOpPublisher:
    def __init__(self, *a, **k): pass
    def publish_all(self, *a, **k): pass
    def shutdown(self): pass


def _make_real_publisher_class():
    class _RealPublisher(Node):
        def __init__(self, cfg: RvizPublisherConfig, schema: StateSchema):
            super().__init__(cfg.node_name)
            self.cfg = cfg
            self.schema = schema
            self._tf = TransformBroadcaster(self)
            self._pose_pub = self.create_publisher(PoseStamped, "/rl_debug/ego_pose", 10)
            self._path_pub = self.create_publisher(Path, "/rl_debug/waypoints", 10)
            self._veh_pub = self.create_publisher(MarkerArray, "/rl_debug/vehicles", 10)
            self._fov_pub = self.create_publisher(Marker, "/rl_debug/fov", 10)
            self._tl_pub = self.create_publisher(Marker, "/rl_debug/traffic", 10)
            self._ped_pub = self.create_publisher(MarkerArray, "/rl_debug/pedestrians", 10)
            self._goal_pub = self.create_publisher(Marker, "/rl_debug/goal", 10)

        def _now(self):
            return self.get_clock().now().to_msg()

        def publish_all(
            self,
            ego_world_xy,            # (x, y)
            ego_yaw: float,
            ego: EgoState,
            actors: List[Optional[ActorSnapshot]],
            waypoints: List[Optional[WaypointPoint]],
            traffic: TrafficInfo,
        ) -> None:
            now = self._now()
            ex, ey = ego_world_xy
            qx, qy, qz, qw = _yaw_to_quat(ego_yaw)

            # --- TF
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.cfg.frame_map
            t.child_frame_id = self.cfg.frame_ego
            t.transform.translation.x = float(ex)
            t.transform.translation.y = float(ey)
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self._tf.sendTransform(t)

            # --- Ego pose (in map frame)
            ps = PoseStamped()
            ps.header.stamp = now
            ps.header.frame_id = self.cfg.frame_map
            ps.pose.position.x = float(ex)
            ps.pose.position.y = float(ey)
            ps.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
            self._pose_pub.publish(ps)

            # --- Waypoints as Path (in ego frame for simplicity)
            path = Path()
            path.header.stamp = now
            path.header.frame_id = self.cfg.frame_ego
            for w in waypoints:
                if w is None:
                    continue
                p = PoseStamped()
                p.header = path.header
                p.pose.position.x = float(w.rel_x)
                p.pose.position.y = float(w.rel_y)
                p.pose.orientation.w = 1.0
                path.poses.append(p)
            self._path_pub.publish(path)

            # --- Vehicle markers in ego frame
            ma = MarkerArray()
            for i, a in enumerate(actors):
                m = Marker()
                m.header.stamp = now
                m.header.frame_id = self.cfg.frame_ego
                m.ns = "vehicles"
                m.id = i
                m.type = Marker.CUBE
                m.action = Marker.ADD if a is not None else Marker.DELETE
                if a is not None:
                    m.pose.position.x = float(a.rel_x)
                    m.pose.position.y = float(a.rel_y)
                    qx2, qy2, qz2, qw2 = _yaw_to_quat(a.yaw_rel)
                    m.pose.orientation = Quaternion(x=qx2, y=qy2, z=qz2, w=qw2)
                    length, width = a.extent if a.extent else (4.5, 1.8)
                    m.scale.x = float(length)
                    m.scale.y = float(width)
                    m.scale.z = 1.6
                    # Color based on distance - closer = more red
                    dist = math.sqrt(a.rel_x**2 + a.rel_y**2)
                    if dist < 10.0:
                        m.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9)  # Red - danger
                    elif dist < 20.0:
                        m.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.8)  # Orange - warning
                    else:
                        m.color = ColorRGBA(r=0.0, g=0.8, b=0.0, a=0.7)  # Green - safe
                    # Add text marker with distance
                    text_m = Marker()
                    text_m.header.stamp = now
                    text_m.header.frame_id = self.cfg.frame_ego
                    text_m.ns = "vehicle_labels"
                    text_m.id = i + 1000
                    text_m.type = Marker.TEXT_VIEW_FACING
                    text_m.action = Marker.ADD
                    text_m.pose.position.x = float(a.rel_x)
                    text_m.pose.position.y = float(a.rel_y)
                    text_m.pose.position.z = 2.5
                    text_m.scale.z = 0.5
                    text_m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                    text_m.text = f"{dist:.1f}m"
                    ma.markers.append(text_m)
                ma.markers.append(m)
            self._veh_pub.publish(ma)

            # --- FOV cone
            half = math.radians(self.schema.fov_deg) * 0.5
            r = self.schema.max_range
            
            fov_marker = Marker()
            fov_marker.header.stamp = now
            fov_marker.header.frame_id = self.cfg.frame_ego
            fov_marker.ns = "fov"
            fov_marker.id = 0
            fov_marker.type = Marker.LINE_STRIP
            fov_marker.action = Marker.ADD
            fov_marker.scale.x = 0.1
            fov_marker.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.4)
            # Add filled FOV area
            fov_fill = Marker()
            fov_fill.header.stamp = now
            fov_fill.header.frame_id = self.cfg.frame_ego
            fov_fill.ns = "fov_fill"
            fov_fill.id = 1
            fov_fill.type = Marker.TRIANGLE_LIST
            fov_fill.action = Marker.ADD
            fov_fill.scale.x = 1.0
            fov_fill.color = ColorRGBA(r=0.0, g=0.3, b=0.8, a=0.15)
            for angle in [i * 0.1 for i in range(-int(half/0.1), int(half/0.1) + 1)]:
                fov_fill.points.append(Point(x=0.0, y=0.0, z=0.1))
                fov_fill.points.append(Point(x=float(r * math.cos(angle + 0.05)), y=float(r * math.sin(angle + 0.05)), z=0.1))
                fov_fill.points.append(Point(x=float(r * math.cos(angle)), y=float(r * math.sin(angle)), z=0.1))
            self._fov_pub.publish(fov_fill)
            fov_marker.points.append(Point(x=0.0, y=0.0, z=0.2))
            fov_marker.points.append(Point(
                x=float(r * math.cos(half)),
                y=float(r * math.sin(half)), z=0.2,
            ))
            fov_marker.points.append(Point(
                x=float(r * math.cos(-half)),
                y=float(r * math.sin(-half)), z=0.2,
            ))
            fov_marker.points.append(Point(x=0.0, y=0.0, z=0.2))
            self._fov_pub.publish(fov_marker)

            # --- Traffic light
            tl_marker = Marker()
            tl_marker.header.stamp = now
            tl_marker.header.frame_id = self.cfg.frame_ego
            tl_marker.ns = "traffic"
            tl_marker.id = 0
            tl_marker.type = Marker.SPHERE
            if traffic.state in ("red", "yellow", "green") and traffic.distance_to_stop > 0:
                tl_marker.action = Marker.ADD
                tl_marker.pose.position.x = float(traffic.distance_to_stop)
                tl_marker.pose.position.y = 0.0
                tl_marker.pose.position.z = 2.0
                tl_marker.scale.x = tl_marker.scale.y = tl_marker.scale.z = 0.8
                col = {
                    "red":    (1.0, 0.0, 0.0),
                    "yellow": (1.0, 1.0, 0.0),
                    "green":  (0.0, 1.0, 0.0),
                }[traffic.state]
                tl_marker.color = ColorRGBA(r=col[0], g=col[1], b=col[2], a=1.0)
            else:
                tl_marker.action = Marker.DELETE
            self._tl_pub.publish(tl_marker)

        def shutdown(self) -> None:
            try:
                self.destroy_node()
            except Exception:
                pass

    return _RealPublisher


if _ROS2_OK:
    _RealPublisherCls = _make_real_publisher_class()


class RvizPublisher:
    """Public facade that is safe whether ROS2 is available or not."""

    def __init__(
        self,
        schema: StateSchema,
        config: Optional[RvizPublisherConfig] = None,
    ):
        self.schema = schema
        self.cfg = config or RvizPublisherConfig()
        self._impl = None
        if _ROS2_OK:
            try:
                if not rclpy.ok():
                    rclpy.init(args=None)
                self._impl = _RealPublisherCls(self.cfg, schema)
            except Exception as e:
                print(f"[RvizPublisher] init failed, falling back to no-op: {e}")
                self._impl = None

    def enabled(self) -> bool:
        return self._impl is not None

    def publish_all(self, **kwargs) -> None:
        if self._impl is None:
            return
        self._impl.publish_all(**kwargs)

    def shutdown(self) -> None:
        if self._impl is not None:
            self._impl.shutdown()
            try:
                if _ROS2_OK and rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass
            self._impl = None
