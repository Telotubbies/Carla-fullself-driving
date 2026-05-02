#!/usr/bin/env python3
"""Standalone CARLA -> ROS2 bridge for RViz2 debugging.

Connects to the running CARLA server as a SECOND client (read-only, does
not alter sync mode), finds the training ego vehicle, and publishes the
ground-truth state (TF + markers + path) on ROS2 topics so `rviz2` can
visualize it live without ROS bridge.

Must be launched with ROS2 sourced AND venv site-packages on PYTHONPATH,
for example:

    source /opt/ros/jazzy/setup.bash
    PYTHONPATH=./venv/lib/python3.12/site-packages:$PYTHONPATH \
        python3 scripts/carla_rviz_bridge.py

`scripts/start_rviz.sh` does this for you.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402

import carla  # type: ignore

import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore

from src.bridge.direct_source import DirectCarlaSource  # noqa: E402
from src.gt_state import DEFAULT_SCHEMA, StateBuilder, StateSchema  # noqa: E402
from src.gt_state.actor_filter import filter_and_rank, pad_to_n  # noqa: E402
from src.viz.rviz_publisher import RvizPublisher, RvizPublisherConfig  # noqa: E402


def find_ego(world: carla.World) -> carla.Vehicle | None:
    vehicles = list(world.get_actors().filter("vehicle.*"))
    if not vehicles:
        return None
    for v in vehicles:
        if "tesla.model3" in v.type_id:
            return v
    return vehicles[0]


class CarlaRvizBridge(Node):
    def __init__(self, host: str, port: int, rate_hz: float = 20.0):
        super().__init__("carla_rviz_bridge")
        self.client = carla.Client(host, port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.schema: StateSchema = DEFAULT_SCHEMA
        self.publisher = RvizPublisher(
            schema=self.schema,
            config=RvizPublisherConfig(frame_map="map", frame_ego="ego"),
        )
        self.builder = StateBuilder(schema=self.schema)
        self.ego: carla.Vehicle | None = None
        self.source: DirectCarlaSource | None = None
        self.timer = self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f"connected to CARLA {host}:{port} map={self.world.get_map().name}",
        )

    def _ensure_ego(self) -> bool:
        if self.ego is not None and self.ego.is_alive:
            return True
        ego = find_ego(self.world)
        if ego is None:
            return False
        self.ego = ego
        self.source = DirectCarlaSource(self.world, ego)
        self.get_logger().info(
            f"tracking ego id={ego.id} type={ego.type_id}",
        )
        return True

    def _tick(self) -> None:
        if not self._ensure_ego() or self.source is None:
            return
        try:
            ego_state = self.source.get_ego_state()
            actors_all = self.source.get_nearby_actors(self.schema.max_range)
            ranked = filter_and_rank(
                actors_all,
                max_range=self.schema.max_range,
                fov_deg=self.schema.fov_deg,
                top_n=self.schema.n_vehicles,
            )
            padded = pad_to_n(ranked, self.schema.n_vehicles)
            wps = self.source.get_waypoints_ahead(
                k=self.schema.k_waypoints,
                spacing=self.schema.waypoint_spacing,
            )
            traffic = self.source.get_traffic_light_ahead(self.schema.max_range)

            tf_world = self.ego.get_transform()
            ego_yaw = math.radians(tf_world.rotation.yaw)
            self.publisher.publish_all(
                ego_world_xy=(tf_world.location.x, tf_world.location.y),
                ego_yaw=ego_yaw,
                ego=ego_state,
                actors=padded,
                waypoints=wps,
                traffic=traffic,
            )
        except Exception as e:
            self.get_logger().warn(f"tick failed: {e}")

    def destroy(self) -> None:
        try:
            self.publisher.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--rate", type=float, default=20.0)
    args = ap.parse_args()

    rclpy.init(args=None)
    node = CarlaRvizBridge(args.host, args.port, args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
