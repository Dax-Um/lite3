#!/usr/bin/env python3
"""IQ9 ROS2 state bridge entrypoint.

Dry-run mode only prints subscriptions. Live mode is intended for the IQ9 ROS2
container and never publishes motion commands.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from lite3_iq9.ros2_state_bridge import Iq9Ros2StateBridge  # noqa: E402
from lite3_iq9.camera_source import CameraSourceConfig  # noqa: E402
from lite3_iq9.runtime_state import RuntimeStateAggregator  # noqa: E402
from lite3_iq9.state_subscribers import TopicFreshnessMonitor  # noqa: E402


REQUIRED_TOPICS = (
    "/odom",
    "/status",
    "/map",
    "/local_costmap/costmap",
    "/global_costmap/costmap",
    "/cmd_vel",
)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the IQ9 read-only ROS2 state bridge.")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--stale-after-sec", type=float, default=1.0)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.dry_run:
        print("mode=dry-run")
        print("motion command publisher: disabled")
        for topic in REQUIRED_TOPICS:
            print(f"subscribe: {topic}")
        camera_source = CameraSourceConfig.from_env(os.environ)
        if camera_source.source_type == "rtsp":
            print("camera source: rtsp via LITE3_RTSP_URL")
            print(f"camera source redacted: {camera_source.redacted_url}")
        else:
            print("camera source: disabled")
        return 0

    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import OccupancyGrid, Odometry
        from std_msgs.msg import Bool
    except ImportError as exc:
        print(f"ROS2 runtime unavailable: {exc}", file=sys.stderr)
        return 2

    rclpy.init(args=None)
    node = rclpy.create_node("lite3_iq9_state_bridge")
    aggregator = RuntimeStateAggregator()
    monitor = TopicFreshnessMonitor(REQUIRED_TOPICS, stale_after_sec=args.stale_after_sec)
    bridge = Iq9Ros2StateBridge(aggregator, monitor)

    def now() -> float:
        return time.monotonic()

    try:
        node.create_subscription(Odometry, "/odom", lambda msg: bridge.on_odom(msg, now()), 10)
        node.create_subscription(Bool, "/status", lambda msg: bridge.on_status(msg, now()), 10)
        node.create_subscription(OccupancyGrid, "/map", lambda msg: bridge.on_map(msg, now()), 10)
        node.create_subscription(
            OccupancyGrid,
            "/local_costmap/costmap",
            lambda msg: bridge.on_local_costmap(msg, now()),
            10,
        )
        node.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            lambda msg: bridge.on_global_costmap(msg, now()),
            10,
        )
        node.create_subscription(Twist, "/cmd_vel", lambda msg: bridge.on_cmd_vel(msg, now()), 10)
        rclpy.spin(node)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
