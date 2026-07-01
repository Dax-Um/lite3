#!/usr/bin/env python3
"""Probe front LiDAR boundary detection from a ROS2 LaserScan topic."""

import argparse
import csv
import math
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from lite3_perception.lidar_boundary_detector import (  # noqa: E402
    BoundaryConfig,
    BoundaryResult,
    LidarBoundaryDetector,
)


FIELDNAMES = [
    "time",
    "min_front_m",
    "valid_points",
    "should_slow",
    "should_stop",
    "lane_end",
]


def format_boundary_row(elapsed_sec: float, result: BoundaryResult) -> dict[str, str]:
    return {
        "time": f"{elapsed_sec:.2f}",
        "min_front_m": (
            ""
            if result.min_front_distance_m is None
            else f"{result.min_front_distance_m:.3f}"
        ),
        "valid_points": str(result.valid_front_points),
        "should_slow": _bool_text(result.should_slow),
        "should_stop": _bool_text(result.should_stop),
        "lane_end": _bool_text(result.lane_end),
    }


def exit_code_for_seen_scan(seen_scan: bool) -> int:
    return 0 if seen_scan else 1


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Log front LiDAR boundary detector output from /scan."
    )
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--front-angle-deg", type=float, default=25.0)
    parser.add_argument("--stop-distance-m", type=float, default=0.60)
    parser.add_argument("--slow-distance-m", type=float, default=1.20)
    parser.add_argument("--boundary-confirm-frames", type=int, default=5)
    parser.add_argument("--duration-sec", type=float, default=20.0)
    return parser.parse_args(argv)


def validate_args(args: argparse.Namespace) -> None:
    if args.front_angle_deg <= 0.0 or args.front_angle_deg > 180.0:
        raise SystemExit("--front-angle-deg must be in range (0, 180]")
    if args.stop_distance_m <= 0.0:
        raise SystemExit("--stop-distance-m must be positive")
    if args.slow_distance_m < args.stop_distance_m:
        raise SystemExit("--slow-distance-m must be >= --stop-distance-m")
    if args.boundary_confirm_frames < 1:
        raise SystemExit("--boundary-confirm-frames must be positive")
    if args.duration_sec <= 0.0:
        raise SystemExit("--duration-sec must be positive")


def run_ros_probe(args: argparse.Namespace) -> int:
    try:
        import rclpy
        from sensor_msgs.msg import LaserScan
    except ImportError as exc:
        print(f"ROS2 runtime unavailable: {exc}", file=sys.stderr)
        return 2

    detector = LidarBoundaryDetector(
        BoundaryConfig(
            front_angle_rad=math.radians(args.front_angle_deg),
            stop_distance_m=args.stop_distance_m,
            slow_distance_m=args.slow_distance_m,
            confirm_frames=args.boundary_confirm_frames,
        )
    )
    writer = csv.DictWriter(sys.stdout, fieldnames=FIELDNAMES)
    start_time = time.monotonic()
    seen_scan = False

    rclpy.init(args=None)
    node = rclpy.create_node("lite3_probe_lidar_boundary")
    try:
        writer.writeheader()

        def on_scan(msg) -> None:
            nonlocal seen_scan
            seen_scan = True
            result = detector.update_scan(
                list(msg.ranges),
                msg.angle_min,
                msg.angle_increment,
            )
            writer.writerow(format_boundary_row(time.monotonic() - start_time, result))
            sys.stdout.flush()

        node.create_subscription(LaserScan, args.scan_topic, on_scan, 10)

        end_time = start_time + args.duration_sec
        while time.monotonic() < end_time:
            rclpy.spin_once(node, timeout_sec=0.1)

        return exit_code_for_seen_scan(seen_scan)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _bool_text(value: bool) -> str:
    return "true" if value else "false"


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    validate_args(args)
    return run_ros_probe(args)


if __name__ == "__main__":
    raise SystemExit(main())
