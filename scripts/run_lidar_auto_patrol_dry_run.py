#!/usr/bin/env python3
"""Run LiDAR auto patrol with real ROS2 sensors and no UDP output."""

import argparse
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from lite3_behavior.patrol_controller import PatrolController  # noqa: E402
from lite3_behavior.patrol_fsm import PatrolContext, PatrolFSM  # noqa: E402
from lite3_ros.patrol_node import PatrolRosBridge  # noqa: E402
from lite3_ros.patrol_rclpy_node import (  # noqa: E402
    PatrolRclpyNode,
    RuntimeFlags,
    rclpy,
)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run sensor-based Lite3 auto patrol dry-run without UDP output."
    )
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--odom-topic", default="/leg_odom2")
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--duration-sec", type=float, default=30.0)
    parser.add_argument("--patrol-speed-mps", type=float, default=0.05)
    parser.add_argument("--max-lane-count", type=int, default=1)
    parser.add_argument("--lane-spacing-m", type=float, default=0.5)
    return parser.parse_args(argv)


def validate_args(args: argparse.Namespace) -> None:
    if args.duration_sec <= 0.0:
        raise SystemExit("--duration-sec must be positive")
    if args.patrol_speed_mps <= 0.0:
        raise SystemExit("--patrol-speed-mps must be positive")
    if args.patrol_speed_mps > 0.30:
        raise SystemExit("--patrol-speed-mps must be <= 0.30 for dry-run")
    if args.max_lane_count < 1:
        raise SystemExit("--max-lane-count must be positive")
    if args.lane_spacing_m <= 0.0:
        raise SystemExit("--lane-spacing-m must be positive")


def build_node_kwargs(args: argparse.Namespace) -> dict:
    context = PatrolContext(
        max_lane_count=args.max_lane_count,
        lane_spacing_m=args.lane_spacing_m,
        patrol_speed_mps=args.patrol_speed_mps,
    )
    bridge = PatrolRosBridge(PatrolController(fsm=PatrolFSM(context)))
    return {
        "scan_topic": args.scan_topic,
        "odom_topic": args.odom_topic,
        "imu_topic": args.imu_topic,
        "bridge": bridge,
        "runtime_flags": RuntimeFlags(
            motion_host_reachable=True,
            preflight_ok=True,
            auto_mode_ok=True,
            stand_ready_ok=True,
        ),
        "motion_output": None,
        "auto_start": True,
    }


def run_dry_run(args: argparse.Namespace) -> int:
    if rclpy is None:
        print("ROS2 runtime unavailable; source /opt/ros/jazzy/setup.bash", file=sys.stderr)
        return 2

    rclpy.init(args=None)
    node = PatrolRclpyNode(**build_node_kwargs(args))
    print("No UDP packets sent.")
    try:
        end_time = time.monotonic() + args.duration_sec
        while time.monotonic() < end_time:
            rclpy.spin_once(node, timeout_sec=0.1)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    validate_args(args)
    return run_dry_run(args)


if __name__ == "__main__":
    raise SystemExit(main())
