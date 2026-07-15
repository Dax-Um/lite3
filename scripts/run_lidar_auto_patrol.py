#!/usr/bin/env python3
"""Run gated LiDAR auto patrol with real ROS2 sensors and UDP output."""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from lite3_behavior.patrol_controller import PatrolController  # noqa: E402
from lite3_behavior.patrol_fsm import PatrolContext, PatrolFSM  # noqa: E402
from lite3_common.types import MotionLimits  # noqa: E402
from lite3_control.runtime_motion_output import RuntimeMotionOutput  # noqa: E402
from lite3_control.udp_driver import Lite3UdpDriver  # noqa: E402
from lite3_ros.patrol_node import PatrolRosBridge  # noqa: E402
from lite3_ros.patrol_rclpy_node import (  # noqa: E402
    PatrolRclpyNode,
    RuntimeFlags,
    rclpy,
)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run gated Lite3 LiDAR auto patrol with UDP motion output."
    )
    parser.add_argument("--host", required=True)
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--odom-topic", default="/leg_odom2")
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--max-lane-count", type=int, default=1)
    parser.add_argument("--patrol-speed-mps", type=float, default=0.05)
    parser.add_argument("--lane-spacing-m", type=float, default=0.5)
    parser.add_argument("--duration-sec", type=float, default=10.0)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--preflight-ok", action="store_true")
    parser.add_argument("--auto-mode-ok", action="store_true")
    parser.add_argument("--stand-ready-ok", action="store_true")
    return parser.parse_args(argv)


def validate_args(args: argparse.Namespace) -> None:
    if not args.execute:
        raise SystemExit("--execute is required for real UDP runtime")
    if not args.preflight_ok:
        raise SystemExit("--preflight-ok is required")
    if not args.auto_mode_ok:
        raise SystemExit("--auto-mode-ok is required")
    if not args.stand_ready_ok:
        raise SystemExit("--stand-ready-ok is required")
    if args.port < 1 or args.port > 65535:
        raise SystemExit("--port must be in range 1..65535")
    if args.max_lane_count < 1:
        raise SystemExit("--max-lane-count must be positive")
    if args.patrol_speed_mps <= 0.0:
        raise SystemExit("--patrol-speed-mps must be positive")
    if args.patrol_speed_mps > 0.30:
        raise SystemExit("--patrol-speed-mps must be <= 0.30 for first runtime")
    if args.lane_spacing_m <= 0.0:
        raise SystemExit("--lane-spacing-m must be positive")
    if args.duration_sec <= 0.0:
        raise SystemExit("--duration-sec must be positive")
    if args.duration_sec > 30.0:
        raise SystemExit("--duration-sec must be <= 30.0 for first runtime")


def motion_host_reachable(host: str, *, runner=subprocess.run) -> bool:
    result = runner(
        ["ping", "-c", "1", "-W", "1", host],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return result.returncode == 0


def build_node_kwargs(
    args: argparse.Namespace,
    *,
    driver_factory=Lite3UdpDriver,
) -> dict:
    context = PatrolContext(
        max_lane_count=args.max_lane_count,
        lane_spacing_m=args.lane_spacing_m,
        patrol_speed_mps=args.patrol_speed_mps,
    )
    bridge = PatrolRosBridge(PatrolController(fsm=PatrolFSM(context)))
    limits = MotionLimits(
        max_vx_mps=max(0.10, args.patrol_speed_mps),
        max_vy_mps=0.05,
        max_wz_radps=0.20,
    )
    driver = driver_factory(args.host, args.port, limits)
    return {
        "scan_topic": args.scan_topic,
        "odom_topic": args.odom_topic,
        "imu_topic": args.imu_topic,
        "bridge": bridge,
        "runtime_flags": RuntimeFlags(
            motion_host_reachable=True,
            preflight_ok=args.preflight_ok,
            auto_mode_ok=args.auto_mode_ok,
            stand_ready_ok=args.stand_ready_ok,
        ),
        "motion_output": RuntimeMotionOutput(driver),
        "auto_start": True,
    }


def run_runtime(args: argparse.Namespace) -> int:
    if not motion_host_reachable(args.host):
        print(f"motion host is not reachable: {args.host}", file=sys.stderr)
        return 1
    if rclpy is None:
        print("ROS2 runtime unavailable; source /opt/ros/jazzy/setup.bash", file=sys.stderr)
        return 2

    rclpy.init(args=None)
    node = PatrolRclpyNode(**build_node_kwargs(args))
    driver = node.motion_output.driver
    print(
        "Starting gated LiDAR auto patrol "
        f"host={args.host} port={args.port} duration_sec={args.duration_sec}"
    )
    try:
        end_time = time.monotonic() + args.duration_sec
        while time.monotonic() < end_time:
            rclpy.spin_once(node, timeout_sec=0.1)
        return 0
    finally:
        try:
            driver.stop(20, 0.05)
        finally:
            driver.close()
            node.destroy_node()
            rclpy.shutdown()


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    validate_args(args)
    return run_runtime(args)


if __name__ == "__main__":
    raise SystemExit(main())
