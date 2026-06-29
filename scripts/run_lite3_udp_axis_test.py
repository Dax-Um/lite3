#!/usr/bin/env python3
"""Run a guarded single-axis Lite3 UDP motion test."""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Callable

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from lite3_common.types import MotionLimits
from lite3_control.udp_driver import Lite3UdpDriver

DEFAULT_HOST = "192.168.1.120"
DEFAULT_PORT = 43893
DEFAULT_DURATION_SEC = 0.5
DEFAULT_SEND_PERIOD_SEC = 0.05
PRE_STOP_REPEAT = 10
POST_STOP_REPEAT = 20
STOP_PERIOD_SEC = 0.05
MAX_DURATION_SEC_WITHOUT_OVERRIDE = 1.0
AXIS_LIMITS = {
    "vx": 0.10,
    "vy": 0.05,
    "wz": 0.20,
}

DriverFactory = Callable[[str, int, MotionLimits], Lite3UdpDriver]


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send a guarded single-axis Lite3 MotionComplexCMD UDP test."
    )
    parser.add_argument("--axis", choices=("vx", "vy", "wz"), required=True)
    parser.add_argument("--value", type=float, required=True)
    parser.add_argument("--duration-sec", type=float, default=DEFAULT_DURATION_SEC)
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument(
        "--preflight-ok",
        action="store_true",
        help="Required for any non-zero command after physical safety checks are complete.",
    )
    parser.add_argument(
        "--allow-long-test",
        action="store_true",
        help="Allow duration longer than 1.0 second.",
    )
    return parser.parse_args(argv)


def validate_args(args: argparse.Namespace) -> None:
    if not math.isfinite(args.value):
        raise SystemExit("--value must be finite")
    if not math.isfinite(args.duration_sec):
        raise SystemExit("--duration-sec must be finite")
    if args.duration_sec <= 0.0:
        raise SystemExit("--duration-sec must be positive")
    if not args.host:
        raise SystemExit("--host must be non-empty")
    if args.port < 1 or args.port > 65535:
        raise SystemExit("--port must be in range 1..65535")

    limit = AXIS_LIMITS[args.axis]
    if abs(args.value) > limit:
        raise SystemExit(f"{args.axis} value exceeds limit {limit}")
    if args.value != 0.0 and not args.preflight_ok:
        raise SystemExit("non-zero command requires --preflight-ok")
    if (
        args.duration_sec > MAX_DURATION_SEC_WITHOUT_OVERRIDE
        and not args.allow_long_test
    ):
        raise SystemExit("duration over 1.0 second requires --allow-long-test")


def axis_command(axis: str, value: float) -> tuple[float, float, float]:
    if axis == "vx":
        return value, 0.0, 0.0
    if axis == "vy":
        return 0.0, value, 0.0
    if axis == "wz":
        return 0.0, 0.0, value
    raise ValueError(f"unsupported axis: {axis}")


def run_axis_test(
    args: argparse.Namespace,
    *,
    driver_factory: DriverFactory = Lite3UdpDriver,
) -> None:
    limits = MotionLimits(
        max_vx_mps=AXIS_LIMITS["vx"],
        max_vy_mps=AXIS_LIMITS["vy"],
        max_wz_radps=AXIS_LIMITS["wz"],
    )
    driver = driver_factory(args.host, args.port, limits)
    vx, vy, wz = axis_command(args.axis, args.value)

    try:
        print(
            f"starting axis test axis={args.axis} value={args.value} "
            f"duration_sec={args.duration_sec} target={args.host}:{args.port}"
        )
        driver.stop(PRE_STOP_REPEAT, STOP_PERIOD_SEC)
        start = time.monotonic()
        while time.monotonic() - start < args.duration_sec:
            driver.send_cmd_vel(vx, vy, wz)
            time.sleep(DEFAULT_SEND_PERIOD_SEC)
    finally:
        driver.stop(POST_STOP_REPEAT, STOP_PERIOD_SEC)
        driver.close()
        print("axis test complete; stop command sent")


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    validate_args(args)
    run_axis_test(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
