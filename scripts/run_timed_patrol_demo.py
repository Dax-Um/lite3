#!/usr/bin/env python3
"""Run a guarded time-based Lite3 patrol demo."""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path
from typing import Callable, NamedTuple

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from lite3_common.types import MotionLimits
from lite3_control.udp_driver import Lite3UdpDriver

HOST_ENV = "LITE3_MOTION_HOST"
PORT_ENV = "LITE3_MOTION_PORT"

DEFAULT_VX_MPS = 0.20
DEFAULT_FORWARD_SEC = 1.0
DEFAULT_TURN_WZ_RADPS = 0.20
DEFAULT_TURN_SEC = 0.8
DEFAULT_LANE_COUNT = 2
DEFAULT_SEND_PERIOD_SEC = 0.05

SAFE_MAX_VX_MPS = 0.30
SAFE_MAX_WZ_RADPS = 0.30
SAFE_MAX_SEGMENT_SEC = 2.0
ALLOW_FAST_MAX_VX_MPS = 1.0
ALLOW_FAST_MAX_WZ_RADPS = 0.8
ALLOW_LONG_MAX_SEGMENT_SEC = 6.0

PRE_STOP_REPEAT = 10
INTER_SEGMENT_STOP_REPEAT = 10
FINAL_STOP_REPEAT = 60
STOP_PERIOD_SEC = 0.05

DriverFactory = Callable[..., Lite3UdpDriver]


class Segment(NamedTuple):
    name: str
    vx: float
    vy: float
    wz: float
    duration_sec: float


def _env_port() -> int | None:
    raw_port = os.environ.get(PORT_ENV)
    if raw_port is None:
        return None
    return int(raw_port)


def _env_local_port() -> int | None:
    raw_port = os.environ.get("LITE3_MOTION_LOCAL_PORT")
    if raw_port is None or raw_port == "":
        return None
    return int(raw_port)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run a guarded time-based Lite3 patrol demo over UDP."
    )
    parser.add_argument("--host", default=os.environ.get(HOST_ENV, ""))
    parser.add_argument("--port", type=int, default=_env_port())
    parser.add_argument(
        "--local-host",
        default=os.environ.get("LITE3_MOTION_LOCAL_HOST", ""),
        help="Local source IP to bind. Empty binds all local interfaces.",
    )
    parser.add_argument(
        "--local-port",
        type=int,
        default=_env_local_port(),
        help="Optional local UDP source port. Omit to let the OS choose an ephemeral port.",
    )
    parser.add_argument("--lane-count", type=int, default=DEFAULT_LANE_COUNT)
    parser.add_argument("--vx", type=float, default=DEFAULT_VX_MPS)
    parser.add_argument("--forward-sec", type=float, default=DEFAULT_FORWARD_SEC)
    parser.add_argument(
        "--turn-wz",
        type=float,
        default=DEFAULT_TURN_WZ_RADPS,
        help="Reserved for a future turn-around mode; ignored by the current shuttle plan.",
    )
    parser.add_argument(
        "--turn-sec",
        type=float,
        default=DEFAULT_TURN_SEC,
        help="Reserved for a future turn-around mode; ignored by the current shuttle plan.",
    )
    parser.add_argument("--send-period-sec", type=float, default=DEFAULT_SEND_PERIOD_SEC)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually send UDP motion commands. Without this flag, only print the plan.",
    )
    parser.add_argument(
        "--preflight-ok",
        action="store_true",
        help="Required with --execute after area, E-stop, leash, and operator checks.",
    )
    parser.add_argument(
        "--auto-mode-ok",
        action="store_true",
        help="Required with --execute after the Lite3 App is switched to Auto Mode.",
    )
    parser.add_argument(
        "--stand-ready-ok",
        action="store_true",
        help="Required with --execute after the robot is standing and ready.",
    )
    parser.add_argument(
        "--allow-fast",
        action="store_true",
        help="Allow speeds above conservative demo defaults, up to the script hard cap.",
    )
    parser.add_argument(
        "--allow-long-segment",
        action="store_true",
        help="Allow segment durations above conservative demo defaults.",
    )
    return parser.parse_args(argv)


def validate_args(args: argparse.Namespace) -> None:
    _require_finite("--vx", args.vx)
    _require_finite("--forward-sec", args.forward_sec)
    _require_finite("--turn-wz", args.turn_wz)
    _require_finite("--turn-sec", args.turn_sec)
    _require_finite("--send-period-sec", args.send_period_sec)

    if not args.host:
        raise SystemExit("--host must be non-empty")
    if args.port is None:
        raise SystemExit("--port must be provided")
    if args.port < 1 or args.port > 65535:
        raise SystemExit("--port must be in range 1..65535")
    if args.local_port is not None and (
        args.local_port < 1 or args.local_port > 65535
    ):
        raise SystemExit("--local-port must be in range 1..65535")
    if args.lane_count < 1 or args.lane_count > 4:
        raise SystemExit("--lane-count must be in range 1..4")
    if args.forward_sec <= 0.0:
        raise SystemExit("--forward-sec must be positive")
    if args.turn_sec <= 0.0:
        raise SystemExit("--turn-sec must be positive")
    if args.send_period_sec <= 0.0:
        raise SystemExit("--send-period-sec must be positive")

    vx_limit = ALLOW_FAST_MAX_VX_MPS if args.allow_fast else SAFE_MAX_VX_MPS
    wz_limit = ALLOW_FAST_MAX_WZ_RADPS if args.allow_fast else SAFE_MAX_WZ_RADPS
    segment_limit = (
        ALLOW_LONG_MAX_SEGMENT_SEC
        if args.allow_long_segment
        else SAFE_MAX_SEGMENT_SEC
    )
    if abs(args.vx) > vx_limit:
        flag = "--allow-fast" if not args.allow_fast else f"{ALLOW_FAST_MAX_VX_MPS}"
        raise SystemExit(f"--vx exceeds limit; use {flag} only after staged validation")
    if abs(args.turn_wz) > wz_limit:
        flag = "--allow-fast" if not args.allow_fast else f"{ALLOW_FAST_MAX_WZ_RADPS}"
        raise SystemExit(f"--turn-wz exceeds limit; use {flag} only after staged validation")
    if args.forward_sec > segment_limit or args.turn_sec > segment_limit:
        flag = (
            "--allow-long-segment"
            if not args.allow_long_segment
            else f"{ALLOW_LONG_MAX_SEGMENT_SEC}"
        )
        raise SystemExit(f"segment duration exceeds limit; use {flag}")

    if args.execute:
        if not args.preflight_ok:
            raise SystemExit("--execute requires --preflight-ok")
        if not args.auto_mode_ok:
            raise SystemExit("--execute requires --auto-mode-ok")
        if not args.stand_ready_ok:
            raise SystemExit("--execute requires --stand-ready-ok")


def build_plan(args: argparse.Namespace) -> list[Segment]:
    plan: list[Segment] = []
    for index in range(args.lane_count):
        lane_number = index + 1
        direction = 1.0 if index % 2 == 0 else -1.0
        suffix = "forward" if direction > 0.0 else "reverse"
        plan.append(
            Segment(
                name=f"lane_{lane_number}_{suffix}",
                vx=args.vx * direction,
                vy=0.0,
                wz=0.0,
                duration_sec=args.forward_sec,
            )
        )
    return plan


def run_timed_patrol(
    args: argparse.Namespace,
    *,
    driver_factory: DriverFactory = Lite3UdpDriver,
    sleep: Callable[[float], None] = time.sleep,
) -> None:
    plan = build_plan(args)
    if not args.execute:
        print_plan(args, plan)
        return

    limits = MotionLimits(
        max_vx_mps=max(abs(args.vx), 0.01),
        max_vy_mps=0.01,
        max_wz_radps=max(abs(args.turn_wz), 0.01),
    )
    driver = driver_factory(
        args.host,
        args.port,
        limits,
        local_host=args.local_host,
        local_port=args.local_port,
    )

    try:
        print_plan(args, plan)
        driver.stop(PRE_STOP_REPEAT, STOP_PERIOD_SEC)
        for segment in plan:
            print(
                f"segment {segment.name}: vx={segment.vx:.3f} "
                f"wz={segment.wz:.3f} duration_sec={segment.duration_sec:.2f}"
            )
            _run_segment(driver, segment, args.send_period_sec, sleep)
            driver.stop(INTER_SEGMENT_STOP_REPEAT, STOP_PERIOD_SEC)
    finally:
        driver.stop(FINAL_STOP_REPEAT, STOP_PERIOD_SEC)
        driver.close()
        print("timed patrol complete; final stop command sent")


def print_plan(args: argparse.Namespace, plan: list[Segment]) -> None:
    mode = "EXECUTE" if args.execute else "DRY-RUN"
    estimated_forward_m = args.lane_count * abs(args.vx) * args.forward_sec
    print(f"timed patrol mode={mode} host={args.host} port={args.port}")
    print(
        f"lane_count={args.lane_count} estimated_forward_total_m="
        f"{estimated_forward_m:.2f}"
    )
    for segment in plan:
        print(
            f"- {segment.name}: vx={segment.vx:.3f} vy={segment.vy:.3f} "
            f"wz={segment.wz:.3f} duration_sec={segment.duration_sec:.2f}"
        )


def _run_segment(
    driver: Lite3UdpDriver,
    segment: Segment,
    send_period_sec: float,
    sleep: Callable[[float], None],
) -> None:
    steps = max(1, math.ceil(segment.duration_sec / send_period_sec))
    for _index in range(steps):
        driver.send_cmd_vel(segment.vx, segment.vy, segment.wz)
        sleep(send_period_sec)


def _require_finite(name: str, value: float) -> None:
    if not math.isfinite(value):
        raise SystemExit(f"{name} must be finite")


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    validate_args(args)
    run_timed_patrol(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
