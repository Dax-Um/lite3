#!/usr/bin/env python3
"""Run a pure-Python patrol controller dry run with fake odom and scan."""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from lite3_common.types import Pose2D
from lite3_behavior.patrol_controller import PatrolController


HEADER = (
    "time,state,lane_index,direction,min_front,"
    "raw_vx,raw_vy,raw_wz,safe_vx,safe_vy,safe_wz,stop_reason"
)


def run_dry_run() -> str:
    controller = PatrolController()
    rows = [HEADER]
    now = 0.0
    controller.on_odom(Pose2D(0.0, 0.0, 0.0), now)
    controller.on_imu(now)
    controller.on_scan([2.0, 2.0, 2.0], -0.1, 0.1, now)
    controller.on_operator_command("patrol_start", now)

    for step in range(10):
        now = step / 10.0
        controller.on_imu(now)
        if step == 2:
            controller.on_scan([0.5, 0.5, 0.5], -0.1, 0.1, now)
        if step == 3:
            controller.on_scan([2.0, 2.0, 2.0], -0.1, 0.1, now)
        if step == 4:
            controller.on_odom(Pose2D(0.6, 0.0, 0.0), now)
        if step == 5:
            controller.on_odom(Pose2D(0.6, 0.0, 3.14159), now)
        if step == 6:
            controller.on_scan([0.5, 0.5, 0.5], -0.1, 0.1, now)
        if step == 7:
            controller.on_scan([2.0, 2.0, 2.0], -0.1, 0.1, now)
        if step == 8:
            controller.on_odom(Pose2D(1.2, 0.0, 3.14159), now)
        if step == 9:
            controller.on_odom(Pose2D(1.2, 0.0, 0.0), now)
        output = controller.tick(now)
        context = controller.fsm.context()
        rows.append(
            ",".join(
                [
                    f"{now:.1f}",
                    output.state,
                    str(output.lane_index),
                    str(context.direction),
                    "" if output.boundary_min_front_m is None else f"{output.boundary_min_front_m:.3f}",
                    f"{output.raw_cmd.vx:.3f}",
                    f"{output.raw_cmd.vy:.3f}",
                    f"{output.raw_cmd.wz:.3f}",
                    f"{output.safe_cmd.vx:.3f}",
                    f"{output.safe_cmd.vy:.3f}",
                    f"{output.safe_cmd.wz:.3f}",
                    output.stop_reason.value,
                ]
            )
        )

    return "\n".join(rows)


def main() -> int:
    print(run_dry_run())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
