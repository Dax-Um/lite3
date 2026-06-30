#!/usr/bin/env python3
"""Run a sensor-free patrol FSM dry-run sequence."""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from lite3_behavior.patrol_events import PatrolEvent
from lite3_behavior.patrol_fsm import PatrolFSM


def run_dry_run() -> str:
    fsm = PatrolFSM()
    rows = ["time,state,lane_index,direction,vx,vy,wz,event"]
    sequence: list[PatrolEvent | None] = [
        PatrolEvent.PATROL_START,
        None,
        PatrolEvent.LANE_END,
        None,
        PatrolEvent.SIDE_SHIFT_DONE,
        PatrolEvent.TURN_DONE,
        PatrolEvent.LANE_END,
        None,
        PatrolEvent.SIDE_SHIFT_DONE,
        PatrolEvent.TURN_DONE,
    ]

    for index, event in enumerate(sequence):
        now = index / 10.0
        event_label = "tick"
        if event is not None:
            fsm.handle_event(event)
            event_label = event.value
        state = fsm.state()
        context = fsm.context()
        cmd = fsm.tick(now)
        rows.append(
            ",".join(
                [
                    f"{now:.1f}",
                    state.value,
                    str(context.lane_index),
                    str(context.direction),
                    f"{cmd.vx:.3f}",
                    f"{cmd.vy:.3f}",
                    f"{cmd.wz:.3f}",
                    event_label,
                ]
            )
        )

    return "\n".join(rows)


def main() -> int:
    print(run_dry_run())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
