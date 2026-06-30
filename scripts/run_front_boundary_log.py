#!/usr/bin/env python3
"""Format front-boundary detector results as CSV rows."""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from lite3_perception.lidar_boundary_detector import BoundaryResult


HEADER = "timestamp,min_front_distance,valid_points,should_slow,should_stop,lane_end"


def format_boundary_row(timestamp: float, result: BoundaryResult) -> str:
    distance = (
        ""
        if result.min_front_distance_m is None
        else f"{result.min_front_distance_m:.3f}"
    )
    return ",".join(
        [
            f"{timestamp:.3f}",
            distance,
            str(result.valid_front_points),
            _bool_text(result.should_slow),
            _bool_text(result.should_stop),
            _bool_text(result.lane_end),
        ]
    )


def _bool_text(value: bool) -> str:
    return "true" if value else "false"


def main() -> int:
    print(HEADER)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
