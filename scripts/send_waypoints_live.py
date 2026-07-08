#!/usr/bin/env python3
"""Guarded placeholder for future live Nav2 waypoint sending.

This script intentionally does not send an action goal yet. It exists so field
operators have a named entrypoint that refuses by default until live motion is
implemented and approved.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from lite3_iq9.nav2_waypoint_client import Nav2WaypointDryRunClient  # noqa: E402
from lite3_iq9.waypoint_route import WaypointRoute  # noqa: E402


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Guarded live waypoint entrypoint. Does not send robot motion yet."
    )
    parser.add_argument("route_yaml")
    parser.add_argument(
        "--mock-approved",
        action="store_true",
        help="Build and print the goal payload without sending it.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if not args.mock_approved:
        print(
            "refusing live waypoint send: operator approval and ROS2 action sender are not enabled",
            file=sys.stderr,
        )
        return 2

    route = WaypointRoute.from_yaml(Path(args.route_yaml))
    plan = Nav2WaypointDryRunClient(available_actions={"/FollowWaypoints"}).build_goal(route)
    print(json.dumps(asdict(plan), indent=2, sort_keys=True))
    return 0 if plan.ready else 1


if __name__ == "__main__":
    raise SystemExit(main())
