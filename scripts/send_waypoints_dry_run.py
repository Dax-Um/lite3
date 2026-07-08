#!/usr/bin/env python3
"""Build a Nav2 FollowWaypoints goal without sending it."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from lite3_iq9.nav2_waypoint_client import Nav2WaypointDryRunClient
from lite3_iq9.waypoint_route import WaypointRoute


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("route_yaml")
    parser.add_argument(
        "--available-action",
        action="append",
        default=[],
        help="Action visible in the mocked graph. Repeatable.",
    )
    args = parser.parse_args()

    route = WaypointRoute.from_yaml(args.route_yaml)
    client = Nav2WaypointDryRunClient(available_actions=set(args.available_action))
    plan = client.build_goal(route)
    print(
        json.dumps(
            {
                "action_name": plan.action_name,
                "ready": plan.ready,
                "would_send": plan.would_send,
                "reason": plan.reason,
                "poses": [
                    {
                        "frame_id": pose.frame_id,
                        "position": pose.position,
                        "orientation": pose.orientation,
                    }
                    for pose in plan.poses
                ],
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if plan.ready else 1


if __name__ == "__main__":
    raise SystemExit(main())
