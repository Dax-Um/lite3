#!/usr/bin/env python3
"""Select and print the MQTT triangle route without sending a motion goal."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
sys.path.insert(0, str(SRC))

from lite3_mqtt.patrol import (  # noqa: E402
    Nav2PatrolBackend,
    PatrolConfig,
    _validate_route,
)


DEFAULT_PATROL_CONFIG = ROOT / "configs" / "routes" / "mqtt_triangle_patrol.yaml"


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--patrol-config", default=str(DEFAULT_PATROL_CONFIG))
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--action-name", default="/navigate_to_pose")
    parser.add_argument("--timeout-sec", type=float, default=30.0)
    parser.add_argument("--route-clearance-m", type=float, default=0.50)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    backend = Nav2PatrolBackend(
        odom_topic=args.odom_topic,
        action_name=args.action_name,
        timeout_sec=args.timeout_sec,
        route_clearance_m=args.route_clearance_m,
    )
    config = PatrolConfig.from_yaml(args.patrol_config)

    backend.prepare_route()
    backend.wait_until_ready(timeout_sec=args.timeout_sec)
    home = backend.capture_current_pose(waypoint_id="home")
    failures = []
    selected = None
    selected_index = None
    for index, route in enumerate(config.build_candidate_routes(home)):
        _validate_route(route)
        try:
            backend.validate_route(route, start=home)
        except ValueError as exc:
            failures.append(str(exc))
            continue
        selected = route
        selected_index = index
        break
    if selected is None:
        print(
            json.dumps(
                {"ready": False, "reason": "no_safe_route", "failures": failures},
                ensure_ascii=False,
            )
        )
        return 2

    print(
        json.dumps(
            {
                "ready": True,
                "motion_sent": False,
                "candidate_index": selected_index,
                "home": _waypoint_json(home),
                "waypoints": [_waypoint_json(item) for item in selected.waypoints],
                "rejected_candidates": failures,
            },
            ensure_ascii=False,
        )
    )
    return 0


def _waypoint_json(waypoint):
    return {
        "id": waypoint.id,
        "x": waypoint.x,
        "y": waypoint.y,
        "yaw": waypoint.yaw,
    }


if __name__ == "__main__":
    raise SystemExit(main())
