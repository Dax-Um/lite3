#!/usr/bin/env python3
"""Build IQ9 waypoint patrol routes without sending robot motion."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from lite3_iq9.nav2_waypoint_client import Nav2WaypointDryRunClient  # noqa: E402
from lite3_iq9.waypoint_patrol import PatrolOffset, WaypointPatrolConfig, WaypointPatrolPlanner  # noqa: E402
from lite3_iq9.waypoint_route import Waypoint, WaypointRoute  # noqa: E402


DEFAULT_PATROL_CONFIG = (
    Path(__file__).resolve().parents[1] / "configs" / "routes" / "default_waypoint_patrol.yaml"
)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build default, external, or return-home waypoint patrol plans."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    start = subparsers.add_parser("start", help="Build a patrol route from home pose.")
    _add_home_args(start)
    start.add_argument("--route-yaml", help="Use an external waypoint route YAML instead of defaults.")
    start.add_argument(
        "--patrol-config",
        default=str(DEFAULT_PATROL_CONFIG),
        help="YAML file containing default patrol relative offsets.",
    )
    start.add_argument(
        "--state-file",
        default="/tmp/lite3_waypoint_patrol_state.json",
        help="File used to persist the captured home pose for stop/return-home.",
    )
    start.add_argument(
        "--offset",
        action="append",
        nargs="+",
        metavar="VALUE",
        help=(
            "Default patrol relative offset. Use 'DX DY' or "
            "'DX DY YAW_OFFSET [DWELL_SEC]'. Repeatable."
        ),
    )
    _add_common_args(start)

    stop = subparsers.add_parser("stop", help="Build a return-home route.")
    stop.add_argument("--home", nargs=3, metavar=("X", "Y", "YAW"))
    stop.add_argument("--current", nargs=3, required=True, metavar=("X", "Y", "YAW"))
    stop.add_argument(
        "--patrol-config",
        default=str(DEFAULT_PATROL_CONFIG),
        help="YAML file containing default patrol settings.",
    )
    stop.add_argument(
        "--state-file",
        default="/tmp/lite3_waypoint_patrol_state.json",
        help="File containing the home pose saved by start.",
    )
    _add_common_args(stop)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    home = _resolve_home(args)
    config = WaypointPatrolConfig.from_yaml(args.patrol_config)
    offsets = _parse_offsets(getattr(args, "offset", None)) or config.offsets
    planner = WaypointPatrolPlanner(
        default_offsets=offsets,
        default_segments=[] if offsets else config.segments,
        min_distance_m=config.min_distance_m,
        route_id=config.route_id,
        frame_id=config.frame_id,
    )

    if args.command == "start":
        external_route = WaypointRoute.from_yaml(args.route_yaml) if args.route_yaml else None
        route = planner.start_patrol(home, route=external_route)
        _save_home(Path(args.state_file), home)
    else:
        planner.start_patrol(home)
        route = planner.stop_patrol(_waypoint_from_triplet("current", args.current))

    plan = Nav2WaypointDryRunClient(
        available_actions=set(args.available_action),
        action_name=args.action_name,
    ).build_goal(route)
    print(json.dumps(_payload(route, plan), indent=2, sort_keys=True))
    return 0 if plan.ready else 1


def _add_home_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--home", nargs=3, required=True, metavar=("X", "Y", "YAW"))


def _add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--available-action",
        action="append",
        default=[],
        help="Action visible in the mocked graph. Repeatable.",
    )
    parser.add_argument("--action-name", default="/FollowWaypoints")


def _parse_offsets(raw_offsets: list[list[str]] | None) -> list[PatrolOffset]:
    if not raw_offsets:
        return []
    offsets: list[PatrolOffset] = []
    for index, raw in enumerate(raw_offsets, start=1):
        if len(raw) not in (2, 3, 4):
            raise SystemExit(f"--offset #{index} must have 2, 3, or 4 numeric values")
        values = [float(value) for value in raw]
        dx = values[0]
        dy = values[1]
        yaw_offset = values[2] if len(values) >= 3 else 0.0
        dwell_sec = values[3] if len(values) >= 4 else 0.0
        offsets.append(PatrolOffset(dx=dx, dy=dy, yaw_offset=yaw_offset, dwell_sec=dwell_sec))
    return offsets


def _waypoint_from_triplet(waypoint_id: str, values: list[str]) -> Waypoint:
    x, y, yaw = [float(value) for value in values]
    return Waypoint(id=waypoint_id, x=x, y=y, yaw=yaw, dwell_sec=0.0)


def _resolve_home(args: argparse.Namespace) -> Waypoint:
    if args.home is not None:
        return _waypoint_from_triplet("home", args.home)
    return _load_home(Path(args.state_file))


def _save_home(path: Path, home: Waypoint) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(asdict(home), indent=2, sort_keys=True), encoding="utf-8")


def _load_home(path: Path) -> Waypoint:
    if not path.exists():
        raise SystemExit(f"home state file does not exist: {path}")
    data = json.loads(path.read_text(encoding="utf-8"))
    return Waypoint(
        id="home",
        x=float(data["x"]),
        y=float(data["y"]),
        yaw=float(data["yaw"]),
        dwell_sec=float(data.get("dwell_sec", 0.0)),
    )


def _payload(route: WaypointRoute, plan) -> dict:
    return {
        "route": {
            "route_id": route.route_id,
            "frame_id": route.frame_id,
            "loop": route.loop,
            "waypoints": [asdict(waypoint) for waypoint in route.waypoints],
        },
        "plan": {
            "action_name": plan.action_name,
            "ready": plan.ready,
            "would_send": plan.would_send,
            "reason": plan.reason,
            "poses": [asdict(pose) for pose in plan.poses],
        },
    }


if __name__ == "__main__":
    raise SystemExit(main())
