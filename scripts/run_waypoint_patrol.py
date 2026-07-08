#!/usr/bin/env python3
"""Run IQ9 waypoint patrol planning and guarded Nav2 sending."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from lite3_iq9.nav2_waypoint_client import Nav2WaypointDryRunClient  # noqa: E402
from lite3_iq9.waypoint_patrol import WaypointPatrolConfig, WaypointPatrolPlanner  # noqa: E402
from lite3_iq9.waypoint_route import Waypoint, WaypointRoute  # noqa: E402


DEFAULT_PATROL_CONFIG = (
    Path(__file__).resolve().parents[1] / "configs" / "routes" / "default_waypoint_patrol.yaml"
)
DEFAULT_STATE_FILE = "/tmp/lite3_waypoint_patrol_state.json"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="IQ9 waypoint patrol start/stop entrypoint.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    start = subparsers.add_parser("start")
    _add_mode_args(start)
    start.add_argument("--home", nargs=3, metavar=("X", "Y", "YAW"))
    start.add_argument("--route-yaml")
    start.add_argument("--odom-topic", default="/odom")
    start.add_argument("--pose-timeout-sec", type=float, default=5.0)
    _add_common_args(start)

    stop = subparsers.add_parser("stop")
    _add_mode_args(stop)
    stop.add_argument("--current", nargs=3, metavar=("X", "Y", "YAW"))
    stop.add_argument("--odom-topic", default="/odom")
    stop.add_argument("--pose-timeout-sec", type=float, default=5.0)
    _add_common_args(stop)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.execute and not args.allow_robot_motion:
        print(
            "refusing to send waypoint goal: --execute requires --allow-robot-motion",
            file=sys.stderr,
        )
        return 3

    config = WaypointPatrolConfig.from_yaml(args.patrol_config)
    planner = WaypointPatrolPlanner(
        default_offsets=config.offsets,
        default_segments=config.segments,
        min_distance_m=config.min_distance_m,
        route_id=config.route_id,
        frame_id=config.frame_id,
    )

    if args.command == "start":
        home = _resolve_pose(args.home, waypoint_id="home", args=args)
        external_route = WaypointRoute.from_yaml(args.route_yaml) if args.route_yaml else None
        route = planner.start_patrol(home, route=external_route)
        _save_home(Path(args.state_file), home)
    else:
        home = _load_home(Path(args.state_file))
        current = _resolve_pose(args.current, waypoint_id="current", args=args)
        planner.start_patrol(home)
        route = planner.stop_patrol(current)

    if args.dry_run:
        plan = Nav2WaypointDryRunClient(
            available_actions=set(args.available_action),
            action_name=args.action_name,
        ).build_goal(route)
        print(json.dumps(_dry_run_payload(route, plan), indent=2, sort_keys=True))
        return 0 if plan.ready else 1

    from lite3_iq9.ros2_waypoint_sender import send_follow_waypoints

    result = send_follow_waypoints(
        route,
        action_name=args.action_name,
        timeout_sec=args.action_timeout_sec,
    )
    print(json.dumps({"mode": "execute", "route": _route_payload(route), "result": result}, indent=2))
    return 0 if result.get("accepted") else 1


def _add_mode_args(parser: argparse.ArgumentParser) -> None:
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--dry-run", action="store_true")
    mode.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--allow-robot-motion",
        action="store_true",
        help="Required with --execute because this sends a real FollowWaypoints goal.",
    )


def _add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--state-file", default=DEFAULT_STATE_FILE)
    parser.add_argument("--patrol-config", default=str(DEFAULT_PATROL_CONFIG))
    parser.add_argument("--action-name", default="/FollowWaypoints")
    parser.add_argument("--action-timeout-sec", type=float, default=10.0)
    parser.add_argument(
        "--available-action",
        action="append",
        default=[],
        help="Dry-run visible action. Repeatable.",
    )


def _resolve_pose(values: list[str] | None, *, waypoint_id: str, args: argparse.Namespace) -> Waypoint:
    if values is not None:
        return _waypoint_from_triplet(waypoint_id, values)
    if args.dry_run:
        raise SystemExit(f"--{waypoint_id if waypoint_id != 'home' else 'home'} is required in dry-run")
    from lite3_iq9.ros2_waypoint_sender import capture_current_pose

    return capture_current_pose(
        odom_topic=args.odom_topic,
        timeout_sec=args.pose_timeout_sec,
        waypoint_id=waypoint_id,
    )


def _waypoint_from_triplet(waypoint_id: str, values: list[str]) -> Waypoint:
    x, y, yaw = [float(value) for value in values]
    return Waypoint(id=waypoint_id, x=x, y=y, yaw=yaw, dwell_sec=0.0)


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


def _dry_run_payload(route: WaypointRoute, plan) -> dict:
    return {
        "mode": "dry-run",
        "route": _route_payload(route),
        "plan": {
            "action_name": plan.action_name,
            "ready": plan.ready,
            "would_send": plan.would_send,
            "reason": plan.reason,
            "poses": [asdict(pose) for pose in plan.poses],
        },
    }


def _route_payload(route: WaypointRoute) -> dict:
    return {
        "route_id": route.route_id,
        "frame_id": route.frame_id,
        "loop": route.loop,
        "waypoints": [asdict(waypoint) for waypoint in route.waypoints],
    }


if __name__ == "__main__":
    raise SystemExit(main())
