"""Nav2 waypoint action dry-run helpers.

This module intentionally does not import rclpy. It builds serializable goal
payloads that unit tests and scripts can validate before any live action send.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from lite3_iq9.waypoint_route import WaypointRoute


@dataclass(frozen=True)
class PoseStampedDraft:
    frame_id: str
    position: dict[str, float]
    orientation: dict[str, float]


@dataclass(frozen=True)
class WaypointGoalPlan:
    action_name: str
    ready: bool
    would_send: bool
    reason: str
    poses: list[PoseStampedDraft]


class Nav2WaypointDryRunClient:
    def __init__(self, available_actions: set[str], action_name: str = "/FollowWaypoints") -> None:
        self.available_actions = set(available_actions)
        self.action_name = action_name

    def build_goal(self, route: WaypointRoute) -> WaypointGoalPlan:
        poses = [
            PoseStampedDraft(
                frame_id=route.frame_id,
                position={"x": waypoint.x, "y": waypoint.y, "z": 0.0},
                orientation=_yaw_to_quaternion(waypoint.yaw),
            )
            for waypoint in route.waypoints
        ]
        if self.action_name not in self.available_actions:
            return WaypointGoalPlan(
                action_name=self.action_name,
                ready=False,
                would_send=False,
                reason=f"{self.action_name} action is not available",
                poses=poses,
            )
        return WaypointGoalPlan(
            action_name=self.action_name,
            ready=True,
            would_send=False,
            reason="dry-run only; no action goal sent",
            poses=poses,
        )


def _yaw_to_quaternion(yaw: float) -> dict[str, float]:
    half_yaw = yaw / 2.0
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(half_yaw),
        "w": math.cos(half_yaw),
    }

