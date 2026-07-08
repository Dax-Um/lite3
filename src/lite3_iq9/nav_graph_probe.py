"""Read-only Nav2 graph readiness evaluation for IQ9."""

from __future__ import annotations

from dataclasses import dataclass


DEFAULT_REQUIRED_NODES = frozenset(
    {
        "/hdl_localization",
        "/planner_server",
        "/controller_server",
        "/global_costmap/global_costmap",
        "/local_costmap/local_costmap",
        "/waypoint_follower",
    }
)
DEFAULT_REQUIRED_TOPICS = frozenset(
    {
        "/odom",
        "/status",
        "/map",
        "/global_costmap/costmap",
        "/local_costmap/costmap",
        "/cmd_vel",
    }
)
DEFAULT_REQUIRED_ACTIONS = frozenset({"/FollowWaypoints", "/navigate_to_pose"})


@dataclass(frozen=True)
class NavGraphSnapshot:
    nodes: set[str]
    topics: set[str]
    actions: set[str]
    cmd_vel_publishers: set[str]
    cmd_vel_subscribers: set[str]


@dataclass(frozen=True)
class NavGraphReport:
    ok: bool
    missing_nodes: list[str]
    missing_topics: list[str]
    missing_actions: list[str]
    cmd_vel_ready: bool
    reasons: list[str]


class NavGraphProbe:
    def __init__(
        self,
        required_nodes: set[str] | None = None,
        required_topics: set[str] | None = None,
        required_actions: set[str] | None = None,
    ) -> None:
        self.required_nodes = set(required_nodes or DEFAULT_REQUIRED_NODES)
        self.required_topics = set(required_topics or DEFAULT_REQUIRED_TOPICS)
        self.required_actions = set(required_actions or DEFAULT_REQUIRED_ACTIONS)

    def evaluate(self, snapshot: NavGraphSnapshot) -> NavGraphReport:
        missing_nodes = sorted(self.required_nodes - snapshot.nodes)
        missing_topics = sorted(self.required_topics - snapshot.topics)
        missing_actions = sorted(self.required_actions - snapshot.actions)

        has_controller = bool(snapshot.cmd_vel_publishers & {"controller_server", "recoveries_server"})
        has_motion_sender = "motion_sender" in snapshot.cmd_vel_subscribers
        cmd_vel_ready = has_controller and has_motion_sender

        reasons: list[str] = []
        reasons.extend(f"missing node {name}" for name in missing_nodes)
        reasons.extend(f"missing topic {name}" for name in missing_topics)
        reasons.extend(f"missing action {name}" for name in missing_actions)
        if not has_controller:
            reasons.append("controller_server is not publishing /cmd_vel")
        if not has_motion_sender:
            reasons.append("motion_sender is not subscribed to /cmd_vel")

        return NavGraphReport(
            ok=not missing_nodes and not missing_topics and not missing_actions and cmd_vel_ready,
            missing_nodes=missing_nodes,
            missing_topics=missing_topics,
            missing_actions=missing_actions,
            cmd_vel_ready=cmd_vel_ready,
            reasons=reasons,
        )

