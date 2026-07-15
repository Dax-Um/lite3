"""Top-level patrol behavior controller."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot, pi

from lite3_behavior.command_arbiter import ArbiterInput, CommandArbiter
from lite3_behavior.patrol_events import PatrolEvent, PatrolState
from lite3_behavior.patrol_fsm import PatrolContext, PatrolFSM
from lite3_common.types import Pose2D, StopReason, Twist2D
from lite3_control.safety_filter import SafetyConfig, SafetyFilter
from lite3_navigation.odom_tracker import OdomTracker
from lite3_navigation.return_home import ReturnHomeConfig, ReturnHomeController, normalize_angle
from lite3_perception.lidar_boundary_detector import (
    BoundaryConfig,
    BoundaryResult,
    LidarBoundaryDetector,
)


ZERO_TWIST = Twist2D(0.0, 0.0, 0.0)


@dataclass(frozen=True)
class ControllerOutput:
    raw_cmd: Twist2D
    safe_cmd: Twist2D
    state: str
    stop_reason: StopReason
    lane_index: int
    return_home_active: bool
    boundary_min_front_m: float | None


class PatrolController:
    def __init__(
        self,
        *,
        fsm: PatrolFSM | None = None,
        odom_tracker: OdomTracker | None = None,
        return_home: ReturnHomeController | None = None,
        boundary_detector: LidarBoundaryDetector | None = None,
        safety_filter: SafetyFilter | None = None,
        arbiter: CommandArbiter | None = None,
    ):
        self.fsm = fsm or PatrolFSM()
        self.odom_tracker = odom_tracker or OdomTracker()
        self.return_home = return_home or ReturnHomeController(ReturnHomeConfig())
        self.boundary_detector = boundary_detector or LidarBoundaryDetector(
            BoundaryConfig(confirm_frames=1)
        )
        self.safety_filter = safety_filter or SafetyFilter(SafetyConfig())
        self.arbiter = arbiter or CommandArbiter()
        self.current_pose: Pose2D | None = None
        self.boundary_result: BoundaryResult | None = None
        self._boundary_lane_end_consumed = False
        self._shift_start_pose: Pose2D | None = None
        self._turn_start_yaw: float | None = None
        self._emergency_stop = False

    def on_operator_command(self, command: str, now: float) -> None:
        if command == "patrol_start":
            if self.current_pose is None:
                raise RuntimeError("patrol_start requires current odom pose")
            self.odom_tracker.start_session(self.current_pose, now)
            self.fsm.handle_event(PatrolEvent.PATROL_START)
            return
        if command == "return_home":
            home = self.odom_tracker.home_pose()
            if home is None:
                raise RuntimeError("return_home requires home pose")
            self.return_home.start(home, self.odom_tracker.path_trace())
            self.fsm.handle_event(PatrolEvent.RETURN_HOME)
            return
        if command == "emergency_stop":
            self._emergency_stop = True
            self.safety_filter.set_emergency_stop(True)
            self.fsm.handle_event(PatrolEvent.EMERGENCY_STOP)
            return
        if command == "reset":
            self._emergency_stop = False
            self._boundary_lane_end_consumed = False
            self._shift_start_pose = None
            self._turn_start_yaw = None
            self.safety_filter.set_emergency_stop(False)
            self.return_home.cancel()
            self.fsm.handle_event(PatrolEvent.RESET)
            return
        raise ValueError(f"unsupported operator command: {command}")

    def on_odom(self, pose: Pose2D, now: float) -> None:
        self.current_pose = pose
        self.odom_tracker.sample(pose, now)
        self.safety_filter.update_odom(now)

    def on_imu(self, now: float) -> None:
        self.safety_filter.update_imu(now)

    def on_scan(
        self,
        ranges,
        angle_min: float,
        angle_increment: float,
        now: float,
    ) -> None:
        self.boundary_result = self.boundary_detector.update_scan(
            list(ranges),
            angle_min,
            angle_increment,
        )
        if not self.boundary_result.lane_end:
            self._boundary_lane_end_consumed = False
        self.safety_filter.update_lidar(now)

    def tick(self, now: float) -> ControllerOutput:
        self._apply_boundary()
        self._apply_progress_events()
        self.safety_filter.set_front_obstacle(
            self.boundary_result.should_stop if self.boundary_result is not None else False
        )

        return_home_cmd = ZERO_TWIST
        return_home_active = self.return_home.active()
        if return_home_active:
            if self.fsm.state() is PatrolState.PAUSE_AND_RETURN_HOME:
                self.fsm.tick(now)
            if self.current_pose is None:
                return_home_cmd = ZERO_TWIST
            else:
                return_home_cmd, done = self.return_home.tick(self.current_pose)
                if done:
                    self.fsm.handle_event(PatrolEvent.RETURN_DONE)
                    return_home_active = False

        patrol_cmd = ZERO_TWIST if return_home_active else self.fsm.tick(now)
        raw_cmd = self.arbiter.select(
            ArbiterInput(
                emergency_stop=self._emergency_stop,
                return_home_active=return_home_active,
                return_home_cmd=return_home_cmd,
                manual_active=False,
                manual_cmd=ZERO_TWIST,
                patrol_cmd=patrol_cmd,
            )
        )

        self.safety_filter.mark_command(now)
        safe_cmd, stop_reason = self.safety_filter.filter_cmd(raw_cmd, now)
        context = self.fsm.context()
        return ControllerOutput(
            raw_cmd=raw_cmd,
            safe_cmd=safe_cmd,
            state=self.fsm.state().value,
            stop_reason=stop_reason,
            lane_index=context.lane_index,
            return_home_active=return_home_active,
            boundary_min_front_m=(
                self.boundary_result.min_front_distance_m
                if self.boundary_result is not None
                else None
            ),
        )

    def _apply_boundary(self) -> None:
        if self.boundary_result is None:
            return
        if self.boundary_result.lane_end and not self._boundary_lane_end_consumed:
            self.fsm.handle_event(PatrolEvent.LANE_END)
            self._boundary_lane_end_consumed = True

    def _apply_progress_events(self) -> None:
        state = self.fsm.state()
        if state is not PatrolState.SHIFT_TO_NEXT_LANE:
            self._shift_start_pose = None
        if state is not PatrolState.TURN_AROUND:
            self._turn_start_yaw = None
        if self.current_pose is None:
            return

        if state is PatrolState.SHIFT_TO_NEXT_LANE:
            if self._shift_start_pose is None:
                self._shift_start_pose = self.current_pose
            elif _distance(self._shift_start_pose, self.current_pose) >= self.fsm.context().lane_spacing_m:
                self.fsm.handle_event(PatrolEvent.SIDE_SHIFT_DONE)
                self._shift_start_pose = None

        if self.fsm.state() is PatrolState.TURN_AROUND:
            if self._turn_start_yaw is None:
                self._turn_start_yaw = self.current_pose.yaw
            elif abs(normalize_angle(self.current_pose.yaw - self._turn_start_yaw)) >= pi - 0.17:
                self.fsm.handle_event(PatrolEvent.TURN_DONE)
                self._turn_start_yaw = None


def _distance(a: Pose2D, b: Pose2D) -> float:
    return hypot(a.x - b.x, a.y - b.y)
