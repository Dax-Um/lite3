"""ROS-facing coyote status and durable media bridge core."""

from __future__ import annotations

import hashlib
import json
import logging
import math
import os
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Deque, Dict, Iterable, Optional, Protocol, Set, Tuple, Union

from lite3_mqtt.contract import DetectionType, PatrolAction, decode_object
from lite3_motion.target_tracking import TargetTrackingPolicy
from lite3_motion.local_avoidance import (
    AvoidanceConfig,
    ClearanceSnapshot,
    LocalAvoidancePolicy,
)
from lite3_perception.coyote_spool import validate_event_id


COYOTE_STATUS_TIMEOUT_SEC = 1.0
# Target pursuit profile. Keep these separate from search and obstacle
# avoidance values so a tracking-speed change never changes safety behavior.
COYOTE_FORWARD_SPEED_MPS = 2.00
COYOTE_SEARCH_ADVANCE_SPEED_MPS = 0.50
COYOTE_CONTROL_HZ = 20.0
COYOTE_TURN_SPEED_RADPS = 1.45  # Target left/right alignment toward center.
COYOTE_SEARCH_TURN_SPEED_RADPS = 1.45  # Search/reacquisition only.
# High target-alignment speed with small, feedback-friendly corrections.
# Kept separate from the existing search/avoidance turn cadence.
COYOTE_TARGET_ALIGN_TURN_STEP_SEC = 0.12
COYOTE_TARGET_ALIGN_TURN_PAUSE_SEC = 0.05
COYOTE_TURN_STEP_SEC = 0.25
COYOTE_TURN_PAUSE_SEC = 0.10
COYOTE_SEARCH_SWEEP_RAD = 2.0 * math.pi
COYOTE_SEARCH_ADVANCE_M = 1.00
COYOTE_REPOSITION_RESCAN_CLEARANCE_M = 0.75
COYOTE_REPOSITION_MIN_ADVANCE_M = 0.20
COYOTE_SENSOR_TIMEOUT_SEC = 0.50
COYOTE_TURN_CLEARANCE_M = 0.45
COYOTE_FORWARD_CLEARANCE_M = 0.70
COYOTE_PROGRESS_TIMEOUT_SEC = 2.0
COYOTE_ALIGN_TIMEOUT_SEC = 15.0
COYOTE_SEARCH_TURN_TIMEOUT_SEC = 90.0
COYOTE_SEARCH_ADVANCE_TIMEOUT_SEC = 20.0
COYOTE_TRACK_FORWARD_TIMEOUT_SEC = 60.0
COYOTE_SEARCH_SESSION_TIMEOUT_SEC = 180.0
COYOTE_PROGRESS_DISTANCE_M = 0.02
COYOTE_PROGRESS_YAW_RAD = 0.03
COYOTE_ADVANCE_LATERAL_TOLERANCE_M = 0.05
COYOTE_ADVANCE_YAW_TOLERANCE_RAD = 0.10


@dataclass(frozen=True)
class CoyoteStatus:
    timestamp_sec: float
    detect: str
    motion: str
    side: str
    height_ratio: float = 0.0
    long_jump_ready: bool = False


def parse_coyote_status(payload: Union[bytes, str]) -> CoyoteStatus:
    value = decode_object(payload)
    timestamp = value.get("ts")
    if isinstance(timestamp, bool) or not isinstance(timestamp, (int, float)):
        raise ValueError("coyote ts must be a Unix-seconds JSON number")
    timestamp = float(timestamp)
    if not math.isfinite(timestamp) or timestamp <= 0.0:
        raise ValueError("coyote ts must be finite and positive")

    detect = _enum_string(value, "detect", ("detected", "not_detected"))
    motion = _enum_string(value, "motion", ("forward", "stop"))
    side = _enum_string(value, "side", ("center", "left", "right", "none"))
    if detect == "not_detected" and (motion != "stop" or side != "none"):
        raise ValueError("not_detected status must use motion=stop and side=none")
    if detect == "detected" and motion == "forward" and side != "center":
        raise ValueError("detected forward status must use side=center")
    if detect == "detected" and motion == "stop" and side == "none":
        raise ValueError("detected stop status must identify center, left, or right")
    height_ratio = value.get("height_ratio", 0.0)
    if isinstance(height_ratio, bool) or not isinstance(height_ratio, (int, float)):
        raise ValueError("coyote height_ratio must be a JSON number")
    height_ratio = float(height_ratio)
    if not math.isfinite(height_ratio) or not 0.0 <= height_ratio <= 1.0:
        raise ValueError("coyote height_ratio must be in [0, 1]")
    long_jump_ready = value.get("long_jump_ready", False)
    if not isinstance(long_jump_ready, bool):
        raise ValueError("coyote long_jump_ready must be a boolean")
    return CoyoteStatus(
        timestamp_sec=timestamp,
        detect=detect,
        motion=motion,
        side=side,
        height_ratio=height_ratio,
        long_jump_ready=long_jump_ready,
    )


class MotionSink(Protocol):
    def acquire(self) -> None:
        ...

    def send_cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        ...

    def release(self) -> None:
        ...


class CoyoteMotionController:
    """Execute the four-field status contract with a bounded search sequence."""

    def __init__(
        self,
        motion_sink: MotionSink,
        *,
        forward_speed_mps: float = COYOTE_FORWARD_SPEED_MPS,
        turn_speed_radps: float = COYOTE_TURN_SPEED_RADPS,
        search_turn_speed_radps: float = COYOTE_SEARCH_TURN_SPEED_RADPS,
        turn_step_sec: float = COYOTE_TURN_STEP_SEC,
        turn_pause_sec: float = COYOTE_TURN_PAUSE_SEC,
        target_align_turn_step_sec: float = COYOTE_TARGET_ALIGN_TURN_STEP_SEC,
        target_align_turn_pause_sec: float = COYOTE_TARGET_ALIGN_TURN_PAUSE_SEC,
        search_sweep_rad: float = COYOTE_SEARCH_SWEEP_RAD,
        search_advance_m: float = COYOTE_SEARCH_ADVANCE_M,
        search_advance_speed_mps: float = COYOTE_SEARCH_ADVANCE_SPEED_MPS,
        sensor_timeout_sec: float = COYOTE_SENSOR_TIMEOUT_SEC,
        turn_clearance_m: float = COYOTE_TURN_CLEARANCE_M,
        forward_clearance_m: float = COYOTE_FORWARD_CLEARANCE_M,
        progress_timeout_sec: float = COYOTE_PROGRESS_TIMEOUT_SEC,
        align_timeout_sec: float = COYOTE_ALIGN_TIMEOUT_SEC,
        search_turn_timeout_sec: float = COYOTE_SEARCH_TURN_TIMEOUT_SEC,
        search_advance_timeout_sec: float = COYOTE_SEARCH_ADVANCE_TIMEOUT_SEC,
        track_forward_timeout_sec: float = COYOTE_TRACK_FORWARD_TIMEOUT_SEC,
        search_session_timeout_sec: float = COYOTE_SEARCH_SESSION_TIMEOUT_SEC,
        progress_distance_m: float = COYOTE_PROGRESS_DISTANCE_M,
        progress_yaw_rad: float = COYOTE_PROGRESS_YAW_RAD,
        advance_lateral_tolerance_m: float = COYOTE_ADVANCE_LATERAL_TOLERANCE_M,
        advance_yaw_tolerance_rad: float = COYOTE_ADVANCE_YAW_TOLERANCE_RAD,
        require_scan: bool = True,
        local_avoidance_policy: Optional[LocalAvoidancePolicy] = None,
        timeout_sec: float = COYOTE_STATUS_TIMEOUT_SEC,
        wall_clock: Callable[[], float] = time.time,
        monotonic_clock: Callable[[], float] = time.monotonic,
        ready: Callable[[], bool] = lambda: True,
        on_coyote_complete: Optional[Callable[[str, str], None]] = None,
        on_broken_cup_complete: Optional[Callable[[str, str], None]] = None,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        if not math.isfinite(forward_speed_mps) or forward_speed_mps <= 0.0:
            raise ValueError("forward_speed_mps must be finite and positive")
        if not math.isfinite(turn_speed_radps) or turn_speed_radps <= 0.0:
            raise ValueError("turn_speed_radps must be finite and positive")
        if (
            not math.isfinite(search_turn_speed_radps)
            or search_turn_speed_radps <= 0.0
        ):
            raise ValueError(
                "search_turn_speed_radps must be finite and positive"
            )
        if min(
            turn_step_sec,
            turn_pause_sec,
            target_align_turn_step_sec,
            target_align_turn_pause_sec,
            search_sweep_rad,
        ) <= 0.0:
            raise ValueError("turn timing and search sweep must be positive")
        if not math.isfinite(search_advance_m) or search_advance_m <= 0.0:
            raise ValueError("search_advance_m must be finite and positive")
        if (
            not math.isfinite(search_advance_speed_mps)
            or search_advance_speed_mps <= 0.0
        ):
            raise ValueError("search_advance_speed_mps must be finite and positive")
        if min(sensor_timeout_sec, turn_clearance_m, forward_clearance_m) <= 0.0:
            raise ValueError("sensor timeout and clearances must be positive")
        if min(
            progress_timeout_sec,
            align_timeout_sec,
            search_turn_timeout_sec,
            search_advance_timeout_sec,
            track_forward_timeout_sec,
            search_session_timeout_sec,
            progress_distance_m,
            progress_yaw_rad,
            advance_lateral_tolerance_m,
            advance_yaw_tolerance_rad,
        ) <= 0.0:
            raise ValueError("motion watchdog values must be positive")
        if not math.isfinite(timeout_sec) or timeout_sec <= 0.0:
            raise ValueError("timeout_sec must be finite and positive")
        self.motion_sink = motion_sink
        self.forward_speed_mps = forward_speed_mps
        self.turn_speed_radps = turn_speed_radps
        self.search_turn_speed_radps = search_turn_speed_radps
        self.turn_step_sec = turn_step_sec
        self.turn_pause_sec = turn_pause_sec
        self.target_align_turn_step_sec = target_align_turn_step_sec
        self.target_align_turn_pause_sec = target_align_turn_pause_sec
        self.search_sweep_rad = search_sweep_rad
        self.search_advance_m = search_advance_m
        self.search_advance_speed_mps = search_advance_speed_mps
        self.sensor_timeout_sec = sensor_timeout_sec
        self.turn_clearance_m = turn_clearance_m
        self.forward_clearance_m = forward_clearance_m
        self.progress_timeout_sec = progress_timeout_sec
        self.align_timeout_sec = align_timeout_sec
        self.search_turn_timeout_sec = search_turn_timeout_sec
        self.search_advance_timeout_sec = search_advance_timeout_sec
        self.track_forward_timeout_sec = track_forward_timeout_sec
        self.search_session_timeout_sec = search_session_timeout_sec
        self.progress_distance_m = progress_distance_m
        self.progress_yaw_rad = progress_yaw_rad
        self.advance_lateral_tolerance_m = advance_lateral_tolerance_m
        self.advance_yaw_tolerance_rad = advance_yaw_tolerance_rad
        self.require_scan = bool(require_scan)
        self.target_tracking_policy = TargetTrackingPolicy()
        self._reacquire_anchor = None
        self._target_reacquire_requested = False
        self._reacquire_observe_until = 0.0
        self._reacquire_sweep_commanded_rad = 0.0
        self._reacquire_sweep_limit_rad = math.pi / 2.0
        self._last_reacquired_observation_id = 0
        self.local_avoidance_policy = local_avoidance_policy or LocalAvoidancePolicy(
            AvoidanceConfig(
                hard_stop_m=0.32,
                # Local PointCloud detour uses body-scale thresholds, not the
                # older scan-gating defaults retained for compatibility.
                forward_clearance_m=0.50,
                turn_clearance_m=0.32,
            )
        )
        self.timeout_sec = timeout_sec
        self.wall_clock = wall_clock
        self.monotonic_clock = monotonic_clock
        self.ready = ready
        self.on_coyote_complete = on_coyote_complete
        self.on_broken_cup_complete = on_broken_cup_complete
        self.logger = logger or logging.getLogger(__name__)
        self._lock = threading.Lock()
        self._status = None  # type: Optional[CoyoteStatus]
        self._status_type = None  # type: Optional[DetectionType]
        self._synthetic_search_status = False
        self._received_at = None  # type: Optional[float]
        self._last_reason = "no_status"
        self._scan_received_at = None  # type: Optional[float]
        self._left_clearance_m = None  # type: Optional[float]
        self._right_clearance_m = None  # type: Optional[float]
        self._front_clearance_m = None  # type: Optional[float]
        self._clearance_snapshot = None  # type: Optional[ClearanceSnapshot]
        self._target_obstacle_matched = False
        self._odom_received_at = None  # type: Optional[float]
        self._odom_xy = None  # type: Optional[Tuple[float, float]]
        self._odom_yaw = None  # type: Optional[float]
        self._search_armed = False
        self._search_started_at = None  # type: Optional[float]
        self._active_detection_type = None  # type: Optional[DetectionType]
        self._active_event_id = None  # type: Optional[str]
        # Inference never acquires motion by itself. Each detection type is
        # terminal until its own MQTT trigger arms a new search.
        self._detection_state = {
            DetectionType.COYOTE: "complete",
            DetectionType.BROKEN_CUP: "complete",
        }
        self._emergency_latched = False
        self._operator_stop_latched = False
        self._external_action_hold = False
        self._search_phase = "idle"
        self._search_direction = -1
        self._sweep_angle_rad = 0.0
        self._sweep_commanded_rad = 0.0
        self._last_sweep_yaw = None  # type: Optional[float]
        self._search_turn_active = False
        self._advance_start_xy = None  # type: Optional[Tuple[float, float]]
        self._advance_start_yaw = None  # type: Optional[float]
        self._advance_completed = False
        self._reposition_target_yaw = None  # type: Optional[float]
        self._pulse_direction = 0
        self._pulse_active_until = 0.0
        self._pulse_pause_until = 0.0
        self._recent_events = deque(maxlen=256)  # type: Deque[str]
        self._recent_event_set = set()  # type: Set[str]
        self._recent_patrol_commands = deque(maxlen=64)  # type: Deque[Tuple[int, str]]
        self._recent_patrol_command_set = set()  # type: Set[Tuple[int, str]]
        self._last_patrol_timestamp = None  # type: Optional[int]
        self._motion_key = None  # type: Optional[Tuple[str, int]]
        self._motion_started_at = None  # type: Optional[float]
        self._motion_progress_at = None  # type: Optional[float]
        self._motion_progress_xy = None  # type: Optional[Tuple[float, float]]
        self._motion_progress_yaw = None  # type: Optional[float]

    @property
    def last_reason(self) -> str:
        with self._lock:
            return self._last_reason

    @property
    def emergency_latched(self) -> bool:
        with self._lock:
            return self._emergency_latched

    def handle_status(
        self,
        payload: Union[bytes, str],
        detection_type: DetectionType = DetectionType.COYOTE,
    ) -> CoyoteStatus:
        try:
            detection_type = DetectionType(detection_type)
        except ValueError as exc:
            raise ValueError("unsupported coyote status detection type") from exc
        try:
            status = parse_coyote_status(payload)
        except Exception:
            with self._lock:
                if (
                    self._search_armed
                    and self._active_detection_type is not detection_type
                ):
                    raise
                self._status = None
                self._status_type = None
                self._synthetic_search_status = False
                self._received_at = None
                self._search_armed = False
                self._search_started_at = None
                self._active_detection_type = None
                self._search_phase = "idle"
                self._search_turn_active = False
                self._reset_motion_guard_locked()
                self._last_reason = "invalid_status"
                self._send_stop()
                self._release_output_locked()
            raise

        now_wall = self.wall_clock()
        now_monotonic = self.monotonic_clock()
        reason = self._status_reason(status, now_wall, 0.0)
        with self._lock:
            if self._detection_state[detection_type] != "searching":
                self._last_reason = "{}_{}_ignored".format(
                    detection_type.value.lower(),
                    self._detection_state[detection_type],
                )
                self._send_stop()
                return status
            if (
                self._search_armed
                and self._active_detection_type is not detection_type
            ):
                return status
            self._status = status
            self._status_type = detection_type
            self._synthetic_search_status = False
            self._received_at = now_monotonic
            self.target_tracking_policy.observe(
                detect=status.detect,
                side=status.side,
                pose=((self._odom_xy[0], self._odom_xy[1], self._odom_yaw) if self._odom_xy is not None and self._odom_yaw is not None else None),
                now=now_monotonic,
            )
            self._last_reason = reason
            # A stop advisory is applied in this callback. Turning, if any,
            # starts on a later control tick so a zero command always separates
            # forward motion from rotation.
            if status.motion == "stop" or reason != "forward":
                self._send_stop()
        return status

    def start_search(
        self,
        event_id: str,
        detection_type: DetectionType = DetectionType.COYOTE,
    ) -> bool:
        if not isinstance(event_id, str) or not event_id or len(event_id) > 128:
            raise ValueError("search event_id must be a non-empty string up to 128 chars")
        try:
            detection_type = DetectionType(detection_type)
        except ValueError as exc:
            raise ValueError("unsupported search detection type") from exc
        with self._lock:
            if self._emergency_latched:
                self._last_reason = "emergency_latched"
                return False
            if event_id in self._recent_event_set:
                return False
            if len(self._recent_events) == self._recent_events.maxlen:
                evicted = self._recent_events.popleft()
                self._recent_event_set.discard(evicted)
            self._recent_events.append(event_id)
            self._recent_event_set.add(event_id)
            if self._search_armed:
                self._last_reason = "search_trigger_coalesced"
                return False
            self._operator_stop_latched = False
            self._external_action_hold = False
            self._detection_state[detection_type] = "searching"
            acquire = getattr(self.motion_sink, "acquire", None)
            if callable(acquire):
                acquire()
            self._search_armed = True
            self._active_detection_type = detection_type
            self._active_event_id = event_id
            now_wall = self.wall_clock()
            now_monotonic = self.monotonic_clock()
            self._search_started_at = now_monotonic
            status_is_fresh = (
                self._status is not None
                and self._status_type is detection_type
                and not self._synthetic_search_status
                and self._received_at is not None
                and abs(now_wall - self._status.timestamp_sec) <= self.timeout_sec
                and 0.0 <= now_monotonic - self._received_at <= self.timeout_sec
            )
            if not status_is_fresh:
                self._status = CoyoteStatus(
                    timestamp_sec=now_wall,
                    detect="not_detected",
                    motion="stop",
                    side="none",
                )
                self._status_type = detection_type
                self._received_at = now_monotonic
                self._synthetic_search_status = True
            else:
                self._synthetic_search_status = False
            self._advance_completed = False
            self.target_tracking_policy.clear()
            self._last_reacquired_observation_id = 0
            self._reacquire_anchor = None
            self._target_reacquire_requested = False
            self._reacquire_observe_until = 0.0
            self._reacquire_sweep_commanded_rad = 0.0
            self._begin_scan_locked(direction=-1, phase="primary_turn")
            self._last_reason = "search_triggered"
            self._send_stop()
        return True

    def handle_patrol_command(
        self,
        action: PatrolAction,
        timestamp: int,
    ) -> bool:
        """Apply the MQTT patrol command ordering rules to search ownership."""
        try:
            action = PatrolAction(action)
        except ValueError as exc:
            raise ValueError("unsupported patrol action") from exc
        if isinstance(timestamp, bool) or not isinstance(timestamp, int) or timestamp <= 0:
            raise ValueError("patrol timestamp must be a positive epoch-millisecond integer")

        key = (timestamp, action.value)
        with self._lock:
            if key in self._recent_patrol_command_set:
                return False
            if (
                action in {
                    PatrolAction.START,
                    PatrolAction.RETURN_HOME,
                    PatrolAction.RESET,
                }
                and self._last_patrol_timestamp is not None
                and timestamp <= self._last_patrol_timestamp
            ):
                self.logger.warning(
                    "out-of-order coyote patrol command ignored action=%s timestamp=%s last=%s",
                    action.value,
                    timestamp,
                    self._last_patrol_timestamp,
                )
                return False
            if len(self._recent_patrol_commands) == self._recent_patrol_commands.maxlen:
                evicted = self._recent_patrol_commands.popleft()
                self._recent_patrol_command_set.discard(evicted)
            self._recent_patrol_commands.append(key)
            self._recent_patrol_command_set.add(key)
            if self._last_patrol_timestamp is None:
                self._last_patrol_timestamp = timestamp
            else:
                self._last_patrol_timestamp = max(
                    self._last_patrol_timestamp,
                    timestamp,
                )

            if action is PatrolAction.EMERGENCY_STOP:
                self._emergency_latched = True
                self._operator_stop_latched = True
                self._stop_locked("emergency_stop")
            elif action is PatrolAction.RESET:
                self._stop_locked("reset")
                self._emergency_latched = False
                self._operator_stop_latched = False
            elif action is PatrolAction.START:
                self._operator_stop_latched = False
                self._stop_locked("patrol_start")
            else:
                self._operator_stop_latched = True
                self._stop_locked("patrol_{}".format(action.value.lower()))
        return True

    def update_scan(
        self,
        ranges: Iterable[float],
        angle_min: float,
        angle_increment: float,
    ) -> None:
        if not math.isfinite(angle_min) or not math.isfinite(angle_increment):
            raise ValueError("scan angles must be finite")
        if angle_increment == 0.0:
            raise ValueError("scan angle_increment must be non-zero")
        left = []
        right = []
        front = []
        for index, raw_range in enumerate(ranges):
            try:
                distance = float(raw_range)
            except (TypeError, ValueError):
                continue
            if not math.isfinite(distance) or distance <= 0.0:
                continue
            angle = _normalize_angle(angle_min + index * angle_increment)
            # Include the rear leading quadrant swept by a rectangular body
            # during each turn, not only the camera-facing side sector.
            if (
                math.radians(15.0) <= angle <= math.radians(135.0)
                or math.radians(-180.0) <= angle <= math.radians(-105.0)
            ):
                left.append(distance)
            if (
                math.radians(-135.0) <= angle <= math.radians(-15.0)
                or math.radians(105.0) <= angle <= math.radians(180.0)
            ):
                right.append(distance)
            if abs(angle) <= math.radians(20.0):
                front.append(distance)
        self.update_clearances(
            front_m=min(front) if front else None,
            left_m=min(left) if left else None,
            right_m=min(right) if right else None,
        )

    def update_clearances(
        self,
        *,
        front_m: Optional[float],
        left_m: Optional[float],
        right_m: Optional[float],
    ) -> None:
        """Update local obstacle ranges from either LaserScan or PointCloud2.

        This is deliberately sensor-agnostic: callbacks only update the latest
        snapshot, while ``tick`` remains the sole cmd_vel arbitration point.
        """
        values = (front_m, left_m, right_m)
        for value in values:
            if value is not None and (not math.isfinite(value) or value <= 0.0):
                raise ValueError("clearance values must be positive finite values or None")
        with self._lock:
            self._front_clearance_m = front_m
            self._left_clearance_m = left_m
            self._right_clearance_m = right_m
            self._scan_received_at = self.monotonic_clock()
            self._clearance_snapshot = ClearanceSnapshot(
                front_m=front_m,
                left_m=left_m,
                right_m=right_m,
                received_at_monotonic=self._scan_received_at,
            )

    def update_target_obstacle_match(self, matched: bool) -> None:
        """Set only by a future RGB/RealSense↔LiDAR association module."""
        with self._lock:
            self._target_obstacle_matched = bool(matched)

    def latest_clearance_snapshot(self) -> Optional[ClearanceSnapshot]:
        """Return callback state only; this method never writes motion."""
        with self._lock:
            return self._clearance_snapshot

    def update_odom(self, x: float, y: float, yaw: float) -> None:
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise ValueError("odom pose must be finite")
        now = self.monotonic_clock()
        yaw = _normalize_angle(yaw)
        with self._lock:
            if (
                self._search_phase in {"primary_turn", "secondary_turn"}
                and self._search_turn_active
            ):
                if self._last_sweep_yaw is not None:
                    delta = _normalize_angle(yaw - self._last_sweep_yaw)
                    self._sweep_angle_rad += max(
                        0.0,
                        delta * self._search_direction,
                    )
                self._last_sweep_yaw = yaw
            if self._motion_key is not None:
                kind = self._motion_key[0]
                progressed = False
                if kind in {"align", "search_turn"}:
                    if self._motion_progress_yaw is None:
                        self._motion_progress_yaw = yaw
                    elif abs(
                        _normalize_angle(yaw - self._motion_progress_yaw)
                    ) >= self.progress_yaw_rad:
                        self._motion_progress_yaw = yaw
                        progressed = True
                elif kind in {"search_advance", "track_forward"}:
                    if self._motion_progress_xy is None:
                        self._motion_progress_xy = (float(x), float(y))
                    elif math.hypot(
                        float(x) - self._motion_progress_xy[0],
                        float(y) - self._motion_progress_xy[1],
                    ) >= self.progress_distance_m:
                        self._motion_progress_xy = (float(x), float(y))
                        progressed = True
                if progressed:
                    self._motion_progress_at = now
            self._odom_xy = (float(x), float(y))
            self._odom_yaw = yaw
            self._odom_received_at = now

    def latest_motion_pose(self) -> Optional[Tuple[float, float, float]]:
        """Return the latest direct Motion Host pose without Nav2/TF coupling."""
        with self._lock:
            if self._odom_xy is None or self._odom_yaw is None:
                return None
            return (self._odom_xy[0], self._odom_xy[1], self._odom_yaw)

    def pause_for_external_action(self) -> Optional[str]:
        """Stop/release direct UDP while a one-shot motion action owns robot."""
        with self._lock:
            if not self._search_armed or not self._active_event_id:
                return None
            if self._external_action_hold:
                return None
            self._external_action_hold = True
            self._send_stop()
            self._release_output_locked()
            return self._active_event_id

    def resume_after_external_action(self, event_id: str) -> bool:
        with self._lock:
            if (
                not self._external_action_hold
                or not self._search_armed
                or self._active_event_id != event_id
            ):
                return False
            acquire = getattr(self.motion_sink, "acquire", None)
            if callable(acquire):
                acquire()
            self._external_action_hold = False
            self._send_stop()
            return True

    def tick(self) -> Tuple[float, float, float]:
        now_wall = self.wall_clock()
        now_monotonic = self.monotonic_clock()
        with self._lock:
            if self._external_action_hold:
                self._last_reason = "external_action_hold"
                return (0.0, 0.0, 0.0)
            if self._search_armed and self._synthetic_search_status:
                self._status = CoyoteStatus(
                    timestamp_sec=now_wall,
                    detect="not_detected",
                    motion="stop",
                    side="none",
                )
                self._received_at = now_monotonic
            status = self._status
            received_at = self._received_at
            receive_age = math.inf if received_at is None else now_monotonic - received_at
            reason = self._status_reason(status, now_wall, receive_age)
            if self._search_armed and reason == "stale_status":
                # Perception may publish only when its detection state changes.
                # A quiet interval means the target is not currently observed;
                # it must continue the already armed search, not cancel it.
                self._status = CoyoteStatus(
                    timestamp_sec=now_wall,
                    detect="not_detected",
                    motion="stop",
                    side="none",
                )
                self._synthetic_search_status = True
                self._received_at = now_monotonic
                status = self._status
                reason = "not_detected"
            if (
                self._search_armed
                and self._search_started_at is not None
                and now_monotonic - self._search_started_at
                > self.search_session_timeout_sec
            ):
                command, reason = self._complete_active_detection_locked(
                    outcome="NOT_FOUND",
                    reason="search_session_timeout",
                )
            elif reason in {"forward", "motion_stop"} and not self._search_armed:
                command = (0.0, 0.0, 0.0)
                reason = "detection_not_armed"
                self._search_turn_active = False
                self._reset_pulse_locked()
            elif reason == "forward":
                if not self._scan_fresh_locked(now_monotonic):
                    command = (0.0, 0.0, 0.0)
                    reason = "lidar_stale"
                elif not self._odom_fresh_locked(now_monotonic):
                    command = (0.0, 0.0, 0.0)
                    reason = "odom_stale"
                else:
                    command = (self.forward_speed_mps, 0.0, 0.0)
                    self._search_turn_active = False
                    self._reset_pulse_locked()
            elif reason == "motion_stop" and status is not None:
                command, reason = self._detected_stop_command_locked(
                    status,
                    now_monotonic,
                )
            elif reason == "not_detected" and self._search_armed:
                command, reason = self._search_command_locked(now_monotonic)
            else:
                command = (0.0, 0.0, 0.0)
                self._search_turn_active = False
                self._reset_pulse_locked()
                if reason in {"future_status", "stale_status"}:
                    self._search_armed = False
                    self._search_started_at = None
                    self._active_detection_type = None
                    self._search_phase = "idle"
            # One final priority arbitration point owns the command.  In the
            # current RGB-only demo no scan arrives and behavior is unchanged;
            # once the perception LiDAR bridge publishes a fresh scan, this
            # becomes active without introducing a second UDP writer.
            if reason in {
                "target_centered",
                "detection_not_armed",
                "emergency_stop",
                "stopped",
                "transport_not_ready",
                "future_status",
                "stale_status",
                "no_status",
            }:
                self.local_avoidance_policy.reset()
            if (
                self._clearance_snapshot is not None
                and self._scan_received_at is not None
                and 0.0 <= now_monotonic - self._scan_received_at
                <= self.sensor_timeout_sec
            ):
                base_reason = reason
                override = self.local_avoidance_policy.arbitrate(
                    command,
                    self._clearance_snapshot,
                    target_matched=self._target_obstacle_matched,
                )
                if override is not None:
                    command = (override.vx, override.vy, override.wz)
                    reason = override.reason
                    if (
                        base_reason in {"forward", "align_left", "align_right"}
                        and reason in {
                            "obstacle_avoid_left", "obstacle_avoid_right",
                            "obstacle_turn_left", "obstacle_turn_right",
                            "obstacle_bypass_left", "obstacle_bypass_right",
                        }
                        and self.target_tracking_policy.preferred_search_direction(
                            now=now_monotonic
                        ) is not None
                    ):
                        # A target pursuit was diverted. On loss, return to the
                        # last observation and face the target before scanning.
                        self._target_reacquire_requested = True
                    # The PointCloud policy may reverse the scan direction to
                    # an open side.  Count the yaw in the direction actually
                    # commanded, otherwise a blocked clockwise scan never
                    # reaches its 360-degree completion condition.
                    if (
                        self._search_phase in {"primary_turn", "secondary_turn"}
                        and reason in {
                            "obstacle_avoid_left",
                            "obstacle_avoid_right",
                            "obstacle_turn_left",
                            "obstacle_turn_right",
                        }
                        and abs(command[2]) > 1e-9
                    ):
                        self._search_direction = 1 if command[2] > 0.0 else -1
                        self._last_sweep_yaw = self._odom_yaw
                        self._search_turn_active = True
            command, reason = self._guard_motion_locked(
                command,
                reason,
                now_monotonic,
            )
            try:
                self.motion_sink.send_cmd_vel(*command)
            except Exception:
                try:
                    self._send_stop()
                finally:
                    raise
            self._last_reason = reason
            if (
                command == (0.0, 0.0, 0.0)
                and not self._search_armed
                and self._should_release_locked(reason)
            ):
                self._release_output_locked()
        return command

    def stop(self) -> None:
        with self._lock:
            self._operator_stop_latched = True
            self._stop_locked("stopped")

    def emergency_stop(self) -> None:
        with self._lock:
            self._emergency_latched = True
            self._operator_stop_latched = True
            self._stop_locked("emergency_stop")

    def reset(self) -> None:
        with self._lock:
            self._stop_locked("reset")
            self._emergency_latched = False
            self._operator_stop_latched = False

    def _stop_locked(self, reason: str) -> None:
        self._status = None
        self._status_type = None
        self._synthetic_search_status = False
        self._received_at = None
        self._search_armed = False
        self._search_started_at = None
        self._active_detection_type = None
        self._active_event_id = None
        self._reacquire_anchor = None
        self._target_reacquire_requested = False
        self._reacquire_observe_until = 0.0
        self._reacquire_sweep_commanded_rad = 0.0
        self._last_reacquired_observation_id = 0
        self.target_tracking_policy.clear()
        self._external_action_hold = False
        self._search_phase = "idle"
        self._search_turn_active = False
        self._reset_pulse_locked()
        self._reset_motion_guard_locked()
        self._last_reason = reason
        self._send_stop()
        self._release_output_locked()

    def _status_reason(
        self,
        status: Optional[CoyoteStatus],
        now_wall: float,
        receive_age: float,
    ) -> str:
        try:
            is_ready = bool(self.ready())
        except Exception:
            is_ready = False
        if not is_ready:
            return "transport_not_ready"
        if status is None:
            return "no_status"
        payload_age = now_wall - status.timestamp_sec
        if payload_age < -self.timeout_sec:
            return "future_status"
        if payload_age > self.timeout_sec or receive_age > self.timeout_sec:
            return "stale_status"
        if status.detect != "detected":
            return "not_detected"
        if status.motion != "forward":
            return "motion_stop"
        return "forward"

    def _detected_stop_command_locked(
        self,
        status: CoyoteStatus,
        now: float,
    ) -> Tuple[Tuple[float, float, float], str]:
        if status.side == "center":
            return self._complete_active_detection_locked(
                outcome="TARGET_REACHED",
                reason="target_centered",
            )
        direction = 1 if status.side == "left" else -1
        self._search_turn_active = False
        if not self._scan_fresh_locked(now):
            return (0.0, 0.0, 0.0), "lidar_stale"
        if not self._odom_fresh_locked(now):
            return (0.0, 0.0, 0.0), "odom_stale"
        command = self._pulse_turn_locked(
            direction,
            now,
            speed_radps=self.turn_speed_radps,
            step_sec=self.target_align_turn_step_sec,
            pause_sec=self.target_align_turn_pause_sec,
        )
        return command, "align_left" if direction > 0 else "align_right"

    def _search_command_locked(
        self,
        now: float,
    ) -> Tuple[Tuple[float, float, float], str]:
        if not self._odom_fresh_locked(now):
            return (0.0, 0.0, 0.0), "odom_stale"
        # A normal target pursuit keeps the scan phase armed.  Therefore this
        # check must happen before the primary-turn branch, otherwise a detour
        # would fall straight back into a generic 360-degree scan.
        if self._search_phase not in {
            "reacquire_anchor", "reacquire_observe", "reacquire_sweep"
        }:
            # Every visible target status replaces the anchor. When that target
            # is lost, the newest unconsumed observation wins over generic
            # search regardless of whether a LiDAR detour happened.
            self._target_reacquire_requested = False
            anchor = self.target_tracking_policy.reacquire_anchor(
                pose=(self._odom_xy[0], self._odom_xy[1], self._odom_yaw),
                now=now,
                allow_near=True,
            )
            if (
                anchor is not None
                and anchor.observation_id > self._last_reacquired_observation_id
            ):
                self._last_reacquired_observation_id = anchor.observation_id
                self._reacquire_anchor = anchor
                self._search_phase = "reacquire_anchor"
                return self._reacquire_anchor_command_locked()
        if self._search_phase == "wait_reposition_scan":
            if not self._scan_fresh_locked(now):
                return (0.0, 0.0, 0.0), "search_wait_lidar"
            return self._begin_reposition_locked()
        if self._search_phase not in {"primary_turn", "secondary_turn"} and not self._scan_fresh_locked(now):
            return (0.0, 0.0, 0.0), "lidar_stale"
        if self._search_phase == "reposition_turn":
            return self._reposition_turn_command_locked()
        if self._search_phase == "reacquire_anchor":
            return self._reacquire_anchor_command_locked()
        if self._search_phase == "reacquire_observe":
            if now < self._reacquire_observe_until:
                return (0.0, 0.0, 0.0), "reacquire_observe"
            # Do not immediately lose the remembered bearing to a 360-degree
            # scan. First inspect only that target-side sector, then leave for
            # the next open local search point if RGB still has no target.
            direction = self._reacquire_anchor.direction if self._reacquire_anchor else -1
            self._reacquire_anchor = None
            self._search_phase = "reacquire_sweep"
            self._search_direction = 1 if direction > 0 else -1
            self._reacquire_sweep_commanded_rad = 0.0
            self._search_turn_active = False
            self._reset_pulse_locked()
        if self._search_phase == "reacquire_sweep":
            if not self._turn_clear_locked(self._search_direction):
                self._search_phase = "wait_reposition_scan"
                self._search_turn_active = False
                self._reset_pulse_locked()
                return (0.0, 0.0, 0.0), "reacquire_sector_blocked"
            if self._reacquire_sweep_commanded_rad >= self._reacquire_sweep_limit_rad:
                self._search_phase = "wait_reposition_scan"
                self._search_turn_active = False
                self._reset_pulse_locked()
                return (0.0, 0.0, 0.0), "reacquire_sector_complete"
            was_turning = self._search_turn_active
            command = self._pulse_turn_locked(
                self._search_direction,
                now,
                speed_radps=self.search_turn_speed_radps,
            )
            self._search_turn_active = command[2] != 0.0
            if self._search_turn_active and not was_turning:
                self._reacquire_sweep_commanded_rad += (
                    self.search_turn_speed_radps * self.turn_step_sec
                )
            return command, (
                "reacquire_sweep_left"
                if self._search_direction > 0 else "reacquire_sweep_right"
            )
        if self._search_phase == "advance":
            return self._advance_command_locked()
        if self._search_phase not in {"primary_turn", "secondary_turn"}:
            preferred_direction = self.target_tracking_policy.preferred_search_direction(now=now)
            self._begin_scan_locked(direction=preferred_direction or -1, phase="primary_turn")

        direction = self._search_direction
        if not self._turn_clear_locked(direction):
            if self._search_phase == "primary_turn":
                self._begin_scan_locked(direction=1, phase="secondary_turn")
                return (0.0, 0.0, 0.0), "search_reverse"
            # Both directions are currently blocked.  Keep the mission armed
            # so a later LiDAR update can resume this same scan rather than
            # silently abandoning the coyote search.
            self._search_turn_active = False
            self._reset_pulse_locked()
            return (0.0, 0.0, 0.0), "search_turn_blocked"

        if self._sweep_commanded_rad >= self.search_sweep_rad:
            # Every successful 1 m leg creates a new local search point.
            # Continue from that point until the bounded session expires; only
            # then complete NOT_FOUND and use the existing direct home return.
            self._advance_completed = False
            self._search_phase = "wait_reposition_scan"
            self._search_turn_active = False
            self._reset_pulse_locked()
            if not self._scan_fresh_locked(now):
                return (0.0, 0.0, 0.0), "search_wait_lidar"
            return self._begin_reposition_locked()

        was_turning = self._search_turn_active
        command = self._pulse_turn_locked(
            direction,
            now,
            speed_radps=self.search_turn_speed_radps,
        )
        self._search_turn_active = command[2] != 0.0
        if self._search_turn_active and not was_turning:
            self._sweep_commanded_rad += self.search_turn_speed_radps * self.turn_step_sec
            self._last_sweep_yaw = self._odom_yaw
        return command, "search_clockwise" if direction < 0 else "search_counterclockwise"

    def _reacquire_anchor_command_locked(self) -> Tuple[Tuple[float, float, float], str]:
        """Return to the last robot pose that still had a target observation."""
        anchor = self._reacquire_anchor
        if anchor is None or self._odom_xy is None or self._odom_yaw is None:
            return (0.0, 0.0, 0.0), "odom_stale"
        dx = anchor.x - self._odom_xy[0]
        dy = anchor.y - self._odom_xy[1]
        distance = math.hypot(dx, dy)
        desired_yaw = anchor.target_yaw if distance <= 0.15 else math.atan2(dy, dx)
        yaw_error = _normalize_angle(desired_yaw - self._odom_yaw)
        if abs(yaw_error) > self.advance_yaw_tolerance_rad:
            return (0.0, 0.0, math.copysign(self.search_turn_speed_radps, yaw_error)), "reacquire_turn"
        if distance <= 0.15:
            # Give RGB one short, stationary observation window after facing
            # the remembered target bearing. Only a fresh miss starts a scan.
            self._search_phase = "reacquire_observe"
            self._reacquire_observe_until = self.monotonic_clock() + 0.40
            return (0.0, 0.0, 0.0), "reacquire_ready"
        if not self._front_clear_locked():
            self._reacquire_anchor = None
            self._begin_scan_locked(direction=anchor.direction or -1, phase="primary_turn")
            return (0.0, 0.0, 0.0), "reacquire_blocked"
        return (self.search_advance_speed_mps, 0.0, 0.0), "reacquire_drive"

    def _advance_command_locked(self) -> Tuple[Tuple[float, float, float], str]:
        if self._odom_xy is None:
            return (0.0, 0.0, 0.0), "odom_stale"
        if self._advance_start_xy is None:
            self._advance_start_xy = self._odom_xy
            self._advance_start_yaw = self._odom_yaw
        dx = self._odom_xy[0] - self._advance_start_xy[0]
        dy = self._odom_xy[1] - self._advance_start_xy[1]
        # A local LiDAR detour intentionally changes heading.  Count the
        # travelled ground distance, not projection against the pre-detour
        # heading, so one obstacle does not abort the search mission.
        travelled_distance = math.hypot(dx, dy)
        if (
            travelled_distance >= COYOTE_REPOSITION_MIN_ADVANCE_M
            and self._front_clearance_m is not None
            and self._front_clearance_m <= COYOTE_REPOSITION_RESCAN_CLEARANCE_M
        ):
            self._advance_completed = True
            self._begin_scan_locked(direction=-1, phase="secondary_turn")
            return (0.0, 0.0, 0.0), "search_reposition_ready"
        if travelled_distance >= self.search_advance_m:
            self._advance_completed = True
            self._begin_scan_locked(direction=-1, phase="secondary_turn")
            return (0.0, 0.0, 0.0), "search_advance_complete"
        return (self.search_advance_speed_mps, 0.0, 0.0), "search_advance"

    def _begin_reposition_locked(self) -> Tuple[Tuple[float, float, float], str]:
        """Face the widest observed sector before leaving a failed scan."""
        if self._odom_yaw is None:
            return (0.0, 0.0, 0.0), "odom_stale"
        candidates = (
            ("front", self._front_clearance_m),
            ("left", self._left_clearance_m),
            ("right", self._right_clearance_m),
        )
        known = [(name, distance) for name, distance in candidates if distance is not None]
        if not known:
            return (0.0, 0.0, 0.0), "lidar_stale"
        direction_name, _ = max(known, key=lambda candidate: candidate[1])
        yaw_offset = {
            "front": 0.0,
            "left": math.pi / 2.0,
            "right": -math.pi / 2.0,
        }[direction_name]
        self._reposition_target_yaw = _normalize_angle(self._odom_yaw + yaw_offset)
        self._search_phase = "reposition_turn"
        self._search_turn_active = False
        self._reset_pulse_locked()
        self.logger.info(
            "coyote search widest-sector=%s clearance=%.2f",
            direction_name,
            _,
        )
        return self._reposition_turn_command_locked()

    def _reposition_turn_command_locked(self) -> Tuple[Tuple[float, float, float], str]:
        if self._odom_yaw is None or self._reposition_target_yaw is None:
            return (0.0, 0.0, 0.0), "odom_stale"
        error = _normalize_angle(self._reposition_target_yaw - self._odom_yaw)
        if abs(error) <= self.advance_yaw_tolerance_rad:
            self._search_phase = "advance"
            self._advance_start_xy = None
            self._advance_start_yaw = self._odom_yaw
            self._search_turn_active = False
            return (0.0, 0.0, 0.0), "search_reposition_aligned"
        self._search_turn_active = True
        return (
            0.0,
            0.0,
            math.copysign(self.search_turn_speed_radps, error),
        ), "search_reposition_turn"

    def _begin_scan_locked(self, *, direction: int, phase: str) -> None:
        self._search_phase = phase
        self._search_direction = 1 if direction > 0 else -1
        self._sweep_angle_rad = 0.0
        self._sweep_commanded_rad = 0.0
        self._last_sweep_yaw = self._odom_yaw
        self._search_turn_active = False
        self._advance_start_xy = None
        self._advance_start_yaw = None
        self._reposition_target_yaw = None
        self._reset_pulse_locked()

    def _pulse_turn_locked(
        self,
        direction: int,
        now: float,
        *,
        speed_radps: float,
        step_sec: Optional[float] = None,
        pause_sec: Optional[float] = None,
    ) -> Tuple[float, float, float]:
        direction = 1 if direction > 0 else -1
        if direction != self._pulse_direction:
            self._reset_pulse_locked()
            self._pulse_direction = direction
        if now < self._pulse_active_until:
            return (0.0, 0.0, direction * speed_radps)
        if now < self._pulse_pause_until:
            return (0.0, 0.0, 0.0)
        active_sec = self.turn_step_sec if step_sec is None else step_sec
        inactive_sec = self.turn_pause_sec if pause_sec is None else pause_sec
        self._pulse_active_until = now + active_sec
        self._pulse_pause_until = self._pulse_active_until + inactive_sec
        return (0.0, 0.0, direction * speed_radps)

    def _reset_pulse_locked(self) -> None:
        self._pulse_direction = 0
        self._pulse_active_until = 0.0
        self._pulse_pause_until = 0.0

    def _guard_motion_locked(
        self,
        command: Tuple[float, float, float],
        reason: str,
        now: float,
    ) -> Tuple[Tuple[float, float, float], str]:
        key = self._motion_key_for_reason(reason)
        moving = any(abs(value) > 1e-9 for value in command)
        if not moving:
            if key != self._motion_key:
                self._reset_motion_guard_locked()
            return command, reason
        if key is None:
            return self._abort_motion_locked("motion_reason_invalid")
        if key != self._motion_key:
            self._motion_key = key
            self._motion_started_at = now
            self._motion_progress_at = now
            self._motion_progress_xy = self._odom_xy
            self._motion_progress_yaw = self._odom_yaw

        duration_limit = {
            "align": self.align_timeout_sec,
            "search_turn": self.search_turn_timeout_sec,
            "search_advance": self.search_advance_timeout_sec,
            "track_forward": self.track_forward_timeout_sec,
        }[key[0]]
        if (
            self._motion_started_at is None
            or now - self._motion_started_at > duration_limit
        ):
            return self._abort_motion_locked("motion_phase_timeout")
        if (
            self._motion_progress_at is None
            or now - self._motion_progress_at > self.progress_timeout_sec
        ):
            if key[0] in {"search_turn", "align"}:
                # Direct UDP turn commands are completed from perception's
                # side/center result.  Nav odometry may stay unchanged during
                # that adjustment, so it must not terminate the mission.
                self._search_turn_active = False
                self._reset_pulse_locked()
                self._reset_motion_guard_locked()
                return (0.0, 0.0, 0.0), "turn_progress_wait"
            return self._abort_motion_locked("motion_progress_timeout")
        return command, reason

    @staticmethod
    def _motion_key_for_reason(reason: str) -> Optional[Tuple[str, int]]:
        if reason == "align_left":
            return ("align", 1)
        if reason == "align_right":
            return ("align", -1)
        if reason == "search_clockwise":
            return ("search_turn", -1)
        if reason == "search_counterclockwise":
            return ("search_turn", 1)
        if reason == "search_advance":
            return ("search_advance", 0)
        if reason == "search_reposition_turn":
            return ("search_turn", 0)
        if reason == "reacquire_turn":
            return ("search_turn", 0)
        if reason == "reacquire_drive":
            return ("search_advance", 0)
        if reason == "reacquire_sweep_left":
            return ("search_turn", 1)
        if reason == "reacquire_sweep_right":
            return ("search_turn", -1)
        if reason == "obstacle_slow":
            return ("track_forward", 0)
        if reason == "forward":
            return ("track_forward", 0)
        if reason in {"obstacle_avoid_left", "obstacle_turn_left"}:
            return ("search_turn", 1)
        if reason in {"obstacle_avoid_right", "obstacle_turn_right"}:
            return ("search_turn", -1)
        if reason in {"obstacle_bypass_left", "obstacle_bypass_right"}:
            return ("track_forward", 0)
        return None

    def _abort_motion_locked(
        self,
        reason: str,
    ) -> Tuple[Tuple[float, float, float], str]:
        self._search_armed = False
        self._search_started_at = None
        self._active_detection_type = None
        self._active_event_id = None
        self._reacquire_anchor = None
        self._target_reacquire_requested = False
        self._reacquire_observe_until = 0.0
        self._reacquire_sweep_commanded_rad = 0.0
        self._last_reacquired_observation_id = 0
        self.target_tracking_policy.clear()
        self._search_phase = "idle"
        self._search_turn_active = False
        self._reset_pulse_locked()
        self._reset_motion_guard_locked()
        return (0.0, 0.0, 0.0), reason

    def _reset_motion_guard_locked(self) -> None:
        self._motion_key = None
        self._motion_started_at = None
        self._motion_progress_at = None
        self._motion_progress_xy = None
        self._motion_progress_yaw = None

    def _scan_fresh_locked(self, now: float) -> bool:
        if not self.require_scan:
            return True
        return (
            self._scan_received_at is not None
            and 0.0 <= now - self._scan_received_at <= self.sensor_timeout_sec
        )

    def _odom_fresh_locked(self, now: float) -> bool:
        return (
            self._odom_received_at is not None
            and 0.0 <= now - self._odom_received_at <= self.sensor_timeout_sec
        )

    def _turn_clear_locked(self, direction: int) -> bool:
        if not self.require_scan:
            return True
        clearance = self._left_clearance_m if direction > 0 else self._right_clearance_m
        return clearance is not None and clearance >= self.turn_clearance_m

    def _front_clear_locked(self) -> bool:
        if not self.require_scan:
            return True
        return (
            self._front_clearance_m is not None
            and self._front_clearance_m >= self.forward_clearance_m
        )

    @staticmethod
    def _should_release_locked(reason: str) -> bool:
        return reason in {
            "future_status",
            "invalid_status",
            "lidar_stale",
            "motion_phase_timeout",
            "motion_progress_timeout",
            "motion_reason_invalid",
            "no_status",
            "not_detected",
            "odom_stale",
            "search_advance_blocked",
            "search_advance_deviated",
            "search_exhausted",
            "search_not_found",
            "search_session_timeout",
            "search_turn_blocked",
            "stale_status",
            "stopped",
            "target_centered",
            "transport_not_ready",
        }

    def _release_output_locked(self) -> None:
        release = getattr(self.motion_sink, "release", None)
        if callable(release):
            release()

    def _publish_detection_complete_locked(
        self,
        detection_type: DetectionType,
        event_id: str,
        outcome: str,
    ) -> None:
        callback = (
            self.on_coyote_complete
            if detection_type is DetectionType.COYOTE
            else self.on_broken_cup_complete
        )
        if callback is None:
            return
        try:
            callback(event_id, outcome)
        except Exception:
            self.logger.exception(
                "%s COMPLETE callback failed event_id=%s",
                detection_type.value,
                event_id,
            )

    def _complete_active_detection_locked(
        self,
        *,
        outcome: str,
        reason: str,
    ) -> Tuple[Tuple[float, float, float], str]:
        completed_event_id = self._active_event_id
        completed_type = self._active_detection_type
        self._search_armed = False
        self._search_started_at = None
        self._active_detection_type = None
        self._active_event_id = None
        self._reacquire_anchor = None
        self._target_reacquire_requested = False
        self._reacquire_observe_until = 0.0
        self._reacquire_sweep_commanded_rad = 0.0
        self._last_reacquired_observation_id = 0
        self.target_tracking_policy.clear()
        self._search_phase = "idle"
        self._search_turn_active = False
        self._reset_pulse_locked()
        self._reset_motion_guard_locked()
        if completed_type is not None:
            self._detection_state[completed_type] = "complete"
            if completed_event_id:
                self._publish_detection_complete_locked(
                    completed_type,
                    completed_event_id,
                    outcome,
                )
        return (0.0, 0.0, 0.0), reason

    def _send_stop(self) -> None:
        self.motion_sink.send_cmd_vel(0.0, 0.0, 0.0)


class MediaPublisher(Protocol):
    def publish_image(
        self,
        detection_type: DetectionType,
        *,
        event_id: str,
        jpeg_bytes: Optional[bytes],
    ) -> None:
        ...

@dataclass(frozen=True)
class CoyoteMediaClaim:
    manifest_path: Path
    manifest: Dict[str, Any]

    @property
    def kind(self) -> str:
        return str(self.manifest["kind"])

    @property
    def event_id(self) -> str:
        return str(self.manifest["event_id"])


class CoyoteSpoolReader:
    """Read atomic spool entries and claim each artifact at most once."""

    def __init__(self, root: Union[str, Path]) -> None:
        self.root = Path(root).expanduser().resolve()
        self.events_dir = self.root / "events"
        self._status_token = None  # type: Optional[Tuple[int, int]]

    def read_status_if_changed(self) -> Optional[str]:
        path = self.root / "status.json"
        try:
            stat = path.stat()
        except FileNotFoundError:
            return None
        token = (stat.st_mtime_ns, stat.st_size)
        if token == self._status_token:
            return None
        body = path.read_text(encoding="utf-8")
        self._status_token = token
        return body

    def claim_next(self) -> Optional[CoyoteMediaClaim]:
        for ready_path, manifest in self.ready_manifests():
            claim = self.claim(str(manifest["event_id"]), str(manifest["kind"]))
            if claim is not None:
                return claim
        return None

    def ready_manifests(self) -> Tuple[Tuple[Path, Dict[str, Any]], ...]:
        if not self.events_dir.exists():
            return ()
        candidates = sorted(self.events_dir.glob("*/image.ready.json"))
        ready = []
        for ready_path in candidates:
            try:
                manifest = json.loads(ready_path.read_text(encoding="utf-8"))
                self._validate_manifest(ready_path, manifest)
            except Exception:
                continue
            ready.append((ready_path, manifest))
        return tuple(ready)

    def claim(self, event_id: str, kind: str) -> Optional[CoyoteMediaClaim]:
        event_id = validate_event_id(event_id)
        if kind != "image":
            raise ValueError("coyote media kind must be image")
        ready_path = self.events_dir / event_id / "{}.ready.json".format(kind)
        if any(
            (ready_path.parent / "{}.{}.json".format(kind, state)).exists()
            for state in ("published", "failed")
        ):
            return None
        sending_path = ready_path.with_name("{}.sending.json".format(kind))
        try:
            os.replace(str(ready_path), str(sending_path))
        except FileNotFoundError:
            return None
        try:
            manifest = json.loads(sending_path.read_text(encoding="utf-8"))
            self._validate_manifest(sending_path, manifest)
        except Exception as exc:
            self.fail(CoyoteMediaClaim(sending_path, {}), str(exc))
            return None
        return CoyoteMediaClaim(sending_path, manifest)

    def release(self, claim: CoyoteMediaClaim) -> None:
        ready_path = claim.manifest_path.with_name(
            claim.manifest_path.name.replace(".sending.json", ".ready.json")
        )
        os.replace(str(claim.manifest_path), str(ready_path))

    def publish(self, claim: CoyoteMediaClaim, publisher: MediaPublisher) -> None:
        manifest = claim.manifest
        data = None
        if manifest["result"] == "SUCCESS":
            media_path = Path(str(manifest["path"]))
            data = media_path.read_bytes()
            if len(data) != int(manifest["bytes"]):
                raise ValueError("spooled media size changed after ready manifest")
            if hashlib.sha256(data).hexdigest() != str(manifest["sha256"]):
                raise ValueError("spooled media hash changed after ready manifest")
        if claim.kind == "image":
            publisher.publish_image(
                DetectionType.COYOTE,
                event_id=claim.event_id,
                jpeg_bytes=data,
            )
        else:
            raise ValueError("unsupported coyote media kind")

    def complete(self, claim: CoyoteMediaClaim) -> Path:
        published_path = claim.manifest_path.with_name(
            claim.manifest_path.name.replace(".sending.json", ".published.json")
        )
        os.replace(str(claim.manifest_path), str(published_path))
        return published_path

    def fail(self, claim: CoyoteMediaClaim, reason: str) -> Path:
        failed_path = claim.manifest_path.with_name(
            claim.manifest_path.name.replace(".sending.json", ".failed.json")
        )
        try:
            os.replace(str(claim.manifest_path), str(failed_path))
        except FileNotFoundError:
            failed_path.parent.mkdir(parents=True, exist_ok=True)
            failed_path.write_text("{}", encoding="utf-8")
        error_path = failed_path.with_name(failed_path.stem + ".error.txt")
        error_path.write_text(str(reason)[:2048], encoding="utf-8")
        return failed_path

    def _validate_manifest(self, manifest_path: Path, manifest: Any) -> None:
        if not isinstance(manifest, dict):
            raise ValueError("coyote media manifest must be a JSON object")
        if manifest.get("version") != 1:
            raise ValueError("unsupported coyote media manifest version")
        event_id = validate_event_id(manifest.get("event_id"))
        if manifest_path.parent.name != event_id:
            raise ValueError("manifest event_id does not match its spool directory")
        kind = manifest.get("kind")
        if kind != "image":
            raise ValueError("manifest kind must be image")
        if manifest.get("format") != "jpeg":
            raise ValueError("manifest format does not match media kind")
        result = manifest.get("result", "SUCCESS")
        if result not in ("SUCCESS", "FAIL"):
            raise ValueError("manifest result must be SUCCESS or FAIL")
        manifest["result"] = result
        if result == "SUCCESS":
            media_path = Path(str(manifest.get("path", ""))).expanduser().resolve()
            expected_path = (manifest_path.parent / "image.jpg").resolve()
            if media_path != expected_path or not _is_relative_to(
                media_path, self.events_dir
            ):
                raise ValueError("manifest media path escapes its event directory")
            size = manifest.get("bytes")
            if isinstance(size, bool) or not isinstance(size, int) or size <= 0:
                raise ValueError("manifest bytes must be a positive integer")
            digest = manifest.get("sha256")
            if not isinstance(digest, str) or len(digest) != 64:
                raise ValueError("manifest sha256 must be a hex digest")


class CoyoteMediaWorker:
    """Bounded worker so image base64 publishing never blocks the ROS timer."""

    def __init__(
        self,
        reader: CoyoteSpoolReader,
        publisher: MediaPublisher,
        *,
        max_pending: int = 2,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        if max_pending <= 0:
            raise ValueError("max_pending must be positive")
        self.reader = reader
        self.publisher = publisher
        self.logger = logger or logging.getLogger(__name__)
        self._queue = queue.Queue(maxsize=max_pending)  # type: queue.Queue
        self._closed = threading.Event()
        self._thread = threading.Thread(
            target=self._run,
            name="coyote-media-mqtt",
            daemon=True,
        )
        self._thread.start()

    def submit(self, claim: CoyoteMediaClaim) -> bool:
        if self._closed.is_set():
            return False
        try:
            self._queue.put_nowait(claim)
        except queue.Full:
            return False
        return True

    def close(self, timeout_sec: float = 15.0) -> None:
        self._closed.set()
        self._thread.join(timeout=timeout_sec)
        if self._thread.is_alive():
            self.logger.error("coyote media worker did not stop within %.1fs", timeout_sec)

    def _run(self) -> None:
        while not self._closed.is_set() or not self._queue.empty():
            try:
                claim = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                self.reader.publish(claim, self.publisher)
            except Exception as exc:
                self._fail_claim(claim, str(exc))
                self.logger.exception(
                    "coyote media publish failed event_id=%s kind=%s",
                    claim.event_id,
                    claim.kind,
                )
            else:
                try:
                    self.reader.complete(claim)
                except Exception as exc:
                    self._fail_claim(claim, str(exc))
                    self.logger.exception(
                        "coyote media completion failed event_id=%s kind=%s",
                        claim.event_id,
                        claim.kind,
                    )
            finally:
                self._queue.task_done()

    def _fail_claim(self, claim: CoyoteMediaClaim, reason: str) -> None:
        try:
            self.reader.fail(claim, reason)
        except Exception:
            self.logger.exception(
                "coyote media failure tombstone failed event_id=%s kind=%s",
                claim.event_id,
                claim.kind,
            )


def _enum_string(value: Dict[str, Any], key: str, allowed: Tuple[str, ...]) -> str:
    item = value.get(key)
    if not isinstance(item, str) or item not in allowed:
        raise ValueError("{} must be one of {}".format(key, ", ".join(allowed)))
    return item


def _normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _optional_string(value: Dict[str, Any], key: str) -> str:
    item = value.get(key, "")
    if not isinstance(item, str):
        raise ValueError("{} must be a string".format(key))
    return item


def _is_relative_to(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root.resolve())
    except ValueError:
        return False
    return True
