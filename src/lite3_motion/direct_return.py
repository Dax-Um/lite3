"""Map-free return-home executor using Motion Host yaw and direct UDP velocity."""
from __future__ import annotations

import math
import logging
import time
from dataclasses import dataclass
from typing import Callable, Optional, Protocol, Tuple

from .local_return import ReturnVector, normalize_angle
from .local_avoidance import ClearanceSnapshot, LocalAvoidancePolicy


class VelocityDriver(Protocol):
    def send_cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        ...


@dataclass(frozen=True)
class DirectReturnConfig:
    forward_speed_mps: float = 1.50
    turn_speed_radps: float = 0.65
    # Motion Host yaw is available directly on IQ9.  Use a tight terminal
    # tolerance and slow down near the requested heading instead of stopping
    # a fast fixed-rate turn with a large residual yaw error.
    yaw_tolerance_rad: float = 0.01
    turn_heading_kp: float = 1.50
    turn_min_wz_radps: float = 0.08
    control_period_sec: float = 0.05
    turn_timeout_sec: float = 12.0
    # A 3x4 m demo can require several avoidance detours. This is a hard
    # ceiling, not the normal stop condition; live position progress below
    # detects a genuine stall much sooner.
    drive_timeout_sec: float = 60.0
    drive_progress_timeout_sec: float = 12.0
    drive_progress_distance_m: float = 0.10
    distance_tolerance_m: float = 0.015
    # Brake into the saved direct Motion Host home coordinate instead of
    # retaining the demo's 1.5 m/s speed until the final control tick.
    arrival_slowdown_distance_m: float = 0.80
    arrival_speed_kp: float = 1.50
    arrival_min_speed_mps: float = 0.10
    # Keep the robot on the captured return heading while it drives home.
    drive_heading_kp: float = 1.20
    drive_max_wz_radps: float = 0.35
    clearance_timeout_sec: float = 0.50


class DirectReturnExecutor:
    """Turn to the stored home vector then drive its bounded distance forward.

    This deliberately uses neither Nav2, TF nor `/odom`. ``yaw_provider`` is
    fed by IQ9's direct Motion Host Robot State receiver.
    """

    def __init__(
        self,
        driver: VelocityDriver,
        yaw_provider: Callable[[], float | None],
        *,
        config: DirectReturnConfig = DirectReturnConfig(),
        position_provider: Optional[Callable[[], Optional[Tuple[float, float]]]] = None,
        clearance_provider: Optional[Callable[[], Optional[ClearanceSnapshot]]] = None,
        avoidance_policy: Optional[LocalAvoidancePolicy] = None,
        logger: Optional[logging.Logger] = None,
        sleep: Callable[[float], None] = time.sleep,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        if min(config.forward_speed_mps, config.turn_speed_radps, config.yaw_tolerance_rad, config.turn_heading_kp, config.turn_min_wz_radps, config.control_period_sec, config.turn_timeout_sec, config.drive_timeout_sec, config.drive_progress_timeout_sec, config.drive_progress_distance_m, config.distance_tolerance_m, config.arrival_slowdown_distance_m, config.arrival_speed_kp, config.arrival_min_speed_mps, config.drive_heading_kp, config.drive_max_wz_radps, config.clearance_timeout_sec) <= 0:
            raise ValueError("direct return config values must be positive")
        self.driver = driver
        self.yaw_provider = yaw_provider
        self.config = config
        self.position_provider = position_provider
        self.clearance_provider = clearance_provider
        self.avoidance_policy = avoidance_policy
        self.logger = logger or logging.getLogger(__name__)
        self._last_avoidance_reason = None
        self.sleep = sleep
        self.monotonic = monotonic

    def run(self, vector: ReturnVector) -> None:
        if vector.distance_m <= 0.0:
            raise ValueError("return vector distance must be positive")
        deadline = self.monotonic() + self.config.turn_timeout_sec
        try:
            while True:
                yaw = self.yaw_provider()
                if yaw is None or not math.isfinite(yaw):
                    raise RuntimeError("direct Motion Host yaw is unavailable")
                error = normalize_angle(vector.target_heading_at(yaw) - yaw)
                if abs(error) <= self.config.yaw_tolerance_rad:
                    break
                if self.monotonic() >= deadline:
                    raise TimeoutError("direct return turn timed out")
                self._send_safe(
                    0.0,
                    0.0,
                    self._turn_wz(error),
                )
                self.sleep(self.config.control_period_sec)

            start_position = None
            if self.position_provider is not None:
                start_position = self.position_provider()
                if start_position is None or not all(math.isfinite(value) for value in start_position):
                    raise RuntimeError("direct Motion Host position is unavailable")
            remaining = vector.distance_m
            drive_deadline = self.monotonic() + self.config.drive_timeout_sec
            while remaining > self.config.distance_tolerance_m:
                if self.monotonic() >= drive_deadline:
                    raise TimeoutError("direct return forward drive timed out")
                yaw = self.yaw_provider()
                if yaw is None or not math.isfinite(yaw):
                    raise RuntimeError("direct Motion Host yaw became unavailable")
                heading_error = normalize_angle(vector.target_heading_at(yaw) - yaw)
                yaw_correction = max(
                    -self.config.drive_max_wz_radps,
                    min(
                        self.config.drive_max_wz_radps,
                        self.config.drive_heading_kp * heading_error,
                    ),
                )
                command = self._send_safe(
                    self.config.forward_speed_mps,
                    0.0,
                    yaw_correction,
                )
                self.sleep(self.config.control_period_sec)
                if start_position is None:
                    remaining -= max(0.0, command[0]) * self.config.control_period_sec
                    continue
                current_position = self.position_provider()
                if current_position is None or not all(math.isfinite(value) for value in current_position):
                    raise RuntimeError("direct Motion Host position became unavailable")
                travelled = math.hypot(
                    current_position[0] - start_position[0],
                    current_position[1] - start_position[1],
                )
                remaining = max(0.0, vector.distance_m - travelled)
        finally:
            self.driver.send_cmd_vel(0.0, 0.0, 0.0)

    def run_to_position(self, target_position: Tuple[float, float]) -> None:
        """Return to an IQ9 Motion Host ``x,y`` home position in closed loop.

        Unlike the RealSense target-offset fallback, the target is the exact
        direct Robot State position captured before a mission.  The heading
        and remaining distance are recalculated from live Motion Host state
        on every control tick; Nav2, TF and perception-host odometry are not
        involved.
        """
        if self.position_provider is None:
            raise RuntimeError("direct Motion Host position provider is unavailable")
        if not all(math.isfinite(value) for value in target_position):
            raise ValueError("direct home position must be finite")

        started_at = self.monotonic()
        deadline = started_at + self.config.drive_timeout_sec
        last_progress_at = started_at
        last_progress_position = None
        try:
            while True:
                now = self.monotonic()
                if now >= deadline:
                    raise TimeoutError("direct return-to-position hard timeout")
                current_position = self.position_provider()
                if current_position is None or not all(
                    math.isfinite(value) for value in current_position
                ):
                    raise RuntimeError("direct Motion Host position is unavailable")
                dx = target_position[0] - current_position[0]
                dy = target_position[1] - current_position[1]
                remaining = math.hypot(dx, dy)
                if remaining <= self.config.distance_tolerance_m:
                    return current_position, remaining
                if last_progress_position is None or math.hypot(
                    current_position[0] - last_progress_position[0],
                    current_position[1] - last_progress_position[1],
                ) >= self.config.drive_progress_distance_m:
                    last_progress_position = current_position
                    last_progress_at = now
                elif now - last_progress_at >= self.config.drive_progress_timeout_sec:
                    raise TimeoutError("direct return-to-position stalled")

                yaw = self.yaw_provider()
                if yaw is None or not math.isfinite(yaw):
                    raise RuntimeError("direct Motion Host yaw is unavailable")
                target_heading = math.atan2(dy, dx)
                heading_error = normalize_angle(target_heading - yaw)

                # Never arc forward when the robot is materially misaligned.
                # It first faces the recomputed home direction, then performs
                # small heading corrections while moving straight toward it.
                if abs(heading_error) > 0.10:
                    self._send_safe(0.0, 0.0, self._turn_wz(heading_error))
                else:
                    yaw_correction = max(
                        -self.config.drive_max_wz_radps,
                        min(
                            self.config.drive_max_wz_radps,
                            self.config.drive_heading_kp * heading_error,
                        ),
                    )
                    forward_speed = self.config.forward_speed_mps
                    if remaining < self.config.arrival_slowdown_distance_m:
                        forward_speed = max(
                            self.config.arrival_min_speed_mps,
                            min(
                                self.config.forward_speed_mps,
                                self.config.arrival_speed_kp * remaining,
                            ),
                        )
                    self._send_safe(
                        forward_speed,
                        0.0,
                        yaw_correction,
                    )
                self.sleep(self.config.control_period_sec)
        finally:
            self.driver.send_cmd_vel(0.0, 0.0, 0.0)

    def _turn_wz(self, error: float) -> float:
        """Bounded proportional yaw command for a precise final heading."""
        magnitude = min(
            self.config.turn_speed_radps,
            max(
                self.config.turn_min_wz_radps,
                self.config.turn_heading_kp * abs(error),
            ),
        )
        return math.copysign(magnitude, error)

    def _send_safe(self, vx: float, vy: float, wz: float) -> Tuple[float, float, float]:
        """Apply a fresh local-obstacle override, then emit one UDP command."""
        command = (vx, vy, wz)
        snapshot = self.clearance_provider() if self.clearance_provider else None
        if (
            snapshot is not None
            and self.avoidance_policy is not None
            and 0.0 <= self.monotonic() - snapshot.received_at_monotonic
            <= self.config.clearance_timeout_sec
        ):
            override = self.avoidance_policy.arbitrate(command, snapshot)
            if override is not None:
                command = (override.vx, override.vy, override.wz)
                if override.reason != self._last_avoidance_reason:
                    self.logger.warning(
                        "direct-return avoidance reason=%s cmd_vel=(%.2f, %.2f, %.2f)",
                        override.reason,
                        *command,
                    )
                self._last_avoidance_reason = override.reason
            elif self._last_avoidance_reason is not None:
                self.logger.info("direct-return avoidance cleared")
                self._last_avoidance_reason = None
        self.driver.send_cmd_vel(*command)
        return command

    def spin_relative(self, angle_rad: float) -> None:
        """Turn a requested angle using direct Motion Host yaw feedback."""
        if not math.isfinite(angle_rad) or abs(angle_rad) <= 0.0:
            raise ValueError("relative turn angle must be finite and non-zero")
        initial_yaw = self.yaw_provider()
        if initial_yaw is None or not math.isfinite(initial_yaw):
            raise RuntimeError("direct Motion Host yaw is unavailable")
        target_yaw = normalize_angle(initial_yaw + angle_rad)
        deadline = self.monotonic() + self.config.turn_timeout_sec
        try:
            while True:
                yaw = self.yaw_provider()
                if yaw is None or not math.isfinite(yaw):
                    raise RuntimeError("direct Motion Host yaw became unavailable")
                error = normalize_angle(target_yaw - yaw)
                if abs(error) <= self.config.yaw_tolerance_rad:
                    return
                if self.monotonic() >= deadline:
                    raise TimeoutError("direct final turn timed out")
                self._send_safe(
                    0.0,
                    0.0,
                    self._turn_wz(error),
                )
                self.sleep(self.config.control_period_sec)
        finally:
            self.driver.send_cmd_vel(0.0, 0.0, 0.0)
