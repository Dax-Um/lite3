"""Deterministic local obstacle arbitration for direct Lite3 motion.

This module deliberately has no ROS or UDP dependency.  ROS callbacks update
one ``ClearanceSnapshot``; one motion tick asks this policy for a decision and
is the only place allowed to emit a velocity command.  That keeps perception,
target tracking and return-home callbacks from racing each other.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class ClearanceSnapshot:
    front_m: Optional[float]
    left_m: Optional[float]
    right_m: Optional[float]
    received_at_monotonic: float


@dataclass(frozen=True)
class AvoidanceConfig:
    # Deliberately compact clearances: body envelope plus a small braking
    # margin, not a map/navigation inflation radius.
    hard_stop_m: float = 0.32
    forward_clearance_m: float = 0.50
    turn_clearance_m: float = 0.32
    avoid_turn_wz_radps: float = 0.85
    # Do not hand control back after a tiny turn.  First face a clearly open
    # corridor, then drive straight through that corridor before RGB target
    # alignment can turn the body back toward the obstacle.
    resume_clearance_m: float = 0.65
    min_turn_sec: float = 1.35
    bypass_speed_mps: float = 0.50
    bypass_sec: float = 1.00


@dataclass(frozen=True)
class AvoidanceDecision:
    vx: float
    vy: float
    wz: float
    reason: str


class LocalAvoidancePolicy:
    """Override a requested direct command only when clearance requires it."""

    def __init__(self, config: AvoidanceConfig = AvoidanceConfig()) -> None:
        if min(
            config.hard_stop_m,
            config.forward_clearance_m,
            config.turn_clearance_m,
            config.avoid_turn_wz_radps,
            config.resume_clearance_m,
            config.min_turn_sec,
            config.bypass_speed_mps,
            config.bypass_sec,
        ) <= 0.0:
            raise ValueError("avoidance config values must be positive")
        if config.hard_stop_m > config.forward_clearance_m:
            raise ValueError("hard_stop_m must not exceed forward_clearance_m")
        if config.resume_clearance_m < config.forward_clearance_m:
            raise ValueError("resume_clearance_m must not be below forward_clearance_m")
        self.config = config
        self._phase = "idle"
        self._direction = None  # type: Optional[int]
        self._phase_started_at = None  # type: Optional[float]

    def reset(self) -> None:
        """Clear a completed/aborted detour before a new independent mission."""
        self._phase = "idle"
        self._direction = None
        self._phase_started_at = None

    def arbitrate(
        self,
        requested: tuple[float, float, float],
        snapshot: Optional[ClearanceSnapshot],
        *,
        target_matched: bool = False,
    ) -> Optional[AvoidanceDecision]:
        """Return an override, or ``None`` when the requested command is safe.

        ``target_matched`` must only be true after RGB/RealSense and LiDAR
        associate the same front cluster.  A matched target is held (never
        treated as a generic obstacle to evade); the caller can then complete
        the target mission from its normal distance rule.
        """
        vx, vy, wz = requested
        if snapshot is None:
            return None
        if target_matched:
            self.reset()
        if abs(vx) <= 1e-9 and abs(vy) <= 1e-9 and abs(wz) <= 1e-9:
            return self._active_detour(snapshot)
        active = self._active_detour(snapshot)
        if active is not None:
            return active
        if vx > 0.0:
            front = snapshot.front_m
            if front is None or front >= self.config.forward_clearance_m:
                return None
            if target_matched:
                return AvoidanceDecision(0.0, 0.0, 0.0, "target_proximity_hold")
            direction = self._open_turn_direction(snapshot)
            if direction is None:
                reason = "obstacle_hard_stop" if front <= self.config.hard_stop_m else "obstacle_no_turn_clearance"
                return AvoidanceDecision(0.0, 0.0, 0.0, reason)
            self._phase = "turn"
            self._direction = direction
            self._phase_started_at = snapshot.received_at_monotonic
            return self._turn_decision(direction)
        if wz != 0.0:
            clearance = snapshot.left_m if wz > 0.0 else snapshot.right_m
            if clearance is not None and clearance >= self.config.turn_clearance_m:
                return None
            direction = self._open_turn_direction(snapshot)
            if direction is None:
                return AvoidanceDecision(0.0, 0.0, 0.0, "obstacle_turn_blocked")
            return AvoidanceDecision(
                0.0, 0.0, direction * min(abs(wz), self.config.avoid_turn_wz_radps),
                "obstacle_turn_left" if direction > 0 else "obstacle_turn_right",
            )
        return None

    def _active_detour(
        self,
        snapshot: ClearanceSnapshot,
    ) -> Optional[AvoidanceDecision]:
        """Continue the selected detour until the robot has passed its nose."""
        if self._phase == "idle":
            return None
        direction = self._direction
        if direction is None:
            self.reset()
            return None
        if self._phase == "turn":
            front = snapshot.front_m
            started = self._phase_started_at or snapshot.received_at_monotonic
            if (
                snapshot.received_at_monotonic - started >= self.config.min_turn_sec
                and (front is None or front >= self.config.resume_clearance_m)
            ):
                self._phase = "bypass"
                self._phase_started_at = snapshot.received_at_monotonic
                return self._bypass_decision(direction)
            # Keep the originally selected side unless it became occupied.
            side_clearance = snapshot.left_m if direction > 0 else snapshot.right_m
            if side_clearance is None or side_clearance < self.config.turn_clearance_m:
                alternate = self._open_turn_direction(snapshot)
                if alternate is None:
                    return AvoidanceDecision(0.0, 0.0, 0.0, "obstacle_no_turn_clearance")
                self._direction = alternate
                direction = alternate
            return self._turn_decision(direction)
        if self._phase == "bypass":
            front = snapshot.front_m
            if front is not None and front < self.config.forward_clearance_m:
                self._phase = "turn"
                self._phase_started_at = snapshot.received_at_monotonic
                return self._turn_decision(direction)
            started = self._phase_started_at or snapshot.received_at_monotonic
            if snapshot.received_at_monotonic - started < self.config.bypass_sec:
                return self._bypass_decision(direction)
            self.reset()
            return None
        self.reset()
        return None

    def _turn_decision(self, direction: int) -> AvoidanceDecision:
        return AvoidanceDecision(
            0.0,
            0.0,
            direction * self.config.avoid_turn_wz_radps,
            "obstacle_avoid_left" if direction > 0 else "obstacle_avoid_right",
        )

    def _bypass_decision(self, direction: int) -> AvoidanceDecision:
        return AvoidanceDecision(
            self.config.bypass_speed_mps,
            0.0,
            0.0,
            "obstacle_bypass_left" if direction > 0 else "obstacle_bypass_right",
        )

    def _open_turn_direction(self, snapshot: ClearanceSnapshot) -> Optional[int]:
        left = snapshot.left_m
        right = snapshot.right_m
        left_clear = left is not None and left >= self.config.turn_clearance_m
        right_clear = right is not None and right >= self.config.turn_clearance_m
        if left_clear and right_clear:
            return 1 if left >= right else -1
        if left_clear:
            return 1
        if right_clear:
            return -1
        return None
