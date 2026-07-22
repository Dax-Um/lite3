"""Target observation memory for safe post-avoidance reacquisition."""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class TargetObservationAnchor:
    x: float
    y: float
    yaw: float
    target_yaw: float
    direction: int
    observation_id: int
    observed_at: float


class TargetTrackingPolicy:
    """Store target observation anchors; this module never writes motion."""

    def __init__(
        self,
        *,
        reacquire_timeout_sec: float = 5.0,
        return_distance_m: float = 0.25,
        side_bearing_rad: float = 0.70,
    ) -> None:
        if (
            reacquire_timeout_sec <= 0.0
            or return_distance_m <= 0.0
            or not 0.0 < side_bearing_rad < 1.5707963267948966
        ):
            raise ValueError("target reacquisition limits must be positive")
        self.reacquire_timeout_sec = reacquire_timeout_sec
        self.return_distance_m = return_distance_m
        self.side_bearing_rad = side_bearing_rad
        self._anchor: Optional[TargetObservationAnchor] = None
        self._next_observation_id = 1

    def observe(
        self,
        *,
        detect: str,
        side: str,
        pose: Optional[Tuple[float, float, float]],
        now: Optional[float] = None,
    ) -> None:
        if detect != "detected" or pose is None:
            return
        observed_at = time.monotonic() if now is None else now
        direction = 1 if side == "left" else -1 if side == "right" else 0
        target_yaw = float(pose[2]) + direction * self.side_bearing_rad
        self._anchor = TargetObservationAnchor(
            x=float(pose[0]), y=float(pose[1]), yaw=float(pose[2]),
            target_yaw=target_yaw,
            direction=direction,
            observation_id=self._next_observation_id,
            observed_at=observed_at,
        )
        self._next_observation_id += 1

    def clear(self) -> None:
        """Discard anchors from a completed or newly-started mission."""
        self._anchor = None

    def reacquire_anchor(
        self,
        *,
        pose: Optional[Tuple[float, float, float]],
        now: Optional[float] = None,
        allow_near: bool = False,
    ) -> Optional[TargetObservationAnchor]:
        anchor = self._anchor
        current = time.monotonic() if now is None else now
        if anchor is None or pose is None:
            return None
        if current - anchor.observed_at > self.reacquire_timeout_sec:
            return None
        dx = float(pose[0]) - anchor.x
        dy = float(pose[1]) - anchor.y
        if (
            not allow_near
            and dx * dx + dy * dy < self.return_distance_m * self.return_distance_m
        ):
            return None
        return anchor

    def preferred_search_direction(self, *, now: Optional[float] = None) -> Optional[int]:
        anchor = self._anchor
        current = time.monotonic() if now is None else now
        if anchor is None or current - anchor.observed_at > self.reacquire_timeout_sec:
            return None
        return anchor.direction or None
