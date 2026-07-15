"""Front boundary detector for lane-end events."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class BoundaryConfig:
    front_angle_rad: float = 0.436332
    stop_distance_m: float = 0.60
    slow_distance_m: float = 1.20
    confirm_frames: int = 5
    min_valid_points: int = 3
    min_range_m: float = 0.05
    max_range_m: float = 10.0


@dataclass(frozen=True)
class BoundaryResult:
    lane_end: bool
    should_slow: bool
    should_stop: bool
    min_front_distance_m: float | None
    valid_front_points: int


class LidarBoundaryDetector:
    def __init__(self, config: BoundaryConfig = BoundaryConfig()):
        self.config = config
        self._hit_count = 0

    def update_scan(
        self,
        ranges: list[float],
        angle_min: float,
        angle_increment: float,
    ) -> BoundaryResult:
        if angle_increment <= 0.0:
            raise ValueError("angle_increment must be positive")

        valid_ranges = [
            value
            for index, value in enumerate(ranges)
            if self._front_valid_range(index, value, angle_min, angle_increment)
        ]
        valid_count = len(valid_ranges)
        if valid_count < self.config.min_valid_points:
            self._hit_count = 0
            return BoundaryResult(
                lane_end=False,
                should_slow=False,
                should_stop=False,
                min_front_distance_m=None,
                valid_front_points=valid_count,
            )

        min_front = min(valid_ranges)
        should_slow = min_front < self.config.slow_distance_m
        should_stop = min_front < self.config.stop_distance_m
        if should_stop:
            self._hit_count += 1
        else:
            self._hit_count = 0

        return BoundaryResult(
            lane_end=self._hit_count >= self.config.confirm_frames,
            should_slow=should_slow,
            should_stop=should_stop,
            min_front_distance_m=min_front,
            valid_front_points=valid_count,
        )

    def reset(self) -> None:
        self._hit_count = 0

    def _front_valid_range(
        self,
        index: int,
        value: float,
        angle_min: float,
        angle_increment: float,
    ) -> bool:
        angle = angle_min + index * angle_increment
        return (
            abs(angle) <= self.config.front_angle_rad
            and isfinite(value)
            and self.config.min_range_m <= value <= self.config.max_range_m
        )
