"""Map-free local return-vector math for the coyote demo.

Only two target observations and Motion Host IMU yaw are used.  Coordinates
are kept in the yaw frame of the first observation; no Nav2, TF or `/odom`
input is required.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


def normalize_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


@dataclass(frozen=True)
class TargetObservation:
    """Planar target vector in the robot body frame at one capture instant."""

    forward_m: float
    left_m: float
    yaw_rad: float

    def __post_init__(self) -> None:
        if not all(math.isfinite(value) for value in (self.forward_m, self.left_m, self.yaw_rad)):
            raise ValueError("target observation values must be finite")
        if math.hypot(self.forward_m, self.left_m) <= 0.05:
            raise ValueError("target observation is too close to define a return vector")


@dataclass(frozen=True)
class ReturnVector:
    """Start-home vector represented in the first IMU yaw frame."""

    x_m: float
    y_m: float
    reference_yaw_rad: float

    @property
    def distance_m(self) -> float:
        return math.hypot(self.x_m, self.y_m)

    def target_heading_at(self, current_yaw_rad: float) -> float:
        """Desired robot yaw for forward-only drive to home."""
        if not math.isfinite(current_yaw_rad):
            raise ValueError("current yaw must be finite")
        world_heading = math.atan2(self.y_m, self.x_m)
        return normalize_angle(world_heading + self.reference_yaw_rad)


def _rotate(vector_x: float, vector_y: float, yaw_rad: float) -> tuple[float, float]:
    cos_yaw, sin_yaw = math.cos(yaw_rad), math.sin(yaw_rad)
    return (
        cos_yaw * vector_x - sin_yaw * vector_y,
        sin_yaw * vector_x + cos_yaw * vector_y,
    )


def calculate_return_vector(
    start: TargetObservation,
    stop: TargetObservation,
    *,
    min_distance_m: float = 0.10,
    max_distance_m: float = 10.0,
) -> ReturnVector:
    """Calculate stop->home vector from one stationary target seen twice.

    ``start`` and ``stop`` are target vectors in their respective body frames.
    The robot displacement is target_start - target_stop in one common yaw
    frame, so the return vector is its inverse.
    """
    if not 0.0 < min_distance_m < max_distance_m:
        raise ValueError("return distance bounds must be positive and ordered")
    yaw_delta = normalize_angle(stop.yaw_rad - start.yaw_rad)
    stop_in_start_frame = _rotate(stop.forward_m, stop.left_m, yaw_delta)
    # Target-start minus target-stop is robot start->stop.  Invert it to get
    # robot stop->home, still in the start yaw frame.
    return_x = stop_in_start_frame[0] - start.forward_m
    return_y = stop_in_start_frame[1] - start.left_m
    distance = math.hypot(return_x, return_y)
    if distance < min_distance_m:
        raise ValueError("return vector is too short; target captures are not distinct")
    if distance > max_distance_m:
        raise ValueError("return vector exceeds local-demo bound")
    return ReturnVector(return_x, return_y, start.yaw_rad)
