import math

import pytest

from lite3_motion.local_return import (
    TargetObservation,
    calculate_return_vector,
)


def test_return_is_reverse_of_forward_approach_without_turn():
    start = TargetObservation(forward_m=3.0, left_m=0.0, yaw_rad=0.0)
    stop = TargetObservation(forward_m=1.0, left_m=0.0, yaw_rad=0.0)
    vector = calculate_return_vector(start, stop)
    assert vector.x_m == pytest.approx(-2.0)
    assert vector.y_m == pytest.approx(0.0)
    assert vector.distance_m == pytest.approx(2.0)
    assert vector.target_heading_at(0.0) == pytest.approx(math.pi)


def test_return_accounts_for_yaw_change_between_snapshots():
    # Target is initially 2 m ahead.  Robot turns left 90 degrees without
    # translating; target is then 2 m to its right.  No return is required.
    start = TargetObservation(forward_m=2.0, left_m=0.0, yaw_rad=0.0)
    stop = TargetObservation(forward_m=0.0, left_m=-2.0, yaw_rad=math.pi / 2.0)
    with pytest.raises(ValueError, match="too short"):
        calculate_return_vector(start, stop)


def test_return_rejects_unreasonable_distance():
    start = TargetObservation(forward_m=20.0, left_m=0.0, yaw_rad=0.0)
    stop = TargetObservation(forward_m=1.0, left_m=0.0, yaw_rad=0.0)
    with pytest.raises(ValueError, match="exceeds"):
        calculate_return_vector(start, stop, max_distance_m=5.0)
