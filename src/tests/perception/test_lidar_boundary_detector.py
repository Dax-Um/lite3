import math

import pytest

from lite3_perception.lidar_boundary_detector import BoundaryConfig, LidarBoundaryDetector


def test_ignores_ranges_outside_front_angle():
    detector = LidarBoundaryDetector(BoundaryConfig(front_angle_rad=0.1, min_valid_points=1))

    result = detector.update_scan([0.3, 2.0, 0.3], angle_min=-0.2, angle_increment=0.2)

    assert result.min_front_distance_m == 2.0
    assert result.valid_front_points == 1
    assert result.should_stop is False


def test_ignores_nan_and_inf_ranges():
    detector = LidarBoundaryDetector(BoundaryConfig(min_valid_points=2))

    result = detector.update_scan(
        [math.nan, 0.5, math.inf, 0.8, -math.inf],
        angle_min=-0.2,
        angle_increment=0.1,
    )

    assert result.valid_front_points == 2
    assert result.min_front_distance_m == 0.5
    assert result.should_stop is True


def test_requires_min_valid_points():
    detector = LidarBoundaryDetector(BoundaryConfig(min_valid_points=3))

    result = detector.update_scan([0.4, 0.5], angle_min=-0.1, angle_increment=0.1)

    assert result.min_front_distance_m is None
    assert result.valid_front_points == 2
    assert result.should_slow is False
    assert result.should_stop is False
    assert result.lane_end is False


def test_should_slow_inside_slow_distance():
    detector = LidarBoundaryDetector(BoundaryConfig(min_valid_points=3))

    result = detector.update_scan([1.1, 1.3, 1.4], angle_min=-0.1, angle_increment=0.1)

    assert result.should_slow is True
    assert result.should_stop is False


def test_should_stop_inside_stop_distance():
    detector = LidarBoundaryDetector(BoundaryConfig(min_valid_points=3))

    result = detector.update_scan([0.5, 0.7, 0.8], angle_min=-0.1, angle_increment=0.1)

    assert result.should_slow is True
    assert result.should_stop is True
    assert result.lane_end is False


def test_lane_end_after_confirm_frames():
    detector = LidarBoundaryDetector(BoundaryConfig(confirm_frames=3, min_valid_points=3))

    results = [
        detector.update_scan([0.5, 0.5, 0.5], angle_min=-0.1, angle_increment=0.1)
        for _ in range(3)
    ]

    assert [result.lane_end for result in results] == [False, False, True]


def test_hit_count_resets_when_clear():
    detector = LidarBoundaryDetector(BoundaryConfig(confirm_frames=2, min_valid_points=3))

    detector.update_scan([0.5, 0.5, 0.5], angle_min=-0.1, angle_increment=0.1)
    clear = detector.update_scan([2.0, 2.0, 2.0], angle_min=-0.1, angle_increment=0.1)
    blocked_again = detector.update_scan([0.5, 0.5, 0.5], angle_min=-0.1, angle_increment=0.1)

    assert clear.lane_end is False
    assert clear.should_stop is False
    assert blocked_again.lane_end is False


def test_invalid_angle_increment_is_rejected():
    detector = LidarBoundaryDetector()

    with pytest.raises(ValueError, match="angle_increment"):
        detector.update_scan([1.0, 1.0, 1.0], angle_min=-0.1, angle_increment=0.0)


def test_reset_clears_confirmed_lane_end():
    detector = LidarBoundaryDetector(BoundaryConfig(confirm_frames=2, min_valid_points=3))
    detector.update_scan([0.5, 0.5, 0.5], angle_min=-0.1, angle_increment=0.1)
    assert detector.update_scan([0.5, 0.5, 0.5], angle_min=-0.1, angle_increment=0.1).lane_end

    detector.reset()

    assert not detector.update_scan([0.5, 0.5, 0.5], angle_min=-0.1, angle_increment=0.1).lane_end
