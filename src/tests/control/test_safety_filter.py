import pytest

from lite3_common.types import StopReason, Twist2D
from lite3_control.safety_filter import SafetyConfig, SafetyFilter


ZERO = Twist2D(0.0, 0.0, 0.0)


def safe_filter(now: float = 10.0) -> SafetyFilter:
    safety_filter = SafetyFilter(SafetyConfig())
    safety_filter.mark_command(now)
    safety_filter.update_lidar(now)
    safety_filter.update_odom(now)
    safety_filter.update_imu(now)
    return safety_filter


def test_emergency_stop_returns_zero():
    safety_filter = safe_filter()
    safety_filter.set_emergency_stop(True)

    cmd, reason = safety_filter.filter_cmd(Twist2D(0.01, 0.0, 0.0), now=10.0)

    assert cmd == ZERO
    assert reason is StopReason.EMERGENCY_STOP


def test_missing_lidar_timestamp_returns_lidar_timeout():
    safety_filter = SafetyFilter(SafetyConfig())
    safety_filter.mark_command(10.0)
    safety_filter.update_odom(10.0)
    safety_filter.update_imu(10.0)

    cmd, reason = safety_filter.filter_cmd(Twist2D(0.01, 0.0, 0.0), now=10.0)

    assert cmd == ZERO
    assert reason is StopReason.LIDAR_TIMEOUT


def test_lidar_timeout_returns_zero():
    safety_filter = safe_filter(now=10.0)
    safety_filter.mark_command(10.51)

    cmd, reason = safety_filter.filter_cmd(Twist2D(0.01, 0.0, 0.0), now=10.51)

    assert cmd == ZERO
    assert reason is StopReason.LIDAR_TIMEOUT


def test_odom_timeout_returns_zero():
    safety_filter = safe_filter(now=10.0)
    safety_filter.mark_command(10.51)
    safety_filter.update_lidar(10.51)
    safety_filter.update_imu(10.51)

    cmd, reason = safety_filter.filter_cmd(Twist2D(0.01, 0.0, 0.0), now=10.51)

    assert cmd == ZERO
    assert reason is StopReason.ODOM_TIMEOUT


def test_imu_timeout_returns_zero():
    safety_filter = safe_filter(now=10.0)
    safety_filter.mark_command(10.51)
    safety_filter.update_lidar(10.51)
    safety_filter.update_odom(10.51)

    cmd, reason = safety_filter.filter_cmd(Twist2D(0.01, 0.0, 0.0), now=10.51)

    assert cmd == ZERO
    assert reason is StopReason.IMU_TIMEOUT


def test_front_obstacle_returns_zero():
    safety_filter = safe_filter()
    safety_filter.set_front_obstacle(True)

    cmd, reason = safety_filter.filter_cmd(Twist2D(0.01, 0.0, 0.0), now=10.0)

    assert cmd == ZERO
    assert reason is StopReason.FRONT_OBSTACLE


def test_velocity_clamp_applies_when_safe():
    safety_filter = safe_filter()

    cmd, reason = safety_filter.filter_cmd(Twist2D(9.0, -9.0, 9.0), now=10.0)

    assert cmd == pytest.approx(Twist2D(0.10, -0.05, 0.20))
    assert reason is StopReason.NONE


def test_stop_priority_emergency_before_timeout():
    safety_filter = SafetyFilter(SafetyConfig())
    safety_filter.set_emergency_stop(True)

    cmd, reason = safety_filter.filter_cmd(Twist2D(0.01, 0.0, 0.0), now=10.0)

    assert cmd == ZERO
    assert reason is StopReason.EMERGENCY_STOP


def test_mark_command_resets_command_timeout():
    safety_filter = safe_filter(now=10.0)

    cmd, reason = safety_filter.filter_cmd(Twist2D(0.01, 0.0, 0.0), now=10.31)

    assert cmd == ZERO
    assert reason is StopReason.COMMAND_TIMEOUT

    safety_filter.mark_command(10.31)
    cmd, reason = safety_filter.filter_cmd(Twist2D(0.01, 0.0, 0.0), now=10.31)

    assert cmd == Twist2D(0.01, 0.0, 0.0)
    assert reason is StopReason.NONE


def test_sensor_timeouts_can_be_disabled_for_dry_run():
    safety_filter = SafetyFilter(
        SafetyConfig(
            require_lidar=False,
            require_odom=False,
            require_imu=False,
        )
    )
    safety_filter.mark_command(10.0)

    cmd, reason = safety_filter.filter_cmd(Twist2D(0.01, 0.0, 0.0), now=10.0)

    assert cmd == Twist2D(0.01, 0.0, 0.0)
    assert reason is StopReason.NONE
