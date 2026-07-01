import pytest

from lite3_behavior.patrol_controller import ControllerOutput
from lite3_common.types import StopReason, Twist2D
from lite3_control.runtime_motion_output import (
    RuntimeMotionOutput,
    RuntimeOutputConfig,
)


class FakeDriver:
    def __init__(self):
        self.calls = []
        self.raise_on_send = False

    def send_cmd_vel(self, vx, vy, wz):
        self.calls.append(("send_cmd_vel", vx, vy, wz))
        if self.raise_on_send:
            raise RuntimeError("send failed")

    def stop(self, repeat, dt_sec):
        self.calls.append(("stop", repeat, dt_sec))


def output(
    *,
    safe_cmd=Twist2D(0.1, 0.0, 0.2),
    stop_reason=StopReason.NONE,
):
    return ControllerOutput(
        raw_cmd=Twist2D(0.0, 0.0, 0.0),
        safe_cmd=safe_cmd,
        state="move_along_lane",
        stop_reason=stop_reason,
        lane_index=0,
        return_home_active=False,
        boundary_min_front_m=None,
    )


def test_publish_sends_safe_cmd_when_no_stop_reason():
    driver = FakeDriver()
    publisher = RuntimeMotionOutput(driver)

    publisher.publish(output(safe_cmd=Twist2D(0.12, -0.03, 0.4)))

    assert driver.calls == [("send_cmd_vel", 0.12, -0.03, 0.4)]


def test_publish_repeats_stop_when_lidar_timeout():
    driver = FakeDriver()
    publisher = RuntimeMotionOutput(
        driver,
        RuntimeOutputConfig(stop_repeat=7, stop_dt_sec=0.02),
    )

    publisher.publish(output(stop_reason=StopReason.LIDAR_TIMEOUT))

    assert driver.calls == [("stop", 7, 0.02)]


def test_publish_repeats_stop_when_front_obstacle():
    driver = FakeDriver()
    publisher = RuntimeMotionOutput(
        driver,
        RuntimeOutputConfig(stop_repeat=12, stop_dt_sec=0.03),
    )

    publisher.publish(output(stop_reason=StopReason.FRONT_OBSTACLE))

    assert driver.calls == [("stop", 12, 0.03)]


def test_publish_sends_zero_velocity_once_when_no_stop_reason():
    driver = FakeDriver()
    publisher = RuntimeMotionOutput(driver)

    publisher.publish(output(safe_cmd=Twist2D(0.0, 0.0, 0.0)))

    assert driver.calls == [("send_cmd_vel", 0.0, 0.0, 0.0)]


def test_publish_attempts_stop_after_send_exception():
    driver = FakeDriver()
    driver.raise_on_send = True
    publisher = RuntimeMotionOutput(
        driver,
        RuntimeOutputConfig(stop_repeat=5, stop_dt_sec=0.01),
    )

    with pytest.raises(RuntimeError, match="send failed"):
        publisher.publish(output())

    assert driver.calls == [
        ("send_cmd_vel", 0.1, 0.0, 0.2),
        ("stop", 5, 0.01),
    ]
