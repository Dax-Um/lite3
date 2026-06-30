from math import pi

import pytest

from lite3_behavior.patrol_controller import PatrolController
from lite3_common.types import Pose2D, StopReason, Twist2D


def ready_controller(now: float = 10.0) -> PatrolController:
    controller = PatrolController()
    controller.on_odom(Pose2D(0.0, 0.0, 0.0), now)
    controller.on_imu(now)
    controller.on_scan([2.0, 2.0, 2.0], angle_min=-0.1, angle_increment=0.1, now=now)
    return controller


def test_patrol_start_requires_odom():
    controller = PatrolController()

    with pytest.raises(RuntimeError, match="odom"):
        controller.on_operator_command("patrol_start", now=10.0)


def test_patrol_start_stores_home_pose():
    controller = ready_controller()

    controller.on_operator_command("patrol_start", now=10.0)

    assert controller.odom_tracker.home_pose() == Pose2D(0.0, 0.0, 0.0)


def test_lane_end_transitions_to_shift():
    controller = ready_controller()
    controller.on_operator_command("patrol_start", now=10.0)
    controller.tick(now=10.0)

    controller.on_scan([0.5, 0.5, 0.5], angle_min=-0.1, angle_increment=0.1, now=10.1)
    output = controller.tick(now=10.1)
    controller.on_scan([2.0, 2.0, 2.0], angle_min=-0.1, angle_increment=0.1, now=10.2)
    output = controller.tick(now=10.2)
    output = controller.tick(now=10.3)

    assert output.state == "shift_to_next_lane"
    assert output.safe_cmd == Twist2D(0.0, 0.04, 0.0)


def test_side_shift_done_from_odom_distance():
    controller = ready_controller()
    controller.on_operator_command("patrol_start", now=10.0)
    controller.tick(now=10.0)
    controller.on_scan([0.5, 0.5, 0.5], angle_min=-0.1, angle_increment=0.1, now=10.1)
    controller.tick(now=10.1)
    controller.on_scan([2.0, 2.0, 2.0], angle_min=-0.1, angle_increment=0.1, now=10.2)
    controller.tick(now=10.2)

    controller.on_odom(Pose2D(0.6, 0.0, 0.0), now=10.3)
    output = controller.tick(now=10.3)

    assert output.state == "turn_around"


def test_turn_done_from_yaw_delta():
    controller = ready_controller()
    controller.on_operator_command("patrol_start", now=10.0)
    controller.tick(now=10.0)
    controller.on_scan([0.5, 0.5, 0.5], angle_min=-0.1, angle_increment=0.1, now=10.1)
    controller.tick(now=10.1)
    controller.on_scan([2.0, 2.0, 2.0], angle_min=-0.1, angle_increment=0.1, now=10.2)
    controller.tick(now=10.2)
    controller.on_odom(Pose2D(0.6, 0.0, 0.0), now=10.3)
    controller.tick(now=10.3)

    controller.on_odom(Pose2D(0.6, 0.0, pi), now=10.4)
    output = controller.tick(now=10.4)

    assert output.state == "move_along_lane"
    assert output.lane_index == 1


def test_return_home_preempts_patrol():
    controller = ready_controller()
    controller.on_operator_command("patrol_start", now=10.0)
    controller.tick(now=10.0)
    controller.on_odom(Pose2D(1.0, 0.0, pi), now=10.1)

    controller.on_operator_command("return_home", now=10.1)
    output = controller.tick(now=10.1)

    assert output.return_home_active is True
    assert output.raw_cmd.vx > 0.0
    assert output.state == "return_home"


def test_lidar_timeout_outputs_stop():
    controller = ready_controller(now=10.0)
    controller.on_operator_command("patrol_start", now=10.0)

    output = controller.tick(now=10.6)

    assert output.safe_cmd == Twist2D(0.0, 0.0, 0.0)
    assert output.stop_reason is StopReason.LIDAR_TIMEOUT


def test_odom_timeout_outputs_stop():
    controller = ready_controller(now=10.0)
    controller.on_operator_command("patrol_start", now=10.0)
    controller.on_scan([2.0, 2.0, 2.0], angle_min=-0.1, angle_increment=0.1, now=10.6)

    output = controller.tick(now=10.6)

    assert output.safe_cmd == Twist2D(0.0, 0.0, 0.0)
    assert output.stop_reason is StopReason.ODOM_TIMEOUT


def test_emergency_stop_outputs_stop():
    controller = ready_controller()
    controller.on_operator_command("patrol_start", now=10.0)

    controller.on_operator_command("emergency_stop", now=10.1)
    output = controller.tick(now=10.1)

    assert output.safe_cmd == Twist2D(0.0, 0.0, 0.0)
    assert output.stop_reason is StopReason.EMERGENCY_STOP


def test_finish_outputs_zero_command():
    controller = ready_controller()
    controller.on_operator_command("patrol_start", now=10.0)
    controller.fsm._state = controller.fsm.state().FINISH

    output = controller.tick(now=10.0)

    assert output.raw_cmd == Twist2D(0.0, 0.0, 0.0)
    assert output.safe_cmd == Twist2D(0.0, 0.0, 0.0)
