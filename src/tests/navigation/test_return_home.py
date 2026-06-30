import pytest

from lite3_common.types import PathPoint, Pose2D, Twist2D
from lite3_navigation.return_home import ReturnHomeConfig, ReturnHomeController


def controller() -> ReturnHomeController:
    return ReturnHomeController(
        ReturnHomeConfig(
            home_position_tolerance_m=0.25,
            home_yaw_tolerance_rad=0.17,
            face_home_yaw_tolerance_rad=0.25,
            max_vx_mps=0.12,
            max_wz_radps=0.20,
        )
    )


def test_start_direct_return_uses_home_pose():
    return_home = controller()
    home = Pose2D(0.0, 0.0, 0.0)
    path_trace = [PathPoint(home, 10.0)]

    return_home.start(home, path_trace)

    assert return_home.active() is True


def test_turn_to_home_before_drive():
    return_home = controller()
    return_home.start(Pose2D(0.0, 0.0, 0.0))

    cmd, done = return_home.tick(Pose2D(1.0, 0.0, 0.0))

    assert done is False
    assert cmd.vx == 0.0
    assert cmd.vy == 0.0
    assert cmd.wz == pytest.approx(0.20)


def test_drive_to_home_uses_target_yaw():
    return_home = controller()
    return_home.start(Pose2D(0.0, 0.0, 0.0))

    cmd, done = return_home.tick(Pose2D(1.0, 0.0, 3.141592653589793))

    assert done is False
    assert cmd.vx == pytest.approx(0.12)
    assert cmd.vy == 0.0
    assert abs(cmd.wz) <= 0.20


def test_final_align_requires_yaw_tolerance():
    return_home = controller()
    return_home.start(Pose2D(0.0, 0.0, 1.0))

    cmd, done = return_home.tick(Pose2D(0.1, 0.0, 0.0))

    assert done is False
    assert cmd.vx == 0.0
    assert cmd.vy == 0.0
    assert cmd.wz == pytest.approx(0.20)


def test_done_after_position_and_yaw_tolerance():
    return_home = controller()
    return_home.start(Pose2D(0.0, 0.0, 1.0))

    cmd, done = return_home.tick(Pose2D(0.1, 0.0, 0.9))

    assert cmd == Twist2D(0.0, 0.0, 0.0)
    assert done is True
    assert return_home.active() is False


def test_cancel_makes_controller_inactive():
    return_home = controller()
    return_home.start(Pose2D(0.0, 0.0, 0.0))

    return_home.cancel()

    assert return_home.active() is False
    assert return_home.tick(Pose2D(1.0, 0.0, 0.0)) == (Twist2D(0.0, 0.0, 0.0), True)
