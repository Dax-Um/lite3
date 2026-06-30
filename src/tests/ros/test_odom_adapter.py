from types import SimpleNamespace

import pytest

from lite3_ros.odom_adapter import odom_to_pose2d


def test_odom_to_pose2d_extracts_position_and_yaw():
    msg = SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=1.0, y=2.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        )
    )

    pose = odom_to_pose2d(msg)

    assert pose.x == 1.0
    assert pose.y == 2.0
    assert pose.yaw == pytest.approx(0.0)
