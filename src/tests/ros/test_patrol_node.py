from types import SimpleNamespace

from lite3_common.types import StopReason
from lite3_ros.patrol_node import PatrolRosBridge


def odom_msg(x: float, y: float, z: float = 0.0, w: float = 1.0):
    return SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=z, w=w),
            )
        )
    )


def scan_msg(ranges):
    return SimpleNamespace(ranges=ranges, angle_min=-0.1, angle_increment=0.1)


def test_patrol_ros_bridge_connects_topics_to_controller():
    bridge = PatrolRosBridge()

    bridge.handle_odom(odom_msg(0.0, 0.0), now=10.0)
    bridge.handle_imu(now=10.0)
    bridge.handle_scan(scan_msg([2.0, 2.0, 2.0]), now=10.0)
    bridge.handle_operator_command("patrol_start", now=10.0)
    output = bridge.tick(now=10.0)

    assert output.stop_reason is StopReason.NONE
    assert output.state == "move_along_lane"
