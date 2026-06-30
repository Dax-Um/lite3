"""ROS2 odometry adapter."""

from math import atan2

from lite3_common.types import Pose2D


def odom_to_pose2d(msg) -> Pose2D:
    pose = msg.pose.pose
    orientation = pose.orientation
    yaw = _yaw_from_quaternion(
        float(orientation.x),
        float(orientation.y),
        float(orientation.z),
        float(orientation.w),
    )
    return Pose2D(float(pose.position.x), float(pose.position.y), yaw)


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return atan2(siny_cosp, cosy_cosp)
