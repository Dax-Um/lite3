"""ROS2 message callback adapter for IQ9 runtime state.

The module is importable without rclpy. A real rclpy node can wire these
callbacks to subscriptions on the target device.
"""

from __future__ import annotations

from math import atan2

from lite3_common.types import Pose2D, Twist2D
from lite3_iq9.runtime_state import RuntimeStateAggregator
from lite3_iq9.state_subscribers import TopicFreshnessMonitor


class Iq9Ros2StateBridge:
    def __init__(
        self,
        aggregator: RuntimeStateAggregator,
        topic_monitor: TopicFreshnessMonitor,
    ) -> None:
        self.aggregator = aggregator
        self.topic_monitor = topic_monitor
        self._last_pose: Pose2D | None = None

    def on_odom(self, msg, stamp_sec: float) -> None:
        self._last_pose = _pose_from_odom(msg)
        self.aggregator.update_pose(self._last_pose, stamp_sec, localization_ok=False)
        self.topic_monitor.mark_seen("/odom", stamp_sec)

    def on_status(self, msg, stamp_sec: float) -> None:
        localization_ok = _status_ok(msg)
        if self._last_pose is not None:
            self.aggregator.update_pose(self._last_pose, stamp_sec, localization_ok)
        self.topic_monitor.mark_seen("/status", stamp_sec)

    def on_map(self, _msg, stamp_sec: float) -> None:
        self.aggregator.update_map(stamp_sec)
        self.topic_monitor.mark_seen("/map", stamp_sec)

    def on_local_costmap(self, _msg, stamp_sec: float) -> None:
        self.aggregator.update_local_costmap(stamp_sec)
        self.topic_monitor.mark_seen("/local_costmap/costmap", stamp_sec)

    def on_global_costmap(self, _msg, stamp_sec: float) -> None:
        self.aggregator.update_global_costmap(stamp_sec)
        self.topic_monitor.mark_seen("/global_costmap/costmap", stamp_sec)

    def on_cmd_vel(self, msg, stamp_sec: float) -> None:
        self.aggregator.update_cmd_vel(_twist_from_msg(msg), stamp_sec)
        self.topic_monitor.mark_seen("/cmd_vel", stamp_sec)

    def on_camera_color(self, _msg, stamp_sec: float) -> None:
        self.aggregator.update_camera(stamp_sec)
        self.topic_monitor.mark_seen("/camera/color/image_raw", stamp_sec)


def _pose_from_odom(msg) -> Pose2D:
    pose = msg.pose.pose
    orientation = pose.orientation
    yaw = _yaw_from_quaternion(
        float(orientation.x),
        float(orientation.y),
        float(orientation.z),
        float(orientation.w),
    )
    return Pose2D(float(pose.position.x), float(pose.position.y), yaw)


def _twist_from_msg(msg) -> Twist2D:
    return Twist2D(
        vx=float(msg.linear.x),
        vy=float(msg.linear.y),
        wz=float(msg.angular.z),
    )


def _status_ok(msg) -> bool:
    if hasattr(msg, "has_converged"):
        return bool(msg.has_converged)
    if hasattr(msg, "data"):
        return bool(msg.data)
    if hasattr(msg, "localization_ok"):
        return bool(msg.localization_ok)
    if hasattr(msg, "status"):
        value = msg.status
        if isinstance(value, str):
            return value.lower() in {"ok", "ready", "localized", "tracking"}
        return bool(value)
    return True


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return atan2(siny_cosp, cosy_cosp)
