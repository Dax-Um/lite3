"""Actual ROS2 rclpy node wiring for Lite3 LiDAR patrol runtime."""

from dataclasses import dataclass
import time

from lite3_control.readiness_gate import (
    ReadinessGate,
    ReadinessInput,
    ReadinessResult,
)
from lite3_ros.patrol_node import PatrolRosBridge


try:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import Imu, LaserScan
    from std_msgs.msg import String
except ImportError:
    rclpy = None
    Twist = None
    Odometry = None
    Node = object
    Imu = None
    LaserScan = None
    String = None


@dataclass
class TopicTimestamps:
    scan: float | None = None
    odom: float | None = None
    imu: float | None = None


@dataclass(frozen=True)
class RuntimeFlags:
    motion_host_reachable: bool = False
    preflight_ok: bool = False
    auto_mode_ok: bool = False
    stand_ready_ok: bool = False


def build_readiness_input(
    *,
    now: float,
    timestamps: TopicTimestamps,
    flags: RuntimeFlags,
) -> ReadinessInput:
    return ReadinessInput(
        now=now,
        scan_last_seen=timestamps.scan,
        odom_last_seen=timestamps.odom,
        imu_last_seen=timestamps.imu,
        motion_host_reachable=flags.motion_host_reachable,
        preflight_ok=flags.preflight_ok,
        auto_mode_ok=flags.auto_mode_ok,
        stand_ready_ok=flags.stand_ready_ok,
    )


def format_status_text(output, readiness: ReadinessResult) -> str:
    reasons = "|".join(readiness.reasons) if readiness.reasons else "none"
    boundary = (
        ""
        if output.boundary_min_front_m is None
        else f"{output.boundary_min_front_m:.3f}"
    )
    return " ".join(
        [
            f"ready={_bool_text(readiness.ready)}",
            f"reasons={reasons}",
            f"state={output.state}",
            f"lane_index={output.lane_index}",
            f"return_home={_bool_text(output.return_home_active)}",
            f"raw_cmd={output.raw_cmd.vx:.3f},{output.raw_cmd.vy:.3f},{output.raw_cmd.wz:.3f}",
            f"safe_cmd={output.safe_cmd.vx:.3f},{output.safe_cmd.vy:.3f},{output.safe_cmd.wz:.3f}",
            f"stop_reason={output.stop_reason.value}",
            f"boundary_min_front_m={boundary}",
        ]
    )


class PatrolRclpyNode(Node):
    def __init__(
        self,
        *,
        scan_topic: str = "/scan",
        odom_topic: str = "/leg_odom2",
        imu_topic: str = "/imu/data",
        operator_topic: str = "/lite3/patrol/operator_command",
        status_topic: str = "/lite3/patrol/status",
        cmd_vel_safe_topic: str = "/lite3/patrol/cmd_vel_safe",
        bridge: PatrolRosBridge | None = None,
        readiness_gate: ReadinessGate | None = None,
        runtime_flags: RuntimeFlags = RuntimeFlags(),
        motion_output=None,
        timer_period_sec: float = 0.05,
    ):
        if rclpy is None:
            raise RuntimeError("rclpy is required to create PatrolRclpyNode")
        super().__init__("lite3_lidar_auto_patrol")
        self.bridge = bridge or PatrolRosBridge()
        self.readiness_gate = readiness_gate or ReadinessGate()
        self.runtime_flags = runtime_flags
        self.motion_output = motion_output
        self.timestamps = TopicTimestamps()

        self.status_publisher = self.create_publisher(String, status_topic, 10)
        self.cmd_vel_safe_publisher = self.create_publisher(Twist, cmd_vel_safe_topic, 10)

        self.create_subscription(LaserScan, scan_topic, self._on_scan, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(Imu, imu_topic, self._on_imu, 10)
        self.create_subscription(String, operator_topic, self._on_operator_command, 10)
        self.create_timer(timer_period_sec, self._tick)

    def _now(self) -> float:
        return time.monotonic()

    def _on_scan(self, msg) -> None:
        now = self._now()
        self.timestamps.scan = now
        self.bridge.handle_scan(msg, now)

    def _on_odom(self, msg) -> None:
        now = self._now()
        self.timestamps.odom = now
        self.bridge.handle_odom(msg, now)

    def _on_imu(self, _msg) -> None:
        now = self._now()
        self.timestamps.imu = now
        self.bridge.handle_imu(now)

    def _on_operator_command(self, msg) -> None:
        self.bridge.handle_operator_command(str(msg.data), self._now())

    def _tick(self) -> None:
        now = self._now()
        output = self.bridge.tick(now)
        readiness = self.readiness_gate.check(
            build_readiness_input(
                now=now,
                timestamps=self.timestamps,
                flags=self.runtime_flags,
            )
        )
        self.status_publisher.publish(String(data=format_status_text(output, readiness)))
        self.cmd_vel_safe_publisher.publish(_to_twist_msg(output.safe_cmd))
        if self.motion_output is not None and readiness.ready:
            self.motion_output.publish(output)


def spin_node(node: PatrolRclpyNode | None = None) -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is required to spin PatrolRclpyNode")
    rclpy.init(args=None)
    actual_node = node or PatrolRclpyNode()
    try:
        rclpy.spin(actual_node)
    finally:
        actual_node.destroy_node()
        rclpy.shutdown()


def _to_twist_msg(cmd):
    msg = Twist()
    msg.linear.x = cmd.vx
    msg.linear.y = cmd.vy
    msg.angular.z = cmd.wz
    return msg


def _bool_text(value: bool) -> str:
    return "true" if value else "false"
