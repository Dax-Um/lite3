"""ROS2 patrol node adapter.

This module intentionally keeps the import-time surface free of ``rclpy`` so
core controller wiring can be unit-tested on non-target machines. The target
runtime can wrap ``PatrolRosBridge`` in an actual ROS2 node.
"""

from lite3_behavior.patrol_controller import ControllerOutput, PatrolController
from lite3_ros.lidar_adapter import scan_to_boundary_input
from lite3_ros.odom_adapter import odom_to_pose2d


class PatrolRosBridge:
    def __init__(self, controller: PatrolController | None = None):
        self.controller = controller or PatrolController()

    def handle_odom(self, msg, now: float) -> None:
        self.controller.on_odom(odom_to_pose2d(msg), now)

    def handle_imu(self, now: float) -> None:
        self.controller.on_imu(now)

    def handle_scan(self, msg, now: float) -> None:
        ranges, angle_min, angle_increment = scan_to_boundary_input(msg)
        self.controller.on_scan(ranges, angle_min, angle_increment, now)

    def handle_operator_command(self, command: str, now: float) -> None:
        self.controller.on_operator_command(command, now)

    def tick(self, now: float) -> ControllerOutput:
        return self.controller.tick(now)
