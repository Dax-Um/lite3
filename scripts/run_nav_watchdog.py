#!/usr/bin/env python3
"""Perception-host watchdog for an IQ9-owned Nav2 patrol action."""

from __future__ import annotations

import argparse
import time

import rclpy
from action_msgs.msg import GoalInfo
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import UInt64


class NavHeartbeatWatchdog(Node):
    def __init__(self, *, timeout_sec: float, check_period_sec: float) -> None:
        super().__init__("lite3_nav_watchdog")
        self.timeout_sec = timeout_sec
        self.last_heartbeat = None
        self.armed = False
        self.cancel_client = self.create_client(
            CancelGoal,
            "/FollowWaypoints/_action/cancel_goal",
        )
        self.zero_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(UInt64, "/lite3/nav/heartbeat", self._on_heartbeat, 10)
        self.create_timer(check_period_sec, self._check)

    def _on_heartbeat(self, _msg) -> None:
        self.last_heartbeat = time.monotonic()
        self.armed = True

    def _check(self) -> None:
        if not self.armed or self.last_heartbeat is None:
            return
        age = time.monotonic() - self.last_heartbeat
        if age <= self.timeout_sec:
            return
        self.armed = False
        self.get_logger().error(
            f"IQ9 navigation heartbeat stale for {age:.3f}s; "
            "canceling all waypoint goals"
        )
        self.zero_pub.publish(Twist())
        if not self.cancel_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().error("FollowWaypoints cancel service is unavailable")
            return
        request = CancelGoal.Request()
        request.goal_info = GoalInfo()
        self.cancel_client.call_async(request)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout-sec", type=float, default=3.0)
    parser.add_argument("--check-period-sec", type=float, default=0.2)
    args = parser.parse_args()
    if args.timeout_sec <= 0.0 or args.check_period_sec <= 0.0:
        parser.error("timeout values must be positive")

    rclpy.init(args=None)
    node = NavHeartbeatWatchdog(
        timeout_sec=args.timeout_sec,
        check_period_sec=args.check_period_sec,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
