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
    CANCEL_SERVICES = (
        "/navigate_to_pose/_action/cancel_goal",
        "/FollowWaypoints/_action/cancel_goal",
        "/follow_path/_action/cancel_goal",
        "/spin/_action/cancel_goal",
        "/backup/_action/cancel_goal",
        "/wait/_action/cancel_goal",
    )

    def __init__(
        self,
        *,
        timeout_sec: float,
        check_period_sec: float,
        reset_quiet_sec: float = 0.25,
    ) -> None:
        super().__init__("lite3_nav_watchdog")
        self.timeout_sec = timeout_sec
        self.reset_quiet_sec = reset_quiet_sec
        self.last_heartbeat = None
        self.armed = False
        self.stale = False
        self.next_cancel_attempt = 0.0
        self.reset_pending = False
        self.reset_token = None
        self.reset_quiet_since = None
        self.cancel_drain_error = False
        self.cancel_clients = {
            service: self.create_client(CancelGoal, service)
            for service in self.CANCEL_SERVICES
        }
        self.cancel_futures = {}
        self.zero_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.reset_ack_pub = self.create_publisher(
            UInt64,
            "/lite3/nav/watchdog_reset_ack",
            10,
        )
        self.arm_ack_pub = self.create_publisher(
            UInt64,
            "/lite3/nav/watchdog_arm_ack",
            10,
        )
        self.create_subscription(UInt64, "/lite3/nav/heartbeat", self._on_heartbeat, 10)
        self.create_subscription(
            UInt64,
            "/lite3/nav/watchdog_reset",
            self._on_reset_request,
            10,
        )
        self.create_timer(check_period_sec, self._check)

    def _on_heartbeat(self, msg) -> None:
        if msg.data == 0:
            self._begin_reset(token=None)
            return
        if self.stale or self.reset_pending:
            self.get_logger().error(
                "nonzero heartbeat ignored while fail-safe is latched; "
                "a completed reset handshake is required"
            )
            return
        self.last_heartbeat = time.monotonic()
        self.armed = True
        self._publish_token(self.arm_ack_pub, int(msg.data))

    def _on_reset_request(self, msg) -> None:
        token = int(msg.data)
        if token == 0:
            self.get_logger().error("watchdog reset token must be nonzero")
            return
        if self.reset_pending and self.reset_token == token:
            return
        self._begin_reset(token=token)

    def _begin_reset(self, *, token) -> None:
        # Stop scheduling cancel requests immediately, but keep the stale latch
        # and zero output until every already-sent RPC has completed.
        self.last_heartbeat = None
        self.armed = False
        self.reset_pending = True
        self.reset_token = token
        self.reset_quiet_since = None

    def _check(self) -> None:
        now = time.monotonic()
        if self.reset_pending:
            if self.stale:
                self.zero_pub.publish(Twist())
            if self._harvest_cancel_futures():
                self.reset_quiet_since = None
                return
            if self.cancel_drain_error or not self._cancel_services_ready():
                self.reset_quiet_since = None
                return
            if self.reset_quiet_since is None:
                self.reset_quiet_since = now
                return
            if now - self.reset_quiet_since < self.reset_quiet_sec:
                return
            token = self.reset_token
            self.cancel_futures.clear()
            self.stale = False
            self.reset_pending = False
            self.reset_token = None
            self.reset_quiet_since = None
            self.next_cancel_attempt = 0.0
            if token is not None:
                self._publish_token(self.reset_ack_pub, token)
            return

        if not self.armed or self.last_heartbeat is None:
            return
        age = now - self.last_heartbeat
        if age <= self.timeout_sec:
            return
        if not self.stale:
            self.stale = True
            self.get_logger().error(
                f"IQ9 navigation heartbeat stale for {age:.3f}s; "
                "holding zero velocity and canceling all navigation goals"
            )
        # Keep sending zero while stale. Cancellation is the primary stop;
        # repeated zero output is a second layer if the action server is slow.
        self.zero_pub.publish(Twist())

        self._harvest_cancel_futures()
        if now < self.next_cancel_attempt:
            return
        self.next_cancel_attempt = now + 1.0
        for service, client in self.cancel_clients.items():
            future = self.cancel_futures.get(service)
            if future is not None:
                continue
            if not client.service_is_ready() and not client.wait_for_service(
                timeout_sec=0.0
            ):
                self.get_logger().error(f"cancel service unavailable: {service}")
                continue
            request = CancelGoal.Request()
            request.goal_info = GoalInfo()
            self.cancel_futures[service] = client.call_async(request)

    def _harvest_cancel_futures(self) -> bool:
        pending = False
        for service, future in list(self.cancel_futures.items()):
            if not future.done():
                pending = True
                continue
            try:
                response = future.result()
            except Exception as exc:
                self.cancel_drain_error = True
                self.get_logger().error(f"cancel failed service={service}: {exc}")
            else:
                if response is None or response.return_code != 0:
                    self.cancel_drain_error = True
                    code = None if response is None else response.return_code
                    self.get_logger().error(
                        f"cancel rejected service={service} return_code={code}"
                    )
            del self.cancel_futures[service]
        return pending

    def _cancel_services_ready(self) -> bool:
        ready = True
        for service, client in self.cancel_clients.items():
            if not client.service_is_ready() and not client.wait_for_service(
                timeout_sec=0.0
            ):
                self.get_logger().error(f"cancel service unavailable: {service}")
                ready = False
        return ready

    @staticmethod
    def _publish_token(publisher, token: int) -> None:
        message = UInt64()
        message.data = token
        publisher.publish(message)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout-sec", type=float, default=3.0)
    parser.add_argument("--check-period-sec", type=float, default=0.2)
    parser.add_argument("--reset-quiet-sec", type=float, default=0.25)
    args = parser.parse_args()
    if min(args.timeout_sec, args.check_period_sec, args.reset_quiet_sec) <= 0.0:
        parser.error("timeout values must be positive")

    rclpy.init(args=None)
    node = NavHeartbeatWatchdog(
        timeout_sec=args.timeout_sec,
        check_period_sec=args.check_period_sec,
        reset_quiet_sec=args.reset_quiet_sec,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
