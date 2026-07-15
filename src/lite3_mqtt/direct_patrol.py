"""Minimal MQTT-triggered triangle patrol for ROS 2 Foxy.

Nav2 owns planning, obstacle avoidance, recovery, and goal completion.  This
module only captures the current map pose and sends all three triangle
vertices in one ``/FollowWaypoints`` goal.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import replace
from pathlib import Path
from typing import Dict, Optional, Union

from lite3_mqtt.patrol import PatrolConfig, Waypoint, WaypointRoute


class DirectPatrolController:
    """Run one generated route repeatedly until STOP or an action failure."""

    def __init__(
        self,
        *,
        backend,
        patrol_config: Union[str, Path],
        max_loops: int = 0,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        if max_loops < 0:
            raise ValueError("max_loops must be >= 0")
        self.backend = backend
        self.config = PatrolConfig.from_yaml(patrol_config)
        self.max_loops = max_loops
        self.logger = logger or logging.getLogger(__name__)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._home = None
        self._emergency_latched = False

    @property
    def active(self) -> bool:
        with self._lock:
            return self._thread is not None and self._thread.is_alive()

    @property
    def home(self) -> Optional[Waypoint]:
        with self._lock:
            return self._home

    @property
    def emergency_latched(self) -> bool:
        with self._lock:
            return self._emergency_latched

    def start(self) -> bool:
        with self._lock:
            if self._emergency_latched:
                self.logger.error("START rejected: emergency stop is latched; RESET required")
                return False
            if self._thread is not None and self._thread.is_alive():
                return False
            self._stop.clear()
            self.backend.prepare_route()
            self._thread = threading.Thread(
                target=self._run,
                name="mqtt-direct-triangle-patrol",
                daemon=True,
            )
            self._thread.start()
        return True

    def stop(self) -> None:
        self._stop.set()
        self.backend.cancel_active()

    def prepare_motion(self) -> None:
        """Detection owns a different path; do not probe or mutate Nav2 here."""
        self.stop()

    def return_home(self) -> bool:
        with self._lock:
            home = self._home
            active = self._thread
            if self._emergency_latched or home is None:
                return False
        self.stop()
        thread = threading.Thread(
            target=self._return_home,
            args=(active, home),
            name="mqtt-direct-return-home",
            daemon=True,
        )
        thread.start()
        return True

    def emergency_stop(self) -> None:
        with self._lock:
            self._emergency_latched = True
        self.stop()

    def reset(self) -> None:
        self.stop()
        with self._lock:
            self._emergency_latched = False
            self._home = None

    def close(self, timeout_sec: float = 5.0) -> None:
        self.stop()
        with self._lock:
            thread = self._thread
        if thread is not None:
            thread.join(timeout=timeout_sec)

    def _run(self) -> None:
        try:
            home = self.backend.capture_current_pose(waypoint_id="home")
            if self._stop.is_set():
                return
            route = self.config.build_route(home)
            with self._lock:
                self._home = home
            self.logger.info(
                "patrol started home=(%.3f, %.3f, %.3f) waypoints=%s",
                home.x,
                home.y,
                home.yaw,
                [(point.id, point.x, point.y, point.yaw) for point in route.waypoints],
            )
            loops = 0
            while not self._stop.is_set():
                self.backend.prepare_route()
                if self._stop.is_set():
                    break
                result = self.backend.send_route(route)
                if self._stop.is_set():
                    break
                if not _succeeded(result):
                    self.logger.error("patrol route failed: %s", result)
                    break
                loops += 1
                if self.max_loops and loops >= self.max_loops:
                    break
                self.logger.info("patrol route completed; starting next loop")
        except Exception:
            self.logger.exception("continuous patrol stopped by error")
        finally:
            with self._lock:
                if self._thread is threading.current_thread():
                    self._thread = None
            self.logger.info("patrol loop stopped")

    def _return_home(self, active, home: Waypoint) -> None:
        if active is not None:
            active.join(timeout=5.0)
            if active.is_alive():
                self.logger.error("return-home aborted: patrol goal did not cancel")
                return
        try:
            current = self.backend.capture_current_pose(waypoint_id="current")
            yaw = math.atan2(home.y - current.y, home.x - current.x)
            route = WaypointRoute(
                route_id="return_home",
                frame_id=self.config.frame_id,
                loop=False,
                waypoints=[replace(home, id="home_return", yaw=yaw)],
            )
            self.backend.prepare_route()
            result = self.backend.send_route(route)
            if not _succeeded(result):
                self.logger.error("return-home route failed: %s", result)
        except Exception:
            self.logger.exception("return-home failed")


class DirectMockPatrolBackend:
    def __init__(
        self,
        *,
        home: Optional[Waypoint] = None,
        route_duration_sec: float = 0.1,
    ) -> None:
        self.current_pose = home or Waypoint("home", 0.0, 0.0, 0.0)
        self.route_duration_sec = route_duration_sec
        self.routes = []
        self._cancel = threading.Event()

    def prepare_route(self) -> None:
        self._cancel.clear()

    def capture_current_pose(self, *, waypoint_id: str) -> Waypoint:
        return replace(self.current_pose, id=waypoint_id)

    def send_route(self, route: WaypointRoute) -> Dict[str, object]:
        self.routes.append(route)
        if self._cancel.wait(max(0.0, self.route_duration_sec)):
            return {"accepted": True, "status": "CANCELED", "missed_waypoints": []}
        self.current_pose = replace(route.waypoints[-1], id="current")
        return {"accepted": True, "status": 4, "missed_waypoints": []}

    def cancel_active(self) -> None:
        self._cancel.set()


class DirectNav2PatrolBackend:
    """Use only ``/odom`` and one three-pose ``/FollowWaypoints`` action."""

    def __init__(
        self,
        *,
        odom_topic: str = "/odom",
        action_name: str = "/FollowWaypoints",
        timeout_sec: float = 10.0,
        route_timeout_sec: float = 300.0,
        cancel_timeout_sec: float = 5.0,
    ) -> None:
        if min(timeout_sec, route_timeout_sec, cancel_timeout_sec) <= 0.0:
            raise ValueError("Nav2 timeouts must be positive")
        self.odom_topic = odom_topic
        self.action_name = action_name
        self.timeout_sec = timeout_sec
        self.route_timeout_sec = route_timeout_sec
        self.cancel_timeout_sec = cancel_timeout_sec
        self._cancel = threading.Event()

    def prepare_route(self) -> None:
        self._cancel.clear()

    def capture_current_pose(self, *, waypoint_id: str) -> Waypoint:
        import rclpy
        from nav_msgs.msg import Odometry

        rclpy.init(args=None)
        node = rclpy.create_node("lite3_direct_pose_capture")
        captured = {"pose": None, "error": None}

        def on_odom(message) -> None:
            position = message.pose.pose.position
            orientation = message.pose.pose.orientation
            values = (
                position.x,
                position.y,
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            )
            if not all(math.isfinite(float(value)) for value in values):
                captured["error"] = "{} contains a non-finite pose".format(
                    self.odom_topic
                )
                return
            yaw = math.atan2(
                2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
            )
            captured["pose"] = Waypoint(
                waypoint_id,
                float(position.x),
                float(position.y),
                yaw,
            )

        subscription = node.create_subscription(Odometry, self.odom_topic, on_odom, 10)
        deadline = time.monotonic() + self.timeout_sec
        try:
            while (
                rclpy.ok()
                and captured["pose"] is None
                and captured["error"] is None
                and time.monotonic() < deadline
            ):
                rclpy.spin_once(node, timeout_sec=0.05)
            if captured["error"] is not None:
                raise RuntimeError(str(captured["error"]))
            if captured["pose"] is None:
                raise TimeoutError("timed out waiting for {}".format(self.odom_topic))
            return captured["pose"]
        finally:
            node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()

    def send_route(self, route: WaypointRoute) -> Dict[str, object]:
        import rclpy
        from nav2_msgs.action import FollowWaypoints
        from rclpy.action import ActionClient

        rclpy.init(args=None)
        node = rclpy.create_node("lite3_direct_triangle_patrol")
        client = ActionClient(node, FollowWaypoints, self.action_name)
        route_deadline = time.monotonic() + self.route_timeout_sec
        try:
            if not client.wait_for_server(timeout_sec=self.timeout_sec):
                raise TimeoutError("{} action server is unavailable".format(self.action_name))
            if self._cancel.is_set():
                return _canceled_result(False, 0)

            goal = FollowWaypoints.Goal()
            goal.poses = [
                _pose_stamped(node, route.frame_id, waypoint)
                for waypoint in route.waypoints
            ]
            send_future = client.send_goal_async(goal)
            acceptance_deadline = min(
                time.monotonic() + self.timeout_sec,
                route_deadline,
            )
            while rclpy.ok() and not send_future.done():
                rclpy.spin_once(node, timeout_sec=0.05)
                if time.monotonic() >= acceptance_deadline:
                    raise TimeoutError("FollowWaypoints goal acceptance timed out")
            goal_handle = send_future.result()
            if goal_handle is None or not goal_handle.accepted:
                return {
                    "accepted": False,
                    "status": None,
                    "missed_waypoints": list(range(len(route.waypoints))),
                    "reason": "goal_rejected",
                }

            logging.getLogger(__name__).info(
                "Nav2 FollowWaypoints goal sent count=%s waypoints=%s",
                len(route.waypoints),
                [(point.id, point.x, point.y, point.yaw) for point in route.waypoints],
            )
            result_future = goal_handle.get_result_async()
            cancel_future = None
            cancel_deadline = None
            cancel_reason = None
            while rclpy.ok() and not result_future.done():
                rclpy.spin_once(node, timeout_sec=0.05)
                now = time.monotonic()
                if self._cancel.is_set() and cancel_future is None:
                    cancel_future = goal_handle.cancel_goal_async()
                    cancel_deadline = now + self.cancel_timeout_sec
                    cancel_reason = "operator_cancel"
                if now >= route_deadline and cancel_future is None:
                    cancel_future = goal_handle.cancel_goal_async()
                    cancel_deadline = now + self.cancel_timeout_sec
                    cancel_reason = "route_timeout"
                if cancel_deadline is not None and now >= cancel_deadline:
                    return {
                        "accepted": True,
                        "status": "CANCEL_TIMEOUT",
                        "missed_waypoints": [],
                        "reason": cancel_reason,
                    }

            if not result_future.done() or self._cancel.is_set():
                return _canceled_result(True, 0)
            wrapped = result_future.result()
            status = None if wrapped is None else wrapped.status
            action_result = None if wrapped is None else wrapped.result
            missed = (
                []
                if action_result is None
                else list(getattr(action_result, "missed_waypoints", []))
            )
            if status != 4 or missed:
                return {
                    "accepted": True,
                    "status": status,
                    "missed_waypoints": missed,
                    "reason": "nav2_goal_failed",
                }
            return {
                "accepted": True,
                "status": 4,
                "missed_waypoints": [],
                "reason": None,
            }
        finally:
            client.destroy()
            node.destroy_node()
            rclpy.shutdown()

    def cancel_active(self) -> None:
        self._cancel.set()


def _pose_stamped(node, frame_id: str, waypoint: Waypoint):
    from geometry_msgs.msg import PoseStamped

    pose = PoseStamped()
    pose.header.frame_id = frame_id
    # IQ9 and the perception host do not share a system clock.  A zero stamp
    # tells Nav2/tf2 to use the latest transform, matching a local RViz goal.
    pose.header.stamp.sec = 0
    pose.header.stamp.nanosec = 0
    pose.pose.position.x = waypoint.x
    pose.pose.position.y = waypoint.y
    pose.pose.orientation.z = math.sin(waypoint.yaw / 2.0)
    pose.pose.orientation.w = math.cos(waypoint.yaw / 2.0)
    return pose


def _canceled_result(accepted: bool, index: int) -> Dict[str, object]:
    return {
        "accepted": accepted,
        "status": "CANCELED",
        "missed_waypoints": [index],
        "reason": "operator_cancel",
    }


def _succeeded(result: Dict[str, object]) -> bool:
    return bool(result.get("accepted")) and result.get("status") in {4, "SUCCEEDED"}
