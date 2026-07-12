"""Python 3.8-compatible continuous patrol for the ROS2 Foxy container."""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Dict, List, Optional, Protocol, Union

import yaml


@dataclass(frozen=True)
class Waypoint:
    id: str
    x: float
    y: float
    yaw: float
    dwell_sec: float = 0.0


@dataclass(frozen=True)
class WaypointRoute:
    route_id: str
    frame_id: str
    loop: bool
    waypoints: List[Waypoint]


@dataclass(frozen=True)
class PatrolOffset:
    dx: float
    dy: float
    yaw_offset: float = 0.0
    dwell_sec: float = 0.0


@dataclass(frozen=True)
class PatrolSegment:
    distance_m: float
    turn_rad: float = 0.0
    dwell_sec: float = 0.0


@dataclass(frozen=True)
class PatrolConfig:
    route_id: str
    frame_id: str
    min_distance_m: float
    equilateral_triangle_side_m: Optional[float]
    equilateral_triangle_heading_deg: Optional[float]
    forward_distances_m: List[float]
    offsets: List[PatrolOffset]
    segments: List[PatrolSegment]

    @classmethod
    def from_yaml(cls, path: Union[str, Path]) -> "PatrolConfig":
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            raise ValueError("patrol config must contain a mapping")
        route_id = _required_string(data, "route_id")
        frame_id = _required_string(data, "frame_id")
        if frame_id != "map":
            raise ValueError("frame_id must be 'map'")
        min_distance_m = _required_number(data, "min_distance_m")
        raw_triangle_side = data.get("equilateral_triangle_side_m")
        triangle_side = (
            None
            if raw_triangle_side is None
            else _required_value_number(raw_triangle_side, "equilateral_triangle_side_m")
        )
        raw_triangle_heading = data.get("equilateral_triangle_heading_deg")
        triangle_heading_deg = (
            None
            if raw_triangle_heading is None
            else _required_value_number(
                raw_triangle_heading,
                "equilateral_triangle_heading_deg",
            )
        )
        if triangle_heading_deg is not None and triangle_side is None:
            raise ValueError(
                "equilateral_triangle_heading_deg requires equilateral_triangle_side_m"
            )
        raw_forward_distances = data.get("forward_distances_m", [])
        if not isinstance(raw_forward_distances, list):
            raise ValueError("forward_distances_m must be a list")
        forward_distances_m = [
            _required_list_number(value, "forward_distances_m", index)
            for index, value in enumerate(raw_forward_distances, 1)
        ]
        offsets = [_parse_offset(item, index) for index, item in enumerate(data.get("offsets", []), 1)]
        segments = [
            _parse_segment(item, index) for index, item in enumerate(data.get("segments", []), 1)
        ]
        mode_count = sum(
            (
                triangle_side is not None,
                bool(forward_distances_m),
                bool(offsets),
                bool(segments),
            )
        )
        if mode_count != 1:
            raise ValueError(
                "exactly one of equilateral_triangle_side_m, forward_distances_m, "
                "offsets, or segments must be configured"
            )
        if triangle_side is not None and triangle_side < min_distance_m:
            raise ValueError("equilateral triangle side is below min_distance_m")
        if forward_distances_m and len(forward_distances_m) != 2:
            raise ValueError("forward_distances_m must contain exactly two target distances")
        if any(distance < min_distance_m for distance in forward_distances_m):
            raise ValueError("forward distance is below min_distance_m")
        if any(
            later <= earlier
            for earlier, later in zip(forward_distances_m, forward_distances_m[1:])
        ):
            raise ValueError("forward_distances_m must be strictly increasing")
        for item in offsets:
            if math.hypot(item.dx, item.dy) < min_distance_m:
                raise ValueError("patrol offset is below min_distance_m")
        for item in segments:
            if item.distance_m < min_distance_m:
                raise ValueError("patrol segment is below min_distance_m")
        return cls(
            route_id,
            frame_id,
            min_distance_m,
            triangle_side,
            triangle_heading_deg,
            forward_distances_m,
            offsets,
            segments,
        )

    def build_route(self, home: Waypoint) -> WaypointRoute:
        if self.equilateral_triangle_side_m is not None:
            route_points = _equilateral_triangle_route(
                home,
                self.equilateral_triangle_side_m,
                heading=(
                    home.yaw
                    if self.equilateral_triangle_heading_deg is None
                    else math.radians(self.equilateral_triangle_heading_deg)
                ),
            )
        elif self.forward_distances_m:
            points = _forward_waypoints(home, self.forward_distances_m)
            reverse_yaw = _normalize(home.yaw + math.pi)
            route_points = [
                replace(home, id="home"),
                points[0],
                replace(points[1], yaw=reverse_yaw),
                replace(points[0], id="p1_return", yaw=reverse_yaw),
                replace(home, id="home_return", yaw=home.yaw),
            ]
        elif self.offsets:
            points = _offset_waypoints(home, self.offsets)
            route_points = [replace(home, id="home")] + points
        else:
            points = _segment_waypoints(home, self.segments)
            route_points = [replace(home, id="home")] + points
        # Forward mode uses only three physical coordinates. The repeated p1/home
        # poses set the return heading so every travel leg is forward-facing.
        return WaypointRoute(
            route_id=self.route_id,
            frame_id=self.frame_id,
            loop=True,
            waypoints=route_points,
        )


class PatrolBackend(Protocol):
    def capture_current_pose(self, *, waypoint_id: str) -> Waypoint:
        """Capture the current map pose."""

    def send_route(self, route: WaypointRoute) -> Dict[str, object]:
        """Send one FollowWaypoints route and wait for its result."""

    def cancel_active(self) -> None:
        """Request cancellation of the active route."""


class PatrolStartupGate(Protocol):
    def ensure_ready(self) -> None:
        """Prepare the remote navigation stack and distributed ROS graph."""


class ContinuousPatrolController:
    def __init__(
        self,
        *,
        backend: PatrolBackend,
        patrol_config: Union[str, Path],
        startup_gate: Optional[PatrolStartupGate] = None,
        max_loops: int = 0,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        self.backend = backend
        self.config = PatrolConfig.from_yaml(patrol_config)
        self.startup_gate = startup_gate
        if max_loops < 0:
            raise ValueError("max_loops must be >= 0")
        self.max_loops = max_loops
        self.logger = logger or logging.getLogger(__name__)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None  # type: Optional[threading.Thread]
        self._home = None  # type: Optional[Waypoint]

    @property
    def active(self) -> bool:
        with self._lock:
            return self._thread is not None and self._thread.is_alive()

    @property
    def home(self) -> Optional[Waypoint]:
        with self._lock:
            return self._home

    def start(self) -> bool:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return False
            self._stop.clear()
            self._thread = threading.Thread(
                target=self._run_loop,
                name="mqtt-continuous-patrol",
                daemon=True,
            )
            self._thread.start()
        return True

    def stop(self) -> None:
        self._stop.set()
        self.backend.cancel_active()

    def return_home(self) -> bool:
        with self._lock:
            home = self._home
            active_thread = self._thread
        if home is None:
            self.logger.warning("RETURN_HOME ignored: patrol home has not been captured")
            return False
        self.stop()
        threading.Thread(
            target=self._return_home_after_stop,
            args=(active_thread, home),
            name="mqtt-return-home",
            daemon=True,
        ).start()
        return True

    def emergency_stop(self) -> None:
        self.stop()

    def reset(self) -> None:
        self.stop()
        with self._lock:
            self._home = None

    def close(self, timeout_sec: float = 3.0) -> None:
        self.stop()
        with self._lock:
            thread = self._thread
        if thread is not None:
            thread.join(timeout=timeout_sec)

    def _run_loop(self) -> None:
        try:
            if self.startup_gate is not None:
                self.startup_gate.ensure_ready()
            if self._stop.is_set():
                return
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
                [(item.id, item.x, item.y) for item in route.waypoints],
            )
            completed_loops = 0
            while not self._stop.is_set():
                result = self.backend.send_route(route)
                if self._stop.is_set():
                    break
                if not _succeeded(result):
                    self.logger.error("patrol route failed: %s", result)
                    break
                completed_loops += 1
                if self.max_loops and completed_loops >= self.max_loops:
                    self.logger.info("patrol loop limit reached loops=%s", completed_loops)
                    break
                self.logger.info("patrol route completed; starting next loop")
        except Exception:
            self.logger.exception("continuous patrol stopped by error")
        finally:
            with self._lock:
                if self._thread is threading.current_thread():
                    self._thread = None
            self.logger.info("patrol loop stopped")

    def _return_home_after_stop(
        self,
        active_thread: Optional[threading.Thread],
        home: Waypoint,
    ) -> None:
        if active_thread is not None:
            active_thread.join(timeout=5.0)
            if active_thread.is_alive():
                self.logger.error("return-home aborted: active patrol did not cancel")
                return
        try:
            current = self.backend.capture_current_pose(waypoint_id="current")
            route = WaypointRoute(
                route_id="return_home",
                frame_id=self.config.frame_id,
                loop=False,
                waypoints=[current, replace(home, id="home_return")],
            )
            result = self.backend.send_route(route)
            if not _succeeded(result):
                self.logger.error("return-home route failed: %s", result)
        except Exception:
            self.logger.exception("return-home failed")


class MockPatrolBackend:
    """Non-moving backend used by the MQTT sample and automated tests."""

    def __init__(
        self,
        *,
        home: Optional[Waypoint] = None,
        route_duration_sec: float = 0.1,
    ) -> None:
        self.current_pose = home or Waypoint(id="home", x=0.0, y=0.0, yaw=0.0)
        self.route_duration_sec = route_duration_sec
        self.routes = []  # type: List[WaypointRoute]
        self._cancel = threading.Event()

    def capture_current_pose(self, *, waypoint_id: str) -> Waypoint:
        return replace(self.current_pose, id=waypoint_id)

    def send_route(self, route: WaypointRoute) -> Dict[str, object]:
        self._cancel.clear()
        self.routes.append(route)
        if self._cancel.wait(timeout=max(0.0, self.route_duration_sec)):
            return {"accepted": True, "status": "CANCELED", "missed_waypoints": []}
        self.current_pose = replace(route.waypoints[-1], id="current")
        return {"accepted": True, "status": "SUCCEEDED", "missed_waypoints": []}

    def cancel_active(self) -> None:
        self._cancel.set()


class Nav2PatrolBackend:
    """Cancelable ROS2 Foxy FollowWaypoints backend."""

    def __init__(
        self,
        *,
        odom_topic: str = "/odom",
        action_name: str = "/FollowWaypoints",
        timeout_sec: float = 10.0,
    ) -> None:
        self.odom_topic = odom_topic
        self.action_name = action_name
        self.timeout_sec = timeout_sec
        self._cancel = threading.Event()

    def capture_current_pose(self, *, waypoint_id: str) -> Waypoint:
        import rclpy
        from nav_msgs.msg import Odometry

        rclpy.init(args=None)
        node = rclpy.create_node("lite3_mqtt_pose_capture")
        captured = {"pose": None}  # type: Dict[str, Any]

        def on_msg(msg) -> None:
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
            cosy_cosp = 1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2)
            captured["pose"] = Waypoint(
                id=waypoint_id,
                x=float(position.x),
                y=float(position.y),
                yaw=math.atan2(siny_cosp, cosy_cosp),
            )

        subscription = node.create_subscription(Odometry, self.odom_topic, on_msg, 10)
        deadline = time.monotonic() + self.timeout_sec
        try:
            while rclpy.ok() and captured["pose"] is None and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            if captured["pose"] is None:
                raise TimeoutError("timed out waiting for {}".format(self.odom_topic))
            return captured["pose"]
        finally:
            node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()

    def cancel_active(self) -> None:
        self._cancel.set()

    def wait_until_ready(self, timeout_sec: Optional[float] = None) -> None:
        import rclpy
        from nav2_msgs.action import FollowWaypoints
        from rclpy.action import ActionClient

        rclpy.init(args=None)
        node = rclpy.create_node("lite3_mqtt_nav_readiness")
        client = ActionClient(node, FollowWaypoints, self.action_name)
        deadline = time.monotonic() + (
            self.timeout_sec if timeout_sec is None else timeout_sec
        )
        last_reasons = []
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                topics = {name for name, _ in node.get_topic_names_and_types()}
                action_ready = client.wait_for_server(timeout_sec=0.1)
                odom_ready = self.odom_topic in topics and bool(
                    node.get_publishers_info_by_topic(self.odom_topic)
                )
                cmd_vel_ready = "/cmd_vel" in topics
                motion_sender_ready = False
                if cmd_vel_ready:
                    subscriptions = node.get_subscriptions_info_by_topic("/cmd_vel")
                    motion_sender_ready = any(
                        "motion_sender" in info.node_name for info in subscriptions
                    )
                last_reasons = []
                if not action_ready:
                    last_reasons.append("{} action unavailable".format(self.action_name))
                if not odom_ready:
                    last_reasons.append("{} publisher unavailable".format(self.odom_topic))
                if not cmd_vel_ready:
                    last_reasons.append("/cmd_vel topic unavailable")
                elif not motion_sender_ready:
                    last_reasons.append("motion_sender is not subscribed to /cmd_vel")
                if not last_reasons:
                    return
            raise TimeoutError("Nav2 DDS readiness timed out: {}".format(", ".join(last_reasons)))
        finally:
            client.destroy()
            node.destroy_node()
            rclpy.shutdown()

    def send_route(self, route: WaypointRoute) -> Dict[str, object]:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from nav2_msgs.action import FollowWaypoints
        from rclpy.action import ActionClient

        self._cancel.clear()
        rclpy.init(args=None)
        node = rclpy.create_node("lite3_mqtt_waypoint_patrol")
        client = ActionClient(node, FollowWaypoints, self.action_name)
        try:
            if not client.wait_for_server(timeout_sec=self.timeout_sec):
                raise TimeoutError("{} action server is not available".format(self.action_name))
            if self._cancel.is_set():
                return {"accepted": False, "status": "CANCELED", "missed_waypoints": []}

            goal = FollowWaypoints.Goal()
            for waypoint in route.waypoints:
                pose = PoseStamped()
                pose.header.frame_id = route.frame_id
                pose.header.stamp = node.get_clock().now().to_msg()
                pose.pose.position.x = waypoint.x
                pose.pose.position.y = waypoint.y
                half_yaw = waypoint.yaw / 2.0
                pose.pose.orientation.z = math.sin(half_yaw)
                pose.pose.orientation.w = math.cos(half_yaw)
                goal.poses.append(pose)

            send_future = client.send_goal_async(goal)
            if not _spin_future(node, send_future, self.timeout_sec, self._cancel):
                return {"accepted": False, "status": "CANCELED", "missed_waypoints": []}
            goal_handle = send_future.result()
            if goal_handle is None or not goal_handle.accepted:
                return {"accepted": False, "status": None, "missed_waypoints": []}

            result_future = goal_handle.get_result_async()
            cancel_future = None
            while rclpy.ok() and not result_future.done():
                rclpy.spin_once(node, timeout_sec=0.1)
                if self._cancel.is_set() and cancel_future is None:
                    cancel_future = goal_handle.cancel_goal_async()
            if not result_future.done():
                return {"accepted": True, "status": "CANCELED", "missed_waypoints": []}
            result = result_future.result()
            return {
                "accepted": True,
                "status": result.status,
                "missed_waypoints": list(result.result.missed_waypoints),
            }
        finally:
            client.destroy()
            node.destroy_node()
            rclpy.shutdown()


def _offset_waypoints(home: Waypoint, offsets: List[PatrolOffset]) -> List[Waypoint]:
    cos_yaw = math.cos(home.yaw)
    sin_yaw = math.sin(home.yaw)
    points = []
    for index, item in enumerate(offsets, 1):
        points.append(
            Waypoint(
                id="p{}".format(index),
                x=home.x + item.dx * cos_yaw - item.dy * sin_yaw,
                y=home.y + item.dx * sin_yaw + item.dy * cos_yaw,
                yaw=_normalize(home.yaw + item.yaw_offset),
                dwell_sec=item.dwell_sec,
            )
        )
    return points


def _forward_waypoints(home: Waypoint, distances: List[float]) -> List[Waypoint]:
    """Project targets only onto the robot's forward axis; no lateral offset."""
    cos_yaw = math.cos(home.yaw)
    sin_yaw = math.sin(home.yaw)
    return [
        Waypoint(
            id="p{}".format(index),
            x=home.x + distance * cos_yaw,
            y=home.y + distance * sin_yaw,
            yaw=home.yaw,
        )
        for index, distance in enumerate(distances, 1)
    ]


def _equilateral_triangle_route(
    home: Waypoint,
    side_m: float,
    *,
    heading: float,
) -> List[Waypoint]:
    """Build two new vertices and headings that face each triangle edge."""
    cos_yaw = math.cos(heading)
    sin_yaw = math.sin(heading)
    p1 = Waypoint(
        id="p1",
        x=home.x + side_m * cos_yaw,
        y=home.y + side_m * sin_yaw,
        yaw=_normalize(heading + 2.0 * math.pi / 3.0),
    )
    p2 = Waypoint(
        id="p2",
        x=home.x
        + side_m * (0.5 * cos_yaw - (math.sqrt(3.0) / 2.0) * sin_yaw),
        y=home.y
        + side_m * (0.5 * sin_yaw + (math.sqrt(3.0) / 2.0) * cos_yaw),
        yaw=_normalize(heading - 2.0 * math.pi / 3.0),
    )
    return [
        p1,
        p2,
        replace(home, id="home_return", yaw=heading),
    ]


def _segment_waypoints(home: Waypoint, segments: List[PatrolSegment]) -> List[Waypoint]:
    x, y, yaw = home.x, home.y, home.yaw
    points = []
    for index, item in enumerate(segments, 1):
        yaw = _normalize(yaw + item.turn_rad)
        x += item.distance_m * math.cos(yaw)
        y += item.distance_m * math.sin(yaw)
        points.append(Waypoint("p{}".format(index), x, y, yaw, item.dwell_sec))
    return points


def _parse_offset(item: Any, index: int) -> PatrolOffset:
    if not isinstance(item, dict):
        raise ValueError("offset #{} must be a mapping".format(index))
    return PatrolOffset(
        dx=_required_number(item, "dx"),
        dy=_required_number(item, "dy"),
        yaw_offset=float(item.get("yaw_offset", 0.0) or 0.0),
        dwell_sec=float(item.get("dwell_sec", 0.0) or 0.0),
    )


def _parse_segment(item: Any, index: int) -> PatrolSegment:
    if not isinstance(item, dict):
        raise ValueError("segment #{} must be a mapping".format(index))
    turn_rad = (
        _required_number(item, "turn_rad")
        if "turn_rad" in item
        else math.radians(float(item.get("turn_deg", 0.0) or 0.0))
    )
    return PatrolSegment(
        distance_m=_required_number(item, "distance_m"),
        turn_rad=turn_rad,
        dwell_sec=float(item.get("dwell_sec", 0.0) or 0.0),
    )


def _required_string(data: Dict[str, Any], key: str) -> str:
    value = data.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ValueError("{} must be a non-empty string".format(key))
    return value.strip()


def _required_number(data: Dict[str, Any], key: str) -> float:
    value = data.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("{} must be numeric".format(key))
    return float(value)


def _required_list_number(value: Any, key: str, index: int) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("{} #{} must be numeric".format(key, index))
    return float(value)


def _required_value_number(value: Any, key: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("{} must be numeric".format(key))
    return float(value)


def _normalize(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _spin_future(node, future, timeout_sec: float, cancel: threading.Event) -> bool:
    import rclpy

    deadline = time.monotonic() + timeout_sec
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if cancel.is_set():
            return False
    if not future.done():
        raise TimeoutError("timed out waiting for FollowWaypoints goal acceptance")
    return True


def _succeeded(result: Dict[str, object]) -> bool:
    return bool(result.get("accepted")) and result.get("status") in {4, "SUCCEEDED"}
