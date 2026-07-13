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
        if min_distance_m <= 0.0:
            raise ValueError("min_distance_m must be positive")
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
                points[0],
                replace(points[1], yaw=reverse_yaw),
                replace(points[0], id="p1_return", yaw=reverse_yaw),
                replace(home, id="home_return", yaw=home.yaw),
            ]
        elif self.offsets:
            points = _offset_waypoints(home, self.offsets)
            route_points = points + [replace(home, id="home_return")]
        else:
            points = _segment_waypoints(home, self.segments)
            route_points = points + [replace(home, id="home_return")]
        # Forward mode uses only three physical coordinates. The repeated p1/home
        # poses set the return heading so every travel leg is forward-facing.
        return WaypointRoute(
            route_id=self.route_id,
            frame_id=self.frame_id,
            loop=True,
            waypoints=route_points,
        )

    def build_candidate_routes(self, home: Waypoint) -> List[WaypointRoute]:
        if (
            self.equilateral_triangle_side_m is None
            or self.equilateral_triangle_heading_deg is not None
        ):
            return [self.build_route(home)]
        heading_offsets_deg = (0.0, 30.0, -30.0, 60.0, -60.0, 90.0, -90.0,
                               120.0, -120.0, 150.0, -150.0, 180.0)
        routes = []
        for offset_deg in heading_offsets_deg:
            waypoints = _equilateral_triangle_route(
                home,
                self.equilateral_triangle_side_m,
                heading=_normalize(home.yaw + math.radians(offset_deg)),
            )
            routes.append(
                WaypointRoute(
                    route_id=self.route_id,
                    frame_id=self.frame_id,
                    loop=True,
                    waypoints=waypoints,
                )
            )
        return routes


class PatrolBackend(Protocol):
    def capture_current_pose(self, *, waypoint_id: str) -> Waypoint:
        """Capture the current map pose."""

    def prepare_route(self) -> None:
        """Clear cancellation state before the controller's final stop check."""

    def validate_route(self, route: WaypointRoute, *, start: Waypoint) -> None:
        """Fail before motion when the route is outside known free space."""

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
        self._return_thread = None  # type: Optional[threading.Thread]
        self._home = None  # type: Optional[Waypoint]
        self._return_cancel = threading.Event()
        self._emergency_latched = False

    @property
    def active(self) -> bool:
        with self._lock:
            patrol_active = self._thread is not None and self._thread.is_alive()
            return_active = (
                self._return_thread is not None and self._return_thread.is_alive()
            )
            return patrol_active or return_active

    @property
    def emergency_latched(self) -> bool:
        with self._lock:
            return self._emergency_latched

    @property
    def home(self) -> Optional[Waypoint]:
        with self._lock:
            return self._home

    def start(self) -> bool:
        with self._lock:
            if self._emergency_latched:
                self.logger.error("START rejected: emergency stop is latched; RESET required")
                return False
            patrol_active = self._thread is not None and self._thread.is_alive()
            return_active = (
                self._return_thread is not None and self._return_thread.is_alive()
            )
            if patrol_active or return_active:
                return False
            self._stop.clear()
            self._return_cancel.set()
            self._thread = threading.Thread(
                target=self._run_loop,
                name="mqtt-continuous-patrol",
                daemon=True,
            )
            self._thread.start()
        return True

    def stop(self) -> None:
        self._stop.set()
        self._return_cancel.set()
        self.backend.cancel_active()

    def return_home(self) -> bool:
        with self._lock:
            home = self._home
            active_thread = self._thread
            return_active = (
                self._return_thread is not None and self._return_thread.is_alive()
            )
            if self._emergency_latched:
                self.logger.error("RETURN_HOME rejected: emergency stop is latched")
                return False
            if return_active:
                return False
            if home is None:
                self.logger.warning("RETURN_HOME ignored: patrol home has not been captured")
                return False
            self._stop.set()
            self._return_cancel.clear()
            return_thread = threading.Thread(
                target=self._return_home_after_stop,
                args=(active_thread, home),
                name="mqtt-return-home",
                daemon=True,
            )
            self._return_thread = return_thread
        self.backend.cancel_active()
        return_thread.start()
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

    def close(self, timeout_sec: float = 10.0) -> None:
        self.stop()
        with self._lock:
            thread = self._thread
            return_thread = self._return_thread
        if thread is not None:
            thread.join(timeout=timeout_sec)
        if return_thread is not None:
            return_thread.join(timeout=timeout_sec)

    def _run_loop(self) -> None:
        try:
            self.backend.prepare_route()
            if self._stop.is_set():
                return
            if self.startup_gate is not None:
                self.startup_gate.ensure_ready()
            if self._stop.is_set():
                return
            home = self.backend.capture_current_pose(waypoint_id="home")
            if self._stop.is_set():
                return
            route = self._select_valid_route(home)
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
                self.backend.prepare_route()
                if self._stop.is_set():
                    break
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

    def _select_valid_route(self, home: Waypoint) -> WaypointRoute:
        failures = []
        for route in self.config.build_candidate_routes(home):
            if self._stop.is_set():
                raise RuntimeError("patrol stopped during route preflight")
            _validate_route(route)
            try:
                self.backend.validate_route(route, start=home)
                return route
            except ValueError as exc:
                failures.append(str(exc))
        raise ValueError(
            "no safe patrol route candidate: {}".format(" | ".join(failures[-4:]))
        )

    def _return_home_after_stop(
        self,
        active_thread: Optional[threading.Thread],
        home: Waypoint,
    ) -> None:
        if active_thread is not None:
            active_thread.join(timeout=10.0)
            if active_thread.is_alive():
                self.logger.error("return-home aborted: active patrol did not cancel")
                return
        try:
            if self._return_cancel.is_set():
                return
            current = self.backend.capture_current_pose(waypoint_id="current")
            route = WaypointRoute(
                route_id="return_home",
                frame_id=self.config.frame_id,
                loop=False,
                waypoints=[replace(home, id="home_return")],
            )
            _validate_route(route)
            self.backend.prepare_route()
            if self._return_cancel.is_set():
                self.backend.cancel_active()
                return
            self.backend.validate_route(route, start=current)
            self.backend.prepare_route()
            if self._return_cancel.is_set():
                self.backend.cancel_active()
                return
            result = self.backend.send_route(route)
            if not _succeeded(result):
                self.logger.error("return-home route failed: %s", result)
        except Exception:
            self.logger.exception("return-home failed")
        finally:
            with self._lock:
                if self._return_thread is threading.current_thread():
                    self._return_thread = None


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

    def prepare_route(self) -> None:
        self._cancel.clear()

    def validate_route(self, route: WaypointRoute, *, start: Waypoint) -> None:
        _validate_route(route)
        _validate_waypoint(start)

    def send_route(self, route: WaypointRoute) -> Dict[str, object]:
        self.routes.append(route)
        if self._cancel.wait(timeout=max(0.0, self.route_duration_sec)):
            return {"accepted": True, "status": "CANCELED", "missed_waypoints": []}
        self.current_pose = replace(route.waypoints[-1], id="current")
        return {"accepted": True, "status": "SUCCEEDED", "missed_waypoints": []}

    def cancel_active(self) -> None:
        self._cancel.set()


class NavSafetyState:
    """Reception-time health for data that must stay live during navigation."""

    REQUIRED_STREAMS = ("odom", "status", "local_costmap", "global_costmap")

    def __init__(self) -> None:
        self.last_seen = {}  # type: Dict[str, float]
        self.localization_converged = None  # type: Optional[bool]
        self.lateral_speed_mps = 0.0
        self.odom_frame_ok = None  # type: Optional[bool]

    def mark(self, stream: str, now: float) -> None:
        self.last_seen[stream] = now

    def mark_localization(self, *, now: float, converged: bool) -> None:
        self.mark("status", now)
        self.localization_converged = bool(converged)

    def mark_odom(self, *, now: float, frame_id: str) -> None:
        self.mark("odom", now)
        self.odom_frame_ok = str(frame_id).strip("/") == "map"

    def mark_cmd_vel(self, lateral_speed_mps: float) -> None:
        self.lateral_speed_mps = float(lateral_speed_mps)

    def blocking_reasons(
        self,
        *,
        now: float,
        max_age_sec: float,
        max_lateral_speed_mps: float,
    ) -> List[str]:
        reasons = []
        for stream in self.REQUIRED_STREAMS:
            seen = self.last_seen.get(stream)
            if seen is None:
                reasons.append("{}_missing".format(stream))
            elif now - seen > max_age_sec:
                reasons.append("{}_stale".format(stream))
        if self.localization_converged is False:
            reasons.append("localization_not_converged")
        if self.odom_frame_ok is False:
            reasons.append("odom_frame_not_map")
        if abs(self.lateral_speed_mps) > max_lateral_speed_mps:
            reasons.append("lateral_cmd_vel")
        return reasons


class Nav2PatrolBackend:
    """Cancelable ROS2 Foxy FollowWaypoints backend."""

    def __init__(
        self,
        *,
        odom_topic: str = "/odom",
        action_name: str = "/FollowWaypoints",
        timeout_sec: float = 10.0,
        max_data_age_sec: float = 2.0,
        max_lateral_speed_mps: float = 0.02,
        route_timeout_sec: float = 300.0,
        cancel_timeout_sec: float = 5.0,
    ) -> None:
        if min(timeout_sec, max_data_age_sec, route_timeout_sec, cancel_timeout_sec) <= 0.0:
            raise ValueError("Nav2 timeout values must be positive")
        if max_lateral_speed_mps < 0.0:
            raise ValueError("max_lateral_speed_mps must be non-negative")
        self.odom_topic = odom_topic
        self.action_name = action_name
        self.timeout_sec = timeout_sec
        self.max_data_age_sec = max_data_age_sec
        self.max_lateral_speed_mps = max_lateral_speed_mps
        self.route_timeout_sec = route_timeout_sec
        self.cancel_timeout_sec = cancel_timeout_sec
        self._cancel = threading.Event()

    def prepare_route(self) -> None:
        self._cancel.clear()

    def validate_route(self, route: WaypointRoute, *, start: Waypoint) -> None:
        import rclpy
        from nav_msgs.msg import OccupancyGrid

        rclpy.init(args=None)
        node = rclpy.create_node("lite3_mqtt_route_preflight")
        captured = {"costmap": None}  # type: Dict[str, Any]
        subscription = node.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            lambda msg: captured.__setitem__("costmap", msg),
            10,
        )
        deadline = time.monotonic() + self.timeout_sec
        try:
            while (
                rclpy.ok()
                and captured["costmap"] is None
                and not self._cancel.is_set()
                and time.monotonic() < deadline
            ):
                rclpy.spin_once(node, timeout_sec=0.1)
            if self._cancel.is_set():
                raise RuntimeError("route preflight canceled")
            costmap = captured["costmap"]
            if costmap is None:
                raise TimeoutError("global costmap preflight timed out")
            _validate_route_on_costmap(route, start=start, costmap=costmap)
        finally:
            node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()

    def capture_current_pose(self, *, waypoint_id: str) -> Waypoint:
        import rclpy
        from nav_msgs.msg import Odometry

        rclpy.init(args=None)
        node = rclpy.create_node("lite3_mqtt_pose_capture")
        captured = {"pose": None}  # type: Dict[str, Any]

        def on_msg(msg) -> None:
            frame_id = str(msg.header.frame_id).strip("/")
            if frame_id != "map":
                captured["error"] = "{} frame is {!r}, expected 'map'".format(
                    self.odom_topic,
                    msg.header.frame_id,
                )
                return
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            values = (
                position.x,
                position.y,
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            )
            if not all(math.isfinite(float(value)) for value in values):
                captured["error"] = "{} contains non-finite pose values".format(
                    self.odom_topic
                )
                return
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
            while (
                rclpy.ok()
                and captured["pose"] is None
                and captured.get("error") is None
                and time.monotonic() < deadline
            ):
                rclpy.spin_once(node, timeout_sec=0.1)
            if captured["pose"] is None:
                if captured.get("error"):
                    raise RuntimeError(str(captured["error"]))
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
        safety = NavSafetyState()
        subscriptions = _create_safety_subscriptions(node, safety, self.odom_topic)
        deadline = time.monotonic() + (
            self.timeout_sec if timeout_sec is None else timeout_sec
        )
        last_reasons = []
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                action_ready = client.wait_for_server(timeout_sec=0.1)
                last_reasons = _nav_graph_reasons(
                    node,
                    action_ready=action_ready,
                    action_name=self.action_name,
                    odom_topic=self.odom_topic,
                )
                last_reasons.extend(
                    safety.blocking_reasons(
                        now=time.monotonic(),
                        max_age_sec=self.max_data_age_sec,
                        max_lateral_speed_mps=self.max_lateral_speed_mps,
                    )
                )
                if not last_reasons:
                    last_reasons.extend(_controller_nonholonomic_reasons(node))
                if not last_reasons:
                    return
            raise TimeoutError("Nav2 DDS readiness timed out: {}".format(", ".join(last_reasons)))
        finally:
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            client.destroy()
            node.destroy_node()
            rclpy.shutdown()

    def send_route(self, route: WaypointRoute) -> Dict[str, object]:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from nav2_msgs.action import FollowWaypoints
        from rclpy.action import ActionClient
        from std_msgs.msg import UInt64

        rclpy.init(args=None)
        node = rclpy.create_node("lite3_mqtt_waypoint_patrol")
        client = ActionClient(node, FollowWaypoints, self.action_name)
        heartbeat_pub = node.create_publisher(UInt64, "/lite3/nav/heartbeat", 10)
        safety = NavSafetyState()
        subscriptions = _create_safety_subscriptions(node, safety, self.odom_topic)
        try:
            if not client.wait_for_server(timeout_sec=self.timeout_sec):
                raise TimeoutError("{} action server is not available".format(self.action_name))
            if self._cancel.is_set():
                return {"accepted": False, "status": "CANCELED", "missed_waypoints": []}

            _wait_for_safe_samples(
                node,
                safety,
                timeout_sec=self.timeout_sec,
                max_age_sec=self.max_data_age_sec,
                max_lateral_speed_mps=self.max_lateral_speed_mps,
                cancel=self._cancel,
            )
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

            _publish_nav_heartbeat(heartbeat_pub, UInt64)
            next_heartbeat = time.monotonic() + 0.5
            send_future = client.send_goal_async(goal)
            acceptance_deadline = time.monotonic() + self.timeout_sec
            while rclpy.ok() and not send_future.done():
                rclpy.spin_once(node, timeout_sec=0.1)
                if time.monotonic() >= next_heartbeat:
                    _publish_nav_heartbeat(heartbeat_pub, UInt64)
                    next_heartbeat = time.monotonic() + 0.5
                if time.monotonic() >= acceptance_deadline:
                    raise TimeoutError("timed out waiting for FollowWaypoints goal acceptance")
            goal_handle = send_future.result()
            if goal_handle is None or not goal_handle.accepted:
                return {"accepted": False, "status": None, "missed_waypoints": []}

            result_future = goal_handle.get_result_async()
            cancel_future = None
            cancel_deadline = None
            cancel_reason = None
            route_deadline = time.monotonic() + self.route_timeout_sec
            next_graph_check = time.monotonic() + 1.0
            while rclpy.ok() and not result_future.done():
                rclpy.spin_once(node, timeout_sec=0.1)
                now = time.monotonic()
                if now >= next_heartbeat:
                    _publish_nav_heartbeat(heartbeat_pub, UInt64)
                    next_heartbeat = now + 0.5
                if cancel_reason is None and self._cancel.is_set():
                    cancel_reason = "operator_cancel"
                if cancel_reason is None and now >= route_deadline:
                    cancel_reason = "route_timeout"
                if cancel_reason is None:
                    reasons = safety.blocking_reasons(
                        now=now,
                        max_age_sec=self.max_data_age_sec,
                        max_lateral_speed_mps=self.max_lateral_speed_mps,
                    )
                    if reasons:
                        cancel_reason = "safety:" + ",".join(reasons)
                if cancel_reason is None and now >= next_graph_check:
                    graph_reasons = _nav_graph_reasons(
                        node,
                        action_ready=client.wait_for_server(timeout_sec=0.0),
                        action_name=self.action_name,
                        odom_topic=self.odom_topic,
                    )
                    if graph_reasons:
                        cancel_reason = "graph:" + ",".join(graph_reasons)
                    next_graph_check = now + 1.0
                if cancel_reason is not None and cancel_future is None:
                    cancel_future = goal_handle.cancel_goal_async()
                    cancel_deadline = now + self.cancel_timeout_sec
                if cancel_deadline is not None and now >= cancel_deadline:
                    return {
                        "accepted": True,
                        "status": "CANCEL_TIMEOUT",
                        "missed_waypoints": [],
                        "reason": cancel_reason,
                    }
            if not result_future.done():
                return {
                    "accepted": True,
                    "status": "CANCELED",
                    "missed_waypoints": [],
                    "reason": cancel_reason or "rclpy_shutdown",
                }
            result = result_future.result()
            return {
                "accepted": True,
                "status": result.status,
                "missed_waypoints": list(result.result.missed_waypoints),
                "reason": cancel_reason,
            }
        finally:
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            node.destroy_publisher(heartbeat_pub)
            client.destroy()
            node.destroy_node()
            rclpy.shutdown()


def _create_safety_subscriptions(node, state: NavSafetyState, odom_topic: str):
    from geometry_msgs.msg import Twist
    from hdl_localization.msg import ScanMatchingStatus
    from nav_msgs.msg import OccupancyGrid, Odometry

    def now() -> float:
        return time.monotonic()

    return [
        node.create_subscription(
            Odometry,
            odom_topic,
            lambda msg: state.mark_odom(now=now(), frame_id=msg.header.frame_id),
            10,
        ),
        node.create_subscription(
            ScanMatchingStatus,
            "/status",
            lambda msg: state.mark_localization(
                now=now(),
                converged=msg.has_converged,
            ),
            10,
        ),
        node.create_subscription(
            OccupancyGrid,
            "/local_costmap/costmap",
            lambda msg: state.mark("local_costmap", now()),
            10,
        ),
        node.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            lambda msg: state.mark("global_costmap", now()),
            10,
        ),
        node.create_subscription(
            Twist,
            "/cmd_vel",
            lambda msg: state.mark_cmd_vel(msg.linear.y),
            10,
        ),
    ]


def _publish_nav_heartbeat(publisher, message_type) -> None:
    message = message_type()
    message.data = time.monotonic_ns() & ((1 << 64) - 1)
    publisher.publish(message)


def _wait_for_safe_samples(
    node,
    state: NavSafetyState,
    *,
    timeout_sec: float,
    max_age_sec: float,
    max_lateral_speed_mps: float,
    cancel: threading.Event,
) -> None:
    import rclpy

    deadline = time.monotonic() + timeout_sec
    last_reasons = []
    while rclpy.ok() and not cancel.is_set() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        last_reasons = state.blocking_reasons(
            now=time.monotonic(),
            max_age_sec=max_age_sec,
            max_lateral_speed_mps=max_lateral_speed_mps,
        )
        if not last_reasons:
            return
    if cancel.is_set():
        return
    raise TimeoutError("Nav2 safety samples timed out: {}".format(", ".join(last_reasons)))


def _nav_graph_reasons(
    node,
    *,
    action_ready: bool,
    action_name: str,
    odom_topic: str,
) -> List[str]:
    required_nodes = {
        "/hdl_localization",
        "/planner_server",
        "/controller_server",
        "/global_costmap/global_costmap",
        "/local_costmap/local_costmap",
        "/waypoint_follower",
    }
    required_topics = {
        odom_topic,
        "/status",
        "/map",
        "/global_costmap/costmap",
        "/local_costmap/costmap",
        "/cmd_vel",
    }
    nodes = set()
    for name, namespace in node.get_node_names_and_namespaces():
        prefix = "" if namespace == "/" else namespace.rstrip("/")
        nodes.add("{}/{}".format(prefix, name))
    topics = {name for name, _ in node.get_topic_names_and_types()}
    reasons = []
    reasons.extend("missing node {}".format(name) for name in sorted(required_nodes - nodes))
    reasons.extend("missing topic {}".format(name) for name in sorted(required_topics - topics))
    if not action_ready:
        reasons.append("{} action unavailable".format(action_name))

    publishers = node.get_publishers_info_by_topic("/cmd_vel")
    publisher_names = {info.node_name.lstrip("/") for info in publishers}
    if "controller_server" not in publisher_names:
        reasons.append("controller_server is not publishing /cmd_vel")
    unexpected = publisher_names - {
        "controller_server",
        "recoveries_server",
        "lite3_nav_watchdog",
    }
    if unexpected:
        reasons.append("unexpected /cmd_vel publishers: {}".format(",".join(sorted(unexpected))))
    subscriptions = node.get_subscriptions_info_by_topic("/cmd_vel")
    subscription_names = {info.node_name.lstrip("/") for info in subscriptions}
    if "motion_sender" not in subscription_names:
        reasons.append("motion_sender is not subscribed to /cmd_vel")
    return reasons


def _controller_nonholonomic_reasons(node) -> List[str]:
    from rcl_interfaces.srv import GetParameters

    names = (
        "FollowPath.min_vel_y",
        "FollowPath.max_vel_y",
        "FollowPath.acc_lim_y",
        "FollowPath.decel_lim_y",
        "FollowPath.vy_samples",
    )
    client = node.create_client(GetParameters, "/controller_server/get_parameters")
    try:
        if not client.wait_for_service(timeout_sec=0.5):
            return ["controller parameter service unavailable"]
        request = GetParameters.Request()
        request.names = list(names)
        future = client.call_async(request)
        deadline = time.monotonic() + 0.5
        while not future.done() and time.monotonic() < deadline:
            import rclpy

            rclpy.spin_once(node, timeout_sec=0.05)
        if not future.done() or future.result() is None:
            return ["controller parameter query timed out"]
        values = future.result().values
        if len(values) != len(names):
            return ["controller parameter query returned incomplete data"]
        expected_types = (3, 3, 3, 3, 2)
        if tuple(value.type for value in values) != expected_types:
            return ["controller non-holonomic parameters have unexpected types"]
        numeric = [
            value.double_value if value.type == 3 else float(value.integer_value)
            for value in values
        ]
        min_y, max_y, acc_y, decel_y, samples_y = numeric
        reasons = []
        if abs(min_y) > 1e-9 or abs(max_y) > 1e-9:
            reasons.append("controller lateral velocity is enabled")
        if abs(acc_y) > 1e-9 or abs(decel_y) > 1e-9:
            reasons.append("controller lateral acceleration is enabled")
        if int(samples_y) != 1:
            reasons.append("controller vy_samples must be 1")
        return reasons
    finally:
        node.destroy_client(client)


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
    offset = PatrolOffset(
        dx=_required_number(item, "dx"),
        dy=_required_number(item, "dy"),
        yaw_offset=float(item.get("yaw_offset", 0.0) or 0.0),
        dwell_sec=float(item.get("dwell_sec", 0.0) or 0.0),
    )
    if not math.isfinite(offset.yaw_offset):
        raise ValueError("offset #{} yaw_offset must be finite".format(index))
    if not math.isfinite(offset.dwell_sec) or offset.dwell_sec < 0.0:
        raise ValueError("offset #{} dwell_sec must be finite and non-negative".format(index))
    return offset


def _parse_segment(item: Any, index: int) -> PatrolSegment:
    if not isinstance(item, dict):
        raise ValueError("segment #{} must be a mapping".format(index))
    turn_rad = (
        _required_number(item, "turn_rad")
        if "turn_rad" in item
        else math.radians(float(item.get("turn_deg", 0.0) or 0.0))
    )
    segment = PatrolSegment(
        distance_m=_required_number(item, "distance_m"),
        turn_rad=turn_rad,
        dwell_sec=float(item.get("dwell_sec", 0.0) or 0.0),
    )
    if not math.isfinite(segment.turn_rad):
        raise ValueError("segment #{} turn must be finite".format(index))
    if not math.isfinite(segment.dwell_sec) or segment.dwell_sec < 0.0:
        raise ValueError("segment #{} dwell_sec must be finite and non-negative".format(index))
    return segment


def _required_string(data: Dict[str, Any], key: str) -> str:
    value = data.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ValueError("{} must be a non-empty string".format(key))
    return value.strip()


def _required_number(data: Dict[str, Any], key: str) -> float:
    value = data.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("{} must be numeric".format(key))
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("{} must be finite".format(key))
    return number


def _required_list_number(value: Any, key: str, index: int) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("{} #{} must be numeric".format(key, index))
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("{} #{} must be finite".format(key, index))
    return number


def _required_value_number(value: Any, key: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("{} must be numeric".format(key))
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("{} must be finite".format(key))
    return number


def _normalize(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _succeeded(result: Dict[str, object]) -> bool:
    return (
        bool(result.get("accepted"))
        and result.get("status") in {4, "SUCCEEDED"}
        and not result.get("missed_waypoints")
    )


def _validate_route(route: WaypointRoute) -> None:
    if route.frame_id != "map":
        raise ValueError("route frame_id must be 'map'")
    if not route.waypoints:
        raise ValueError("route must contain at least one waypoint")
    for waypoint in route.waypoints:
        _validate_waypoint(waypoint)


def _validate_waypoint(waypoint: Waypoint) -> None:
    if not waypoint.id.strip():
        raise ValueError("waypoint id must be non-empty")
    values = (waypoint.x, waypoint.y, waypoint.yaw, waypoint.dwell_sec)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("waypoint {} contains non-finite values".format(waypoint.id))
    if waypoint.dwell_sec < 0.0:
        raise ValueError("waypoint {} dwell_sec must be non-negative".format(waypoint.id))


def _validate_route_on_costmap(route: WaypointRoute, *, start: Waypoint, costmap) -> None:
    """Require every straight triangle leg to stay in known, non-lethal cells."""
    info = costmap.info
    resolution = float(info.resolution)
    width = int(info.width)
    height = int(info.height)
    if resolution <= 0.0 or width <= 0 or height <= 0:
        raise ValueError("global costmap metadata is invalid")
    if len(costmap.data) != width * height:
        raise ValueError("global costmap data size does not match metadata")

    origin = info.origin
    orientation = origin.orientation
    origin_yaw = math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
    )
    cos_yaw = math.cos(origin_yaw)
    sin_yaw = math.sin(origin_yaw)

    def cell_value(x: float, y: float) -> int:
        dx = x - float(origin.position.x)
        dy = y - float(origin.position.y)
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        col = int(math.floor(local_x / resolution))
        row = int(math.floor(local_y / resolution))
        if col < 0 or row < 0 or col >= width or row >= height:
            raise ValueError("route leaves global costmap at ({:.3f}, {:.3f})".format(x, y))
        return int(costmap.data[row * width + col])

    points = [start] + list(route.waypoints)
    sample_step = max(0.02, resolution * 0.5)
    for leg_index, (leg_start, leg_end) in enumerate(zip(points, points[1:]), 1):
        distance = math.hypot(leg_end.x - leg_start.x, leg_end.y - leg_start.y)
        samples = max(1, int(math.ceil(distance / sample_step)))
        for index in range(samples + 1):
            # The robot is already physically occupying the first sample. Some
            # global costmaps retain a lethal cell under its current footprint.
            if leg_index == 1 and distance * (float(index) / float(samples)) <= 0.35:
                continue
            ratio = float(index) / float(samples)
            x = leg_start.x + (leg_end.x - leg_start.x) * ratio
            y = leg_start.y + (leg_end.y - leg_start.y) * ratio
            value = cell_value(x, y)
            if value != 0:
                raise ValueError(
                    "route leg {} is not free at ({:.3f}, {:.3f}); cost={}".format(
                        leg_index,
                        x,
                        y,
                        value,
                    )
                )
