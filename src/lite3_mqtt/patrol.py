"""Python 3.8-compatible continuous patrol for the ROS2 Foxy container."""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass, field, replace
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
    absolute_waypoints: List[Tuple[float, float]] = field(default_factory=list)

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
        absolute_waypoints = [
            _parse_absolute_waypoint(item, index)
            for index, item in enumerate(data.get("absolute_waypoints", []), 1)
        ]
        mode_count = sum(
            (
                triangle_side is not None,
                bool(forward_distances_m),
                bool(offsets),
                bool(segments),
                bool(absolute_waypoints),
            )
        )
        if mode_count != 1:
            raise ValueError(
                "exactly one of equilateral_triangle_side_m, forward_distances_m, "
                "offsets, segments, or absolute_waypoints must be configured"
            )
        if absolute_waypoints and len(absolute_waypoints) != 2:
            raise ValueError("absolute_waypoints must contain exactly two points")
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
            absolute_waypoints,
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
        elif self.absolute_waypoints:
            route_points = _absolute_waypoint_route(home, self.absolute_waypoints)
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
        """Send the route one navigation goal at a time and wait for completion."""

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
        self._startup_lock = threading.Lock()
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

    def prepare_motion(self) -> None:
        """Prepare remote Nav2/watchdog without capturing a pose or sending a goal."""
        with self._lock:
            if self._emergency_latched:
                raise RuntimeError("motion preparation rejected: emergency stop is latched")
        self._ensure_startup_ready()

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
            self._ensure_startup_ready()
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
                if self.startup_gate is not None:
                    # Route validation can take long enough for a stale
                    # watchdog cancel or an orphan action to appear.  Repeat
                    # the cancel/reset barrier immediately before arming the
                    # watchdog and sending the first goal of every loop.
                    self._ensure_startup_ready()
                if self._stop.is_set():
                    break
                self.backend.prepare_route()
                if self._stop.is_set():
                    break
                result = self.backend.send_route(route)
                if self._stop.is_set():
                    self.logger.info("patrol route stopped result=%s", result)
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

    def _ensure_startup_ready(self) -> None:
        if self.startup_gate is None:
            return
        with self._startup_lock:
            self.startup_gate.ensure_ready()

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
            return_distance = math.hypot(home.x - current.x, home.y - current.y)
            return_yaw = (
                math.atan2(home.y - current.y, home.x - current.x)
                if return_distance > 1e-6
                else current.yaw
            )
            route = WaypointRoute(
                route_id="return_home",
                frame_id=self.config.frame_id,
                loop=False,
                waypoints=[replace(home, id="home_return", yaw=return_yaw)],
            )
            _validate_route(route)
            self.backend.prepare_route()
            if self._return_cancel.is_set():
                self.backend.cancel_active()
                return
            self.backend.validate_route(route, start=current)
            self._ensure_startup_ready()
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
        self.odom_sample_count = 0
        self.localization_converged = None  # type: Optional[bool]
        self.lateral_speed_mps = 0.0
        self.cmd_vel_valid = None  # type: Optional[bool]
        self.odom_frame_ok = None  # type: Optional[bool]
        self.odom_pose_valid = None  # type: Optional[bool]
        self.pose = None  # type: Optional[Waypoint]

    def mark(self, stream: str, now: float) -> None:
        self.last_seen[stream] = now

    def mark_localization(self, *, now: float, converged: bool) -> None:
        self.mark("status", now)
        self.localization_converged = bool(converged)

    def mark_odom(
        self,
        *,
        now: float,
        frame_id: str,
        x: Optional[float] = None,
        y: Optional[float] = None,
        yaw: Optional[float] = None,
    ) -> None:
        self.odom_sample_count += 1
        self.mark("odom", now)
        self.odom_frame_ok = str(frame_id).strip("/") == "map"
        if x is None or y is None or yaw is None:
            self.odom_pose_valid = False
            self.pose = None
            return
        values = (float(x), float(y), float(yaw))
        if all(math.isfinite(value) for value in values):
            self.odom_pose_valid = True
            self.pose = Waypoint("odom", values[0], values[1], values[2])
        else:
            self.odom_pose_valid = False
            self.pose = None

    def arrival_error(self, waypoint: Waypoint):
        if (
            self.pose is None
            or self.odom_frame_ok is not True
            or self.odom_pose_valid is not True
        ):
            return None
        return (
            math.hypot(self.pose.x - waypoint.x, self.pose.y - waypoint.y),
            abs(_normalize(self.pose.yaw - waypoint.yaw)),
        )

    def mark_cmd_vel(self, lateral_speed_mps: float, *, valid: bool = True) -> None:
        value = float(lateral_speed_mps)
        self.cmd_vel_valid = bool(valid) and math.isfinite(value)
        self.lateral_speed_mps = value if self.cmd_vel_valid else 0.0

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
        if self.odom_pose_valid is False:
            reasons.append("odom_pose_invalid")
        if self.cmd_vel_valid is False:
            reasons.append("cmd_vel_invalid")
        elif abs(self.lateral_speed_mps) > max_lateral_speed_mps:
            reasons.append("lateral_cmd_vel")
        return reasons


class NavGoalStatusState:
    """Fail-closed startup barrier for one navigation action server.

    Foxy action servers do not publish an initial empty GoalStatusArray.  A
    never-used, idle server therefore looks identical to a disconnected status
    stream until some goal changes state.  Readiness combines the transient
    status stream with a zero-id/zero-stamp CancelGoal request (cancel all), then
    requires a short quiet period before the watchdog is disarmed.
    """

    TERMINAL_STATUSES = frozenset((4, 5, 6))

    def __init__(self, action_name: str) -> None:
        self.action_name = action_name
        self.received = False
        self.statuses = []  # type: List[int]
        self.nonterminal_goal_ids = set()  # type: set[bytes]
        self.pending_canceled_goal_ids = set()  # type: set[bytes]
        self.status_publisher_seen = False
        self.cancel_service_seen = False
        self.cancel_response_received = False
        self.clean_cancel_rounds = 0
        self.cancel_error = None  # type: Optional[str]
        self.quiet_since = None  # type: Optional[float]
        self.next_cancel_probe_at = 0.0

    def update(self, message) -> None:
        self.received = True
        self.statuses = [int(item.status) for item in message.status_list]
        status_by_goal_id = {}
        nonterminal_goal_ids = set()
        for item in message.status_list:
            goal_id = _goal_id_bytes(getattr(item, "goal_info", None))
            status = int(item.status)
            if goal_id is not None:
                status_by_goal_id[goal_id] = status
                if status not in self.TERMINAL_STATUSES:
                    nonterminal_goal_ids.add(goal_id)
        self.nonterminal_goal_ids = nonterminal_goal_ids
        self.pending_canceled_goal_ids = {
            goal_id
            for goal_id in self.pending_canceled_goal_ids
            if status_by_goal_id.get(goal_id) not in self.TERMINAL_STATUSES
        }
        if self._has_nonterminal_status() or self.pending_canceled_goal_ids:
            self.clean_cancel_rounds = 0
            self.quiet_since = None

    def mark_cancel_service_ready(self, ready: bool) -> None:
        self.cancel_service_seen = bool(ready)
        if not self.cancel_service_seen:
            self.quiet_since = None

    def mark_status_publisher_ready(self, ready: bool) -> None:
        self.status_publisher_seen = bool(ready)
        if not self.status_publisher_seen:
            self.quiet_since = None

    def mark_cancel_error(self, error: object) -> None:
        self.cancel_error = str(error)
        self.quiet_since = None

    def mark_cancel_response(
        self,
        response,
        *,
        now: float,
        probe_interval_sec: float,
    ) -> None:
        self.cancel_response_received = True
        self.next_cancel_probe_at = now + probe_interval_sec
        return_code = int(response.return_code)
        goals_canceling = list(response.goals_canceling)
        # Foxy rcl_action returns ERROR_NONE with an empty list when there is no
        # cancelable goal.  Nav2's SimpleActionServer accepts cancellation for
        # its active goals, so any other code is a fail-closed anomaly here.
        if return_code != 0:
            self.mark_cancel_error("cancel-all return_code={}".format(return_code))
            return
        for goal_info in goals_canceling:
            goal_id = _goal_id_bytes(goal_info)
            if goal_id is None or not any(goal_id):
                self.mark_cancel_error("cancel-all returned an invalid goal id")
                return
            self.pending_canceled_goal_ids.add(goal_id)
        if goals_canceling or self._has_nonterminal_status() or self.pending_canceled_goal_ids:
            self.clean_cancel_rounds = 0
        else:
            self.clean_cancel_rounds += 1
        self.quiet_since = None

    def cancel_probe_needed(self, *, now: float, pending: bool) -> bool:
        if pending or self.cancel_error is not None:
            return False
        return (
            not self.cancel_response_received
            or self.clean_cancel_rounds < 2
            and now >= self.next_cancel_probe_at
        )

    def observe_quiet(self, *, now: float) -> None:
        if (
            self.cancel_error is not None
            or not self.cancel_response_received
            or self._has_nonterminal_status()
            or self.pending_canceled_goal_ids
            or self.clean_cancel_rounds < 2
        ):
            self.quiet_since = None
        elif self.quiet_since is None:
            self.quiet_since = now

    def blocking_reasons(self, *, now: float, quiet_sec: float) -> List[str]:
        if self.cancel_error is not None:
            return [
                "navigation_cancel_error:{}:{}".format(
                    self.action_name,
                    self.cancel_error,
                )
            ]
        if not self.status_publisher_seen:
            return ["navigation_status_publisher_missing:{}".format(self.action_name)]
        if not self.cancel_service_seen:
            return ["navigation_cancel_service_missing:{}".format(self.action_name)]
        if not self.cancel_response_received:
            return ["navigation_cancel_response_missing:{}".format(self.action_name)]
        nonterminal = self._nonterminal_statuses()
        if nonterminal:
            return [
                "navigation_goal_not_terminal:{}".format(
                    "{}:{}".format(
                        self.action_name,
                        ",".join(str(status) for status in nonterminal),
                    )
                )
            ]
        if self.pending_canceled_goal_ids:
            return [
                "navigation_canceled_goal_terminal_status_missing:{}:{}".format(
                    self.action_name,
                    len(self.pending_canceled_goal_ids),
                )
            ]
        if self.clean_cancel_rounds < 2:
            return [
                "navigation_idle_cancel_rounds:{}:{}/2".format(
                    self.action_name,
                    self.clean_cancel_rounds,
                )
            ]
        if self.quiet_since is None or now - self.quiet_since < quiet_sec:
            return ["navigation_idle_settling:{}".format(self.action_name)]
        return []

    def _nonterminal_statuses(self) -> List[int]:
        return [
            status for status in self.statuses if status not in self.TERMINAL_STATUSES
        ]

    def _has_nonterminal_status(self) -> bool:
        return bool(self._nonterminal_statuses())


def _goal_id_bytes(goal_info) -> Optional[bytes]:
    goal_id = getattr(goal_info, "goal_id", None)
    raw_uuid = getattr(goal_id, "uuid", None)
    if raw_uuid is None:
        return None
    try:
        return bytes(raw_uuid)
    except (TypeError, ValueError):
        return None


class Nav2PatrolBackend:
    """Cancelable ROS2 Foxy backend using one NavigateToPose goal per leg."""

    ARRIVAL_SETTLE_ODOM_SAMPLES = 2

    def __init__(
        self,
        *,
        odom_topic: str = "/odom",
        action_name: str = "/navigate_to_pose",
        compute_path_action_name: str = "/compute_path_to_pose",
        timeout_sec: float = 10.0,
        max_data_age_sec: float = 2.0,
        max_lateral_speed_mps: float = 0.02,
        route_timeout_sec: float = 300.0,
        cancel_timeout_sec: float = 5.0,
        goal_acceptance_timeout_sec: float = 5.0,
        route_clearance_m: float = 0.50,
        max_path_detour_ratio: float = 3.0,
        arrival_position_tolerance_m: float = 0.30,
        arrival_yaw_tolerance_rad: float = 0.35,
        arrival_retry_limit: int = 1,
        progress_timeout_sec: float = 20.0,
        progress_distance_m: float = 0.10,
        progress_yaw_rad: float = 0.15,
        nav_idle_quiet_sec: float = 0.5,
    ) -> None:
        if min(
            timeout_sec,
            max_data_age_sec,
            route_timeout_sec,
            cancel_timeout_sec,
            goal_acceptance_timeout_sec,
        ) <= 0.0:
            raise ValueError("Nav2 timeout values must be positive")
        if max_lateral_speed_mps < 0.0:
            raise ValueError("max_lateral_speed_mps must be non-negative")
        if route_clearance_m < 0.0:
            raise ValueError("route_clearance_m must be non-negative")
        if max_path_detour_ratio < 1.0:
            raise ValueError("max_path_detour_ratio must be >= 1")
        if min(arrival_position_tolerance_m, arrival_yaw_tolerance_rad) <= 0.0:
            raise ValueError("arrival tolerances must be positive")
        if (
            isinstance(arrival_retry_limit, bool)
            or not isinstance(arrival_retry_limit, int)
            or arrival_retry_limit not in (0, 1)
        ):
            raise ValueError("arrival_retry_limit must be 0 or 1")
        if min(progress_timeout_sec, progress_distance_m, progress_yaw_rad) <= 0.0:
            raise ValueError("progress thresholds must be positive")
        if nav_idle_quiet_sec <= 0.0:
            raise ValueError("nav_idle_quiet_sec must be positive")
        self.odom_topic = odom_topic
        self.action_name = action_name
        self.compute_path_action_name = compute_path_action_name
        self.timeout_sec = timeout_sec
        self.max_data_age_sec = max_data_age_sec
        self.max_lateral_speed_mps = max_lateral_speed_mps
        self.route_timeout_sec = route_timeout_sec
        self.cancel_timeout_sec = cancel_timeout_sec
        self.goal_acceptance_timeout_sec = goal_acceptance_timeout_sec
        self.route_clearance_m = route_clearance_m
        self.max_path_detour_ratio = max_path_detour_ratio
        self.arrival_position_tolerance_m = arrival_position_tolerance_m
        self.arrival_yaw_tolerance_rad = arrival_yaw_tolerance_rad
        self.arrival_retry_limit = arrival_retry_limit
        self.progress_timeout_sec = progress_timeout_sec
        self.progress_distance_m = progress_distance_m
        self.progress_yaw_rad = progress_yaw_rad
        self.nav_idle_quiet_sec = nav_idle_quiet_sec
        self._cancel = threading.Event()
        self._goal_state_uncertain = False

    def prepare_route(self) -> None:
        if self._goal_state_uncertain:
            raise RuntimeError(
                "navigation goal state is uncertain; restart the MQTT runtime "
                "after confirming Nav2 has no active goal"
            )
        self._cancel.clear()

    def validate_route(self, route: WaypointRoute, *, start: Waypoint) -> None:
        # NavigateToPose performs planning, costmap clearing and recovery as one
        # behavior-tree operation. The added direct ComputePathToPose preflight
        # bypassed those recoveries and rejected maps that NavigateToPose had
        # already driven successfully. Keep structural validation here and let
        # Nav2 validate each live leg when it is executed.
        _validate_route(route)
        _validate_waypoint(start)

    def _compute_path(
        self,
        node,
        client,
        *,
        frame_id: str,
        goal: Waypoint,
        leg_index: int,
    ):
        import rclpy
        from nav2_msgs.action import ComputePathToPose

        request = ComputePathToPose.Goal()
        # ROS2 Foxy interface: geometry_msgs/PoseStamped pose + planner_id.
        request.pose = _waypoint_pose_stamped(node, goal, frame_id)
        send_future = client.send_goal_async(request)
        deadline = time.monotonic() + self.timeout_sec
        while rclpy.ok() and not send_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if self._cancel.is_set():
                raise RuntimeError("route preflight canceled")
        if not send_future.done():
            raise TimeoutError("compute-path leg {} acceptance timed out".format(leg_index))
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise ValueError("compute-path leg {} was rejected".format(leg_index))

        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + self.timeout_sec
        while rclpy.ok() and not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if self._cancel.is_set():
                goal_handle.cancel_goal_async()
                raise RuntimeError("route preflight canceled")
        if not result_future.done():
            goal_handle.cancel_goal_async()
            raise TimeoutError("compute-path leg {} result timed out".format(leg_index))
        wrapped = result_future.result()
        if wrapped is None or wrapped.status != 4:
            status = None if wrapped is None else wrapped.status
            raise ValueError(
                "compute-path leg {} failed status={}".format(leg_index, status)
            )
        return wrapped.result.path

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
            try:
                yaw = _quaternion_yaw(msg.pose.pose.orientation)
            except ValueError:
                captured["error"] = "{} contains non-finite pose values".format(
                    self.odom_topic
                )
                return
            if not all(math.isfinite(float(value)) for value in (position.x, position.y)):
                captured["error"] = "{} contains non-finite pose values".format(
                    self.odom_topic
                )
                return
            captured["pose"] = Waypoint(
                id=waypoint_id,
                x=float(position.x),
                y=float(position.y),
                yaw=yaw,
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
        from action_msgs.msg import GoalStatusArray
        from action_msgs.srv import CancelGoal
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import UInt64

        rclpy.init(args=None)
        node = rclpy.create_node("lite3_mqtt_nav_readiness")
        client = ActionClient(node, NavigateToPose, self.action_name)
        safety = NavSafetyState()
        subscriptions = _create_safety_subscriptions(node, safety, self.odom_topic)
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        guarded_actions = []
        for action_name in (
            self.action_name.rstrip("/"),
            "/FollowWaypoints",
            "/follow_path",
            "/spin",
            "/backup",
            "/wait",
        ):
            if action_name and action_name not in guarded_actions:
                guarded_actions.append(action_name)
        goal_status_states = []
        cancel_clients = {}
        cancel_futures = {}
        for action_name in guarded_actions:
            goal_status = NavGoalStatusState(action_name)
            goal_status_states.append(goal_status)
            subscriptions.append(
                node.create_subscription(
                    GoalStatusArray,
                    action_name + "/_action/status",
                    goal_status.update,
                    status_qos,
                )
            )
            cancel_clients[action_name] = node.create_client(
                CancelGoal,
                action_name + "/_action/cancel_goal",
            )
            cancel_futures[action_name] = None
        watchdog_reset = {"token": None, "acked": False, "next_publish_at": 0.0}

        def on_watchdog_reset_ack(message) -> None:
            token = watchdog_reset["token"]
            if token is not None and int(message.data) == token:
                watchdog_reset["acked"] = True

        subscriptions.append(
            node.create_subscription(
                UInt64,
                "/lite3/nav/watchdog_reset_ack",
                on_watchdog_reset_ack,
                10,
            )
        )
        watchdog_reset_pub = node.create_publisher(
            UInt64,
            "/lite3/nav/watchdog_reset",
            10,
        )
        deadline = time.monotonic() + (
            self.timeout_sec if timeout_sec is None else timeout_sec
        )
        last_reasons = []
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                now = time.monotonic()
                action_ready = client.wait_for_server(timeout_sec=0.1)
                last_reasons = _nav_graph_reasons(
                    node,
                    action_ready=action_ready,
                    action_name=self.action_name,
                    odom_topic=self.odom_topic,
                )
                last_reasons.extend(
                    safety.blocking_reasons(
                        now=now,
                        max_age_sec=self.max_data_age_sec,
                        max_lateral_speed_mps=self.max_lateral_speed_mps,
                    )
                )
                for goal_status in goal_status_states:
                    action_name = goal_status.action_name
                    try:
                        status_publishers = node.get_publishers_info_by_topic(
                            action_name + "/_action/status"
                        )
                    except Exception:
                        status_publishers = []
                    goal_status.mark_status_publisher_ready(bool(status_publishers))
                    cancel_client = cancel_clients[action_name]
                    cancel_ready = cancel_client.service_is_ready()
                    if not cancel_ready:
                        cancel_ready = cancel_client.wait_for_service(timeout_sec=0.0)
                    goal_status.mark_cancel_service_ready(cancel_ready)

                    future = cancel_futures[action_name]
                    if future is not None and future.done():
                        try:
                            response = future.result()
                            if response is None:
                                raise RuntimeError("empty cancel-all response")
                        except Exception as exc:
                            goal_status.mark_cancel_error(exc)
                        else:
                            goal_status.mark_cancel_response(
                                response,
                                now=now,
                                probe_interval_sec=self.nav_idle_quiet_sec,
                            )
                        cancel_futures[action_name] = None
                        future = None

                    pending = future is not None
                    if (
                        cancel_ready
                        and goal_status.cancel_probe_needed(now=now, pending=pending)
                    ):
                        # Default GoalInfo is an all-zero UUID and timestamp,
                        # which Foxy defines as cancel every goal on this action.
                        # This never commands motion and makes orphan actions
                        # observable before the watchdog is disarmed.
                        cancel_futures[action_name] = cancel_client.call_async(
                            CancelGoal.Request()
                        )
                    goal_status.observe_quiet(now=now)
                    last_reasons.extend(
                        goal_status.blocking_reasons(
                            now=now,
                            quiet_sec=self.nav_idle_quiet_sec,
                        )
                    )
                if not last_reasons:
                    last_reasons.extend(_controller_nonholonomic_reasons(node))
                if not last_reasons:
                    if watchdog_reset["token"] is None:
                        watchdog_reset["token"] = _new_nav_token()
                        watchdog_reset["acked"] = False
                        _publish_token(
                            watchdog_reset_pub,
                            UInt64,
                            watchdog_reset["token"],
                        )
                        watchdog_reset["next_publish_at"] = now + 1.0
                    elif (
                        not watchdog_reset["acked"]
                        and now >= watchdog_reset["next_publish_at"]
                    ):
                        _publish_token(
                            watchdog_reset_pub,
                            UInt64,
                            watchdog_reset["token"],
                        )
                        watchdog_reset["next_publish_at"] = now + 1.0
                    if watchdog_reset["acked"]:
                        return
                    last_reasons.append("watchdog_reset_ack_missing")
                elif watchdog_reset["token"] is not None:
                    # An old acknowledgment must not authorize a later state
                    # after readiness has become unsafe again.
                    watchdog_reset["token"] = None
                    watchdog_reset["acked"] = False
                    watchdog_reset["next_publish_at"] = 0.0
            raise TimeoutError("Nav2 DDS readiness timed out: {}".format(", ".join(last_reasons)))
        finally:
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            for cancel_client in cancel_clients.values():
                node.destroy_client(cancel_client)
            node.destroy_publisher(watchdog_reset_pub)
            client.destroy()
            node.destroy_node()
            rclpy.shutdown()

    def send_route(self, route: WaypointRoute) -> Dict[str, object]:
        import rclpy
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        from std_msgs.msg import UInt64

        rclpy.init(args=None)
        node = rclpy.create_node("lite3_mqtt_waypoint_patrol")
        client = ActionClient(node, NavigateToPose, self.action_name)
        heartbeat_pub = node.create_publisher(UInt64, "/lite3/nav/heartbeat", 10)
        safety = NavSafetyState()
        subscriptions = _create_safety_subscriptions(node, safety, self.odom_topic)
        watchdog_arm = {"token": None, "acked": False}

        def on_watchdog_arm_ack(message) -> None:
            token = watchdog_arm["token"]
            if token is not None and int(message.data) == token:
                watchdog_arm["acked"] = True

        subscriptions.append(
            node.create_subscription(
                UInt64,
                "/lite3/nav/watchdog_arm_ack",
                on_watchdog_arm_ack,
                10,
            )
        )
        # True means a goal request may still be accepted or executing even if
        # this process can no longer observe a terminal action result.  In that
        # case the heartbeat must expire naturally so the perception-host
        # watchdog keeps canceling goals and publishing zero velocity.
        goal_may_be_active = False
        any_goal_accepted = False
        try:
            server_deadline = time.monotonic() + self.timeout_sec
            action_ready = False
            while not self._cancel.is_set() and time.monotonic() < server_deadline:
                if client.wait_for_server(timeout_sec=0.1):
                    action_ready = True
                    break
            if self._cancel.is_set():
                return {"accepted": False, "status": "CANCELED", "missed_waypoints": []}
            if not action_ready:
                raise TimeoutError("{} action server is not available".format(self.action_name))

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

            route_deadline = time.monotonic() + self.route_timeout_sec
            watchdog_arm["token"] = _new_nav_token()
            watchdog_arm["acked"] = False
            arm_deadline = min(
                time.monotonic() + self.timeout_sec,
                route_deadline,
            )
            next_arm_publish = 0.0
            arm_failure_reason = None
            while rclpy.ok() and not watchdog_arm["acked"]:
                now = time.monotonic()
                if self._cancel.is_set():
                    arm_failure_reason = "operator_cancel"
                    break
                safety_reasons = safety.blocking_reasons(
                    now=now,
                    max_age_sec=self.max_data_age_sec,
                    max_lateral_speed_mps=self.max_lateral_speed_mps,
                )
                if safety_reasons:
                    arm_failure_reason = "safety:" + ",".join(safety_reasons)
                    break
                if now >= arm_deadline:
                    arm_failure_reason = "watchdog_arm_ack_timeout"
                    break
                if now >= next_arm_publish:
                    _publish_nav_heartbeat(
                        heartbeat_pub,
                        UInt64,
                        token=watchdog_arm["token"],
                    )
                    next_arm_publish = now + 0.2
                rclpy.spin_once(node, timeout_sec=0.1)
            if not watchdog_arm["acked"]:
                return {
                    "accepted": False,
                    "status": "WATCHDOG_ARM_FAILED",
                    "missed_waypoints": [0],
                    "reason": arm_failure_reason or "rclpy_shutdown",
                }

            next_heartbeat = time.monotonic() + 0.5
            waypoint_index = 0
            arrival_retry_count = 0
            while waypoint_index < len(route.waypoints):
                waypoint = route.waypoints[waypoint_index]
                if self._cancel.is_set():
                    return {
                        "accepted": any_goal_accepted,
                        "status": "CANCELED",
                        "missed_waypoints": [waypoint_index],
                        "reason": "operator_cancel",
                    }
                if time.monotonic() >= route_deadline:
                    return {
                        "accepted": any_goal_accepted,
                        "status": "CANCELED",
                        "missed_waypoints": [waypoint_index],
                        "reason": "route_timeout",
                    }
                _wait_for_safe_samples(
                    node,
                    safety,
                    timeout_sec=min(
                        self.timeout_sec,
                        max(0.1, route_deadline - time.monotonic()),
                    ),
                    max_age_sec=self.max_data_age_sec,
                    max_lateral_speed_mps=self.max_lateral_speed_mps,
                    cancel=self._cancel,
                )
                if self._cancel.is_set():
                    return {
                        "accepted": any_goal_accepted,
                        "status": "CANCELED",
                        "missed_waypoints": [waypoint_index],
                        "reason": "operator_cancel",
                    }

                goal = NavigateToPose.Goal()
                goal.pose = _waypoint_pose_stamped(node, waypoint, route.frame_id)
                send_future = client.send_goal_async(goal)
                goal_may_be_active = True
                acceptance_deadline = min(
                    time.monotonic() + self.goal_acceptance_timeout_sec,
                    route_deadline,
                )
                acceptance_cancel_reason = None
                next_acceptance_graph_check = time.monotonic()
                while rclpy.ok() and not send_future.done():
                    rclpy.spin_once(node, timeout_sec=0.1)
                    now = time.monotonic()
                    if acceptance_cancel_reason is None and self._cancel.is_set():
                        acceptance_cancel_reason = "operator_cancel"
                        acceptance_deadline = min(
                            acceptance_deadline,
                            now + self.cancel_timeout_sec,
                        )
                    if acceptance_cancel_reason is None and now >= route_deadline:
                        acceptance_cancel_reason = "route_timeout"
                    if acceptance_cancel_reason is None:
                        reasons = safety.blocking_reasons(
                            now=now,
                            max_age_sec=self.max_data_age_sec,
                            max_lateral_speed_mps=self.max_lateral_speed_mps,
                        )
                        if reasons:
                            acceptance_cancel_reason = "safety:" + ",".join(reasons)
                    if (
                        acceptance_cancel_reason is None
                        and now >= next_acceptance_graph_check
                    ):
                        graph_reasons = _nav_graph_reasons(
                            node,
                            action_ready=client.wait_for_server(timeout_sec=0.0),
                            action_name=self.action_name,
                            odom_topic=self.odom_topic,
                        )
                        if graph_reasons:
                            acceptance_cancel_reason = "graph:" + ",".join(graph_reasons)
                        next_acceptance_graph_check = now + 1.0
                    if acceptance_cancel_reason is not None:
                        # The action server may already be executing even though
                        # its acceptance response is unavailable.  Stop refreshing
                        # the heartbeat so the perception-host watchdog cancels
                        # the goal independently, and bound the local wait.
                        acceptance_deadline = min(
                            acceptance_deadline,
                            now + self.cancel_timeout_sec,
                        )
                    elif now >= next_heartbeat:
                        _publish_nav_heartbeat(
                            heartbeat_pub,
                            UInt64,
                            token=watchdog_arm["token"],
                        )
                        next_heartbeat = now + 0.5
                    if now >= acceptance_deadline:
                        return {
                            "accepted": any_goal_accepted,
                            "status": "CANCEL_TIMEOUT"
                            if acceptance_cancel_reason is not None
                            else "ACCEPTANCE_TIMEOUT",
                            "missed_waypoints": [waypoint_index],
                            "reason": acceptance_cancel_reason
                            or "goal_acceptance_timeout",
                        }
                if not send_future.done():
                    return {
                        "accepted": any_goal_accepted,
                        "status": "CANCELED",
                        "missed_waypoints": [waypoint_index],
                        "reason": "rclpy_shutdown",
                    }
                goal_handle = send_future.result()
                if goal_handle is None or not goal_handle.accepted:
                    if goal_handle is not None:
                        goal_may_be_active = False
                    return {
                        "accepted": any_goal_accepted,
                        "status": None,
                        "missed_waypoints": [waypoint_index],
                        "reason": "goal_rejected",
                    }
                any_goal_accepted = True

                result_future = goal_handle.get_result_async()
                cancel_future = None
                cancel_deadline = None
                cancel_reason = acceptance_cancel_reason
                if cancel_reason is not None:
                    cancel_future = goal_handle.cancel_goal_async()
                    cancel_deadline = time.monotonic() + self.cancel_timeout_sec
                next_graph_check = time.monotonic() + 1.0
                last_progress_pose = safety.pose
                last_progress_at = time.monotonic()
                while rclpy.ok() and not result_future.done():
                    rclpy.spin_once(node, timeout_sec=0.1)
                    now = time.monotonic()
                    if cancel_reason is None and self._cancel.is_set():
                        cancel_reason = "operator_cancel"
                    if cancel_reason is None and now >= route_deadline:
                        cancel_reason = "route_timeout"
                    current_pose = safety.pose
                    if current_pose is not None:
                        if last_progress_pose is None or _goal_progressed(
                            last_progress_pose,
                            current_pose,
                            waypoint,
                            distance_m=self.progress_distance_m,
                            yaw_rad=self.progress_yaw_rad,
                            yaw_progress_position_m=self.arrival_position_tolerance_m,
                        ):
                            last_progress_pose = current_pose
                            last_progress_at = now
                    if (
                        cancel_reason is None
                        and now - last_progress_at >= self.progress_timeout_sec
                    ):
                        cancel_reason = "stalled"
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
                    if cancel_reason is None and now >= next_heartbeat:
                        _publish_nav_heartbeat(
                            heartbeat_pub,
                            UInt64,
                            token=watchdog_arm["token"],
                        )
                        next_heartbeat = now + 0.5
                    if cancel_deadline is not None and now >= cancel_deadline:
                        return {
                            "accepted": True,
                            "status": "CANCEL_TIMEOUT",
                            "missed_waypoints": [waypoint_index],
                            "reason": cancel_reason,
                        }
                if not result_future.done():
                    return {
                        "accepted": True,
                        "status": "CANCELED",
                        "missed_waypoints": [waypoint_index],
                        "reason": cancel_reason or "rclpy_shutdown",
                    }
                terminal_observed_at = time.monotonic()
                result = result_future.result()
                if result is not None:
                    goal_may_be_active = False
                if result is None or result.status != 4 or cancel_reason is not None:
                    return {
                        "accepted": True,
                        "status": None if result is None else result.status,
                        "missed_waypoints": [waypoint_index],
                        "reason": cancel_reason,
                    }
                # Nav2 may report success at an approximate endpoint.  Settle
                # on at least two map-frame odometry samples received after the
                # terminal result and validate the newest one before advancing.
                # This avoids accepting a queued or transient first sample.
                terminal_odom_sample_count = safety.odom_sample_count
                arrival_sample_deadline = terminal_observed_at + min(
                    self.timeout_sec,
                    self.max_data_age_sec,
                )
                fresh_arrival_samples = 0
                while (
                    rclpy.ok()
                    and not self._cancel.is_set()
                    and time.monotonic() < arrival_sample_deadline
                ):
                    rclpy.spin_once(node, timeout_sec=0.05)
                    now = time.monotonic()
                    if now >= next_heartbeat:
                        _publish_nav_heartbeat(
                            heartbeat_pub,
                            UInt64,
                            token=watchdog_arm["token"],
                        )
                        next_heartbeat = now + 0.5
                    fresh_arrival_samples = (
                        safety.odom_sample_count - terminal_odom_sample_count
                    )
                    if fresh_arrival_samples >= self.ARRIVAL_SETTLE_ODOM_SAMPLES:
                        break
                if self._cancel.is_set():
                    return {
                        "accepted": True,
                        "status": "CANCELED",
                        "missed_waypoints": [waypoint_index],
                        "reason": "operator_cancel",
                    }
                if fresh_arrival_samples < self.ARRIVAL_SETTLE_ODOM_SAMPLES:
                    result = {
                        "accepted": True,
                        "status": "ARRIVAL_MISMATCH",
                        "missed_waypoints": [waypoint_index],
                        "reason": "arrival_pose_stale",
                        "fresh_odom_samples": fresh_arrival_samples,
                    }
                    result.update(
                        _arrival_diagnostics(
                            waypoint_index=waypoint_index,
                            waypoint=waypoint,
                            actual=safety.pose,
                            retry_count=arrival_retry_count,
                        )
                    )
                    return result
                arrival_error = safety.arrival_error(waypoint)
                if arrival_error is None:
                    result = {
                        "accepted": True,
                        "status": "ARRIVAL_MISMATCH",
                        "missed_waypoints": [waypoint_index],
                        "reason": "arrival_pose_missing",
                    }
                    result.update(
                        _arrival_diagnostics(
                            waypoint_index=waypoint_index,
                            waypoint=waypoint,
                            actual=safety.pose,
                            retry_count=arrival_retry_count,
                        )
                    )
                    return result
                position_error, yaw_error = arrival_error
                if (
                    position_error > self.arrival_position_tolerance_m
                    or yaw_error > self.arrival_yaw_tolerance_rad
                ):
                    if (
                        arrival_retry_count < self.arrival_retry_limit
                        and position_error <= self.route_clearance_m
                    ):
                        arrival_retry_count += 1
                        logging.getLogger(__name__).warning(
                            "arrival mismatch waypoint=%s index=%s "
                            "target=(%.3f, %.3f, %.3f) "
                            "actual=(%.3f, %.3f, %.3f) "
                            "position_error=%.3f yaw_error=%.3f "
                            "retry_count=%s/%s; retrying same waypoint",
                            waypoint.id,
                            waypoint_index,
                            waypoint.x,
                            waypoint.y,
                            waypoint.yaw,
                            safety.pose.x,
                            safety.pose.y,
                            safety.pose.yaw,
                            position_error,
                            yaw_error,
                            arrival_retry_count,
                            self.arrival_retry_limit,
                        )
                        continue
                    result = {
                        "accepted": True,
                        "status": "ARRIVAL_MISMATCH",
                        "missed_waypoints": [waypoint_index],
                        "reason": "arrival_mismatch",
                        "position_error_m": position_error,
                        "yaw_error_rad": yaw_error,
                    }
                    result.update(
                        _arrival_diagnostics(
                            waypoint_index=waypoint_index,
                            waypoint=waypoint,
                            actual=safety.pose,
                            retry_count=arrival_retry_count,
                        )
                    )
                    return result

                dwell_deadline = time.monotonic() + waypoint.dwell_sec
                while (
                    waypoint.dwell_sec > 0.0
                    and rclpy.ok()
                    and not self._cancel.is_set()
                    and time.monotonic() < dwell_deadline
                ):
                    rclpy.spin_once(node, timeout_sec=0.1)
                    now = time.monotonic()
                    if now >= next_heartbeat:
                        _publish_nav_heartbeat(
                            heartbeat_pub,
                            UInt64,
                            token=watchdog_arm["token"],
                        )
                        next_heartbeat = now + 0.5
                waypoint_index += 1
                arrival_retry_count = 0
            return {
                "accepted": True,
                "status": 4,
                "missed_waypoints": [],
                "reason": None,
            }
        finally:
            if goal_may_be_active:
                self._goal_state_uncertain = True
            elif rclpy.ok():
                _publish_nav_disarm(heartbeat_pub, UInt64)
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            node.destroy_publisher(heartbeat_pub)
            client.destroy()
            node.destroy_node()
            rclpy.shutdown()


def _twist_is_finite(message) -> bool:
    return all(
        math.isfinite(float(value))
        for value in (
            message.linear.x,
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
            message.angular.z,
        )
    )


def _create_safety_subscriptions(node, state: NavSafetyState, odom_topic: str):
    from geometry_msgs.msg import Twist
    from hdl_localization.msg import ScanMatchingStatus
    from nav_msgs.msg import OccupancyGrid, Odometry

    def now() -> float:
        return time.monotonic()

    def on_odom(msg) -> None:
        try:
            yaw = _quaternion_yaw(msg.pose.pose.orientation)
        except ValueError:
            yaw = float("nan")
        state.mark_odom(
            now=now(),
            frame_id=msg.header.frame_id,
            x=msg.pose.pose.position.x,
            y=msg.pose.pose.position.y,
            yaw=yaw,
        )

    return [
        node.create_subscription(
            Odometry,
            odom_topic,
            on_odom,
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
            lambda msg: state.mark_cmd_vel(
                msg.linear.y,
                valid=_twist_is_finite(msg),
            ),
            10,
        ),
    ]


def _new_nav_token() -> int:
    return (time.monotonic_ns() & ((1 << 64) - 1)) or 1


def _publish_token(publisher, message_type, token: int) -> None:
    message = message_type()
    message.data = int(token)
    publisher.publish(message)


def _publish_nav_heartbeat(publisher, message_type, token: Optional[int] = None) -> int:
    token = _new_nav_token() if token is None else int(token)
    _publish_token(publisher, message_type, token)
    return token


def _publish_nav_disarm(publisher, message_type) -> None:
    message = message_type()
    message.data = 0
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
        "/bt_navigator",
        "/lite3_nav_watchdog",
    }
    required_topics = {
        odom_topic,
        "/status",
        "/map",
        "/global_costmap/costmap",
        "/local_costmap/costmap",
        "/cmd_vel",
        "/lite3/nav/heartbeat",
    }
    discovered_nodes = []
    for name, namespace in node.get_node_names_and_namespaces():
        prefix = "" if namespace == "/" else namespace.rstrip("/")
        discovered_nodes.append("{}/{}".format(prefix, name))
    nodes = set(discovered_nodes)
    topics = {name for name, _ in node.get_topic_names_and_types()}
    reasons = []
    reasons.extend("missing node {}".format(name) for name in sorted(required_nodes - nodes))
    reasons.extend("missing topic {}".format(name) for name in sorted(required_topics - topics))
    if discovered_nodes.count("/motion_sender") != 1:
        reasons.append(
            "expected exactly one /motion_sender node; found {}".format(
                discovered_nodes.count("/motion_sender")
            )
        )
    if not action_ready:
        reasons.append("{} action unavailable".format(action_name))

    publishers = node.get_publishers_info_by_topic("/cmd_vel")
    publisher_names = {info.node_name.lstrip("/") for info in publishers}
    if "controller_server" not in publisher_names:
        reasons.append("controller_server is not publishing /cmd_vel")
    if "lite3_nav_watchdog" not in publisher_names:
        reasons.append("lite3_nav_watchdog is not publishing /cmd_vel")
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
    """Build the last-known-good three-pose FollowWaypoints triangle."""
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
        replace(home, id="home_return", yaw=_normalize(heading)),
    ]


def _absolute_waypoint_route(
    home: Waypoint,
    points: List[Tuple[float, float]],
) -> List[Waypoint]:
    """Build fixed map goals while keeping every travel leg forward-facing."""
    route = []
    source_x, source_y = home.x, home.y
    for index, (x, y) in enumerate(points, 1):
        yaw = math.atan2(y - source_y, x - source_x)
        route.append(Waypoint("p{}".format(index), x, y, yaw))
        source_x, source_y = x, y
    return_yaw = math.atan2(home.y - source_y, home.x - source_x)
    route.append(replace(home, id="home_return", yaw=return_yaw))
    return route


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


def _parse_absolute_waypoint(item: Any, index: int) -> Tuple[float, float]:
    if not isinstance(item, dict):
        raise ValueError("absolute_waypoints #{} must be a mapping".format(index))
    return (
        _required_number(item, "x"),
        _required_number(item, "y"),
    )


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


def _quaternion_yaw(orientation) -> float:
    x = float(orientation.x)
    y = float(orientation.y)
    z = float(orientation.z)
    w = float(orientation.w)
    if not all(math.isfinite(value) for value in (x, y, z, w)):
        raise ValueError("quaternion contains non-finite values")
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-9:
        raise ValueError("quaternion has zero norm")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def _goal_progressed(
    reference: Waypoint,
    current: Waypoint,
    goal: Waypoint,
    *,
    distance_m: float,
    yaw_rad: float,
    yaw_progress_position_m: float,
) -> bool:
    reference_distance = math.hypot(goal.x - reference.x, goal.y - reference.y)
    current_distance = math.hypot(goal.x - current.x, goal.y - current.y)
    reference_yaw_error = abs(_normalize(goal.yaw - reference.yaw))
    current_yaw_error = abs(_normalize(goal.yaw - current.yaw))
    return (
        reference_distance - current_distance >= distance_m
        or (
            current_distance <= yaw_progress_position_m
            and reference_yaw_error - current_yaw_error >= yaw_rad
        )
    )


def _arrival_diagnostics(
    *,
    waypoint_index: int,
    waypoint: Waypoint,
    actual: Optional[Waypoint],
    retry_count: int,
) -> Dict[str, object]:
    return {
        "waypoint_index": waypoint_index,
        "waypoint_id": waypoint.id,
        "target_pose": _pose_diagnostics(waypoint),
        "actual_pose": _pose_diagnostics(actual),
        "retry_count": retry_count,
    }


def _pose_diagnostics(pose: Optional[Waypoint]):
    if pose is None:
        return None
    return {"x": pose.x, "y": pose.y, "yaw": pose.yaw}


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


def _waypoint_pose_stamped(node, waypoint: Waypoint, frame_id: str):
    from geometry_msgs.msg import PoseStamped

    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = waypoint.x
    pose.pose.position.y = waypoint.y
    half_yaw = waypoint.yaw / 2.0
    pose.pose.orientation.z = math.sin(half_yaw)
    pose.pose.orientation.w = math.cos(half_yaw)
    return pose


class _CostmapView:
    def __init__(self, costmap) -> None:
        info = costmap.info
        self.resolution = float(info.resolution)
        self.width = int(info.width)
        self.height = int(info.height)
        if self.resolution <= 0.0 or self.width <= 0 or self.height <= 0:
            raise ValueError("occupancy grid metadata is invalid")
        if len(costmap.data) != self.width * self.height:
            raise ValueError("occupancy grid data size does not match metadata")
        self.data = costmap.data
        self.origin_x = float(info.origin.position.x)
        self.origin_y = float(info.origin.position.y)
        orientation = info.origin.orientation
        origin_yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
        )
        self.cos_yaw = math.cos(origin_yaw)
        self.sin_yaw = math.sin(origin_yaw)

    def cell_value(self, x: float, y: float) -> int:
        dx = x - self.origin_x
        dy = y - self.origin_y
        local_x = self.cos_yaw * dx + self.sin_yaw * dy
        local_y = -self.sin_yaw * dx + self.cos_yaw * dy
        col = int(math.floor(local_x / self.resolution))
        row = int(math.floor(local_y / self.resolution))
        if col < 0 or row < 0 or col >= self.width or row >= self.height:
            raise ValueError("route leaves occupancy grid at ({:.3f}, {:.3f})".format(x, y))
        return int(self.data[row * self.width + col])

    def require_known_free(
        self,
        x: float,
        y: float,
        *,
        label: str,
    ) -> None:
        value = self.cell_value(x, y)
        if value != 0:
            raise ValueError(
                "{} is not known free at ({:.3f}, {:.3f}); cost={}".format(
                    label,
                    x,
                    y,
                    value,
                )
            )

    def require_clearance(
        self,
        x: float,
        y: float,
        *,
        clearance_m: float,
        label: str,
    ) -> None:
        self.require_known_free(x, y, label=label)
        if clearance_m <= 0.0:
            return
        cells = int(math.ceil(clearance_m / self.resolution))
        for row_offset in range(-cells, cells + 1):
            for col_offset in range(-cells, cells + 1):
                dx = float(col_offset) * self.resolution
                dy = float(row_offset) * self.resolution
                if math.hypot(dx, dy) > clearance_m:
                    continue
                sample_x = x + self.cos_yaw * dx - self.sin_yaw * dy
                sample_y = y + self.sin_yaw * dx + self.cos_yaw * dy
                self.require_known_free(sample_x, sample_y, label=label)

    def require_corridor(
        self,
        start: Waypoint,
        goal: Waypoint,
        *,
        clearance_m: float,
        label: str,
    ) -> None:
        distance = math.hypot(goal.x - start.x, goal.y - start.y)
        sample_step = max(0.02, self.resolution * 0.5)
        sample_count = max(1, int(math.ceil(distance / sample_step)))
        for sample_index in range(1, sample_count + 1):
            ratio = float(sample_index) / float(sample_count)
            self.require_clearance(
                start.x + (goal.x - start.x) * ratio,
                start.y + (goal.y - start.y) * ratio,
                clearance_m=clearance_m,
                label=label,
            )


def _validate_route_on_costmap(
    route: WaypointRoute,
    *,
    start: Waypoint,
    costmap,
    clearance_m: float = 0.50,
) -> None:
    """Require every patrol leg to have a known-free straight corridor.

    Foxy cannot ask ComputePathToPose to plan from an arbitrary future start,
    so this is intentionally conservative for legs after the current one.  The
    live first leg is additionally checked by ComputePathToPose.
    """
    _validate_waypoint(start)
    grid_frame = str(costmap.header.frame_id).strip("/")
    expected_frame = route.frame_id.strip("/")
    if grid_frame != expected_frame:
        raise ValueError(
            "occupancy grid frame {!r} does not match route frame {!r}".format(
                costmap.header.frame_id,
                route.frame_id,
            )
        )
    view = _CostmapView(costmap)
    leg_start = start
    for waypoint in route.waypoints:
        view.require_corridor(
            leg_start,
            waypoint,
            clearance_m=clearance_m,
            label="corridor {}->{}".format(leg_start.id, waypoint.id),
        )
        leg_start = waypoint


def _validate_computed_path(
    path,
    *,
    frame_id: str,
    start: Waypoint,
    goal: Waypoint,
    leg_index: int,
    costmap,
    max_detour_ratio: float,
) -> None:
    poses = list(path.poses)
    if not poses:
        raise ValueError("compute-path leg {} returned an empty path".format(leg_index))
    path_frame = str(path.header.frame_id).strip("/")
    if path_frame and path_frame != frame_id.strip("/"):
        raise ValueError(
            "compute-path leg {} returned frame {!r}, expected {!r}".format(
                leg_index,
                path.header.frame_id,
                frame_id,
            )
        )

    view = _CostmapView(costmap)
    previous_x = float(poses[0].pose.position.x)
    previous_y = float(poses[0].pose.position.y)
    start_error = math.hypot(previous_x - start.x, previous_y - start.y)
    start_tolerance = max(0.35, view.resolution * 2.0)
    if start_error > start_tolerance:
        raise ValueError(
            "compute-path leg {} starts {:.3f}m from captured pose".format(
                leg_index,
                start_error,
            )
        )
    path_length = start_error
    for index, pose_stamped in enumerate(poses):
        pose_frame = str(pose_stamped.header.frame_id).strip("/")
        if pose_frame and pose_frame != frame_id.strip("/"):
            raise ValueError(
                "compute-path leg {} pose {} has frame {!r}".format(
                    leg_index,
                    index,
                    pose_stamped.header.frame_id,
                )
            )
        x = float(pose_stamped.pose.position.x)
        y = float(pose_stamped.pose.position.y)
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError("compute-path leg {} contains non-finite poses".format(leg_index))
        if index:
            path_length += math.hypot(x - previous_x, y - previous_y)
        previous_x, previous_y = x, y
        # The first samples may overlap the robot footprint in a rolling
        # obstacle layer. The planner has already accepted that start pose.
        if math.hypot(x - start.x, y - start.y) > 0.35:
            view.require_known_free(
                x,
                y,
                label="compute-path leg {}".format(leg_index),
            )

    endpoint_error = math.hypot(previous_x - goal.x, previous_y - goal.y)
    endpoint_tolerance = max(0.15, view.resolution * 2.0)
    if endpoint_error > endpoint_tolerance:
        raise ValueError(
            "compute-path leg {} ends {:.3f}m from goal".format(
                leg_index,
                endpoint_error,
            )
        )
    direct_distance = math.hypot(goal.x - start.x, goal.y - start.y)
    maximum_length = max(direct_distance + 0.5, direct_distance * max_detour_ratio)
    if path_length > maximum_length:
        raise ValueError(
            "compute-path leg {} detour is too long: {:.3f}m > {:.3f}m".format(
                leg_index,
                path_length,
                maximum_length,
            )
        )
