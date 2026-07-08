"""IQ9-side waypoint patrol route planning.

This module builds Nav2 waypoint routes only. It does not send ROS2 action
goals or robot motion commands.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any

import yaml

from lite3_iq9.waypoint_route import Waypoint, WaypointRoute


class PatrolModeState(Enum):
    IDLE = "idle"
    ACTIVE = "active"
    RETURNING_HOME = "returning_home"


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
class WaypointPatrolConfig:
    route_id: str
    frame_id: str
    min_distance_m: float
    offsets: list[PatrolOffset]
    segments: list[PatrolSegment]

    @classmethod
    def from_yaml(cls, path: str | Path) -> "WaypointPatrolConfig":
        config_path = Path(path)
        data = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            raise ValueError("patrol config must contain a mapping")
        return cls.from_mapping(data)

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> "WaypointPatrolConfig":
        route_id = _required_str(data, "route_id")
        frame_id = _required_str(data, "frame_id")
        if frame_id != "map":
            raise ValueError("frame_id must be 'map'")
        raw_offsets = data.get("offsets", [])
        raw_segments = data.get("segments", [])
        if not isinstance(raw_offsets, list):
            raise ValueError("offsets must be a list")
        if not isinstance(raw_segments, list):
            raise ValueError("segments must be a list")
        if not raw_offsets and not raw_segments:
            raise ValueError("offsets or segments must be a non-empty list")
        return cls(
            route_id=route_id,
            frame_id=frame_id,
            min_distance_m=_required_float(data, "min_distance_m", "config"),
            offsets=[_parse_offset(item, index) for index, item in enumerate(raw_offsets, start=1)],
            segments=[
                _parse_segment(item, index) for index, item in enumerate(raw_segments, start=1)
            ],
        )


class WaypointPatrolPlanner:
    def __init__(
        self,
        default_offsets: list[PatrolOffset] | None = None,
        *,
        default_segments: list[PatrolSegment] | None = None,
        min_distance_m: float = 3.0,
        route_id: str = "default_patrol",
        frame_id: str = "map",
    ) -> None:
        if default_offsets is None:
            default_offsets = []
        if default_segments is None:
            default_segments = []
        if not default_offsets and not default_segments:
            raise ValueError("default_offsets or default_segments must not be empty")
        self.min_distance_m = float(min_distance_m)
        self.default_offsets = list(default_offsets)
        self.default_segments = list(default_segments)
        self.route_id = route_id
        self.frame_id = frame_id
        self._validate_defaults()
        self._home: Waypoint | None = None
        self._state = PatrolModeState.IDLE

    @property
    def state(self) -> PatrolModeState:
        return self._state

    @property
    def home(self) -> Waypoint | None:
        return self._home

    def start_patrol(
        self,
        current_pose: Waypoint,
        *,
        route: WaypointRoute | None = None,
    ) -> WaypointRoute:
        self._home = _rename_waypoint(current_pose, "home")
        self._state = PatrolModeState.ACTIVE
        if route is not None:
            return route
        return self._build_default_route(self._home)

    def stop_patrol(self, current_pose: Waypoint) -> WaypointRoute:
        if self._home is None:
            raise RuntimeError("cannot return home before a home pose is captured")
        self._state = PatrolModeState.RETURNING_HOME
        return WaypointRoute(
            route_id="return_home",
            frame_id=self.frame_id,
            loop=False,
            waypoints=[
                _rename_waypoint(current_pose, "current"),
                _rename_waypoint(self._home, "home_return"),
            ],
        )

    def mark_idle(self) -> None:
        self._state = PatrolModeState.IDLE

    def _build_default_route(self, home: Waypoint) -> WaypointRoute:
        if self.default_segments:
            patrol_points = _waypoints_from_segments(home, self.default_segments)
        else:
            patrol_points = [
                _waypoint_from_offset(index=index, home=home, offset=offset)
                for index, offset in enumerate(self.default_offsets, start=1)
            ]
        return WaypointRoute(
            route_id=self.route_id,
            frame_id=self.frame_id,
            loop=False,
            waypoints=[home, *patrol_points, _rename_waypoint(home, "home_return")],
        )

    def _validate_defaults(self) -> None:
        for index, offset in enumerate(self.default_offsets, start=1):
            distance = math.hypot(offset.dx, offset.dy)
            if distance < self.min_distance_m:
                raise ValueError(
                    f"default offset #{index} distance {distance:.3f}m is below "
                    f"min_distance_m {self.min_distance_m:.3f}m"
                )
        for index, segment in enumerate(self.default_segments, start=1):
            if segment.distance_m < self.min_distance_m:
                raise ValueError(
                    f"default segment #{index} distance {segment.distance_m:.3f}m is below "
                    f"min_distance_m {self.min_distance_m:.3f}m"
                )


def _waypoint_from_offset(index: int, home: Waypoint, offset: PatrolOffset) -> Waypoint:
    cos_yaw = math.cos(home.yaw)
    sin_yaw = math.sin(home.yaw)
    x = home.x + offset.dx * cos_yaw - offset.dy * sin_yaw
    y = home.y + offset.dx * sin_yaw + offset.dy * cos_yaw
    return Waypoint(
        id=f"p{index}",
        x=x,
        y=y,
        yaw=_normalize_angle(home.yaw + offset.yaw_offset),
        dwell_sec=offset.dwell_sec,
    )


def _waypoints_from_segments(home: Waypoint, segments: list[PatrolSegment]) -> list[Waypoint]:
    x = home.x
    y = home.y
    yaw = home.yaw
    waypoints: list[Waypoint] = []
    for index, segment in enumerate(segments, start=1):
        yaw = _normalize_angle(yaw + segment.turn_rad)
        x += segment.distance_m * math.cos(yaw)
        y += segment.distance_m * math.sin(yaw)
        waypoints.append(
            Waypoint(
                id=f"p{index}",
                x=x,
                y=y,
                yaw=yaw,
                dwell_sec=segment.dwell_sec,
            )
        )
    return waypoints


def _rename_waypoint(waypoint: Waypoint, waypoint_id: str) -> Waypoint:
    return Waypoint(
        id=waypoint_id,
        x=waypoint.x,
        y=waypoint.y,
        yaw=waypoint.yaw,
        dwell_sec=waypoint.dwell_sec,
    )


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _parse_offset(data: Any, index: int) -> PatrolOffset:
    if not isinstance(data, dict):
        raise ValueError(f"offset #{index} must be a mapping")
    return PatrolOffset(
        dx=_required_float(data, "dx", f"offset #{index}"),
        dy=_required_float(data, "dy", f"offset #{index}"),
        yaw_offset=float(data.get("yaw_offset", 0.0) or 0.0),
        dwell_sec=float(data.get("dwell_sec", 0.0) or 0.0),
    )


def _parse_segment(data: Any, index: int) -> PatrolSegment:
    if not isinstance(data, dict):
        raise ValueError(f"segment #{index} must be a mapping")
    if "turn_rad" in data:
        turn_rad = _required_float(data, "turn_rad", f"segment #{index}")
    else:
        turn_rad = math.radians(float(data.get("turn_deg", 0.0) or 0.0))
    return PatrolSegment(
        distance_m=_required_float(data, "distance_m", f"segment #{index}"),
        turn_rad=turn_rad,
        dwell_sec=float(data.get("dwell_sec", 0.0) or 0.0),
    )


def _required_str(data: dict[str, Any], key: str) -> str:
    value = data.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{key} must be a non-empty string")
    return value.strip()


def _required_float(data: dict[str, Any], key: str, owner: str) -> float:
    value = data.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{owner} {key} must be numeric")
    return float(value)
