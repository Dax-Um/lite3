"""Waypoint route loading and validation for IQ9 Nav2 clients."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


class RouteValidationError(ValueError):
    """Raised when a waypoint route file is invalid."""


@dataclass(frozen=True)
class Waypoint:
    id: str
    x: float
    y: float
    yaw: float
    dwell_sec: float = 0.0


@dataclass(frozen=True)
class Geofence:
    min_x: float
    max_x: float
    min_y: float
    max_y: float

    def contains(self, waypoint: Waypoint) -> bool:
        return (
            self.min_x <= waypoint.x <= self.max_x
            and self.min_y <= waypoint.y <= self.max_y
        )


@dataclass(frozen=True)
class WaypointRoute:
    route_id: str
    frame_id: str
    loop: bool
    waypoints: list[Waypoint]
    geofence: Geofence | None = None

    @classmethod
    def from_yaml(cls, path: str | Path) -> "WaypointRoute":
        route_path = Path(path)
        data = yaml.safe_load(route_path.read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            raise RouteValidationError("route yaml must contain a mapping")
        return cls.from_mapping(data)

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> "WaypointRoute":
        route_id = _required_str(data, "route_id")
        frame_id = _required_str(data, "frame_id")
        if frame_id != "map":
            raise RouteValidationError("frame_id must be 'map'")

        raw_waypoints = data.get("waypoints")
        if not isinstance(raw_waypoints, list) or not raw_waypoints:
            raise RouteValidationError("waypoints must be a non-empty list")

        geofence = _parse_geofence(data.get("geofence"))
        waypoints = [_parse_waypoint(item, index) for index, item in enumerate(raw_waypoints)]
        if geofence is not None:
            _validate_geofence(waypoints, geofence)
        return cls(
            route_id=route_id,
            frame_id=frame_id,
            loop=bool(data.get("loop", False)),
            waypoints=waypoints,
            geofence=geofence,
        )


def _required_str(data: dict[str, Any], key: str) -> str:
    value = data.get(key)
    if not isinstance(value, str) or not value.strip():
        raise RouteValidationError(f"{key} must be a non-empty string")
    return value.strip()


def _required_float(data: dict[str, Any], key: str, waypoint_id: str) -> float:
    value = data.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RouteValidationError(f"waypoint {waypoint_id} {key} must be numeric")
    return float(value)


def _parse_waypoint(data: Any, index: int) -> Waypoint:
    if not isinstance(data, dict):
        raise RouteValidationError(f"waypoint #{index} must be a mapping")
    waypoint_id = _required_str(data, "id")
    return Waypoint(
        id=waypoint_id,
        x=_required_float(data, "x", waypoint_id),
        y=_required_float(data, "y", waypoint_id),
        yaw=_required_float(data, "yaw", waypoint_id),
        dwell_sec=float(data.get("dwell_sec", 0.0) or 0.0),
    )


def _parse_geofence(data: Any) -> Geofence | None:
    if data is None:
        return None
    if not isinstance(data, dict):
        raise RouteValidationError("geofence must be a mapping")
    geofence = Geofence(
        min_x=_required_float(data, "min_x", "geofence"),
        max_x=_required_float(data, "max_x", "geofence"),
        min_y=_required_float(data, "min_y", "geofence"),
        max_y=_required_float(data, "max_y", "geofence"),
    )
    if geofence.min_x > geofence.max_x or geofence.min_y > geofence.max_y:
        raise RouteValidationError("geofence min bounds must be <= max bounds")
    return geofence


def _validate_geofence(waypoints: list[Waypoint], geofence: Geofence) -> None:
    for waypoint in waypoints:
        if not geofence.contains(waypoint):
            raise RouteValidationError(f"waypoint {waypoint.id} is outside geofence")
