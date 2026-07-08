"""Small mockable web status boundary for IQ9.

The production server can wrap this app with a real HTTP runtime. The core
payload builder is intentionally side-effect free and never sends motion.
"""

from __future__ import annotations

import json
import os
import time
from collections.abc import Callable
from pathlib import Path

from lite3_iq9.runtime_state import RuntimeStateAggregator
from lite3_iq9.camera_source import CameraSourceConfig


class StatusApp:
    def __init__(
        self,
        aggregator: RuntimeStateAggregator,
        *,
        clock: Callable[[], float] | None = None,
        active_route: Callable[[], str | None] | None = None,
        current_waypoint: Callable[[], str | None] | None = None,
        action_state: Callable[[], str] | None = None,
        camera_source: CameraSourceConfig | None = None,
    ) -> None:
        self.aggregator = aggregator
        self.clock = clock or time.monotonic
        self.active_route = active_route or (lambda: None)
        self.current_waypoint = current_waypoint or (lambda: None)
        self.action_state = action_state or (lambda: "idle")
        self.camera_source = camera_source or CameraSourceConfig.from_env(os.environ)

    def handle_get(self, path: str) -> tuple[str, int, dict[str, str]]:
        if path == "/":
            return _index_html(), 200, {"Content-Type": "text/html"}
        if path != "/api/status":
            return "not found", 404, {"Content-Type": "text/plain"}

        payload = build_status_payload(
            self.aggregator,
            now_sec=self.clock(),
            active_route=self.active_route(),
            current_waypoint=self.current_waypoint(),
            action_state=self.action_state(),
            camera_source=self.camera_source,
        )
        return json.dumps(payload, sort_keys=True), 200, {"Content-Type": "application/json"}


def create_app(
    aggregator: RuntimeStateAggregator,
    *,
    clock: Callable[[], float] | None = None,
    active_route: Callable[[], str | None] | None = None,
    current_waypoint: Callable[[], str | None] | None = None,
    action_state: Callable[[], str] | None = None,
    camera_source: CameraSourceConfig | None = None,
) -> StatusApp:
    return StatusApp(
        aggregator,
        clock=clock,
        active_route=active_route,
        current_waypoint=current_waypoint,
        action_state=action_state,
        camera_source=camera_source,
    )


def build_status_payload(
    aggregator: RuntimeStateAggregator,
    *,
    now_sec: float,
    active_route: str | None,
    current_waypoint: str | None,
    action_state: str,
    camera_source: CameraSourceConfig | None = None,
) -> dict[str, object]:
    snapshot = aggregator.snapshot(now_sec)
    nav = dict(snapshot.nav)
    nav.update(
        {
            "active_route": active_route,
            "current_waypoint": current_waypoint,
            "action_state": action_state,
        }
    )
    return {
        "ready_for_waypoint": snapshot.ready_for_waypoint,
        "blocking_reasons": snapshot.blocking_reasons,
        "robot": snapshot.robot,
        "nav": nav,
        "sensors": snapshot.sensors,
        "camera": _camera_payload(camera_source or CameraSourceConfig.from_env(os.environ)),
    }


def _index_html() -> str:
    return (Path(__file__).resolve().parent / "static" / "index.html").read_text(encoding="utf-8")


def _camera_payload(config: CameraSourceConfig) -> dict[str, object]:
    return {
        "source_type": config.source_type,
        "stream_url": config.url,
        "redacted_url": config.redacted_url,
        "yolo_input_enabled": config.yolo_input_enabled,
    }
