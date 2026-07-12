"""Camera node: UDP in → continuous frames out (callback / latest-frame).

This is the ROS-independent half of the split architecture:

  UdpCameraNode  --frame-->  (ROS topic)  --frame-->  PerceptionNode
"""

from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable

from lite3_perception.udp_camera_receiver import (
    CameraFrame,
    ReceiverStats,
    UdpCameraConfig,
    UdpJpegCameraReceiver,
)


FrameCallback = Callable[[CameraFrame], None]
StatsCallback = Callable[[dict[str, Any]], None]


@dataclass(frozen=True)
class CameraNodeConfig:
    udp: UdpCameraConfig
    status_period_sec: float = 1.0
    stale_frame_sec: float = 2.0


class UdpCameraNode:
    """Continuously receive UDP camera frames and fan them out."""

    def __init__(
        self,
        config: CameraNodeConfig | None = None,
        *,
        receiver: UdpJpegCameraReceiver | None = None,
        on_frame: FrameCallback | None = None,
        on_status: StatsCallback | None = None,
    ):
        self.config = config or CameraNodeConfig(udp=UdpCameraConfig())
        self.on_frame = on_frame
        self.on_status = on_status
        self.receiver = receiver or UdpJpegCameraReceiver(
            self.config.udp, on_frame=self._handle_frame
        )
        # If caller provided a receiver without wiring, still hook it.
        if receiver is not None and receiver.on_frame is None:
            receiver.on_frame = self._handle_frame

        self._stop = threading.Event()
        self._status_thread: threading.Thread | None = None
        self._frames_forwarded = 0
        self._lock = threading.Lock()
        self._latest: CameraFrame | None = None

    @property
    def is_running(self) -> bool:
        return self.receiver.is_running

    @property
    def frames_forwarded(self) -> int:
        return self._frames_forwarded

    def get_latest_frame(self) -> CameraFrame | None:
        with self._lock:
            return self._latest

    def start(self) -> None:
        self.receiver.start()
        if self.on_status is not None and (
            self._status_thread is None or not self._status_thread.is_alive()
        ):
            self._stop.clear()
            self._status_thread = threading.Thread(
                target=self._status_loop, name="udp-camera-status", daemon=True
            )
            self._status_thread.start()

    def stop(self, join_timeout: float = 2.0) -> None:
        self._stop.set()
        if self._status_thread is not None:
            self._status_thread.join(timeout=join_timeout)
            self._status_thread = None
        self.receiver.stop(join_timeout=join_timeout)

    def _handle_frame(self, frame: CameraFrame) -> None:
        with self._lock:
            self._latest = frame
            self._frames_forwarded += 1
        if self.on_frame is not None:
            self.on_frame(frame)

    def _status_loop(self) -> None:
        period = max(0.2, self.config.status_period_sec)
        while not self._stop.wait(period):
            if self.on_status is None:
                continue
            try:
                self.on_status(self.health())
            except Exception:
                pass

    def health(self) -> dict[str, Any]:
        stats: ReceiverStats = self.receiver.stats
        now = time.monotonic()
        age = (
            None
            if stats.last_frame_monotonic is None
            else now - stats.last_frame_monotonic
        )
        return {
            "node": "udp_camera",
            "running": self.is_running,
            "backend": stats.backend,
            "open_ok": stats.open_ok,
            "frames_forwarded": self._frames_forwarded,
            "frames_completed": stats.frames_completed,
            "bytes_received": stats.bytes_received,
            "last_frame_age_sec": age,
            "stream_ok": age is not None and age <= self.config.stale_frame_sec,
            "bind": f"{self.config.udp.bind_host}:{self.config.udp.port}",
            "last_error": stats.last_error,
        }

    def health_json(self) -> str:
        return json.dumps(self.health(), separators=(",", ":"))

    def __enter__(self) -> "UdpCameraNode":
        self.start()
        return self

    def __exit__(self, *args) -> None:
        self.stop()
