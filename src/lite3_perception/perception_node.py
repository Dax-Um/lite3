"""Perception node: image in → detections out.

Designed to sit *downstream* of the UDP camera node:

  UdpCameraNode --(topic)--> PerceptionNode --(topic)--> consumers

Frame intake is either:
  - ``process_frame()`` / ``process_jpeg()`` called by a ROS subscriber, or
  - optional direct ``on_frame`` wiring for non-ROS tests.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Protocol

from lite3_perception.udp_camera_receiver import CameraFrame


@dataclass(frozen=True)
class Detection:
    label: str
    score: float
    bbox_xyxy: tuple[float, float, float, float] | None = None
    attrs: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class PerceptionResult:
    frame_timestamp_monotonic: float
    processed_at_monotonic: float
    frame_sequence: int
    frame_width: int | None
    frame_height: int | None
    detections: list[Detection] = field(default_factory=list)
    status: str = "ok"
    detail: str = ""

    def to_json(self) -> str:
        return json.dumps(
            {
                "frame_timestamp_monotonic": self.frame_timestamp_monotonic,
                "processed_at_monotonic": self.processed_at_monotonic,
                "frame_sequence": self.frame_sequence,
                "frame_width": self.frame_width,
                "frame_height": self.frame_height,
                "status": self.status,
                "detail": self.detail,
                "detections": [
                    {
                        "label": d.label,
                        "score": d.score,
                        "bbox_xyxy": d.bbox_xyxy,
                        "attrs": d.attrs,
                    }
                    for d in self.detections
                ],
            },
            separators=(",", ":"),
        )


class FrameDetector(Protocol):
    def process(self, frame: CameraFrame) -> list[Detection]:
        ...


class PassthroughDetector:
    """Placeholder: accept frames, emit no detections yet."""

    def process(self, frame: CameraFrame) -> list[Detection]:
        _ = frame
        return []


ResultCallback = Callable[[PerceptionResult], None]


@dataclass(frozen=True)
class PerceptionNodeConfig:
    # 0 = every frame; otherwise skip work to target rate.
    target_fps: float = 0.0


class PerceptionNode:
    """Stateless-ish processor: frame → result (+ optional callback)."""

    def __init__(
        self,
        config: PerceptionNodeConfig | None = None,
        *,
        detector: FrameDetector | None = None,
        on_result: ResultCallback | None = None,
    ):
        self.config = config or PerceptionNodeConfig()
        if self.config.target_fps < 0.0:
            raise ValueError("target_fps must be zero or positive")
        self.detector: FrameDetector = detector or PassthroughDetector()
        self.on_result = on_result
        self._frames_processed = 0
        self._last_process_monotonic: float | None = None
        self._latest_result: PerceptionResult | None = None
        self._min_interval = (
            0.0 if self.config.target_fps <= 0 else 1.0 / self.config.target_fps
        )

    @property
    def frames_processed(self) -> int:
        return self._frames_processed

    def get_latest_result(self) -> PerceptionResult | None:
        return self._latest_result

    def should_process_now(self, now: float | None = None) -> bool:
        if self._min_interval <= 0:
            return True
        now = time.monotonic() if now is None else now
        if self._last_process_monotonic is None:
            return True
        return (now - self._last_process_monotonic) >= self._min_interval

    def process_jpeg(
        self,
        jpeg_bytes: bytes,
        *,
        sequence: int = 0,
        width: int | None = None,
        height: int | None = None,
        frame_timestamp_monotonic: float | None = None,
    ) -> PerceptionResult | None:
        """Entry point used by the ROS image subscriber."""
        now = time.monotonic()
        if not self.should_process_now(now):
            return None
        frame = CameraFrame(
            jpeg_bytes=jpeg_bytes,
            timestamp_monotonic=(
                now
                if frame_timestamp_monotonic is None
                else frame_timestamp_monotonic
            ),
            width=width,
            height=height,
            sequence=sequence,
            source="topic",
        )
        return self.process_frame(frame)

    def process_frame(self, frame: CameraFrame) -> PerceptionResult:
        now = time.monotonic()
        try:
            detections = list(self.detector.process(frame))
            status = "ok"
            detail = ""
        except Exception as exc:  # noqa: BLE001 - keep node alive
            detections = []
            status = "error"
            detail = str(exc)

        result = PerceptionResult(
            frame_timestamp_monotonic=frame.timestamp_monotonic,
            processed_at_monotonic=now,
            frame_sequence=frame.sequence,
            frame_width=frame.width,
            frame_height=frame.height,
            detections=detections,
            status=status,
            detail=detail,
        )
        self._latest_result = result
        self._frames_processed += 1
        self._last_process_monotonic = now
        if self.on_result is not None:
            try:
                self.on_result(result)
            except Exception:
                pass
        return result

    def health(self) -> dict[str, Any]:
        return {
            "node": "perception",
            "frames_processed": self._frames_processed,
            "has_result": self._latest_result is not None,
            "last_status": None
            if self._latest_result is None
            else self._latest_result.status,
            "last_detection_count": 0
            if self._latest_result is None
            else len(self._latest_result.detections),
        }
