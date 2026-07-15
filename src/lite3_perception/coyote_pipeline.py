"""Small storage adapter for the existing IQ9 coyote state machine API."""

from __future__ import annotations

import math
import threading
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

import cv2
import numpy as np

try:
    from lite3_perception.coyote_spool import CoyoteMediaSpool, CoyoteSpoolConfig
except ImportError:  # IQ9 native deployment fallback next to this file
    from coyote_spool import CoyoteMediaSpool, CoyoteSpoolConfig  # type: ignore


LogFn = Callable[[dict], None]


@dataclass
class CoyoteTopicConfig:
    status_topic: str = "/lite3/data/coyote/status"
    image_topic: str = "/lite3/data/coyote/image"
    video_topic: str = "/lite3/data/coyote/video"
    status_every_n: int = 10
    hit_confirm: int = 3
    miss_lost: int = 8
    near_enter: float = 0.65
    near_exit: float = 0.50
    # Retained for the current IQ9 main() call. The external contract fixes
    # this band at 40%..60%, so any other value is rejected fail-closed.
    center_half_ratio: float = 0.10
    video_seconds: float = 5.0
    media_cooldown_sec: float = 30.0
    video_max_fps: float = 10.0
    video_jpeg_quality: int = 70
    image_jpeg_quality: int = 85
    file_sink_dir: str = "/home/ubuntu/iq9_coyote/outputs/spool"
    publish_enabled: bool = True
    # Retained constructor fields keep the current IQ9 main() call compatible.
    # MQTT remains disabled there; the Foxy bridge owns the MQTT clients.
    mqtt_host: str = ""
    mqtt_port: int = 1883
    mqtt_username: str = ""
    mqtt_password: str = ""
    mqtt_client_id: str = "iq9_perception_node"
    mqtt_qos: int = 0


@dataclass
class CoyoteController:
    """Drop-in controller used by the existing QNN inference loop."""

    cfg: CoyoteTopicConfig
    log: LogFn
    detect: str = "not_detected"
    motion: str = "stop"
    side: str = "none"
    hit_streak: int = 0
    miss_streak: int = 0
    near_mode: bool = False
    episode_id: int = 0
    event_id: str = ""
    episode_active: bool = False
    last_media_ts: float = 0.0
    area_ratio: float = 0.0
    height_ratio: float = 0.0
    score: float = 0.0
    frame_id: int = 0
    collecting_video: bool = False
    video_start_ts: float = 0.0
    video_event_id: str = ""
    video_capture_started_at_ms: int = 0
    video_frames: List[np.ndarray] = field(default_factory=list)
    last_video_sample_ts: float = 0.0
    _encode_lock: threading.Lock = field(default_factory=threading.Lock)
    _encode_busy: bool = False
    _encode_thread: Optional[threading.Thread] = None

    def __post_init__(self) -> None:
        if not math.isfinite(self.cfg.video_seconds) or self.cfg.video_seconds < 5.0:
            raise ValueError("coyote video_seconds must be at least 5.0")
        if not (
            math.isfinite(self.cfg.near_exit)
            and math.isfinite(self.cfg.near_enter)
            and 0.0 <= self.cfg.near_exit < self.cfg.near_enter <= 1.0
        ):
            raise ValueError("coyote near thresholds must satisfy 0 <= exit < enter <= 1")
        if not math.isclose(
            self.cfg.center_half_ratio,
            0.10,
            rel_tol=0.0,
            abs_tol=1e-9,
        ):
            raise ValueError("coyote center band must remain fixed at 40%..60%")
        if self.cfg.hit_confirm <= 0 or self.cfg.miss_lost <= 0:
            raise ValueError("coyote hit/miss thresholds must be positive")
        if self.cfg.status_every_n <= 0:
            raise ValueError("coyote status_every_n must be positive")
        if (
            not math.isfinite(self.cfg.video_max_fps)
            or self.cfg.video_max_fps <= 0.0
        ):
            raise ValueError("coyote video_max_fps must be finite and positive")
        if (
            not math.isfinite(self.cfg.media_cooldown_sec)
            or self.cfg.media_cooldown_sec < 0.0
        ):
            raise ValueError("coyote media cooldown must be finite and non-negative")
        if not 1 <= self.cfg.image_jpeg_quality <= 100:
            raise ValueError("coyote image JPEG quality must be in [1, 100]")
        if not self.cfg.file_sink_dir and self.cfg.publish_enabled:
            raise ValueError("coyote bridge requires file_sink_dir")
        self._spool = None  # type: Optional[CoyoteMediaSpool]
        if self.cfg.publish_enabled:
            self._spool = CoyoteMediaSpool(
                CoyoteSpoolConfig(Path(self.cfg.file_sink_dir))
            )

    def close(self) -> None:
        if self.collecting_video:
            capture_age = time.time() - self.video_start_ts
            if capture_age >= self.cfg.video_seconds:
                self._finish_video_async()
            else:
                self.collecting_video = False
                self.video_frames = []
                self.log(
                    {
                        "type": "video_incomplete_drop",
                        "ts": time.time(),
                        "event_id": self.video_event_id,
                        "capture_sec": round(capture_age, 3),
                    }
                )
                self._write_media_failure(
                    "video",
                    "capture stopped before configured clip duration",
                    event_id=self.video_event_id,
                )
        if self._encode_thread is not None:
            self._encode_thread.join(timeout=15.0)

    def update(
        self,
        frame_id: int,
        bgr: np.ndarray,
        annotated: np.ndarray,
        best_bbox_xyxy: Optional[List[float]],
        best_score: float,
    ) -> Dict[str, Any]:
        self.frame_id = frame_id
        height, width = bgr.shape[:2]
        frame_area = float(max(width * height, 1))
        observed_side = "none"
        if best_bbox_xyxy is not None:
            try:
                bbox = tuple(float(value) for value in best_bbox_xyxy)
                score = float(best_score)
                valid_detection = (
                    len(bbox) == 4
                    and all(math.isfinite(value) for value in bbox)
                    and not isinstance(best_score, bool)
                    and math.isfinite(score)
                    and 0.0 < score <= 1.0
                    and 0.0 <= bbox[0] < bbox[2] <= float(width)
                    and 0.0 <= bbox[1] < bbox[3] <= float(height)
                )
            except (TypeError, ValueError):
                valid_detection = False
            if not valid_detection:
                self.log(
                    {
                        "type": "invalid_detection_drop",
                        "ts": time.time(),
                        "frame_id": frame_id,
                    }
                )
                best_bbox_xyxy = None
                best_score = 0.0
            else:
                best_bbox_xyxy = list(bbox)
                best_score = score
        if best_bbox_xyxy is not None:
            x1, y1, x2, y2 = best_bbox_xyxy
            center_ratio = ((float(x1) + float(x2)) * 0.5) / float(max(width, 1))
            if center_ratio < 0.4:
                observed_side = "left"
            elif center_ratio > 0.6:
                observed_side = "right"
            else:
                observed_side = "center"
            self.area_ratio = (
                max(0.0, x2 - x1) * max(0.0, y2 - y1) / frame_area
            )
            self.height_ratio = max(0.0, y2 - y1) / float(max(height, 1))
            self.score = float(best_score)
            self.hit_streak += 1
            self.miss_streak = 0
        else:
            self.hit_streak = 0
            self.miss_streak += 1

        previous_detect = self.detect
        if self.detect == "not_detected" and self.hit_streak >= self.cfg.hit_confirm:
            self.detect = "detected"
        elif self.detect == "detected" and self.miss_streak >= self.cfg.miss_lost:
            self.detect = "not_detected"
            self.near_mode = False
            self.motion = "stop"
            self._end_episode()

        if self.detect == "detected":
            if best_bbox_xyxy is not None:
                self.side = observed_side
                if not self.near_mode and self.height_ratio >= self.cfg.near_enter:
                    self.near_mode = True
                elif self.near_mode and self.height_ratio <= self.cfg.near_exit:
                    self.near_mode = False
            else:
                self.side = "none"
            self.motion = (
                "forward"
                if self.side == "center" and not self.near_mode
                else "stop"
            )
        else:
            self.motion = "stop"
            self.side = "none"
            self.near_mode = False

        if previous_detect == "not_detected" and self.detect == "detected":
            self._start_episode(annotated)
        if self.collecting_video:
            self._maybe_sample_video(annotated)
            if time.time() - self.video_start_ts >= self.cfg.video_seconds:
                self._finish_video_async()

        status = self._status_payload()
        if (
            self._spool is not None
            and frame_id % max(self.cfg.status_every_n, 1) == 0
        ):
            try:
                self._spool.write_status(status)
                if status["detect"] == "detected":
                    self.log(
                        {
                            "type": "status_publish",
                            "ts": time.time(),
                            "topic": self.cfg.status_topic,
                            "payload": status,
                        }
                    )
            except Exception as exc:
                self.log(
                    {
                        "type": "status_spool_error",
                        "ts": time.time(),
                        "error": str(exc),
                    }
                )
        return status

    def _status_payload(self) -> Dict[str, Any]:
        # Detection hysteresis remains internal for media episode stability.
        # The four-field control contract describes only the current frame, so
        # a frame without a bbox must not emit detected+side:none.
        public_detect = self.detect if self.side != "none" else "not_detected"
        return {
            "ts": time.time(),
            "detect": public_detect,
            "motion": self.motion if public_detect == "detected" else "stop",
            "side": self.side if public_detect == "detected" else "none",
        }

    def _episode_tag(self) -> str:
        return "e-{:04d}".format(self.episode_id)

    def _start_episode(self, annotated: np.ndarray) -> None:
        self.episode_id += 1
        if self._spool is not None:
            self.event_id = self._spool.new_event_id()
        else:
            self.event_id = "coyote-{}-{}".format(
                int(time.time() * 1000), uuid.uuid4().hex[:8]
            )
        self.episode_active = True
        self.log(
            {
                "type": "episode_start",
                "ts": time.time(),
                "episode_id": self._episode_tag(),
                "event_id": self.event_id,
                "score": round(self.score, 4),
                "area_ratio": round(self.area_ratio, 4),
                "height_ratio": round(self.height_ratio, 4),
            }
        )
        now = time.time()
        if now - self.last_media_ts < self.cfg.media_cooldown_sec:
            return
        if self._spool is None:
            return
        if (
            self.collecting_video
            and now - self.video_start_ts >= self.cfg.video_seconds
        ):
            self._finish_video_async()
        previous_capture_active = self.collecting_video
        try:
            ok, buffer = cv2.imencode(
                ".jpg",
                annotated,
                [int(cv2.IMWRITE_JPEG_QUALITY), int(self.cfg.image_jpeg_quality)],
            )
            if not ok:
                raise RuntimeError("JPEG encoder returned false")
            manifest = self._spool.write_image(self.event_id, buffer.tobytes())
            self.log(
                {
                    "type": "spool_image",
                    "ts": time.time(),
                    "event_id": self.event_id,
                    "path": manifest["path"],
                    "bytes": manifest["bytes"],
                }
            )
        except Exception as exc:
            self.log(
                {
                    "type": "image_spool_error",
                    "ts": time.time(),
                    "event_id": self.event_id,
                    "error": str(exc),
                }
            )
            self._write_media_failure("image", str(exc))
        self.last_media_ts = now
        if previous_capture_active:
            self.log(
                {
                    "type": "video_capture_busy",
                    "ts": time.time(),
                    "event_id": self.event_id,
                    "active_event_id": self.video_event_id,
                }
            )
            self._write_media_failure(
                "video",
                "previous five-second clip is still recording",
                event_id=self.event_id,
            )
            return
        self.collecting_video = True
        self.video_start_ts = now
        self.video_capture_started_at_ms = int(now * 1000)
        self.video_event_id = self.event_id
        self.video_frames = [self._resize_video_frame(annotated)]
        self.last_video_sample_ts = now

    def _end_episode(self) -> None:
        if not self.episode_active:
            return
        self.log(
            {
                "type": "episode_end",
                "ts": time.time(),
                "episode_id": self._episode_tag(),
                "event_id": self.event_id,
            }
        )
        self.episode_active = False

    def _maybe_sample_video(self, annotated: np.ndarray) -> None:
        now = time.time()
        if now - self.last_video_sample_ts < 1.0 / max(self.cfg.video_max_fps, 1.0):
            return
        self.last_video_sample_ts = now
        self.video_frames.append(self._resize_video_frame(annotated))

    @staticmethod
    def _resize_video_frame(frame: np.ndarray) -> np.ndarray:
        height, width = frame.shape[:2]
        if width <= 640:
            return frame.copy()
        scale = 640.0 / float(width)
        return cv2.resize(
            frame,
            (640, int(height * scale)),
            interpolation=cv2.INTER_AREA,
        )

    def _finish_video_async(self) -> None:
        if not self.collecting_video:
            return
        self.collecting_video = False
        frames = self.video_frames
        self.video_frames = []
        if not frames:
            self._write_media_failure(
                "video",
                "no video frames were captured",
                event_id=self.video_event_id,
            )
            return
        with self._encode_lock:
            if self._encode_busy:
                self.log(
                    {
                        "type": "video_encode_busy_drop",
                        "ts": time.time(),
                        "event_id": self.video_event_id,
                    }
                )
                self._write_media_failure(
                    "video",
                    "video encoder is busy",
                    event_id=self.video_event_id,
                )
                return
            self._encode_busy = True
        event_id = self.video_event_id
        capture_started_at_ms = self.video_capture_started_at_ms
        capture_ended_at_ms = int(time.time() * 1000)

        def worker() -> None:
            try:
                fps = min(
                    self.cfg.video_max_fps,
                    max(len(frames) / self.cfg.video_seconds, 1.0),
                )
                data = encode_mp4(frames, fps)
                if data is None or self._spool is None:
                    raise RuntimeError("MP4 encoding failed")
                manifest = self._spool.write_video(
                    event_id,
                    data,
                    duration_ms=int(round(self.cfg.video_seconds * 1000.0)),
                    capture_started_at_ms=capture_started_at_ms,
                    capture_ended_at_ms=capture_ended_at_ms,
                )
                self.log(
                    {
                        "type": "spool_video",
                        "ts": time.time(),
                        "event_id": event_id,
                        "path": manifest["path"],
                        "bytes": manifest["bytes"],
                        "n_frames": len(frames),
                    }
                )
            except Exception as exc:
                self.log(
                    {
                        "type": "video_spool_error",
                        "ts": time.time(),
                        "event_id": event_id,
                        "error": str(exc),
                    }
                )
                self._write_media_failure("video", str(exc), event_id=event_id)
            finally:
                with self._encode_lock:
                    self._encode_busy = False

        self._encode_thread = threading.Thread(
            target=worker,
            name="coyote-video-spool",
            daemon=True,
        )
        self._encode_thread.start()

    def _write_media_failure(
        self,
        kind: str,
        reason: str,
        *,
        event_id: Optional[str] = None,
    ) -> None:
        if self._spool is None:
            return
        try:
            self._spool.write_failure(
                event_id or self.event_id,
                kind,
                reason=reason,
                duration_ms=(
                    int(round(self.cfg.video_seconds * 1000.0))
                    if kind == "video"
                    else None
                ),
            )
        except Exception as exc:
            self.log(
                {
                    "type": "media_failure_spool_error",
                    "ts": time.time(),
                    "event_id": event_id or self.event_id,
                    "kind": kind,
                    "error": str(exc),
                }
            )


def encode_mp4(frames: List[np.ndarray], fps: float) -> Optional[bytes]:
    if not frames:
        return None
    height, width = frames[0].shape[:2]
    temp_dir = Path("/tmp/coyote_media")
    temp_dir.mkdir(parents=True, exist_ok=True)
    path = temp_dir / "clip_{}_{}.mp4".format(
        int(time.time() * 1000), uuid.uuid4().hex[:8]
    )
    writer = cv2.VideoWriter(
        str(path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        max(float(fps), 1.0),
        (width, height),
    )
    if not writer.isOpened():
        return None
    try:
        for frame in frames:
            if frame.shape[:2] != (height, width):
                frame = cv2.resize(frame, (width, height))
            writer.write(frame)
    finally:
        writer.release()
    try:
        data = path.read_bytes()
        if len(data) < 12 or data[4:8] != b"ftyp":
            return None
        return data
    finally:
        try:
            path.unlink()
        except FileNotFoundError:
            pass
