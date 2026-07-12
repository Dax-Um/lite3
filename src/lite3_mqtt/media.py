"""Detection media interfaces and a replaceable OpenCV mock source."""

from __future__ import annotations

import logging
import io
import subprocess
import tempfile
import uuid
from pathlib import Path
from typing import Callable, Optional, Protocol

from lite3_mqtt.contract import (
    DetectionType,
    build_image_payload,
    build_video_payload,
    image_topic,
    video_topic,
)


class AnnotatedMediaSource(Protocol):
    """Adapter boundary for a future YOLO annotated-frame/clip provider."""

    def capture_image(self, detection_type: DetectionType, event_id: str) -> bytes:
        """Return one annotated JPEG."""

    def capture_video(
        self,
        detection_type: DetectionType,
        event_id: str,
        duration_ms: int,
    ) -> bytes:
        """Return an annotated MP4 clip covering duration_ms."""


PublishJson = Callable[[str, dict], None]


class DetectionMediaPublisher:
    def __init__(
        self,
        *,
        media_source: AnnotatedMediaSource,
        publish_json: PublishJson,
        duration_ms: int = 5000,
        clock_ms: Optional[Callable[[], int]] = None,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        if duration_ms <= 0:
            raise ValueError("duration_ms must be positive")
        self.media_source = media_source
        self.publish_json = publish_json
        self.duration_ms = duration_ms
        self.clock_ms = clock_ms
        self.logger = logger or logging.getLogger(__name__)

    def publish_detection(
        self,
        detection_type: DetectionType,
        *,
        event_id: Optional[str] = None,
    ) -> str:
        event_id = event_id or f"{detection_type.value.lower()}-{uuid.uuid4().hex}"

        try:
            jpeg = self.media_source.capture_image(detection_type, event_id)
        except Exception:
            self.logger.exception("annotated image capture failed event_id=%s", event_id)
            jpeg = None
        image_kwargs = {
            "event_id": event_id,
            "detection_type": detection_type,
            "jpeg_bytes": jpeg,
        }
        if self.clock_ms is not None:
            image_kwargs["clock_ms"] = self.clock_ms
        self.publish_json(image_topic(detection_type), build_image_payload(**image_kwargs))

        try:
            mp4 = self.media_source.capture_video(
                detection_type,
                event_id,
                self.duration_ms,
            )
        except Exception:
            self.logger.exception("annotated video capture failed event_id=%s", event_id)
            mp4 = None
        video_kwargs = {
            "event_id": event_id,
            "detection_type": detection_type,
            "mp4_bytes": mp4,
            "duration_ms": self.duration_ms,
        }
        if self.clock_ms is not None:
            video_kwargs["clock_ms"] = self.clock_ms
        self.publish_json(video_topic(detection_type), build_video_payload(**video_kwargs))
        return event_id


class MockAnnotatedMediaSource:
    """Generate annotated JPEG with Pillow and MP4 with Foxy GStreamer."""

    def __init__(self, *, width: int = 640, height: int = 360, fps: int = 5) -> None:
        if width <= 0 or height <= 0 or fps <= 0:
            raise ValueError("mock media dimensions and fps must be positive")
        self.width = width
        self.height = height
        self.fps = fps

    def capture_image(self, detection_type: DetectionType, event_id: str) -> bytes:
        return self._annotated_jpeg(detection_type, event_id)

    def capture_video(
        self,
        detection_type: DetectionType,
        event_id: str,
        duration_ms: int,
    ) -> bytes:
        frame_count = max(1, round(self.fps * duration_ms / 1000.0))
        temp_root = Path(tempfile.gettempdir())
        token = uuid.uuid4().hex
        jpeg_path = temp_root / "lite3-{}.jpg".format(token)
        mp4_path = temp_root / "lite3-{}.mp4".format(token)
        jpeg_path.write_bytes(self._annotated_jpeg(detection_type, event_id))
        pipeline = [
            "gst-launch-1.0",
            "-q",
            "filesrc",
            "location={}".format(jpeg_path),
            "!",
            "jpegdec",
            "!",
            "imagefreeze",
            "num-buffers={}".format(frame_count),
            "!",
            "videoconvert",
            "!",
            "video/x-raw,framerate={}/1,width={},height={}".format(
                self.fps, self.width, self.height
            ),
            "!",
            "x264enc",
            "tune=zerolatency",
            "speed-preset=ultrafast",
            "key-int-max={}".format(self.fps),
            "!",
            "mp4mux",
            "!",
            "filesink",
            "location={}".format(mp4_path),
        ]
        try:
            completed = subprocess.run(
                pipeline,
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=30.0,
            )
            if completed.returncode != 0:
                raise RuntimeError(
                    "GStreamer mock MP4 failed: {}".format(
                        completed.stderr.decode("utf-8", errors="replace")
                    )
                )
            data = mp4_path.read_bytes()
            if not data:
                raise RuntimeError("mock MP4 is empty")
            return data
        finally:
            jpeg_path.unlink(missing_ok=True)
            mp4_path.unlink(missing_ok=True)

    def _annotated_jpeg(self, detection_type: DetectionType, event_id: str) -> bytes:
        try:
            from PIL import Image, ImageDraw
        except ImportError as exc:
            raise RuntimeError("mock media source requires Pillow") from exc

        frame = Image.new("RGB", (self.width, self.height), color=(30, 30, 30))
        draw = ImageDraw.Draw(frame)
        x1, y1 = self.width // 4, self.height // 4
        x2, y2 = self.width * 3 // 4, self.height * 3 // 4
        draw.rectangle((x1, y1, x2, y2), outline=(255, 220, 0), width=4)
        draw.rectangle((x1, y1 - 24, x2, y1), fill=(255, 220, 0))
        draw.text(
            (x1 + 6, y1 - 20),
            "MOCK {}".format(detection_type.value),
            fill=(10, 10, 10),
        )
        draw.text(
            (20, self.height - 24),
            "event={}".format(event_id[:40]),
            fill=(220, 220, 220),
        )
        output = io.BytesIO()
        frame.save(output, format="JPEG", quality=85)
        return output.getvalue()
