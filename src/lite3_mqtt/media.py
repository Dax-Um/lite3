"""Detection media interfaces and a replaceable OpenCV mock source."""

from __future__ import annotations

import logging
import io
import uuid
from typing import Callable, Optional, Protocol

from lite3_mqtt.contract import (
    DetectionType,
    build_image_payload,
    image_topic,
)


class AnnotatedMediaSource(Protocol):
    """Adapter boundary for a future YOLO annotated-frame/clip provider."""

    def capture_image(self, detection_type: DetectionType, event_id: str) -> bytes:
        """Return one annotated JPEG."""

PublishJson = Callable[[str, dict], None]


class DetectionMediaPublisher:
    def __init__(
        self,
        *,
        media_source: Optional[AnnotatedMediaSource],
        publish_json: PublishJson,
        clock_ms: Optional[Callable[[], int]] = None,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        self.media_source = media_source
        self.publish_json = publish_json
        self.clock_ms = clock_ms
        self.logger = logger or logging.getLogger(__name__)

    def publish_detection(
        self,
        detection_type: DetectionType,
        *,
        event_id: Optional[str] = None,
    ) -> str:
        if self.media_source is None:
            raise RuntimeError("publish_detection requires an annotated media source")
        event_id = event_id or f"{detection_type.value.lower()}-{uuid.uuid4().hex}"

        try:
            jpeg = self.media_source.capture_image(detection_type, event_id)
        except Exception:
            self.logger.exception("annotated image capture failed event_id=%s", event_id)
            jpeg = None
        self.publish_image(detection_type, event_id=event_id, jpeg_bytes=jpeg)

        return event_id

    def publish_image(
        self,
        detection_type: DetectionType,
        *,
        event_id: str,
        jpeg_bytes: Optional[bytes],
    ) -> None:
        """Publish one already-produced image without invoking the mock source."""
        image_kwargs = {
            "event_id": event_id,
            "detection_type": detection_type,
            "jpeg_bytes": jpeg_bytes,
        }
        if self.clock_ms is not None:
            image_kwargs["clock_ms"] = self.clock_ms
        self.publish_json(image_topic(detection_type), build_image_payload(**image_kwargs))

class MockAnnotatedMediaSource:
    """Generate an annotated JPEG for bridge smoke tests."""

    def __init__(self, *, width: int = 640, height: int = 360, fps: int = 5) -> None:
        if width <= 0 or height <= 0 or fps <= 0:
            raise ValueError("mock media dimensions and fps must be positive")
        self.width = width
        self.height = height
        self.fps = fps

    def capture_image(self, detection_type: DetectionType, event_id: str) -> bytes:
        return self._annotated_jpeg(detection_type, event_id)

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
