"""Unit tests for PerceptionNode (topic-side processor)."""

from __future__ import annotations

import time

from lite3_perception.perception_node import (
    Detection,
    PassthroughDetector,
    PerceptionNode,
    PerceptionNodeConfig,
)
from lite3_perception.udp_camera_receiver import CameraFrame


class FakeDetector:
    def process(self, frame: CameraFrame) -> list[Detection]:
        return [Detection(label="person", score=0.9, bbox_xyxy=(0.1, 0.1, 0.5, 0.8))]


def _frame(seq: int = 1) -> CameraFrame:
    return CameraFrame(
        jpeg_bytes=b"\xff\xd8\xff\xd9",
        timestamp_monotonic=time.monotonic(),
        width=1280,
        height=720,
        sequence=seq,
    )


def test_process_frame_passthrough():
    node = PerceptionNode(detector=PassthroughDetector())
    result = node.process_frame(_frame())
    assert result.status == "ok"
    assert result.detections == []
    assert node.frames_processed == 1


def test_process_jpeg_entry():
    node = PerceptionNode(detector=FakeDetector())
    result = node.process_jpeg(b"\xff\xd8\xff\xd9", sequence=7, width=640, height=480)
    assert result is not None
    assert result.frame_sequence == 7
    assert result.detections[0].label == "person"
    assert '"label":"person"' in result.to_json()


def test_target_fps_skips():
    node = PerceptionNode(PerceptionNodeConfig(target_fps=1.0), detector=PassthroughDetector())
    assert node.process_jpeg(b"\xff\xd8\xff\xd9", sequence=1) is not None
    # Immediate second call should be rate-limited
    assert node.process_jpeg(b"\xff\xd8\xff\xd9", sequence=2) is None


def test_detector_error_status():
    class Boom:
        def process(self, frame: CameraFrame):
            raise RuntimeError("boom")

    node = PerceptionNode(detector=Boom())
    result = node.process_frame(_frame())
    assert result.status == "error"
    assert "boom" in result.detail
