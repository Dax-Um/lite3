import json
from pathlib import Path

import pytest

from lite3_mqtt.client import MqttConfig
from lite3_perception.coyote_spool import CoyoteMediaSpool, CoyoteSpoolConfig
from lite3_perception.coyote_spool import (
    INTERNAL_IMAGE_TOPIC,
    INTERNAL_STATUS_TOPIC,
    INTERNAL_VIDEO_TOPIC,
)


JPEG = b"\xff\xd8annotated\xff\xd9"
MP4 = b"\x00\x00\x00\x18ftypmp42"


def test_spool_saves_event_scoped_image_and_video_atomically(tmp_path):
    spool = CoyoteMediaSpool(
        CoyoteSpoolConfig(tmp_path),
        clock_ms=lambda: 1783652400000,
    )
    event_id = spool.new_event_id()
    status_path = spool.write_status(
        {
            "ts": 1783652400.0,
            "frame_id": 10,
            "detect": "detected",
            "motion": "forward",
            "intent": "advisory",
            "episode_id": "e-0001",
            "event_id": event_id,
        }
    )
    image = spool.write_image(event_id, JPEG)
    video = spool.write_video(event_id, MP4, duration_ms=5000)

    event_dir = tmp_path / "events" / event_id
    assert json.loads(status_path.read_text())["event_id"] == event_id
    assert (event_dir / "image.jpg").read_bytes() == JPEG
    assert (event_dir / "video.mp4").read_bytes() == MP4
    assert json.loads((event_dir / "image.ready.json").read_text()) == image
    assert json.loads((event_dir / "video.ready.json").read_text()) == video
    assert not list(tmp_path.rglob("*.part-*"))
    assert image["event_id"] == video["event_id"]
    assert video["duration_ms"] == 5000


def test_event_ids_are_unique_even_when_timestamp_is_equal(tmp_path):
    spool = CoyoteMediaSpool(CoyoteSpoolConfig(tmp_path), clock_ms=lambda: 1)

    assert spool.new_event_id() != spool.new_event_id()


@pytest.mark.parametrize("event_id", ("../escape", "with/slash", "", "한글"))
def test_spool_rejects_unsafe_event_id(tmp_path, event_id):
    spool = CoyoteMediaSpool(CoyoteSpoolConfig(tmp_path))

    with pytest.raises(ValueError, match="event_id"):
        spool.write_image(event_id, JPEG)


def test_spool_rejects_invalid_media_without_ready_manifest(tmp_path):
    spool = CoyoteMediaSpool(CoyoteSpoolConfig(tmp_path))
    event_id = spool.new_event_id()

    with pytest.raises(ValueError, match="JPEG"):
        spool.write_image(event_id, b"not-jpeg")
    with pytest.raises(ValueError, match="MP4"):
        spool.write_video(event_id, b"not-mp4", duration_ms=5000)

    assert not list(Path(tmp_path).rglob("*.ready.json"))


def test_existing_coyote_names_are_reused_as_internal_ros_topics():
    assert INTERNAL_STATUS_TOPIC == "/lite3/data/coyote/status"
    assert INTERNAL_IMAGE_TOPIC == "/lite3/data/coyote/image"
    assert INTERNAL_VIDEO_TOPIC == "/lite3/data/coyote/video"


def test_same_event_kind_cannot_be_requeued_after_terminal_state(tmp_path):
    spool = CoyoteMediaSpool(CoyoteSpoolConfig(tmp_path), clock_ms=lambda: 1)
    event_id = spool.new_event_id()
    spool.write_image(event_id, JPEG)
    event_dir = tmp_path / "events" / event_id
    (event_dir / "image.ready.json").replace(event_dir / "image.published.json")

    with pytest.raises(FileExistsError, match="already exists"):
        spool.write_image(event_id, JPEG)


def test_failure_manifest_has_no_fake_media_path(tmp_path):
    spool = CoyoteMediaSpool(CoyoteSpoolConfig(tmp_path), clock_ms=lambda: 1)
    event_id = spool.new_event_id()

    manifest = spool.write_failure(event_id, "image", reason="encoder failed")

    assert manifest["result"] == "FAIL"
    assert "path" not in manifest
    assert "bytes" not in manifest


def test_default_video_limit_leaves_room_for_base64_json_envelope(tmp_path):
    raw_limit = CoyoteSpoolConfig(tmp_path).max_video_bytes
    base64_limit = ((raw_limit + 2) // 3) * 4

    assert base64_limit + 4096 < MqttConfig().max_payload_bytes
