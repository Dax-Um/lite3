import base64

import pytest

from lite3_mqtt.contract import (
    DetectionType,
    PatrolAction,
    Topics,
    build_image_payload,
    build_video_payload,
    parse_detection_trigger,
    parse_patrol_command,
)


def test_patrol_command_requires_epoch_milliseconds_number():
    command = parse_patrol_command('{"timestamp":1783652400000,"action":"START"}')
    assert command.action is PatrolAction.START

    with pytest.raises(ValueError, match="milliseconds"):
        parse_patrol_command('{"timestamp":1783652400,"action":"START"}')
    with pytest.raises(ValueError, match="JSON number"):
        parse_patrol_command('{"timestamp":"1783652400000","action":"START"}')


def test_triggers_map_to_pipeline_detection_types():
    sound = parse_detection_trigger(
        Topics.SOUND_DETECT,
        '{"event_id":"sound-1","timestamp":1783652400000,"event_type":"GLASS_BROKEN"}',
    )
    coyote = parse_detection_trigger(
        Topics.COYOTE_DETECT,
        '{"event_id":"coyote-1","timestamp":1783652400001,"event_type":"COYOTE_DETECTED"}',
    )
    assert sound.detection_type is DetectionType.BROKEN_CUP
    assert coyote.detection_type is DetectionType.COYOTE


def test_media_payloads_encode_bytes_and_keep_independent_timestamps():
    times = iter([1783652400123, 1783652405341])
    clock = lambda: next(times)
    image = build_image_payload(
        event_id="event-1",
        detection_type=DetectionType.BROKEN_CUP,
        jpeg_bytes=b"jpeg",
        clock_ms=clock,
    )
    video = build_video_payload(
        event_id="event-1",
        detection_type=DetectionType.BROKEN_CUP,
        mp4_bytes=b"mp4",
        duration_ms=5000,
        clock_ms=clock,
    )
    assert image["event_id"] == video["event_id"]
    assert image["timestamp"] != video["timestamp"]
    assert base64.b64decode(image["image"]["data_base64"]) == b"jpeg"
    assert base64.b64decode(video["video"]["data_base64"]) == b"mp4"


def test_empty_media_uses_fail_result_and_empty_base64():
    payload = build_image_payload(
        event_id="event-1",
        detection_type=DetectionType.COYOTE,
        jpeg_bytes=None,
    )
    assert payload["result"] == "FAIL"
    assert payload["image"]["data_base64"] == ""
