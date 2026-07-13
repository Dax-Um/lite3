"""MQTT 3.1.1 topic and JSON contract from docs/design/20."""

from __future__ import annotations

import base64
import json
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Dict, Union


class Topics:
    AUTO_PATROL = "/lite3/data/auto_patrol"
    SOUND_DETECT = "/lite3/data/sound_detect"
    COYOTE_DETECT = "/lite3/data/coyote_detect"

    BROKEN_CUP_IMAGE = "/aicenter/data/broken_cup_image"
    BROKEN_CUP_VIDEO = "/aicenter/data/broken_cup_video"
    COYOTE_IMAGE = "/aicenter/data/coyote_image"
    COYOTE_VIDEO = "/aicenter/data/coyote_video"

    SUBSCRIPTIONS = (AUTO_PATROL, SOUND_DETECT, COYOTE_DETECT)


class PatrolAction(str, Enum):
    START = "START"
    STOP = "STOP"
    RETURN_HOME = "RETURN_HOME"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    RESET = "RESET"


class DetectionType(str, Enum):
    BROKEN_CUP = "BROKEN_CUP"
    COYOTE = "COYOTE"


class Result(str, Enum):
    SUCCESS = "SUCCESS"
    FAIL = "FAIL"


@dataclass(frozen=True)
class PatrolCommand:
    timestamp: int
    action: PatrolAction


@dataclass(frozen=True)
class DetectionTrigger:
    event_id: str
    timestamp: int
    detection_type: DetectionType


def epoch_ms() -> int:
    return time.time_ns() // 1_000_000


def decode_object(payload: Union[bytes, str]) -> Dict[str, Any]:
    try:
        value = json.loads(payload.decode("utf-8") if isinstance(payload, bytes) else payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid JSON payload: {exc}") from exc
    if not isinstance(value, dict):
        raise ValueError("MQTT payload must be a JSON object")
    return value


def parse_patrol_command(payload: Union[bytes, str]) -> PatrolCommand:
    value = decode_object(payload)
    timestamp = _required_epoch_ms(value)
    raw_action = _required_string(value, "action")
    try:
        action = PatrolAction(raw_action)
    except ValueError as exc:
        allowed = ", ".join(item.value for item in PatrolAction)
        raise ValueError(f"unsupported patrol action {raw_action!r}; expected {allowed}") from exc
    return PatrolCommand(timestamp=timestamp, action=action)


def parse_detection_trigger(topic: str, payload: Union[bytes, str]) -> DetectionTrigger:
    value = decode_object(payload)
    event_id = _required_string(value, "event_id")
    if len(event_id) > 128:
        raise ValueError("event_id must be at most 128 characters")
    timestamp = _required_epoch_ms(value)
    event_type = _required_string(value, "event_type")

    if topic == Topics.SOUND_DETECT:
        if event_type != "GLASS_BROKEN":
            raise ValueError("sound_detect event_type must be GLASS_BROKEN")
        detection_type = DetectionType.BROKEN_CUP
    elif topic == Topics.COYOTE_DETECT:
        if event_type != "COYOTE_DETECTED":
            raise ValueError("coyote_detect event_type must be COYOTE_DETECTED")
        detection_type = DetectionType.COYOTE
    else:
        raise ValueError(f"unsupported detection trigger topic: {topic}")
    return DetectionTrigger(
        event_id=event_id,
        timestamp=timestamp,
        detection_type=detection_type,
    )


def build_image_payload(
    *,
    event_id: str,
    detection_type: DetectionType,
    jpeg_bytes: Union[bytes, None],
    clock_ms: Callable[[], int] = epoch_ms,
) -> Dict[str, Any]:
    event_id = _validated_event_id(event_id)
    success = _is_jpeg(jpeg_bytes)
    return {
        "event_id": event_id,
        "timestamp": int(clock_ms()),
        "event_type": detection_type.value,
        "result": Result.SUCCESS.value if success else Result.FAIL.value,
        "image": {
            "format": "jpeg",
            "data_base64": _base64(jpeg_bytes if success else None),
        },
    }


def build_video_payload(
    *,
    event_id: str,
    detection_type: DetectionType,
    mp4_bytes: Union[bytes, None],
    duration_ms: int,
    clock_ms: Callable[[], int] = epoch_ms,
) -> Dict[str, Any]:
    event_id = _validated_event_id(event_id)
    if isinstance(duration_ms, bool) or not isinstance(duration_ms, int) or duration_ms <= 0:
        raise ValueError("duration_ms must be a positive integer")
    success = _is_mp4(mp4_bytes)
    return {
        "event_id": event_id,
        "timestamp": int(clock_ms()),
        "event_type": detection_type.value,
        "result": Result.SUCCESS.value if success else Result.FAIL.value,
        "video": {
            "format": "mp4",
            "duration_ms": int(duration_ms),
            "data_base64": _base64(mp4_bytes if success else None),
        },
    }


def image_topic(detection_type: DetectionType) -> str:
    if detection_type is DetectionType.BROKEN_CUP:
        return Topics.BROKEN_CUP_IMAGE
    if detection_type is DetectionType.COYOTE:
        return Topics.COYOTE_IMAGE
    raise ValueError("unsupported detection type: {!r}".format(detection_type))


def video_topic(detection_type: DetectionType) -> str:
    if detection_type is DetectionType.BROKEN_CUP:
        return Topics.BROKEN_CUP_VIDEO
    if detection_type is DetectionType.COYOTE:
        return Topics.COYOTE_VIDEO
    raise ValueError("unsupported detection type: {!r}".format(detection_type))


def compact_json(payload: Dict[str, Any]) -> str:
    return json.dumps(payload, ensure_ascii=False, separators=(",", ":"))


def _base64(data: Union[bytes, None]) -> str:
    return "" if not data else base64.b64encode(data).decode("ascii")


def _validated_event_id(event_id: str) -> str:
    if not isinstance(event_id, str) or not event_id.strip():
        raise ValueError("event_id must be a non-empty string")
    value = event_id.strip()
    if len(value) > 128:
        raise ValueError("event_id must be at most 128 characters")
    return value


def _is_jpeg(data: Union[bytes, None]) -> bool:
    return bool(data and len(data) >= 4 and data[:2] == b"\xff\xd8" and data[-2:] == b"\xff\xd9")


def _is_mp4(data: Union[bytes, None]) -> bool:
    return bool(data and len(data) >= 12 and data[4:8] == b"ftyp")


def _required_string(value: Dict[str, Any], key: str) -> str:
    item = value.get(key)
    if not isinstance(item, str) or not item.strip():
        raise ValueError(f"{key} must be a non-empty string")
    return item.strip()


def _required_epoch_ms(value: Dict[str, Any]) -> int:
    timestamp = value.get("timestamp")
    if isinstance(timestamp, bool) or not isinstance(timestamp, int):
        raise ValueError("timestamp must be an epoch-milliseconds JSON number")
    if timestamp < 1_000_000_000_000:
        raise ValueError("timestamp must be epoch milliseconds, not epoch seconds")
    return timestamp
