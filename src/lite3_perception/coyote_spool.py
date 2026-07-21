"""Atomic, event-scoped storage shared by coyote perception and MQTT bridge."""

from __future__ import annotations

import hashlib
import json
import os
import re
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, Optional, Union


INTERNAL_STATUS_TOPIC = "/lite3/data/coyote/status"
INTERNAL_IMAGE_TOPIC = "/lite3/data/coyote/image"

_EVENT_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")
_JPEG_START = b"\xff\xd8"
_JPEG_END = b"\xff\xd9"


@dataclass(frozen=True)
class CoyoteSpoolConfig:
    root: Path
    max_image_bytes: int = 8 * 1024 * 1024

    def __post_init__(self) -> None:
        if self.max_image_bytes <= 0:
            raise ValueError("spool image limit must be positive")


class CoyoteMediaSpool:
    """Write final media first and a ready manifest last.

    The producer can run in the native QNN Python environment while the Foxy
    bridge observes the same bind-mounted directory.  A bridge must only act
    on ``*.ready.json`` files; ``*.part-*`` files are never complete inputs.
    """

    def __init__(
        self,
        config: CoyoteSpoolConfig,
        *,
        clock_ms: Optional[Callable[[], int]] = None,
    ) -> None:
        self.config = config
        self.root = Path(config.root).expanduser().resolve()
        self.events_dir = self.root / "events"
        self.events_dir.mkdir(parents=True, exist_ok=True)
        self._clock_ms = clock_ms or (lambda: time.time_ns() // 1_000_000)

    def new_event_id(self) -> str:
        return "coyote-{}-{}".format(self._clock_ms(), uuid.uuid4().hex[:8])

    def write_status(self, status: Dict[str, Any]) -> Path:
        if not isinstance(status, dict):
            raise ValueError("coyote status must be a JSON object")
        body = json.dumps(
            status,
            ensure_ascii=False,
            separators=(",", ":"),
        ).encode("utf-8")
        path = self.root / "status.json"
        _atomic_write(path, body)
        return path

    def write_image(
        self,
        event_id: str,
        jpeg_bytes: Union[bytes, bytearray],
        *,
        source_timestamp_ms: Optional[int] = None,
    ) -> Dict[str, Any]:
        event_id = validate_event_id(event_id)
        data = bytes(jpeg_bytes)
        if not (len(data) >= 4 and data.startswith(_JPEG_START) and data.endswith(_JPEG_END)):
            raise ValueError("coyote image must be a complete JPEG")
        if len(data) > self.config.max_image_bytes:
            raise ValueError("coyote image exceeds spool size limit")
        event_dir = self._event_dir(event_id)
        _ensure_new_artifact(event_dir, "image")
        media_path = event_dir / "image.jpg"
        _atomic_write(media_path, data)
        manifest = self._manifest(
            event_id=event_id,
            kind="image",
            media_path=media_path,
            media_format="jpeg",
            data=data,
            source_timestamp_ms=source_timestamp_ms,
        )
        _atomic_write_json(event_dir / "image.ready.json", manifest)
        return manifest

    def write_failure(
        self,
        event_id: str,
        kind: str,
        *,
        reason: str,
    ) -> Dict[str, Any]:
        """Queue a contract-level FAIL without inventing invalid media bytes."""
        event_id = validate_event_id(event_id)
        if kind != "image":
            raise ValueError("coyote media kind must be image")
        event_dir = self._event_dir(event_id)
        _ensure_new_artifact(event_dir, kind)
        manifest = {
            "version": 1,
            "event_id": event_id,
            "kind": kind,
            "topic": INTERNAL_IMAGE_TOPIC,
            "format": "jpeg",
            "result": "FAIL",
            "failure_reason": str(reason)[:512],
            "created_at": int(self._clock_ms()),
        }  # type: Dict[str, Any]
        _atomic_write_json(event_dir / "{}.ready.json".format(kind), manifest)
        return manifest

    def _event_dir(self, event_id: str) -> Path:
        event_dir = self.events_dir / event_id
        event_dir.mkdir(parents=True, exist_ok=True)
        return event_dir

    def _manifest(
        self,
        *,
        event_id: str,
        kind: str,
        media_path: Path,
        media_format: str,
        data: bytes,
        source_timestamp_ms: Optional[int] = None,
    ) -> Dict[str, Any]:
        manifest = {
            "version": 1,
            "event_id": event_id,
            "kind": kind,
            "topic": INTERNAL_IMAGE_TOPIC,
            "format": media_format,
            "result": "SUCCESS",
            "path": str(media_path.resolve()),
            "bytes": len(data),
            "sha256": hashlib.sha256(data).hexdigest(),
            "created_at": int(self._clock_ms()),
        }  # type: Dict[str, Any]
        if source_timestamp_ms is not None:
            manifest["source_timestamp"] = int(source_timestamp_ms)
        return manifest


def validate_event_id(event_id: str) -> str:
    if not isinstance(event_id, str) or not _EVENT_ID.fullmatch(event_id):
        raise ValueError("event_id contains unsupported characters or length")
    return event_id


def _ensure_new_artifact(event_dir: Path, kind: str) -> None:
    for state in ("ready", "sending", "published", "failed"):
        if (event_dir / "{}.{}.json".format(kind, state)).exists():
            raise FileExistsError(
                "coyote {} artifact already exists for event {}".format(
                    kind, event_dir.name
                )
            )


def _atomic_write_json(path: Path, value: Dict[str, Any]) -> None:
    _atomic_write(
        path,
        json.dumps(value, ensure_ascii=False, separators=(",", ":")).encode("utf-8"),
    )


def _atomic_write(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    part = path.with_name("{}.part-{}".format(path.name, uuid.uuid4().hex))
    try:
        with part.open("xb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(str(part), str(path))
        _fsync_directory(path.parent)
    finally:
        try:
            part.unlink()
        except FileNotFoundError:
            pass


def _fsync_directory(path: Path) -> None:
    flags = os.O_RDONLY
    if hasattr(os, "O_DIRECTORY"):
        flags |= os.O_DIRECTORY
    descriptor = os.open(str(path), flags)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
