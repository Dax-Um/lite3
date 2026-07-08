"""Camera source configuration for IQ9 web and inference paths."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping
from urllib.parse import urlsplit, urlunsplit


@dataclass(frozen=True)
class CameraSourceConfig:
    source_type: str
    url: str | None
    redacted_url: str | None
    yolo_input_enabled: bool

    @classmethod
    def from_env(cls, env: Mapping[str, str]) -> "CameraSourceConfig":
        source_type = env.get("LITE3_CAMERA_SOURCE", "").strip().lower()
        rtsp_url = env.get("LITE3_RTSP_URL", "").strip()

        if source_type == "rtsp" or rtsp_url:
            if not rtsp_url:
                return cls(
                    source_type="rtsp",
                    url=None,
                    redacted_url=None,
                    yolo_input_enabled=False,
                )
            _validate_rtsp_url(rtsp_url)
            return cls(
                source_type="rtsp",
                url=rtsp_url,
                redacted_url=_redact_url(rtsp_url),
                yolo_input_enabled=True,
            )

        return cls(
            source_type="disabled",
            url=None,
            redacted_url=None,
            yolo_input_enabled=False,
        )


def _validate_rtsp_url(url: str) -> None:
    parts = urlsplit(url)
    if parts.scheme != "rtsp" or not parts.hostname:
        raise ValueError("LITE3_RTSP_URL must be an rtsp:// URL with a host")


def _redact_url(url: str) -> str:
    parts = urlsplit(url)
    netloc = "<host>"
    if parts.port is not None:
        netloc = f"{netloc}:{parts.port}"
    return urlunsplit((parts.scheme, netloc, parts.path, parts.query, parts.fragment))
