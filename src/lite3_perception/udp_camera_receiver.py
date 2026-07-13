"""Receive Lite3 body-camera frames from UDP via GStreamer (as-is JPEG).

Motion host push pipeline::

    v4l2src ! image/jpeg ! jpegparse ! rtpjpegpay
      ! udpsink host=<iq9> port=5000

IQ9 receive (this module)::

    udpsrc port=5000 ! application/x-rtp,encoding-name=JPEG,payload=26
      ! rtpjpegdepay ! jpegparse ! appsink

JPEG bytes from ``rtpjpegdepay`` are published as-is (no re-encode when using
the GStreamer backend).
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable


DEFAULT_UDP_PORT = 5000
DEFAULT_BIND_HOST = "0.0.0.0"
DEFAULT_PAYLOAD_TYPE = 26


@dataclass(frozen=True)
class UdpCameraConfig:
    bind_host: str = DEFAULT_BIND_HOST
    port: int = DEFAULT_UDP_PORT
    payload_type: int = DEFAULT_PAYLOAD_TYPE
    max_buffers: int = 2
    drop: bool = True
    jpeg_quality: int = 85  # only used by OpenCV BGR fallback re-encode
    open_timeout_sec: float = 5.0
    read_timeout_sec: float = 1.0

    def __post_init__(self) -> None:
        if not self.bind_host.strip():
            raise ValueError("bind_host must be non-empty")
        if not 1 <= self.port <= 65535:
            raise ValueError("port must be in 1..65535")
        if not 0 <= self.payload_type <= 127:
            raise ValueError("payload_type must be in 0..127")
        if self.max_buffers <= 0:
            raise ValueError("max_buffers must be positive")
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be in 1..100")
        if self.open_timeout_sec <= 0.0 or self.read_timeout_sec <= 0.0:
            raise ValueError("camera timeout values must be positive")


@dataclass(frozen=True)
class CameraFrame:
    jpeg_bytes: bytes
    timestamp_monotonic: float
    width: int | None = None
    height: int | None = None
    sequence: int = 0
    source: str = "udp"

    def decode_bgr(self):
        import cv2
        import numpy as np

        arr = np.frombuffer(self.jpeg_bytes, dtype=np.uint8)
        image = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("failed to decode JPEG frame")
        return image


@dataclass
class ReceiverStats:
    frames_completed: int = 0
    frames_dropped: int = 0
    bytes_received: int = 0
    last_frame_monotonic: float | None = None
    last_error: str | None = None
    backend: str = "none"
    open_ok: bool = False


def build_rtp_jpeg_caps(payload_type: int = DEFAULT_PAYLOAD_TYPE) -> str:
    # Keep caps simple — OpenCV and gst-launch both accept this form.
    return (
        "application/x-rtp,media=video,clock-rate=90000,"
        f"encoding-name=JPEG,payload={int(payload_type)}"
    )


def build_jpeg_appsink_pipeline(config: UdpCameraConfig) -> str:
    """Emit JPEG buffers as-is (preferred path)."""
    caps = build_rtp_jpeg_caps(config.payload_type)
    drop = "true" if config.drop else "false"
    # Do not quote caps with nested double-quotes (breaks OpenCV/Gst parse).
    return (
        f"udpsrc address={config.bind_host} port={config.port} caps={caps} "
        f"! rtpjpegdepay ! jpegparse "
        f"! appsink name=sink sync=false max-buffers={config.max_buffers} drop={drop}"
    )


def build_opencv_bgr_pipeline(config: UdpCameraConfig) -> str:
    """OpenCV CAP_GSTREAMER fallback (decoded BGR, then re-encoded JPEG)."""
    # OpenCV is picky: no name= on appsink, simple caps.
    return (
        f"udpsrc address={config.bind_host} port={config.port} ! "
        f"application/x-rtp,encoding-name=JPEG,payload={int(config.payload_type)} ! "
        f"rtpjpegdepay ! jpegdec ! videoconvert ! appsink"
    )


class UdpJpegCameraReceiver:
    """Background reader keeping the latest JPEG frame from UDP."""

    def __init__(
        self,
        config: UdpCameraConfig | None = None,
        *,
        on_frame: Callable[[CameraFrame], None] | None = None,
    ):
        self.config = config or UdpCameraConfig()
        self.on_frame = on_frame
        self.stats = ReceiverStats()
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._latest: CameraFrame | None = None
        self._frame_event = threading.Event()
        self._sequence = 0
        self._backend_impl: _Backend | None = None

    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        if self.is_running:
            return
        self._backend_impl = _open_backend(self.config, self.stats)
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._loop, name="udp-camera-gst", daemon=True
        )
        self._thread.start()

    def stop(self, join_timeout: float = 2.0) -> None:
        self._stop.set()
        if self._backend_impl is not None:
            self._backend_impl.close()
        if self._thread is not None:
            self._thread.join(timeout=join_timeout)
            if self._thread.is_alive():
                raise TimeoutError("UDP camera receiver did not stop within timeout")
            self._thread = None
        self._backend_impl = None
        self.stats.open_ok = False

    def get_latest_frame(self) -> CameraFrame | None:
        with self._lock:
            return self._latest

    def wait_for_frame(self, timeout: float | None = None) -> CameraFrame | None:
        if not self._frame_event.wait(timeout=timeout):
            return None
        self._frame_event.clear()
        return self.get_latest_frame()

    def _loop(self) -> None:
        assert self._backend_impl is not None
        while not self._stop.is_set():
            try:
                jpeg, width, height = self._backend_impl.read_jpeg(
                    timeout_sec=self.config.read_timeout_sec
                )
            except _Timeout:
                continue
            except Exception as exc:  # noqa: BLE001
                self.stats.last_error = str(exc)
                time.sleep(0.05)
                continue
            if not jpeg:
                continue

            now = time.monotonic()
            self._sequence += 1
            frame = CameraFrame(
                jpeg_bytes=jpeg,
                timestamp_monotonic=now,
                width=width,
                height=height,
                sequence=self._sequence,
                source=self.stats.backend,
            )
            with self._lock:
                self._latest = frame
            self.stats.frames_completed += 1
            self.stats.bytes_received += len(jpeg)
            self.stats.last_frame_monotonic = now
            self._frame_event.set()
            if self.on_frame is not None:
                try:
                    self.on_frame(frame)
                except Exception as exc:  # noqa: BLE001
                    self.stats.last_error = f"on_frame: {exc}"

    def __enter__(self) -> "UdpJpegCameraReceiver":
        self.start()
        return self

    def __exit__(self, *args) -> None:
        self.stop()


class _Timeout(Exception):
    pass


class _Backend:
    def read_jpeg(self, timeout_sec: float) -> tuple[bytes | None, int | None, int | None]:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError


def _open_backend(config: UdpCameraConfig, stats: ReceiverStats) -> _Backend:
    errors: list[str] = []
    try:
        backend = _GstJpegBackend(config)
        stats.backend = "gst-jpeg"
        stats.open_ok = True
        return backend
    except Exception as exc:  # noqa: BLE001
        errors.append(f"gst-jpeg: {exc}")

    try:
        backend = _OpenCvBgrBackend(config)
        stats.backend = "opencv-bgr"
        stats.open_ok = True
        return backend
    except Exception as exc:  # noqa: BLE001
        errors.append(f"opencv-bgr: {exc}")

    stats.last_error = "; ".join(errors)
    raise RuntimeError("failed to open UDP camera backend: " + stats.last_error)


class _GstJpegBackend(_Backend):
    """Pull JPEG from rtpjpegdepay via appsink (no GstApp import required)."""

    def __init__(self, config: UdpCameraConfig):
        import gi

        gi.require_version("Gst", "1.0")
        from gi.repository import Gst

        Gst.init(None)
        self._Gst = Gst
        pipeline_str = build_jpeg_appsink_pipeline(config)
        self._pipeline = Gst.parse_launch(pipeline_str)
        self._sink = self._pipeline.get_by_name("sink")
        if self._sink is None:
            raise RuntimeError("appsink 'sink' missing")
        ret = self._pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError(f"pipeline failed: {pipeline_str}")
        # Brief settle so first pull is less likely to time out immediately.
        self._pipeline.get_state(int(0.5 * Gst.SECOND))

    def read_jpeg(self, timeout_sec: float) -> tuple[bytes | None, int | None, int | None]:
        Gst = self._Gst
        timeout_ns = int(max(timeout_sec, 0.01) * Gst.SECOND)
        # Works on GstAppSink without importing GstApp.
        sample = self._sink.emit("try-pull-sample", timeout_ns)
        if sample is None:
            raise _Timeout()
        buf = sample.get_buffer()
        caps = sample.get_caps()
        width = height = None
        if caps is not None and caps.get_size() > 0:
            structure = caps.get_structure(0)
            ok_w, w = structure.get_int("width")
            ok_h, h = structure.get_int("height")
            width = w if ok_w else None
            height = h if ok_h else None
        success, mapinfo = buf.map(Gst.MapFlags.READ)
        if not success:
            return None, width, height
        try:
            data = bytes(mapinfo.data)
        finally:
            buf.unmap(mapinfo)
        return data, width, height

    def close(self) -> None:
        if self._pipeline is not None:
            self._pipeline.set_state(self._Gst.State.NULL)
            self._pipeline = None


class _OpenCvBgrBackend(_Backend):
    def __init__(self, config: UdpCameraConfig):
        import cv2

        self._cv2 = cv2
        self._quality = int(config.jpeg_quality)
        pipeline = build_opencv_bgr_pipeline(config)
        self._cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not self._cap.isOpened():
            raise RuntimeError(f"OpenCV could not open: {pipeline}")

    def read_jpeg(self, timeout_sec: float) -> tuple[bytes | None, int | None, int | None]:
        _ = timeout_sec
        ok, frame = self._cap.read()
        if not ok or frame is None:
            raise _Timeout()
        h, w = frame.shape[:2]
        ok, buf = self._cv2.imencode(
            ".jpg", frame, [int(self._cv2.IMWRITE_JPEG_QUALITY), self._quality]
        )
        if not ok:
            return None, w, h
        return buf.tobytes(), w, h

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None
