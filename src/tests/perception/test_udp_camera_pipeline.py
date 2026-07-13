"""Unit tests for GStreamer pipeline string helpers (no live UDP required)."""

import pytest

from lite3_perception.udp_camera_receiver import (
    UdpCameraConfig,
    build_jpeg_appsink_pipeline,
    build_opencv_bgr_pipeline,
    build_rtp_jpeg_caps,
)


def test_caps_include_jpeg_payload():
    caps = build_rtp_jpeg_caps(26)
    assert "encoding-name=JPEG" in caps
    assert "payload=26" in caps


def test_jpeg_pipeline_uses_depay_not_custom_reassembly():
    pipe = build_jpeg_appsink_pipeline(UdpCameraConfig(port=5000))
    assert "udpsrc" in pipe
    assert "port=5000" in pipe
    assert "rtpjpegdepay" in pipe
    assert "jpegparse" in pipe
    assert "appsink" in pipe
    assert "rtpjpegpay" not in pipe  # we receive, not send
    # Avoid nested quotes that break Gst/OpenCV parsers.
    assert 'caps="' not in pipe


def test_opencv_pipeline_decodes_to_bgr():
    pipe = build_opencv_bgr_pipeline(UdpCameraConfig(port=5000, payload_type=26))
    assert "jpegdec" in pipe
    assert "appsink" in pipe
    assert "rtpjpegdepay" in pipe


def test_bind_host_is_applied_to_both_receiver_pipelines():
    config = UdpCameraConfig(bind_host="192.168.1.215", port=5000)
    assert "address=192.168.1.215" in build_jpeg_appsink_pipeline(config)
    assert "address=192.168.1.215" in build_opencv_bgr_pipeline(config)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"port": 0},
        {"port": 65536},
        {"payload_type": 128},
        {"max_buffers": 0},
        {"read_timeout_sec": 0.0},
    ],
)
def test_invalid_udp_camera_config_is_rejected(kwargs):
    with pytest.raises(ValueError):
        UdpCameraConfig(**kwargs)
