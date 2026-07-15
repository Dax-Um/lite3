import json
import time

import numpy as np
import pytest

try:
    from lite3_perception.coyote_pipeline import CoyoteController, CoyoteTopicConfig
except ModuleNotFoundError as exc:
    if exc.name != "cv2":
        raise
    CoyoteController = None
    CoyoteTopicConfig = None


requires_cv2 = pytest.mark.skipif(CoyoteController is None, reason="cv2 unavailable")


@requires_cv2
def test_existing_controller_api_writes_same_event_image_and_five_second_mp4(tmp_path):
    logs = []
    config = CoyoteTopicConfig(
        file_sink_dir=str(tmp_path),
        status_every_n=1,
        video_seconds=5.0,
        media_cooldown_sec=0.0,
        video_max_fps=10.0,
    )
    controller = CoyoteController(config, logs.append)
    frame = np.zeros((120, 160, 3), dtype=np.uint8)
    annotated = frame.copy()
    annotated[20:80, 30:100] = (0, 165, 255)

    for frame_id in range(1, 4):
        status = controller.update(
            frame_id,
            frame,
            annotated,
            [30.0, 20.0, 100.0, 80.0],
            0.9,
        )

    assert status["detect"] == "detected"
    assert status["motion"] == "forward"
    assert status["side"] == "center"
    assert set(status) == {"ts", "detect", "motion", "side"}
    event_id = controller.event_id
    assert event_id
    event_dir = tmp_path / "events" / event_id
    assert (event_dir / "image.ready.json").exists()
    assert controller.collecting_video is True

    controller.video_frames = [annotated.copy() for _ in range(50)]
    controller.video_start_ts = time.time() - 5.1
    controller._finish_video_async()
    controller.close()

    video_manifest = json.loads((event_dir / "video.ready.json").read_text())
    assert video_manifest["event_id"] == event_id
    assert video_manifest["duration_ms"] == 5000
    assert (event_dir / "video.mp4").read_bytes()[4:8] == b"ftyp"


@requires_cv2
def test_detection_loss_does_not_shorten_in_progress_clip(tmp_path):
    controller = CoyoteController(
        CoyoteTopicConfig(
            file_sink_dir=str(tmp_path),
            status_every_n=100,
            media_cooldown_sec=0.0,
            video_seconds=60.0,
        ),
        lambda value: None,
    )
    frame = np.zeros((60, 80, 3), dtype=np.uint8)
    for frame_id in range(1, 4):
        controller.update(frame_id, frame, frame, [1.0, 1.0, 20.0, 20.0], 0.9)

    for frame_id in range(4, 12):
        controller.update(frame_id, frame, frame, None, 0.0)

    assert controller.detect == "not_detected"
    assert controller.collecting_video is True
    controller.close()

    event_dir = tmp_path / "events" / controller.video_event_id
    failure = json.loads((event_dir / "video.ready.json").read_text())
    assert failure["result"] == "FAIL"


@requires_cv2
@pytest.mark.parametrize(
    ("center_ratio", "expected_side", "expected_motion"),
    (
        (0.399, "left", "stop"),
        (0.4, "center", "forward"),
        (0.5, "center", "forward"),
        (0.6, "center", "forward"),
        (0.601, "right", "stop"),
    ),
)
def test_side_uses_bbox_center_with_inclusive_center_boundaries(
    tmp_path, center_ratio, expected_side, expected_motion
):
    controller = CoyoteController(
        CoyoteTopicConfig(
            file_sink_dir=str(tmp_path),
            hit_confirm=1,
            publish_enabled=False,
        ),
        lambda value: None,
    )
    frame = np.zeros((100, 100, 3), dtype=np.uint8)
    center_x = center_ratio * 100.0

    status = controller.update(
        1,
        frame,
        frame,
        [center_x - 5.0, 10.0, center_x + 5.0, 20.0],
        0.9,
    )

    assert set(status) == {"ts", "detect", "motion", "side"}
    assert isinstance(status["ts"], float)
    assert status["ts"] > 0.0
    assert status["detect"] == "detected"
    assert status["side"] == expected_side
    assert status["motion"] == expected_motion


@requires_cv2
def test_center_detection_stops_when_near(tmp_path):
    controller = CoyoteController(
        CoyoteTopicConfig(
            file_sink_dir=str(tmp_path),
            hit_confirm=1,
            publish_enabled=False,
        ),
        lambda value: None,
    )
    frame = np.zeros((100, 100, 3), dtype=np.uint8)

    status = controller.update(
        1,
        frame,
        frame,
        [0.0, 0.0, 100.0, 70.0],
        0.9,
    )

    assert status["detect"] == "detected"
    assert status["side"] == "center"
    assert status["motion"] == "stop"
    assert controller.near_mode is True


@requires_cv2
def test_near_hysteresis_uses_bbox_height_not_bbox_area(tmp_path):
    controller = CoyoteController(
        CoyoteTopicConfig(
            file_sink_dir=str(tmp_path),
            hit_confirm=1,
            publish_enabled=False,
            near_enter=0.65,
            near_exit=0.50,
        ),
        lambda value: None,
    )
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)

    def update(frame_id, height_ratio):
        y1 = 720.0 * (1.0 - height_ratio)
        return controller.update(
            frame_id,
            frame,
            frame,
            [580.0, y1, 700.0, 720.0],
            0.9,
        )

    assert update(1, 0.649)["motion"] == "forward"
    assert update(2, 0.650)["motion"] == "stop"
    assert controller.near_mode is True
    assert controller.area_ratio < 0.10
    assert update(3, 0.550)["motion"] == "stop"
    assert update(4, 0.500)["motion"] == "forward"
    assert controller.near_mode is False


@requires_cv2
def test_detected_status_log_contains_internal_topic_and_exact_payload(tmp_path):
    logs = []
    controller = CoyoteController(
        CoyoteTopicConfig(
            file_sink_dir=str(tmp_path),
            status_every_n=1,
            hit_confirm=1,
        ),
        logs.append,
    )
    frame = np.zeros((100, 100, 3), dtype=np.uint8)

    status = controller.update(
        1,
        frame,
        frame,
        [45.0, 10.0, 55.0, 20.0],
        0.9,
    )

    publish_log = next(item for item in logs if item["type"] == "status_publish")
    assert publish_log["topic"] == "/lite3/data/coyote/status"
    assert publish_log["payload"] == status
    assert set(publish_log["payload"]) == {"ts", "detect", "motion", "side"}


@requires_cv2
def test_unconfirmed_detection_does_not_publish_side_or_forward(tmp_path):
    controller = CoyoteController(
        CoyoteTopicConfig(
            file_sink_dir=str(tmp_path),
            hit_confirm=2,
            publish_enabled=False,
        ),
        lambda value: None,
    )
    frame = np.zeros((100, 100, 3), dtype=np.uint8)

    status = controller.update(
        1,
        frame,
        frame,
        [45.0, 10.0, 55.0, 20.0],
        0.9,
    )

    assert status == {
        "ts": status["ts"],
        "detect": "not_detected",
        "motion": "stop",
        "side": "none",
    }


@requires_cv2
def test_bbox_loss_immediately_stops_and_clears_side_before_detection_timeout(
    tmp_path,
):
    controller = CoyoteController(
        CoyoteTopicConfig(
            file_sink_dir=str(tmp_path),
            hit_confirm=1,
            miss_lost=3,
            publish_enabled=False,
        ),
        lambda value: None,
    )
    frame = np.zeros((100, 100, 3), dtype=np.uint8)

    detected = controller.update(
        1,
        frame,
        frame,
        [45.0, 10.0, 55.0, 20.0],
        0.9,
    )
    first_miss = controller.update(2, frame, frame, None, 0.0)
    second_miss = controller.update(3, frame, frame, None, 0.0)
    lost = controller.update(4, frame, frame, None, 0.0)

    assert detected["detect"] == "detected"
    assert detected["motion"] == "forward"
    assert detected["side"] == "center"
    assert first_miss["detect"] == "not_detected"
    assert first_miss["motion"] == "stop"
    assert first_miss["side"] == "none"
    assert second_miss["detect"] == "not_detected"
    assert second_miss["motion"] == "stop"
    assert second_miss["side"] == "none"
    assert lost["detect"] == "not_detected"
    assert lost["motion"] == "stop"
    assert lost["side"] == "none"
    assert controller.episode_active is False


@requires_cv2
@pytest.mark.parametrize(
    ("bbox", "score"),
    (
        ([60.0, 10.0, 40.0, 20.0], 0.9),
        ([40.0, 20.0, 60.0, 10.0], 0.9),
        ([-1.0, 10.0, 60.0, 20.0], 0.9),
        ([40.0, 10.0, 101.0, 20.0], 0.9),
        ([40.0, 10.0, 60.0, 20.0], 0.0),
        ([40.0, 10.0, 60.0, 20.0], 1.1),
    ),
)
def test_invalid_bbox_or_score_cannot_become_forward(tmp_path, bbox, score):
    logs = []
    controller = CoyoteController(
        CoyoteTopicConfig(
            file_sink_dir=str(tmp_path),
            hit_confirm=1,
            publish_enabled=False,
        ),
        logs.append,
    )
    frame = np.zeros((100, 100, 3), dtype=np.uint8)

    status = controller.update(1, frame, frame, bbox, score)

    assert status["detect"] == "not_detected"
    assert status["motion"] == "stop"
    assert status["side"] == "none"
    assert any(item["type"] == "invalid_detection_drop" for item in logs)


@requires_cv2
def test_overlapping_episode_never_orphans_previous_video(tmp_path):
    controller = CoyoteController(
        CoyoteTopicConfig(
            file_sink_dir=str(tmp_path),
            hit_confirm=1,
            miss_lost=1,
            media_cooldown_sec=0.0,
            video_seconds=60.0,
        ),
        lambda value: None,
    )
    frame = np.zeros((100, 100, 3), dtype=np.uint8)
    bbox = [45.0, 10.0, 55.0, 20.0]

    controller.update(1, frame, frame, bbox, 0.9)
    first_event_id = controller.event_id
    controller.update(2, frame, frame, None, 0.0)
    controller.update(3, frame, frame, bbox, 0.9)
    second_event_id = controller.event_id

    assert second_event_id != first_event_id
    assert controller.video_event_id == first_event_id
    second_video = json.loads(
        (tmp_path / "events" / second_event_id / "video.ready.json").read_text()
    )
    assert second_video["result"] == "FAIL"

    controller.close()
    assert (tmp_path / "events" / first_event_id / "video.ready.json").exists()


@requires_cv2
@pytest.mark.parametrize(
    "values",
    (
        {"near_enter": float("nan")},
        {"near_exit": 0.8, "near_enter": 0.7},
        {"center_half_ratio": 0.20},
        {"video_seconds": float("inf")},
        {"video_max_fps": 0.0},
    ),
)
def test_fail_closed_configuration_validation(tmp_path, values):
    with pytest.raises(ValueError):
        CoyoteController(
            CoyoteTopicConfig(file_sink_dir=str(tmp_path), **values),
            lambda value: None,
        )
