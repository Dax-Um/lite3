import json
import math

import pytest

from lite3_mqtt.contract import DetectionType, PatrolAction
from lite3_mqtt.coyote_bridge import (
    CoyoteMediaWorker,
    CoyoteMotionController,
    CoyoteSpoolReader,
    parse_coyote_status,
)
from lite3_perception.coyote_spool import CoyoteMediaSpool, CoyoteSpoolConfig


JPEG = b"\xff\xd8annotated\xff\xd9"
MP4 = b"\x00\x00\x00\x18ftypmp42"


def status_payload(**overrides):
    value = {
        "ts": 100.0,
        "detect": "detected",
        "motion": "forward",
        "side": "center",
    }
    value.update(overrides)
    return json.dumps(value)


class FakeMotionSink:
    def __init__(self):
        self.commands = []
        self.releases = 0
        self.acquires = 0

    def acquire(self):
        self.acquires += 1

    def send_cmd_vel(self, vx, vy, wz):
        self.commands.append((vx, vy, wz))

    def release(self):
        self.releases += 1


def scan(
    *,
    left=2.0,
    right=2.0,
    front=2.0,
    clockwise_rear=2.0,
    counterclockwise_rear=2.0,
):
    values = []
    for degrees in range(-180, 181):
        value = 2.0
        if -105 <= degrees <= -15:
            value = right
        if 15 <= degrees <= 105:
            value = left
        if 136 <= degrees <= 180:
            value = min(value, clockwise_rear)
        if -180 <= degrees <= -136:
            value = min(value, counterclockwise_rear)
        if -20 <= degrees <= 20:
            value = min(value, front)
        values.append(value)
    return values, -math.pi, math.radians(1.0)


def prime_sensors(controller, *, x=0.0, y=0.0, yaw=0.0, **clearances):
    controller.update_scan(*scan(**clearances))
    controller.update_odom(x, y, yaw)


def test_status_parser_matches_perception_contract():
    status = parse_coyote_status(status_payload())

    assert status.detect == "detected"
    assert status.motion == "forward"
    assert status.side == "center"
    assert status.timestamp_sec == 100.0


@pytest.mark.parametrize(
    "override",
    (
        {"ts": "100"},
        {"ts": float("nan")},
        {"detect": "unknown"},
        {"motion": "left"},
        {"side": "up"},
        {"detect": "not_detected", "motion": "forward", "side": "center"},
        {"detect": "not_detected", "motion": "stop", "side": "left"},
        {"detect": "detected", "motion": "forward", "side": "left"},
        {"detect": "detected", "motion": "stop", "side": "none"},
    ),
)
def test_status_parser_rejects_invalid_control_fields(override):
    with pytest.raises(ValueError):
        parse_coyote_status(status_payload(**override))


def test_only_fresh_detected_forward_moves_on_x_axis():
    wall = [100.0]
    monotonic = [10.0]
    sink = FakeMotionSink()
    controller = CoyoteMotionController(
        sink,
        wall_clock=lambda: wall[0],
        monotonic_clock=lambda: monotonic[0],
    )
    prime_sensors(controller)

    controller.start_search("forward-event")
    controller.handle_status(status_payload())
    assert controller.tick() == (1.00, 0.0, 0.0)
    assert sink.commands[-1] == (1.00, 0.0, 0.0)


def test_visible_center_status_auto_arms_without_search_event():
    sink = FakeMotionSink()
    controller = CoyoteMotionController(
        sink,
        wall_clock=lambda: 100.0,
        monotonic_clock=lambda: 10.0,
    )
    prime_sensors(controller)

    controller.handle_status(status_payload())

    assert sink.acquires == 1
    assert controller.tick() == (1.00, 0.0, 0.0)
    assert controller.last_reason == "forward"


def test_visible_side_status_auto_arms_and_aligns_without_search_event():
    sink = FakeMotionSink()
    controller = CoyoteMotionController(
        sink,
        wall_clock=lambda: 100.0,
        monotonic_clock=lambda: 10.0,
    )
    prime_sensors(controller)

    controller.handle_status(status_payload(motion="stop", side="right"))

    assert sink.acquires == 1
    assert controller.tick() == (0.0, 0.0, -0.60)
    assert controller.last_reason == "align_right"


def test_operator_stop_blocks_status_auto_arm_until_reset():
    sink = FakeMotionSink()
    controller = CoyoteMotionController(
        sink,
        wall_clock=lambda: 100.0,
        monotonic_clock=lambda: 10.0,
    )
    prime_sensors(controller)
    controller.handle_patrol_command(PatrolAction.STOP, 1000)

    controller.handle_status(status_payload())

    assert controller.tick() == (0.0, 0.0, 0.0)
    assert sink.acquires == 0

    controller.handle_patrol_command(PatrolAction.RESET, 1001)
    controller.handle_status(status_payload())
    assert sink.acquires == 1
    assert controller.tick() == (1.00, 0.0, 0.0)


def test_already_visible_center_goes_forward_without_search_turn():
    sink = FakeMotionSink()
    controller = CoyoteMotionController(
        sink,
        wall_clock=lambda: 100.0,
        monotonic_clock=lambda: 10.0,
    )
    prime_sensors(controller)
    controller.handle_status(status_payload())

    controller.start_search("already-visible-center")

    assert controller.tick() == (1.00, 0.0, 0.0)
    assert controller.last_reason == "forward"


def test_already_visible_side_aligns_without_blind_search_turn():
    sink = FakeMotionSink()
    controller = CoyoteMotionController(
        sink,
        wall_clock=lambda: 100.0,
        monotonic_clock=lambda: 10.0,
    )
    prime_sensors(controller)
    controller.handle_status(status_payload(motion="stop", side="left"))

    controller.start_search("already-visible-left")

    assert controller.tick() == (0.0, 0.0, 0.60)
    assert controller.last_reason == "align_left"


def test_visible_status_always_preempts_search_then_aligns_and_goes_forward():
    sink = FakeMotionSink()
    controller = CoyoteMotionController(
        sink,
        wall_clock=lambda: 100.0,
        monotonic_clock=lambda: 10.0,
    )
    prime_sensors(controller)
    controller.start_search("search-then-visible")
    assert controller.tick() == (0.0, 0.0, -1.35)

    controller.handle_status(status_payload(motion="stop", side="left"))
    assert sink.commands[-1] == (0.0, 0.0, 0.0)
    assert controller.tick() == (0.0, 0.0, 0.60)
    assert controller.last_reason == "align_left"

    controller.handle_status(status_payload(motion="forward", side="center"))
    assert controller.tick() == (1.00, 0.0, 0.0)
    assert controller.last_reason == "forward"


def test_detected_stop_sends_zero_before_turning_toward_side():
    monotonic = [10.0]
    sink = FakeMotionSink()
    controller = CoyoteMotionController(
        sink,
        wall_clock=lambda: 100.0,
        monotonic_clock=lambda: monotonic[0],
    )
    prime_sensors(controller)

    controller.start_search("side-event")
    controller.handle_status(status_payload(motion="stop", side="left"))
    assert sink.commands[-1] == (0.0, 0.0, 0.0)
    assert controller.tick() == (0.0, 0.0, 0.60)

    controller.handle_status(status_payload(motion="stop", side="right"))
    assert sink.commands[-1] == (0.0, 0.0, 0.0)
    assert controller.tick() == (0.0, 0.0, -0.60)


def test_side_turn_is_pulsed_with_a_stop_pause():
    monotonic = [10.0]
    sink = FakeMotionSink()
    controller = CoyoteMotionController(
        sink,
        wall_clock=lambda: 100.0,
        monotonic_clock=lambda: monotonic[0],
    )
    prime_sensors(controller)
    controller.start_search("pulse-event")
    controller.handle_status(status_payload(motion="stop", side="left"))

    assert controller.tick() == (0.0, 0.0, 0.60)
    monotonic[0] = 10.26
    assert controller.tick() == (0.0, 0.0, 0.0)
    monotonic[0] = 10.36
    assert controller.tick() == (0.0, 0.0, 0.60)


def test_detected_stop_center_holds_and_releases_motion_output():
    sink = FakeMotionSink()
    controller = CoyoteMotionController(
        sink,
        wall_clock=lambda: 100.0,
        monotonic_clock=lambda: 10.0,
    )
    prime_sensors(controller)

    controller.start_search("center-event")
    controller.handle_status(status_payload(motion="stop", side="center"))
    assert controller.tick() == (0.0, 0.0, 0.0)
    assert controller.last_reason == "target_centered"
    assert sink.releases == 1


def test_not_detected_does_not_search_until_an_event_arms_it():
    sink = FakeMotionSink()
    controller = CoyoteMotionController(
        sink,
        wall_clock=lambda: 100.0,
        monotonic_clock=lambda: 10.0,
    )
    prime_sensors(controller)
    controller.handle_status(
        status_payload(detect="not_detected", motion="stop", side="none")
    )

    assert controller.tick() == (0.0, 0.0, 0.0)
    assert controller.last_reason == "not_detected"


def test_initial_event_searches_clockwise_and_deduplicates_event_id():
    sink = FakeMotionSink()
    controller = CoyoteMotionController(
        sink,
        wall_clock=lambda: 100.0,
        monotonic_clock=lambda: 10.0,
    )
    prime_sensors(controller)
    controller.handle_status(
        status_payload(detect="not_detected", motion="stop", side="none")
    )

    assert controller.start_search("event-1") is True
    assert sink.commands[-1] == (0.0, 0.0, 0.0)
    assert controller.tick() == (0.0, 0.0, -1.35)
    assert controller.last_reason == "search_clockwise"
    assert controller.start_search("event-1") is False


def test_distinct_events_are_coalesced_while_one_search_owns_motion():
    sink = FakeMotionSink()
    controller = CoyoteMotionController(
        sink,
        wall_clock=lambda: 100.0,
        monotonic_clock=lambda: 10.0,
    )

    assert controller.start_search("event-a") is True
    assert controller.start_search("event-b") is False
    assert controller.last_reason == "search_trigger_coalesced"
    assert sink.acquires == 1


def test_clockwise_blocked_reverses_to_counterclockwise():
    sink = FakeMotionSink()
    controller = CoyoteMotionController(
        sink,
        wall_clock=lambda: 100.0,
        monotonic_clock=lambda: 10.0,
    )
    prime_sensors(controller, right=0.2, left=2.0)
    controller.handle_status(
        status_payload(detect="not_detected", motion="stop", side="none")
    )
    controller.start_search("event-2")

    assert controller.tick() == (0.0, 0.0, 0.0)
    assert controller.last_reason == "search_reverse"
    assert controller.tick() == (0.0, 0.0, 1.35)
    assert controller.last_reason == "search_counterclockwise"


def test_clockwise_turn_checks_rear_leading_lidar_sector():
    sink = FakeMotionSink()
    controller = CoyoteMotionController(
        sink,
        wall_clock=lambda: 100.0,
        monotonic_clock=lambda: 10.0,
    )
    prime_sensors(controller, clockwise_rear=0.2)
    controller.handle_status(
        status_payload(detect="not_detected", motion="stop", side="none")
    )
    controller.start_search("rear-obstacle-event")

    assert controller.tick() == (0.0, 0.0, 0.0)
    assert controller.last_reason == "search_reverse"


def test_nav_motion_before_search_ownership_does_not_consume_scan_budget():
    ready = [False]
    sink = FakeMotionSink()
    controller = CoyoteMotionController(
        sink,
        search_sweep_rad=0.20,
        wall_clock=lambda: 100.0,
        monotonic_clock=lambda: 10.0,
        ready=lambda: ready[0],
    )
    prime_sensors(controller)
    controller.handle_status(
        status_payload(detect="not_detected", motion="stop", side="none")
    )
    controller.start_search("handoff-event")
    assert controller.tick() == (0.0, 0.0, 0.0)

    controller.update_odom(0.0, 0.0, -1.0)
    ready[0] = True
    assert controller.tick() == (0.0, 0.0, -1.35)
    controller.update_odom(0.0, 0.0, -1.10)
    assert controller.tick() == (0.0, 0.0, -1.35)
    assert controller.last_reason == "search_clockwise"


def test_search_advances_exactly_half_meter_then_scans_again():
    monotonic = [10.0]
    sink = FakeMotionSink()
    controller = CoyoteMotionController(
        sink,
        search_sweep_rad=0.20,
        wall_clock=lambda: 100.0,
        monotonic_clock=lambda: monotonic[0],
    )
    prime_sensors(controller)
    controller.handle_status(
        status_payload(detect="not_detected", motion="stop", side="none")
    )
    controller.start_search("event-3")
    assert controller.tick() == (0.0, 0.0, -1.35)

    controller.update_odom(0.0, 0.0, -0.21)
    assert controller.tick() == (0.0, 0.0, 0.0)
    assert controller.last_reason == "search_scan_complete"
    assert controller.tick() == (0.05, 0.0, 0.0)
    assert controller.last_reason == "search_advance"

    heading = -0.21
    controller.update_odom(
        0.49 * math.cos(heading),
        0.49 * math.sin(heading),
        heading,
    )
    assert controller.tick() == (0.05, 0.0, 0.0)
    controller.update_odom(
        0.50 * math.cos(heading),
        0.50 * math.sin(heading),
        heading,
    )
    assert controller.tick() == (0.0, 0.0, 0.0)
    assert controller.last_reason == "search_advance_complete"
    assert controller.tick() == (0.0, 0.0, -1.35)


def test_search_advance_stops_if_motion_drifts_sideways():
    sink = FakeMotionSink()
    controller = CoyoteMotionController(
        sink,
        search_sweep_rad=0.20,
        wall_clock=lambda: 100.0,
        monotonic_clock=lambda: 10.0,
    )
    prime_sensors(controller)
    controller.start_search("advance-drift-event")
    controller.tick()
    search_yaw = -0.21
    controller.update_odom(0.0, 0.0, search_yaw)
    controller.tick()
    assert controller.tick() == (0.05, 0.0, 0.0)

    lateral_distance = 0.11
    controller.update_odom(
        -math.sin(search_yaw) * lateral_distance,
        math.cos(search_yaw) * lateral_distance,
        search_yaw,
    )

    assert controller.tick() == (0.0, 0.0, 0.0)
    assert controller.last_reason == "search_advance_deviated"
    assert sink.releases >= 1


def test_search_never_advances_a_second_time_for_same_event():
    sink = FakeMotionSink()
    controller = CoyoteMotionController(
        sink,
        search_sweep_rad=0.20,
        wall_clock=lambda: 100.0,
        monotonic_clock=lambda: 10.0,
    )
    prime_sensors(controller)
    controller.handle_status(
        status_payload(detect="not_detected", motion="stop", side="none")
    )
    controller.start_search("event-4")
    controller.tick()
    controller.update_odom(0.0, 0.0, -0.21)
    controller.tick()
    controller.tick()
    heading = -0.21
    endpoint = (
        0.50 * math.cos(heading),
        0.50 * math.sin(heading),
    )
    controller.update_odom(endpoint[0], endpoint[1], heading)
    controller.tick()
    controller.tick()
    controller.update_odom(endpoint[0], endpoint[1], -0.42)

    assert controller.tick() == (0.0, 0.0, 0.0)
    assert controller.last_reason == "search_exhausted"
    assert sink.releases >= 1


def test_turn_and_forward_fail_closed_on_stale_or_blocked_lidar():
    monotonic = [10.0]
    sink = FakeMotionSink()
    controller = CoyoteMotionController(
        sink,
        wall_clock=lambda: 100.0,
        monotonic_clock=lambda: monotonic[0],
    )
    prime_sensors(controller, front=0.2)
    controller.start_search("blocked-forward-event")
    controller.handle_status(status_payload())
    assert controller.tick() == (0.0, 0.0, 0.0)
    assert controller.last_reason == "forward_blocked"

    controller.update_scan(*scan())
    monotonic[0] = 10.51
    assert controller.tick() == (0.0, 0.0, 0.0)
    assert controller.last_reason == "lidar_stale"


def test_detected_forward_and_side_alignment_require_fresh_odom():
    sink = FakeMotionSink()
    controller = CoyoteMotionController(
        sink,
        wall_clock=lambda: 100.0,
        monotonic_clock=lambda: 10.0,
    )
    controller.update_scan(*scan())
    controller.start_search("no-odom-event")

    controller.handle_status(status_payload())
    assert controller.tick() == (0.0, 0.0, 0.0)
    assert controller.last_reason == "odom_stale"

    controller.handle_status(status_payload(motion="stop", side="left"))
    assert controller.tick() == (0.0, 0.0, 0.0)
    assert controller.last_reason == "odom_stale"


def test_stuck_fresh_odom_trips_motion_progress_watchdog():
    wall = [100.0]
    monotonic = [10.0]
    sink = FakeMotionSink()
    controller = CoyoteMotionController(
        sink,
        progress_timeout_sec=0.50,
        wall_clock=lambda: wall[0],
        monotonic_clock=lambda: monotonic[0],
    )
    prime_sensors(controller)
    controller.start_search("stuck-turn-event")

    for step in range(5):
        elapsed = step * 0.15
        wall[0] = 100.0 + elapsed
        monotonic[0] = 10.0 + elapsed
        controller.handle_status(
            status_payload(
                ts=wall[0],
                detect="not_detected",
                motion="stop",
                side="none",
            )
        )
        controller.update_scan(*scan())
        controller.update_odom(0.0, 0.0, 0.0)
        command = controller.tick()

    assert command == (0.0, 0.0, 0.0)
    assert controller.last_reason == "motion_progress_timeout"
    assert sink.releases >= 1


def test_alignment_has_an_absolute_phase_deadline_even_with_progress():
    wall = [100.0]
    monotonic = [10.0]
    sink = FakeMotionSink()
    controller = CoyoteMotionController(
        sink,
        align_timeout_sec=0.30,
        progress_timeout_sec=1.0,
        wall_clock=lambda: wall[0],
        monotonic_clock=lambda: monotonic[0],
    )
    prime_sensors(controller)
    controller.start_search("align-timeout-event")
    controller.handle_status(status_payload(motion="stop", side="left"))
    assert controller.tick() == (0.0, 0.0, 0.60)

    monotonic[0] = 10.36
    wall[0] = 100.36
    controller.update_scan(*scan())
    controller.update_odom(0.0, 0.0, 0.10)
    controller.handle_status(
        status_payload(ts=wall[0], motion="stop", side="left")
    )
    assert controller.tick() == (0.0, 0.0, 0.0)
    assert controller.last_reason == "motion_phase_timeout"


def test_search_session_deadline_survives_status_phase_changes():
    wall = [100.0]
    monotonic = [10.0]
    sink = FakeMotionSink()
    controller = CoyoteMotionController(
        sink,
        search_session_timeout_sec=0.30,
        align_timeout_sec=1.0,
        progress_timeout_sec=1.0,
        wall_clock=lambda: wall[0],
        monotonic_clock=lambda: monotonic[0],
    )
    prime_sensors(controller)
    controller.start_search("session-timeout-event")
    controller.handle_status(status_payload(motion="stop", side="left"))
    assert controller.tick() == (0.0, 0.0, 0.60)

    monotonic[0] = 10.20
    wall[0] = 100.20
    prime_sensors(controller, yaw=0.10)
    controller.handle_status(
        status_payload(ts=wall[0], motion="stop", side="right")
    )
    assert controller.tick() == (0.0, 0.0, -0.60)

    monotonic[0] = 10.31
    wall[0] = 100.31
    prime_sensors(controller, yaw=0.20)
    controller.handle_status(
        status_payload(ts=wall[0], motion="stop", side="left")
    )
    assert controller.tick() == (0.0, 0.0, 0.0)
    assert controller.last_reason == "search_session_timeout"
    assert sink.releases >= 1


def test_emergency_stop_latches_until_reset():
    sink = FakeMotionSink()
    controller = CoyoteMotionController(
        sink,
        wall_clock=lambda: 100.0,
        monotonic_clock=lambda: 10.0,
    )
    prime_sensors(controller)
    assert controller.start_search("before-estop") is True

    controller.emergency_stop()
    assert controller.emergency_latched is True
    assert controller.start_search("while-estop") is False
    assert sink.commands[-1] == (0.0, 0.0, 0.0)

    controller.reset()
    assert controller.emergency_latched is False
    assert controller.start_search("after-reset") is True


def test_patrol_timestamp_order_cannot_clear_newer_emergency_stop():
    sink = FakeMotionSink()
    controller = CoyoteMotionController(
        sink,
        wall_clock=lambda: 100.0,
        monotonic_clock=lambda: 10.0,
    )

    assert controller.handle_patrol_command(
        PatrolAction.EMERGENCY_STOP,
        200,
    )
    assert controller.emergency_latched is True
    assert not controller.handle_patrol_command(PatrolAction.RESET, 199)
    assert controller.emergency_latched is True
    assert not controller.start_search("still-latched")

    assert controller.handle_patrol_command(PatrolAction.RESET, 201)
    assert controller.emergency_latched is False
    assert not controller.handle_patrol_command(PatrolAction.RESET, 201)
    assert controller.start_search("after-ordered-reset")


def test_glass_search_ignores_coyote_status_until_matching_status_arrives():
    sink = FakeMotionSink()
    controller = CoyoteMotionController(
        sink,
        wall_clock=lambda: 100.0,
        monotonic_clock=lambda: 10.0,
    )
    prime_sensors(controller)
    assert controller.start_search("glass-event", DetectionType.BROKEN_CUP)

    controller.handle_status(status_payload(), DetectionType.COYOTE)
    assert controller.tick() == (0.0, 0.0, -1.35)
    controller.handle_status(status_payload(), DetectionType.BROKEN_CUP)
    assert controller.tick() == (1.00, 0.0, 0.0)


def test_payload_age_and_receive_watchdog_both_stop():
    wall = [100.0]
    monotonic = [10.0]
    sink = FakeMotionSink()
    controller = CoyoteMotionController(
        sink,
        wall_clock=lambda: wall[0],
        monotonic_clock=lambda: monotonic[0],
    )
    prime_sensors(controller)

    controller.start_search("stale-event")
    controller.handle_status(status_payload())
    wall[0] = 101.01
    monotonic[0] = 11.01
    assert controller.tick() == (0.0, 0.0, 0.0)
    assert controller.last_reason == "stale_status"

    controller.update_scan(*scan())
    controller.update_odom(0.0, 0.0, 0.0)
    controller.handle_status(
        status_payload(
            ts=101.01,
            detect="not_detected",
            motion="stop",
            side="none",
        )
    )
    assert controller.tick() == (0.0, 0.0, 0.0)
    assert controller.last_reason == "not_detected"


def test_transport_not_ready_stops_even_with_fresh_forward_status():
    ready = [True]
    sink = FakeMotionSink()
    controller = CoyoteMotionController(
        sink,
        wall_clock=lambda: 100.0,
        monotonic_clock=lambda: 10.0,
        ready=lambda: ready[0],
    )
    prime_sensors(controller)
    controller.start_search("transport-event")
    controller.handle_status(status_payload())
    ready[0] = False

    assert controller.tick() == (0.0, 0.0, 0.0)
    assert controller.last_reason == "transport_not_ready"


def test_malformed_status_fails_closed_immediately():
    sink = FakeMotionSink()
    controller = CoyoteMotionController(
        sink,
        wall_clock=lambda: 100.0,
        monotonic_clock=lambda: 10.0,
    )

    with pytest.raises(ValueError):
        controller.handle_status("not-json")

    assert sink.commands[-1] == (0.0, 0.0, 0.0)


class FakePublisher:
    def __init__(self):
        self.calls = []

    def publish_image(self, detection_type, *, event_id, jpeg_bytes):
        self.calls.append(("image", detection_type, event_id, jpeg_bytes))

    def publish_video(
        self,
        detection_type,
        *,
        event_id,
        mp4_bytes,
        duration_ms=None,
    ):
        self.calls.append(
            ("video", detection_type, event_id, mp4_bytes, duration_ms)
        )


def test_spool_reader_publishes_image_then_video_once(tmp_path):
    spool = CoyoteMediaSpool(CoyoteSpoolConfig(tmp_path), clock_ms=lambda: 1000)
    event_id = spool.new_event_id()
    spool.write_image(event_id, JPEG)
    spool.write_video(event_id, MP4, duration_ms=5000)
    reader = CoyoteSpoolReader(tmp_path)
    publisher = FakePublisher()

    image = reader.claim_next()
    assert image is not None and image.kind == "image"
    reader.publish(image, publisher)
    reader.complete(image)
    video = reader.claim_next()
    assert video is not None and video.kind == "video"
    reader.publish(video, publisher)
    reader.complete(video)

    assert [call[0] for call in publisher.calls] == ["image", "video"]
    assert {call[2] for call in publisher.calls} == {event_id}
    assert publisher.calls[0][1] is DetectionType.COYOTE
    assert publisher.calls[1][-1] == 5000
    assert reader.claim_next() is None


def test_generation_failure_publishes_fail_then_allows_video(tmp_path):
    spool = CoyoteMediaSpool(CoyoteSpoolConfig(tmp_path), clock_ms=lambda: 1000)
    event_id = spool.new_event_id()
    spool.write_failure(event_id, "image", reason="jpeg failed")
    spool.write_video(event_id, MP4, duration_ms=5000)
    reader = CoyoteSpoolReader(tmp_path)
    publisher = FakePublisher()

    image = reader.claim_next()
    assert image is not None and image.kind == "image"
    reader.publish(image, publisher)
    reader.complete(image)
    video = reader.claim_next()

    assert publisher.calls[0] == (
        "image",
        DetectionType.COYOTE,
        event_id,
        None,
    )
    assert video is not None and video.kind == "video"


def test_malformed_status_is_forwarded_once_for_immediate_fail_close(tmp_path):
    status_path = tmp_path / "status.json"
    status_path.write_text("not-json")
    reader = CoyoteSpoolReader(tmp_path)

    assert reader.read_status_if_changed() == "not-json"
    assert reader.read_status_if_changed() is None


def test_part_files_and_previous_sending_claims_are_not_replayed(tmp_path):
    event_dir = tmp_path / "events" / "coyote-1-deadbeef"
    event_dir.mkdir(parents=True)
    (event_dir / "image.ready.json.part-deadbeef").write_text("{}")
    (event_dir / "video.sending.json").write_text("{}")

    assert CoyoteSpoolReader(tmp_path).claim_next() is None


def test_media_worker_completes_claim_without_blocking_submit(tmp_path):
    spool = CoyoteMediaSpool(CoyoteSpoolConfig(tmp_path), clock_ms=lambda: 1000)
    event_id = spool.new_event_id()
    spool.write_image(event_id, JPEG)
    reader = CoyoteSpoolReader(tmp_path)
    publisher = FakePublisher()
    worker = CoyoteMediaWorker(reader, publisher)
    claim = reader.claim_next()

    assert claim is not None
    assert worker.submit(claim) is True
    worker.close()

    assert publisher.calls[0][0] == "image"
    assert (tmp_path / "events" / event_id / "image.published.json").exists()


def test_media_worker_survives_completion_failure_and_drains_next_claim(tmp_path):
    spool = CoyoteMediaSpool(CoyoteSpoolConfig(tmp_path), clock_ms=lambda: 1000)
    first_id = spool.new_event_id()
    second_id = spool.new_event_id()
    spool.write_image(first_id, JPEG)
    spool.write_image(second_id, JPEG)

    class OneCompletionFailureReader(CoyoteSpoolReader):
        def __init__(self, root):
            super().__init__(root)
            self.fail_once = True

        def complete(self, claim):
            if self.fail_once:
                self.fail_once = False
                raise OSError("simulated completion failure")
            return super().complete(claim)

    reader = OneCompletionFailureReader(tmp_path)
    publisher = FakePublisher()
    first = reader.claim_next()
    second = reader.claim_next()
    worker = CoyoteMediaWorker(reader, publisher)

    assert first is not None and second is not None
    assert worker.submit(first)
    assert worker.submit(second)
    worker.close()

    assert len(publisher.calls) == 2
    assert (tmp_path / "events" / first.event_id / "image.failed.json").exists()
    assert (tmp_path / "events" / second.event_id / "image.published.json").exists()
