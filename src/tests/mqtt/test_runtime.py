import time
import threading

from lite3_mqtt.contract import DetectionType, Topics
from lite3_mqtt.runtime import Lite3MqttRuntime


class FakePatrol:
    def __init__(self):
        self.calls = []

    def start(self):
        self.calls.append("start")
        return True

    def stop(self):
        self.calls.append("stop")

    def prepare_motion(self):
        self.calls.append("prepare_motion")

    def return_home(self):
        self.calls.append("return_home")
        return True

    def emergency_stop(self):
        self.calls.append("emergency_stop")

    def reset(self):
        self.calls.append("reset")

    def close(self):
        self.calls.append("close")


class FakeDetectionPublisher:
    def __init__(self):
        self.calls = []

    def publish_detection(self, detection_type, *, event_id=None):
        self.calls.append((detection_type.value, event_id))


def test_runtime_routes_mqtt_commands_and_deduplicates_events():
    patrol = FakePatrol()
    detection = FakeDetectionPublisher()
    runtime = Lite3MqttRuntime(patrol=patrol, detection_publisher=detection)

    runtime.handle_message(
        Topics.AUTO_PATROL,
        b'{"timestamp":1783652400000,"action":"START"}',
    )
    sound = b'{"event_id":"sound-1","timestamp":1783652400001,"event_type":"GLASS_BROKEN"}'
    runtime.handle_message(Topics.SOUND_DETECT, sound)
    runtime.handle_message(Topics.SOUND_DETECT, sound)

    deadline = time.monotonic() + 1.0
    while not detection.calls and time.monotonic() < deadline:
        time.sleep(0.01)
    runtime.close()

    assert patrol.calls[0] == "start"
    assert patrol.calls.count("stop") == 2
    assert "prepare_motion" in patrol.calls
    assert detection.calls == [("BROKEN_CUP", "sound-1")]


def test_connection_loss_stops_active_patrol_without_latching_emergency():
    patrol = FakePatrol()
    runtime = Lite3MqttRuntime(
        patrol=patrol,
        detection_publisher=FakeDetectionPublisher(),
    )
    runtime.handle_connection_lost()
    runtime.close()
    assert patrol.calls[:1] == ["stop"]


def test_patrol_only_mode_ignores_detection_without_stopping_patrol():
    patrol = FakePatrol()
    detection = FakeDetectionPublisher()
    runtime = Lite3MqttRuntime(
        patrol=patrol,
        detection_publisher=detection,
        patrol_only=True,
    )

    runtime.handle_message(
        Topics.COYOTE_DETECT,
        b'{"event_id":"ignored","timestamp":1783652400001,'
        b'"event_type":"COYOTE_DETECTED"}',
    )
    runtime.close()

    assert patrol.calls == ["close"]
    assert detection.calls == []


def test_patrol_only_mode_does_not_block_repeated_test_timestamp():
    patrol = FakePatrol()
    runtime = Lite3MqttRuntime(
        patrol=patrol,
        detection_publisher=FakeDetectionPublisher(),
        patrol_only=True,
    )
    command = b'{"timestamp":1783652400000,"action":"START"}'

    runtime.handle_message(Topics.AUTO_PATROL, command)
    runtime.handle_message(Topics.AUTO_PATROL, command)
    runtime.close()

    assert patrol.calls.count("start") == 2


def test_production_trigger_stops_patrol_without_publishing_mock_media():
    patrol = FakePatrol()
    detection = FakeDetectionPublisher()
    runtime = Lite3MqttRuntime(
        patrol=patrol,
        detection_publisher=detection,
        publish_trigger_media=False,
    )

    runtime.handle_message(
        Topics.COYOTE_DETECT,
        b'{"event_id":"coyote-real","timestamp":1783652400001,'
        b'"event_type":"COYOTE_DETECTED"}',
    )
    runtime.close()

    assert patrol.calls[:1] == ["stop"]
    assert "prepare_motion" in patrol.calls
    assert detection.calls == []


def test_detection_queue_is_bounded():
    class BlockingPublisher:
        def __init__(self):
            self.started = threading.Event()
            self.release = threading.Event()

        def publish_detection(self, detection_type, *, event_id=None):
            self.started.set()
            self.release.wait(timeout=1.0)

    publisher = BlockingPublisher()
    runtime = Lite3MqttRuntime(
        patrol=FakePatrol(),
        detection_publisher=publisher,
        max_pending_detections=2,
    )
    assert runtime.report_detection(DetectionType.COYOTE, event_id="event-1")
    assert publisher.started.wait(timeout=1.0)
    assert runtime.report_detection(DetectionType.COYOTE, event_id="event-2")
    assert runtime.report_detection(DetectionType.COYOTE, event_id="event-3") is False
    publisher.release.set()
    runtime.close()


def test_failed_detection_can_be_retried_with_same_event_id():
    class FailingOncePublisher:
        def __init__(self):
            self.calls = 0

        def publish_detection(self, detection_type, *, event_id=None):
            self.calls += 1
            if self.calls == 1:
                raise RuntimeError("publish failed")

    publisher = FailingOncePublisher()
    runtime = Lite3MqttRuntime(
        patrol=FakePatrol(),
        detection_publisher=publisher,
    )
    assert runtime.report_detection(DetectionType.BROKEN_CUP, event_id="retry-me")
    deadline = time.monotonic() + 1.0
    while publisher.calls < 1 and time.monotonic() < deadline:
        time.sleep(0.01)
    deadline = time.monotonic() + 1.0
    retried = False
    while not retried and time.monotonic() < deadline:
        retried = runtime.report_detection(
            DetectionType.BROKEN_CUP,
            event_id="retry-me",
        )
        if not retried:
            time.sleep(0.01)
    runtime.close()
    assert retried
    assert publisher.calls == 2


def test_patrol_command_is_ignored_after_close():
    patrol = FakePatrol()
    runtime = Lite3MqttRuntime(
        patrol=patrol,
        detection_publisher=FakeDetectionPublisher(),
    )
    runtime.close()
    runtime.handle_message(
        Topics.AUTO_PATROL,
        b'{"timestamp":1783652400000,"action":"START"}',
    )
    assert "start" not in patrol.calls


def test_old_start_cannot_override_newer_stop():
    patrol = FakePatrol()
    runtime = Lite3MqttRuntime(
        patrol=patrol,
        detection_publisher=FakeDetectionPublisher(),
    )
    runtime.handle_message(
        Topics.AUTO_PATROL,
        b'{"timestamp":1783652400100,"action":"STOP"}',
    )
    runtime.handle_message(
        Topics.AUTO_PATROL,
        b'{"timestamp":1783652400000,"action":"START"}',
    )
    runtime.close()
    assert "stop" in patrol.calls
    assert "start" not in patrol.calls


def test_exact_duplicate_patrol_command_is_idempotent():
    patrol = FakePatrol()
    runtime = Lite3MqttRuntime(
        patrol=patrol,
        detection_publisher=FakeDetectionPublisher(),
    )
    command = b'{"timestamp":1783652400000,"action":"START"}'
    runtime.handle_message(Topics.AUTO_PATROL, command)
    runtime.handle_message(Topics.AUTO_PATROL, command)
    runtime.close()
    assert patrol.calls.count("start") == 1


def test_failed_patrol_handler_can_retry_the_exact_command():
    class FailingOncePatrol(FakePatrol):
        def __init__(self):
            super().__init__()
            self.attempts = 0

        def start(self):
            self.attempts += 1
            if self.attempts == 1:
                raise RuntimeError("temporary start failure")
            return super().start()

    patrol = FailingOncePatrol()
    runtime = Lite3MqttRuntime(
        patrol=patrol,
        detection_publisher=FakeDetectionPublisher(),
    )
    command = b'{"timestamp":1783652400000,"action":"START"}'

    runtime.handle_message(Topics.AUTO_PATROL, command)
    runtime.handle_message(Topics.AUTO_PATROL, command)
    runtime.close()

    assert patrol.attempts == 2
    assert patrol.calls.count("start") == 1


def test_old_safety_commands_are_never_blocked_by_timestamp_ordering():
    patrol = FakePatrol()
    runtime = Lite3MqttRuntime(
        patrol=patrol,
        detection_publisher=FakeDetectionPublisher(),
    )
    runtime.handle_message(
        Topics.AUTO_PATROL,
        b'{"timestamp":1783652401000,"action":"START"}',
    )
    runtime.handle_message(
        Topics.AUTO_PATROL,
        b'{"timestamp":1783652400000,"action":"STOP"}',
    )
    runtime.handle_message(
        Topics.AUTO_PATROL,
        b'{"timestamp":1783652399000,"action":"EMERGENCY_STOP"}',
    )
    runtime.close()

    assert patrol.calls[:3] == ["start", "stop", "emergency_stop"]


def test_old_reset_cannot_clear_a_newer_emergency_stop():
    patrol = FakePatrol()
    runtime = Lite3MqttRuntime(
        patrol=patrol,
        detection_publisher=FakeDetectionPublisher(),
    )
    runtime.handle_message(
        Topics.AUTO_PATROL,
        b'{"timestamp":1783652401000,"action":"EMERGENCY_STOP"}',
    )
    runtime.handle_message(
        Topics.AUTO_PATROL,
        b'{"timestamp":1783652400000,"action":"RESET"}',
    )
    runtime.close()

    assert "emergency_stop" in patrol.calls
    assert "reset" not in patrol.calls


def test_connection_loss_after_close_does_not_touch_closed_patrol():
    patrol = FakePatrol()
    runtime = Lite3MqttRuntime(
        patrol=patrol,
        detection_publisher=FakeDetectionPublisher(),
    )

    runtime.close()
    runtime.handle_connection_lost()

    assert "emergency_stop" not in patrol.calls


def test_close_drains_detection_worker_even_when_patrol_close_fails():
    class FailingClosePatrol(FakePatrol):
        def close(self):
            raise RuntimeError("close failed")

    publisher = FakeDetectionPublisher()
    runtime = Lite3MqttRuntime(
        patrol=FailingClosePatrol(),
        detection_publisher=publisher,
    )
    assert runtime.report_detection(DetectionType.COYOTE, event_id="event-before-close")

    try:
        runtime.close()
    except RuntimeError as exc:
        assert str(exc) == "close failed"
    else:
        raise AssertionError("patrol close error must still be reported")

    assert publisher.calls == [("COYOTE", "event-before-close")]
