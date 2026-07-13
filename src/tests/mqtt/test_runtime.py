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

    assert patrol.calls[:1] == ["start"]
    assert detection.calls == [("BROKEN_CUP", "sound-1")]


def test_connection_loss_latches_emergency_stop():
    patrol = FakePatrol()
    runtime = Lite3MqttRuntime(
        patrol=patrol,
        detection_publisher=FakeDetectionPublisher(),
    )
    runtime.handle_connection_lost()
    runtime.close()
    assert patrol.calls[:1] == ["emergency_stop"]


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
