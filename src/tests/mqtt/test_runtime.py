import time

from lite3_mqtt.contract import Topics
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
