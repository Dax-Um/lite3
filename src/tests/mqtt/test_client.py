import logging
import threading
from types import SimpleNamespace

from lite3_mqtt.client import MqttConfig, PahoMqttClient


def _client(callback=lambda topic, payload: None):
    client = object.__new__(PahoMqttClient)
    client.config = MqttConfig()
    client._message_callback = callback
    client._connection_lost_callback = None
    client._logger = logging.getLogger("test-mqtt-client")
    client._connected = threading.Event()
    client._stopping = threading.Event()
    client._subscription_mid = None
    return client


def test_retained_commands_are_not_dispatched():
    received = []
    client = _client(lambda topic, payload: received.append((topic, payload)))
    message = SimpleNamespace(topic="/lite3/data/auto_patrol", payload=b"{}", retain=True)
    client._on_message(None, None, message)
    assert received == []


def test_unexpected_disconnect_calls_safety_callback():
    called = []
    client = _client()
    client._connection_lost_callback = lambda: called.append("lost")
    client._connected.set()
    client._on_disconnect(None, None, None, 1, None)
    assert not client._connected.is_set()
    assert called == ["lost"]


def test_suback_marks_client_ready_only_for_expected_mid():
    client = _client()
    client._subscription_mid = 7
    client._on_subscribe(None, None, 6, [0, 0, 0], None)
    assert not client._connected.is_set()
    client._on_subscribe(None, None, 7, [0, 0, 0], None)
    assert client._connected.is_set()


def test_publish_waits_for_local_socket_send_completion():
    class Info:
        rc = 0

        def __init__(self):
            self.wait_timeout = None

        def wait_for_publish(self, timeout):
            self.wait_timeout = timeout

        def is_published(self):
            return True

    info = Info()

    class Inner:
        def publish(self, topic, body, qos, retain):
            assert qos == 0
            assert retain is False
            return info

    client = _client()
    client._client = Inner()
    client._connected.set()
    client.publish_json("/aicenter/data/test", {"ok": True})
    assert info.wait_timeout == client.config.publish_timeout_sec
