import logging
import sys
import threading
import types
from types import SimpleNamespace

from lite3_mqtt.client import MqttConfig, PahoMqttClient
from lite3_mqtt.contract import Topics


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


def test_reconnect_clears_ready_until_new_suback_and_resubscribes_qos_zero():
    subscriptions = []

    class Inner:
        def subscribe(self, requested):
            subscriptions.append(requested)
            return 0, len(subscriptions)

    client = _client()
    client._connected.set()

    client._on_connect(Inner(), None, None, 0, None)

    assert not client._connected.is_set()
    assert subscriptions == [[(topic, 0) for topic in Topics.SUBSCRIPTIONS]]
    client._on_subscribe(None, None, 1, [0, 0, 0], None)
    assert client._connected.is_set()

    client._on_connect(Inner(), None, None, 0, None)
    assert not client._connected.is_set()
    client._on_subscribe(None, None, 2, [0, 0, 0], None)
    assert client._connected.is_set()


def test_publisher_only_client_is_ready_after_connack_without_subscribe():
    class Inner:
        def subscribe(self, requested):
            raise AssertionError("publisher-only client must not subscribe")

    client = _client()
    client.config = MqttConfig(subscriptions=())

    client._on_connect(Inner(), None, None, 0, None)

    assert client.connected is True
    assert client._subscription_mid is None


def test_incomplete_suback_never_marks_client_ready():
    client = _client()
    client._subscription_mid = 7

    client._on_subscribe(None, None, 7, [0, 0], None)

    assert not client._connected.is_set()


def test_application_callback_error_does_not_escape_paho_network_callback():
    def fail(topic, payload):
        raise RuntimeError("bad application callback")

    client = _client(fail)
    message = SimpleNamespace(topic=Topics.AUTO_PATROL, payload=b"{}", retain=False)

    client._on_message(None, None, message)


def test_paho_client_is_constructed_as_mqtt_311_clean_session(monkeypatch):
    captured = {}

    class FakeClient:
        def __init__(self, **kwargs):
            captured.update(kwargs)

    fake_client_module = types.ModuleType("paho.mqtt.client")
    fake_client_module.MQTTv311 = 4
    fake_client_module.CallbackAPIVersion = SimpleNamespace(VERSION2=2)
    fake_client_module.Client = FakeClient
    fake_mqtt_package = types.ModuleType("paho.mqtt")
    fake_mqtt_package.client = fake_client_module
    fake_paho_package = types.ModuleType("paho")
    fake_paho_package.mqtt = fake_mqtt_package
    monkeypatch.setitem(sys.modules, "paho", fake_paho_package)
    monkeypatch.setitem(sys.modules, "paho.mqtt", fake_mqtt_package)
    monkeypatch.setitem(sys.modules, "paho.mqtt.client", fake_client_module)

    PahoMqttClient(MqttConfig(), on_message=lambda topic, payload: None)

    assert captured["protocol"] == fake_client_module.MQTTv311
    assert captured["clean_session"] is True
    assert captured["callback_api_version"] == 2


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
