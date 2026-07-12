"""Small paho-mqtt adapter pinned to MQTT 3.1.1 and QoS 0."""

from __future__ import annotations

import logging
import threading
from dataclasses import dataclass
from typing import Callable, Optional

from lite3_mqtt.contract import Topics, compact_json


MessageCallback = Callable[[str, bytes], None]


@dataclass(frozen=True)
class MqttConfig:
    host: str = "127.0.0.1"
    port: int = 1883
    client_id: str = "lite3-runtime"
    keepalive_sec: int = 30
    connect_timeout_sec: float = 10.0
    username: Optional[str] = None
    password: Optional[str] = None
    max_payload_bytes: int = 50 * 1024 * 1024


class PahoMqttClient:
    def __init__(
        self,
        config: MqttConfig,
        *,
        on_message: MessageCallback,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        self.config = config
        self._message_callback = on_message
        self._logger = logger or logging.getLogger(__name__)
        self._connected = threading.Event()
        self._client = self._build_client()

    def _build_client(self):
        try:
            import paho.mqtt.client as mqtt
        except ImportError as exc:
            raise RuntimeError(
                "paho-mqtt is required; install the project dependencies first"
            ) from exc

        kwargs = {
            "client_id": self.config.client_id,
            "clean_session": True,
            "protocol": mqtt.MQTTv311,
        }
        client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            **kwargs,
        )
        if self.config.username:
            client.username_pw_set(self.config.username, self.config.password)
        client.on_connect = self._on_connect
        client.on_disconnect = self._on_disconnect
        client.on_message = self._on_message
        return client

    def start(self) -> None:
        self._client.connect(
            self.config.host,
            port=self.config.port,
            keepalive=self.config.keepalive_sec,
        )
        self._client.loop_start()
        if not self._connected.wait(timeout=self.config.connect_timeout_sec):
            self._client.loop_stop()
            raise TimeoutError(
                f"timed out connecting to MQTT broker {self.config.host}:{self.config.port}"
            )

    def stop(self) -> None:
        try:
            self._client.disconnect()
        finally:
            self._client.loop_stop()
            self._connected.clear()

    def publish_json(self, topic: str, payload: dict) -> None:
        body = compact_json(payload).encode("utf-8")
        if len(body) > self.config.max_payload_bytes:
            raise ValueError(
                f"MQTT payload is {len(body)} bytes; limit is {self.config.max_payload_bytes}"
            )
        info = self._client.publish(topic, body, qos=0, retain=False)
        if info.rc != 0:
            raise RuntimeError(f"MQTT publish failed topic={topic} rc={info.rc}")

    def _on_connect(self, client, userdata, flags, reason_code, properties) -> None:
        _ = userdata, flags, properties
        if reason_code != 0:
            self._logger.error("MQTT connect rejected rc=%s", reason_code)
            return
        for topic in Topics.SUBSCRIPTIONS:
            client.subscribe(topic, qos=0)
        self._connected.set()
        self._logger.info("MQTT connected and subscribed to %s", Topics.SUBSCRIPTIONS)

    def _on_disconnect(
        self,
        client,
        userdata,
        disconnect_flags,
        reason_code,
        properties,
    ) -> None:
        _ = client, userdata, disconnect_flags, properties
        self._connected.clear()
        if reason_code != 0:
            self._logger.warning("unexpected MQTT disconnect rc=%s", reason_code)

    def _on_message(self, client, userdata, message) -> None:
        _ = client, userdata
        self._message_callback(str(message.topic), bytes(message.payload))
