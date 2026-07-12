#!/usr/bin/env python3
"""Publish sample Lite3 commands/triggers or run an end-to-end scenario."""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from lite3_mqtt.contract import Topics, epoch_ms  # noqa: E402


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "command",
        choices=("patrol-start", "patrol-stop", "sound", "coyote", "scenario", "listen"),
    )
    parser.add_argument("--broker-host", default="127.0.0.1")
    parser.add_argument("--broker-port", type=int, default=1883)
    parser.add_argument("--timeout-sec", type=float, default=15.0)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    import paho.mqtt.client as mqtt

    client = mqtt.Client(
        callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
        client_id=f"lite3-sample-peer-{time.time_ns()}",
        clean_session=True,
        protocol=mqtt.MQTTv311,
    )

    connected = threading.Event()
    received = []
    receive_event = threading.Event()

    def on_connect(inner, userdata, flags, reason_code, properties):
        _ = userdata, flags, properties
        if reason_code == 0:
            inner.subscribe("/aicenter/data/#", qos=0)
            connected.set()

    def on_message(inner, userdata, message):
        _ = inner, userdata
        payload = json.loads(message.payload.decode("utf-8"))
        received.append((str(message.topic), payload))
        print(json.dumps({"topic": message.topic, "payload": _summary(payload)}))
        receive_event.set()

    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(args.broker_host, args.broker_port, keepalive=30)
    client.loop_start()
    try:
        if not connected.wait(timeout=5.0):
            raise TimeoutError("sample peer could not connect to broker")
        if args.command == "listen":
            time.sleep(args.timeout_sec)
            return 0
        if args.command == "scenario":
            return _scenario(client, received, receive_event, args.timeout_sec)
        topic, payload = _message(args.command)
        client.publish(topic, json.dumps(payload, separators=(",", ":")), qos=0, retain=False)
        print(json.dumps({"published": topic, "payload": payload}))
        return 0
    finally:
        client.disconnect()
        client.loop_stop()


def _scenario(client, received, receive_event, timeout_sec: float) -> int:
    for command in ("patrol-start", "sound", "coyote"):
        topic, payload = _message(command)
        client.publish(topic, json.dumps(payload, separators=(",", ":")), qos=0, retain=False)
    expected = {
        Topics.BROKEN_CUP_IMAGE,
        Topics.BROKEN_CUP_VIDEO,
        Topics.COYOTE_IMAGE,
        Topics.COYOTE_VIDEO,
    }
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if expected.issubset({topic for topic, _ in received}):
            break
        receive_event.clear()
        receive_event.wait(timeout=0.2)
    stop_topic, stop_payload = _message("patrol-stop")
    client.publish(stop_topic, json.dumps(stop_payload, separators=(",", ":")), qos=0, retain=False)

    topics = [topic for topic, _ in received]
    if not expected.issubset(set(topics)):
        print(json.dumps({"scenario": "FAIL", "missing_topics": sorted(expected - set(topics))}))
        return 1
    if topics.index(Topics.BROKEN_CUP_IMAGE) > topics.index(Topics.BROKEN_CUP_VIDEO):
        print(json.dumps({"scenario": "FAIL", "reason": "broken-cup video arrived first"}))
        return 1
    if topics.index(Topics.COYOTE_IMAGE) > topics.index(Topics.COYOTE_VIDEO):
        print(json.dumps({"scenario": "FAIL", "reason": "coyote video arrived first"}))
        return 1
    _validate_event_pair(received, Topics.BROKEN_CUP_IMAGE, Topics.BROKEN_CUP_VIDEO)
    _validate_event_pair(received, Topics.COYOTE_IMAGE, Topics.COYOTE_VIDEO)
    print(json.dumps({"scenario": "PASS", "received_topics": topics}))
    return 0


def _message(command):
    now = epoch_ms()
    if command == "patrol-start":
        return Topics.AUTO_PATROL, {"timestamp": now, "action": "START"}
    if command == "patrol-stop":
        return Topics.AUTO_PATROL, {"timestamp": now, "action": "STOP"}
    if command == "sound":
        return Topics.SOUND_DETECT, {
            "event_id": f"sound-{now}",
            "timestamp": now,
            "event_type": "GLASS_BROKEN",
        }
    if command == "coyote":
        return Topics.COYOTE_DETECT, {
            "event_id": f"coyote-{now}",
            "timestamp": now,
            "event_type": "COYOTE_DETECTED",
        }
    raise ValueError(command)


def _validate_event_pair(received, image_topic, video_topic):
    image_payload = next(payload for topic, payload in received if topic == image_topic)
    video_payload = next(payload for topic, payload in received if topic == video_topic)
    if image_payload["event_id"] != video_payload["event_id"]:
        raise RuntimeError(f"event_id mismatch for {image_topic} and {video_topic}")
    if image_payload["result"] != "SUCCESS" or video_payload["result"] != "SUCCESS":
        raise RuntimeError(f"media generation failed for event_id={image_payload['event_id']}")


def _summary(payload):
    summary = dict(payload)
    for field in ("image", "video"):
        media = summary.get(field)
        if isinstance(media, dict) and "data_base64" in media:
            media = dict(media)
            media["data_base64"] = f"<{len(media['data_base64'])} base64 chars>"
            summary[field] = media
    return summary


if __name__ == "__main__":
    raise SystemExit(main())
