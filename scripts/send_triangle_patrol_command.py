#!/usr/bin/env python3
"""Publish one Lite3 patrol command without probing any ROS service."""

from __future__ import annotations

import argparse
import json
import os
import time
import uuid

TOPIC = "/lite3/data/auto_patrol"


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("START", "STOP", "RETURN_HOME", "EMERGENCY_STOP", "RESET"))
    parser.add_argument("--broker-host", default=os.environ.get("MQTT_HOST", "127.0.0.1"))
    parser.add_argument("--broker-port", type=int, default=int(os.environ.get("MQTT_PORT", "1883")))
    parser.add_argument("--username", default=os.environ.get("MQTT_USER") or None)
    parser.add_argument("--password", default=os.environ.get("MQTT_PASS") or None)
    parser.add_argument("--timeout-sec", type=float, default=3.0)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    import paho.mqtt.client as mqtt

    payload = {
        "timestamp": int(time.time() * 1000),
        "action": args.action,
    }
    client = mqtt.Client(
        callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
        client_id="lite3-patrol-test-{}".format(uuid.uuid4().hex[:8]),
        protocol=mqtt.MQTTv311,
    )
    if args.username is not None:
        client.username_pw_set(args.username, args.password)
    client.connect(args.broker_host, args.broker_port, keepalive=15)
    client.loop_start()
    try:
        result = client.publish(
            TOPIC,
            json.dumps(payload, separators=(",", ":")),
            qos=0,
            retain=False,
        )
        deadline = time.monotonic() + args.timeout_sec
        while not result.is_published() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not result.is_published():
            raise TimeoutError("MQTT publish timed out")
    finally:
        client.disconnect()
        client.loop_stop()
    print(json.dumps({"topic": TOPIC, "payload": payload}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
