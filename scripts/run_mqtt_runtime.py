#!/usr/bin/env python3
"""Run the Lite3 MQTT patrol and mock-detection pipeline."""

from __future__ import annotations

import argparse
import logging
import os
import signal
import sys
import threading
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
sys.path.insert(0, str(SRC))

from lite3_mqtt.client import MqttConfig, PahoMqttClient  # noqa: E402
from lite3_mqtt.contract import Topics, parse_detection_trigger  # noqa: E402
from lite3_mqtt.media import DetectionMediaPublisher, MockAnnotatedMediaSource  # noqa: E402
from lite3_mqtt.direct_patrol import (  # noqa: E402
    DirectMockPatrolBackend,
    DirectNav2PatrolBackend,
    DirectPatrolController,
)
from lite3_mqtt.patrol import (  # noqa: E402
    Waypoint,
)
from lite3_mqtt.runtime import Lite3MqttRuntime  # noqa: E402


DEFAULT_PATROL_CONFIG = ROOT / "configs" / "routes" / "mqtt_triangle_patrol.yaml"


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--broker-host", default=os.environ.get("MQTT_HOST", "127.0.0.1"))
    parser.add_argument(
        "--broker-port",
        type=int,
        default=int(os.environ.get("MQTT_PORT", "1883")),
    )
    parser.add_argument("--client-id", default="lite3-runtime")
    parser.add_argument("--username", default=os.environ.get("MQTT_USER") or None)
    parser.add_argument("--password", default=os.environ.get("MQTT_PASS") or None)
    parser.add_argument("--patrol-config", default=str(DEFAULT_PATROL_CONFIG))
    parser.add_argument("--patrol-backend", choices=("mock", "nav2"), default="mock")
    parser.add_argument("--allow-robot-motion", action="store_true")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--action-name", default="/FollowWaypoints")
    parser.add_argument("--nav-timeout-sec", type=float, default=10.0)
    parser.add_argument("--nav-route-timeout-sec", type=float, default=300.0)
    parser.add_argument("--nav-cancel-timeout-sec", type=float, default=5.0)
    parser.add_argument("--mock-home", nargs=3, type=float, default=(0.0, 0.0, 0.0))
    parser.add_argument("--mock-route-duration-sec", type=float, default=0.2)
    parser.add_argument(
        "--max-patrol-loops",
        type=int,
        default=0,
        help="0 repeats forever; use 1 for the first physical movement test.",
    )
    parser.add_argument("--video-duration-ms", type=int, default=5000)
    parser.add_argument(
        "--mock-detection-media",
        action="store_true",
        help="publish generated test image/video directly from MQTT triggers",
    )
    parser.add_argument("--run-seconds", type=float, default=0.0)
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper()),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    logger = logging.getLogger("lite3-mqtt")
    if args.patrol_backend == "nav2" and not args.allow_robot_motion:
        print(
            "refusing Nav2 MQTT runtime: --patrol-backend nav2 requires --allow-robot-motion",
            file=sys.stderr,
        )
        return 3

    if args.patrol_backend == "nav2":
        backend = DirectNav2PatrolBackend(
            odom_topic=args.odom_topic,
            action_name=args.action_name,
            timeout_sec=args.nav_timeout_sec,
            route_timeout_sec=args.nav_route_timeout_sec,
            cancel_timeout_sec=args.nav_cancel_timeout_sec,
        )
    else:
        x, y, yaw = args.mock_home
        backend = DirectMockPatrolBackend(
            home=Waypoint(id="home", x=x, y=y, yaw=yaw),
            route_duration_sec=args.mock_route_duration_sec,
        )

    patrol = DirectPatrolController(
        backend=backend,
        patrol_config=args.patrol_config,
        max_loops=args.max_patrol_loops,
        logger=logger,
    )
    stop = threading.Event()
    runtime_holder = {}

    def on_message(topic: str, payload: bytes) -> None:
        if topic in {Topics.SOUND_DETECT, Topics.COYOTE_DETECT}:
            trigger = parse_detection_trigger(topic, payload)
            # The patrol runtime owns the active Nav2 goal.  Relinquish it as
            # soon as a detection event arrives so the coyote bridge can take
            # over /cmd_vel after Nav2 becomes idle.
            patrol.stop()
            logger.info(
                "patrol stopped for detection event_id=%s type=%s",
                trigger.event_id,
                trigger.detection_type.value,
            )
            return
        runtime_holder["runtime"].handle_message(topic, payload)

    def on_connection_lost() -> None:
        runtime = runtime_holder.get("runtime")
        if runtime is not None:
            runtime.handle_connection_lost()

    client = PahoMqttClient(
        MqttConfig(
            host=args.broker_host,
            port=args.broker_port,
            client_id=args.client_id,
            username=args.username,
            password=args.password,
            subscriptions=(
                Topics.AUTO_PATROL,
                Topics.SOUND_DETECT,
                Topics.COYOTE_DETECT,
            ),
        ),
        on_message=on_message,
        on_connection_lost=on_connection_lost,
        logger=logger,
    )
    media_publisher = DetectionMediaPublisher(
        media_source=MockAnnotatedMediaSource(),
        publish_json=client.publish_json,
        duration_ms=args.video_duration_ms,
        logger=logger,
    )
    runtime = Lite3MqttRuntime(
        patrol=patrol,
        detection_publisher=media_publisher,
        publish_trigger_media=args.mock_detection_media,
        patrol_only=True,
        logger=logger,
    )
    runtime_holder["runtime"] = runtime

    def request_stop(signum=None, frame=None) -> None:
        _ = signum, frame
        stop.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    timer = None
    if args.run_seconds > 0:
        timer = threading.Timer(args.run_seconds, request_stop)
        timer.daemon = True
        timer.start()

    try:
        client.start()
        logger.info(
            "runtime ready broker=%s:%s patrol_backend=%s",
            args.broker_host,
            args.broker_port,
            args.patrol_backend,
        )
        stop.wait()
    finally:
        if timer is not None:
            timer.cancel()
        try:
            runtime.close()
        finally:
            client.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
