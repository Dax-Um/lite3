#!/usr/bin/env python3
"""Run the Lite3 MQTT patrol and mock-detection pipeline."""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import threading
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
sys.path.insert(0, str(SRC))

from lite3_mqtt.client import MqttConfig, PahoMqttClient  # noqa: E402
from lite3_mqtt.media import DetectionMediaPublisher, MockAnnotatedMediaSource  # noqa: E402
from lite3_mqtt.perception_host import (  # noqa: E402
    PerceptionHostConfig,
    PerceptionHostNavManager,
    PerceptionHostStartupGate,
)
from lite3_mqtt.patrol import (  # noqa: E402
    ContinuousPatrolController,
    MockPatrolBackend,
    Nav2PatrolBackend,
    Waypoint,
)
from lite3_mqtt.runtime import Lite3MqttRuntime  # noqa: E402


DEFAULT_PATROL_CONFIG = ROOT / "configs" / "routes" / "mqtt_triangle_patrol.yaml"


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--broker-host", default="127.0.0.1")
    parser.add_argument("--broker-port", type=int, default=1883)
    parser.add_argument("--client-id", default="lite3-runtime")
    parser.add_argument("--username")
    parser.add_argument("--password")
    parser.add_argument("--patrol-config", default=str(DEFAULT_PATROL_CONFIG))
    parser.add_argument("--patrol-backend", choices=("mock", "nav2"), default="mock")
    parser.add_argument("--allow-robot-motion", action="store_true")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--action-name", default="/FollowWaypoints")
    parser.add_argument("--perception-host", default="192.168.1.103")
    parser.add_argument("--perception-user", default="ysc")
    parser.add_argument("--perception-remote-root", default="/home/ysc/lite3")
    parser.add_argument("--perception-connect-timeout-sec", type=float, default=5.0)
    parser.add_argument("--perception-ready-timeout-sec", type=float, default=90.0)
    parser.add_argument("--nav-data-stale-sec", type=float, default=2.0)
    parser.add_argument("--nav-route-timeout-sec", type=float, default=300.0)
    parser.add_argument("--nav-cancel-timeout-sec", type=float, default=5.0)
    parser.add_argument("--max-lateral-speed-mps", type=float, default=0.02)
    parser.add_argument("--no-auto-start-perception-nav", action="store_true")
    parser.add_argument("--mock-home", nargs=3, type=float, default=(0.0, 0.0, 0.0))
    parser.add_argument("--mock-route-duration-sec", type=float, default=0.2)
    parser.add_argument(
        "--max-patrol-loops",
        type=int,
        default=0,
        help="0 repeats forever; use 1 for the first physical movement test.",
    )
    parser.add_argument("--video-duration-ms", type=int, default=5000)
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
        backend = Nav2PatrolBackend(
            odom_topic=args.odom_topic,
            action_name=args.action_name,
            timeout_sec=args.perception_ready_timeout_sec,
            max_data_age_sec=args.nav_data_stale_sec,
            max_lateral_speed_mps=args.max_lateral_speed_mps,
            route_timeout_sec=args.nav_route_timeout_sec,
            cancel_timeout_sec=args.nav_cancel_timeout_sec,
        )
        manager = PerceptionHostNavManager(
            PerceptionHostConfig(
                host=args.perception_host,
                user=args.perception_user,
                remote_root=args.perception_remote_root,
                connect_timeout_sec=args.perception_connect_timeout_sec,
                ready_timeout_sec=args.perception_ready_timeout_sec,
                auto_start_navigation=not args.no_auto_start_perception_nav,
            ),
            logger=logger,
        )
        startup_gate = PerceptionHostStartupGate(manager, backend)
    else:
        x, y, yaw = args.mock_home
        backend = MockPatrolBackend(
            home=Waypoint(id="home", x=x, y=y, yaw=yaw),
            route_duration_sec=args.mock_route_duration_sec,
        )
        startup_gate = None

    patrol = ContinuousPatrolController(
        backend=backend,
        patrol_config=args.patrol_config,
        startup_gate=startup_gate,
        max_loops=args.max_patrol_loops,
        logger=logger,
    )
    stop = threading.Event()
    runtime_holder = {}

    def on_message(topic: str, payload: bytes) -> None:
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
        runtime.close()
        client.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
