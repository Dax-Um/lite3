#!/usr/bin/env python3
"""Run the Lite3 MQTT patrol and mock-detection pipeline."""

from __future__ import annotations

import argparse
import json
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
from lite3_mqtt.contract import (  # noqa: E402
    DetectionType,
    InternalRosTopics,
    Topics,
    parse_detection_trigger,
)
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


class InternalMissionEventSubscriber:
    """Receive private ROS mission transitions in a context separate from Nav2 calls."""

    def __init__(self, runtime: Lite3MqttRuntime, logger: logging.Logger) -> None:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from std_msgs.msg import String

        self._rclpy = rclpy
        self._runtime = runtime
        self._context = Context()
        rclpy.init(args=None, context=self._context)
        self._node = rclpy.create_node(
            "lite3_internal_mission_receiver",
            context=self._context,
        )
        self._node.create_subscription(
            String,
            InternalRosTopics.MISSION_EVENT,
            lambda message: runtime.handle_mission_event(
                message.data,
                on_coyote_home_reached=self.publish_coyote_home_reached,
            ),
            10,
        )
        self._mission_start_pub = self._node.create_publisher(
            String,
            InternalRosTopics.MISSION_START,
            10,
        )
        self._home_reached_pub = self._node.create_publisher(
            String,
            InternalRosTopics.MISSION_HOME_REACHED,
            10,
        )
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)
        self._thread = threading.Thread(
            target=self._executor.spin,
            name="lite3-internal-mission-receiver",
            daemon=True,
        )
        self._thread.start()
        logger.info("internal mission subscriber ready topic=%s", InternalRosTopics.MISSION_EVENT)

    def publish_coyote_start(self, event_id: str) -> None:
        from std_msgs.msg import String

        message = String()
        message.data = json.dumps(
            {
                "event_id": event_id,
                "source": "coyote",
                "requested_action": "START_SEARCH",
            },
            separators=(",", ":"),
        )
        self._mission_start_pub.publish(message)

    def publish_coyote_home_reached(self, event_id: str) -> None:
        from std_msgs.msg import String

        message = String()
        message.data = json.dumps(
            {"event_id": event_id, "source": "coyote", "state": "HOME_REACHED"},
            separators=(",", ":"),
        )
        self._home_reached_pub.publish(message)
        self._runtime.release_coyote_mission(event_id)

    def close(self) -> None:
        self._executor.shutdown()
        self._node.destroy_node()
        self._rclpy.shutdown(context=self._context)
        self._thread.join(timeout=2.0)


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
    parser.add_argument(
        "--mock-detection-media",
        action="store_true",
        help="publish a generated test image directly from MQTT triggers",
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
            if trigger.detection_type is DetectionType.COYOTE:
                runtime = runtime_holder["runtime"]
                if not runtime.reserve_coyote_mission(trigger.event_id):
                    logger.info(
                        "coyote mission trigger ignored while another mission is active event_id=%s",
                        trigger.event_id,
                    )
                    return
                if not patrol.capture_home():
                    runtime.release_coyote_mission(trigger.event_id)
                    logger.error(
                        "coyote mission rejected: unable to capture home event_id=%s",
                        trigger.event_id,
                    )
                    return
                logger.info(
                    "coyote mission home captured; IQ9 coyote search starts independently event_id=%s",
                    trigger.event_id,
                )
                return
            if trigger.detection_type is DetectionType.BROKEN_CUP:
                published = runtime_holder["runtime"].report_detection(
                    trigger.detection_type,
                    event_id=trigger.event_id,
                )
                logger.info(
                    "broken-cup image requested event_id=%s accepted=%s",
                    trigger.event_id,
                    published,
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
    mission_subscriber = InternalMissionEventSubscriber(runtime, logger)

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
            mission_subscriber.close()
        except Exception:
            logger.exception("internal mission subscriber cleanup failed")
        try:
            runtime.close()
        finally:
            client.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
