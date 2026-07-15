#!/usr/bin/env python3
"""Run the Foxy coyote ROS/spool to MQTT bridge."""

from __future__ import annotations

import argparse
import logging
import math
import os
import queue
import signal
import sys
import threading
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
sys.path.insert(0, str(SRC))

from lite3_common.config import (  # noqa: E402
    load_lite3_network_config,
    load_motion_limits_config,
)
from lite3_control.udp_driver import Lite3UdpDriver  # noqa: E402
from lite3_mqtt.client import MqttConfig, PahoMqttClient  # noqa: E402
from lite3_mqtt.contract import (  # noqa: E402
    PatrolAction,
    Topics,
    parse_detection_trigger,
    parse_patrol_command,
)
from lite3_mqtt.coyote_bridge import (  # noqa: E402
    COYOTE_CONTROL_HZ,
    COYOTE_FORWARD_SPEED_MPS,
    COYOTE_SEARCH_ADVANCE_M,
    COYOTE_SEARCH_TURN_SPEED_RADPS,
    COYOTE_STATUS_TIMEOUT_SEC,
    COYOTE_TURN_SPEED_RADPS,
    CoyoteMediaWorker,
    CoyoteMotionController,
    CoyoteSpoolReader,
)
from lite3_mqtt.media import DetectionMediaPublisher  # noqa: E402
from lite3_ros.coyote_bridge_rclpy_node import (  # noqa: E402
    CoyoteMqttBridgeNode,
    CoyoteMotionOutputNode,
    CoyoteSpoolAdapterNode,
)


DEFAULT_SPOOL_DIR = "/home/ubuntu/iq9_coyote/outputs/spool"


class DisabledMotionSink:
    def acquire(self) -> None:
        pass

    def send_cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        _ = vx, vy, wz

    def release(self) -> None:
        pass


def parse_args(argv=None) -> argparse.Namespace:
    network = load_lite3_network_config(ROOT)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--broker-host", default=os.environ.get("MQTT_HOST", "127.0.0.1"))
    parser.add_argument(
        "--broker-port",
        type=int,
        default=int(os.environ.get("MQTT_PORT", "1883")),
    )
    parser.add_argument("--client-id", default="lite3-coyote-media-bridge")
    parser.add_argument("--username", default=os.environ.get("MQTT_USER") or None)
    parser.add_argument("--password", default=os.environ.get("MQTT_PASS") or None)
    parser.add_argument(
        "--spool-dir",
        default=os.environ.get("COYOTE_SPOOL_DIR", DEFAULT_SPOOL_DIR),
    )
    parser.add_argument("--control-hz", type=float, default=COYOTE_CONTROL_HZ)
    parser.add_argument(
        "--status-timeout-sec",
        type=float,
        default=COYOTE_STATUS_TIMEOUT_SEC,
    )
    parser.add_argument(
        "--forward-speed-mps",
        type=float,
        default=None,
    )
    parser.add_argument(
        "--turn-speed-radps",
        type=float,
        default=None,
    )
    parser.add_argument(
        "--search-turn-speed-radps",
        type=float,
        default=None,
    )
    parser.add_argument(
        "--search-advance-m",
        type=float,
        default=COYOTE_SEARCH_ADVANCE_M,
    )
    parser.add_argument(
        "--motion-output",
        choices=("disabled", "ros", "udp"),
        default="disabled",
    )
    parser.add_argument("--motion-host", default=network.motion_host_ip)
    parser.add_argument("--motion-port", type=int, default=network.motion_host_command_port)
    parser.add_argument("--allow-robot-motion", action="store_true")
    parser.add_argument("--preflight-ok", action="store_true")
    parser.add_argument("--auto-mode-ok", action="store_true")
    parser.add_argument("--exclusive-motion-ok", action="store_true")
    parser.add_argument("--run-seconds", type=float, default=0.0)
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args(argv)
    limits = load_motion_limits_config(ROOT)
    if args.forward_speed_mps is None:
        # ROS tracking uses the requested production speed.  Keep the direct
        # UDP fallback at the shared conservative motion limit.
        args.forward_speed_mps = (
            COYOTE_FORWARD_SPEED_MPS
            if args.motion_output != "udp"
            else limits.max_vx_mps
        )
    if args.turn_speed_radps is None:
        args.turn_speed_radps = (
            COYOTE_TURN_SPEED_RADPS
            if args.motion_output != "udp"
            else limits.max_wz_radps
        )
    if args.search_turn_speed_radps is None:
        args.search_turn_speed_radps = (
            COYOTE_SEARCH_TURN_SPEED_RADPS
            if args.motion_output != "udp"
            else limits.max_wz_radps
        )
    return args


def validate_args(args: argparse.Namespace) -> None:
    limits = load_motion_limits_config(ROOT)
    forward_speed_limit = (
        COYOTE_FORWARD_SPEED_MPS
        if args.motion_output != "udp"
        else limits.max_vx_mps
    )
    turn_speed_limit = (
        COYOTE_TURN_SPEED_RADPS
        if args.motion_output != "udp"
        else limits.max_wz_radps
    )
    search_turn_speed_limit = (
        COYOTE_SEARCH_TURN_SPEED_RADPS
        if args.motion_output != "udp"
        else limits.max_wz_radps
    )
    if not math.isfinite(args.control_hz) or args.control_hz < 20.0:
        raise SystemExit("--control-hz must be finite and at least 20")
    if (
        not math.isfinite(args.status_timeout_sec)
        or args.status_timeout_sec <= 0.0
        or args.status_timeout_sec > 1.0
    ):
        raise SystemExit("--status-timeout-sec must be in (0, 1.0]")
    if (
        not math.isfinite(args.forward_speed_mps)
        or args.forward_speed_mps <= 0.0
        or args.forward_speed_mps > forward_speed_limit
    ):
        raise SystemExit(
            "--forward-speed-mps must be positive and no greater than {}".format(
                forward_speed_limit
            )
        )
    if (
        not math.isfinite(args.turn_speed_radps)
        or args.turn_speed_radps <= 0.0
        or args.turn_speed_radps > turn_speed_limit
    ):
        raise SystemExit(
            "--turn-speed-radps must be positive and no greater than {}".format(
                turn_speed_limit
            )
        )
    if (
        not math.isfinite(args.search_turn_speed_radps)
        or args.search_turn_speed_radps <= 0.0
        or args.search_turn_speed_radps > search_turn_speed_limit
    ):
        raise SystemExit(
            "--search-turn-speed-radps must be positive and no greater than "
            "{}".format(search_turn_speed_limit)
        )
    if (
        not math.isfinite(args.search_advance_m)
        or args.search_advance_m <= 0.0
        or args.search_advance_m > 0.50
    ):
        raise SystemExit("--search-advance-m must be in (0, 0.50]")
    if not math.isfinite(args.run_seconds) or args.run_seconds < 0.0:
        raise SystemExit("--run-seconds must be finite and non-negative")
    if args.motion_output == "ros" and not args.allow_robot_motion:
        raise SystemExit("ROS coyote motion requires --allow-robot-motion")
    if args.motion_output == "udp":
        missing = [
            flag
            for flag, enabled in (
                ("--allow-robot-motion", args.allow_robot_motion),
                ("--preflight-ok", args.preflight_ok),
                ("--auto-mode-ok", args.auto_mode_ok),
                ("--exclusive-motion-ok", args.exclusive_motion_ok),
            )
            if not enabled
        ]
        if missing:
            raise SystemExit(
                "coyote motion requires {}".format(", ".join(missing))
            )


def main(argv=None) -> int:
    args = parse_args(argv)
    validate_args(args)
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper()),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    logger = logging.getLogger("lite3-coyote-bridge")

    driver = None
    media_client = None
    bridge_node = None
    search_events = queue.Queue(maxsize=32)

    def on_trigger(topic: str, payload: bytes) -> None:
        if topic == Topics.AUTO_PATROL:
            command = parse_patrol_command(payload)
            if (
                command.action is PatrolAction.EMERGENCY_STOP
                and bridge_node is not None
            ):
                bridge_node.motion_controller.handle_patrol_command(
                    command.action,
                    command.timestamp,
                )
                return
            item = ("patrol", command.action.value, command.timestamp)
        else:
            trigger = parse_detection_trigger(topic, payload)
            item = (
                "search",
                trigger.event_id,
                trigger.detection_type.value,
            )
        try:
            search_events.put_nowait(item)
        except queue.Full:
            if topic == Topics.AUTO_PATROL and bridge_node is not None:
                # Never bypass FIFO with an enabling command. A full QoS-0
                # control queue fails closed and requires a newer RESET.
                bridge_node.motion_controller.handle_patrol_command(
                    PatrolAction.EMERGENCY_STOP,
                    command.timestamp,
                )
                logger.error(
                    "coyote control queue full; emergency latched action=%s",
                    command.action.value,
                )
            else:
                logger.error("detection search trigger dropped: queue full")

    media_client = PahoMqttClient(
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
        on_message=on_trigger,
        on_connection_lost=lambda: (
            bridge_node.motion_controller.emergency_stop()
            if bridge_node is not None
            else None
        ),
        logger=logger,
    )
    media_publisher = DetectionMediaPublisher(
        media_source=None,
        publish_json=media_client.publish_json,
        duration_ms=5000,
        logger=logger,
    )
    reader = CoyoteSpoolReader(args.spool_dir)
    worker = CoyoteMediaWorker(reader, media_publisher, logger=logger)
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    stop = threading.Event()
    timer = None
    adapter_node = None
    motion_output_node = None
    motion = None
    executor = None
    client_started = False

    def request_stop(signum=None, frame=None) -> None:
        _ = signum, frame
        stop.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    if args.run_seconds > 0.0:
        timer = threading.Timer(args.run_seconds, request_stop)
        timer.daemon = True
        timer.start()

    try:
        media_client.start()
        client_started = True
        rclpy.init(args=None)
        motion_output_node = CoyoteMotionOutputNode()
        if args.motion_output == "udp":
            driver = Lite3UdpDriver(
                args.motion_host,
                args.motion_port,
                load_motion_limits_config(ROOT),
            )
            motion_sink = driver
        elif args.motion_output == "ros":
            motion_sink = motion_output_node
        else:
            motion_sink = DisabledMotionSink()
        motion = CoyoteMotionController(
            motion_sink,
            forward_speed_mps=args.forward_speed_mps,
            turn_speed_radps=args.turn_speed_radps,
            search_turn_speed_radps=args.search_turn_speed_radps,
            search_advance_m=args.search_advance_m,
            timeout_sec=args.status_timeout_sec,
            ready=lambda: (
                media_client is not None
                and media_client.connected
                and (
                    args.motion_output == "disabled"
                    or (
                        motion_output_node is not None
                        and motion_output_node.motion_ready()
                    )
                )
            ),
            logger=logger,
        )
        motion_output_node.bind_controller(motion)
        adapter_node = CoyoteSpoolAdapterNode(
            reader=reader,
            media_ready=lambda: media_client.connected,
        )
        bridge_node = CoyoteMqttBridgeNode(
            reader=reader,
            media_worker=worker,
            motion_controller=motion,
            search_events=search_events,
            control_hz=args.control_hz,
        )
        executor = SingleThreadedExecutor()
        executor.add_node(motion_output_node)
        executor.add_node(adapter_node)
        executor.add_node(bridge_node)
        logger.info(
            "coyote bridge ready broker=%s:%s spool=%s motion=%s",
            args.broker_host,
            args.broker_port,
            args.spool_dir,
            args.motion_output,
        )
        while rclpy.ok() and not stop.is_set():
            executor.spin_once(timeout_sec=0.1)
    finally:
        if timer is not None:
            timer.cancel()
        cleanup_steps = []
        if executor is not None and motion_output_node is not None:
            cleanup_steps.append(
                (
                    "remove motion output node",
                    lambda: executor.remove_node(motion_output_node),
                )
            )
        if executor is not None and bridge_node is not None:
            cleanup_steps.append(
                ("remove bridge node", lambda: executor.remove_node(bridge_node))
            )
        if executor is not None and adapter_node is not None:
            cleanup_steps.append(
                ("remove adapter node", lambda: executor.remove_node(adapter_node))
            )
        if bridge_node is not None:
            cleanup_steps.append(("destroy bridge node", bridge_node.destroy_node))
        if adapter_node is not None:
            cleanup_steps.append(("destroy adapter node", adapter_node.destroy_node))
        if motion_output_node is not None:
            cleanup_steps.append(
                ("destroy motion output node", motion_output_node.destroy_node)
            )
        if executor is not None:
            cleanup_steps.append(("shutdown executor", executor.shutdown))
        if rclpy.ok():
            cleanup_steps.append(("shutdown rclpy", rclpy.shutdown))
        cleanup_steps.append(("close media worker", worker.close))
        if client_started:
            cleanup_steps.append(("stop MQTT client", media_client.stop))
        if driver is not None:
            cleanup_steps.append(
                ("send repeated final stop", lambda: driver.stop(10, 0.05))
            )
            cleanup_steps.append(("close motion driver", driver.close))
        for label, cleanup in cleanup_steps:
            try:
                cleanup()
            except Exception:
                logger.exception("cleanup failed: %s", label)
        stop.set()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
