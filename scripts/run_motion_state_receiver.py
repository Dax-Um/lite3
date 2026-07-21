#!/usr/bin/env python3
"""Publish direct Motion Host Robot State UDP data inside the IQ9 Foxy Docker."""

from __future__ import annotations

import argparse
import json
import logging
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from lite3_motion.state_receiver import MotionStateUdpReceiver  # noqa: E402


STATE_TOPIC = "/lite3/motion/state"
STATUS_TOPIC = "/lite3/motion/status"
BASIC_STATE_TOPIC = "/lite3/motion/robot_basic_state"
MOTION_STATE_TOPIC = "/lite3/motion/robot_motion_state"
YAW_TOPIC = "/lite3/motion/yaw_rad"


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind-host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=43897)
    parser.add_argument("--stale-after-sec", type=float, default=0.30)
    parser.add_argument("--log-interval-sec", type=float, default=2.0)
    return parser.parse_args(argv)


def _state_payload(state):
    return {
        "source": "motion_host_udp",
        "received_at_monotonic": state.received_at_monotonic,
        "robot_basic_state": state.robot_basic_state,
        "robot_gait_state": state.robot_gait_state,
        "robot_policy_state": state.robot_policy_state,
        "robot_motion_state": state.robot_motion_state,
        "rpy_deg": [state.roll_deg, state.pitch_deg, state.yaw_deg],
        "yaw_rad": state.yaw_rad,
        "yaw_vel_radps": state.yaw_vel_radps,
        "pos_world": [
            state.pos_world_x_m,
            state.pos_world_y_m,
            state.pos_world_yaw_rad,
        ],
        "vel_body": [
            state.vel_body_x_mps,
            state.vel_body_y_mps,
            state.vel_body_yaw_radps,
        ],
    }


def main(argv=None):
    args = parse_args(argv)
    if args.stale_after_sec <= 0.0 or args.log_interval_sec <= 0.0:
        raise SystemExit("stale/log intervals must be positive")
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64, Int32, String

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s lite3-motion-state: %(message)s",
    )
    logger = logging.getLogger("lite3-motion-state")
    rclpy.init(args=None)
    node = Node("lite3_motion_state_receiver")
    receiver = MotionStateUdpReceiver(bind_host=args.bind_host, port=args.port)
    state_pub = node.create_publisher(String, STATE_TOPIC, 10)
    status_pub = node.create_publisher(String, STATUS_TOPIC, 10)
    basic_pub = node.create_publisher(Int32, BASIC_STATE_TOPIC, 10)
    motion_pub = node.create_publisher(Int32, MOTION_STATE_TOPIC, 10)
    yaw_pub = node.create_publisher(Float64, YAW_TOPIC, 10)
    runtime = {
        "last_state_at": None,
        "last_log_at": 0.0,
        "last_basic": None,
        "last_gait": None,
        "last_policy": None,
        "last_motion": None,
        "packets": 0,
        "errors": 0,
    }

    def publish_status(reason):
        now = time.monotonic()
        age = math.inf if runtime["last_state_at"] is None else now - runtime["last_state_at"]
        message = String()
        message.data = json.dumps({
            "source": "motion_host_udp",
            "bind": "{}:{}".format(args.bind_host, args.port),
            "ready": age <= args.stale_after_sec,
            "age_sec": None if not math.isfinite(age) else round(age, 4),
            "packets": runtime["packets"],
            "parse_errors": runtime["errors"],
            "reason": reason,
        }, separators=(",", ":"))
        status_pub.publish(message)

    def poll():
        try:
            states = receiver.drain()
        except ValueError as exc:
            runtime["errors"] += 1
            logger.warning("MOTION_STATE RX rejected: %s", exc)
            states = []
        for state in states:
            runtime["packets"] += 1
            runtime["last_state_at"] = state.received_at_monotonic
            payload = _state_payload(state)
            message = String()
            message.data = json.dumps(payload, separators=(",", ":"))
            state_pub.publish(message)
            basic = Int32()
            basic.data = state.robot_basic_state
            basic_pub.publish(basic)
            motion = Int32()
            motion.data = state.robot_motion_state
            motion_pub.publish(motion)
            yaw = Float64()
            yaw.data = state.yaw_rad
            yaw_pub.publish(yaw)
            if (
                runtime["last_basic"] != state.robot_basic_state
                or runtime["last_gait"] != state.robot_gait_state
                or runtime["last_policy"] != state.robot_policy_state
                or runtime["last_motion"] != state.robot_motion_state
            ):
                runtime["last_basic"] = state.robot_basic_state
                runtime["last_gait"] = state.robot_gait_state
                runtime["last_policy"] = state.robot_policy_state
                runtime["last_motion"] = state.robot_motion_state
                logger.info(
                    "MOTION_STATE RX basic=%d gait=%d policy=%d motion=%d yaw=%.3f pos=(%.3f,%.3f) packets=%d",
                    state.robot_basic_state,
                    state.robot_gait_state,
                    state.robot_policy_state,
                    state.robot_motion_state,
                    state.yaw_rad,
                    state.pos_world_x_m, state.pos_world_y_m, runtime["packets"],
                )
        now = time.monotonic()
        if now - runtime["last_log_at"] >= args.log_interval_sec:
            runtime["last_log_at"] = now
            publish_status("receiving" if states else "waiting_or_stale")
            age = math.inf if runtime["last_state_at"] is None else now - runtime["last_state_at"]
            logger.info(
                "MOTION_STATE health ready=%s age=%s packets=%d errors=%d",
                age <= args.stale_after_sec,
                "none" if not math.isfinite(age) else "{:.3f}s".format(age),
                runtime["packets"], runtime["errors"],
            )

    node.create_timer(0.01, poll)
    logger.info("MOTION_STATE receiver listening bind=%s:%d", args.bind_host, args.port)
    try:
        rclpy.spin(node)
    finally:
        receiver.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
