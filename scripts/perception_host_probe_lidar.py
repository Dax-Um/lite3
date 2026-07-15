#!/usr/bin/env python3
"""Wait for one fresh, non-empty RoboSense PointCloud2 sample."""

from __future__ import annotations

import argparse
import math
import time


def validate_pointcloud(
    message,
    *,
    now_sec: float,
    expected_frame: str,
    max_age_sec: float,
) -> str:
    frame_id = str(message.header.frame_id).strip("/")
    if frame_id != expected_frame.strip("/"):
        return "frame {!r} != {!r}".format(message.header.frame_id, expected_frame)
    stamp_sec = float(message.header.stamp.sec) + (
        float(message.header.stamp.nanosec) / 1_000_000_000.0
    )
    if not math.isfinite(stamp_sec) or stamp_sec <= 0.0:
        return "timestamp is invalid"
    age_sec = abs(float(now_sec) - stamp_sec)
    if age_sec > max_age_sec:
        return "sample age {:.3f}s exceeds {:.3f}s".format(age_sec, max_age_sec)
    width = int(message.width)
    height = int(message.height)
    point_step = int(message.point_step)
    row_step = int(message.row_step)
    if width <= 0 or height <= 0 or point_step <= 0 or row_step <= 0:
        return "point cloud dimensions are empty"
    if row_step < point_step * width:
        return "row_step is smaller than point_step * width"
    field_names = {str(field.name) for field in message.fields}
    if not {"x", "y", "z"}.issubset(field_names):
        return "point cloud is missing x/y/z fields"
    minimum_bytes = row_step * height
    if len(message.data) < minimum_bytes:
        return "point cloud data is truncated: {} < {}".format(
            len(message.data), minimum_bytes
        )
    return ""


def wait_for_fresh_pointcloud(
    *,
    timeout_sec: float,
    expected_frame: str,
    max_age_sec: float,
) -> bool:
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2

    rclpy.init(args=None)
    node = rclpy.create_node("lite3_rslidar_freshness_probe")
    state = {"ready": False, "reason": "no sample received"}
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.VOLATILE

    def on_message(message) -> None:
        reason = validate_pointcloud(
            message,
            now_sec=node.get_clock().now().nanoseconds / 1_000_000_000.0,
            expected_frame=expected_frame,
            max_age_sec=max_age_sec,
        )
        state["reason"] = reason
        state["ready"] = not reason

    subscription = node.create_subscription(PointCloud2, "/rslidar_points", on_message, qos)
    deadline = time.monotonic() + timeout_sec
    try:
        while rclpy.ok() and not state["ready"] and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if state["ready"]:
            print("fresh /rslidar_points sample received")
            return True
        print("/rslidar_points not ready: {}".format(state["reason"]))
        print(
            "power-on verification required: inspect the actual /rslidar_points "
            "header.frame_id and header.stamp against the perception-host clock "
            "(expected frame={!r}, max age={:.3f}s)".format(
                expected_frame, max_age_sec
            )
        )
        return False
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout-sec", type=float, default=15.0)
    parser.add_argument("--max-age-sec", type=float, default=5.0)
    parser.add_argument("--expected-frame", default="rslidar")
    args = parser.parse_args(argv)
    if (
        not math.isfinite(args.timeout_sec)
        or not math.isfinite(args.max_age_sec)
        or args.timeout_sec <= 0.0
        or args.max_age_sec <= 0.0
    ):
        parser.error("timeout values must be positive")
    if not args.expected_frame.strip("/"):
        parser.error("expected frame must be non-empty")
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    return 0 if wait_for_fresh_pointcloud(
        timeout_sec=args.timeout_sec,
        expected_frame=args.expected_frame,
        max_age_sec=args.max_age_sec,
    ) else 1


if __name__ == "__main__":
    raise SystemExit(main())
