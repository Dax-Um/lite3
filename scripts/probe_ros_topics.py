#!/usr/bin/env python3
"""Probe required ROS2 topics without sending any robot motion commands."""

import argparse
import csv
import sys
import time


class TopicStats:
    def __init__(self, topic: str):
        self.topic = topic
        self.count = 0
        self.first_seen: float | None = None
        self.last_seen: float | None = None

    def mark_seen(self, now: float) -> None:
        if self.first_seen is None:
            self.first_seen = now
        self.last_seen = now
        self.count += 1

    def hz(self) -> float:
        if self.count < 2 or self.first_seen is None or self.last_seen is None:
            return 0.0
        elapsed = self.last_seen - self.first_seen
        if elapsed <= 0.0:
            return 0.0
        return (self.count - 1) / elapsed

    def as_row(self, *, now: float, min_hz: float, stale_sec: float) -> dict[str, str]:
        if self.last_seen is None:
            return {
                "topic": self.topic,
                "seen": "false",
                "count": "0",
                "hz": "0.0",
                "last_age_sec": "",
                "status": "missing",
            }

        last_age = now - self.last_seen
        hz = self.hz()
        status = "ok"
        if last_age > stale_sec:
            status = "stale"
        elif hz < min_hz:
            status = "slow"

        return {
            "topic": self.topic,
            "seen": "true",
            "count": str(self.count),
            "hz": f"{hz:.1f}",
            "last_age_sec": f"{last_age:.2f}",
            "status": status,
        }


def exit_code_for_rows(rows: list[dict[str, str]]) -> int:
    return 0 if all(row["status"] == "ok" for row in rows) else 1


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Probe required ROS2 topics and report freshness as CSV."
    )
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--odom-topic", default="/leg_odom2")
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--duration-sec", type=float, default=5.0)
    parser.add_argument("--min-scan-hz", type=float, default=1.0)
    parser.add_argument("--min-odom-hz", type=float, default=1.0)
    parser.add_argument("--min-imu-hz", type=float, default=1.0)
    parser.add_argument("--stale-sec", type=float, default=0.5)
    return parser.parse_args(argv)


def validate_args(args: argparse.Namespace) -> None:
    if args.duration_sec <= 0.0:
        raise SystemExit("--duration-sec must be positive")
    if args.stale_sec <= 0.0:
        raise SystemExit("--stale-sec must be positive")
    for name in ("min_scan_hz", "min_odom_hz", "min_imu_hz"):
        if getattr(args, name) < 0.0:
            raise SystemExit(f"--{name.replace('_', '-')} must be non-negative")


def run_ros_probe(args: argparse.Namespace) -> int:
    try:
        import rclpy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu, LaserScan
    except ImportError as exc:
        print(f"ROS2 runtime unavailable: {exc}", file=sys.stderr)
        return 2

    stats = {
        args.scan_topic: TopicStats(args.scan_topic),
        args.odom_topic: TopicStats(args.odom_topic),
        args.imu_topic: TopicStats(args.imu_topic),
    }

    rclpy.init(args=None)
    node = rclpy.create_node("lite3_probe_ros_topics")
    try:
        node.create_subscription(
            LaserScan,
            args.scan_topic,
            lambda _msg: stats[args.scan_topic].mark_seen(time.monotonic()),
            10,
        )
        node.create_subscription(
            Odometry,
            args.odom_topic,
            lambda _msg: stats[args.odom_topic].mark_seen(time.monotonic()),
            10,
        )
        node.create_subscription(
            Imu,
            args.imu_topic,
            lambda _msg: stats[args.imu_topic].mark_seen(time.monotonic()),
            10,
        )

        end_time = time.monotonic() + args.duration_sec
        while time.monotonic() < end_time:
            rclpy.spin_once(node, timeout_sec=0.1)

        now = time.monotonic()
        rows = [
            stats[args.scan_topic].as_row(
                now=now,
                min_hz=args.min_scan_hz,
                stale_sec=args.stale_sec,
            ),
            stats[args.odom_topic].as_row(
                now=now,
                min_hz=args.min_odom_hz,
                stale_sec=args.stale_sec,
            ),
            stats[args.imu_topic].as_row(
                now=now,
                min_hz=args.min_imu_hz,
                stale_sec=args.stale_sec,
            ),
        ]
        write_rows(rows)
        return exit_code_for_rows(rows)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def write_rows(rows: list[dict[str, str]]) -> None:
    fieldnames = ["topic", "seen", "count", "hz", "last_age_sec", "status"]
    writer = csv.DictWriter(sys.stdout, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(rows)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    validate_args(args)
    return run_ros_probe(args)


if __name__ == "__main__":
    raise SystemExit(main())
