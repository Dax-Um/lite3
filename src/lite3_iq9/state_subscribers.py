"""Mockable IQ9 topic freshness tracking.

This module does not import ROS2. A future rclpy adapter can call mark_seen()
from real subscriptions while unit tests feed deterministic timestamps.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class TopicFreshnessReport:
    ready: bool
    blocking_reasons: list[str]
    topics: dict[str, dict[str, bool | float | str | None]]


class TopicFreshnessMonitor:
    def __init__(self, required_topics: tuple[str, ...], stale_after_sec: float) -> None:
        if stale_after_sec <= 0.0:
            raise ValueError("stale_after_sec must be positive")
        self.required_topics = tuple(required_topics)
        self.stale_after_sec = stale_after_sec
        self._last_seen: dict[str, float] = {}

    def mark_seen(self, topic: str, stamp_sec: float) -> None:
        self._last_seen[topic] = stamp_sec

    def snapshot(self, now_sec: float) -> TopicFreshnessReport:
        topics: dict[str, dict[str, bool | float | str | None]] = {}
        reasons: list[str] = []

        for topic in self.required_topics:
            last_seen = self._last_seen.get(topic)
            if last_seen is None:
                topics[topic] = {
                    "seen": False,
                    "last_seen_sec": None,
                    "age_sec": None,
                    "status": "missing",
                }
                reasons.append(f"{topic}_missing")
                continue

            age_sec = round(now_sec - last_seen, 6)
            status = "ok"
            if age_sec > self.stale_after_sec:
                status = "stale"
                reasons.append(f"{topic}_stale")

            topics[topic] = {
                "seen": True,
                "last_seen_sec": last_seen,
                "age_sec": age_sec,
                "status": status,
            }

        return TopicFreshnessReport(
            ready=not reasons,
            blocking_reasons=reasons,
            topics=topics,
        )
