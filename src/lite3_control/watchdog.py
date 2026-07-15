"""Command freshness watchdog."""

from __future__ import annotations

TIME_EPSILON = 1e-9


class CommandWatchdog:
    def __init__(self, timeout_sec: float):
        self.timeout_sec = timeout_sec
        self.last_output_time: float | None = None

    def mark_output(self, now: float) -> None:
        self.last_output_time = now

    def expired(self, now: float) -> bool:
        if self.last_output_time is None:
            return True
        return now - self.last_output_time > self.timeout_sec + TIME_EPSILON
