"""Lite3 odometry tracker."""

from dataclasses import dataclass
from math import hypot

from lite3_common.types import PathPoint, Pose2D


@dataclass(frozen=True)
class OdomTrackerConfig:
    sample_period_sec: float = 0.2
    min_distance_step_m: float = 0.05


class OdomTracker:
    def __init__(self, config: OdomTrackerConfig = OdomTrackerConfig()):
        self.config = config
        self._active = False
        self._current_pose: Pose2D | None = None
        self._home_pose: Pose2D | None = None
        self._path_trace: list[PathPoint] = []
        self._last_sample_time: float | None = None

    def start_session(self, pose: Pose2D, now: float) -> None:
        self._active = True
        self._current_pose = pose
        self._home_pose = pose
        self._path_trace = [PathPoint(pose=pose, timestamp=now)]
        self._last_sample_time = now

    def sample(self, pose: Pose2D, now: float) -> None:
        self._current_pose = pose
        if not self._active:
            return
        if not self._path_trace:
            self._append_sample(pose, now)
            return

        last_point = self._path_trace[-1]
        enough_time = (
            self._last_sample_time is None
            or now - self._last_sample_time >= self.config.sample_period_sec
        )
        enough_distance = _distance(last_point.pose, pose) >= self.config.min_distance_step_m
        if enough_time and enough_distance:
            self._append_sample(pose, now)

    def stop_session(self) -> None:
        self._active = False

    def active(self) -> bool:
        return self._active

    def current_pose(self) -> Pose2D | None:
        return self._current_pose

    def home_pose(self) -> Pose2D | None:
        return self._home_pose

    def path_trace(self) -> list[PathPoint]:
        return list(self._path_trace)

    def _append_sample(self, pose: Pose2D, now: float) -> None:
        self._path_trace.append(PathPoint(pose=pose, timestamp=now))
        self._last_sample_time = now


def _distance(a: Pose2D, b: Pose2D) -> float:
    return hypot(a.x - b.x, a.y - b.y)
