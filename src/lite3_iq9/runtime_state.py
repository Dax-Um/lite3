"""Runtime state aggregation for IQ9 web and waypoint gating."""

from __future__ import annotations

from dataclasses import dataclass

from lite3_common.types import Pose2D, Twist2D


@dataclass(frozen=True)
class RuntimeSnapshot:
    ready_for_waypoint: bool
    blocking_reasons: list[str]
    robot: dict[str, float | bool | None]
    nav: dict[str, object]
    sensors: dict[str, float | bool | None]


class RuntimeStateAggregator:
    def __init__(
        self,
        max_pose_age_sec: float = 0.5,
        max_costmap_age_sec: float = 1.0,
        max_camera_age_sec: float = 0.5,
    ) -> None:
        self.max_pose_age_sec = max_pose_age_sec
        self.max_costmap_age_sec = max_costmap_age_sec
        self.max_camera_age_sec = max_camera_age_sec
        self._pose: Pose2D | None = None
        self._pose_stamp_sec: float | None = None
        self._localization_ok = False
        self._map_stamp_sec: float | None = None
        self._local_costmap_stamp_sec: float | None = None
        self._global_costmap_stamp_sec: float | None = None
        self._camera_stamp_sec: float | None = None
        self._cmd_vel: Twist2D | None = None
        self._cmd_vel_stamp_sec: float | None = None

    def update_pose(self, pose: Pose2D, stamp_sec: float, localization_ok: bool) -> None:
        self._pose = pose
        self._pose_stamp_sec = stamp_sec
        self._localization_ok = localization_ok

    def update_local_costmap(self, stamp_sec: float) -> None:
        self._local_costmap_stamp_sec = stamp_sec

    def update_global_costmap(self, stamp_sec: float) -> None:
        self._global_costmap_stamp_sec = stamp_sec

    def update_map(self, stamp_sec: float) -> None:
        self._map_stamp_sec = stamp_sec

    def update_camera(self, stamp_sec: float) -> None:
        self._camera_stamp_sec = stamp_sec

    def update_cmd_vel(self, twist: Twist2D, stamp_sec: float) -> None:
        self._cmd_vel = twist
        self._cmd_vel_stamp_sec = stamp_sec

    def snapshot(self, now_sec: float) -> RuntimeSnapshot:
        pose_age = _age(now_sec, self._pose_stamp_sec)
        map_age = _age(now_sec, self._map_stamp_sec)
        costmap_age = _age(now_sec, self._local_costmap_stamp_sec)
        global_costmap_age = _age(now_sec, self._global_costmap_stamp_sec)
        camera_age = _age(now_sec, self._camera_stamp_sec)
        reasons: list[str] = []

        if self._pose is None:
            reasons.append("pose_missing")
        elif pose_age is None or pose_age > self.max_pose_age_sec:
            reasons.append("pose_stale")

        if not self._localization_ok:
            reasons.append("localization_not_ok")

        if self._local_costmap_stamp_sec is None:
            reasons.append("local_costmap_missing")
        elif costmap_age is None or costmap_age > self.max_costmap_age_sec:
            reasons.append("local_costmap_stale")

        if self._map_stamp_sec is None:
            reasons.append("map_missing")

        if self._global_costmap_stamp_sec is None:
            reasons.append("global_costmap_missing")
        elif global_costmap_age is None or global_costmap_age > self.max_costmap_age_sec:
            reasons.append("global_costmap_stale")

        robot = {
            "x": self._pose.x if self._pose else None,
            "y": self._pose.y if self._pose else None,
            "yaw": self._pose.yaw if self._pose else None,
            "localization_ok": self._localization_ok,
            "pose_age_sec": pose_age,
        }
        nav = {
            "last_cmd_vel": _twist_dict(self._cmd_vel),
            "cmd_vel_age_sec": _age(now_sec, self._cmd_vel_stamp_sec),
        }
        sensors = {
            "map_ok": self._map_stamp_sec is not None,
            "lidar_ok": self._local_costmap_stamp_sec is not None,
            "camera_ok": self._camera_stamp_sec is not None
            and camera_age is not None
            and camera_age <= self.max_camera_age_sec,
            "global_costmap_ok": self._global_costmap_stamp_sec is not None
            and global_costmap_age is not None
            and global_costmap_age <= self.max_costmap_age_sec,
            "map_age_sec": map_age,
            "local_costmap_age_sec": costmap_age,
            "global_costmap_age_sec": global_costmap_age,
            "camera_age_sec": camera_age,
        }
        return RuntimeSnapshot(
            ready_for_waypoint=not reasons,
            blocking_reasons=reasons,
            robot=robot,
            nav=nav,
            sensors=sensors,
        )


def _age(now_sec: float, stamp_sec: float | None) -> float | None:
    if stamp_sec is None:
        return None
    return round(now_sec - stamp_sec, 6)


def _twist_dict(twist: Twist2D | None) -> dict[str, float]:
    if twist is None:
        return {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0}
    return {"linear_x": twist.vx, "linear_y": twist.vy, "angular_z": twist.wz}
