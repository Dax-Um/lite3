"""Direct odom-line return-home controller."""

from dataclasses import dataclass
from enum import Enum
from math import atan2, hypot, pi

from lite3_common.types import PathPoint, Pose2D, Twist2D
from lite3_control.udp_driver import clamp


ZERO_TWIST = Twist2D(0.0, 0.0, 0.0)


@dataclass(frozen=True)
class ReturnHomeConfig:
    home_position_tolerance_m: float = 0.25
    home_yaw_tolerance_rad: float = 0.17
    face_home_yaw_tolerance_rad: float = 0.25
    max_vx_mps: float = 0.12
    max_wz_radps: float = 0.20
    k_linear: float = 0.6
    k_yaw: float = 1.5


class ReturnHomeState(Enum):
    INACTIVE = "inactive"
    TURN_TO_HOME = "turn_to_home"
    DRIVE_TO_HOME = "drive_to_home"
    FINAL_ALIGN = "final_align"


class ReturnHomeController:
    def __init__(self, config: ReturnHomeConfig):
        self.config = config
        self._home: Pose2D | None = None
        self._path_trace: list[PathPoint] = []
        self._state = ReturnHomeState.INACTIVE

    def start(self, home: Pose2D, path_trace: list[PathPoint] | None = None) -> None:
        self._home = home
        self._path_trace = list(path_trace or [])
        self._state = ReturnHomeState.TURN_TO_HOME

    def active(self) -> bool:
        return self._state is not ReturnHomeState.INACTIVE

    def tick(self, current: Pose2D) -> tuple[Twist2D, bool]:
        if self._home is None or self._state is ReturnHomeState.INACTIVE:
            return ZERO_TWIST, True

        distance = _distance(current, self._home)
        if distance <= self.config.home_position_tolerance_m:
            return self._final_align(current)

        yaw_error = _target_yaw_error(current, self._home)
        if abs(yaw_error) > self.config.face_home_yaw_tolerance_rad:
            self._state = ReturnHomeState.TURN_TO_HOME
            return Twist2D(0.0, 0.0, _yaw_cmd(yaw_error, self.config)), False

        self._state = ReturnHomeState.DRIVE_TO_HOME
        return (
            Twist2D(
                vx=clamp(self.config.k_linear * distance, self.config.max_vx_mps),
                vy=0.0,
                wz=_yaw_cmd(yaw_error, self.config),
            ),
            False,
        )

    def cancel(self) -> None:
        self._state = ReturnHomeState.INACTIVE

    def _final_align(self, current: Pose2D) -> tuple[Twist2D, bool]:
        assert self._home is not None
        yaw_error = normalize_angle(self._home.yaw - current.yaw)
        if abs(yaw_error) <= self.config.home_yaw_tolerance_rad:
            self.cancel()
            return ZERO_TWIST, True

        self._state = ReturnHomeState.FINAL_ALIGN
        return Twist2D(0.0, 0.0, _yaw_cmd(yaw_error, self.config)), False


def normalize_angle(angle: float) -> float:
    while angle > pi:
        angle -= 2.0 * pi
    while angle <= -pi:
        angle += 2.0 * pi
    return angle


def _target_yaw_error(current: Pose2D, home: Pose2D) -> float:
    return normalize_angle(atan2(home.y - current.y, home.x - current.x) - current.yaw)


def _distance(current: Pose2D, home: Pose2D) -> float:
    return hypot(home.x - current.x, home.y - current.y)


def _yaw_cmd(yaw_error: float, config: ReturnHomeConfig) -> float:
    return clamp(config.k_yaw * yaw_error, config.max_wz_radps)
