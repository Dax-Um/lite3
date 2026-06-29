"""Shared patrol data types."""

from dataclasses import dataclass
from enum import Enum


@dataclass(frozen=True)
class Twist2D:
    vx: float
    vy: float
    wz: float


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class PathPoint:
    pose: Pose2D
    timestamp: float


@dataclass(frozen=True)
class MotionLimits:
    max_vx_mps: float = 0.10
    max_vy_mps: float = 0.05
    max_wz_radps: float = 0.20


class StopReason(Enum):
    NONE = "none"
    EMERGENCY_STOP = "emergency_stop"
    COMMAND_TIMEOUT = "command_timeout"
    LIDAR_TIMEOUT = "lidar_timeout"
    ODOM_TIMEOUT = "odom_timeout"
    IMU_TIMEOUT = "imu_timeout"
    FRONT_OBSTACLE = "front_obstacle"
    DRIVER_ERROR = "driver_error"
