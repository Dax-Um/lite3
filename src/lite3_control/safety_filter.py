"""Final software safety filter before robot output."""

from dataclasses import dataclass

from lite3_common.types import StopReason, Twist2D
from lite3_control.udp_driver import clamp


ZERO_TWIST = Twist2D(0.0, 0.0, 0.0)
TIME_EPSILON = 1e-9


@dataclass
class SensorTimestamps:
    lidar: float | None = None
    odom: float | None = None
    imu: float | None = None


@dataclass(frozen=True)
class SafetyConfig:
    command_timeout_sec: float = 0.30
    lidar_timeout_sec: float = 0.50
    odom_timeout_sec: float = 0.50
    imu_timeout_sec: float = 0.50
    max_vx_mps: float = 0.10
    max_vy_mps: float = 0.05
    max_wz_radps: float = 0.20
    require_lidar: bool = True
    require_odom: bool = True
    require_imu: bool = True


class SafetyFilter:
    def __init__(self, config: SafetyConfig):
        self.config = config
        self.sensor_timestamps = SensorTimestamps()
        self.last_command_time: float | None = None
        self.emergency_stop = False
        self.front_obstacle = False

    def update_lidar(self, now: float) -> None:
        self.sensor_timestamps.lidar = now

    def update_odom(self, now: float) -> None:
        self.sensor_timestamps.odom = now

    def update_imu(self, now: float) -> None:
        self.sensor_timestamps.imu = now

    def mark_command(self, now: float) -> None:
        self.last_command_time = now

    def set_emergency_stop(self, enabled: bool) -> None:
        self.emergency_stop = enabled

    def set_front_obstacle(self, enabled: bool) -> None:
        self.front_obstacle = enabled

    def filter_cmd(self, cmd: Twist2D, now: float) -> tuple[Twist2D, StopReason]:
        stop_reason = self._stop_reason(now)
        if stop_reason is not StopReason.NONE:
            return ZERO_TWIST, stop_reason

        return (
            Twist2D(
                vx=clamp(cmd.vx, self.config.max_vx_mps),
                vy=clamp(cmd.vy, self.config.max_vy_mps),
                wz=clamp(cmd.wz, self.config.max_wz_radps),
            ),
            StopReason.NONE,
        )

    def _stop_reason(self, now: float) -> StopReason:
        if self.emergency_stop:
            return StopReason.EMERGENCY_STOP
        if _expired(self.last_command_time, now, self.config.command_timeout_sec):
            return StopReason.COMMAND_TIMEOUT
        if self.config.require_lidar and _expired(
            self.sensor_timestamps.lidar,
            now,
            self.config.lidar_timeout_sec,
        ):
            return StopReason.LIDAR_TIMEOUT
        if self.config.require_odom and _expired(
            self.sensor_timestamps.odom,
            now,
            self.config.odom_timeout_sec,
        ):
            return StopReason.ODOM_TIMEOUT
        if self.config.require_imu and _expired(
            self.sensor_timestamps.imu,
            now,
            self.config.imu_timeout_sec,
        ):
            return StopReason.IMU_TIMEOUT
        if self.front_obstacle:
            return StopReason.FRONT_OBSTACLE
        return StopReason.NONE


def _expired(last_seen: float | None, now: float, timeout_sec: float) -> bool:
    if last_seen is None:
        return True
    return now - last_seen > timeout_sec + TIME_EPSILON
