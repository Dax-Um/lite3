"""Execution readiness gate for real robot patrol runtime."""

from dataclasses import dataclass


TIME_EPSILON = 1e-9


@dataclass(frozen=True)
class ReadinessConfig:
    scan_timeout_sec: float = 0.50
    odom_timeout_sec: float = 0.50
    imu_timeout_sec: float = 0.50
    require_scan: bool = True
    require_odom: bool = True
    require_imu: bool = True


@dataclass(frozen=True)
class ReadinessInput:
    now: float
    scan_last_seen: float | None
    odom_last_seen: float | None
    imu_last_seen: float | None
    motion_host_reachable: bool
    preflight_ok: bool
    auto_mode_ok: bool
    stand_ready_ok: bool


@dataclass(frozen=True)
class ReadinessResult:
    ready: bool
    reasons: tuple[str, ...]


class ReadinessGate:
    def __init__(self, config: ReadinessConfig = ReadinessConfig()):
        self.config = config

    def check(self, item: ReadinessInput) -> ReadinessResult:
        reasons: list[str] = []

        if not item.preflight_ok:
            reasons.append("preflight")
        if not item.motion_host_reachable:
            reasons.append("motion_host")
        if not item.auto_mode_ok:
            reasons.append("auto_mode")
        if not item.stand_ready_ok:
            reasons.append("stand_ready")

        self._append_sensor_reason(
            reasons,
            "scan",
            item.scan_last_seen,
            item.now,
            self.config.scan_timeout_sec,
            self.config.require_scan,
        )
        self._append_sensor_reason(
            reasons,
            "odom",
            item.odom_last_seen,
            item.now,
            self.config.odom_timeout_sec,
            self.config.require_odom,
        )
        self._append_sensor_reason(
            reasons,
            "imu",
            item.imu_last_seen,
            item.now,
            self.config.imu_timeout_sec,
            self.config.require_imu,
        )

        return ReadinessResult(ready=not reasons, reasons=tuple(reasons))

    @staticmethod
    def _append_sensor_reason(
        reasons: list[str],
        name: str,
        last_seen: float | None,
        now: float,
        timeout_sec: float,
        required: bool,
    ) -> None:
        if not required:
            return
        if last_seen is None:
            reasons.append(f"{name}_missing")
            return
        if _expired(last_seen, now, timeout_sec):
            reasons.append(f"{name}_stale")


def _expired(last_seen: float, now: float, timeout_sec: float) -> bool:
    return now - last_seen > timeout_sec + TIME_EPSILON
