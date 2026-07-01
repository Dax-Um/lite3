"""Adapter from patrol controller output to the runtime motion driver."""

from dataclasses import dataclass

from lite3_common.types import StopReason


@dataclass(frozen=True)
class RuntimeOutputConfig:
    stop_repeat: int = 10
    stop_dt_sec: float = 0.05


class RuntimeMotionOutput:
    def __init__(self, driver, config: RuntimeOutputConfig = RuntimeOutputConfig()):
        self.driver = driver
        self.config = config

    def publish(self, controller_output) -> None:
        if controller_output.stop_reason is not StopReason.NONE:
            self._stop()
            return

        cmd = controller_output.safe_cmd
        try:
            self.driver.send_cmd_vel(cmd.vx, cmd.vy, cmd.wz)
        except Exception:
            self._stop()
            raise

    def _stop(self) -> None:
        self.driver.stop(self.config.stop_repeat, self.config.stop_dt_sec)
