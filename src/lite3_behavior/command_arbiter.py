"""Command priority arbiter."""

from dataclasses import dataclass

from lite3_common.types import Twist2D


ZERO_TWIST = Twist2D(0.0, 0.0, 0.0)


@dataclass(frozen=True)
class ArbiterInput:
    emergency_stop: bool
    return_home_active: bool
    return_home_cmd: Twist2D
    manual_active: bool
    manual_cmd: Twist2D
    patrol_cmd: Twist2D


class CommandArbiter:
    def select(self, item: ArbiterInput) -> Twist2D:
        if item.emergency_stop:
            return ZERO_TWIST
        if item.return_home_active:
            return item.return_home_cmd
        if item.manual_active:
            return item.manual_cmd
        return item.patrol_cmd
