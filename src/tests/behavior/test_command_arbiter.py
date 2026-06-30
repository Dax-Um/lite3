from lite3_behavior.command_arbiter import ArbiterInput, CommandArbiter
from lite3_common.types import Twist2D


ZERO = Twist2D(0.0, 0.0, 0.0)


def test_arbiter_priority_emergency_over_return_home():
    selected = CommandArbiter().select(
        ArbiterInput(
            emergency_stop=True,
            return_home_active=True,
            return_home_cmd=Twist2D(0.1, 0.0, 0.0),
            manual_active=True,
            manual_cmd=Twist2D(0.0, 0.1, 0.0),
            patrol_cmd=Twist2D(0.0, 0.0, 0.1),
        )
    )

    assert selected == ZERO


def test_arbiter_priority_return_home_over_manual():
    selected = CommandArbiter().select(
        ArbiterInput(
            emergency_stop=False,
            return_home_active=True,
            return_home_cmd=Twist2D(0.1, 0.0, 0.0),
            manual_active=True,
            manual_cmd=Twist2D(0.0, 0.1, 0.0),
            patrol_cmd=Twist2D(0.0, 0.0, 0.1),
        )
    )

    assert selected == Twist2D(0.1, 0.0, 0.0)


def test_arbiter_priority_manual_over_patrol():
    selected = CommandArbiter().select(
        ArbiterInput(
            emergency_stop=False,
            return_home_active=False,
            return_home_cmd=ZERO,
            manual_active=True,
            manual_cmd=Twist2D(0.0, 0.1, 0.0),
            patrol_cmd=Twist2D(0.0, 0.0, 0.1),
        )
    )

    assert selected == Twist2D(0.0, 0.1, 0.0)


def test_arbiter_uses_patrol_when_no_higher_priority_input():
    selected = CommandArbiter().select(
        ArbiterInput(
            emergency_stop=False,
            return_home_active=False,
            return_home_cmd=ZERO,
            manual_active=False,
            manual_cmd=ZERO,
            patrol_cmd=Twist2D(0.0, 0.0, 0.1),
        )
    )

    assert selected == Twist2D(0.0, 0.0, 0.1)
