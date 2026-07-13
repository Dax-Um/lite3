from types import SimpleNamespace

from lite3_iq9.ros2_state_bridge import _status_ok


def test_hdl_localization_status_uses_has_converged():
    assert _status_ok(SimpleNamespace(has_converged=True)) is True
    assert _status_ok(SimpleNamespace(has_converged=False)) is False
