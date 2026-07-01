from lite3_control.readiness_gate import (
    ReadinessConfig,
    ReadinessGate,
    ReadinessInput,
)


def ready_input(**overrides):
    values = {
        "now": 10.0,
        "scan_last_seen": 9.9,
        "odom_last_seen": 9.9,
        "imu_last_seen": 9.9,
        "motion_host_reachable": True,
        "preflight_ok": True,
        "auto_mode_ok": True,
        "stand_ready_ok": True,
    }
    values.update(overrides)
    return ReadinessInput(**values)


def assert_not_ready_with_reason(item, reason):
    result = ReadinessGate().check(item)

    assert result.ready is False
    assert reason in result.reasons


def test_missing_preflight_is_not_ready():
    assert_not_ready_with_reason(ready_input(preflight_ok=False), "preflight")


def test_missing_auto_mode_is_not_ready():
    assert_not_ready_with_reason(ready_input(auto_mode_ok=False), "auto_mode")


def test_missing_stand_ready_is_not_ready():
    assert_not_ready_with_reason(ready_input(stand_ready_ok=False), "stand_ready")


def test_motion_host_unreachable_is_not_ready():
    assert_not_ready_with_reason(
        ready_input(motion_host_reachable=False),
        "motion_host",
    )


def test_missing_scan_is_not_ready():
    assert_not_ready_with_reason(ready_input(scan_last_seen=None), "scan_missing")


def test_stale_scan_is_not_ready():
    assert_not_ready_with_reason(ready_input(scan_last_seen=9.49), "scan_stale")


def test_stale_odom_is_not_ready():
    assert_not_ready_with_reason(ready_input(odom_last_seen=9.49), "odom_stale")


def test_stale_imu_is_not_ready():
    assert_not_ready_with_reason(ready_input(imu_last_seen=9.49), "imu_stale")


def test_all_required_inputs_fresh_is_ready():
    result = ReadinessGate().check(ready_input())

    assert result.ready is True
    assert result.reasons == ()


def test_disabled_imu_requirement_allows_missing_imu():
    gate = ReadinessGate(ReadinessConfig(require_imu=False))

    result = gate.check(ready_input(imu_last_seen=None))

    assert result.ready is True
    assert result.reasons == ()
