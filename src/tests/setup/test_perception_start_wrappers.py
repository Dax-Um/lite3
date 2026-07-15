import subprocess
import os
import stat
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
LIDAR = ROOT / "scripts" / "perception_host_start_lidar.sh"
NAV = ROOT / "scripts" / "perception_host_start_navigation.sh"
STATUS = ROOT / "scripts" / "perception_host_nav_status.sh"
INSTALLER = ROOT / "scripts" / "install_perception_host_services.sh"
LIDAR_PROBE = ROOT / "scripts" / "perception_host_probe_lidar.py"


def test_start_wrappers_are_valid_bash():
    for script in (LIDAR, NAV, STATUS):
        subprocess.run(["bash", "-n", str(script)], check=True)


def test_remote_wrappers_are_executable_and_installer_copies_helpers():
    for script in (LIDAR, NAV, STATUS):
        assert script.stat().st_mode & stat.S_IXUSR

    installer = INSTALLER.read_text(encoding="utf-8")
    assert "perception_host_prepare_nav_config.py" in installer
    assert "perception_host_probe_lidar.py" in installer
    assert "perception_host_start_watchdog.sh" in installer
    assert "run_nav_watchdog.py" in installer


def test_lidar_start_requires_fresh_samples_and_detaches_vendor_launch():
    source = LIDAR.read_text(encoding="utf-8")

    assert 'python3 "$LIDAR_PROBE"' in source
    assert 'LITE3_LIDAR_EXPECTED_FRAME:-rslidar' in source
    assert '--expected-frame "$LIDAR_EXPECTED_FRAME"' in source
    assert 'lidar_is_fresh "$READY_TIMEOUT_SEC"' in source
    assert 'nohup bash "$LIDAR_SCRIPT"' in source
    assert 'pgrep -af "start_rslidar.sh|rslidar"' not in source


def test_navigation_recovers_partial_stack_and_applies_persisted_safety_config():
    source = NAV.read_text(encoding="utf-8")
    status = STATUS.read_text(encoding="utf-8")

    assert 'python3 "$CONFIG_SCRIPT" --apply' in source
    assert '"$STOP_SCRIPT" --execute' in source
    assert 'nohup bash "$NAV_SCRIPT"' in source
    assert 'python3 "$CONFIG_SCRIPT"' in status
    assert 'python3 "$CONFIG_SCRIPT" --live' in status
    assert 'LITE3_LIVE_CONFIG_TIMEOUT_SEC:-25' in status
    assert 'timeout "$LIVE_CONFIG_TIMEOUT_SEC" python3 "$CONFIG_SCRIPT" --live' in status
    assert 'python3 "$LIDAR_PROBE"' in status
    assert '--expected-frame "$LIDAR_EXPECTED_FRAME"' in status
    assert "/bt_navigator" in status


def test_wrappers_call_out_required_power_on_frame_and_stamp_verification():
    for script in (LIDAR, NAV, STATUS):
        source = script.read_text(encoding="utf-8")
        assert "power-on check: verify actual /rslidar_points" in source
        assert "header.frame_id and header.stamp" in source
