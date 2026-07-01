import importlib.util
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[3]
    / "scripts"
    / "run_lidar_auto_patrol_dry_run.py"
)


def load_script():
    spec = importlib.util.spec_from_file_location(
        "run_lidar_auto_patrol_dry_run",
        SCRIPT_PATH,
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_build_node_kwargs_never_attaches_motion_output():
    script = load_script()
    args = script.parse_args(
        [
            "--scan-topic",
            "/scan",
            "--odom-topic",
            "/leg_odom2",
            "--imu-topic",
            "/imu/data",
            "--duration-sec",
            "30",
            "--patrol-speed-mps",
            "0.05",
        ]
    )

    kwargs = script.build_node_kwargs(args)

    assert kwargs["scan_topic"] == "/scan"
    assert kwargs["odom_topic"] == "/leg_odom2"
    assert kwargs["imu_topic"] == "/imu/data"
    assert kwargs["motion_output"] is None
    assert kwargs["auto_start"] is True
    assert kwargs["runtime_flags"].preflight_ok is True
    assert kwargs["runtime_flags"].auto_mode_ok is True
    assert kwargs["runtime_flags"].stand_ready_ok is True


def test_parse_args_rejects_non_positive_duration():
    script = load_script()
    args = script.parse_args(["--duration-sec", "0"])

    try:
        script.validate_args(args)
    except SystemExit as exc:
        assert "--duration-sec" in str(exc)
    else:
        raise AssertionError("expected SystemExit")
