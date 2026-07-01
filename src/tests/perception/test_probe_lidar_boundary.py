import importlib.util
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[3] / "scripts" / "probe_lidar_boundary.py"


def load_script():
    spec = importlib.util.spec_from_file_location("probe_lidar_boundary", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_format_boundary_row_outputs_expected_columns():
    script = load_script()
    result = script.BoundaryResult(
        lane_end=True,
        should_slow=True,
        should_stop=True,
        min_front_distance_m=0.52,
        valid_front_points=31,
    )

    row = script.format_boundary_row(0.2, result)

    assert row == {
        "time": "0.20",
        "min_front_m": "0.520",
        "valid_points": "31",
        "should_slow": "true",
        "should_stop": "true",
        "lane_end": "true",
    }


def test_format_boundary_row_handles_missing_front_distance():
    script = load_script()
    result = script.BoundaryResult(
        lane_end=False,
        should_slow=False,
        should_stop=False,
        min_front_distance_m=None,
        valid_front_points=1,
    )

    row = script.format_boundary_row(0.1, result)

    assert row["min_front_m"] == ""
    assert row["valid_points"] == "1"
    assert row["should_stop"] == "false"


def test_exit_code_reports_failure_when_no_scan_was_seen():
    script = load_script()

    assert script.exit_code_for_seen_scan(True) == 0
    assert script.exit_code_for_seen_scan(False) == 1
