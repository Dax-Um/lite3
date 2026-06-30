import importlib.util
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[3] / "scripts" / "run_front_boundary_log.py"


def load_script():
    spec = importlib.util.spec_from_file_location("run_front_boundary_log", SCRIPT_PATH)
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
        min_front_distance_m=0.5,
        valid_front_points=4,
    )

    row = script.format_boundary_row(1.25, result)

    assert row == "1.250,0.500,4,true,true,true"


def test_format_boundary_row_handles_missing_distance():
    script = load_script()
    result = script.BoundaryResult(
        lane_end=False,
        should_slow=False,
        should_stop=False,
        min_front_distance_m=None,
        valid_front_points=0,
    )

    row = script.format_boundary_row(1.25, result)

    assert row == "1.250,,0,false,false,false"
