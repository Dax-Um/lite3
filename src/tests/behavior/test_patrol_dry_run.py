import importlib.util
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[3] / "scripts" / "run_patrol_dry_run.py"


def load_script():
    spec = importlib.util.spec_from_file_location("run_patrol_dry_run", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_patrol_dry_run_outputs_status_rows():
    script = load_script()

    lines = script.run_dry_run().splitlines()

    assert lines[0] == (
        "time,state,lane_index,direction,min_front,"
        "raw_vx,raw_vy,raw_wz,safe_vx,safe_vy,safe_wz,stop_reason"
    )
    assert any("move_along_lane" in line for line in lines)
    assert any("shift_to_next_lane" in line for line in lines)
    assert any("turn_around" in line for line in lines)
    assert lines[-1].split(",")[1] == "finish"
