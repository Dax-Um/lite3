import importlib.util
import json
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[3] / "scripts" / "run_waypoint_patrol.py"


def load_script():
    spec = importlib.util.spec_from_file_location("run_waypoint_patrol", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_start_dry_run_builds_default_route_and_saves_home(capsys, tmp_path: Path):
    script = load_script()
    state_file = tmp_path / "state.json"

    result = script.main(
        [
            "start",
            "--dry-run",
            "--home",
            "1.0",
            "2.0",
            "0.5",
            "--state-file",
            str(state_file),
            "--available-action",
            "/FollowWaypoints",
        ]
    )

    assert result == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["mode"] == "dry-run"
    assert payload["route"]["route_id"] == "default_patrol"
    assert [item["id"] for item in payload["route"]["waypoints"]] == [
        "home",
        "p1",
        "p2",
        "home_return",
    ]
    assert state_file.exists()


def test_stop_dry_run_uses_saved_home(capsys, tmp_path: Path):
    script = load_script()
    state_file = tmp_path / "state.json"
    script.main(
        [
            "start",
            "--dry-run",
            "--home",
            "1.0",
            "2.0",
            "0.5",
            "--state-file",
            str(state_file),
            "--available-action",
            "/FollowWaypoints",
        ]
    )
    capsys.readouterr()

    result = script.main(
        [
            "stop",
            "--dry-run",
            "--current",
            "4.0",
            "2.0",
            "0.0",
            "--state-file",
            str(state_file),
            "--available-action",
            "/FollowWaypoints",
        ]
    )

    assert result == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["route"]["route_id"] == "return_home"
    assert payload["route"]["waypoints"][-1]["x"] == 1.0
    assert payload["route"]["waypoints"][-1]["y"] == 2.0


def test_execute_without_motion_approval_refuses_before_sending(capsys, tmp_path: Path):
    script = load_script()

    result = script.main(
        [
            "start",
            "--execute",
            "--home",
            "0.0",
            "0.0",
            "0.0",
            "--state-file",
            str(tmp_path / "state.json"),
            "--available-action",
            "/FollowWaypoints",
        ]
    )

    assert result == 3
    assert "refusing" in capsys.readouterr().err


def test_start_dry_run_uses_external_route_yaml(capsys, tmp_path: Path):
    route_file = tmp_path / "route.yaml"
    route_file.write_text(
        """
route_id: web_route
frame_id: map
waypoints:
  - id: web1
    x: 1.0
    y: 0.0
    yaw: 0.0
  - id: web2
    x: 2.0
    y: 0.0
    yaw: 0.0
  - id: web3
    x: 3.0
    y: 0.0
    yaw: 0.0
""",
        encoding="utf-8",
    )
    script = load_script()

    result = script.main(
        [
            "start",
            "--dry-run",
            "--home",
            "0.0",
            "0.0",
            "0.0",
            "--route-yaml",
            str(route_file),
            "--state-file",
            str(tmp_path / "state.json"),
            "--available-action",
            "/FollowWaypoints",
        ]
    )

    assert result == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["route"]["route_id"] == "web_route"
    assert [item["id"] for item in payload["route"]["waypoints"]] == ["web1", "web2", "web3"]
