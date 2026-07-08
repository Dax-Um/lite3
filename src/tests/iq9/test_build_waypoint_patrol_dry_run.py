import importlib.util
import json
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[3] / "scripts" / "build_waypoint_patrol_dry_run.py"


def load_script():
    spec = importlib.util.spec_from_file_location("build_waypoint_patrol_dry_run", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_builds_default_patrol_goal_from_home_pose(capsys):
    script = load_script()

    result = script.main(
        [
            "start",
            "--home",
            "1.0",
            "2.0",
            "0.0",
            "--available-action",
            "/FollowWaypoints",
        ]
    )

    assert result == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["route"]["route_id"] == "default_patrol"
    assert [item["id"] for item in payload["route"]["waypoints"]] == [
        "home",
        "p1",
        "p2",
        "home_return",
    ]
    assert payload["plan"]["ready"] is True
    assert payload["plan"]["would_send"] is False
    assert len(payload["plan"]["poses"]) == 4


def test_builds_return_home_goal_when_patrol_is_stopped(capsys):
    script = load_script()

    result = script.main(
        [
            "stop",
            "--home",
            "1.0",
            "2.0",
            "0.5",
            "--current",
            "4.0",
            "2.0",
            "0.0",
            "--available-action",
            "/FollowWaypoints",
        ]
    )

    assert result == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["route"]["route_id"] == "return_home"
    assert [item["id"] for item in payload["route"]["waypoints"]] == ["current", "home_return"]
    assert payload["route"]["waypoints"][-1]["x"] == 1.0
    assert payload["route"]["waypoints"][-1]["y"] == 2.0


def test_uses_external_route_yaml_for_start(capsys, tmp_path: Path):
    route_file = tmp_path / "route.yaml"
    route_file.write_text(
        """
route_id: web_route
frame_id: map
loop: false
waypoints:
  - id: a
    x: 1.0
    y: 0.0
    yaw: 0.0
  - id: b
    x: 2.0
    y: 0.0
    yaw: 0.0
  - id: c
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
            "--home",
            "0.0",
            "0.0",
            "0.0",
            "--route-yaml",
            str(route_file),
            "--available-action",
            "/FollowWaypoints",
        ]
    )

    assert result == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["route"]["route_id"] == "web_route"
    assert [item["id"] for item in payload["route"]["waypoints"]] == ["a", "b", "c"]


def test_start_saves_home_and_stop_uses_saved_home(capsys, tmp_path: Path):
    state_file = tmp_path / "patrol_state.json"
    script = load_script()

    start_result = script.main(
        [
            "start",
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

    stop_result = script.main(
        [
            "stop",
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

    assert start_result == 0
    assert stop_result == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["route"]["route_id"] == "return_home"
    assert payload["route"]["waypoints"][-1]["x"] == 1.0
    assert payload["route"]["waypoints"][-1]["y"] == 2.0
    assert payload["route"]["waypoints"][-1]["yaw"] == 0.5
