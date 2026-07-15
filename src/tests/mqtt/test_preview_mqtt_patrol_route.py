import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace


SCRIPT = Path(__file__).resolve().parents[3] / "scripts" / "preview_mqtt_patrol_route.py"


def load_script():
    spec = importlib.util.spec_from_file_location("preview_mqtt_patrol_route", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_preview_defaults_to_non_follow_waypoint_action_and_safe_clearance():
    args = load_script().parse_args([])

    assert args.action_name == "/navigate_to_pose"
    assert args.route_clearance_m == 0.50


def test_preview_payload_contains_pose_without_motion_fields():
    module = load_script()
    waypoint = module._waypoint_json(
        SimpleNamespace(id="p1", x=1.0, y=2.0, yaw=0.3)
    )

    assert waypoint == {"id": "p1", "x": 1.0, "y": 2.0, "yaw": 0.3}


def test_preview_selects_route_without_ever_sending_motion(monkeypatch, capsys):
    module = load_script()
    home = SimpleNamespace(id="home", x=1.0, y=2.0, yaw=0.0)
    route = SimpleNamespace(
        frame_id="map",
        waypoints=[SimpleNamespace(id="p1", x=3.0, y=2.0, yaw=0.0)],
    )
    calls = []

    class Backend:
        def __init__(self, **kwargs):
            calls.append(("init", kwargs))

        def prepare_route(self):
            calls.append(("prepare", None))

        def wait_until_ready(self, timeout_sec=None):
            calls.append(("ready", timeout_sec))

        def capture_current_pose(self, *, waypoint_id):
            calls.append(("pose", waypoint_id))
            return home

        def validate_route(self, candidate, *, start):
            calls.append(("validate", candidate, start))

        def send_route(self, candidate):
            raise AssertionError("preview must never call send_route")

    class Config:
        def build_candidate_routes(self, captured_home):
            assert captured_home is home
            return [route]

    monkeypatch.setattr(module, "Nav2PatrolBackend", Backend)
    monkeypatch.setattr(
        module.PatrolConfig,
        "from_yaml",
        classmethod(lambda cls, path: Config()),
    )
    monkeypatch.setattr(module, "_validate_route", lambda candidate: None)

    assert module.main([]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["motion_sent"] is False
    assert payload["home"]["id"] == "home"
    assert payload["waypoints"][0]["id"] == "p1"
    assert not any(call[0] == "send_route" for call in calls)
