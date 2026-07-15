import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[3] / "scripts" / "run_mqtt_runtime.py"


def load_script():
    spec = importlib.util.spec_from_file_location("run_mqtt_runtime", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_direct_nav2_runtime_exposes_only_required_timeouts():
    args = load_script().parse_args([])

    assert args.action_name == "/FollowWaypoints"
    assert args.nav_timeout_sec == 10.0
    assert args.nav_route_timeout_sec == 300.0
    assert args.nav_cancel_timeout_sec == 5.0
    source = SCRIPT.read_text(encoding="utf-8")
    assert "PerceptionHostNavManager" not in source
    assert "nav_arrival" not in source
    assert "nav_route_clearance" not in source


def test_patrol_runtime_subscribes_to_patrol_and_detection_takeover_events():
    source = SCRIPT.read_text(encoding="utf-8")

    assert "Topics.AUTO_PATROL," in source
    assert "Topics.SOUND_DETECT," in source
    assert "Topics.COYOTE_DETECT," in source
    assert "parse_detection_trigger(topic, payload)" in source
    assert "patrol.stop()" in source


def test_production_runtime_does_not_publish_mock_media_from_search_trigger():
    script = load_script()

    assert script.parse_args([]).mock_detection_media is False
    assert (
        script.parse_args(["--mock-detection-media"]).mock_detection_media
        is True
    )


def test_mqtt_connection_defaults_come_from_the_single_container_environment(
    monkeypatch,
):
    monkeypatch.setenv("MQTT_HOST", "broker.internal")
    monkeypatch.setenv("MQTT_PORT", "2883")
    monkeypatch.setenv("MQTT_USER", "robot")
    monkeypatch.setenv("MQTT_PASS", "secret")

    args = load_script().parse_args([])

    assert args.broker_host == "broker.internal"
    assert args.broker_port == 2883
    assert args.username == "robot"
    assert args.password == "secret"


def test_mqtt_client_cleanup_is_nested_under_runtime_close():
    source = SCRIPT.read_text(encoding="utf-8")

    assert "try:\n            runtime.close()\n        finally:\n            client.stop()" in source
