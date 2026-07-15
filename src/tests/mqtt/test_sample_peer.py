import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[3] / "scripts" / "run_mqtt_sample_peer.py"


def _load_script():
    spec = importlib.util.spec_from_file_location("run_mqtt_sample_peer", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_motion_commands_require_explicit_authorization_before_mqtt_connect(capsys):
    module = _load_script()

    assert module.main(["patrol-start"]) == 3
    assert module.main(["patrol-return-home"]) == 3
    assert module.main(["patrol-reset"]) == 3

    assert "--allow-patrol-start" in capsys.readouterr().err


def test_default_scenario_never_publishes_a_patrol_command():
    module = _load_script()

    assert module._scenario_commands(False) == ["sound", "coyote"]


def test_authorized_scenario_brackets_detection_with_start_and_stop():
    module = _load_script()
    published = []

    class Info:
        rc = 0

        def wait_for_publish(self, timeout):
            assert timeout == 5.0

        def is_published(self):
            return True

    class Client:
        def publish(self, topic, payload, qos, retain):
            published.append((topic, payload, qos, retain))
            return Info()

    assert module._scenario_commands(True) == ["patrol-start", "sound", "coyote"]

    # Preload valid media responses so the scenario completes without a broker
    # while preserving the production topic constants for every other test.
    received = [
        (
            module.Topics.BROKEN_CUP_IMAGE,
            {"event_id": "broken", "result": "SUCCESS"},
        ),
        (
            module.Topics.BROKEN_CUP_VIDEO,
            {"event_id": "broken", "result": "SUCCESS"},
        ),
        (
            module.Topics.COYOTE_IMAGE,
            {"event_id": "coyote", "result": "SUCCESS"},
        ),
        (
            module.Topics.COYOTE_VIDEO,
            {"event_id": "coyote", "result": "SUCCESS"},
        ),
    ]
    assert (
        module._scenario(
            Client(),
            received,
            __import__("threading").Event(),
            0.0,
            include_patrol_start=True,
        )
        == 0
    )

    auto_patrol_payloads = [
        payload for topic, payload, qos, retain in published if topic == module.Topics.AUTO_PATROL
    ]
    assert '"action":"START"' in auto_patrol_payloads[0]
    assert '"action":"STOP"' in auto_patrol_payloads[-1]
    assert all(qos == 0 and retain is False for _, _, qos, retain in published)
