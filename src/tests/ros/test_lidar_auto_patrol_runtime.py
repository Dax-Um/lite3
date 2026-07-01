import importlib.util
from pathlib import Path
from types import SimpleNamespace


SCRIPT_PATH = Path(__file__).resolve().parents[3] / "scripts" / "run_lidar_auto_patrol.py"


def load_script():
    spec = importlib.util.spec_from_file_location("run_lidar_auto_patrol", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_validate_args_requires_execute_and_operator_confirmations():
    script = load_script()
    args = script.parse_args(["--host", "192.168.1.120", "--port", "43893"])

    try:
        script.validate_args(args)
    except SystemExit as exc:
        assert "--execute" in str(exc)
    else:
        raise AssertionError("expected SystemExit")

    args = script.parse_args(
        ["--host", "192.168.1.120", "--port", "43893", "--execute"]
    )
    try:
        script.validate_args(args)
    except SystemExit as exc:
        assert "--preflight-ok" in str(exc)
    else:
        raise AssertionError("expected SystemExit")


def test_motion_host_reachable_uses_successful_runner_return_code():
    script = load_script()

    def runner(_cmd, **_kwargs):
        return SimpleNamespace(returncode=0)

    assert script.motion_host_reachable("192.168.1.120", runner=runner)


def test_motion_host_unreachable_on_nonzero_runner_return_code():
    script = load_script()

    def runner(_cmd, **_kwargs):
        return SimpleNamespace(returncode=1)

    assert not script.motion_host_reachable("192.168.1.120", runner=runner)


def test_build_node_kwargs_attaches_runtime_motion_output():
    script = load_script()
    created = []

    class FakeDriver:
        def __init__(self, host, port, limits):
            self.host = host
            self.port = port
            self.limits = limits
            created.append(self)

    args = script.parse_args(
        [
            "--host",
            "192.168.1.120",
            "--port",
            "43893",
            "--execute",
            "--preflight-ok",
            "--auto-mode-ok",
            "--stand-ready-ok",
            "--patrol-speed-mps",
            "0.05",
        ]
    )

    kwargs = script.build_node_kwargs(args, driver_factory=FakeDriver)

    assert created[0].host == "192.168.1.120"
    assert created[0].port == 43893
    assert kwargs["motion_output"] is not None
    assert kwargs["runtime_flags"].motion_host_reachable is True
    assert kwargs["runtime_flags"].preflight_ok is True
    assert kwargs["auto_start"] is True
