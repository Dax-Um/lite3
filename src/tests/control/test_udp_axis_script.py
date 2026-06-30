import importlib.util
from pathlib import Path

import pytest


SCRIPT_PATH = Path(__file__).resolve().parents[3] / "scripts" / "run_lite3_udp_axis_test.py"
EXAMPLE_HOST = "203.0.113.10"
EXAMPLE_PORT = "12000"


def target_args() -> list[str]:
    return ["--host", EXAMPLE_HOST, "--port", EXAMPLE_PORT]


def load_script():
    spec = importlib.util.spec_from_file_location("run_lite3_udp_axis_test", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class FakeDriver:
    def __init__(self, host, port, limits):
        self.host = host
        self.port = port
        self.limits = limits
        self.commands = []
        self.stops = []
        self.closed = False

    def send_cmd_vel(self, vx, vy, wz):
        self.commands.append((vx, vy, wz))

    def stop(self, repeat, dt_sec):
        self.stops.append((repeat, dt_sec))

    def close(self):
        self.closed = True


def test_rejects_nonzero_command_without_preflight_ok():
    script = load_script()
    args = script.parse_args(["--axis", "vx", "--value", "0.03", *target_args()])

    with pytest.raises(SystemExit, match="--preflight-ok"):
        script.validate_args(args)


def test_rejects_nonzero_command_without_auto_mode_ok():
    script = load_script()
    args = script.parse_args([
        "--axis",
        "vx",
        "--value",
        "0.03",
        "--preflight-ok",
        *target_args(),
    ])

    with pytest.raises(SystemExit, match="--auto-mode-ok"):
        script.validate_args(args)


def test_allows_zero_command_without_preflight_ok():
    script = load_script()
    args = script.parse_args(["--axis", "vx", "--value", "0.0", *target_args()])

    script.validate_args(args)


def test_rejects_long_duration_without_override():
    script = load_script()
    args = script.parse_args([
        "--axis",
        "vx",
        "--value",
        "0.03",
        "--duration-sec",
        "1.5",
        "--preflight-ok",
        "--auto-mode-ok",
        *target_args(),
    ])

    with pytest.raises(SystemExit, match="--allow-long-test"):
        script.validate_args(args)


def test_rejects_axis_limit_violation():
    script = load_script()
    args = script.parse_args([
        "--axis",
        "vy",
        "--value",
        "0.06",
        "--preflight-ok",
        "--auto-mode-ok",
        *target_args(),
    ])

    with pytest.raises(SystemExit, match="limit"):
        script.validate_args(args)


def test_axis_command_maps_only_selected_axis():
    script = load_script()

    assert script.axis_command("vx", 0.03) == (0.03, 0.0, 0.0)
    assert script.axis_command("vy", -0.02) == (0.0, -0.02, 0.0)
    assert script.axis_command("wz", 0.05) == (0.0, 0.0, 0.05)


def test_run_axis_test_stops_before_and_after_command_loop(monkeypatch):
    script = load_script()
    args = script.parse_args([
        "--axis",
        "vx",
        "--value",
        "0.03",
        "--duration-sec",
        "0.11",
        "--preflight-ok",
        "--auto-mode-ok",
        *target_args(),
    ])
    script.validate_args(args)
    fake = FakeDriver(args.host, args.port, script.MotionLimits())

    times = iter([0.0, 0.0, 0.05, 0.10, 0.12])
    monkeypatch.setattr(script.time, "monotonic", lambda: next(times))
    monkeypatch.setattr(script.time, "sleep", lambda _dt: None)

    script.run_axis_test(args, driver_factory=lambda host, port, limits: fake)

    assert fake.stops == [(10, 0.05), (20, 0.05)]
    assert fake.commands == [(0.03, 0.0, 0.0)] * 3
    assert fake.closed is True
