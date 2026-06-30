import importlib.util
from pathlib import Path

import pytest


SCRIPT_PATH = Path(__file__).resolve().parents[3] / "scripts" / "run_timed_patrol_demo.py"
EXAMPLE_HOST = "203.0.113.10"
EXAMPLE_PORT = "12000"


def target_args() -> list[str]:
    return ["--host", EXAMPLE_HOST, "--port", EXAMPLE_PORT]


def load_script():
    spec = importlib.util.spec_from_file_location("run_timed_patrol_demo", SCRIPT_PATH)
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


def test_dry_run_is_default_and_builds_two_lane_shuttle_plan():
    script = load_script()
    args = script.parse_args([*target_args()])
    script.validate_args(args)

    plan = script.build_plan(args)

    assert args.execute is False
    assert [(segment.name, segment.vx, segment.wz) for segment in plan] == [
        ("lane_1_forward", 0.2, 0.0),
        ("lane_2_reverse", -0.2, 0.0),
    ]


def test_lane_count_one_moves_one_forward_segment_only():
    script = load_script()
    args = script.parse_args(["--lane-count", "1", *target_args()])

    plan = script.build_plan(args)

    assert [(segment.name, segment.vx, segment.wz) for segment in plan] == [
        ("lane_1_forward", 0.2, 0.0),
    ]


def test_execute_requires_all_physical_ready_flags():
    script = load_script()
    args = script.parse_args(["--execute", *target_args()])

    with pytest.raises(SystemExit, match="--preflight-ok"):
        script.validate_args(args)

    args = script.parse_args(["--execute", "--preflight-ok", *target_args()])
    with pytest.raises(SystemExit, match="--auto-mode-ok"):
        script.validate_args(args)

    args = script.parse_args([
        "--execute",
        "--preflight-ok",
        "--auto-mode-ok",
        *target_args(),
    ])
    with pytest.raises(SystemExit, match="--stand-ready-ok"):
        script.validate_args(args)


def test_execute_rejects_fast_or_long_segments_without_overrides():
    script = load_script()
    ready_flags = ["--execute", "--preflight-ok", "--auto-mode-ok", "--stand-ready-ok"]

    args = script.parse_args([*ready_flags, "--vx", "0.31", *target_args()])
    with pytest.raises(SystemExit, match="--allow-fast"):
        script.validate_args(args)

    args = script.parse_args([*ready_flags, "--forward-sec", "2.1", *target_args()])
    with pytest.raises(SystemExit, match="--allow-long-segment"):
        script.validate_args(args)


def test_run_timed_patrol_sends_stops_between_segments_and_final_stop():
    script = load_script()
    args = script.parse_args([
        "--execute",
        "--preflight-ok",
        "--auto-mode-ok",
        "--stand-ready-ok",
        "--lane-count",
        "2",
        "--vx",
        "0.1",
        "--forward-sec",
        "0.11",
        "--turn-wz",
        "0.1",
        "--turn-sec",
        "0.06",
        "--send-period-sec",
        "0.05",
        *target_args(),
    ])
    script.validate_args(args)
    fake = FakeDriver(args.host, args.port, script.MotionLimits())

    script.run_timed_patrol(
        args,
        driver_factory=lambda host, port, limits: fake,
        sleep=lambda _dt: None,
    )

    assert fake.stops == [
        (10, 0.05),
        (10, 0.05),
        (10, 0.05),
        (60, 0.05),
    ]
    assert fake.commands == [
        (0.1, 0.0, 0.0),
        (0.1, 0.0, 0.0),
        (0.1, 0.0, 0.0),
        (-0.1, 0.0, 0.0),
        (-0.1, 0.0, 0.0),
        (-0.1, 0.0, 0.0),
    ]
    assert fake.closed is True
