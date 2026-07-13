from collections import deque

import pytest

from lite3_mqtt.perception_host import (
    CommandResult,
    PerceptionHostConfig,
    PerceptionHostNavManager,
    PerceptionHostStartupGate,
)


class FakeRunner:
    def __init__(self, results):
        self.results = deque(results)
        self.commands = []

    def run(self, command, timeout_sec):
        self.commands.append((command, timeout_sec))
        return self.results.popleft()


def ok():
    return CommandResult(0, "ready", "")


def failed(detail="not ready"):
    return CommandResult(1, "", detail)


def test_ready_navigation_only_checks_status():
    runner = FakeRunner([ok(), ok()])
    manager = PerceptionHostNavManager(PerceptionHostConfig(), runner=runner)

    manager.ensure_navigation()

    assert len(runner.commands) == 2
    assert "perception_host_nav_status.sh" in runner.commands[0][0][-1]
    assert "perception_host_start_watchdog.sh" in runner.commands[1][0][-1]
    assert "ysc@192.168.1.103" in runner.commands[0][0]


def test_not_ready_starts_existing_wrapper_then_polls_status():
    runner = FakeRunner([failed(), ok(), ok(), failed(), ok(), ok()])
    ticks = iter([0.0, 0.0, 1.0])
    manager = PerceptionHostNavManager(
        PerceptionHostConfig(ready_timeout_sec=10.0),
        runner=runner,
        sleep=lambda seconds: None,
        monotonic=lambda: next(ticks),
    )

    manager.ensure_navigation()

    remote_commands = [item[0][-1] for item in runner.commands]
    assert "perception_host_nav_status.sh" in remote_commands[0]
    assert "perception_host_start_lidar.sh" in remote_commands[1]
    assert "perception_host_start_navigation.sh" in remote_commands[2]
    assert "perception_host_nav_status.sh" in remote_commands[3]
    assert "perception_host_nav_status.sh" in remote_commands[4]
    assert "perception_host_start_watchdog.sh" in remote_commands[5]


def test_auto_start_can_be_disabled():
    manager = PerceptionHostNavManager(
        PerceptionHostConfig(auto_start_navigation=False),
        runner=FakeRunner([failed("offline")]),
    )

    with pytest.raises(RuntimeError, match="not ready"):
        manager.ensure_navigation()


def test_startup_gate_uses_existing_ros_graph_without_ssh():
    calls = []

    class Manager:
        def ensure_navigation(self):
            calls.append("ssh")

        def ensure_watchdog(self):
            calls.append("watchdog")

    class Backend:
        def wait_until_ready(self, timeout_sec=None):
            calls.append("ros")

    PerceptionHostStartupGate(Manager(), Backend()).ensure_ready()
    assert calls == ["ros", "watchdog"]


def test_startup_gate_starts_remote_nav_only_when_ros_graph_is_missing():
    calls = []

    class Manager:
        def ensure_navigation(self):
            calls.append("ssh")

    class Backend:
        def __init__(self):
            self.count = 0

        def wait_until_ready(self, timeout_sec=None):
            self.count += 1
            calls.append("ros")
            if self.count == 1:
                raise TimeoutError("not ready")

    PerceptionHostStartupGate(Manager(), Backend()).ensure_ready()
    assert calls == ["ros", "ssh", "ros"]
