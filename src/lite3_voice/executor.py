"""State-gated executor for the fixed Lite3 voice-action allowlist."""
from __future__ import annotations

import json
import logging
import math
import os
import signal
import time
from pathlib import Path

from lite3_common.types import MotionLimits
from lite3_control.udp_driver import CMD_FLAT_GAIT_MIDDLE, CMD_FLAT_GAIT_SLOW, CMD_HELLO, CMD_MANUAL_MODE, CMD_MOONWALK, CMD_NAVIGATION_MODE, CMD_STAND_SIT, CMD_STOP_ACTION, Lite3UdpDriver

from .policy import SITTING, STANDING, PlannedStep, plan

LOG = logging.getLogger("lite3_voice.executor")
SEND_PERIOD_SEC = 0.05
VOICE_FORWARD_DISTANCE_M = 1.0
NAVIGATION_MODE_SETTLE_SEC = 0.50
VOICE_LINEAR_SPEED_MPS = 0.50
VOICE_TURN_SPEED_RADPS = 1.45


class StateUnavailable(RuntimeError):
    pass


class MotionStateFile:
    def __init__(self, path: str | Path, stale_after_sec: float = 0.50):
        self.path = Path(path)
        self.stale_after_sec = stale_after_sec

    def read(self) -> dict:
        try:
            value = json.loads(self.path.read_text(encoding="utf-8"))
            age = time.time() - float(value["written_at_unix"])
        except (OSError, KeyError, ValueError, json.JSONDecodeError) as exc:
            raise StateUnavailable("state snapshot unavailable") from exc
        if age < 0.0 or age > self.stale_after_sec:
            raise StateUnavailable("state snapshot stale: {:.3f}s".format(age))
        return value

    def wait_basic(self, desired: int, timeout_sec: float) -> dict:
        deadline = time.monotonic() + timeout_sec
        last_error: Exception | None = None
        while time.monotonic() < deadline:
            try:
                state = self.read()
                if int(state["robot_basic_state"]) == desired:
                    return state
            except StateUnavailable as exc:
                last_error = exc
            time.sleep(0.05)
        raise StateUnavailable("timed out waiting for state {} ({})".format(desired, last_error))


def _yaw_delta(previous: float, current: float) -> float:
    return (current - previous + math.pi) % (2.0 * math.pi) - math.pi


class VoiceActionExecutor:
    def __init__(self, state_file: str, host: str, port: int):
        self.state = MotionStateFile(state_file)
        self.driver = Lite3UdpDriver(host, port, MotionLimits(max_vx_mps=VOICE_LINEAR_SPEED_MPS, max_vy_mps=0.05, max_wz_radps=VOICE_TURN_SPEED_RADPS))

    def close(self) -> None:
        self.driver.close()

    def execute(self, steps: list[dict]) -> None:
        # Read before sending any packet. This also rejects a stopped receiver.
        self.state.read()
        if steps and steps[0].get("action_id") in {
            "move_forward", "move_backward", "turn_left_full", "turn_right_full", "moonwalk",
        } and int(self.state.read()["robot_basic_state"]) == SITTING:
            self._posture("stand_up")
        for raw_step in steps:
            step = PlannedStep(**raw_step)
            if step.kind == "posture":
                self._posture(step.action_id)
            elif step.kind == "velocity":
                self._velocity(step.action_id)
            elif step.kind == "turn":
                self._turn(step.action_id)
            elif step.kind == "simple":
                self._simple(step.action_id)
            else:
                raise ValueError("unsupported planned step: {}".format(step.kind))

    def execute_action(self, action_id: str) -> None:
        """Plan against the live state, never the resolver's earlier snapshot."""
        basic_state = int(self.state.read()["robot_basic_state"])
        steps = [step.__dict__ for step in plan(action_id, basic_state)]
        self.execute(steps)

    def _posture(self, action_id: str) -> None:
        target = STANDING if action_id == "stand_up" else SITTING
        if int(self.state.read()["robot_basic_state"]) == target:
            return
        self.driver.send_simple_command(CMD_STAND_SIT)
        self.state.wait_basic(target, timeout_sec=15.0)

    def _require_standing(self) -> None:
        if int(self.state.read()["robot_basic_state"]) != STANDING:
            raise StateUnavailable("action requires stable standing state")

    def _velocity(self, action_id: str) -> None:
        self._require_standing()
        self._enter_navigation_mode()
        if action_id == "move_forward":
            self._move_forward_distance(VOICE_FORWARD_DISTANCE_M)
            return
        self._stream(-VOICE_LINEAR_SPEED_MPS, 0.0, 0.0, duration_sec=2.0)

    def _move_forward_distance(self, distance_m: float) -> None:
        """Move one bounded, odometry-measured voice-command segment."""
        if distance_m <= 0.0:
            raise ValueError("voice forward distance must be positive")
        self._enter_medium_navigation_mode()
        start = self.state.read()["pos_world"]
        if not isinstance(start, list) or len(start) < 2:
            raise StateUnavailable("position state unavailable")
        start_x, start_y = float(start[0]), float(start[1])
        deadline = time.monotonic() + 15.0
        self.driver.stop(repeat=10, dt_sec=SEND_PERIOD_SEC)
        try:
            while time.monotonic() < deadline:
                self._require_standing()
                position = self.state.read()["pos_world"]
                travelled = math.hypot(float(position[0]) - start_x, float(position[1]) - start_y)
                if travelled >= distance_m:
                    return
                self.driver.send_cmd_vel(VOICE_LINEAR_SPEED_MPS, 0.0, 0.0)
                time.sleep(SEND_PERIOD_SEC)
            raise StateUnavailable("forward motion timed out before 1.0 m")
        finally:
            self.driver.stop(repeat=20, dt_sec=SEND_PERIOD_SEC)
            self.driver.send_simple_command(CMD_FLAT_GAIT_SLOW)

    def _turn(self, action_id: str) -> None:
        self._require_standing()
        self._enter_navigation_mode()
        # Lite3UdpDriver negates yaw on the wire.  These signs therefore map
        # to physical left/right in the same convention as Coyote search.
        wz = VOICE_TURN_SPEED_RADPS if action_id == "turn_left_full" else -VOICE_TURN_SPEED_RADPS
        start = float(self.state.read()["yaw_rad"])
        previous, turned = start, 0.0
        deadline = time.monotonic() + 45.0
        try:
            while turned < 2.0 * math.pi and time.monotonic() < deadline:
                state = self.state.read()
                current = float(state["yaw_rad"])
                turned += abs(_yaw_delta(previous, current))
                previous = current
                self.driver.send_cmd_vel(0.0, 0.0, wz)
                time.sleep(SEND_PERIOD_SEC)
            if turned < 2.0 * math.pi:
                raise StateUnavailable("turn timed out before one complete rotation")
        finally:
            self.driver.stop(repeat=20, dt_sec=SEND_PERIOD_SEC)

    def _stream(self, vx: float, vy: float, wz: float, *, duration_sec: float) -> None:
        self.driver.stop(repeat=10, dt_sec=SEND_PERIOD_SEC)
        deadline = time.monotonic() + duration_sec
        try:
            while time.monotonic() < deadline:
                self._require_standing()
                self.driver.send_cmd_vel(vx, vy, wz)
                time.sleep(SEND_PERIOD_SEC)
        finally:
            self.driver.stop(repeat=20, dt_sec=SEND_PERIOD_SEC)

    def _enter_navigation_mode(self) -> None:
        """Select the Motion Host mode that accepts direct velocity commands."""
        self.driver.send_simple_command(CMD_NAVIGATION_MODE)
        time.sleep(NAVIGATION_MODE_SETTLE_SEC)

    def _enter_medium_navigation_mode(self) -> None:
        """Coyote-compatible gait transition for the bounded voice forward action."""
        self.driver.send_simple_command(CMD_MANUAL_MODE)
        time.sleep(NAVIGATION_MODE_SETTLE_SEC)
        self.driver.send_simple_command(CMD_FLAT_GAIT_MIDDLE)
        time.sleep(NAVIGATION_MODE_SETTLE_SEC)
        self._enter_navigation_mode()

    def _simple(self, action_id: str) -> None:
        if action_id == "stop":
            self.driver.stop(repeat=20, dt_sec=SEND_PERIOD_SEC)
            self.driver.send_simple_command(CMD_STOP_ACTION)
        elif action_id == "hello":
            self.driver.send_simple_command(CMD_HELLO)
        elif action_id == "moonwalk":
            self._require_standing()
            self.driver.send_simple_command(CMD_MOONWALK)
        else:
            raise ValueError("unsupported simple action: {}".format(action_id))


def run(executor: VoiceActionExecutor, event_path: str) -> int:
    stopping = False
    def stop(_signum: int, _frame: object) -> None:
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    LOG.info("executor listening: events=%s", event_path)
    with open(event_path, "a+", encoding="utf-8") as stream:
        stream.seek(0, os.SEEK_END)
        while not stopping:
            line = stream.readline()
            if not line:
                time.sleep(0.10)
                continue
            try:
                event = json.loads(line)
                if event.get("type") != "voice_action" or not event.get("accepted"):
                    continue
                executor.execute_action(event["action_id"])
                LOG.info("VOICE_EXECUTED action=%s", event.get("action_id"))
            except (KeyError, ValueError, StateUnavailable, OSError) as exc:
                LOG.error("VOICE_REJECTED %s", exc)
    return 0
