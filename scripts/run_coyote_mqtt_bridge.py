#!/usr/bin/env python3
"""Run the Foxy coyote ROS/spool to MQTT bridge."""

from __future__ import annotations

import argparse
import json
import logging
import math
import os
import queue
import signal
import sys
import threading
import time
from typing import Callable, Optional
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
sys.path.insert(0, str(SRC))

from lite3_common.config import (  # noqa: E402
    load_lite3_network_config,
    load_motion_limits_config,
)
from lite3_control.udp_driver import (  # noqa: E402
    CMD_FLAT_GAIT_FAST,
    CMD_FLAT_GAIT_SLOW,
    CMD_HELLO,
    CMD_LONG_JUMP,
    CMD_MANUAL_MODE,
    CMD_NAVIGATION_MODE,
    CMD_STAND_SIT,
    Lite3UdpDriver,
)
from lite3_mqtt.client import MqttConfig, PahoMqttClient  # noqa: E402
from lite3_mqtt.contract import (  # noqa: E402
    DetectionType,
    PatrolAction,
    Topics,
    build_coyote_complete_payload,
    parse_detection_trigger,
    parse_patrol_command,
)
from lite3_mqtt.coyote_bridge import (  # noqa: E402
    COYOTE_CONTROL_HZ,
    COYOTE_FORWARD_SPEED_MPS,
    COYOTE_SEARCH_ADVANCE_M,
    COYOTE_SEARCH_TURN_SPEED_RADPS,
    COYOTE_STATUS_TIMEOUT_SEC,
    COYOTE_TURN_SPEED_RADPS,
    CoyoteMediaWorker,
    CoyoteMotionController,
    CoyoteSpoolReader,
)
from lite3_mqtt.media import DetectionMediaPublisher  # noqa: E402
from lite3_motion.direct_return import (  # noqa: E402
    DirectReturnConfig,
    DirectReturnExecutor,
)
from lite3_motion.local_avoidance import LocalAvoidancePolicy  # noqa: E402
from lite3_motion.local_return import (  # noqa: E402
    TargetObservation,
    calculate_return_vector,
)
from lite3_ros.coyote_bridge_rclpy_node import (  # noqa: E402
    CoyoteMqttBridgeNode,
    CoyoteMotionOutputNode,
    CoyoteSpoolAdapterNode,
)


DEFAULT_SPOOL_DIR = "/home/ubuntu/iq9_coyote/outputs/spool"
DEFAULT_REALSENSE_REQUEST_DIR = "/home/ubuntu/iq9_coyote/outputs/realsense_requests"
DEFAULT_AUDIO_REQUEST_DIR = "/home/ubuntu/iq9_coyote/audio_requests"
BARK_HEIGHT_RATIO = 0.15
TTS_MISSION_STARTED = "Coyote search mission started."
TTS_MISSION_COMPLETE = "Coyote has been deterred. Returning home."
TTS_HOME_ARRIVED = "Mission complete. I am home."
# Demo default: Long Jump is mechanically non-deterministic on this setup.
# Keep its implementation available, but never enter it from the coyote flow.
ENABLE_LONG_JUMP = False


class CoyoteAudioCue:
    """Event-scoped file requests for the IQ9 host audio service.

    Keeping requests in a shared directory avoids coupling ROS/Docker to a
    host audio backend. The same protocol supports future ``speak`` requests.
    """

    def __init__(self, request_dir: str, *, logger: logging.Logger) -> None:
        self.request_dir = Path(request_dir)
        self.logger = logger
        self._lock = threading.Lock()
        self._armed_event_id = None
        self._started_event_ids = set()
        self._request_sequence = 0

    def arm(self, event_id: str) -> None:
        with self._lock:
            self._armed_event_id = event_id

    def observe(self, status) -> None:
        if not (
            status.detect == "detected"
            and status.side == "center"
            and status.height_ratio >= BARK_HEIGHT_RATIO
        ):
            return
        with self._lock:
            event_id = self._armed_event_id
            if not event_id or event_id in self._started_event_ids:
                return
            self._started_event_ids.add(event_id)
        self._request("start_loop", event_id, cue="dog_bark")
        self.logger.info(
            "coyote bark cue started event_id=%s height_ratio=%.3f threshold=%.2f",
            event_id, status.height_ratio, BARK_HEIGHT_RATIO,
        )

    def complete(self, event_id: str) -> None:
        with self._lock:
            if self._armed_event_id == event_id:
                self._armed_event_id = None
            should_stop = event_id in self._started_event_ids
            self._started_event_ids.discard(event_id)
        if should_stop:
            self._request("stop", event_id)
            self.logger.info("coyote bark cue stopped event_id=%s", event_id)

    def speak(self, event_id: str, text: str) -> None:
        """Queue mission speech; host audio service gives it priority over bark."""
        self._request("speak", event_id, text=text)
        self.logger.info("coyote TTS requested event_id=%s text=%s", event_id, text)

    def speak_and_wait(self, event_id: str, text: str, timeout_sec: float = 30.0) -> None:
        """Wait for host acknowledgement before allowing posture movement."""
        self.request_dir.mkdir(parents=True, exist_ok=True)
        completion_path = self.request_dir / "{}-tts-complete-{}.done".format(
            event_id, int(time.time() * 1000)
        )
        self._request(
            "speak",
            event_id,
            text=text,
            completion_path=str(completion_path),
        )
        self.logger.info("coyote TTS waiting before Stand event_id=%s text=%s", event_id, text)
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if completion_path.is_file():
                try:
                    response = json.loads(completion_path.read_text(encoding="utf-8"))
                finally:
                    completion_path.unlink(missing_ok=True)
                if response.get("status") == "completed":
                    self.logger.info("coyote TTS completed before Stand event_id=%s", event_id)
                    return
                raise RuntimeError("TTS playback failed before Stand: {}".format(response))
            time.sleep(0.05)
        raise TimeoutError("timed out waiting for TTS before Stand")

    def prepare_tts(self, event_id: str, cue_id: str, text: str) -> None:
        """Start host CPU synthesis without blocking robot motion."""
        self._request("prepare_tts", event_id, cue_id=cue_id, text=text)
        self.logger.info("coyote TTS prepare requested event_id=%s cue_id=%s", event_id, cue_id)

    def play_prepared_and_wait(
        self,
        event_id: str,
        cue_id: str,
        timeout_sec: float = 30.0,
    ) -> None:
        """Wait only for already-prepared WAV playback, never synthesis."""
        completion_path = self.request_dir / "{}-tts-play-{}.done".format(
            event_id, int(time.time() * 1000)
        )
        self._request(
            "play_prepared_tts",
            event_id,
            cue_id=cue_id,
            completion_path=str(completion_path),
        )
        self._wait_for_tts_completion(completion_path, event_id, cue_id, timeout_sec)

    def play_prepared(self, event_id: str, cue_id: str) -> None:
        """Play a prepared phrase asynchronously; no robot-side delay."""
        completion_path = self.request_dir / "{}-tts-play-{}.done".format(
            event_id, int(time.time() * 1000)
        )
        self._request(
            "play_prepared_tts",
            event_id,
            cue_id=cue_id,
            completion_path=str(completion_path),
        )
        self.logger.info("coyote prepared TTS playback requested event_id=%s cue_id=%s", event_id, cue_id)

    def _wait_for_tts_completion(
        self,
        completion_path: Path,
        event_id: str,
        cue_id: str,
        timeout_sec: float,
    ) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if completion_path.is_file():
                try:
                    response = json.loads(completion_path.read_text(encoding="utf-8"))
                finally:
                    completion_path.unlink(missing_ok=True)
                if response.get("status") == "completed":
                    self.logger.info("coyote prepared TTS completed event_id=%s cue_id=%s", event_id, cue_id)
                    return
                raise RuntimeError("prepared TTS playback failed: {}".format(response))
            time.sleep(0.05)
        raise TimeoutError("timed out waiting for prepared TTS playback")

    def _request(self, action: str, event_id: str, **payload) -> None:
        self.request_dir.mkdir(parents=True, exist_ok=True)
        request = {"action": action, "event_id": event_id, **payload}
        with self._lock:
            self._request_sequence += 1
            sequence = self._request_sequence
        target = self.request_dir / "{:013d}-{:06d}-{}-{}.json".format(
            int(time.time() * 1000), sequence, action, event_id
        )
        temporary = target.with_suffix(".part")
        temporary.write_text(json.dumps(request, separators=(",", ":")), encoding="utf-8")
        os.replace(temporary, target)
COMPLETION_FAST_HOLD_SEC = 1.00
COMPLETION_FAST_MANUAL_SETTLE_SEC = 0.50
COMPLETION_MANUAL_SETTLE_SEC = 0.50
COMPLETION_ZERO_PULSE_SEC = 0.50
COMPLETION_SLOW_SETTLE_SEC = 0.50
COMPLETION_NAVIGATION_SETTLE_SEC = 1.00
COMPLETION_DIRECT_RELEASE_TIMEOUT_SEC = 1.00
# Match the coyote search turn rate for the post-completion 180-degree turn.
COMPLETION_DIRECT_TURN_WZ_RADPS = -COYOTE_SEARCH_TURN_SPEED_RADPS
COMPLETION_DIRECT_TURN_PERIOD_SEC = 1.0 / COYOTE_CONTROL_HZ
COMPLETION_DIRECT_TURN_STEPS = int(
    math.ceil(math.pi / abs(COMPLETION_DIRECT_TURN_WZ_RADPS) / COMPLETION_DIRECT_TURN_PERIOD_SEC)
)
COMPLETION_DIRECT_TURN_STOP_SEC = 0.30
# The prior 12 cm standoff became the next mission's newly captured "home",
# causing a consistent one-sided creep over repeated Coyote runs.
DIRECT_RETURN_HOME_STANDOFF_M = 0.04
COMPLETION_HELLO_SETTLE_SEC = 4.00
LONG_JUMP_SETTLE_SEC = 5.00
LONG_JUMP_MANUAL_SETTLE_SEC = 1.00
LONG_JUMP_STANDING_SETTLE_SEC = 1.00
LONG_JUMP_AI_STAND_SETTLE_SEC = 2.00
LONG_JUMP_STATIONARY_TIMEOUT_SEC = 8.00
LONG_JUMP_MAX_BODY_SPEED_MPS = 0.05
LONG_JUMP_MAX_YAW_SPEED_RADPS = 0.10
LONG_JUMP_START_TIMEOUT_SEC = 2.00
LONG_JUMP_POST_ACTION_SETTLE_SEC = 2.00
MISSION_STAND_SETTLE_SEC = 2.50
MISSION_POSTURE_STATE_TIMEOUT_SEC = 5.00
MISSION_HOME_SIT_TTS_DELAY_SEC = 0.00
ROBOT_BASIC_STATE_SITTING = 1
ROBOT_BASIC_STATE_PREPARING = 4
ROBOT_BASIC_STATE_SIT_TO_STAND = 5
ROBOT_BASIC_STATE_STANDING = 6
MISSION_STAND_RETRY_OBSERVE_SEC = 1.00


class DisabledMotionSink:
    def acquire(self) -> None:
        pass

    def send_cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        _ = vx, vy, wz

    def release(self) -> None:
        pass


class GatedUdpMotionSink:
    """Enable direct UDP velocity only while one coyote search owns it."""

    def __init__(self, driver: Lite3UdpDriver) -> None:
        self.driver = driver
        self._lock = threading.Lock()
        self._active = False
        self._released = threading.Event()
        self._released.set()

    def acquire(self) -> None:
        with self._lock:
            self._active = True
            self._released.clear()

    def send_cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        with self._lock:
            if not self._active:
                return
            self.driver.send_cmd_vel(vx, vy, wz)

    def release(self) -> None:
        # CoyoteMotionController sends the terminal zero before release().
        # Do not emit another packet or allow its later 20 Hz idle ticks out.
        with self._lock:
            self._active = False
            self._released.set()

    def wait_released(self, timeout_sec: float) -> bool:
        return self._released.wait(timeout_sec)


class RealSenseReturnStore:
    """Collect exactly two target observations from the existing QNN process."""

    def __init__(self, root: str, yaw_provider, *, logger: logging.Logger) -> None:
        self.root = Path(root)
        self.request_dir = self.root / "requests"
        self.response_dir = self.root / "responses"
        self.yaw_provider = yaw_provider
        self.logger = logger
        self._lock = threading.Lock()
        self._start = {}
        self._vector = {}

    def capture_start(self, event_id: str) -> None:
        with self._lock:
            if event_id in self._start:
                return
        observation = self._capture(event_id, "start")
        with self._lock:
            self._start[event_id] = observation
        self.logger.info(
            "coyote RealSense start captured event_id=%s forward=%.3f left=%.3f",
            event_id, observation.forward_m, observation.left_m,
        )

    def capture_stop_and_calculate(self, event_id: str):
        with self._lock:
            start = self._start.get(event_id)
        if start is None:
            raise RuntimeError("missing RealSense start observation")
        # A stop event is an instantaneous wide-camera decision, whereas a
        # RealSense RGB/depth pair can briefly be between frames.  Keep the
        # target-facing posture and retry short negative responses instead of
        # discarding a valid return-home vector on one frame.
        deadline = time.monotonic() + 5.0
        stop = None
        last_error = None
        while time.monotonic() < deadline:
            try:
                stop = self._capture(event_id, "stop")
                break
            except RuntimeError as exc:
                last_error = exc
                if "coyote is not detected" not in str(exc):
                    raise
                time.sleep(0.20)
        if stop is None:
            raise last_error or RuntimeError("RealSense stop observation timed out")
        vector = calculate_return_vector(start, stop)
        with self._lock:
            self._vector[event_id] = vector
        self.logger.info(
            "coyote return vector captured event_id=%s distance=%.3f x=%.3f y=%.3f",
            event_id, vector.distance_m, vector.x_m, vector.y_m,
        )
        return vector

    def vector_for(self, event_id: str):
        with self._lock:
            return self._vector.get(event_id)

    def _capture(self, event_id: str, phase: str) -> TargetObservation:
        request_id = "{}-{}-{}".format(event_id, phase, int(time.time() * 1000))
        self.request_dir.mkdir(parents=True, exist_ok=True)
        self.response_dir.mkdir(parents=True, exist_ok=True)
        target = self.request_dir / (request_id + ".json")
        temporary = self.request_dir / (request_id + ".part")
        temporary.write_text(json.dumps({"request_id": request_id}), encoding="utf-8")
        os.replace(temporary, target)
        response_path = self.response_dir / (request_id + ".json")
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            if response_path.exists():
                response = json.loads(response_path.read_text(encoding="utf-8"))
                if response.get("result") != "SUCCESS":
                    raise RuntimeError("RealSense observation failed: {}".format(response.get("reason")))
                # Pair the observation with the most recent direct Motion
                # Host pose *after* the RGB/depth response arrives.  Reading
                # yaw before writing the request could associate a moving
                # robot's later frame with an earlier heading.
                pose = self.yaw_provider()
                if pose is None:
                    raise RuntimeError("direct Motion Host yaw is unavailable")
                yaw = float(pose[2])
                self.logger.info(
                    "coyote RealSense %s paired event_id=%s forward=%.3f left=%.3f yaw=%.3f",
                    phase,
                    event_id,
                    float(response["forward_m"]),
                    float(response["left_m"]),
                    yaw,
                )
                return TargetObservation(
                    forward_m=float(response["forward_m"]),
                    left_m=float(response["left_m"]),
                    yaw_rad=yaw,
                )
            time.sleep(0.05)
        raise TimeoutError("timed out waiting for RealSense observation")


class DirectMotionHomeStore:
    """Session home anchor captured from IQ9 Motion Host Robot State."""

    def __init__(self, logger: logging.Logger) -> None:
        self._logger = logger
        self._lock = threading.Lock()
        self._home = {}
        self._anchor = None

    def capture(self, event_id: str, pose) -> None:
        if pose is None or len(pose) < 3 or not all(math.isfinite(float(value)) for value in pose[:3]):
            raise RuntimeError("direct Motion Host home pose is unavailable")
        home = (float(pose[0]), float(pose[1]), float(pose[2]))
        with self._lock:
            if self._anchor is None:
                self._anchor = home
                captured = True
            else:
                captured = False
            self._home[event_id] = self._anchor
            anchor = self._anchor
        if captured:
            self._logger.info(
                "coyote direct home anchor captured event_id=%s x=%.3f y=%.3f yaw=%.3f",
                event_id,
                anchor[0],
                anchor[1],
                anchor[2],
            )
        else:
            self._logger.info(
                "coyote direct home anchor reused event_id=%s x=%.3f y=%.3f yaw=%.3f",
                event_id,
                anchor[0],
                anchor[1],
                anchor[2],
            )

    def pose_for(self, event_id: str):
        with self._lock:
            return self._home.get(event_id)


class CompletionActionRoutine:
    """Run the coyote completion gesture before handing Nav2 back home."""

    def __init__(
        self,
        driver: Lite3UdpDriver,
        *,
        on_finished: Callable[[str], None],
        logger: logging.Logger,
        sleep: Callable[[float], None] = time.sleep,
        wait_for_direct_release: Callable[[float], bool] = lambda _timeout: True,
        wait_for_robot_basic_state: Callable[[int, float], bool] = (
            lambda _state, _timeout: True
        ),
        yaw_provider: Callable[[], float | None] = lambda: None,
        on_after_slow: Callable[[str], None] = lambda _event_id: None,
    ) -> None:
        self.driver = driver
        self.on_finished = on_finished
        self.logger = logger
        self.sleep = sleep
        self.wait_for_direct_release = wait_for_direct_release
        self.wait_for_robot_basic_state = wait_for_robot_basic_state
        self.yaw_provider = yaw_provider
        self.on_after_slow = on_after_slow
        self._lock = threading.Lock()
        self._active_event_ids = set()
        self._threads = []

    def start(self, event_id: str) -> bool:
        with self._lock:
            if event_id in self._active_event_ids:
                return False
            self._active_event_ids.add(event_id)
            worker = threading.Thread(
                target=self._run,
                args=(event_id,),
                name="lite3-coyote-completion-{}".format(event_id),
                daemon=False,
            )
            self._threads.append(worker)
            worker.start()
        return True

    def wait(self, timeout_sec: float) -> None:
        with self._lock:
            workers = list(self._threads)
        deadline = time.monotonic() + timeout_sec
        for worker in workers:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return
            worker.join(remaining)

    def _run(self, event_id: str) -> None:
        completed = False
        try:
            if not self.wait_for_direct_release(COMPLETION_DIRECT_RELEASE_TIMEOUT_SEC):
                raise RuntimeError("timed out waiting for coyote UDP output release")
            # COMPLETE has already stopped direct coyote motion.  Restore the
            # Manual-mode transition that precedes the Fast gait command.
            self.driver.send_simple_command(CMD_MANUAL_MODE)
            self.logger.info("coyote completion Manual mode requested event_id=%s", event_id)
            # Motion Host occasionally ignores a gait switch when it arrives
            # during the Manual-mode transition.  This settle applies only
            # to Fast gait; the rest of the completion choreography remains
            # unchanged.
            self.sleep(COMPLETION_FAST_MANUAL_SETTLE_SEC)
            self.driver.send_simple_command(CMD_FLAT_GAIT_FAST)
            self.logger.info("coyote completion Fast gait requested event_id=%s", event_id)
            self.sleep(COMPLETION_FAST_HOLD_SEC)

            # Send exactly two zero axis packets between Fast and Slow, then
            # stop all coyote-axis output before the completion actions.
            self.driver.send_cmd_vel(0.0, 0.0, 0.0)
            self.sleep(COMPLETION_ZERO_PULSE_SEC)
            self.driver.send_cmd_vel(0.0, 0.0, 0.0)
            self.sleep(COMPLETION_ZERO_PULSE_SEC)
            self.driver.send_simple_command(CMD_FLAT_GAIT_SLOW)
            self.logger.info("coyote completion Slow gait requested event_id=%s", event_id)
            self.sleep(COMPLETION_SLOW_SETTLE_SEC)
            self.on_after_slow(event_id)
            self.driver.send_simple_command(CMD_NAVIGATION_MODE)
            self.logger.info(
                "coyote completion Navigation mode requested for direct turn event_id=%s",
                event_id,
            )
            self.sleep(COMPLETION_NAVIGATION_SETTLE_SEC)
            yaw_before_turn = self.yaw_provider()
            if yaw_before_turn is None or not math.isfinite(yaw_before_turn):
                raise RuntimeError("direct Motion Host yaw is unavailable before Hello turn")
            # This used to be a fixed number of UDP packets.  Motion Host's
            # physical turn rate is not exactly the commanded rate, so use
            # its direct yaw feedback and stop at 180 degrees instead.
            DirectReturnExecutor(
                self.driver,
                self.yaw_provider,
                config=DirectReturnConfig(
                    turn_speed_radps=abs(COMPLETION_DIRECT_TURN_WZ_RADPS),
                ),
            ).spin_relative(math.pi)
            yaw_after_turn = self.yaw_provider()
            self.logger.info(
                "coyote completion Hello pre-turn complete event_id=%s yaw_before=%.3f yaw_after=%.3f",
                event_id,
                yaw_before_turn,
                yaw_after_turn if yaw_after_turn is not None else float("nan"),
            )
            self.sleep(COMPLETION_DIRECT_TURN_STOP_SEC)
            self.driver.send_simple_command(CMD_MANUAL_MODE)
            self.logger.info("coyote completion Manual mode requested for Hello event_id=%s", event_id)
            self.sleep(COMPLETION_MANUAL_SETTLE_SEC)
            # The robot is already standing after coyote tracking.  Hello is
            # a Manual-mode action; do not toggle Sit/Stand here because that
            # extra posture cycle adds delay and perturbs the return pose.
            self.driver.send_simple_command(CMD_HELLO)
            self.logger.info("coyote completion Hello requested from Manual mode event_id=%s", event_id)
            self.sleep(COMPLETION_HELLO_SETTLE_SEC)
            self.driver.send_simple_command(CMD_NAVIGATION_MODE)
            self.logger.info(
                "coyote completion Navigation mode requested for return-home event_id=%s",
                event_id,
            )
            self.sleep(COMPLETION_NAVIGATION_SETTLE_SEC)
            completed = True
        except Exception:
            self.logger.exception("coyote completion action failed event_id=%s", event_id)
        finally:
            if not completed:
                self.logger.error(
                    "coyote completion aborted before return-home event_id=%s",
                    event_id,
                )
                with self._lock:
                    self._active_event_ids.discard(event_id)
                return
            try:
                self.on_finished(event_id)
            except Exception:
                self.logger.exception(
                    "coyote completion return-home handoff failed event_id=%s",
                    event_id,
                )
            finally:
                with self._lock:
                    self._active_event_ids.discard(event_id)

class RobotMotionStateTracker:
    """Wait for documented Motion Host action states from direct Robot State."""

    def __init__(self) -> None:
        self._state = None
        self._state_since = None
        self._vel_body = None
        self._stationary_since = None
        self._condition = threading.Condition()

    def update(self, state: int) -> None:
        with self._condition:
            normalized = int(state)
            if self._state != normalized:
                self._state = normalized
                self._state_since = time.monotonic()
            self._condition.notify_all()

    def update_motion_payload(self, payload: dict) -> None:
        try:
            state = int(payload["robot_motion_state"])
            velocity = payload["vel_body"]
            if not isinstance(velocity, list) or len(velocity) != 3:
                raise ValueError("vel_body must contain three values")
            vx, vy, wz = (float(value) for value in velocity)
            if not all(math.isfinite(value) for value in (vx, vy, wz)):
                raise ValueError("vel_body must be finite")
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("invalid direct Motion Host state") from exc
        with self._condition:
            if self._state != state:
                self._state = state
                self._state_since = time.monotonic()
            self._vel_body = (vx, vy, wz)
            self._condition.notify_all()

    def wait_for_stable_state(
        self,
        expected_state: int,
        hold_sec: float,
        timeout_sec: float,
    ) -> bool:
        """Require one unchanged Motion Host state for a short settle period."""
        deadline = time.monotonic() + timeout_sec
        expected = int(expected_state)
        with self._condition:
            while True:
                now = time.monotonic()
                if (
                    self._state == expected
                    and self._state_since is not None
                    and now - self._state_since >= hold_sec
                ):
                    return True
                remaining = deadline - now
                if remaining <= 0.0:
                    return False
                self._condition.wait(min(remaining, 0.10))

    def wait_for_stationary(self, timeout_sec: float, hold_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        with self._condition:
            while True:
                now = time.monotonic()
                velocity = self._vel_body
                stationary = (
                    self._state == 0
                    and velocity is not None
                    and math.hypot(velocity[0], velocity[1]) <= LONG_JUMP_MAX_BODY_SPEED_MPS
                    and abs(velocity[2]) <= LONG_JUMP_MAX_YAW_SPEED_RADPS
                )
                if stationary:
                    if self._stationary_since is None:
                        self._stationary_since = now
                    if now - self._stationary_since >= hold_sec:
                        return True
                else:
                    self._stationary_since = None
                remaining = deadline - now
                if remaining <= 0.0:
                    return False
                self._condition.wait(min(remaining, 0.10))

    def wait_for(self, expected_state: int, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        with self._condition:
            while self._state != int(expected_state):
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False
                self._condition.wait(remaining)
            return True

    def wait_until_not(self, state: int, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        with self._condition:
            while self._state == int(state):
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False
                self._condition.wait(remaining)
            return True


class CoyoteMissionStartRoutine:
    """Toggle the documented Stand/Sit command around one coyote mission."""

    def __init__(
        self,
        driver: Lite3UdpDriver,
        *,
        on_search_ready: Callable[[str], None],
        logger: logging.Logger,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self.driver = driver
        self.on_search_ready = on_search_ready
        self.logger = logger
        self.sleep = sleep
        self._lock = threading.Condition()
        self._active_event_id = None
        self._stand_toggle_event_id = None
        self._robot_basic_state = None

    def update_robot_basic_state(self, state: int) -> None:
        """Receive Motion Host's RobotState.robot_basic_state through ROS."""
        with self._lock:
            self._robot_basic_state = int(state)
            self._lock.notify_all()

    def _wait_for_basic_state(self, states, timeout_sec: float):
        deadline = time.monotonic() + timeout_sec
        with self._lock:
            while self._robot_basic_state not in states:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._lock.wait(remaining)
            return self._robot_basic_state

    def _wait_for_any_basic_state(self, timeout_sec: float):
        """Wait for the first Motion Host posture sample without assuming its enum."""
        deadline = time.monotonic() + timeout_sec
        with self._lock:
            while self._robot_basic_state is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._lock.wait(remaining)
            return self._robot_basic_state

    def wait_for_robot_basic_state(self, state: int, timeout_sec: float) -> bool:
        return self._wait_for_basic_state({int(state)}, timeout_sec) == int(state)

    def current_robot_basic_state(self):
        with self._lock:
            return self._robot_basic_state

    def start(
        self,
        event_id: str,
        before_stand: Optional[Callable[[], None]] = None,
    ) -> bool:
        with self._lock:
            if self._active_event_id is not None:
                self.logger.info(
                    "coyote mission start ignored active_event_id=%s duplicate_event_id=%s",
                    self._active_event_id,
                    event_id,
                )
                return False
            self._active_event_id = event_id
        thread = threading.Thread(
            target=self._run,
            args=(event_id, before_stand),
            name="lite3-coyote-stand-{}".format(event_id),
            daemon=True,
        )
        thread.start()
        return True

    def sit_after_home(
        self,
        event_id: str,
        on_sit_confirmed: Optional[Callable[[], None]] = None,
    ) -> None:
        with self._lock:
            if self._active_event_id != event_id:
                self.logger.info(
                    "coyote mission Sit ignored active_event_id=%s event_id=%s",
                    self._active_event_id,
                    event_id,
                )
                return
        try:
            with self._lock:
                should_toggle_sit = self._stand_toggle_event_id == event_id
            if should_toggle_sit:
                self.driver.send_simple_command(CMD_MANUAL_MODE)
                self.sleep(COMPLETION_MANUAL_SETTLE_SEC)
                self.driver.send_simple_command(CMD_STAND_SIT)
                state = self._wait_for_basic_state(
                    {ROBOT_BASIC_STATE_SITTING},
                    MISSION_POSTURE_STATE_TIMEOUT_SEC,
                )
                if state == ROBOT_BASIC_STATE_SITTING:
                    self.logger.info(
                        "coyote mission Sit confirmed at home event_id=%s",
                        event_id,
                    )
                    if MISSION_HOME_SIT_TTS_DELAY_SEC > 0.0:
                        self.sleep(MISSION_HOME_SIT_TTS_DELAY_SEC)
                    if on_sit_confirmed is not None:
                        on_sit_confirmed()
                else:
                    self.logger.error(
                        "coyote mission Sit was not confirmed at home event_id=%s",
                        event_id,
                    )
        finally:
            with self._lock:
                if self._active_event_id == event_id:
                    self._active_event_id = None
                if self._stand_toggle_event_id == event_id:
                    self._stand_toggle_event_id = None

    def _run(self, event_id: str, before_stand: Optional[Callable[[], None]]) -> None:
        try:
            self.driver.send_cmd_vel(0.0, 0.0, 0.0)
            if before_stand is not None:
                before_stand()
            # Motion Host V1.1 defines Stand/Sit as a toggle command; it does
            # not define RobotState numeric posture values.  A coyote mission
            # starts from Sit by contract, so pair this toggle with one Sit
            # toggle after RETURN_HOME for the same event.
            state = self.current_robot_basic_state()
            if state != ROBOT_BASIC_STATE_STANDING:
                for attempt in (1, 2):
                    self.driver.send_simple_command(CMD_STAND_SIT)
                    with self._lock:
                        if self._active_event_id == event_id:
                            self._stand_toggle_event_id = event_id
                    self.logger.info(
                        "coyote mission Stand requested event_id=%s attempt=%d",
                        event_id,
                        attempt,
                    )
                    state = self._wait_for_basic_state(
                        {
                            ROBOT_BASIC_STATE_PREPARING,
                            ROBOT_BASIC_STATE_SIT_TO_STAND,
                            ROBOT_BASIC_STATE_STANDING,
                        },
                        MISSION_STAND_RETRY_OBSERVE_SEC,
                    )
                    if state in {
                        ROBOT_BASIC_STATE_PREPARING,
                        ROBOT_BASIC_STATE_SIT_TO_STAND,
                    }:
                        state = self._wait_for_basic_state(
                            {ROBOT_BASIC_STATE_STANDING},
                            MISSION_POSTURE_STATE_TIMEOUT_SEC,
                        )
                    if state == ROBOT_BASIC_STATE_STANDING:
                        break
                    self.logger.warning(
                        "coyote mission Stand command was not observed event_id=%s attempt=%d",
                        event_id,
                        attempt,
                    )
            if state != ROBOT_BASIC_STATE_STANDING:
                raise RuntimeError("timed out waiting for robot standing state")
            self.logger.info("coyote mission Stand confirmed event_id=%s", event_id)
            # Motion Host applies velocity UDP commands only in navigation mode.
            self.driver.send_simple_command(CMD_NAVIGATION_MODE)
            self.on_search_ready(event_id)
        except Exception:
            self.logger.exception("coyote mission start failed event_id=%s", event_id)
            with self._lock:
                if self._active_event_id == event_id:
                    self._active_event_id = None
                if self._stand_toggle_event_id == event_id:
                    self._stand_toggle_event_id = None


def parse_args(argv=None) -> argparse.Namespace:
    network = load_lite3_network_config(ROOT)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--broker-host", default=os.environ.get("MQTT_HOST", "127.0.0.1"))
    parser.add_argument(
        "--broker-port",
        type=int,
        default=int(os.environ.get("MQTT_PORT", "1883")),
    )
    parser.add_argument("--client-id", default="lite3-coyote-media-bridge")
    parser.add_argument("--username", default=os.environ.get("MQTT_USER") or None)
    parser.add_argument("--password", default=os.environ.get("MQTT_PASS") or None)
    parser.add_argument(
        "--spool-dir",
        default=os.environ.get("COYOTE_SPOOL_DIR", DEFAULT_SPOOL_DIR),
    )
    parser.add_argument("--control-hz", type=float, default=COYOTE_CONTROL_HZ)
    parser.add_argument(
        "--status-timeout-sec",
        type=float,
        default=COYOTE_STATUS_TIMEOUT_SEC,
    )
    parser.add_argument(
        "--forward-speed-mps",
        type=float,
        default=None,
    )
    parser.add_argument(
        "--turn-speed-radps",
        type=float,
        default=None,
    )
    parser.add_argument(
        "--search-turn-speed-radps",
        type=float,
        default=None,
    )
    parser.add_argument(
        "--search-advance-m",
        type=float,
        default=COYOTE_SEARCH_ADVANCE_M,
    )
    parser.add_argument(
        "--motion-output",
        choices=("disabled", "ros", "udp"),
        default="disabled",
    )
    parser.add_argument("--motion-host", default=network.motion_host_ip)
    parser.add_argument("--motion-port", type=int, default=network.motion_host_command_port)
    parser.add_argument("--allow-robot-motion", action="store_true")
    parser.add_argument("--preflight-ok", action="store_true")
    parser.add_argument("--auto-mode-ok", action="store_true")
    parser.add_argument("--exclusive-motion-ok", action="store_true")
    parser.add_argument("--run-seconds", type=float, default=0.0)
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args(argv)
    limits = load_motion_limits_config(ROOT)
    if args.forward_speed_mps is None:
        args.forward_speed_mps = COYOTE_FORWARD_SPEED_MPS
    if args.turn_speed_radps is None:
        args.turn_speed_radps = COYOTE_TURN_SPEED_RADPS
    if args.search_turn_speed_radps is None:
        args.search_turn_speed_radps = COYOTE_SEARCH_TURN_SPEED_RADPS
    return args


def validate_args(args: argparse.Namespace) -> None:
    limits = load_motion_limits_config(ROOT)
    forward_speed_limit = (
        COYOTE_FORWARD_SPEED_MPS
        if args.motion_output != "udp"
        else limits.max_vx_mps
    )
    turn_speed_limit = (
        COYOTE_TURN_SPEED_RADPS
        if args.motion_output != "udp"
        else limits.max_wz_radps
    )
    search_turn_speed_limit = (
        COYOTE_SEARCH_TURN_SPEED_RADPS
        if args.motion_output != "udp"
        else limits.max_wz_radps
    )
    if not math.isfinite(args.control_hz) or args.control_hz < 20.0:
        raise SystemExit("--control-hz must be finite and at least 20")
    if (
        not math.isfinite(args.status_timeout_sec)
        or args.status_timeout_sec <= 0.0
        or args.status_timeout_sec > 1.0
    ):
        raise SystemExit("--status-timeout-sec must be in (0, 1.0]")
    if (
        not math.isfinite(args.forward_speed_mps)
        or args.forward_speed_mps <= 0.0
        or args.forward_speed_mps > forward_speed_limit
    ):
        raise SystemExit(
            "--forward-speed-mps must be positive and no greater than {}".format(
                forward_speed_limit
            )
        )
    if (
        not math.isfinite(args.turn_speed_radps)
        or args.turn_speed_radps <= 0.0
        or args.turn_speed_radps > turn_speed_limit
    ):
        raise SystemExit(
            "--turn-speed-radps must be positive and no greater than {}".format(
                turn_speed_limit
            )
        )
    if (
        not math.isfinite(args.search_turn_speed_radps)
        or args.search_turn_speed_radps <= 0.0
        or args.search_turn_speed_radps > search_turn_speed_limit
    ):
        raise SystemExit(
            "--search-turn-speed-radps must be positive and no greater than "
            "{}".format(search_turn_speed_limit)
        )
    if (
        not math.isfinite(args.search_advance_m)
        or args.search_advance_m <= 0.0
        or args.search_advance_m > 1.00
    ):
        raise SystemExit("--search-advance-m must be in (0, 1.00]")
    if not math.isfinite(args.run_seconds) or args.run_seconds < 0.0:
        raise SystemExit("--run-seconds must be finite and non-negative")
    if args.motion_output == "ros" and not args.allow_robot_motion:
        raise SystemExit("ROS coyote motion requires --allow-robot-motion")
    if args.motion_output == "udp":
        missing = [
            flag
            for flag, enabled in (
                ("--allow-robot-motion", args.allow_robot_motion),
                ("--preflight-ok", args.preflight_ok),
                ("--auto-mode-ok", args.auto_mode_ok),
                ("--exclusive-motion-ok", args.exclusive_motion_ok),
            )
            if not enabled
        ]
        if missing:
            raise SystemExit(
                "coyote motion requires {}".format(", ".join(missing))
            )


def main(argv=None) -> int:
    args = parse_args(argv)
    validate_args(args)
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper()),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    logger = logging.getLogger("lite3-coyote-bridge")

    driver = None
    direct_motion_sink = None
    completion_driver = None
    completion_actions = None
    mission_start_actions = None
    media_client = None
    bridge_node = None
    search_events = queue.Queue(maxsize=32)

    def on_trigger(topic: str, payload: bytes) -> None:
        if topic == Topics.AUTO_PATROL:
            command = parse_patrol_command(payload)
            if (
                command.action is PatrolAction.EMERGENCY_STOP
                and bridge_node is not None
            ):
                bridge_node.motion_controller.handle_patrol_command(
                    command.action,
                    command.timestamp,
                )
                return
            item = ("patrol", command.action.value, command.timestamp)
        else:
            trigger = parse_detection_trigger(topic, payload)
            if trigger.detection_type is DetectionType.COYOTE:
                # Search begins in this IQ9 process.  Home capture remains in
                # the runtime for return-home only; it must not gate tracking.
                if mission_start_actions is None:
                    raise RuntimeError("coyote mission start routine is not ready")
                audio_cue.prepare_tts(
                    trigger.event_id,
                    "mission-start",
                    TTS_MISSION_STARTED,
                )
                if mission_start_actions.start(
                    trigger.event_id,
                    before_stand=lambda: audio_cue.play_prepared_and_wait(
                        trigger.event_id, "mission-start"
                    ),
                ):
                    logger.info(
                        "coyote MQTT trigger started locally event_id=%s",
                        trigger.event_id,
                    )
                return
            item = (
                "search",
                trigger.event_id,
                trigger.detection_type.value,
            )
        try:
            search_events.put_nowait(item)
        except queue.Full:
            if topic == Topics.AUTO_PATROL and bridge_node is not None:
                # Never bypass FIFO with an enabling command. A full QoS-0
                # control queue fails closed and requires a newer RESET.
                bridge_node.motion_controller.handle_patrol_command(
                    PatrolAction.EMERGENCY_STOP,
                    command.timestamp,
                )
                logger.error(
                    "coyote control queue full; emergency latched action=%s",
                    command.action.value,
                )
            else:
                logger.error("detection search trigger dropped: queue full")

    media_client = PahoMqttClient(
        MqttConfig(
            host=args.broker_host,
            port=args.broker_port,
            client_id=args.client_id,
            username=args.username,
            password=args.password,
            # This demo only arms coyote.  The published contracts for patrol
            # and glass-break remain defined, but cannot start motion here.
            subscriptions=(Topics.COYOTE_DETECT,),
        ),
        on_message=on_trigger,
        on_connection_lost=lambda: (
            bridge_node.motion_controller.emergency_stop()
            if bridge_node is not None
            else None
        ),
        logger=logger,
    )
    media_publisher = DetectionMediaPublisher(
        media_source=None,
        publish_json=media_client.publish_json,
        logger=logger,
    )
    reader = CoyoteSpoolReader(args.spool_dir)
    worker = CoyoteMediaWorker(reader, media_publisher, logger=logger)
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    stop = threading.Event()
    timer = None
    adapter_node = None
    motion_output_node = None
    motion = None
    executor = None
    client_started = False

    def request_stop(signum=None, frame=None) -> None:
        _ = signum, frame
        stop.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    if args.run_seconds > 0.0:
        timer = threading.Timer(args.run_seconds, request_stop)
        timer.daemon = True
        timer.start()

    try:
        media_client.start()
        client_started = True
        rclpy.init(args=None)
        # Feed direct Motion Host RobotState UDP (published locally by
        # lite3_motion_state_receiver) into CoyoteMotionController.  This
        # deliberately does not use perception-host /odom.
        motion_output_node = CoyoteMotionOutputNode()
        completion_driver = Lite3UdpDriver(
            args.motion_host,
            args.motion_port,
            load_motion_limits_config(ROOT),
        )

        return_store = RealSenseReturnStore(
            DEFAULT_REALSENSE_REQUEST_DIR,
            lambda: motion.latest_motion_pose() if motion is not None else None,
            logger=logger,
        )
        direct_home_store = DirectMotionHomeStore(logger)
        audio_cue = CoyoteAudioCue(DEFAULT_AUDIO_REQUEST_DIR, logger=logger)
        real_sense_capture_lock = threading.Lock()
        real_sense_capture_pending = set()
        real_sense_capture_started = set()
        long_jump_lock = threading.Lock()
        long_jump_event_ids = set()
        robot_motion_state = RobotMotionStateTracker()

        def handoff_return_home(event_id: str) -> None:
            # Synthesize while the robot drives home; Sit has no fixed
            # audio delay and simply starts the already-prepared WAV.
            audio_cue.prepare_tts(event_id, "mission-home", TTS_HOME_ARRIVED)
            home = direct_home_store.pose_for(event_id)
            vector = return_store.vector_for(event_id)
            if home is None and vector is None:
                # A wide-camera mission can complete even when the narrow
                # RealSense view did not see the target at mission start.
                # Never leave the robot standing because that optional return
                # vector was unavailable; skip only the direct return.
                logger.error(
                    "coyote direct return-home skipped: RealSense vector unavailable "
                    "event_id=%s",
                    event_id,
                )
                mission_start_actions.sit_after_home(
                    event_id,
                    on_sit_confirmed=lambda: audio_cue.play_prepared(
                        event_id, "mission-home"
                    ),
                )
                return
            # Completion runs in its own thread.  Do not share the mutable
            # tracking-policy state with the still-ticking coyote controller.
            return_avoidance_policy = (
                LocalAvoidancePolicy(motion.local_avoidance_policy.config)
                if motion is not None
                else None
            )
            executor = DirectReturnExecutor(
                completion_driver,
                lambda: (
                    motion.latest_motion_pose()[2]
                    if motion is not None and motion.latest_motion_pose() is not None
                    else None
                ),
                position_provider=lambda: (
                    motion.latest_motion_pose()[:2]
                    if motion is not None and motion.latest_motion_pose() is not None
                    else None
                ),
                clearance_provider=lambda: (
                    motion.latest_clearance_snapshot()
                    if motion is not None
                    else None
                ),
                avoidance_policy=return_avoidance_policy,
                config=DirectReturnConfig(
                    distance_tolerance_m=DIRECT_RETURN_HOME_STANDOFF_M,
                ),
                logger=logger,
            )
            if home is not None:
                logger.info(
                    "coyote direct return-home started event_id=%s home=(%.3f,%.3f)",
                    event_id,
                    home[0],
                    home[1],
                )
                final_position, final_error = executor.run_to_position(home[:2])
                logger.info(
                    "coyote direct home closed-loop arrived event_id=%s final=(%.3f,%.3f) error=%.3f",
                    event_id,
                    final_position[0],
                    final_position[1],
                    final_error,
                )
            else:
                logger.warning(
                    "coyote direct home unavailable; using RealSense fallback event_id=%s distance=%.3f",
                    event_id,
                    vector.distance_m,
                )
                executor.run(vector)
            logger.info("coyote direct return-home arrived event_id=%s", event_id)
            executor.spin_relative(math.pi)
            logger.info("coyote final 180-degree turn arrived event_id=%s", event_id)
            mission_start_actions.sit_after_home(
                event_id,
                on_sit_confirmed=lambda: audio_cue.play_prepared(
                    event_id, "mission-home"
                ),
            )

        def enqueue_coyote_search(event_id: str) -> None:
            try:
                search_events.put_nowait(
                    ("search", event_id, DetectionType.COYOTE.value)
                )
            except queue.Full:
                logger.error("coyote mission search dropped: queue full event_id=%s", event_id)

        def capture_then_enqueue_search(event_id: str) -> None:
            # Do not sample on the MQTT trigger: at that point the wide camera
            # may have the coyote at its edge while the narrower RealSense does
            # not see it.  The status observer below starts this capture only
            # once wide-camera tracking has centred the target.
            with real_sense_capture_lock:
                real_sense_capture_pending.add(event_id)
            try:
                direct_home_store.capture(
                    event_id,
                    motion.latest_motion_pose() if motion is not None else None,
                )
            except Exception:
                # Keep the existing RealSense vector return as a fallback.
                logger.exception(
                    "coyote direct home capture failed; RealSense fallback remains event_id=%s",
                    event_id,
                )
            audio_cue.arm(event_id)
            enqueue_coyote_search(event_id)

        def capture_when_target_centered(status) -> None:
            if not (
                status.detect == "detected"
                and status.motion == "forward"
                and status.side == "center"
            ):
                return
            with real_sense_capture_lock:
                candidates = real_sense_capture_pending - real_sense_capture_started
                if not candidates:
                    return
                event_id = next(iter(candidates))
                real_sense_capture_started.add(event_id)

            # This is asynchronous by design: coordinate capture must not
            # hold the 20 Hz RGB motion loop or delay the approach.
            def capture_start() -> None:
                try:
                    return_store.capture_start(event_id)
                except Exception:
                    logger.exception(
                        "coyote RealSense centered-start capture failed; "
                        "search continues event_id=%s",
                        event_id,
                    )
            threading.Thread(
                target=capture_start,
                name="lite3-coyote-rs-start-{}".format(event_id),
                daemon=True,
            ).start()

        def start_long_jump_when_ready(status) -> None:
            """Insert one documented Long Jump before final near completion."""
            if not status.long_jump_ready or motion is None:
                return
            event_id = motion.pause_for_external_action()
            if event_id is None:
                return
            with long_jump_lock:
                if event_id in long_jump_event_ids:
                    # A previous action for this event owns the pause. Restore
                    # direct tracking if this was a duplicate status update.
                    motion.resume_after_external_action(event_id)
                    return
                long_jump_event_ids.add(event_id)

            def run_long_jump() -> None:
                try:
                    if direct_motion_sink is not None and not direct_motion_sink.wait_released(1.0):
                        raise RuntimeError("timed out waiting for direct UDP release before Long Jump")
                    if (
                        mission_start_actions.current_robot_basic_state()
                        != ROBOT_BASIC_STATE_STANDING
                    ):
                        raise RuntimeError("Long Jump requires stable standing state")
                    # Motion Host accepts the verified Long Jump path only
                    # after Manual mode.  Keep the requested Slow-gait
                    # preparation, but do not add any posture action here.
                    completion_driver.send_simple_command(CMD_MANUAL_MODE)
                    logger.info("coyote Long Jump Manual mode requested event_id=%s", event_id)
                    time.sleep(COMPLETION_MANUAL_SETTLE_SEC)
                    completion_driver.send_simple_command(CMD_FLAT_GAIT_SLOW)
                    logger.info("coyote Long Jump Slow gait restored event_id=%s", event_id)
                    time.sleep(LONG_JUMP_STANDING_SETTLE_SEC)
                    logger.info(
                        "coyote Long Jump Slow-gait settle %.1fs event_id=%s",
                        LONG_JUMP_STANDING_SETTLE_SEC,
                        event_id,
                    )
                    completion_driver.send_simple_command(CMD_LONG_JUMP)
                    logger.info("coyote Long Jump requested event_id=%s", event_id)
                    if not robot_motion_state.wait_for(
                        11,
                        LONG_JUMP_START_TIMEOUT_SEC,
                    ):
                        raise RuntimeError(
                            "Motion Host did not confirm Long Jump state within {:.1f}s".format(
                                LONG_JUMP_START_TIMEOUT_SEC
                            )
                        )
                    logger.info("coyote Long Jump state confirmed event_id=%s", event_id)
                    if not robot_motion_state.wait_until_not(11, LONG_JUMP_SETTLE_SEC):
                        raise RuntimeError("Motion Host Long Jump did not finish in time")
                    logger.info("coyote Long Jump state completed event_id=%s", event_id)
                    basic_state = mission_start_actions.current_robot_basic_state()
                    if basic_state == ROBOT_BASIC_STATE_STANDING:
                        logger.info("coyote Long Jump already standing event_id=%s", event_id)
                    else:
                        raise RuntimeError(
                            "unexpected basic state after Long Jump: {}".format(basic_state)
                        )
                    # Match joystick behavior: keep all axis output paused
                    # after the landing so the robot can regain balance before
                    # Navigation mode and RGB tracking resume.
                    logger.info(
                        "coyote Long Jump landing settle %.1fs event_id=%s",
                        LONG_JUMP_POST_ACTION_SETTLE_SEC,
                        event_id,
                    )
                    time.sleep(LONG_JUMP_POST_ACTION_SETTLE_SEC)
                    if not motion.resume_after_external_action(event_id):
                        raise RuntimeError("coyote search no longer active after Long Jump")
                    logger.info("coyote Long Jump complete; tracking resumed event_id=%s", event_id)
                except Exception:
                    logger.exception("coyote Long Jump failed event_id=%s", event_id)
                    motion.stop()

            threading.Thread(
                target=run_long_jump,
                name="lite3-coyote-long-jump-{}".format(event_id),
                daemon=True,
            ).start()

        def on_coyote_status(status) -> None:
            capture_when_target_centered(status)
            audio_cue.observe(status)
            if ENABLE_LONG_JUMP:
                start_long_jump_when_ready(status)

        mission_start_actions = CoyoteMissionStartRoutine(
            completion_driver,
            on_search_ready=capture_then_enqueue_search,
            logger=logger,
        )
        if args.motion_output == "udp":
            driver = Lite3UdpDriver(
                args.motion_host,
                args.motion_port,
                load_motion_limits_config(ROOT),
            )
            direct_motion_sink = GatedUdpMotionSink(driver)
            motion_sink = direct_motion_sink
        elif args.motion_output == "ros":
            motion_sink = motion_output_node
        else:
            motion_sink = DisabledMotionSink()
        def speak_deterrence_after_slow(event_id: str) -> None:
            """Play only a prepared WAV after Fast/Slow has finished."""
            try:
                audio_cue.play_prepared_and_wait(event_id, "mission-complete")
            except Exception:
                # Audio failure must not strand the completed robot before
                # the existing turn and direct return-home sequence.
                logger.exception(
                    "coyote deterrence TTS failed; completion continues event_id=%s",
                    event_id,
                )

        completion_actions = CompletionActionRoutine(
            completion_driver,
            on_finished=handoff_return_home,
            logger=logger,
            wait_for_direct_release=(
                direct_motion_sink.wait_released
                if direct_motion_sink is not None
                else lambda _timeout: True
            ),
            wait_for_robot_basic_state=(
                mission_start_actions.wait_for_robot_basic_state
            ),
            yaw_provider=lambda: (
                motion.latest_motion_pose()[2]
                if motion is not None and motion.latest_motion_pose() is not None
                else None
            ),
            on_after_slow=speak_deterrence_after_slow,
        )
        def handle_coyote_complete_event(event_id: str, completion_reason: str) -> None:
            audio_cue.complete(event_id)
            if completion_reason == "TARGET_REACHED":
                audio_cue.prepare_tts(
                    event_id,
                    "mission-complete",
                    TTS_MISSION_COMPLETE,
                )
            payload = build_coyote_complete_payload(
                event_id=event_id,
                completion_reason=completion_reason,
            )
            media_client.publish_json(Topics.COYOTE_COMPLETE, payload)
            logger.info(
                "coyote COMPLETE published topic=%s event_id=%s reason=%s",
                Topics.COYOTE_COMPLETE,
                event_id,
                completion_reason,
            )
            if completion_reason == "TARGET_REACHED":
                if completion_actions is None:
                    raise RuntimeError("coyote completion action routine is not ready")
                # This callback is invoked while CoyoteMotionController owns
                # its lock.  Capturing needs the latest direct motion yaw, so
                # it must run after that lock is released; otherwise Fast/
                # Slow/Hello would never start due to a lock cycle.
                def capture_then_complete() -> None:
                    # Capture while the target is still in front, before the
                    # completion turn/Hello choreography changes robot heading.
                    try:
                        return_store.capture_stop_and_calculate(event_id)
                    except Exception:
                        # RealSense is supplementary to the normal RGB coyote
                        # mission. Its unavailable start/stop pair must not
                        # block completion after detected+stop.
                        logger.exception(
                            "coyote RealSense return vector unavailable; "
                            "completion continues event_id=%s",
                            event_id,
                        )
                    completion_actions.start(event_id)

                threading.Thread(
                    target=capture_then_complete,
                    name="lite3-coyote-complete-{}".format(event_id),
                    daemon=True,
                ).start()
                return
            # A completed search with no target has no gesture; return home
            # immediately while preserving the same internal mission handoff.
            handoff_return_home(event_id)

        def publish_coyote_complete(event_id: str, completion_reason: str) -> None:
            """Publish COMPLETE on ROS; the subscriber starts completion motion."""
            if bridge_node is None:
                raise RuntimeError("coyote ROS bridge is not ready")
            bridge_node.publish_coyote_complete_event(event_id, completion_reason)

        def publish_broken_cup_complete(event_id: str, completion_reason: str) -> None:
            if bridge_node is None:
                raise RuntimeError("coyote mission bridge is not ready")
            # Broken-cup completion is an internal mission handoff only. It
            # intentionally has no external MQTT COMPLETE topic.
            bridge_node.publish_return_home_request(event_id, source="broken_cup")
            logger.info(
                "broken-cup COMPLETE handed off event_id=%s reason=%s",
                event_id,
                completion_reason,
            )

        motion = CoyoteMotionController(
            motion_sink,
            forward_speed_mps=args.forward_speed_mps,
            turn_speed_radps=args.turn_speed_radps,
            search_turn_speed_radps=args.search_turn_speed_radps,
            search_advance_m=args.search_advance_m,
            timeout_sec=args.status_timeout_sec,
            # The map-free IQ9 demo uses direct Motion Host pose/yaw and
            # camera observations. Perception-host LiDAR is intentionally not
            # a runtime prerequisite for search, alignment or forward motion.
            require_scan=False,
            # Coyote owns /cmd_vel only after the Nav2 watchdog has cancelled
            # any previous Nav2 output and acknowledged the handoff.  Before
            # that ACK, the watchdog intentionally publishes zero velocity.
            ready=lambda: (
                media_client is not None
                and media_client.connected
                and (
                    args.motion_output in {"disabled", "udp"}
                    or (
                        motion_output_node is not None
                        and motion_output_node.motion_ready()
                    )
                )
            ),
            on_coyote_complete=publish_coyote_complete,
            on_broken_cup_complete=publish_broken_cup_complete,
            logger=logger,
        )
        motion_output_node.bind_controller(motion)
        adapter_node = CoyoteSpoolAdapterNode(
            reader=reader,
            media_ready=lambda: media_client.connected,
        )
        bridge_node = CoyoteMqttBridgeNode(
            reader=reader,
            media_worker=worker,
            motion_controller=motion,
            search_events=search_events,
            on_coyote_mission_start=mission_start_actions.start,
            on_coyote_home_reached=mission_start_actions.sit_after_home,
            on_robot_basic_state=mission_start_actions.update_robot_basic_state,
            on_robot_motion_state=robot_motion_state.update,
            on_motion_state=robot_motion_state.update_motion_payload,
            on_coyote_status=on_coyote_status,
            on_coyote_complete_event=handle_coyote_complete_event,
            control_hz=args.control_hz,
        )
        executor = SingleThreadedExecutor()
        executor.add_node(motion_output_node)
        executor.add_node(adapter_node)
        executor.add_node(bridge_node)
        logger.info(
            "coyote bridge ready broker=%s:%s spool=%s motion=%s",
            args.broker_host,
            args.broker_port,
            args.spool_dir,
            args.motion_output,
        )
        while rclpy.ok() and not stop.is_set():
            executor.spin_once(timeout_sec=0.1)
    finally:
        if timer is not None:
            timer.cancel()
        if completion_actions is not None:
            completion_actions.wait(
                COMPLETION_FAST_HOLD_SEC
                + COMPLETION_ZERO_PULSE_SEC * 2
                + COMPLETION_SLOW_SETTLE_SEC
                + COMPLETION_NAVIGATION_SETTLE_SEC * 2
                + COMPLETION_DIRECT_TURN_PERIOD_SEC * COMPLETION_DIRECT_TURN_STEPS
                + COMPLETION_DIRECT_TURN_STOP_SEC
                + COMPLETION_MANUAL_SETTLE_SEC
                + COMPLETION_HELLO_SETTLE_SEC
                + 1.0
            )
        cleanup_steps = []
        if executor is not None and motion_output_node is not None:
            cleanup_steps.append(
                (
                    "remove motion output node",
                    lambda: executor.remove_node(motion_output_node),
                )
            )
        if executor is not None and bridge_node is not None:
            cleanup_steps.append(
                ("remove bridge node", lambda: executor.remove_node(bridge_node))
            )
        if executor is not None and adapter_node is not None:
            cleanup_steps.append(
                ("remove adapter node", lambda: executor.remove_node(adapter_node))
            )
        if bridge_node is not None:
            cleanup_steps.append(("destroy bridge node", bridge_node.destroy_node))
        if adapter_node is not None:
            cleanup_steps.append(("destroy adapter node", adapter_node.destroy_node))
        if motion_output_node is not None:
            cleanup_steps.append(
                ("destroy motion output node", motion_output_node.destroy_node)
            )
        if executor is not None:
            cleanup_steps.append(("shutdown executor", executor.shutdown))
        if rclpy.ok():
            cleanup_steps.append(("shutdown rclpy", rclpy.shutdown))
        cleanup_steps.append(("close media worker", worker.close))
        if client_started:
            cleanup_steps.append(("stop MQTT client", media_client.stop))
        if driver is not None:
            cleanup_steps.append(
                ("send repeated final stop", lambda: driver.stop(10, 0.05))
            )
            cleanup_steps.append(("close motion driver", driver.close))
        if completion_driver is not None:
            cleanup_steps.append(
                ("close completion action driver", completion_driver.close)
            )
        for label, cleanup in cleanup_steps:
            try:
                cleanup()
            except Exception:
                logger.exception("cleanup failed: %s", label)
        stop.set()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
