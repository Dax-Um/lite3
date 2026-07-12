"""SSH orchestration for the Nav2 stack on the perception host."""

from __future__ import annotations

import logging
import shlex
import subprocess
import time
from dataclasses import dataclass
from pathlib import PurePosixPath
from typing import Callable, List, Optional, Protocol


@dataclass(frozen=True)
class CommandResult:
    returncode: int
    stdout: str
    stderr: str


class CommandRunner(Protocol):
    def run(self, command: List[str], timeout_sec: float) -> CommandResult:
        """Run one bounded command."""


class SubprocessCommandRunner:
    def run(self, command: List[str], timeout_sec: float) -> CommandResult:
        try:
            completed = subprocess.run(
                command,
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=timeout_sec,
            )
        except subprocess.TimeoutExpired as exc:
            raise TimeoutError("command timed out: {}".format(command)) from exc
        return CommandResult(
            returncode=completed.returncode,
            stdout=completed.stdout.decode("utf-8", errors="replace"),
            stderr=completed.stderr.decode("utf-8", errors="replace"),
        )


@dataclass(frozen=True)
class PerceptionHostConfig:
    host: str = "192.168.1.103"
    user: str = "ysc"
    remote_root: str = "/home/ysc/lite3"
    connect_timeout_sec: float = 5.0
    command_timeout_sec: float = 30.0
    ready_timeout_sec: float = 90.0
    poll_interval_sec: float = 2.0
    auto_start_navigation: bool = True


class PerceptionHostNavManager:
    """Reuse the existing perception_host_* wrappers over SSH."""

    def __init__(
        self,
        config: PerceptionHostConfig,
        *,
        runner: Optional[CommandRunner] = None,
        sleep: Callable[[float], None] = time.sleep,
        monotonic: Callable[[], float] = time.monotonic,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        self.config = config
        self.runner = runner or SubprocessCommandRunner()
        self.sleep = sleep
        self.monotonic = monotonic
        self.logger = logger or logging.getLogger(__name__)

    def ensure_navigation(self) -> None:
        status = self._remote_wrapper("perception_host_nav_status.sh", "--execute")
        if status.returncode == 0:
            self.logger.info("perception host navigation is already ready")
            return
        if not self.config.auto_start_navigation:
            raise RuntimeError(self._failure("perception host navigation is not ready", status))

        self.logger.info("perception host navigation is not ready; requesting LiDAR and Nav start")
        lidar = self._remote_wrapper("perception_host_start_lidar.sh", "--execute")
        if lidar.returncode != 0:
            raise RuntimeError(self._failure("perception host LiDAR start failed", lidar))
        started = self._remote_wrapper("perception_host_start_navigation.sh", "--execute")
        if started.returncode != 0:
            raise RuntimeError(self._failure("perception host navigation start failed", started))

        deadline = self.monotonic() + self.config.ready_timeout_sec
        last_status = started
        while self.monotonic() < deadline:
            last_status = self._remote_wrapper(
                "perception_host_nav_status.sh",
                "--execute",
            )
            if last_status.returncode == 0:
                self.logger.info("perception host navigation became ready")
                return
            self.sleep(self.config.poll_interval_sec)
        raise TimeoutError(
            self._failure("perception host navigation readiness timed out", last_status)
        )

    def _remote_wrapper(self, name: str, mode: str) -> CommandResult:
        script = PurePosixPath(self.config.remote_root) / "scripts" / name
        remote_command = "{} {}".format(shlex.quote(str(script)), shlex.quote(mode))
        command = [
            "ssh",
            "-o",
            "BatchMode=yes",
            "-o",
            "ConnectTimeout={}".format(max(1, int(self.config.connect_timeout_sec))),
            "-o",
            "StrictHostKeyChecking=accept-new",
            "-o",
            "UserKnownHostsFile=/tmp/lite3_perception_known_hosts",
            "{}@{}".format(self.config.user, self.config.host),
            remote_command,
        ]
        result = self.runner.run(command, timeout_sec=self.config.command_timeout_sec)
        self.logger.debug(
            "perception wrapper=%s rc=%s stdout=%r stderr=%r",
            name,
            result.returncode,
            result.stdout[-500:],
            result.stderr[-500:],
        )
        return result

    @staticmethod
    def _failure(message: str, result: CommandResult) -> str:
        detail = (result.stderr or result.stdout).strip()
        return "{} rc={} detail={}".format(message, result.returncode, detail[-1000:])


class PerceptionHostStartupGate:
    """Remote Nav orchestration followed by local ROS2 DDS readiness."""

    def __init__(
        self,
        manager: PerceptionHostNavManager,
        nav2_backend,
        *,
        initial_probe_timeout_sec: float = 3.0,
    ) -> None:
        self.manager = manager
        self.nav2_backend = nav2_backend
        self.initial_probe_timeout_sec = initial_probe_timeout_sec

    def ensure_ready(self) -> None:
        try:
            self.nav2_backend.wait_until_ready(timeout_sec=self.initial_probe_timeout_sec)
            return
        except TimeoutError:
            pass
        self.manager.ensure_navigation()
        self.nav2_backend.wait_until_ready()
