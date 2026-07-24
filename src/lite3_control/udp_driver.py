"""Lite3 UDP command driver."""

from __future__ import annotations

import math
import os
import socket
import struct
import time
from typing import Protocol

from lite3_common.types import MotionLimits


DEFAULT_MOTION_HOST_ENV = "LITE3_MOTION_HOST"
DEFAULT_MOTION_PORT_ENV = "LITE3_MOTION_PORT"

MOTION_COMPLEX_CMD_FORMAT = "<iiid"
MOTION_COMPLEX_CMD_SIZE = struct.calcsize(MOTION_COMPLEX_CMD_FORMAT)
MOTION_COMPLEX_CMD_DATA_SIZE = 8
MOTION_COMPLEX_CMD_TYPE = 1
MOTION_SIMPLE_CMD_FORMAT = "<iii"
MOTION_SIMPLE_CMD_SIZE = struct.calcsize(MOTION_SIMPLE_CMD_FORMAT)
MOTION_SIMPLE_CMD_TYPE = 0

CMD_LINEAR_X = 320
CMD_LINEAR_Y = 325
CMD_ANGULAR_Z = 321

# Motion-host simple commands.  The command header is the same UDP transport
# used for cmd_vel, with no trailing double payload.
CMD_FLAT_GAIT_SLOW = 0x21010300
CMD_FLAT_GAIT_MIDDLE = 0x21010307
CMD_FLAT_GAIT_FAST = 0x21010303
CMD_TWIST_JUMP = 0x2101020D
CMD_LONG_JUMP = 0x2101050B
CMD_HELLO = 0x21010507
CMD_MOONWALK = 0x2101030C
CMD_STOP_ACTION = 0x21010C0B
CMD_MANUAL_MODE = 0x21010C02
CMD_NAVIGATION_MODE = 0x21010C03
CMD_STAND_SIT = 0x21010202


class DriverSendError(RuntimeError):
    """Raised when a command packet cannot be sent."""


class DatagramSocket(Protocol):
    def bind(self, address: tuple[str, int]) -> None:
        ...

    def sendto(self, packet: bytes, address: tuple[str, int]) -> int:
        ...

    def close(self) -> None:
        ...


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class Lite3UdpDriver:
    def __init__(
        self,
        host: str | None = None,
        port: int | None = None,
        limits: MotionLimits | None = None,
        *,
        local_host: str | None = None,
        local_port: int | None = None,
        sock: DatagramSocket | None = None,
    ):
        host = host or os.environ.get(DEFAULT_MOTION_HOST_ENV, "")
        port = port if port is not None else _port_from_env()
        if not host:
            raise ValueError("host must be provided explicitly or via LITE3_MOTION_HOST")
        if port < 1 or port > 65535:
            raise ValueError("port must be in range 1..65535")

        self.host = host
        self.port = port
        self.limits = limits or MotionLimits()
        self._socket = sock or socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if local_port is not None:
            if local_port < 1 or local_port > 65535:
                raise ValueError("local_port must be in range 1..65535")
            self._socket.bind((local_host or "", local_port))
        self._address = (self.host, self.port)

    def pack_motion_complex_cmd(self, cmd_code: int, value: float) -> bytes:
        return struct.pack(
            MOTION_COMPLEX_CMD_FORMAT,
            cmd_code,
            MOTION_COMPLEX_CMD_DATA_SIZE,
            MOTION_COMPLEX_CMD_TYPE,
            value,
        )

    def pack_motion_simple_cmd(self, cmd_code: int, value: int = 0) -> bytes:
        """Encode a motion-host type-0 command as three little-endian int32s."""
        if isinstance(cmd_code, bool) or not isinstance(cmd_code, int):
            raise ValueError("simple command code must be an int32")
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError("simple command value must be an int32")
        if not -(2**31) <= cmd_code < 2**31:
            raise ValueError("simple command code must fit int32")
        if not -(2**31) <= value < 2**31:
            raise ValueError("simple command value must fit int32")
        return struct.pack(
            MOTION_SIMPLE_CMD_FORMAT,
            cmd_code,
            value,
            MOTION_SIMPLE_CMD_TYPE,
        )

    def send_simple_command(self, cmd_code: int, value: int = 0) -> None:
        packet = self.pack_motion_simple_cmd(cmd_code, value)
        try:
            self._socket.sendto(packet, self._address)
        except OSError as exc:
            raise DriverSendError(str(exc)) from exc

    def send_cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        self._reject_non_finite(vx, vy, wz)
        commands = (
            (CMD_LINEAR_X, clamp(vx, self.limits.max_vx_mps)),
            (CMD_LINEAR_Y, clamp(vy, self.limits.max_vy_mps)),
            (CMD_ANGULAR_Z, -clamp(wz, self.limits.max_wz_radps)),
        )

        for cmd_code, value in commands:
            packet = self.pack_motion_complex_cmd(cmd_code, value)
            try:
                self._socket.sendto(packet, self._address)
            except OSError as exc:
                raise DriverSendError(str(exc)) from exc

    def stop(self, repeat: int, dt_sec: float) -> None:
        for index in range(repeat):
            self.send_cmd_vel(0.0, 0.0, 0.0)
            if dt_sec > 0.0 and index < repeat - 1:
                time.sleep(dt_sec)

    def close(self) -> None:
        self._socket.close()

    @staticmethod
    def _reject_non_finite(*values: float) -> None:
        if not all(math.isfinite(value) for value in values):
            raise ValueError("velocity values must be finite")


def _port_from_env() -> int:
    raw_port = os.environ.get(DEFAULT_MOTION_PORT_ENV)
    if raw_port is None:
        raise ValueError("port must be provided explicitly or via LITE3_MOTION_PORT")
    return int(raw_port)
