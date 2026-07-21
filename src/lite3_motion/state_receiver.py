"""Parse Motion Host ``0x0901`` Robot State UDP packets on IQ9.

The Motion Host sends one RobotStateUpload packet at 50 Hz to one configured
destination.  This module deliberately contains no ROS code, allowing its
binary layout and freshness handling to be tested independently.
"""

from __future__ import annotations

import math
import socket
import struct
import time
from dataclasses import dataclass
from typing import Callable


ROBOT_STATE_CODE = 0x0901
COMMAND_HEADER_SIZE = 12
# The deployed Motion Host build sends the current 200-byte ``0x0901`` form.
# It has two leading int32 values, then rpy at payload offset 8.  This differs
# from the older public header (three leading ints) but is verified from the
# actual datagrams on IQ9.  Do not use native/aligned ``struct`` formats here.
ROBOT_STATE_PAYLOAD_SIZE = 200
ROBOT_STATE_RPY_OFFSET = COMMAND_HEADER_SIZE + 8
ROBOT_STATE_RPY_VEL_OFFSET = ROBOT_STATE_RPY_OFFSET + 24
ROBOT_STATE_XYZ_ACC_OFFSET = ROBOT_STATE_RPY_VEL_OFFSET + 24
ROBOT_STATE_POS_WORLD_OFFSET = ROBOT_STATE_XYZ_ACC_OFFSET + 24
ROBOT_STATE_VEL_WORLD_OFFSET = ROBOT_STATE_POS_WORLD_OFFSET + 24
ROBOT_STATE_VEL_BODY_OFFSET = ROBOT_STATE_VEL_WORLD_OFFSET + 24
# The deployed 200-byte form omits the legacy policy-state int.  After
# ``vel_body`` it retains touch-down, charging/padding and error-state fields,
# followed by the documented RobotStateUpload motion state.
ROBOT_STATE_MOTION_STATE_OFFSET = ROBOT_STATE_VEL_BODY_OFFSET + 24 + 12
MIN_ROBOT_STATE_PACKET_SIZE = COMMAND_HEADER_SIZE + ROBOT_STATE_PAYLOAD_SIZE


@dataclass(frozen=True)
class MotionState:
    robot_basic_state: int
    robot_gait_state: int
    robot_policy_state: int
    robot_motion_state: int
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    roll_vel_radps: float
    pitch_vel_radps: float
    yaw_vel_radps: float
    acc_x_mps2: float
    acc_y_mps2: float
    acc_z_mps2: float
    pos_world_x_m: float
    pos_world_y_m: float
    pos_world_yaw_rad: float
    vel_world_x_mps: float
    vel_world_y_mps: float
    vel_world_yaw_radps: float
    vel_body_x_mps: float
    vel_body_y_mps: float
    vel_body_yaw_radps: float
    received_at_monotonic: float

    @property
    def yaw_rad(self) -> float:
        return math.radians(self.yaw_deg)


def parse_robot_state(
    packet: bytes,
    *,
    received_at_monotonic: float | None = None,
) -> MotionState:
    """Decode a Motion Host RobotStateUpload datagram.

    The deployed vendor structs have gained trailing fields across versions.
    Only the verified current layout through ``vel_body`` is decoded. Other
    Motion Host traffic (for example ``0x0906`` joint/IMU data) is deliberately
    ignored by :meth:`MotionStateUdpReceiver.drain` rather than logged as an
    error.
    """
    if len(packet) < MIN_ROBOT_STATE_PACKET_SIZE:
        raise ValueError("robot state packet is shorter than the stable prefix")
    code, parameter_size, command_type = struct.unpack_from("<iii", packet, 0)
    if code != ROBOT_STATE_CODE:
        raise ValueError("unexpected motion state command code: 0x{:04x}".format(code))
    if command_type != 1:
        raise ValueError("robot state command type must be 1")
    if parameter_size != ROBOT_STATE_PAYLOAD_SIZE:
        raise ValueError("unexpected robot state payload size: {}".format(parameter_size))
    if parameter_size + COMMAND_HEADER_SIZE > len(packet):
        raise ValueError("robot state packet is truncated for declared payload size")

    basic, gait = struct.unpack_from("<ii", packet, COMMAND_HEADER_SIZE)
    rpy = struct.unpack_from("<3d", packet, ROBOT_STATE_RPY_OFFSET)
    rpy_vel = struct.unpack_from("<3d", packet, ROBOT_STATE_RPY_VEL_OFFSET)
    xyz_acc = struct.unpack_from("<3d", packet, ROBOT_STATE_XYZ_ACC_OFFSET)
    pos_world = struct.unpack_from("<3d", packet, ROBOT_STATE_POS_WORLD_OFFSET)
    vel_world = struct.unpack_from("<3d", packet, ROBOT_STATE_VEL_WORLD_OFFSET)
    vel_body = struct.unpack_from("<3d", packet, ROBOT_STATE_VEL_BODY_OFFSET)
    motion_state = struct.unpack_from("<i", packet, ROBOT_STATE_MOTION_STATE_OFFSET)[0]
    values = rpy + rpy_vel + xyz_acc + pos_world + vel_world + vel_body
    if not all(math.isfinite(value) for value in values):
        raise ValueError("robot state contains non-finite numeric values")
    return MotionState(
        robot_basic_state=basic,
        robot_gait_state=gait,
        # The current deployed packet does not carry the legacy policy field.
        robot_policy_state=0,
        robot_motion_state=motion_state,
        roll_deg=rpy[0],
        pitch_deg=rpy[1],
        yaw_deg=rpy[2],
        roll_vel_radps=rpy_vel[0],
        pitch_vel_radps=rpy_vel[1],
        yaw_vel_radps=rpy_vel[2],
        acc_x_mps2=xyz_acc[0],
        acc_y_mps2=xyz_acc[1],
        acc_z_mps2=xyz_acc[2],
        pos_world_x_m=pos_world[0],
        pos_world_y_m=pos_world[1],
        pos_world_yaw_rad=pos_world[2],
        vel_world_x_mps=vel_world[0],
        vel_world_y_mps=vel_world[1],
        vel_world_yaw_radps=vel_world[2],
        vel_body_x_mps=vel_body[0],
        vel_body_y_mps=vel_body[1],
        vel_body_yaw_radps=vel_body[2],
        received_at_monotonic=(
            time.monotonic() if received_at_monotonic is None else received_at_monotonic
        ),
    )


class MotionStateUdpReceiver:
    """Non-blocking UDP socket bound to the Motion Host state destination."""

    def __init__(
        self,
        *,
        bind_host: str = "0.0.0.0",
        port: int = 43897,
        socket_factory: Callable[..., socket.socket] = socket.socket,
    ) -> None:
        if not bind_host:
            raise ValueError("bind_host must not be empty")
        if not 1 <= int(port) <= 65535:
            raise ValueError("port must be in 1..65535")
        self.bind_host = bind_host
        self.port = int(port)
        self._socket = socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind((self.bind_host, self.port))
        self._socket.setblocking(False)

    def drain(self, *, max_packets: int = 32) -> list[MotionState]:
        if max_packets <= 0:
            raise ValueError("max_packets must be positive")
        values: list[MotionState] = []
        for _ in range(max_packets):
            try:
                packet, _address = self._socket.recvfrom(2048)
            except BlockingIOError:
                break
            # 43897 carries all Motion Host upload frames.  State is only the
            # exact 0x0901/200-byte frame; silently skip the rest.
            if len(packet) < COMMAND_HEADER_SIZE:
                continue
            code, parameter_size, command_type = struct.unpack_from("<iii", packet, 0)
            if (
                code != ROBOT_STATE_CODE
                or parameter_size != ROBOT_STATE_PAYLOAD_SIZE
                or command_type != 1
            ):
                continue
            try:
                values.append(parse_robot_state(packet))
            except ValueError:
                # A malformed state packet cannot be used for motion decisions.
                continue
        return values

    def close(self) -> None:
        self._socket.close()
