import math
import struct

import pytest

from lite3_motion.state_receiver import (
    COMMAND_HEADER_SIZE,
    MIN_ROBOT_STATE_PACKET_SIZE,
    ROBOT_STATE_CODE,
    parse_robot_state,
)


def packet(*, code=ROBOT_STATE_CODE, command_type=1, yaw_deg=90.0):
    raw = bytearray(MIN_ROBOT_STATE_PACKET_SIZE)
    struct.pack_into("<iii", raw, 0, code, len(raw) - COMMAND_HEADER_SIZE, command_type)
    struct.pack_into("<ii", raw, COMMAND_HEADER_SIZE, 6, 5)
    struct.pack_into("<3d", raw, COMMAND_HEADER_SIZE + 8, 1.0, 2.0, yaw_deg)
    struct.pack_into("<3d", raw, COMMAND_HEADER_SIZE + 32, 0.1, 0.2, 0.3)
    struct.pack_into("<3d", raw, COMMAND_HEADER_SIZE + 56, 0.4, 0.5, 9.8)
    struct.pack_into("<3d", raw, COMMAND_HEADER_SIZE + 80, 3.0, 4.0, 0.7)
    struct.pack_into("<3d", raw, COMMAND_HEADER_SIZE + 104, 0.6, 0.7, 0.8)
    struct.pack_into("<3d", raw, COMMAND_HEADER_SIZE + 128, 0.9, 1.0, 1.1)
    return bytes(raw)


def test_parse_robot_state_common_prefix():
    state = parse_robot_state(packet(), received_at_monotonic=12.5)
    assert state.robot_basic_state == 6
    assert state.robot_gait_state == 5
    assert state.yaw_rad == pytest.approx(math.pi / 2.0)
    assert state.pos_world_x_m == pytest.approx(3.0)
    assert state.pos_world_y_m == pytest.approx(4.0)
    assert state.vel_body_yaw_radps == pytest.approx(1.1)
    assert state.received_at_monotonic == 12.5


@pytest.mark.parametrize("kwargs", [
    {"code": 0x0902},
    {"command_type": 0},
])
def test_rejects_non_robot_state_packets(kwargs):
    with pytest.raises(ValueError):
        parse_robot_state(packet(**kwargs))
