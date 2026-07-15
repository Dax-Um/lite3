from __future__ import annotations

import math
import struct

import pytest

from lite3_common.types import MotionLimits
from lite3_control.udp_driver import DriverSendError, Lite3UdpDriver


EXAMPLE_HOST = "203.0.113.10"
EXAMPLE_PORT = 12000


class FakeSocket:
    def __init__(self, *, fail: bool = False):
        self.fail = fail
        self.bound = []
        self.sent = []
        self.closed = False

    def bind(self, address: tuple[str, int]) -> None:
        self.bound.append(address)

    def sendto(self, packet: bytes, address: tuple[str, int]) -> int:
        if self.fail:
            raise OSError("send failed")
        self.sent.append((packet, address))
        return len(packet)

    def close(self) -> None:
        self.closed = True


def unpack_packet(packet: bytes) -> tuple[int, int, int, float]:
    return struct.unpack("<iiid", packet)


def make_driver(fake_socket: FakeSocket | None = None) -> Lite3UdpDriver:
    return Lite3UdpDriver(
        EXAMPLE_HOST,
        EXAMPLE_PORT,
        MotionLimits(max_vx_mps=0.10, max_vy_mps=0.05, max_wz_radps=0.20),
        sock=fake_socket or FakeSocket(),
    )


def test_packet_size_is_20_bytes():
    driver = make_driver()

    packet = driver.pack_motion_complex_cmd(320, 0.1)

    assert len(packet) == 20


def test_packet_fields_for_vx():
    driver = make_driver()

    packet = driver.pack_motion_complex_cmd(320, 0.1)

    assert unpack_packet(packet) == (320, 8, 1, 0.1)


def test_packet_fields_for_vy():
    driver = make_driver()

    packet = driver.pack_motion_complex_cmd(325, -0.05)

    assert unpack_packet(packet) == (325, 8, 1, -0.05)


def test_wz_sign_is_reversed():
    fake_socket = FakeSocket()
    driver = make_driver(fake_socket)

    driver.send_cmd_vel(0.0, 0.0, 0.2)

    cmd_code, size, value_type, value = unpack_packet(fake_socket.sent[2][0])
    assert (cmd_code, size, value_type) == (321, 8, 1)
    assert value == pytest.approx(-0.2)


def test_velocity_is_clamped():
    fake_socket = FakeSocket()
    driver = make_driver(fake_socket)

    driver.send_cmd_vel(9.0, -9.0, 9.0)

    values = [unpack_packet(packet)[3] for packet, _address in fake_socket.sent]
    assert values == pytest.approx([0.10, -0.05, -0.20])


def test_nan_velocity_is_rejected():
    driver = make_driver()

    with pytest.raises(ValueError, match="finite"):
        driver.send_cmd_vel(math.nan, 0.0, 0.0)


def test_invalid_host_is_rejected():
    with pytest.raises(ValueError, match="host"):
        Lite3UdpDriver("", EXAMPLE_PORT, MotionLimits(), sock=FakeSocket())


def test_invalid_port_is_rejected():
    with pytest.raises(ValueError, match="port"):
        Lite3UdpDriver(EXAMPLE_HOST, 70000, MotionLimits(), sock=FakeSocket())


def test_invalid_local_port_is_rejected():
    with pytest.raises(ValueError, match="local_port"):
        Lite3UdpDriver(
            EXAMPLE_HOST,
            EXAMPLE_PORT,
            MotionLimits(),
            local_port=70000,
            sock=FakeSocket(),
        )


def test_local_port_binds_udp_source_socket():
    fake_socket = FakeSocket()

    Lite3UdpDriver(
        EXAMPLE_HOST,
        EXAMPLE_PORT,
        MotionLimits(),
        local_host="192.0.2.10",
        local_port=43893,
        sock=fake_socket,
    )

    assert fake_socket.bound == [("192.0.2.10", 43893)]


def test_send_cmd_vel_sends_three_packets_to_motion_host():
    fake_socket = FakeSocket()
    driver = make_driver(fake_socket)

    driver.send_cmd_vel(0.01, 0.02, 0.03)

    assert [unpack_packet(packet)[0] for packet, _address in fake_socket.sent] == [320, 325, 321]
    assert [address for _packet, address in fake_socket.sent] == [
        (EXAMPLE_HOST, EXAMPLE_PORT),
        (EXAMPLE_HOST, EXAMPLE_PORT),
        (EXAMPLE_HOST, EXAMPLE_PORT),
    ]


def test_stop_sends_zero_commands_repeatedly():
    fake_socket = FakeSocket()
    driver = make_driver(fake_socket)

    driver.stop(repeat=2, dt_sec=0.0)

    assert len(fake_socket.sent) == 6
    assert [unpack_packet(packet)[3] for packet, _address in fake_socket.sent] == [
        0.0,
        0.0,
        -0.0,
        0.0,
        0.0,
        -0.0,
    ]


def test_send_error_raises_driver_send_error():
    driver = make_driver(FakeSocket(fail=True))

    with pytest.raises(DriverSendError, match="send failed"):
        driver.send_cmd_vel(0.01, 0.0, 0.0)


def test_close_closes_socket():
    fake_socket = FakeSocket()
    driver = make_driver(fake_socket)

    driver.close()

    assert fake_socket.closed is True
