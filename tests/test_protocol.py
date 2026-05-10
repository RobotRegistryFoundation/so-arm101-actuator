"""Tests for the SCS wire protocol layer."""

import pytest

from so_arm101_actuator.protocol import SCSProtocol, _build_packet
from tests.conftest import FakeSerial


def test_build_packet_ping_motor_2():
    # PING (instruction 0x01) to motor 2: no params.
    # Expected: FF FF 02 02 01 FA
    # cks = ~(2+2+1) = ~5 = 0xFA
    pkt = _build_packet(motor_id=2, instruction=0x01, params=b"")
    assert pkt == b"\xff\xff\x02\x02\x01\xfa"


def test_build_packet_write_position():
    # WRITE_DATA (0x03) to motor 2, register addr 0x2A, value 2048 (low=0x00, high=0x08).
    # body = [02, 05, 03, 2A, 00, 08]; params_sum = 0x32; total = 0x3C; cks = ~0x3C & 0xFF = 0xC3
    pkt = _build_packet(motor_id=2, instruction=0x03, params=b"\x2a\x00\x08")
    assert pkt == b"\xff\xff\x02\x05\x03\x2a\x00\x08\xc3"


def test_build_packet_checksum_wraps_correctly():
    # Sum > 0xFF case: ID=0xFE, LEN=0x02, INST=0x01 → sum=0x101 & 0xFF = 0x01 → cks = 0xFE
    pkt = _build_packet(motor_id=0xFE, instruction=0x01, params=b"")
    assert pkt[-1] == 0xFE


def _status_ok(motor_id: int) -> bytes:
    """A successful status response (no error)."""
    body = bytes([motor_id, 0x02, 0x00])
    cks = (~sum(body)) & 0xFF
    return b"\xff\xff" + body + bytes([cks])


def test_set_position_writes_correct_packet():
    fake = FakeSerial(scripted_reads=[_status_ok(motor_id=2)])
    proto = SCSProtocol(serial=fake)
    proto.set_position(motor_id=2, ticks=2048)

    # Expected: WRITE_DATA to register 0x2A with [low, high] = [00, 08]
    expected = _build_packet(
        motor_id=2,
        instruction=0x03,
        params=b"\x2a\x00\x08",
    )
    assert fake.written == [expected]


def test_set_position_clamps_ticks_to_14_bit_range():
    fake = FakeSerial(scripted_reads=[_status_ok(2)])
    proto = SCSProtocol(serial=fake)
    with pytest.raises(ValueError):
        proto.set_position(motor_id=2, ticks=-1)


def _status_with_data(motor_id: int, data: bytes) -> bytes:
    """A successful status response carrying `data` bytes."""
    body = bytes([motor_id, len(data) + 2, 0x00]) + data
    cks = (~sum(body)) & 0xFF
    return b"\xff\xff" + body + bytes([cks])


def test_read_position_returns_servo_value():
    # Servo reports ticks=2048 → low=00, high=08
    fake = FakeSerial(scripted_reads=[_status_with_data(motor_id=2, data=b"\x00\x08")])
    proto = SCSProtocol(serial=fake)
    assert proto.read_position(motor_id=2) == 2048


def test_read_position_writes_read_data_packet():
    fake = FakeSerial(scripted_reads=[_status_with_data(motor_id=2, data=b"\x00\x08")])
    proto = SCSProtocol(serial=fake)
    proto.read_position(motor_id=2)
    # READ_DATA(0x02) at addr 0x38 for 2 bytes
    expected = _build_packet(
        motor_id=2,
        instruction=0x02,
        params=b"\x38\x02",
    )
    assert fake.written == [expected]
