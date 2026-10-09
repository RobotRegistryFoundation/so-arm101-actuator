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


def test_read_temperature_returns_celsius():
    fake = FakeSerial(scripted_reads=[_status_with_data(motor_id=2, data=b"\x2d")])  # 45 C
    proto = SCSProtocol(serial=fake)
    assert proto.read_temperature(motor_id=2) == 45


def test_ping_returns_true_on_response():
    fake = FakeSerial(scripted_reads=[_status_ok(motor_id=2)])
    proto = SCSProtocol(serial=fake)
    assert proto.ping(motor_id=2) is True


def test_read_goal_position_reads_the_setpoint_register():
    """Goal_Position (0x2A) is what the servo is holding; Present_Position
    (0x38) is where gravity has left it. A fresh gateway plans from the first."""
    fake = FakeSerial(scripted_reads=[_status_with_data(motor_id=3, data=b"\x10\x08")])
    proto = SCSProtocol(serial=fake)
    assert proto.read_goal_position(motor_id=3) == 0x0810
    assert fake.written == [_build_packet(motor_id=3, instruction=0x02, params=b"\x2a\x02")]


@pytest.mark.parametrize("method", ["read_position", "read_goal_position"])
def test_a_reply_without_the_header_is_a_protocol_error_not_a_position(method):
    from so_arm101_actuator.errors import ProtocolError

    fake = FakeSerial(scripted_reads=[b"\x00\x00\x02\x04\x00\x00\x08\xf1"])
    proto = SCSProtocol(serial=fake)
    with pytest.raises(ProtocolError):
        getattr(proto, method)(motor_id=2)


@pytest.mark.parametrize("method", ["read_position", "read_goal_position"])
def test_a_short_reply_is_a_protocol_error_not_a_position(method):
    from so_arm101_actuator.errors import ProtocolError

    fake = FakeSerial(scripted_reads=[b"\xff\xff\x02"])
    proto = SCSProtocol(serial=fake)
    with pytest.raises(ProtocolError):
        getattr(proto, method)(motor_id=2)


def _reply(motor_id: int, data: bytes, *, error: int = 0, length: int | None = None,
           checksum: int | None = None) -> bytes:
    body = bytes([motor_id, len(data) + 2 if length is None else length, error]) + data
    return b"\xff\xff" + body + bytes([(~sum(body)) & 0xFF if checksum is None else checksum])


@pytest.mark.parametrize("method", ["read_position", "read_goal_position"])
@pytest.mark.parametrize("reply", [
    _reply(3, b"\x00\x08"),                      # another servo's answer
    _reply(2, b"\x00\x08", checksum=0x00),       # corrupted
    _reply(2, b"\x00\x08", length=0x05),         # not the reply to a 2-byte read
], ids=["wrong servo", "bad checksum", "bad length"])
def test_a_reply_that_is_not_this_servos_answer_is_refused(method, reply):
    """A stale packet left in the buffer, another servo's reply or line noise
    must not become a position: the goal read is a move's starting point and a
    stop's hold (review of #7)."""
    from so_arm101_actuator.errors import ProtocolError

    proto = SCSProtocol(serial=FakeSerial(scripted_reads=[reply]))
    with pytest.raises(ProtocolError):
        getattr(proto, method)(motor_id=2)


def test_the_goal_read_refuses_a_servo_reporting_an_error_and_the_position_read_does_not():
    """The goal register is trusted as a plan's start and a stop's hold, so a
    servo flagging overload or overheat there is refused. Present_Position is
    still reported: a stop and a state read must not depend on a healthy servo."""
    from so_arm101_actuator.errors import ProtocolError

    flagged = _reply(2, b"\x00\x08", error=0x20)  # overload
    assert SCSProtocol(serial=FakeSerial([flagged])).read_position(motor_id=2) == 0x0800
    with pytest.raises(ProtocolError, match="flagged error 0x20"):
        SCSProtocol(serial=FakeSerial([flagged])).read_goal_position(motor_id=2)
