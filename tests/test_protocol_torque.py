"""TDD (B1): SCSProtocol.set_torque — Torque_Enable register (0x28) write.

This is the single hardware-register write that never existed before and will
eventually fire on a real servo. It must mirror set_position's exact framing
(WRITE_DATA 0x03) and drain the status packet so the bus stays in sync.
"""

from __future__ import annotations

import pytest

from so_arm101_actuator.protocol import SCSProtocol, _build_packet
from tests.conftest import FakeSerial


def _status_ok(motor_id: int) -> bytes:
    """A successful (no-error) status response — same shape set_position drains."""
    body = bytes([motor_id, 0x02, 0x00])
    cks = (~sum(body)) & 0xFF
    return b"\xff\xff" + body + bytes([cks])


def test_set_torque_enable_writes_torque_enable_register():
    fake = FakeSerial(scripted_reads=[_status_ok(2)])
    proto = SCSProtocol(serial=fake)
    proto.set_torque(motor_id=2, enable=True)
    # WRITE_DATA(0x03) to Torque_Enable register 0x28, value 0x01.
    expected = _build_packet(motor_id=2, instruction=0x03, params=b"\x28\x01")
    assert fake.written == [expected]


def test_set_torque_disable_writes_zero():
    fake = FakeSerial(scripted_reads=[_status_ok(5)])
    proto = SCSProtocol(serial=fake)
    proto.set_torque(motor_id=5, enable=False)
    expected = _build_packet(motor_id=5, instruction=0x03, params=b"\x28\x00")
    assert fake.written == [expected]


def test_set_torque_drains_status_packet():
    # Must consume the 6-byte status response (exactly one read), like set_position,
    # or the next read on the bus would mis-frame.
    fake = FakeSerial(scripted_reads=[_status_ok(2)])
    proto = SCSProtocol(serial=fake)
    proto.set_torque(motor_id=2, enable=True)
    with pytest.raises(IndexError):  # scripted read already consumed by set_torque
        fake.read(6)
