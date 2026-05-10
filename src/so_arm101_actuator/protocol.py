"""SCS/Feetech servo wire protocol layer.

Vendored from the same protocol bob_wave.py uses on Bob (RPi 5 + SO-ARM101).
No external servo-library dependency — we own the bytes.
"""

from __future__ import annotations


def _build_packet(motor_id: int, instruction: int, params: bytes) -> bytes:
    """Build a Dynamixel-1.0-style packet:
    [FF FF] [ID] [LEN] [INST] [PARAMS] [CKS]
    where LEN = len(PARAMS) + 2 and CKS = ~(ID + LEN + INST + sum(PARAMS)) & 0xFF.
    """
    length = len(params) + 2
    body = bytes([motor_id, length, instruction]) + params
    checksum = (~sum(body)) & 0xFF
    return b"\xff\xff" + body + bytes([checksum])


SCS_REG_GOAL_POSITION = 0x2A
SCS_INST_WRITE_DATA = 0x03
SCS_REG_PRESENT_POSITION = 0x38
SCS_INST_READ_DATA = 0x02
SCS_REG_PRESENT_TEMPERATURE = 0x3F
SCS_INST_PING = 0x01
SCS_TICKS_MAX = 4095  # 12-bit encoder, but reg is 16-bit; allow 0..4095 for SO-ARM101


class SCSProtocol:
    """Empty class shell — methods land in Tasks 6-8."""

    def __init__(self, serial) -> None:  # noqa: ANN001 — duck-typed
        self._serial = serial

    def set_position(self, motor_id: int, ticks: int) -> None:
        """Write Goal_Position on `motor_id`. `ticks` must be 0..SCS_TICKS_MAX."""
        if not 0 <= ticks <= SCS_TICKS_MAX:
            raise ValueError(f"ticks {ticks} outside 0..{SCS_TICKS_MAX}")
        params = bytes([SCS_REG_GOAL_POSITION, ticks & 0xFF, (ticks >> 8) & 0xFF])
        pkt = _build_packet(
            motor_id=motor_id,
            instruction=SCS_INST_WRITE_DATA,
            params=params,
        )
        self._serial.write(pkt)
        # Drain status packet (6 bytes for a no-param OK response).
        self._serial.read(6)

    def read_position(self, motor_id: int) -> int:
        """Read Present_Position (2 bytes) from `motor_id`. Returns ticks."""
        params = bytes([SCS_REG_PRESENT_POSITION, 0x02])
        pkt = _build_packet(
            motor_id=motor_id,
            instruction=SCS_INST_READ_DATA,
            params=params,
        )
        self._serial.write(pkt)
        # Status packet: FF FF ID LEN ERR LO HI CKS = 8 bytes
        resp = self._serial.read(8)
        if len(resp) < 8 or resp[:2] != b"\xff\xff":
            from so_arm101_actuator.errors import ProtocolError
            raise ProtocolError(f"bad header: {resp!r}")
        lo, hi = resp[5], resp[6]
        return lo | (hi << 8)

    def read_temperature(self, motor_id: int) -> int:
        """Read Present_Temperature (1 byte) from `motor_id`. Returns temperature in Celsius."""
        params = bytes([SCS_REG_PRESENT_TEMPERATURE, 0x01])
        pkt = _build_packet(
            motor_id=motor_id,
            instruction=SCS_INST_READ_DATA,
            params=params,
        )
        self._serial.write(pkt)
        # 1-byte data → 7-byte status
        resp = self._serial.read(7)
        if len(resp) < 7 or resp[:2] != b"\xff\xff":
            from so_arm101_actuator.errors import ProtocolError
            raise ProtocolError(f"bad header: {resp!r}")
        return resp[5]

    def ping(self, motor_id: int) -> bool:
        """Send a PING to `motor_id`. Returns True if a response is received."""
        pkt = _build_packet(motor_id=motor_id, instruction=SCS_INST_PING, params=b"")
        self._serial.write(pkt)
        resp = self._serial.read(6)
        return len(resp) == 6 and resp[:2] == b"\xff\xff"
