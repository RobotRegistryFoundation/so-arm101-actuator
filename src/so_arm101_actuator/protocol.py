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
SCS_TICKS_MAX = 4095

#: On the STS/SMS series every register below this address is EEPROM; everything
#: at or above it is RAM. Refusing writes below the boundary is one check that
#: covers every irreversible mistake at once:
#:
#:   5  ID / 6 Baud_Rate      — the servo vanishes from the bus, or collides
#:                               with a sibling
#:   9  / 11 position limits  — and if BOTH are zero the servo enters
#:                               continuous-rotation mode, where Goal_Position
#:                               means SPEED. Any "bounded travel" guard would
#:                               then bound nothing at all.
#:   31 Homing_Offset         — silently shifts every future position read,
#:                               invalidating the manifest's taught poses
#:   33 Operating_Mode        — makes Goal_Position mean velocity/PWM/step
#:   13/16/19/28/34/35/36     — the servo's OWN overload, overcurrent and
#:                               thermal protection: the net any stall-probe
#:                               design depends on
#:
#: EEPROM also has a finite write count, so an accidental loop wears it out.
EEPROM_BOUNDARY = 40

#: RAM, but it gates whether EEPROM writes persist — never written here.
SCS_REG_LOCK = 55


def _reject_eeprom_write(register: int) -> None:
    """Refuse to write anywhere that could permanently alter the servo."""
    if register < EEPROM_BOUNDARY:
        raise ValueError(
            f"refusing to write register {register}: below {EEPROM_BOUNDARY} is "
            f"EEPROM (id, baud, limits, offsets, protection thresholds) and the "
            f"change would be permanent"
        )
    if register == SCS_REG_LOCK:
        raise ValueError("refusing to write register 55 (Lock): it controls "
                         "whether EEPROM writes persist")  # 12-bit encoder, but reg is 16-bit; allow 0..4095 for SO-ARM101


class SCSProtocol:
    """Empty class shell — methods land in Tasks 6-8."""

    def __init__(self, serial) -> None:  # noqa: ANN001 — duck-typed
        self._serial = serial

    def set_position(self, motor_id: int, ticks: int) -> None:
        """Write Goal_Position on `motor_id`. `ticks` must be 0..SCS_TICKS_MAX."""
        if not 0 <= ticks <= SCS_TICKS_MAX:
            raise ValueError(f"ticks {ticks} outside 0..{SCS_TICKS_MAX}")
        _reject_eeprom_write(SCS_REG_GOAL_POSITION)
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
        lo, hi = self._read_status(motor_id, 2, what="Present_Position")
        return lo | (hi << 8)

    def _read_status(self, motor_id: int, n: int, *, what: str,
                     allow_error: bool = True) -> bytes:
        """Read the status packet answering a READ of ``n`` bytes, and refuse
        one that is not that answer: wrong length, header, servo id, length
        byte or checksum (a stale packet left in the buffer, another servo's
        reply, line noise). With ``allow_error=False`` a set error byte
        (voltage, angle, overheat, overload) is refused too. Returns the data.
        """
        from so_arm101_actuator.errors import ProtocolError

        resp = self._serial.read(6 + n)
        if len(resp) < 6 + n or resp[:2] != b"\xff\xff":
            raise ProtocolError(f"bad header reading {what} from servo {motor_id}: {resp!r}")
        body = resp[2:-1]
        if body[0] != motor_id:
            raise ProtocolError(
                f"reply to a {what} read of servo {motor_id} came from servo {body[0]}: {resp!r}")
        if body[1] != n + 2:
            raise ProtocolError(f"bad length reading {what} from servo {motor_id}: {resp!r}")
        if (~sum(body)) & 0xFF != resp[-1]:
            raise ProtocolError(f"bad checksum reading {what} from servo {motor_id}: {resp!r}")
        if body[2] and not allow_error:
            raise ProtocolError(
                f"servo {motor_id} flagged error 0x{body[2]:02x} reading {what}: {resp!r}")
        return bytes(body[3:3 + n])

    def read_goal_position(self, motor_id: int) -> int:
        """Read Goal_Position (2 bytes) from `motor_id`: the setpoint the servo
        is holding, which under gravity is not where it IS. Returns ticks."""
        params = bytes([SCS_REG_GOAL_POSITION, 0x02])
        pkt = _build_packet(
            motor_id=motor_id,
            instruction=SCS_INST_READ_DATA,
            params=params,
        )
        self._serial.write(pkt)
        # Trusted as a move's starting point and as a stop's hold, so a reply
        # with the servo's error byte set is refused as well as a malformed one.
        lo, hi = self._read_status(motor_id, 2, what="Goal_Position", allow_error=False)
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
        return self._read_status(motor_id, 1, what="Present_Temperature")[0]

    def ping(self, motor_id: int) -> bool:
        """Send a PING to `motor_id`. Returns True if a response is received."""
        pkt = _build_packet(motor_id=motor_id, instruction=SCS_INST_PING, params=b"")
        self._serial.write(pkt)
        resp = self._serial.read(6)
        return len(resp) == 6 and resp[:2] == b"\xff\xff"
