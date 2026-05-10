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


class SCSProtocol:
    """Empty class shell — methods land in Tasks 6-8."""

    def __init__(self, serial) -> None:  # noqa: ANN001 — duck-typed
        self._serial = serial
