"""Tests for the SCS wire protocol layer."""

import pytest

from so_arm101_actuator.protocol import SCSProtocol, _build_packet


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
