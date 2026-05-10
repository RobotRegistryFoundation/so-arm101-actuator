"""Tests for the FakeSerial fixture itself."""

import pytest

from tests.conftest import FakeSerial


def test_fake_serial_records_writes():
    fake = FakeSerial(scripted_reads=[])
    fake.write(b"\xff\xff\x02")
    fake.write(b"\xff")
    assert fake.written == [b"\xff\xff\x02", b"\xff"]


def test_fake_serial_returns_scripted_reads():
    fake = FakeSerial(scripted_reads=[b"\x01\x02", b"\x03\x04"])
    assert fake.read(2) == b"\x01\x02"
    assert fake.read(2) == b"\x03\x04"


def test_fake_serial_raises_when_scripted_reads_exhausted():
    fake = FakeSerial(scripted_reads=[b"\x01"])
    fake.read(1)
    with pytest.raises(IndexError):
        fake.read(1)
