"""Shared pytest fixtures."""

from __future__ import annotations

import copy
from pathlib import Path

import pytest

from so_arm101_actuator import config, kinematics

#: A real SO-ARM101's geometry, checked in so the tests do not depend on one
#: operator's home directory. Byte-for-byte the physics block of the rig this
#: driver was written against.
FIXTURE_MANIFEST = str(Path(__file__).parent / "fixtures" / "so_arm101.robot.md")


@pytest.fixture(autouse=True)
def _isolate_module_geometry():
    """Undo, after every test, anything a manifest did to the module globals.

    ``config.apply_manifest_calibration`` MUTATES ``config.JOINTS`` and
    ``config.SAFE_RANGE_RAD`` in place — that is the point of it, since the
    whole process serves one robot. In a test session it means the first test to
    read a manifest silently re-zeroes every joint for every test that runs
    after it, and the suite's result then depends on collection order. Snapshot
    and restore so each test starts from the shipped defaults.
    """
    joints = copy.deepcopy(config.JOINTS)
    safe = copy.deepcopy(config.SAFE_RANGE_RAD)
    cache = dict(kinematics._FRONTMATTER_CACHE)
    try:
        yield
    finally:
        config.JOINTS.clear()
        config.JOINTS.update(joints)
        config.SAFE_RANGE_RAD.clear()
        config.SAFE_RANGE_RAD.update(safe)
        kinematics._FRONTMATTER_CACHE.clear()
        kinematics._FRONTMATTER_CACHE.update(cache)


class FakeSerial:
    """Drop-in replacement for `serial.Serial` for unit tests.

    Records each `write()` call into `self.written`. `read(n)` returns the
    next chunk from `scripted_reads` (regardless of n — the chunk size is
    the test author's responsibility). Raises IndexError if exhausted so
    tests fail loud instead of hanging.
    """

    def __init__(self, scripted_reads: list[bytes]) -> None:
        self._scripted_reads = list(scripted_reads)
        self.written: list[bytes] = []

    def write(self, data: bytes) -> int:
        self.written.append(bytes(data))
        return len(data)

    def read(self, n: int) -> bytes:
        if not self._scripted_reads:
            raise IndexError("FakeSerial.scripted_reads exhausted")
        return self._scripted_reads.pop(0)

    def flush(self) -> None:
        pass

    def close(self) -> None:
        pass


class FakeClock:
    """Stands in for ``time`` inside the actuator module: ``sleep`` advances the
    clock instead of waiting. Every gateway-facing motion is now a paced stream
    of setpoints a fixed period apart, and over a perfect simulated servo there
    is nothing to wait for but the pacing itself."""

    def __init__(self) -> None:
        import time as _time

        self._time = _time
        self.now = 1_000.0
        self.slept = 0.0

    def monotonic(self) -> float:
        return self.now

    perf_counter = monotonic

    def sleep(self, seconds: float) -> None:
        if seconds and seconds > 0:
            self.now += seconds
            self.slept += seconds

    def __getattr__(self, name):
        return getattr(self._time, name)


@pytest.fixture
def fake_clock(monkeypatch):
    """Opt in per test: tests that time a real servo stand-in keep real time."""
    import so_arm101_actuator.actuator as act

    clock = FakeClock()
    monkeypatch.setattr(act, "time", clock)
    return clock
