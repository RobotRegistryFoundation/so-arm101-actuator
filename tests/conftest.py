"""Shared pytest fixtures."""

from __future__ import annotations


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
