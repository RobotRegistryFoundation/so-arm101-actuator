"""Status table for the developer-facing sweep CLI.

Uses rich if available; falls back to plain-text printing otherwise.
The table is a UX nicety; cert correctness does not depend on it.
"""

from __future__ import annotations

import time
from statistics import median
from typing import Any

try:
    from rich.live import Live
    from rich.table import Table
    _HAS_RICH = True
except ImportError:
    _HAS_RICH = False


class StatusTable:
    def __init__(self) -> None:
        self._live: Any = None
        self._latencies: list[int] = []
        self._total: int = 0
        self._t_start: float = 0.0

    def start(self, total: int) -> None:
        self._total = total
        self._t_start = time.monotonic()
        if _HAS_RICH:
            self._live = Live(self._render(0, {}, {}, 0), refresh_per_second=4, transient=False)
            self._live.start()

    def update(self, i: int, target: dict[str, float], current: dict[str, float], latency_ms: int) -> None:
        self._latencies.append(latency_ms)
        if _HAS_RICH and self._live is not None:
            self._live.update(self._render(i + 1, target, current, latency_ms))
        else:
            print(f"[{i+1}/{self._total}] latency={latency_ms}ms target={target}")

    def stop(self) -> None:
        if _HAS_RICH and self._live is not None:
            self._live.stop()

    def _render(self, n: int, target: dict[str, float], current: dict[str, float], latency_ms: int):
        if not _HAS_RICH:
            return None
        elapsed = time.monotonic() - self._t_start
        eta = (elapsed / max(n, 1)) * max(self._total - n, 0)
        p50 = int(median(self._latencies)) if self._latencies else 0
        p95_idx = max(0, int(len(self._latencies) * 0.95) - 1) if self._latencies else 0
        p95 = sorted(self._latencies)[p95_idx] if self._latencies else 0
        max_lat = max(self._latencies) if self._latencies else 0

        table = Table(title=f"FULL-SWEEP   iter {n}/{self._total}   "
                            f"elapsed {elapsed:.1f}s   ETA {eta:.1f}s   "
                            f"p50 {p50}ms p95 {p95}ms max {max_lat}ms")
        table.add_column("Joint")
        table.add_column("Target (rad)", justify="right")
        table.add_column("Current (rad)", justify="right")
        table.add_column("Reached")
        table.add_column("Latency (ms)", justify="right")
        for joint in sorted(target.keys()):
            tgt = target[joint]
            cur = current.get(joint, 0.0)
            reached = abs(tgt - cur) < 0.05  # display heuristic; cert truth is gateway-side
            table.add_row(
                joint,
                f"{tgt:+.3f}",
                f"{cur:+.3f}",
                "✓" if reached else "✗",
                str(latency_ms),
            )
        return table
