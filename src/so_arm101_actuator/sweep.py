"""FULL-SWEEP pose generator + developer-facing range-of-motion CLI.

Two surfaces:
- generate_sweep_poses(iterations, seed) — deterministic random pose list within
  SAFE_RANGE_RAD. Imported by opencastor-ops/scripts/hil/generate_full_sweep_envelopes.py
  so cert-intake envelopes and the developer CLI agree on poses for a given seed.
- python -m so_arm101_actuator.sweep — drives the actuator directly with a rich
  status table. NOT used in cert-intake; cert evidence comes from gateway
  outcome telemetry recorded by opencastor-ops/scripts/hil/run.py.
"""

from __future__ import annotations

import argparse
import json
import random
import sys
import time
import uuid

from so_arm101_actuator.config import resolve_safe_range_rad


def generate_sweep_poses(iterations: int, seed: int) -> list[dict[str, float]]:
    """Generate `iterations` random multi-joint poses within SAFE_RANGE_RAD.

    Deterministic given (iterations, seed). Each pose maps every joint name
    in SAFE_RANGE_RAD to a uniform-random value in its [lo, hi] range.
    """
    if iterations < 0:
        raise ValueError(f"iterations must be >= 0, got {iterations}")
    rng = random.Random(seed)
    safe = resolve_safe_range_rad()
    joints = sorted(safe.keys())  # stable ordering for determinism
    poses: list[dict[str, float]] = []
    for _ in range(iterations):
        pose = {j: rng.uniform(*safe[j]) for j in joints}
        poses.append(pose)
    return poses


def _dry_run(poses: list[dict[str, float]], out_path: str) -> int:
    with open(out_path, "w") as f:
        for i, pose in enumerate(poses):
            rec = {"mode": "dry-run", "iteration": i, "target": pose}
            f.write(json.dumps(rec, sort_keys=True) + "\n")
    print(f"dry-run: wrote {len(poses)} poses to {out_path}", file=sys.stderr)
    return 0


def _live_run(poses: list[dict[str, float]], out_path: str) -> int:
    """Drive the actuator directly. Imports kept inside function to avoid
    pulling pyserial during dry-run import."""
    from so_arm101_actuator.actuator import SOArm101Actuator
    from so_arm101_actuator.protocol import open_default_protocol
    try:
        from so_arm101_actuator.sweep_table import StatusTable  # type: ignore
    except ImportError:
        StatusTable = None  # type: ignore[assignment]

    proto = open_default_protocol()
    actuator = SOArm101Actuator(protocol=proto)
    table = StatusTable() if StatusTable else None
    if table is not None:
        table.start(total=len(poses))
    try:
        with open(out_path, "w") as f:
            for i, pose in enumerate(poses):
                t0 = time.monotonic()
                actuator.move(pose)
                state = actuator.read_state()
                latency_ms = int(round((time.monotonic() - t0) * 1000))
                rec = {
                    "mode": "live", "iteration": i,
                    "target": pose, "current": state.get("positions", {}),
                    "latency_ms": latency_ms,
                }
                f.write(json.dumps(rec, sort_keys=True) + "\n")
                if table is not None:
                    table.update(i, pose, state.get("positions", {}), latency_ms)
                actuator.home()
    finally:
        if table is not None:
            table.stop()
    print(f"sweep: wrote {len(poses)} iterations to {out_path}", file=sys.stderr)
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="so_arm101_actuator.sweep")
    parser.add_argument("--iterations", type=int, default=100)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--out", default=f"/tmp/sweep-{uuid.uuid4().hex[:8]}.jsonl")
    args = parser.parse_args(argv)

    poses = generate_sweep_poses(args.iterations, args.seed)
    if args.dry_run:
        return _dry_run(poses, args.out)
    return _live_run(poses, args.out)


if __name__ == "__main__":
    sys.exit(main())
