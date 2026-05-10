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

import random

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
