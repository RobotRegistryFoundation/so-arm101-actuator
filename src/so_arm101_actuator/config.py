"""SO-ARM101 joint configuration + rad↔tick conversion.

Defaults match Bob's wiring and SCS encoder range (0..4095 = full revolution).
4096 ticks per 2π rad → ticks_per_rad = 4096 / (2*math.pi) ≈ 651.9.
"""

from __future__ import annotations

import json as _json
import math
import os as _os
from typing import TypedDict

from so_arm101_actuator.errors import OutOfRangeError, UnknownJointError


class JointSpec(TypedDict):
    motor_id: int
    tick_at_zero_rad: int
    ticks_per_rad: float
    min_rad: float
    max_rad: float


_TICKS_PER_RAD_DEFAULT = 4096 / (2 * math.pi)

JOINTS: dict[str, JointSpec] = {
    "shoulder_pan":   {"motor_id": 1, "tick_at_zero_rad": 2048, "ticks_per_rad": _TICKS_PER_RAD_DEFAULT, "min_rad": -2.0, "max_rad": 2.0},
    "shoulder_lift":  {"motor_id": 2, "tick_at_zero_rad": 2048, "ticks_per_rad": _TICKS_PER_RAD_DEFAULT, "min_rad": -1.5, "max_rad": 1.5},
    "elbow_flex":     {"motor_id": 3, "tick_at_zero_rad": 2048, "ticks_per_rad": _TICKS_PER_RAD_DEFAULT, "min_rad": -1.8, "max_rad": 1.8},
    "wrist_flex":     {"motor_id": 4, "tick_at_zero_rad": 2048, "ticks_per_rad": _TICKS_PER_RAD_DEFAULT, "min_rad": -1.5, "max_rad": 1.5},
    "wrist_roll":     {"motor_id": 5, "tick_at_zero_rad": 2048, "ticks_per_rad": _TICKS_PER_RAD_DEFAULT, "min_rad": -2.5, "max_rad": 2.5},
    "gripper":        {"motor_id": 6, "tick_at_zero_rad": 2048, "ticks_per_rad": _TICKS_PER_RAD_DEFAULT, "min_rad": -0.5, "max_rad": 0.5},
}

HOME_POSE_RAD: dict[str, float] = {name: 0.0 for name in JOINTS}
HOME_POSE_RAD["shoulder_lift"] = 0.10  # gravity-load on Bob; mechanical floor ~tick 2107 (calibrated 2026-05-09)

MOVE_TOLERANCE_RAD: float = 0.02   # ≈ 1.15°


def rad_to_ticks(joint: str, rad: float) -> int:
    """Convert radians → encoder ticks for `joint`. Clamps within tick range."""
    if joint not in JOINTS:
        raise UnknownJointError(joint)
    spec = JOINTS[joint]
    if not (spec["min_rad"] <= rad <= spec["max_rad"]):
        raise OutOfRangeError(f"{joint}={rad:.3f} outside [{spec['min_rad']}, {spec['max_rad']}]")
    ticks = int(round(spec["tick_at_zero_rad"] + rad * spec["ticks_per_rad"]))
    return max(0, min(4095, ticks))


def ticks_to_rad(joint: str, ticks: int) -> float:
    """Convert encoder ticks → radians for `joint`."""
    if joint not in JOINTS:
        raise UnknownJointError(joint)
    spec = JOINTS[joint]
    return (ticks - spec["tick_at_zero_rad"]) / spec["ticks_per_rad"]


def resolve_home_pose_rad() -> dict[str, float]:
    """Return HOME_POSE_RAD merged with SO_ARM101_HOME_POSE_RAD env override.

    Env value MUST be JSON object {joint: rad}. Partial overrides merge with
    HOME_POSE_RAD defaults. Unknown joint names raise ValueError.
    """
    base: dict[str, float] = dict(HOME_POSE_RAD)
    raw = _os.environ.get("SO_ARM101_HOME_POSE_RAD")
    if raw is None:
        return base
    try:
        override = _json.loads(raw)
    except _json.JSONDecodeError as e:
        raise ValueError(f"SO_ARM101_HOME_POSE_RAD: invalid JSON ({e})") from e
    if not isinstance(override, dict):
        raise ValueError("SO_ARM101_HOME_POSE_RAD: must be JSON object")
    for joint, val in override.items():
        if joint not in JOINTS:
            raise ValueError(f"SO_ARM101_HOME_POSE_RAD: unknown joint {joint!r}")
        base[joint] = float(val)
    return base


SAFE_RANGE_RAD: dict[str, tuple[float, float]] = {
    "shoulder_pan":  (-1.40, 1.40),   # observed reachable ±1.46 on Bob; 0.05 margin (calibrated 2026-05-10)
    "shoulder_lift": (-0.90, 1.00),   # observed reachable (-0.95, 1.00); neg side gravity-limited (calibrated 2026-05-10)
    "elbow_flex":    (-0.19, 0.94),   # mechanical floor ~-0.241 rad on Bob; pos near-free to 0.99 (calibrated 2026-05-10)
    "wrist_flex":    (-0.93, 0.41),   # mechanical ceiling ~+0.462 rad on Bob (calibrated 2026-05-10)
    "wrist_roll":    (-1.94, 1.49),   # mechanical ceiling ~+1.537 rad on Bob; asymmetric (calibrated 2026-05-10)
    "gripper":       (0.0, 0.49),     # 0 = closed; 0.49 = near-mechanical-max
}


def resolve_move_tolerance_rad() -> float:
    """Return MOVE_TOLERANCE_RAD merged with SO_ARM101_MOVE_TOLERANCE_RAD env override.

    Env value MUST be a positive float (string-parseable). Default 0.02 rad ≈ 1.15°.
    Operators with rigs that have larger steady-state error (e.g., gravity-loaded
    joints) may want to relax this.
    """
    raw = _os.environ.get("SO_ARM101_MOVE_TOLERANCE_RAD")
    if raw is None:
        return MOVE_TOLERANCE_RAD
    try:
        val = float(raw)
    except ValueError as e:
        raise ValueError(f"SO_ARM101_MOVE_TOLERANCE_RAD: invalid float {raw!r}") from e
    if val <= 0:
        raise ValueError(f"SO_ARM101_MOVE_TOLERANCE_RAD: must be positive, got {val}")
    return val


def resolve_safe_range_rad() -> dict[str, tuple[float, float]]:
    """Return SAFE_RANGE_RAD merged with SO_ARM101_SAFE_RANGE_RAD env override.

    Env value MUST be JSON object {joint: [min, max]}. Each override range
    must lie within JOINTS[joint] mechanical limits and have min < max.
    """
    base: dict[str, tuple[float, float]] = dict(SAFE_RANGE_RAD)
    raw = _os.environ.get("SO_ARM101_SAFE_RANGE_RAD")
    if raw is None:
        return base
    try:
        override = _json.loads(raw)
    except _json.JSONDecodeError as e:
        raise ValueError(f"SO_ARM101_SAFE_RANGE_RAD: invalid JSON ({e})") from e
    if not isinstance(override, dict):
        raise ValueError("SO_ARM101_SAFE_RANGE_RAD: must be JSON object")
    for joint, pair in override.items():
        if joint not in JOINTS:
            raise ValueError(f"SO_ARM101_SAFE_RANGE_RAD: unknown joint {joint!r}")
        if not (isinstance(pair, list) and len(pair) == 2):
            raise ValueError(f"SO_ARM101_SAFE_RANGE_RAD: {joint} must be [min, max]")
        lo, hi = float(pair[0]), float(pair[1])
        if lo >= hi:
            raise ValueError(f"SO_ARM101_SAFE_RANGE_RAD: {joint} min {lo} >= max {hi}")
        spec = JOINTS[joint]
        if lo < spec["min_rad"] or hi > spec["max_rad"]:
            raise ValueError(
                f"SO_ARM101_SAFE_RANGE_RAD: {joint} [{lo}, {hi}] outside mechanical "
                f"[{spec['min_rad']}, {spec['max_rad']}]"
            )
        base[joint] = (lo, hi)
    return base
