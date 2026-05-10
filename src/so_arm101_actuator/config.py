"""SO-ARM101 joint configuration + rad↔tick conversion.

Defaults match Bob's wiring and SCS encoder range (0..4095 = full revolution).
4096 ticks per 2π rad → ticks_per_rad = 4096 / (2*math.pi) ≈ 651.9.
"""

from __future__ import annotations

import math
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
