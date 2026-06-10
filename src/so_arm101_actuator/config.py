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
    tick_at_zero_rad: float  # interpolation anchor; defaults are whole ticks, resolved specs may be fractional
    ticks_per_rad: float
    min_rad: float
    max_rad: float


_TICKS_PER_RAD_DEFAULT = 4096 / (2 * math.pi)

JOINTS: dict[str, JointSpec] = {
    "shoulder_pan": {
        "motor_id": 1,
        "tick_at_zero_rad": 2048,
        "ticks_per_rad": _TICKS_PER_RAD_DEFAULT,
        "min_rad": -2.0,
        "max_rad": 2.0,
    },
    "shoulder_lift": {
        "motor_id": 2,
        "tick_at_zero_rad": 2048,
        "ticks_per_rad": _TICKS_PER_RAD_DEFAULT,
        "min_rad": -1.5,
        "max_rad": 1.5,
    },
    "elbow_flex": {
        "motor_id": 3,
        "tick_at_zero_rad": 2048,
        "ticks_per_rad": _TICKS_PER_RAD_DEFAULT,
        "min_rad": -1.8,
        "max_rad": 1.8,
    },
    "wrist_flex": {
        "motor_id": 4,
        "tick_at_zero_rad": 2048,
        "ticks_per_rad": _TICKS_PER_RAD_DEFAULT,
        "min_rad": -1.5,
        "max_rad": 1.5,
    },
    "wrist_roll": {
        "motor_id": 5,
        "tick_at_zero_rad": 2048,
        "ticks_per_rad": _TICKS_PER_RAD_DEFAULT,
        "min_rad": -2.5,
        "max_rad": 2.5,
    },
    "gripper": {
        "motor_id": 6,
        "tick_at_zero_rad": 2048,
        "ticks_per_rad": _TICKS_PER_RAD_DEFAULT,
        "min_rad": -0.5,
        "max_rad": 0.5,
    },
}

HOME_POSE_RAD: dict[str, float] = {name: 0.0 for name in JOINTS}
HOME_POSE_RAD["shoulder_lift"] = (
    0.10  # gravity-load on Bob; mechanical floor ~tick 2107 (calibrated 2026-05-09)
)

MOVE_TOLERANCE_RAD: float = 0.02  # ≈ 1.15°


def rad_to_ticks_spec(spec: JointSpec, rad: float) -> int:
    """Convert radians → encoder ticks for a resolved `JointSpec`. Clamps to 0..4095.

    Spec-based so callers can convert against a *manifest-resolved* table
    (see `resolve_joints_from_manifest`) rather than only the shipped defaults.
    """
    if not (spec["min_rad"] <= rad <= spec["max_rad"]):
        raise OutOfRangeError(
            f"rad {rad:.3f} outside [{spec['min_rad']}, {spec['max_rad']}]"
        )
    ticks = int(round(spec["tick_at_zero_rad"] + rad * spec["ticks_per_rad"]))
    return max(0, min(4095, ticks))


def ticks_to_rad_spec(spec: JointSpec, ticks: int) -> float:
    """Convert encoder ticks → radians for a resolved `JointSpec`."""
    return (ticks - spec["tick_at_zero_rad"]) / spec["ticks_per_rad"]


def rad_to_ticks(joint: str, rad: float) -> int:
    """Convert radians → encoder ticks for `joint` (shipped defaults). Clamps within range."""
    if joint not in JOINTS:
        raise UnknownJointError(joint)
    return rad_to_ticks_spec(JOINTS[joint], rad)


def ticks_to_rad(joint: str, ticks: int) -> float:
    """Convert encoder ticks → radians for `joint` (shipped defaults)."""
    if joint not in JOINTS:
        raise UnknownJointError(joint)
    return ticks_to_rad_spec(JOINTS[joint], ticks)


def _resolve_one(
    base: JointSpec, lower_steps, upper_steps, encoder_sign: int
) -> JointSpec | None:
    """Resolve one joint's spec from commissioned tick bounds, or ``None`` if the bounds are
    invalid — so the caller keeps the shipped default rather than emit a wrong map.

    ``lower_steps``/``upper_steps`` are the commissioned tick bounds; ``lower`` MUST be the
    lower one (they are named min_steps/max_steps, close_steps/open_steps for a reason).
    ``encoder_sign`` picks which mechanical-rad end the lower bound sits at: ``+1`` → lower@min_rad,
    ``-1`` → lower@max_rad (a reversed servo). Returns ``None`` on non-numeric, equal, or inverted
    (``lower >= upper``) bounds, or a degenerate rad span — every case that would otherwise
    *silently* produce a backward or collapsed (ticks_per_rad=0) map and drive the servo wrong.
    """
    try:
        lo, hi = float(lower_steps), float(upper_steps)
    except (TypeError, ValueError):
        return None
    if not lo < hi:
        return None
    min_rad, max_rad = base["min_rad"], base["max_rad"]
    if max_rad == min_rad:
        return None
    if encoder_sign >= 0:
        tick_at_min_rad, tick_at_max_rad = lo, hi
    else:
        tick_at_min_rad, tick_at_max_rad = hi, lo
    tpr = (tick_at_max_rad - tick_at_min_rad) / (max_rad - min_rad)
    return {
        **base,
        "ticks_per_rad": tpr,
        "tick_at_zero_rad": tick_at_min_rad - min_rad * tpr,
    }


def _resolve_joints_from_frontmatter(frontmatter: dict) -> dict[str, JointSpec]:
    """Build a rad↔tick table from a ROBOT.md `physics` frontmatter, merged over the
    shipped `JOINTS` defaults (the manifest is authoritative *only* where it declares
    commissioned endpoints).

    The fix for the root-cause gripper bug: when `physics.solver.gripper` carries
    `open_steps`/`close_steps` (and per-joint `physics.kinematics[].{min_steps,max_steps}`),
    derive `ticks_per_rad`/`tick_at_zero_rad` from those *real* endpoints so the rad
    interface lands on the true mechanical travel instead of a centered guess.
    """
    joints: dict[str, JointSpec] = {name: dict(spec) for name, spec in JOINTS.items()}
    physics = (frontmatter or {}).get("physics") or {}

    # Arm joints: per-joint commissioned min_steps/max_steps from physics.kinematics.
    for entry in physics.get("kinematics") or []:
        jid = entry.get("id")
        if jid not in joints:
            continue
        min_steps, max_steps = entry.get("min_steps"), entry.get("max_steps")
        if min_steps is None or max_steps is None:
            continue
        resolved = _resolve_one(
            joints[jid], min_steps, max_steps, entry.get("encoder_sign", 1) or 1
        )
        if resolved is not None:
            joints[jid] = resolved
        # else: invalid/degenerate commissioning → keep the shipped default for this joint.

    # Gripper: close↔min_rad, open↔max_rad; close_steps is the lower tick bound, open_steps upper.
    gripper = (physics.get("solver") or {}).get("gripper") or {}
    open_steps, close_steps = gripper.get("open_steps"), gripper.get("close_steps")
    if open_steps is not None and close_steps is not None and "gripper" in joints:
        resolved = _resolve_one(joints["gripper"], close_steps, open_steps, +1)
        if resolved is not None:
            joints["gripper"] = resolved
        # else: inverted/degenerate gripper (close >= open) → keep default; doctor flags it.

    return joints


def resolve_joints_from_manifest(manifest_path) -> dict[str, JointSpec]:  # noqa: ANN001 — str | Path
    """Resolve the rad↔tick table from a ROBOT.md path.

    Reads the manifest frontmatter via the same `rcan.manifest` reader the gateway
    uses, then `_resolve_joints_from_frontmatter`. **Never raises** — a missing or
    unparseable manifest falls back to the shipped `JOINTS` defaults, so calibration
    resolution can never block actuation.
    """
    try:
        from rcan.manifest import from_manifest

        frontmatter = from_manifest(manifest_path).frontmatter
        return _resolve_joints_from_frontmatter(frontmatter)
    except Exception:
        return {name: dict(spec) for name, spec in JOINTS.items()}


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
    "shoulder_pan": (
        -1.40,
        1.40,
    ),  # observed reachable ±1.46 on Bob; 0.05 margin (calibrated 2026-05-10)
    "shoulder_lift": (
        -0.90,
        1.00,
    ),  # observed reachable (-0.95, 1.00); neg side gravity-limited (calibrated 2026-05-10)
    "elbow_flex": (
        -0.19,
        0.94,
    ),  # mechanical floor ~-0.241 rad on Bob; pos near-free to 0.99 (calibrated 2026-05-10)
    "wrist_flex": (
        -0.93,
        0.41,
    ),  # mechanical ceiling ~+0.462 rad on Bob (calibrated 2026-05-10)
    "wrist_roll": (
        -1.94,
        1.49,
    ),  # mechanical ceiling ~+1.537 rad on Bob; asymmetric (calibrated 2026-05-10)
    "gripper": (0.0, 0.49),  # 0 = closed; 0.49 = near-mechanical-max
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
