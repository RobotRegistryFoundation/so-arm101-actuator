"""SO-ARM101 joint configuration + rad↔tick conversion.

Defaults match Bob's wiring and SCS encoder range (0..4095 = full revolution).
4096 ticks per 2π rad → ticks_per_rad = 4096 / (2*math.pi) ≈ 651.9.
"""

from __future__ import annotations

import json as _json
import math
import os as _os
from pathlib import Path as _Path
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


#: Where the robot's own manifest lives, when one is configured. The manifest is
#: AUTHORITATIVE for this robot's geometry; the constants above are only a
#: fallback for a bench with no manifest at all.
MANIFEST_ENV = "ROBOT_MANIFEST"


def load_manifest_calibration(path: str | None = None) -> dict:
    """Per-joint zero and gripper span, read from the robot's signed ROBOT.md.

    This exists because the constants above are GENERIC and this robot is not.
    `tick_at_zero_rad = 2048` is a sensible default for a servo at mid-travel,
    but Bob's gripper zero is 1539 and its jaws only span ticks 1200-1700 — so
    the generic zero puts every real reading far outside the joint's own safe
    range, and the gripper gets excluded from motion as if it were faulty.

    The manifest already carries the right numbers (`zero_pose_steps` per joint,
    plus `physics.gripper.open_steps` / `close_steps`). Reading them is a pure
    software fix: no motion, no probing, nothing to damage.
    """
    source = path or _os.environ.get(MANIFEST_ENV, "")
    if not source:
        return {}
    try:
        text = _Path(source).read_text()
    except OSError:
        return {}
    if not text.startswith("---"):
        return {}
    end = text.find("\n---", 3)
    if end == -1:
        return {}
    try:
        import yaml

        front = yaml.safe_load(text[3:end]) or {}
    except Exception:
        return {}

    out: dict = {"zeros": {}, "gripper": {}}
    # Both live under `physics` in an RCAN v3 manifest — kinematics directly,
    # and the gripper's jaw travel under the solver block.
    physics = front.get("physics") or {}
    for joint in (physics.get("kinematics") or []):
        if isinstance(joint, dict) and "id" in joint and "zero_pose_steps" in joint:
            try:
                out["zeros"][str(joint["id"])] = int(joint["zero_pose_steps"])
            except (TypeError, ValueError):
                continue
    # The taught pose, in TICKS. Ticks are what the servo actually accepts, so a
    # pose expressed this way is independent of whatever zero convention the
    # driver happens to use — which is exactly the bug being fixed here.
    ready = ((physics.get("poses") or {}).get("ready") or {}).get("joints") or {}
    if isinstance(ready, dict):
        out["ready_ticks"] = {str(k): int(v) for k, v in ready.items()
                              if isinstance(v, (int, float))}

    gripper = ((physics.get("solver") or {}).get("gripper") or {})
    for key in ("open_steps", "close_steps"):
        if key in gripper:
            try:
                out["gripper"][key] = int(gripper[key])
            except (TypeError, ValueError):
                pass
    return out


def apply_manifest_calibration(path: str | None = None) -> dict:
    """Fold the manifest's geometry into JOINTS and SAFE_RANGE_RAD.

    Returns a summary of what changed so a caller can report it honestly rather
    than silently altering how the arm interprets every position.
    """
    data = load_manifest_calibration(path)
    if not data:
        return {}
    changed: dict = {"zeros": {}, "safe_range": {}}

    for joint, zero in data.get("zeros", {}).items():
        spec = JOINTS.get(joint)
        if spec is None or spec["tick_at_zero_rad"] == zero:
            continue
        changed["zeros"][joint] = {"was": spec["tick_at_zero_rad"], "now": zero}
        spec["tick_at_zero_rad"] = zero

    # The gripper's usable range is its declared jaw travel, expressed against
    # its own (now correct) zero — not the generic +/-0.5 rad guess.
    grip = data.get("gripper", {})
    if "open_steps" in grip and "close_steps" in grip and "gripper" in JOINTS:
        zero = JOINTS["gripper"]["tick_at_zero_rad"]
        tpr = JOINTS["gripper"]["ticks_per_rad"]
        lo = (min(grip["close_steps"], grip["open_steps"]) - zero) / tpr
        hi = (max(grip["close_steps"], grip["open_steps"]) - zero) / tpr
        changed["safe_range"]["gripper"] = {"was": SAFE_RANGE_RAD.get("gripper"),
                                            "now": (round(lo, 4), round(hi, 4))}
        SAFE_RANGE_RAD["gripper"] = (lo, hi)
        # Mechanical limits must at least contain the declared travel, or a
        # legitimate commanded position would be rejected as out of range.
        JOINTS["gripper"]["min_rad"] = min(JOINTS["gripper"]["min_rad"], lo)
        JOINTS["gripper"]["max_rad"] = max(JOINTS["gripper"]["max_rad"], hi)
    return changed


def gripper_geometry_known(path: str | None = None) -> bool:
    """Did the manifest declare BOTH the gripper's zero and its jaw travel?

    Deliberately asks the manifest rather than the diff returned by
    apply_manifest_calibration(): that diff lists only values that CHANGED, so a
    manifest zero identical to the module constant would be absent from it and
    a correctly-calibrated joint would look uncalibrated.
    """
    data = load_manifest_calibration(path)
    grip = data.get("gripper", {})
    return ("gripper" in data.get("zeros", {})
            and "open_steps" in grip and "close_steps" in grip)


def manifest_home_pose_rad(path: str | None = None) -> dict[str, float]:
    """The manifest's taught `ready` pose, in radians against the CURRENT zeros.

    Prefer this over any hand-tuned radian constant. A radian value is only
    meaningful relative to a zero, so a pose tuned against the wrong zero moves
    the arm somewhere else the moment the zero is corrected — here, 182 ticks
    (~16 degrees) of shoulder_lift. Reading the taught pose in ticks and
    converting through the corrected zeros round-trips exactly.
    """
    data = load_manifest_calibration(path)
    ticks = data.get("ready_ticks") or {}
    return {joint: ticks_to_rad(joint, value)
            for joint, value in ticks.items() if joint in JOINTS}


def resolve_home_pose_rad(path: str | None = None) -> dict[str, float]:
    """Return HOME_POSE_RAD merged with SO_ARM101_HOME_POSE_RAD env override.

    Env value MUST be JSON object {joint: rad}. Partial overrides merge with
    HOME_POSE_RAD defaults. Unknown joint names raise ValueError.

    `path` MUST be threaded through by any caller that also applied a manifest
    from an explicit path. Correcting the tick zeros without also taking the
    taught pose from the SAME manifest is the dangerous combination: the generic
    radian constants then resolve against the new zeros and command the arm to
    its raw zero position instead of the pose someone actually taught it.
    """
    # Prefer the manifest's taught pose. It is stored in TICKS, so it stays
    # correct no matter what zero convention the driver uses — unlike a radian
    # constant, which silently means something different the moment a zero is
    # corrected.
    base: dict[str, float] = dict(HOME_POSE_RAD)
    try:
        taught = manifest_home_pose_rad(path)
        if taught:
            base.update(taught)
    except Exception:
        pass
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
