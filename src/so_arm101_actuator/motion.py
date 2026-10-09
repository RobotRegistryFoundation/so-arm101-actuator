"""How fast, and through where, this driver lets the arm move.

Two facts the motion path used to ignore, both found by the EV-03 hostile-model
test (software in the loop, 8 October 2026):

* **A position command is a full-speed move.** The bus takes Goal_Position and
  nothing else, and an STS3215 runs to a goal at its own top speed. So
  ``speed: 1.0`` (one direct command) drove joints at 270 deg/s against the
  180 deg/s the so_arm101 preset declares, and the tool tip at up to 1.6 m/s.
  Speed has to be spelled as time: small goals, issued on a clock, sized so no
  joint covers more than the declared rate allows.

* **Checking the ends does not check the path.** The declared workspace is a box
  in Cartesian space; the measured safe ranges are a box in joint space. The
  straight joint-space line between two poses is inside the second box (it is
  convex), but the tool tip's path along that line is not a straight line, and
  between two allowed targets it dipped 11-13 mm below the declared floor. So
  every point of the line is checked before anything moves.

The check is on the COMMANDED path. A loaded servo settles short of its goal
(gravity sag; bob measured up to 27 ticks on shoulder_lift), so the real tip
can sit past a face the commanded path only touches. ``SO_ARM101_WORKSPACE_
MARGIN_MM`` keeps the commanded path that far inside every face; set it to the
tip error measured on the arm.

Everything here is arithmetic: nothing in this module touches the bus.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass

from so_arm101_actuator import kinematics as kin
from so_arm101_actuator.errors import DeniedError

#: The joints whose motion moves the tool tip, in chain order.
ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")

#: One paced step per control period. 50 Hz: short enough that a step is a
#: small fraction of any move, long enough for five Goal_Position writes and the
#: odd read to fit on a 1 Mbps bus with time to spare.
CONTROL_PERIOD_S = 0.02

#: Used only when the manifest declares no ``safety.max_joint_velocity_dps``.
#: The same conservative first-motion budget robot-md's ``init`` writes into a
#: new manifest (robot_md.init._FIRST_MOTION_DEFAULT_VELOCITY_DPS).
DEFAULT_MAX_JOINT_DPS = 30.0

#: Slowest ``speed`` accepted. Anything smaller is a caller who has not decided
#: what the scale means (or a subnormal float: 5e-324 used to reach
#: ``ceil(1/speed)`` and come back as an HTTP 500 OverflowError).
MIN_SPEED = 0.01

#: Longest move this driver will pace. A slower request is refused rather than
#: left holding the servo bus (and every other caller) for minutes.
MAX_MOVE_S = 30.0

#: Resolution of the path check: consecutive checked points are at most this
#: far apart at the tip, by the lever-arm bound below.
CHECK_STEP_MM = 1.0

MARGIN_ENV = "SO_ARM101_WORKSPACE_MARGIN_MM"


# --------------------------------------------------------------------------- #
# What the manifest declares
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class MotionLimits:
    """The rates paced motion is held to, and where each came from."""

    joint_rad_s: float
    tool_mm_s: float | None
    joint_source: str
    tool_source: str | None


def _positive_finite(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    value = float(value)
    return value if math.isfinite(value) and value > 0 else None


def motion_limits(manifest_path: str | None) -> MotionLimits:
    """``safety.max_joint_velocity_dps`` and, for this arm,
    ``safety.max_linear_velocity_ms`` read as the tool tip's linear speed.

    A manifest that declares no joint rate gets the conservative default, never
    "unlimited": an undeclared limit is not permission to run at full slew.
    """
    safety = kin.frontmatter(manifest_path).get("safety") or {}
    dps = _positive_finite(safety.get("max_joint_velocity_dps"))
    if dps is None:
        joint_rad_s, joint_source = math.radians(DEFAULT_MAX_JOINT_DPS), (
            f"default {DEFAULT_MAX_JOINT_DPS:g} deg/s (manifest declares no "
            f"safety.max_joint_velocity_dps)")
    else:
        joint_rad_s, joint_source = math.radians(dps), (
            f"safety.max_joint_velocity_dps = {dps:g}")
    mps = _positive_finite(safety.get("max_linear_velocity_ms"))
    if mps is None:
        return MotionLimits(joint_rad_s, None, joint_source, None)
    return MotionLimits(joint_rad_s, mps * 1000.0, joint_source,
                        f"safety.max_linear_velocity_ms = {mps:g}")


def workspace_box(manifest_path: str | None
                  ) -> tuple[tuple[float, float, float], tuple[float, float, float]] | None:
    """``physics.workspace.bounds_mm`` as (lo, hi), or None when none is declared.

    An axis the manifest leaves out is unbounded, as ``within_workspace``
    treats it.
    """
    bounds = (((kin.frontmatter(manifest_path).get("physics") or {}).get("workspace") or {})
              .get("bounds_mm") or {})
    if not bounds:
        return None
    lo, hi = [], []
    for axis in "xyz":
        span = bounds.get(axis)
        if span and len(span) == 2:
            lo.append(float(span[0]))
            hi.append(float(span[1]))
        else:
            lo.append(-math.inf)
            hi.append(math.inf)
    return (lo[0], lo[1], lo[2]), (hi[0], hi[1], hi[2])


def margin_mm() -> float:
    """``SO_ARM101_WORKSPACE_MARGIN_MM``: how far inside every face the
    commanded path must stay. Default 0 (the declared box itself)."""
    raw = os.environ.get(MARGIN_ENV)
    if raw is None or raw.strip() == "":
        return 0.0
    try:
        value = float(raw)
    except ValueError as exc:
        raise ValueError(f"{MARGIN_ENV}: invalid number {raw!r}") from exc
    if not math.isfinite(value) or value < 0:
        raise ValueError(f"{MARGIN_ENV}: must be a finite number >= 0, got {raw!r}")
    return value


# --------------------------------------------------------------------------- #
# Geometry
# --------------------------------------------------------------------------- #

class Chain:
    """The manifest's kinematic chain, read once, with a fast tip position.

    Same convention as ``kinematics.tip_position_mm`` (rotate about the joint's
    declared axis, then translate ``a_mm`` along x and ``d_mm`` along z), without
    re-reading the manifest on every call: the path check evaluates it a few
    thousand times per move.
    """

    def __init__(self, manifest_path: str | None) -> None:
        chain = [j for j in kin._read_chain(manifest_path) if str(j.get("id")) != "gripper"]
        if not chain:
            raise ValueError("manifest declares no kinematic chain")
        self.links = [(str(j.get("id")), str(j.get("axis", "z")).lower(),
                       float(j.get("a_mm", 0.0)), float(j.get("d_mm", 0.0))) for j in chain]
        self.tip_offset = kin.tip_offset_from_manifest(manifest_path)

    def tip_mm(self, q: dict[str, float]) -> tuple[float, float, float]:
        # The rotation is kept as its three columns; each joint mixes two of
        # them (the third is its own axis), which is all R @ R_axis(angle) does.
        c0x, c0y, c0z = 1.0, 0.0, 0.0
        c1x, c1y, c1z = 0.0, 1.0, 0.0
        c2x, c2y, c2z = 0.0, 0.0, 1.0
        px = py = pz = 0.0
        for name, axis, a, d in self.links:
            angle = q.get(name, 0.0)
            c, s = math.cos(angle), math.sin(angle)
            if axis == "x":
                c1x, c1y, c1z, c2x, c2y, c2z = (c * c1x + s * c2x, c * c1y + s * c2y,
                                                c * c1z + s * c2z, c * c2x - s * c1x,
                                                c * c2y - s * c1y, c * c2z - s * c1z)
            elif axis == "y":
                c0x, c0y, c0z, c2x, c2y, c2z = (c * c0x - s * c2x, c * c0y - s * c2y,
                                                c * c0z - s * c2z, s * c0x + c * c2x,
                                                s * c0y + c * c2y, s * c0z + c * c2z)
            else:
                c0x, c0y, c0z, c1x, c1y, c1z = (c * c0x + s * c1x, c * c0y + s * c1y,
                                                c * c0z + s * c1z, c * c1x - s * c0x,
                                                c * c1y - s * c0y, c * c1z - s * c0z)
            px += a * c0x + d * c2x
            py += a * c0y + d * c2y
            pz += a * c0z + d * c2z
        tx, ty, tz = self.tip_offset
        return (px + tx * c0x + ty * c1x + tz * c2x,
                py + tx * c0y + ty * c1y + tz * c2y,
                pz + tx * c0z + ty * c1z + tz * c2z)

    def lever_mm(self) -> dict[str, float]:
        """For each joint, an upper bound on the tip's distance from its axis.

        Joint i's axis passes through the point the chain has reached before
        link i, and every later link turns rigidly with it, so turning joint i
        by an angle moves the tip by at most that angle times this distance. The
        bound is the sum of the remaining link lengths and the tip offset: true
        in every configuration, which is what lets a check of sampled points say
        something about the path between them.
        """
        tip = math.hypot(*self.tip_offset)
        out: dict[str, float] = {}
        remaining = tip
        for name, _axis, a, d in reversed(self.links):
            remaining += math.hypot(a, d)
            out[name] = remaining
        return out


def clearance_mm(point: tuple[float, float, float],
                 box: tuple[tuple[float, float, float], tuple[float, float, float]]) -> float:
    """Distance from the point to the nearest face: positive inside, negative
    outside (by the most-violated face)."""
    lo, hi = box
    return min(min(p - low, high - p) for p, low, high in zip(point, lo, hi))


def tip_bound_mm(delta: dict[str, float], lever: dict[str, float]) -> float:
    """Most the tip can move for these joint changes, in any order."""
    return sum(lever.get(j, 0.0) * abs(v) for j, v in delta.items())


# --------------------------------------------------------------------------- #
# Speed
# --------------------------------------------------------------------------- #

def validate_speed(speed: object) -> float:
    """``speed`` as a number in [MIN_SPEED, 1], or a ``bad_args`` refusal.
    Validated, never clamped."""
    if isinstance(speed, bool) or not isinstance(speed, (int, float)):
        raise DeniedError("bad_args", f"speed must be a number, got {speed!r}")
    value = float(speed)
    if not math.isfinite(value) or not (MIN_SPEED <= value <= 1.0):
        raise DeniedError(
            "bad_args",
            f"speed must be at least {MIN_SPEED:g} and at most 1 (a fraction of "
            f"the declared joint and tool speed limits), got {value!r}")
    return value


@dataclass(frozen=True)
class Plan:
    """Commanded poses, one per control period, ending on the target."""

    steps: list[dict[str, float]]
    period_s: float
    duration_s: float
    joint_rad_s: float
    tool_bound_mm_s: float | None


#: One encoder tick, in radians. Each step is rounded to whole ticks on the
#: wire, so the commanded goal can run up to one tick ahead of the plan.
TICK_RAD = 2 * math.pi / 4096

#: The shortest window a rate is judged over (EV-03 measures joint speed over
#: 50 ms). The plan keeps one tick per window in hand, so the rounded goal's
#: rate over any such window is still within the declared limit.
RATE_WINDOW_S = 0.05


def plan_line(current: dict[str, float], target: dict[str, float], *, speed: float,
              limits: MotionLimits, lever: dict[str, float],
              period_s: float = CONTROL_PERIOD_S) -> Plan:
    """The straight joint-space line from ``current`` to ``target``, cut into
    steps so that no joint moves faster than ``speed`` times the declared joint
    rate and, when a tool speed is declared, the tip's lever-arm bound per step
    stays within ``speed`` times that. Both budgets allow for the step being
    rounded to whole encoder ticks.
    """
    delta = {j: target[j] - current[j] for j in target}
    joint_rate = speed * limits.joint_rad_s
    biggest = max((abs(v) for v in delta.values()), default=0.0)
    rounding = TICK_RAD * period_s / RATE_WINDOW_S
    joint_step = joint_rate * period_s - rounding
    joint_step = joint_step if joint_step > 0 else joint_rate * period_s / 2
    duration = biggest / joint_step * period_s if joint_step > 0 else math.inf
    tool_rate = None
    if limits.tool_mm_s is not None:
        tool_rate = speed * limits.tool_mm_s
        tool_step = tool_rate * period_s - sum(lever.values()) * rounding
        tool_step = tool_step if tool_step > 0 else tool_rate * period_s / 2
        duration = max(duration, tip_bound_mm(delta, lever) / tool_step * period_s)
    if not math.isfinite(duration) or duration > MAX_MOVE_S:
        raise DeniedError(
            "too_slow",
            f"at speed {speed:g} this move would take {duration:.1f} s, longer than "
            f"the {MAX_MOVE_S:g} s this driver will hold the bus for one move; ask "
            f"for a higher speed or a nearer target")
    n = max(1, math.ceil(duration / period_s - 1e-9))
    steps = [{j: current[j] + delta[j] * k / n for j in target} for k in range(1, n + 1)]
    steps[-1] = dict(target)
    return Plan(steps=steps, period_s=period_s, duration_s=n * period_s,
                joint_rad_s=joint_rate, tool_bound_mm_s=tool_rate)


# --------------------------------------------------------------------------- #
# Where
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class PathCheck:
    ok: bool
    detail: str
    min_clearance_mm: float
    worst_point_mm: tuple[float, float, float] | None = None
    start_clearance_mm: float | None = None


#: Finest resolution the path check refines to near a face. A segment it
#: cannot prove inside at this resolution is refused.
REFINE_MM = 0.02


def check_line(current: dict[str, float], target: dict[str, float], *, chain: Chain,
               box: tuple[tuple[float, float, float], tuple[float, float, float]],
               margin: float = 0.0, step_mm: float = CHECK_STEP_MM) -> PathCheck:
    """Does the tip stay inside the box (less ``margin``) all along the
    joint-space line from ``current`` to ``target``?

    The line is sampled so that consecutive samples are at most ``s`` apart at
    the tip by the lever-arm bound. Between two samples with clearances c0 and
    c1, no point of the line has less clearance than
    ``min(c0, c1, (c0 + c1 - s) / 2)``, so requiring that bound to be >= 0
    covers the whole line, not just the samples. Where the bound cannot prove a
    segment (a path that runs close along a face), the segment is halved and
    checked again, down to ``REFINE_MM``.

    An arm that starts outside, or inside the margin (a taught pose on the
    face, a loaded joint that settled past it), is allowed a path that never
    gets worse than where it started; anything else would leave it with no way
    back.
    """
    lever = chain.lever_mm()
    delta = {j: target[j] - current[j] for j in target}
    total = tip_bound_mm(delta, lever)
    n = max(1, math.ceil(total / step_mm))

    def clearance_at(f: float) -> tuple[float, tuple[float, float, float]]:
        q = {j: current[j] + delta[j] * f for j in target}
        tip = chain.tip_mm({**current, **q})
        return clearance_mm(tip, box) - margin, tip

    start, start_tip = clearance_at(0.0)
    required = min(0.0, start)
    slack = REFINE_MM / 2  # what sampling at the finest resolution can leave unproven

    def segment(fa: float, ca: float, fb: float, cb: float, tip_b: tuple
                ) -> tuple[bool, float, tuple]:
        """(proved inside, least clearance the bound allows, where)."""
        s = total * (fb - fa)
        bound = min(ca, cb, (ca + cb - s) / 2)
        if bound >= required - 1e-9:
            return True, bound, tip_b
        if min(ca, cb) < required - slack or s <= REFINE_MM:
            return bound >= required - slack, bound, tip_b
        fm = (fa + fb) / 2
        cm, tip_m = clearance_at(fm)
        ok_a, bound_a, where_a = segment(fa, ca, fm, cm, tip_m)
        if not ok_a:
            return False, bound_a, where_a
        ok_b, bound_b, where_b = segment(fm, cm, fb, cb, tip_b)
        return ok_b, min(bound_a, bound_b), (where_a if bound_a < bound_b else where_b)

    worst, worst_tip = start, start_tip
    c_prev = start
    for k in range(1, n + 1):
        f_prev, f = (k - 1) / n, k / n
        c, tip = clearance_at(f)
        ok, bound, where = segment(f_prev, c_prev, f, c, tip)
        if bound < worst:
            worst, worst_tip = bound, where
        if not ok:
            spot = ", ".join(f"{v:.0f}" for v in where)
            actual = bound + margin
            how = (f"outside the declared workspace ({-actual:.1f} mm past a face" if actual < 0
                   else f"inside the {margin:g} mm margin ({actual:.1f} mm from a face")
            return PathCheck(
                ok=False,
                detail=(f"the tool path from the arm's current pose to this target passes "
                        f"{how} near ({spot}) mm, the first point found). The joint-space "
                        f"line between two inside points need not stay inside. Nothing was "
                        f"moved; move in shorter legs (up, across, down)."),
                min_clearance_mm=round(actual, 2), worst_point_mm=where,
                start_clearance_mm=round(start + margin, 2))
        c_prev = c
    return PathCheck(ok=True, detail="the whole path stays inside the declared workspace",
                     min_clearance_mm=round(worst + margin, 2), worst_point_mm=worst_tip,
                     start_clearance_mm=round(start + margin, 2))


# --------------------------------------------------------------------------- #
# Gravity
# --------------------------------------------------------------------------- #

#: Joints whose load changes with the arm's pose under gravity.
GRAVITY_JOINTS = ("shoulder_lift", "elbow_flex", "wrist_flex")

#: A drop rate below this (mm of tip height per radian) says too little about
#: a joint's load to learn its sag from.
MIN_DROP_RATE_MM = 20.0


def drop_rate_mm(chain: Chain, q: dict[str, float]) -> dict[str, float]:
    """How fast the tip drops when each gravity-loaded joint turns, -dz/dq in
    mm/rad. With the arm's weight lumped at the tip (virtual work), a joint's
    gravity torque, and so how far it sags from its goal, goes with this."""
    z0 = chain.tip_mm(q)[2]
    h = 1e-3
    return {j: -(chain.tip_mm({**q, j: q[j] + h})[2] - z0) / h
            for j in GRAVITY_JOINTS if j in q}
