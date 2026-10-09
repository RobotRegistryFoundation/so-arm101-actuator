"""Planning a motion this arm can be held to.

Every motion the gateway can ask for becomes the same thing here: one straight
line in joint space, from where the arm is being held to where it was asked to
go. Before anything moves, that line is

  * CHECKED along its whole length against the declared workspace, at the
    elbow, the wrist and the tip, both as commanded and as the arm will really
    be (the commanded pose plus the offset the arm shows right now: gravity sag
    and friction), with a margin for what neither can see; and
  * PACED, cut into setpoints a fixed period apart, sized so that neither the
    tool point nor any joint can be asked to move faster than the declared
    limits.

Both answers come out of arithmetic before the bus is touched, so a refusal
costs no motion at all. This module never talks to a servo; the actuator
streams what it returns.

Why a line in joint space and not in Cartesian space: the servos interpolate
nothing, and the path a position servo really takes between two nearby
setpoints is, to within its tracking error, the joint-space line between them.
Checking that line densely is checking the path the arm will take. A single
large Goal_Position jump is not a line at all: each servo slews at its own top
speed and the joints arrive at different times.

What this replaces, as EV-03 found it (simulation, October 2026): only the
target was checked, and only as commanded. Targets on the floor face itself
were accepted, gravity sag left the arm resting below them, and full-slew
jumps overshot on the way: the tip went up to 13 mm below the declared floor,
and the driver's own receipts put it outside the workspace after 270 to 450
moves per ten-minute run.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from so_arm101_actuator.errors import DeniedError
from so_arm101_actuator.kinematics import Box, Chain

#: Path samples are no further apart than this in any joint. 0.005 rad is about
#: 1.9 mm at the SO-ARM101's full 370 mm reach.
PATH_SAMPLE_RAD = 0.005

#: The points of the arm the workspace is checked at.
CHECKED_POINTS = ("tip", "wrist", "elbow")

#: Setpoint period of a paced move. The servos take Goal_Position and nothing
#: else, so a speed limit is a stream of small setpoints at a fixed rate.
PACE_PERIOD_S = 0.02

#: Plan to this fraction of a speed limit, not to the limit itself: a servo
#: catching up with a setpoint stream briefly runs faster than the stream.
PACE_FRACTION = 0.8

#: The longest one paced move may take. A very small `speed` is a real request
#: for a slow move, so it needs a ceiling or it is a hang that holds the bus.
MAX_MOVE_S = 60.0

#: Joint speed limit used only when the manifest declares no kinematic chain
#: (so the tool's speed cannot be computed) and no joint limit either.
FALLBACK_JOINT_DPS = 60.0


@dataclass
class Plan:
    """A checked, paced line. ``setpoints[-1]`` is the goal."""

    start: dict[str, float]
    goal: dict[str, float]
    setpoints: list[dict[str, float]]
    step_s: float
    duration_s: float
    #: The closest any checked point came to a face of the workspace along the
    #: line, as commanded or as predicted (None when no workspace is declared).
    min_clearance_mm: float | None
    #: Which point, on which series, at which face (for the receipt).
    closest: dict | None
    tool_speed_limit_mps: float | None
    joint_speed_limit_dps: float | None
    tool_path_mm: float | None
    notes: list[str] = field(default_factory=list)


def validate_speed(speed: object) -> float:
    """``speed`` as the fraction of the declared limits a move may use.

    Validated, never clamped: 0 means "do not move", which is not a slower move
    but a different request, and 1.5 is a caller who thinks this scale means
    something it does not. A subnormal speed is refused later, as too slow,
    once the move's length is known.
    """
    if isinstance(speed, bool) or not isinstance(speed, (int, float)):
        raise DeniedError("bad_args", f"speed must be a number, got {speed!r}")
    value = float(speed)
    if not math.isfinite(value) or not (0.0 < value <= 1.0):
        raise DeniedError("bad_args", f"speed must be greater than 0 and at most 1, got {speed!r}")
    return value


def _lerp(start: dict[str, float], goal: dict[str, float], s: float) -> dict[str, float]:
    return {j: start[j] + (goal[j] - start[j]) * s for j in goal}


#: How much closer to a face than it started a move that begins inside the
#: margin (or outside the box) may come on its way, provided it ENDS at least the
#: margin inside. Without it, an arm resting near or past a face could not
#: leave: the first few millimetres of almost any joint-space line out of a pose
#: move the tip a little the wrong way. A move that starts inside the box may use
#: it only down to the face itself, never past it; only an arm that is already
#: outside may go up to this much further out on its way back in. Because the
#: move must end fully inside, the allowance cannot be chained into a creep: the
#: next move starts with the whole margin and is held to it.
RECOVERY_DIP_MM = 5.0


def check_line(chain: Chain, box: Box | None, start: dict[str, float], goal: dict[str, float], *,
               offset: dict[str, float], margin_mm: float, samples: int
               ) -> tuple[float | None, dict | None]:
    """Refuse the line if any checked point gets too close to, or past, a face.

    Each point (tip, wrist, elbow) is checked on two series: as COMMANDED (the
    line itself) and as PREDICTED (the line plus ``offset``, the difference
    between where the arm is and where it is being held, which is gravity sag
    and friction and does not go away while it moves).

    A point that starts at least ``margin_mm`` inside the box must stay at
    least that far inside along the whole line. A point that starts closer (or
    outside the box: an arm pushed out, or parked badly) may not come any
    closer than it started, unless the move ends with it at least the margin
    inside, in which case it may dip by up to RECOVERY_DIP_MM on the way: down
    to the face and no further if it started inside the box, and that much
    further out if it started outside. That keeps an arm near or past a face
    recoverable (arm.home) without letting anything walk it further out.

    Returns the closest clearance seen and where. Raises DeniedError
    ``path_leaves_workspace`` naming the first sample that fails.
    """
    if box is None:
        return None, None
    series = {"commanded": None}
    if any(abs(v) > 0.0 for v in offset.values()):
        series["predicted"] = offset
    clear: dict[tuple[str, str], list[float]] = {}
    where: dict[tuple[str, str], list[tuple[float, float, float]]] = {}
    for i in range(samples + 1):
        q = _lerp(start, goal, i / samples)
        for kind, off in series.items():
            pose = q if off is None else {j: v + off.get(j, 0.0) for j, v in q.items()}
            points = chain.points(pose)
            for name in CHECKED_POINTS:
                clear.setdefault((kind, name), []).append(box.clearance(points[name]))
                where.setdefault((kind, name), []).append(points[name])

    closest_c, closest = math.inf, None
    for (kind, name), values in clear.items():
        first, last = values[0], values[-1]
        if first >= margin_mm:
            floor = margin_mm                           # keep the whole margin
        elif last >= margin_mm and first >= 0.0:
            floor = max(0.0, first - RECOVERY_DIP_MM)   # leaving the band: never past the face
        elif last >= margin_mm:
            floor = first - RECOVERY_DIP_MM             # coming back in from outside
        else:
            floor = first                               # staying in the band: never closer
        for i, c in enumerate(values):
            point = where[(kind, name)][i]
            if c < closest_c:
                closest_c = c
                closest = {"point": name, "series": kind, "fraction_of_path": round(i / samples, 3),
                           "face": box.nearest_face(point), "at_mm": [round(v, 1) for v in point]}
            if c < margin_mm and c < floor - 1e-6:
                raise DeniedError(
                    "path_leaves_workspace",
                    f"the {name}, {kind}, would come {c:.1f} mm from the workspace face "
                    f"{box.nearest_face(point)} at {i / samples:.0%} of the way (the margin is "
                    f"{margin_mm:g} mm; the move starts {first:.1f} mm and ends {last:.1f} mm "
                    f"inside). The ends of a move can both be inside the box while the path "
                    f"between them is not. Nothing was moved.")
    return (None if math.isinf(closest_c) else round(closest_c, 2)), closest


def pace_line(chain: Chain, start: dict[str, float], goal: dict[str, float], *, samples: int,
              speed: float, tool_mps: float, joint_dps: float | None,
              period_s: float = PACE_PERIOD_S, fraction: float = PACE_FRACTION,
              max_duration_s: float = MAX_MOVE_S) -> tuple[list[dict[str, float]], float, float, float]:
    """Cut the line into setpoints no faster than the limits allow.

    The line is time-parametrised along its samples: each stretch takes as long
    as the slower of the tool's arc length at the tool speed and the largest
    joint's angle at the joint speed (both scaled by ``speed`` and
    ``fraction``). Setpoints are then spaced evenly in time, no more than
    ``period_s`` apart. Returns (setpoints, step_s, duration_s, tool_path_mm).
    """
    span = max((abs(goal[j] - start[j]) for j in goal), default=0.0)
    tool_rate = tool_mps * 1000.0 * speed * fraction          # mm/s
    joint_rate = math.radians(joint_dps) * speed * fraction if joint_dps else None  # rad/s
    if not (tool_rate > 0.0 and math.isfinite(tool_rate)) or (
            joint_rate is not None and not (joint_rate > 0.0 and math.isfinite(joint_rate))):
        raise DeniedError("too_slow", f"speed {speed!r} is too small to plan a move with")
    if span == 0.0:
        return [dict(goal)], 0.0, 0.0, 0.0
    times = [0.0]
    path = 0.0
    last_tip = chain.points(start)["tip"]
    joint_step = span / samples
    for i in range(1, samples + 1):
        tip = chain.points(_lerp(start, goal, i / samples))["tip"]
        stretch = math.dist(tip, last_tip)
        path += stretch
        dt = stretch / tool_rate
        if joint_rate is not None:
            dt = max(dt, joint_step / joint_rate)
        times.append(times[-1] + dt)
        last_tip = tip
    duration = times[-1]
    if not math.isfinite(duration) or duration > max_duration_s:
        raise DeniedError(
            "too_slow",
            f"at speed {speed:g} this move would take {duration:.0f} s, longer than the "
            f"{max_duration_s:g} s one move may hold the arm. Use a larger speed.")
    if duration <= 0.0:
        return [dict(goal)], 0.0, 0.0, path
    steps = max(1, math.ceil(duration / period_s))
    step_s = duration / steps
    setpoints = []
    i = 0
    for k in range(1, steps + 1):
        t = duration * k / steps
        while i < samples and times[i + 1] < t:
            i += 1
        t0, t1 = times[i], times[min(i + 1, samples)]
        local = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
        s = min(1.0, (i + min(1.0, max(0.0, local))) / samples)
        setpoints.append(_lerp(start, goal, s))
    setpoints[-1] = dict(goal)
    return setpoints, step_s, duration, path


def plan(chain: Chain | None, box: Box | None, start: dict[str, float], goal: dict[str, float], *,
         offset: dict[str, float], margin_mm: float, speed: float, limits: dict,
         check_path: bool = True) -> Plan:
    """Check and pace the joint-space line from ``start`` to ``goal``.

    ``start``, ``goal`` and ``offset`` are radians keyed by joint, over the same
    joints. ``limits`` is :func:`kinematics.declared_speed_limits`. With no
    chain, the workspace cannot be checked: a declared workspace then refuses
    the move outright rather than letting it go unchecked, and without one the
    move is paced by the joint limit alone.
    """
    span = max((abs(goal[j] - start[j]) for j in goal), default=0.0)
    samples = max(1, math.ceil(span / PATH_SAMPLE_RAD))
    notes: list[str] = []
    if chain is None:
        if box is not None and check_path:
            raise DeniedError(
                "no_kinematics",
                "the manifest declares a workspace but no kinematic chain, so the path of this "
                "move cannot be checked against it. Nothing was moved.")
        joint_dps = limits.get("joint_dps") or FALLBACK_JOINT_DPS
        joint_rate = math.radians(joint_dps) * speed * PACE_FRACTION
        duration = span / joint_rate if joint_rate > 0 else math.inf
        if not math.isfinite(duration) or duration > MAX_MOVE_S:
            raise DeniedError(
                "too_slow",
                f"at speed {speed:g} this move would take longer than the {MAX_MOVE_S:g} s one "
                f"move may hold the arm. Use a larger speed.")
        steps = max(1, math.ceil(duration / PACE_PERIOD_S)) if duration > 0 else 1
        setpoints = [_lerp(start, goal, k / steps) for k in range(1, steps + 1)]
        setpoints[-1] = dict(goal)
        return Plan(start, goal, setpoints, duration / steps if duration > 0 else 0.0, duration,
                    None, None, None, joint_dps * speed, None,
                    ["no kinematic chain: paced by joint speed only"])
    if check_path:
        min_clear, closest = check_line(chain, box, start, goal, offset=offset,
                                        margin_mm=margin_mm, samples=samples)
    else:
        min_clear, closest = None, None
        notes.append("path not checked")
    setpoints, step_s, duration, path = pace_line(
        chain, start, goal, samples=samples, speed=speed,
        tool_mps=limits["tool_mps"], joint_dps=limits.get("joint_dps"))
    return Plan(start, goal, setpoints, step_s, duration, min_clear, closest,
                limits["tool_mps"] * speed, (limits.get("joint_dps") or 0.0) * speed or None,
                path, notes)
