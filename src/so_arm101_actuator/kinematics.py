"""Where the gripper tip is, in the arm's own frame.

Forward kinematics only. No IK here, deliberately: the existing IK insists the
tool axis points straight down, and on this arm that demands wrist_flex ~81 deg
against a measured ~26 deg ceiling — so zero of 10,780 sampled workspace points
solve inside the joint limits. FK has no such problem. It answers "given these
joint angles, where is the tip", which is a pure geometry question with exactly
one answer.

FK is what makes a phone useful as the robot's eye. The blocker on this rig has
never been that nobody could see the brick; it is that no measurement of the
brick could be expressed in the ARM's frame. The camera calibration that was
supposed to bridge that is floored at ~75 mm even with a perfect sensor, because
its detector correlates the FK tip against the centroid of every moving pixel —
which is dominated by the forearm, not the tip.

A person tapping the gripper tip on a phone screen is the detector that fixes
that: it localises ONE KNOWN POINT. Pair enough of those taps with the FK tip
position at the same instant and a rigid transform falls out. Hence this module.

Frame convention, from the manifest's `base_frame`: z up, x forward. Lengths in
millimetres to match the manifest and the workspace bounds.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from so_arm101_actuator import config

#: Distance from the wrist-roll flange to the point BETWEEN the jaws — the spot
#: a person means when they tap "the gripper tip". From the manifest's
#: physics.solver.gripper.tip_offset_mm.
DEFAULT_TIP_OFFSET_MM = (30.0, 0.0, 0.0)


def _rot_x(a: float) -> list[list[float]]:
    c, s = math.cos(a), math.sin(a)
    return [[1, 0, 0], [0, c, -s], [0, s, c]]


def _rot_y(a: float) -> list[list[float]]:
    c, s = math.cos(a), math.sin(a)
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]


def _rot_z(a: float) -> list[list[float]]:
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def _matmul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def _apply(r: list[list[float]], v: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(sum(r[i][k] * v[k] for k in range(3)) for i in range(3))


def chain_from_manifest(path: str | None = None) -> list[dict]:
    """The joint chain, in order, as the manifest declares it.

    Read from the manifest rather than hardcoded so a different arm — or a
    re-measured link length on this one — needs no code change.
    """
    data = config.load_manifest_calibration(path)
    chain = data.get("chain") or []
    if chain:
        return chain
    # The loader predates this module and does not surface the chain yet; read
    # it directly rather than duplicating the parse in two places later.
    return _read_chain(path)


#: Parsed frontmatter by path -> (mtime, dict). The closed-loop reach evaluates
#: forward kinematics nine times per step, each of which asks the manifest for
#: the chain, and a Cartesian solve asks four more questions of it. Re-reading
#: and re-parsing a 300-line YAML document every time made a workspace sweep
#: take minutes of pure parsing. Keyed on mtime so a re-signed manifest is
#: picked up without a restart, exactly as the gateway's own cache does it.
_FRONTMATTER_CACHE: dict[str, tuple[float, dict]] = {}


def frontmatter(path: str | None = None) -> dict:
    """The manifest's YAML frontmatter, or {} when there is nothing to read.

    One parser for the whole module. Every geometry question below — the chain,
    the workspace box, the declared IK provider, the tool on the end — is a
    lookup in this dict, so a manifest that cannot be read fails the same way
    everywhere instead of once per feature.

    The returned dict is the cached object, not a copy: this is a read-only
    view of a file, and callers that mutate it are lying to every later reader.
    """
    import os
    from pathlib import Path

    source = path or os.environ.get(config.MANIFEST_ENV, "")
    if not source:
        return {}
    try:
        mtime = os.path.getmtime(source)
    except OSError:
        return {}
    cached = _FRONTMATTER_CACHE.get(source)
    if cached is not None and cached[0] == mtime:
        return cached[1]
    try:
        text = Path(source).read_text()
    except OSError:
        return {}
    front: dict = {}
    if text.startswith("---"):
        end = text.find("\n---", 3)
        if end != -1:
            try:
                import yaml

                parsed = yaml.safe_load(text[3:end]) or {}
                front = parsed if isinstance(parsed, dict) else {}
            except Exception:
                front = {}
    _FRONTMATTER_CACHE[source] = (mtime, front)
    return front


def _read_chain(path: str | None = None) -> list[dict]:
    return ((frontmatter(path).get("physics") or {}).get("kinematics") or [])


def tip_offset_from_manifest(manifest_path: str | None = None
                             ) -> tuple[float, float, float]:
    """`physics.solver.gripper.tip_offset_mm`, falling back to the constant.

    The distance from the wrist flange to the grasp point is a property of
    whatever is bolted on, so a rig that swapped the gripper for a pen is
    described by editing its manifest, not by editing this file.
    """
    raw = (((frontmatter(manifest_path).get("physics") or {}).get("solver") or {})
           .get("gripper") or {}).get("tip_offset_mm")
    if isinstance(raw, (list, tuple)) and len(raw) == 3:
        try:
            return (float(raw[0]), float(raw[1]), float(raw[2]))
        except (TypeError, ValueError):
            pass
    return DEFAULT_TIP_OFFSET_MM


def tip_position_mm(joint_rad: dict[str, float],
                    manifest_path: str | None = None,
                    tip_offset_mm: tuple[float, float, float] | None = None,
                    ) -> tuple[float, float, float]:
    """Gripper tip position in the arm's base frame, in millimetres.

    Walks the manifest's chain in order. Each joint rotates about its declared
    axis, then translates `a_mm` along x and `d_mm` along z — the convention the
    manifest's own link lengths are written in.

    The gripper joint is skipped: opening and closing the jaws moves the
    fingers, not the point between them, and that midpoint is what a person
    means by "the tip".
    """
    chain = _read_chain(manifest_path)
    if not chain:
        raise ValueError("manifest declares no kinematic chain")

    rotation = [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]
    position = (0.0, 0.0, 0.0)

    for joint in chain:
        name = str(joint.get("id"))
        if name == "gripper":
            continue
        angle = float(joint_rad.get(name, 0.0))
        axis = str(joint.get("axis", "z")).lower()
        rot = {"x": _rot_x, "y": _rot_y, "z": _rot_z}.get(axis, _rot_z)(angle)
        rotation = _matmul(rotation, rot)

        offset = (float(joint.get("a_mm", 0.0)), 0.0, float(joint.get("d_mm", 0.0)))
        moved = _apply(rotation, offset)
        position = (position[0] + moved[0], position[1] + moved[1], position[2] + moved[2])

    tip = (tip_offset_mm if tip_offset_mm is not None
           else tip_offset_from_manifest(manifest_path))
    moved = _apply(rotation, tip)
    return (round(position[0] + moved[0], 2),
            round(position[1] + moved[1], 2),
            round(position[2] + moved[2], 2))


def max_reach_mm(manifest_path: str | None = None) -> float:
    """Farthest the tip can get from the base — the sum of the link lengths."""
    chain = _read_chain(manifest_path)
    total = sum(float(j.get("a_mm", 0.0)) for j in chain if str(j.get("id")) != "gripper")
    return total + tip_offset_from_manifest(manifest_path)[0]


def reachable(point_mm: tuple[float, float, float],
              manifest_path: str | None = None) -> tuple[bool, str]:
    """Can the arm physically get its tip here?

    Bounded by the arm's own link geometry rather than by the manifest's
    declared workspace box, because on this robot those two disagree: the box is
    z 0..250mm, while forward kinematics puts every genuinely reachable pose at
    NEGATIVE z. They are expressed in different frames, and enforcing the box
    against FK coordinates refuses every target including ones the arm is
    already holding.

    Which frame is "right" is not something this function can settle. What it
    can do is bound motion by physics — the tip cannot be farther from the base
    than the links are long — and let `within_workspace` report the declared box
    separately, so the disagreement stays visible instead of silently blocking
    everything.
    """
    limit = max_reach_mm(manifest_path)
    distance = math.sqrt(sum(v * v for v in point_mm))
    if distance > limit:
        return False, (f"{distance:.0f}mm from the base is beyond this arm's "
                       f"{limit:.0f}mm reach")
    return True, f"{distance:.0f}mm from the base, within the {limit:.0f}mm reach"


def frame_disagreement(joint_rad: dict[str, float],
                       manifest_path: str | None = None) -> str | None:
    """Warn when FK's frame and the declared workspace cannot both be right.

    Returns None when they agree. Exists because this discrepancy is invisible
    until something refuses to move, and then it looks like a broken arm rather
    than a manifest that does not describe this build.
    """
    tip = tip_position_mm(joint_rad, manifest_path)
    inside, _ = within_workspace(tip, manifest_path)
    if inside:
        return None
    return (f"The arm is physically at {tip} by its own geometry, but the "
            f"manifest's declared workspace excludes that point — so the "
            f"manifest's kinematic zeros do not match this build. Reaching is "
            f"bounded by the arm's real link lengths instead.")


def within_workspace(point_mm: tuple[float, float, float],
                     manifest_path: str | None = None) -> tuple[bool, str]:
    """Is this point inside the manifest's declared envelope?

    Returned as (ok, reason) so a refusal can say WHICH bound failed. "Outside
    the workspace" with no axis named is the kind of message that sends someone
    hunting through a config file.
    """
    bounds = (((frontmatter(manifest_path).get("physics") or {}).get("workspace") or {})
              .get("bounds_mm") or {})
    if not bounds:
        return True, "no workspace declared; not enforcing"

    for axis, value in zip("xyz", point_mm):
        span = bounds.get(axis)
        if not span or len(span) != 2:
            continue
        low, high = float(span[0]), float(span[1])
        if not (low <= value <= high):
            return False, (f"{axis}={value:.0f}mm is outside the declared "
                           f"workspace ({low:.0f} to {high:.0f}mm)")
    return True, "inside the declared workspace"


# --------------------------------------------------------------------------- #
# Closed-loop reaching
# --------------------------------------------------------------------------- #
#
# The analytic IK on this arm is unusable: it constrains the tool axis to point
# straight down, which at tabletop reach demands wrist_flex ~81 deg against a
# ~26 deg mechanical ceiling. Zero of 10,780 sampled workspace points solve
# inside the joint limits. It is not a tuning problem — the constraint is
# geometrically incompatible with this arm's link lengths.
#
# So don't solve. MEASURE. Read where the tip actually is, compare it to where
# it should be, take a small step that shrinks the gap, and look again. That is
# gradient descent on position error, and it has three properties that matter
# more here than elegance:
#
#   * It imposes no constraint on tool orientation, so it can approach at
#     whatever tilt the arm can physically manage.
#   * It uses the encoders, so it corrects its own error instead of trusting a
#     model that we already know disagrees with this robot — forward kinematics
#     currently reports a tip 20 cm below the arm's own base.
#   * Every step is bounded, so a bad Jacobian yields a small wrong move that
#     the next iteration corrects, rather than a joint slamming to a limit.
#
# The Jacobian is computed by finite differences rather than derived
# analytically. It is a handful of extra FK evaluations, all in-process and
# cheap, and it stays correct automatically if the manifest's link lengths are
# ever re-measured.

#: How far each joint is nudged when estimating the Jacobian numerically. Small
#: enough to be a good local gradient, large enough that FK rounding does not
#: dominate the difference.
JACOBIAN_EPS_RAD = 1e-4

#: Damping for the least-squares step, in millimetres of Jacobian scale. Larger
#: means smaller, safer, less exact steps near singular configurations.
DAMPING = 12.0

#: Largest joint change any single step may command. The safety property of this
#: whole approach: an error in the gradient produces a small wrong move that the
#: next measurement corrects, never a lunge.
MAX_STEP_RAD = 0.08

#: Joints the reach controller is allowed to move. wrist_roll is excluded — it
#: spins the gripper about its own axis and moves the tip almost not at all, so
#: including it lets the solver waste steps on a near-null direction.
REACH_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")


def jacobian(joint_rad: dict[str, float], joints: tuple[str, ...] = REACH_JOINTS,
             manifest_path: str | None = None) -> list[list[float]]:
    """d(tip position) / d(joint angle), by central differences. 3 x len(joints)."""
    columns = []
    for name in joints:
        plus = dict(joint_rad)
        minus = dict(joint_rad)
        plus[name] = plus.get(name, 0.0) + JACOBIAN_EPS_RAD
        minus[name] = minus.get(name, 0.0) - JACOBIAN_EPS_RAD
        a = tip_position_mm(plus, manifest_path)
        b = tip_position_mm(minus, manifest_path)
        columns.append([(a[i] - b[i]) / (2 * JACOBIAN_EPS_RAD) for i in range(3)])
    return [[columns[j][i] for j in range(len(joints))] for i in range(3)]


def _solve3(a: list[list[float]], b: list[float]) -> list[float]:
    """Solve a 3x3 system by Gaussian elimination with partial pivoting."""
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda r: abs(m[r][col]))
        if abs(m[pivot][col]) < 1e-12:
            return [0.0, 0.0, 0.0]
        m[col], m[pivot] = m[pivot], m[col]
        for r in range(3):
            if r == col:
                continue
            f = m[r][col] / m[col][col]
            for k in range(col, 4):
                m[r][k] -= f * m[col][k]
    return [m[i][3] / m[i][i] for i in range(3)]


def reach_step(joint_rad: dict[str, float],
               target_mm: tuple[float, float, float],
               joints: tuple[str, ...] = REACH_JOINTS,
               manifest_path: str | None = None,
               gain: float = 1.0) -> tuple[dict[str, float], float]:
    """One bounded step toward `target_mm`. Returns (new joint angles, error mm).

    Uses DAMPED LEAST SQUARES: dq = Jᵀ(JJᵀ + λ²I)⁻¹e.

    The plain transpose was tried first and converges far too slowly to be
    usable — it took a 325mm error down to 10mm and then crawled, gaining under
    2% per step, which looks indistinguishable from a blocked arm. The damping
    term is what keeps this from becoming a raw inverse: near a singularity an
    undamped inverse demands enormous joint motion to gain a millimetre, while
    λ smoothly trades exactness for a bounded step. On an arm with no force
    sensing, a slightly wrong small move beats a correct enormous one.
    """
    tip = tip_position_mm(joint_rad, manifest_path)
    error = [target_mm[i] - tip[i] for i in range(3)]
    distance = math.sqrt(sum(e * e for e in error))

    j = jacobian(joint_rad, joints, manifest_path)

    # A = JJᵀ + λ²I  (3x3)
    lam2 = DAMPING ** 2
    a = [[sum(j[r][c] * j[k][c] for c in range(len(joints))) + (lam2 if r == k else 0.0)
          for k in range(3)] for r in range(3)]
    solved = _solve3(a, error)
    dq = [sum(j[i][c] * solved[i] for i in range(3)) for c in range(len(joints))]
    alpha = gain

    out = dict(joint_rad)
    for c, name in enumerate(joints):
        step = alpha * dq[c]
        step = max(-MAX_STEP_RAD, min(MAX_STEP_RAD, step))
        out[name] = joint_rad.get(name, 0.0) + step
    return out, distance


# --------------------------------------------------------------------------- #
# Analytic IK — the `inhouse-so-arm101` provider the manifest names
# --------------------------------------------------------------------------- #
#
# This is the solver `physics.solver.ik_provider: inhouse-so-arm101` refers to:
# the 3-link planar solve with a straight-down tool axis that ships in the
# robot-md CLI as `robot_md.kinematics.Kinematics.ik_reach`. It is reproduced
# here rather than imported because a driver that must run on a robot cannot
# depend on the authoring CLI being installed next to it — `robot_md` is not in
# the deployed venv on the one rig this has ever run on. The port is
# line-for-line faithful, and `tests/test_kinematics_ik.py` asserts the two
# agree over a grid of targets whenever `robot_md` IS importable, so the copy
# cannot drift silently.
#
# Two things changed on the way across, both about honesty rather than maths:
#
#   * It returns a RESULT instead of raising. Every refusal here becomes a
#     signed deny on the wire, and a deny needs a stable machine-readable code
#     ("unreachable", "joint_limits") — not a message a client has to regex.
#   * The joint limits it checks come from the manifest's `limits_deg`, the same
#     place the reference reads them. The tighter, MEASURED envelope
#     (config.SAFE_RANGE_RAD) is checked by the caller, separately, so the two
#     refusals stay distinguishable: "this arm's design cannot do that" is a
#     different fact from "this particular arm was measured and cannot do that".
#
# Read the warning at the top of the closed-loop section above before reaching
# for this: the straight-down tool constraint is geometrically incompatible with
# the measured envelope of the SO-ARM101 this was written against. It solves
# inside the DECLARED limits for much of the tabletop and inside the MEASURED
# ones almost nowhere. That is a true fact about the arm, and `arm.move_to`
# reports it as a deny rather than pretending otherwise.

#: The provider id this module implements. A manifest naming a different solver
#: gets a deny, not a silent substitution — the number that comes back would
#: otherwise be from a solver nobody chose.
IK_PROVIDER_ID = "inhouse-so-arm101"

#: The four joints the analytic solve determines. wrist_roll is not among them:
#: it rotates about the tool axis, which on this arm is the axis the remaining
#: link offsets lie along, so it moves the tip exactly not at all. Holding it
#: where it was is therefore free, and the caller does exactly that.
IK_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")

#: How far FK may disagree with the target about where the solved pose puts the
#: tip before the solve is treated as unusable. A correct solve round-trips to
#: well under a micrometre; anything near a millimetre means the chain the
#: solver reasoned about is not the chain FK walks.
FK_ROUNDTRIP_TOLERANCE_MM = 1.0


@dataclass(frozen=True)
class IKSolution:
    """The outcome of one analytic solve.

    `ok` is the only thing a caller must branch on. When it is False, `code` is
    a stable identifier suitable for a deny receipt and `detail` is the sentence
    a person reads.
    """

    ok: bool
    code: str = ""
    detail: str = ""
    joints: dict[str, float] = field(default_factory=dict)
    provider: str = IK_PROVIDER_ID


def declared_ik_provider(manifest_path: str | None = None) -> str:
    """`physics.solver.ik_provider`, or "" when the manifest names none."""
    provider = (((frontmatter(manifest_path).get("physics") or {}).get("solver") or {})
                .get("ik_provider"))
    return str(provider) if provider else ""


def joint_limits_rad(manifest_path: str | None = None) -> dict[str, tuple[float, float]]:
    """Per-joint `limits_deg` from the manifest, in radians.

    These are the arm's DESIGN limits as declared. They are wider than the
    measured envelope in `config.SAFE_RANGE_RAD` on every rig seen so far, and
    the difference is the whole reason both checks exist.
    """
    out: dict[str, tuple[float, float]] = {}
    for joint in _read_chain(manifest_path):
        name = str(joint.get("id", ""))
        limits = joint.get("limits_deg")
        if not name or not isinstance(limits, (list, tuple)) or len(limits) != 2:
            continue
        try:
            out[name] = (math.radians(float(limits[0])), math.radians(float(limits[1])))
        except (TypeError, ValueError):
            continue
    return out


def declared_tool(manifest_path: str | None = None) -> str | None:
    """What is on the end of the arm: "pen", "gripper", or nothing known.

    Resolution order, most specific first:

      1. ``SO_ARM101_TOOL`` — an operator who swapped the end effector by hand
         says so here, and outranks a manifest that has not caught up.
      2. ``physics.solver.tool`` (a string, or an object with an ``id``).
      3. ``"gripper"`` when the manifest declares a gripper joint AND a tip
         offset — that is a description of a real end effector, not a guess.
      4. ``None``. Not knowing is reported as not knowing; naming a tool the
         manifest never mentioned would be a claim, and a client deciding
         whether it can draw would be deciding on our invention.

    ``SO_ARM101_TOOL=none`` is how an operator says the flange is bare.
    """
    import os

    raw = (os.environ.get("SO_ARM101_TOOL") or "").strip().lower()
    if raw:
        return None if raw in ("none", "null", "bare") else raw

    solver = (frontmatter(manifest_path).get("physics") or {}).get("solver") or {}
    tool = solver.get("tool")
    if isinstance(tool, dict):
        tool = tool.get("id")
    if isinstance(tool, str) and tool.strip():
        return tool.strip().lower()

    gripper = solver.get("gripper") or {}
    if gripper.get("joint_id") and gripper.get("tip_offset_mm"):
        return "gripper"
    return None


def solve_tool_down(target_mm: tuple[float, float, float],
                    manifest_path: str | None = None) -> IKSolution:
    """Solve for the tip at `target_mm` with the tool pointing straight down.

    Faithful port of `robot_md.kinematics.Kinematics.ik_reach`. The redundancy a
    5-DoF arm has when only a position is asked for is removed by pinning the
    tool axis to base -z, which is what a pen or a top-down grasp needs.

    Returns an :class:`IKSolution`. Never raises for a target it simply cannot
    reach, and never returns a pose it had to bend to produce: a solve that
    lands outside the declared joint limits comes back as `ok=False`, because a
    clamped angle is a different pose than the one asked for and reporting it as
    success is how an arm ends up somewhere nobody chose.
    """
    chain = _read_chain(manifest_path)
    if not chain:
        return IKSolution(False, "no_kinematics",
                          "this robot's manifest declares no kinematic chain, so "
                          "there is no geometry to solve against")

    provider = declared_ik_provider(manifest_path)
    if provider and provider != IK_PROVIDER_ID:
        return IKSolution(
            False, "ik_provider_mismatch",
            f"the manifest names IK provider {provider!r}; this driver implements "
            f"only {IK_PROVIDER_ID!r} and will not substitute another solver",
            provider=provider)

    by_id = {str(j.get("id")): j for j in chain}
    missing = [name for name in IK_JOINTS if name not in by_id]
    if missing:
        return IKSolution(False, "no_kinematics",
                          f"the chain is missing {', '.join(missing)}; the "
                          f"in-house solver needs {', '.join(IK_JOINTS)}")

    def _a(name: str) -> float:
        return float(by_id.get(name, {}).get("a_mm", 0.0) or 0.0)

    def _d(name: str) -> float:
        return float(by_id.get(name, {}).get("d_mm", 0.0) or 0.0)

    x, y, z = (float(v) for v in target_mm)
    l1 = _a("shoulder_lift")
    l2 = _a("elbow_flex")
    # Tool length along the tool's own +x, from the wrist_flex joint to the tip.
    l3 = _a("wrist_flex") + _a("wrist_roll") + abs(tip_offset_from_manifest(manifest_path)[0])
    d1 = _d("shoulder_pan")   # the riser the whole planar problem sits on

    pan = math.atan2(y, x)
    r = math.hypot(x, y)

    # Where the wrist must be for a vertical tool to land the tip on target.
    zw = (z + l3) - d1
    d2 = r * r + zw * zw
    d = math.sqrt(d2)
    if d > l1 + l2 - 1e-6:
        return IKSolution(False, "unreachable",
                          f"the wrist would have to sit {d:.0f}mm from the shoulder "
                          f"to put the tool straight down here; the upper arm and "
                          f"forearm together only span {l1 + l2:.0f}mm")
    if d < abs(l1 - l2) + 1e-6:
        return IKSolution(False, "unreachable",
                          f"{d:.0f}mm from the shoulder is inside the arm's own "
                          f"folded minimum of {abs(l1 - l2):.0f}mm")

    cos_elbow_int = max(-1.0, min(1.0, (l1 * l1 + l2 * l2 - d2) / (2.0 * l1 * l2)))
    elbow_flex = math.pi - math.acos(cos_elbow_int)

    alpha = math.atan2(zw, r)
    cos_beta = max(-1.0, min(1.0, (l1 * l1 + d2 - l2 * l2) / (2.0 * l1 * d)))
    # FK's +theta about y swings +x toward -z, i.e. positive means DOWN, while
    # the law-of-cosines derivation above assumes positive means up. Negating
    # shoulder_lift converts; elbow_flex stays positive, which is the elbow-UP
    # branch in this convention.
    shoulder_lift = -(alpha + math.acos(cos_beta))
    # Three y-rotations put the tool's +x at (cos(sum), 0, -sin(sum)) in the base
    # frame. Straight down is (0, 0, -1), so the three must sum to pi/2.
    wrist_flex = (math.pi / 2) - shoulder_lift - elbow_flex

    solved = {
        "shoulder_pan": pan,
        "shoulder_lift": shoulder_lift,
        "elbow_flex": elbow_flex,
        "wrist_flex": wrist_flex,
    }

    limits = joint_limits_rad(manifest_path)
    for name, angle in solved.items():
        span = limits.get(name)
        if span is None:
            continue
        lo, hi = span
        if not (lo <= angle <= hi):
            return IKSolution(
                False, "joint_limits",
                f"reaching that point with the tool vertical needs {name} at "
                f"{math.degrees(angle):+.1f}deg, outside its declared "
                f"[{math.degrees(lo):+.0f}, {math.degrees(hi):+.0f}]deg")

    # The solve and the forward model must agree about where this pose puts the
    # tip. They are derived independently — one from the law of cosines, one by
    # walking the chain — so a disagreement means the manifest describes a chain
    # the solver's topology assumptions do not fit, and the honest answer is to
    # refuse rather than to drive somewhere on the strength of one of them.
    tip = tip_position_mm(solved, manifest_path)
    drift = math.dist(tip, (x, y, z))
    if drift > FK_ROUNDTRIP_TOLERANCE_MM:
        return IKSolution(
            False, "frame_disagreement",
            f"the solved pose puts the tip at {tip} by this robot's own forward "
            f"kinematics, {drift:.1f}mm from the {(round(x), round(y), round(z))} "
            f"that was asked for — the manifest's chain does not match the "
            f"topology the in-house solver assumes")

    return IKSolution(True, "", "ok", solved)


# --------------------------------------------------------------------------- #
# Warm starts: the nearest pose in the safe envelope, by the arm's own geometry
# --------------------------------------------------------------------------- #

_SAFE_TABLE_CACHE: dict = {}


def _safe_table(manifest_path: str | None, safe_ranges: dict, n: int = 40):
    """(r, z, lift, elbow, wrist) rows over the safe envelope at pan = 0, wrist_roll = 0.

    Pan is a pure rotation about the base z axis, so reachability only depends
    on the tip's radius and height; sampling the three planar joints is enough.
    Built once per (manifest, ranges) and kept: 64,000 forward solves, a few
    seconds, and then every warm start is a table lookup.
    """
    key = (manifest_path, tuple(sorted((j, tuple(v)) for j, v in safe_ranges.items())), n)
    rows = _SAFE_TABLE_CACHE.get(key)
    if rows is None:
        def span(joint, default):
            lo, hi = safe_ranges.get(joint, default)
            return [lo + (hi - lo) * i / (n - 1) for i in range(n)]
        rows = []
        for lift in span("shoulder_lift", (-3.14, 3.14)):
            for elbow in span("elbow_flex", (-3.14, 3.14)):
                for wrist in span("wrist_flex", (-3.14, 3.14)):
                    x, y, z = tip_position_mm({"shoulder_pan": 0.0, "shoulder_lift": lift, "elbow_flex": elbow,
                                               "wrist_flex": wrist, "wrist_roll": 0.0}, manifest_path)
                    rows.append((math.hypot(x, y), z, lift, elbow, wrist))
        _SAFE_TABLE_CACHE[key] = rows
    return rows


def nearest_safe_pose(target_mm: tuple[float, float, float], safe_ranges: dict,
                      manifest_path: str | None = None) -> tuple[dict[str, float], float]:
    """The joint pose inside the safe envelope whose tip is nearest ``target_mm``.

    Returns (joints, predicted_error_mm). Used to warm-start :meth:`reach_point`
    so the servo never has to find its way around a joint limit: a target that
    is reachable at all is reachable from here in a handful of steps.
    """
    x, y, z = target_mm
    r = math.hypot(x, y)
    best = None
    for row in _safe_table(manifest_path, safe_ranges):
        d = math.hypot(row[0] - r, row[1] - z)
        if best is None or d < best[0]:
            best = (d, row)
    d, (_, _, lift, elbow, wrist) = best
    lo, hi = safe_ranges.get("shoulder_pan", (-3.14, 3.14))
    pan = max(lo, min(hi, math.atan2(y, x)))
    pose = {"shoulder_pan": pan, "shoulder_lift": lift, "elbow_flex": elbow, "wrist_flex": wrist}
    tip = tip_position_mm({**pose, "wrist_roll": 0.0}, manifest_path)
    predicted = math.sqrt(sum((tip[i] - target_mm[i]) ** 2 for i in range(3)))
    return pose, predicted
