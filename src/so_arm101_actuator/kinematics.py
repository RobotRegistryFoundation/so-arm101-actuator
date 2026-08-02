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


def _read_chain(path: str | None = None) -> list[dict]:
    import os
    from pathlib import Path

    source = path or os.environ.get(config.MANIFEST_ENV, "")
    if not source:
        return []
    try:
        text = Path(source).read_text()
    except OSError:
        return []
    if not text.startswith("---"):
        return []
    end = text.find("\n---", 3)
    if end == -1:
        return []
    try:
        import yaml

        front = yaml.safe_load(text[3:end]) or {}
    except Exception:
        return []
    return ((front.get("physics") or {}).get("kinematics") or [])


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

    tip = tip_offset_mm if tip_offset_mm is not None else DEFAULT_TIP_OFFSET_MM
    moved = _apply(rotation, tip)
    return (round(position[0] + moved[0], 2),
            round(position[1] + moved[1], 2),
            round(position[2] + moved[2], 2))


def max_reach_mm(manifest_path: str | None = None) -> float:
    """Farthest the tip can get from the base — the sum of the link lengths."""
    chain = _read_chain(manifest_path)
    total = sum(float(j.get("a_mm", 0.0)) for j in chain if str(j.get("id")) != "gripper")
    return total + DEFAULT_TIP_OFFSET_MM[0]


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
    import os
    from pathlib import Path

    source = manifest_path or os.environ.get(config.MANIFEST_ENV, "")
    bounds = {}
    try:
        import yaml

        text = Path(source).read_text()
        front = yaml.safe_load(text[3:text.find("\n---", 3)]) or {}
        bounds = (((front.get("physics") or {}).get("workspace") or {})
                  .get("bounds_mm") or {})
    except Exception:
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
