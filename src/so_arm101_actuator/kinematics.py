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
