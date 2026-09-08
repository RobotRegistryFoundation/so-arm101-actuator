"""The analytic tool-down solve: what it answers, and what it refuses.

No hardware and no serial port anywhere in this file — inverse kinematics is
arithmetic over a manifest, and every claim here is checkable by reading one.
"""

from __future__ import annotations

import math

import pytest

from so_arm101_actuator import kinematics as kin
from tests.conftest import FIXTURE_MANIFEST

M = FIXTURE_MANIFEST


# --------------------------------------------------------------------------- #
# It solves
# --------------------------------------------------------------------------- #

def test_a_solved_pose_puts_the_tip_where_it_was_asked_to():
    """The one claim that matters: forward kinematics on the answer returns
    the question. The solve and the FK walk are derived independently, so this
    is a real cross-check rather than a tautology."""
    target = (150.0, 0.0, 50.0)
    solution = kin.solve_tool_down(target, M)
    assert solution.ok, solution.detail
    tip = kin.tip_position_mm(solution.joints, M)
    assert math.dist(tip, target) < 0.01


def test_it_solves_off_axis_targets_by_swinging_the_base():
    """shoulder_pan is pure azimuth — the planar solve is the same problem
    rotated, so an off-axis target must land as precisely as an on-axis one."""
    target = (170.0, 60.0, 30.0)
    solution = kin.solve_tool_down(target, M)
    assert solution.ok, solution.detail
    assert solution.joints["shoulder_pan"] == pytest.approx(math.atan2(60.0, 170.0))
    assert math.dist(kin.tip_position_mm(solution.joints, M), target) < 0.01


def test_the_solved_tool_axis_really_does_point_straight_down():
    """The constraint that removes the redundancy, stated as geometry rather
    than as an angle sum: descending 10mm from the solved pose must move the
    tip 10mm in -z and nowhere else."""
    solution = kin.solve_tool_down((150.0, 0.0, 50.0), M)
    assert solution.ok
    lower = kin.solve_tool_down((150.0, 0.0, 40.0), M)
    assert lower.ok
    tip_high = kin.tip_position_mm(solution.joints, M)
    tip_low = kin.tip_position_mm(lower.joints, M)
    assert tip_high[2] - tip_low[2] == pytest.approx(10.0, abs=0.02)
    assert tip_high[0] == pytest.approx(tip_low[0], abs=0.02)


def test_wrist_roll_is_a_null_direction_for_the_tip():
    """Why the solver leaves wrist_roll alone and the caller holds it: the roll
    axis and every remaining link offset lie along the same line, so spinning it
    moves the tip by exactly nothing. If this ever fails, `arm.move_to` may no
    longer hold the roll at its current value."""
    solution = kin.solve_tool_down((150.0, 0.0, 50.0), M)
    assert solution.ok
    still = kin.tip_position_mm({**solution.joints, "wrist_roll": 0.0}, M)
    rolled = kin.tip_position_mm({**solution.joints, "wrist_roll": 1.2}, M)
    assert math.dist(still, rolled) < 0.01


# --------------------------------------------------------------------------- #
# It refuses
# --------------------------------------------------------------------------- #

def test_a_point_beyond_the_links_is_unreachable_not_approximated():
    solution = kin.solve_tool_down((200.0, 0.0, 100.0), M)
    assert not solution.ok
    assert solution.code == "unreachable"
    assert "250mm" in solution.detail  # says WHAT it ran out of
    assert solution.joints == {}


def test_a_solve_outside_the_declared_limits_names_the_joint():
    solution = kin.solve_tool_down((100.0, 50.0, 20.0), M)
    assert not solution.ok
    assert solution.code == "joint_limits"
    assert "shoulder_lift" in solution.detail


def test_a_manifest_with_no_chain_is_refused_not_guessed(tmp_path):
    bare = tmp_path / "ROBOT.md"
    bare.write_text("---\nmetadata:\n  robot_name: nothing\n---\n\n# nothing\n")
    solution = kin.solve_tool_down((100.0, 0.0, 50.0), str(bare))
    assert not solution.ok
    assert solution.code == "no_kinematics"


def test_a_manifest_naming_another_solver_is_refused(tmp_path):
    """The manifest chooses the solver. Substituting ours because it happens to
    be the one compiled in would return a number nobody asked for."""
    source = open(M).read().replace("ik_provider: inhouse-so-arm101",
                                    "ik_provider: someone-elses-ik")
    other = tmp_path / "ROBOT.md"
    other.write_text(source)
    solution = kin.solve_tool_down((150.0, 0.0, 50.0), str(other))
    assert not solution.ok
    assert solution.code == "ik_provider_mismatch"
    assert solution.provider == "someone-elses-ik"


def test_a_chain_the_solver_cannot_fit_is_refused_not_driven(tmp_path):
    """The topology assumption made explicit. A chain whose links do not form
    the shoulder/elbow/wrist arrangement the closed form assumes still produces
    ANGLES — they just do not put the tip on the target, which is the most
    dangerous possible way to be wrong. FK catches it."""
    source = open(M).read().replace("      axis: y\n      limits_deg:\n        - -90\n"
                                    "        - 90\n      length_mm: 60\n      a_mm: 60",
                                    "      axis: z\n      limits_deg:\n        - -90\n"
                                    "        - 90\n      length_mm: 60\n      a_mm: 60")
    bent = tmp_path / "ROBOT.md"
    bent.write_text(source)
    solution = kin.solve_tool_down((150.0, 0.0, 50.0), str(bent))
    assert not solution.ok
    assert solution.code == "frame_disagreement"


# --------------------------------------------------------------------------- #
# It is the provider the manifest names
# --------------------------------------------------------------------------- #

def test_it_agrees_with_the_reference_inhouse_solver():
    """`solve_tool_down` is a port of `robot_md.kinematics.Kinematics.ik_reach`,
    the solver `ik_provider: inhouse-so-arm101` refers to. The port exists
    because robot-md is an authoring CLI and is not installed on a robot; this
    test is the thing that stops the copy drifting from the original.

    Skipped when robot-md is not importable, which is the normal case on a
    robot and the whole reason for the port.
    """
    robot_md_kin = pytest.importorskip(
        "robot_md.kinematics",
        reason="robot-md (the authoring CLI) is not installed; the port stands alone",
    )
    reference = robot_md_kin.Kinematics(kin.frontmatter(M))

    compared = 0
    for x in range(-150, 200, 25):
        for y in range(-150, 151, 25):
            for z in range(0, 160, 20):
                target = (float(x), float(y), float(z))
                ours = kin.solve_tool_down(target, M)
                try:
                    theirs = reference.ik_reach(target)
                except robot_md_kin.KinematicsError:
                    # The reference raises exactly where the port refuses. It
                    # does not distinguish "beyond the links" from "outside the
                    # limits"; we do, and both of our codes belong here.
                    assert not ours.ok, f"{target}: we solved what the reference refused"
                    assert ours.code in ("unreachable", "joint_limits"), ours.code
                    continue
                assert ours.ok, f"{target}: reference solved it, we refused ({ours.detail})"
                for joint, angle in theirs.items():
                    assert ours.joints[joint] == pytest.approx(angle, abs=1e-9), joint
                compared += 1
    assert compared > 50, f"only {compared} targets actually solved; grid is not exercising it"


# --------------------------------------------------------------------------- #
# What is on the end of the arm
# --------------------------------------------------------------------------- #

def test_a_declared_gripper_is_reported_as_the_tool():
    assert kin.declared_tool(M) == "gripper"


def test_the_manifest_can_name_a_pen(tmp_path):
    source = open(M).read().replace("    ik_provider: inhouse-so-arm101",
                                    "    tool:\n      id: pen\n"
                                    "    ik_provider: inhouse-so-arm101")
    penned = tmp_path / "ROBOT.md"
    penned.write_text(source)
    assert kin.declared_tool(str(penned)) == "pen"


def test_an_operator_who_swapped_the_tool_outranks_the_manifest(monkeypatch):
    monkeypatch.setenv("SO_ARM101_TOOL", "pen")
    assert kin.declared_tool(M) == "pen"


def test_a_bare_flange_is_reported_as_nothing_not_as_a_gripper(monkeypatch):
    monkeypatch.setenv("SO_ARM101_TOOL", "none")
    assert kin.declared_tool(M) is None


def test_an_undescribed_end_effector_is_unknown_not_invented(tmp_path):
    source = open(M).read()
    start = source.index("    gripper:\n")
    end = source.index("    ik_provider:")
    stripped = tmp_path / "ROBOT.md"
    stripped.write_text(source[:start] + source[end:])
    assert kin.declared_tool(str(stripped)) is None


# --------------------------------------------------------------------------- #
# Reading the manifest
# --------------------------------------------------------------------------- #

def test_frontmatter_is_reparsed_when_the_manifest_changes(tmp_path):
    """The cache is keyed on mtime so a re-signed manifest lands without a
    restart. A cache that never invalidated would pin a robot to the geometry
    it booted with."""
    path = tmp_path / "ROBOT.md"
    path.write_text("---\nmetadata:\n  robot_name: before\n---\n\n# x\n")
    assert kin.frontmatter(str(path))["metadata"]["robot_name"] == "before"
    import os
    path.write_text("---\nmetadata:\n  robot_name: after\n---\n\n# x\n")
    os.utime(path, (0, 0))  # force a different mtime, not merely a later one
    assert kin.frontmatter(str(path))["metadata"]["robot_name"] == "after"


def test_the_tip_offset_comes_from_the_manifest_not_the_constant(tmp_path):
    source = open(M).read().replace("        - 30\n        - 0\n        - 0",
                                    "        - 55\n        - 0\n        - 0")
    longer = tmp_path / "ROBOT.md"
    longer.write_text(source)
    assert kin.tip_offset_from_manifest(str(longer))[0] == 55.0
    # A longer tool reaches farther, and both the reach bound and FK must know.
    assert kin.max_reach_mm(str(longer)) == kin.max_reach_mm(M) + 25.0
