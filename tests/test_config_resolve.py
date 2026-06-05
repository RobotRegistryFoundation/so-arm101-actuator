"""TDD (A2): resolve_joints_from_manifest — manifest-authoritative rad<->tick.

The root-cause bug of the whole pick-place saga: the SHIPPED default maps the gripper
around tick 2048, but the real jaw band on Bob is close~1529 / open~1697, so the rad
"close" command was actually WIDER than open and the jaws never shut. After resolving
from commissioned endpoints, "close" must map NARROWER (lower tick) than "open" and land
inside the real band — closing the bug at the source.
"""

from __future__ import annotations

import math

import pytest

from so_arm101_actuator import config


# Bob's commissioned endpoints (the values discovered empirically: open 1697 / close 1529).
BOB_FRONTMATTER = {
    "physics": {
        "solver": {
            "gripper": {
                "joint_id": "gripper",
                "open_steps": 1697,
                "close_steps": 1529,
                "close_steps_empty": 1457,
            },
        },
        "kinematics": [
            {
                "id": "shoulder_pan",
                "servo_id": 1,
                "encoder_sign": 1,
                "min_steps": 1100,
                "max_steps": 3000,
                "endpoint_source": "commissioned",
            },
        ],
    }
}


def test_resolve_gripper_close_narrower_than_open():
    joints = config._resolve_joints_from_frontmatter(BOB_FRONTMATTER)
    g = joints["gripper"]
    tick_at_min_rad = config.rad_to_ticks_spec(g, g["min_rad"])
    tick_at_max_rad = config.rad_to_ticks_spec(g, g["max_rad"])
    # The rad endpoints land exactly on the commissioned jaw band.
    assert {tick_at_min_rad, tick_at_max_rad} == {1529, 1697}
    # "close" (min_rad end) is NARROWER => a LOWER tick than "open".
    assert tick_at_min_rad < tick_at_max_rad
    # Both inside the real band — NOT centered on the broken default 2048.
    assert 1529 <= tick_at_min_rad <= 1697
    assert 1529 <= tick_at_max_rad <= 1697


def test_default_config_still_carries_the_original_bug():
    # Regression anchor: the shipped default maps the gripper PAST open (the silent bug
    # this whole workstream exists to catch). Commanding rad 0.0 lands at 2048 > open 1697.
    default = config.JOINTS["gripper"]
    assert config.rad_to_ticks_spec(default, 0.0) == 2048
    assert config.rad_to_ticks_spec(default, 0.0) > 1697


def test_resolve_arm_joint_from_commissioned_endpoints():
    joints = config._resolve_joints_from_frontmatter(BOB_FRONTMATTER)
    sp = joints["shoulder_pan"]
    # encoder_sign +1: min_steps<->min_rad, max_steps<->max_rad.
    assert config.rad_to_ticks_spec(sp, sp["min_rad"]) == 1100
    assert config.rad_to_ticks_spec(sp, sp["max_rad"]) == 3000
    expected_tpr = (3000 - 1100) / (sp["max_rad"] - sp["min_rad"])
    assert math.isclose(sp["ticks_per_rad"], expected_tpr, rel_tol=1e-6)


def test_resolve_leaves_joints_at_default_when_no_commissioned_fields():
    # A manifest entry with no min_steps/max_steps must NOT alter the shipped spec.
    joints = config._resolve_joints_from_frontmatter(
        {"physics": {"kinematics": [{"id": "elbow_flex", "servo_id": 3}]}}
    )
    assert joints["elbow_flex"] == config.JOINTS["elbow_flex"]


def test_resolve_from_manifest_file(tmp_path):
    # The path entry point parses ROBOT.md frontmatter (same reader the gateway uses).
    md = tmp_path / "ROBOT.md"
    md.write_text(
        "---\n"
        "metadata:\n"
        "  rrn: RRN-000000000011\n"
        "physics:\n"
        "  solver:\n"
        "    gripper:\n"
        "      joint_id: gripper\n"
        "      open_steps: 1697\n"
        "      close_steps: 1529\n"
        "  kinematics:\n"
        "    - id: shoulder_pan\n"
        "      servo_id: 1\n"
        "      encoder_sign: 1\n"
        "      min_steps: 1100\n"
        "      max_steps: 3000\n"
        "---\n\n# Bob\n\nManifest body.\n"
    )
    joints = config.resolve_joints_from_manifest(md)
    g = joints["gripper"]
    assert config.rad_to_ticks_spec(g, g["min_rad"]) == 1529
    assert config.rad_to_ticks_spec(g, g["max_rad"]) == 1697


def test_resolve_from_missing_manifest_returns_defaults():
    # Resolution must NEVER block actuation: a missing/unparseable manifest => defaults.
    joints = config.resolve_joints_from_manifest("/nonexistent/ROBOT.md")
    assert joints["gripper"] == config.JOINTS["gripper"]


# --- invalid commissioning must skip-to-default, never emit a wrong/collapsed map --------


def test_resolve_skips_inverted_arm_endpoints():
    # min_steps > max_steps with encoder_sign=+1 is an invalid (backward) declaration:
    # keep the shipped default rather than silently emit a reversed map to the servo.
    fm = {
        "physics": {
            "kinematics": [
                {
                    "id": "shoulder_pan",
                    "encoder_sign": 1,
                    "min_steps": 3000,
                    "max_steps": 1000,
                }
            ]
        }
    }
    joints = config._resolve_joints_from_frontmatter(fm)
    assert joints["shoulder_pan"] == config.JOINTS["shoulder_pan"]


def test_resolve_skips_degenerate_equal_endpoints():
    # min_steps == max_steps would collapse all radians to one tick (ticks_per_rad=0): default.
    fm = {
        "physics": {
            "kinematics": [{"id": "elbow_flex", "min_steps": 2000, "max_steps": 2000}]
        }
    }
    joints = config._resolve_joints_from_frontmatter(fm)
    assert joints["elbow_flex"] == config.JOINTS["elbow_flex"]


def test_resolve_skips_inverted_gripper():
    # close_steps >= open_steps is the silent bug: keep default rather than a backward map.
    fm = {"physics": {"solver": {"gripper": {"open_steps": 1529, "close_steps": 1697}}}}
    joints = config._resolve_joints_from_frontmatter(fm)
    assert joints["gripper"] == config.JOINTS["gripper"]


def test_resolve_encoder_sign_negative_reverses_mapping():
    # A reversed servo (encoder_sign=-1): the lower tick bound sits at max_rad, upper at min_rad.
    fm = {
        "physics": {
            "kinematics": [
                {
                    "id": "shoulder_pan",
                    "encoder_sign": -1,
                    "min_steps": 1100,
                    "max_steps": 3000,
                }
            ]
        }
    }
    sp = config._resolve_joints_from_frontmatter(fm)["shoulder_pan"]
    assert sp["ticks_per_rad"] < 0
    assert config.rad_to_ticks_spec(sp, sp["min_rad"]) == 3000
    assert config.rad_to_ticks_spec(sp, sp["max_rad"]) == 1100


def test_resolve_frontmatter_non_numeric_skips_joint_without_raising():
    # The pure helper must NOT raise on non-numeric endpoints — skip the joint, keep default.
    fm = {
        "physics": {
            "kinematics": [
                {"id": "shoulder_pan", "min_steps": "abc", "max_steps": 3000}
            ]
        }
    }
    joints = config._resolve_joints_from_frontmatter(fm)  # must not raise
    assert joints["shoulder_pan"] == config.JOINTS["shoulder_pan"]


def test_resolve_from_manifest_non_numeric_returns_defaults(tmp_path):
    md = tmp_path / "ROBOT.md"
    md.write_text(
        "---\n"
        "physics:\n"
        "  kinematics:\n"
        "    - id: shoulder_pan\n"
        "      min_steps: not_a_number\n"
        "      max_steps: 3000\n"
        "---\n\n# Bob\n"
    )
    joints = config.resolve_joints_from_manifest(md)
    assert joints["shoulder_pan"] == config.JOINTS["shoulder_pan"]
