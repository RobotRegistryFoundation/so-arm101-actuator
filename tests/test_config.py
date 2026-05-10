"""Tests for joint configuration + rad↔tick conversion."""

import math

import pytest

from so_arm101_actuator import config


def test_six_joints_defined():
    assert set(config.JOINTS.keys()) == {
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
        "gripper",
    }


def test_each_joint_has_required_fields():
    for name, joint in config.JOINTS.items():
        assert "motor_id" in joint
        assert "tick_at_zero_rad" in joint
        assert "ticks_per_rad" in joint
        assert "min_rad" in joint
        assert "max_rad" in joint


def test_motor_ids_are_unique():
    ids = [j["motor_id"] for j in config.JOINTS.values()]
    assert len(set(ids)) == len(ids)


def test_rad_to_ticks_zero_position():
    assert config.rad_to_ticks("shoulder_pan", 0.0) == config.JOINTS["shoulder_pan"]["tick_at_zero_rad"]


def test_rad_to_ticks_full_circle_roundtrip():
    target_rad = 0.5
    ticks = config.rad_to_ticks("shoulder_pan", target_rad)
    back = config.ticks_to_rad("shoulder_pan", ticks)
    assert math.isclose(back, target_rad, abs_tol=1e-3)


def test_rad_to_ticks_out_of_range_raises():
    from so_arm101_actuator.errors import OutOfRangeError
    with pytest.raises(OutOfRangeError):
        config.rad_to_ticks("shoulder_pan", 99.0)


def test_home_pose_defined_for_all_joints():
    assert set(config.HOME_POSE_RAD.keys()) == set(config.JOINTS.keys())


def test_move_tolerance_is_small():
    assert 0 < config.MOVE_TOLERANCE_RAD < 0.1
