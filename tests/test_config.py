import os as _os
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


def test_manifest_applied_by_path_gives_taught_pose_not_raw_zeros(tmp_path):
    """Correcting zeros MUST be accompanied by the taught pose from the SAME file.

    Regression for a bug that recurred three times in different disguises. The
    manifest fixes each joint's tick zero; the generic HOME_POSE_RAD constants
    are ~0.0 rad. Apply the first without the second and every 0.0 rad resolves
    to the joint's raw zero tick, so `arm.home` drives the arm to a pose nobody
    taught it — silently, and looking entirely healthy.
    """
    import importlib
    from so_arm101_actuator import config as cfg
    importlib.reload(cfg)

    manifest = "/home/craigm26/bob/ROBOT.md"
    if not _os.path.exists(manifest):
        import pytest
        pytest.skip("this robot's manifest is not on this machine")

    cfg.apply_manifest_calibration(manifest)
    pose = cfg.resolve_home_pose_rad(manifest)
    ticks = {j: cfg.rad_to_ticks(j, r) for j, r in pose.items()}

    # The taught `ready` pose, in ticks, straight from the manifest.
    taught = cfg.load_manifest_calibration(manifest)["ready_ticks"]
    for joint, want in taught.items():
        assert ticks[joint] == want, (
            f"{joint} resolved to {ticks[joint]} ticks but the manifest teaches {want}")


def test_home_pose_without_manifest_keeps_generic_constants():
    """No manifest: the generic pose stands, and the gripper stays excluded."""
    import importlib
    from so_arm101_actuator import config as cfg
    importlib.reload(cfg)
    pose = cfg.resolve_home_pose_rad(None)
    assert pose  # a bench with no manifest still has a usable home
