"""Tests for operator-overridable HOME_POSE_RAD + SAFE_RANGE_RAD env vars."""

import json
import os

import pytest

from so_arm101_actuator import config


def test_resolve_home_pose_rad_no_override(monkeypatch):
    monkeypatch.delenv("SO_ARM101_HOME_POSE_RAD", raising=False)
    pose = config.resolve_home_pose_rad()
    assert pose == config.HOME_POSE_RAD
    assert pose["shoulder_lift"] == pytest.approx(0.10)


def test_resolve_home_pose_rad_partial_env_merges_with_defaults(monkeypatch):
    monkeypatch.setenv("SO_ARM101_HOME_POSE_RAD", json.dumps({"shoulder_pan": 0.05}))
    pose = config.resolve_home_pose_rad()
    assert pose["shoulder_pan"] == pytest.approx(0.05)
    assert pose["shoulder_lift"] == pytest.approx(0.10)  # default preserved


def test_resolve_home_pose_rad_invalid_json_raises(monkeypatch):
    monkeypatch.setenv("SO_ARM101_HOME_POSE_RAD", "{not-json}")
    with pytest.raises(ValueError, match="SO_ARM101_HOME_POSE_RAD"):
        config.resolve_home_pose_rad()


def test_resolve_home_pose_rad_unknown_joint_raises(monkeypatch):
    monkeypatch.setenv("SO_ARM101_HOME_POSE_RAD", json.dumps({"bogus_joint": 0.0}))
    with pytest.raises(ValueError, match="bogus_joint"):
        config.resolve_home_pose_rad()


def test_safe_range_default_inside_mechanical_limits():
    safe = config.SAFE_RANGE_RAD
    for joint, (lo, hi) in safe.items():
        spec = config.JOINTS[joint]
        assert spec["min_rad"] <= lo, f"{joint}: safe min {lo} below mech min {spec['min_rad']}"
        assert hi <= spec["max_rad"], f"{joint}: safe max {hi} above mech max {spec['max_rad']}"


def test_resolve_safe_range_rad_no_override(monkeypatch):
    monkeypatch.delenv("SO_ARM101_SAFE_RANGE_RAD", raising=False)
    safe = config.resolve_safe_range_rad()
    assert safe == config.SAFE_RANGE_RAD


def test_resolve_safe_range_rad_partial_override(monkeypatch):
    import json
    monkeypatch.setenv(
        "SO_ARM101_SAFE_RANGE_RAD",
        json.dumps({"shoulder_pan": [-0.5, 0.5]}),
    )
    safe = config.resolve_safe_range_rad()
    assert safe["shoulder_pan"] == (-0.5, 0.5)
    assert safe["wrist_roll"] == config.SAFE_RANGE_RAD["wrist_roll"]


def test_resolve_safe_range_rad_invalid_min_ge_max_raises(monkeypatch):
    import json
    monkeypatch.setenv("SO_ARM101_SAFE_RANGE_RAD", json.dumps({"shoulder_pan": [0.5, 0.5]}))
    with pytest.raises(ValueError, match="shoulder_pan"):
        config.resolve_safe_range_rad()


def test_resolve_safe_range_rad_outside_mechanical_raises(monkeypatch):
    import json
    monkeypatch.setenv("SO_ARM101_SAFE_RANGE_RAD", json.dumps({"shoulder_pan": [-99.0, 99.0]}))
    with pytest.raises(ValueError, match="mechanical"):
        config.resolve_safe_range_rad()
