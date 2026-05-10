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
