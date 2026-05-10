"""Tests for FULL-SWEEP pose generator + dry-run CLI."""

import json
import subprocess
import sys

import pytest

from so_arm101_actuator import config, sweep


def test_generate_sweep_poses_count():
    poses = sweep.generate_sweep_poses(iterations=10, seed=42)
    assert len(poses) == 10


def test_generate_sweep_poses_six_joints_each():
    poses = sweep.generate_sweep_poses(iterations=3, seed=42)
    expected = set(config.SAFE_RANGE_RAD.keys())
    for pose in poses:
        assert set(pose.keys()) == expected


def test_generate_sweep_poses_within_safe_range():
    poses = sweep.generate_sweep_poses(iterations=100, seed=7)
    safe = config.SAFE_RANGE_RAD
    for pose in poses:
        for joint, val in pose.items():
            lo, hi = safe[joint]
            assert lo <= val <= hi, f"{joint}={val} outside [{lo}, {hi}]"


def test_generate_sweep_poses_deterministic_given_seed():
    a = sweep.generate_sweep_poses(iterations=20, seed=123)
    b = sweep.generate_sweep_poses(iterations=20, seed=123)
    assert a == b


def test_generate_sweep_poses_different_seeds_differ():
    a = sweep.generate_sweep_poses(iterations=20, seed=1)
    b = sweep.generate_sweep_poses(iterations=20, seed=2)
    assert a != b
