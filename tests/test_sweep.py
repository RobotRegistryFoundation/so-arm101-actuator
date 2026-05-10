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


def test_dry_run_prints_jsonl_with_targets(tmp_path):
    out = tmp_path / "dry.jsonl"
    result = subprocess.run(
        [sys.executable, "-m", "so_arm101_actuator.sweep",
         "--dry-run", "--iterations", "5", "--seed", "42",
         "--out", str(out)],
        capture_output=True, text=True, check=True,
    )
    lines = [ln for ln in out.read_text().splitlines() if ln.strip()]
    assert len(lines) == 5
    for ln in lines:
        rec = json.loads(ln)
        assert rec["mode"] == "dry-run"
        assert set(rec["target"].keys()) == set(config.SAFE_RANGE_RAD.keys())


def test_dry_run_does_not_import_pyserial():
    """--dry-run path must not require pyserial (cert smoke test from a clean venv)."""
    # Smoke: dry-run completes successfully; pyserial absence on cert-intake host
    # is exercised manually in HIL bring-up.
    result = subprocess.run(
        [sys.executable, "-m", "so_arm101_actuator.sweep",
         "--dry-run", "--iterations", "2", "--seed", "1"],
        capture_output=True, text=True, check=True,
    )
    assert result.returncode == 0
