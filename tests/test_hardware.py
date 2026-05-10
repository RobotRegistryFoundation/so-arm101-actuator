"""Layer 3 — real hardware tests. Skipped unless SO_ARM101_HARDWARE=1.

Run on Bob:
    SO_ARM101_HARDWARE=1 .venv/bin/pytest tests/test_hardware.py -v
"""

from __future__ import annotations

import os

import pytest

from so_arm101_actuator import SOArm101Actuator
from so_arm101_actuator import config

pytestmark = [
    pytest.mark.hardware,
    pytest.mark.skipif(
        os.environ.get("SO_ARM101_HARDWARE") != "1",
        reason="hardware tests fenced; set SO_ARM101_HARDWARE=1 to run",
    ),
]


@pytest.fixture(scope="module")
def actuator() -> SOArm101Actuator:
    return SOArm101Actuator.from_default_port()


def test_read_state_returns_plausible_positions(actuator):
    state = actuator.read_state()
    for joint, rad in state["positions"].items():
        spec = config.JOINTS[joint]
        assert spec["min_rad"] - 0.1 <= rad <= spec["max_rad"] + 0.1, f"{joint}={rad}"


def test_home_completes(actuator):
    result = actuator.home(timeout_s=15.0)
    assert result["reached"] is True
    for joint, rad in result["final_positions"].items():
        assert abs(rad - config.HOME_POSE_RAD[joint]) <= config.MOVE_TOLERANCE_RAD


def test_move_shoulder_pan(actuator):
    actuator.home(timeout_s=15.0)
    target = 0.1
    result = actuator.move({"shoulder_pan": target}, timeout_s=5.0)
    assert result["reached"] is True
    assert abs(result["final_positions"]["shoulder_pan"] - target) <= config.MOVE_TOLERANCE_RAD
    actuator.home(timeout_s=15.0)
