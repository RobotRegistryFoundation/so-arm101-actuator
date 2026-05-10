"""Unit tests for SOArm101Actuator (mocked protocol; no hardware)."""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from so_arm101_actuator.actuator import SOArm101Actuator
from so_arm101_actuator.errors import OutOfRangeError, UnknownJointError


def _make_actuator(present_positions: dict[int, int] | None = None) -> tuple[SOArm101Actuator, MagicMock]:
    """Build an actuator with a MagicMock SCSProtocol.

    `present_positions` maps motor_id → ticks for read_position calls.
    """
    proto = MagicMock()
    proto.read_position.side_effect = lambda motor_id: (present_positions or {}).get(motor_id, 2048)
    proto.read_temperature.return_value = 30
    actuator = SOArm101Actuator(protocol=proto)
    return actuator, proto


def test_move_dispatches_one_joint():
    actuator, proto = _make_actuator(present_positions={1: 2048})
    result = actuator.move({"shoulder_pan": 0.0}, timeout_s=0.1)
    proto.set_position.assert_called_once_with(motor_id=1, ticks=2048)
    assert result["reached"] is True


def test_move_unknown_joint_raises():
    actuator, proto = _make_actuator()
    with pytest.raises(UnknownJointError):
        actuator.move({"not_a_joint": 0.0})
    proto.set_position.assert_not_called()


def test_move_out_of_range_raises():
    actuator, proto = _make_actuator()
    with pytest.raises(OutOfRangeError):
        actuator.move({"shoulder_pan": 99.0})
    proto.set_position.assert_not_called()


def test_move_returns_final_positions():
    actuator, proto = _make_actuator(present_positions={1: 2048, 2: 2200})
    result = actuator.move({"shoulder_pan": 0.0, "shoulder_lift": 0.233}, timeout_s=0.1)
    assert "shoulder_pan" in result["final_positions"]
    assert "shoulder_lift" in result["final_positions"]


def test_home_uses_home_pose_rad():
    actuator, proto = _make_actuator(present_positions={i: 2048 for i in range(1, 7)})
    result = actuator.home(timeout_s=0.1)
    # All 6 joints commanded to ticks_at_zero_rad (== 2048 for all in default config)
    assert proto.set_position.call_count == 6
    assert result["reached"] is True
