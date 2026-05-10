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


def test_read_state_returns_all_joints():
    actuator, proto = _make_actuator(present_positions={i: 2048 for i in range(1, 7)})
    state = actuator.read_state()
    assert set(state["positions"].keys()) == {
        "shoulder_pan", "shoulder_lift", "elbow_flex",
        "wrist_flex", "wrist_roll", "gripper",
    }
    assert all(abs(v) < 0.01 for v in state["positions"].values())  # all near zero


def test_read_state_includes_temperatures():
    actuator, proto = _make_actuator(present_positions={i: 2048 for i in range(1, 7)})
    state = actuator.read_state()
    assert all(t == 30 for t in state["motor_temps_c"].values())


def test_read_state_skips_motors_that_fail_temperature_read():
    proto = MagicMock()
    proto.read_position.return_value = 2048
    proto.read_temperature.side_effect = [30, 30, 30, 30, 30, IOError("no sensor")]
    actuator = SOArm101Actuator(protocol=proto)
    state = actuator.read_state()
    # 5 motors report temperature; gripper missing
    assert len(state["motor_temps_c"]) == 5
    assert "gripper" not in state["motor_temps_c"]


def test_read_state_has_timestamp():
    actuator, proto = _make_actuator(present_positions={i: 2048 for i in range(1, 7)})
    state = actuator.read_state()
    assert state["timestamp_s"] > 0
