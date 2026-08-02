"""Unit tests for SOArm101Actuator (mocked protocol; no hardware)."""

from __future__ import annotations

import time

from unittest.mock import MagicMock

import pytest

from so_arm101_actuator.actuator import SOArm101Actuator
from so_arm101_actuator.errors import OutOfRangeError, UnknownJointError


def _make_actuator(present_positions: dict[int, int] | None = None) -> tuple[SOArm101Actuator, MagicMock]:
    """Build an actuator with a MagicMock SCSProtocol.

    `present_positions` seeds initial motor_id → ticks readings. Each
    set_position(motor_id, ticks) call updates the simulated state, so a
    subsequent read_position(motor_id) returns the most-recently-commanded
    value — modeling a perfect servo and keeping the mock robust to
    HOME_POSE_RAD changes.
    """
    proto = MagicMock()
    state: dict[int, int] = dict(present_positions or {})

    def fake_set_position(motor_id: int, ticks: int) -> None:
        state[motor_id] = ticks

    def fake_read_position(motor_id: int) -> int:
        return state.get(motor_id, 2048)

    proto.set_position.side_effect = fake_set_position
    proto.read_position.side_effect = fake_read_position
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


# ---------------------------------------------------------------------------
# Task 13 — Actuator Protocol bridge (gateway integration)
# ---------------------------------------------------------------------------

def test_zero_arg_instantiation():
    """Gateway entry-point calls SOArm101Actuator() with no args — must not raise."""
    SOArm101Actuator()  # regression guard: gateway calls actuator_cls() at startup


def test_capabilities_tuple_unchanged():
    actuator, _ = _make_actuator()
    assert actuator.capabilities == ("move", "home", "read_state")


def test_implements_actuator_protocol():
    from robot_md_gateway.actuator import Actuator
    actuator, _ = _make_actuator()
    assert isinstance(actuator, Actuator)


def test_metadata_attributes():
    actuator, _ = _make_actuator()
    assert actuator.name == "so-arm101"
    assert actuator.description
    assert isinstance(actuator.config_schema, dict)


def test_execute_dispatches_move():
    from pathlib import Path
    actuator, proto = _make_actuator(present_positions={1: 2048})
    outcome = actuator.execute(
        envelope={"tool_name": "move", "tool_args": {"joint_positions": {"shoulder_pan": 0.0}, "timeout_s": 0.1}},
        manifest_path=Path("/tmp/dummy.md"),
        tier="actuate",
        config={},
    )
    assert outcome.success is True
    assert outcome.outcome_kind == "executed"
    assert outcome.telemetry["reached"] is True


def test_execute_dispatches_home():
    from pathlib import Path
    actuator, proto = _make_actuator(present_positions={i: 2048 for i in range(1, 7)})
    outcome = actuator.execute(
        envelope={"tool_name": "home", "tool_args": {"timeout_s": 0.1}},
        manifest_path=Path("/tmp/dummy.md"),
        tier="actuate",
        config={},
    )
    assert outcome.success is True
    assert outcome.outcome_kind == "executed"


def test_execute_dispatches_read_state():
    from pathlib import Path
    actuator, _ = _make_actuator(present_positions={i: 2048 for i in range(1, 7)})
    outcome = actuator.execute(
        envelope={"tool_name": "read_state", "tool_args": {}},
        manifest_path=Path("/tmp/dummy.md"),
        tier="read",
        config={},
    )
    assert outcome.success is True
    assert outcome.outcome_kind == "executed"
    assert "positions" in outcome.telemetry


def test_execute_unknown_capability_returns_error_outcome():
    from pathlib import Path
    actuator, _ = _make_actuator()
    outcome = actuator.execute(
        envelope={"tool_name": "teleport", "tool_args": {}},
        manifest_path=Path("/tmp/dummy.md"),
        tier="actuate",
        config={},
    )
    assert outcome.success is False
    assert outcome.outcome_kind == "error"
    assert "unknown capability" in outcome.error_message


def test_execute_actuator_exception_becomes_error_outcome():
    from pathlib import Path
    actuator, _ = _make_actuator()
    outcome = actuator.execute(
        envelope={"tool_name": "move", "tool_args": {"joint_positions": {"not_a_joint": 0.0}}},
        manifest_path=Path("/tmp/dummy.md"),
        tier="actuate",
        config={},
    )
    assert outcome.success is False
    assert outcome.outcome_kind == "error"
    assert "UnknownJointError" in outcome.error_message


def test_reached_agrees_with_the_positions_in_the_same_result():
    """A receipt must not assert failure while reporting arrival.

    The poll loop and the final snapshot are two separate reads. An arm that
    settles between them used to produce reached=False alongside
    final_positions showing every joint on target — one receipt contradicting
    itself. Seen on hardware: a wrist_flex move reported failure and the joint
    was later found 0.03 rad from target, well inside the 0.05 tolerance.

    These receipts are signed evidence and a saved capability replays them, so a
    macro that worked would look broken on every single run.
    """
    from pathlib import Path

    class ArrivesAfterTheLoopGivesUp:
        """Reports the old position while polled, the new one when finally read.

        Switches on ELAPSED TIME rather than a read count, so it arrives in the
        gap between the loop timing out and the closing snapshot regardless of
        how many times the loop happens to poll.
        """

        def __init__(self, target_ticks: int, arrive_after_s: float):
            self._target = target_ticks
            self._arrive_at = time.monotonic() + arrive_after_s

        def set_position(self, motor_id, ticks):
            pass

        def read_position(self, motor_id):
            return self._target if time.monotonic() >= self._arrive_at else 2048

        def read_temperature(self, motor_id):
            return 30

    from so_arm101_actuator import config as cfg
    target_rad = 0.30
    target_ticks = cfg.rad_to_ticks("wrist_flex", target_rad)
    # Arrive just after the 0.25s loop gives up, i.e. inside the gap.
    actuator = SOArm101Actuator(
        protocol=ArrivesAfterTheLoopGivesUp(target_ticks, arrive_after_s=0.26))

    result = actuator.move({"wrist_flex": target_rad}, timeout_s=0.25)

    on_target = abs(result["final_positions"]["wrist_flex"] - target_rad)
    assert on_target <= actuator.move_tolerance_rad, "test fixture did not actually arrive"
    assert result["reached"] is True, (
        "receipt says the move failed while its own final_positions show arrival")
    assert result["max_error_rad"] <= actuator.move_tolerance_rad


def test_a_move_that_genuinely_fails_still_reports_false():
    """The fix must not turn every move into a success."""
    class NeverMoves:
        def set_position(self, motor_id, ticks):
            pass

        def read_position(self, motor_id):
            return 2048

        def read_temperature(self, motor_id):
            return 30

    actuator = SOArm101Actuator(protocol=NeverMoves())
    result = actuator.move({"wrist_flex": 0.60}, timeout_s=0.2)
    assert result["reached"] is False
    assert result["max_error_rad"] > actuator.move_tolerance_rad
