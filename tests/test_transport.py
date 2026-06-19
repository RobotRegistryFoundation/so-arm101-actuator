"""SOArm101Transport — the castor-hal adapter (mocked protocol; no hardware).

Reuses the perfect-servo MagicMock pattern from test_actuator.py. Proves the
SO-ARM101 satisfies the universal Transport seam + drives end-to-end through the
gateway bridge (TransportActuator)."""

from __future__ import annotations

from pathlib import Path
from unittest.mock import MagicMock

import pytest

from castor_hal.actuator import TransportActuator
from castor_hal.errors import TransportError, TransportErrorCode
from castor_hal.goal import BodyTwist, GoalKind, Home, JointPositions

from so_arm101_actuator.transport import SOArm101Transport, make_hal_actuator


def _make_transport():
    """SOArm101Transport over a perfect-servo MagicMock SCSProtocol."""
    proto = MagicMock()
    state: dict[int, int] = {}

    def set_position(motor_id: int, ticks: int) -> None:
        state[motor_id] = ticks

    def read_position(motor_id: int) -> int:
        return state.get(motor_id, 2048)

    proto.set_position.side_effect = set_position
    proto.read_position.side_effect = read_position
    proto.read_temperature.return_value = 30
    return SOArm101Transport(protocol=proto), proto


def test_capabilities():
    t, _ = _make_transport()
    assert GoalKind.JOINT_POSITIONS in t.capabilities
    assert GoalKind.HOME in t.capabilities
    assert not t.supports(BodyTwist())


def test_set_goal_joint_positions_reaches():
    t, proto = _make_transport()
    res = t.set_goal(JointPositions({"shoulder_pan": 0.0}))
    assert res.reached is True
    proto.set_position.assert_called_with(motor_id=1, ticks=2048)
    assert res.state.positions["shoulder_pan"] == pytest.approx(0.0, abs=1e-6)
    assert "elapsed_s" in res.detail


def test_set_goal_home_moves_to_home_pose():
    t, _ = _make_transport()
    res = t.set_goal(Home())
    assert res.reached is True
    # home pose includes the calibrated shoulder_lift offset (0.10 rad)
    assert res.state.positions["shoulder_lift"] == pytest.approx(0.10, abs=0.03)


def test_out_of_range_is_structured_error():
    t, _ = _make_transport()
    with pytest.raises(TransportError) as ei:
        t.set_goal(JointPositions({"shoulder_pan": 99.0}))
    assert ei.value.code is TransportErrorCode.OUT_OF_RANGE


def test_unknown_joint_is_structured_error():
    t, _ = _make_transport()
    with pytest.raises(TransportError) as ei:
        t.set_goal(JointPositions({"nonexistent": 0.0}))
    assert ei.value.code is TransportErrorCode.UNKNOWN_TARGET


def test_unsupported_goal_is_structured_error():
    t, _ = _make_transport()
    with pytest.raises(TransportError) as ei:
        t.set_goal(BodyTwist(linear=(1.0, 0.0, 0.0)))
    assert ei.value.code is TransportErrorCode.UNSUPPORTED_GOAL


def test_read_state_returns_positions_and_temps():
    t, _ = _make_transport()
    st = t.read_state()
    assert set(st.positions) == set(
        ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
    )
    assert st.temperatures_c["shoulder_pan"] == 30
    assert st.connected is True


def test_estop_holds_and_latches():
    t, proto = _make_transport()
    t.estop()
    assert t._estopped is True
    # held every joint: read then re-command its current ticks (6 joints)
    assert proto.set_position.call_count == len(
        ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
    )
    # latched: motion is refused until cleared
    with pytest.raises(TransportError) as ei:
        t.set_goal(JointPositions({"shoulder_pan": 0.0}))
    assert ei.value.code is TransportErrorCode.ESTOPPED
    t.clear_estop()
    assert t.set_goal(JointPositions({"shoulder_pan": 0.0})).reached is True


# -- end-to-end through the gateway bridge ---------------------------------
def test_through_transport_actuator_move():
    t, _ = _make_transport()
    a = TransportActuator(t)
    out = a.execute(
        envelope={"tool_name": "move", "tool_args": {"joint_positions": {"shoulder_pan": 0.0}}},
        manifest_path=Path("ROBOT.md"),
        tier="actuate",
        config={},
    )
    assert out.success is True
    assert out.outcome_kind == "executed"
    assert out.telemetry["goal_kind"] == "joint_positions"


def test_through_transport_actuator_read_state():
    t, _ = _make_transport()
    a = TransportActuator(t)
    out = a.execute(
        envelope={"tool_name": "read_state", "tool_args": {}},
        manifest_path=Path("ROBOT.md"),
        tier="read",
        config={},
    )
    assert out.success is True
    assert "positions" in out.telemetry


def test_make_hal_actuator_is_lazy_and_named():
    a = make_hal_actuator()
    assert isinstance(a, TransportActuator)
    assert a.name == "so-arm101"
