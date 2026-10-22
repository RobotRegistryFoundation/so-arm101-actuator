"""SOArm101Transport — the castor-hal adapter (mocked protocol; no hardware).

castor-hal is an optional extra, so this module is skipped without it. The
adapter reads and stops the arm and moves nothing: its set_goal used to drive
the raw move(), unchecked and unpaced (EV-03), so every goal is now refused
before the bus is touched. Its stop is the wrapped actuator's own latch."""

from __future__ import annotations

from pathlib import Path
from unittest.mock import MagicMock

import pytest

pytest.importorskip("castor_hal", reason="castor-hal is the optional `hal` extra")

from castor_hal.actuator import TransportActuator  # noqa: E402
from castor_hal.errors import TransportError, TransportErrorCode  # noqa: E402
from castor_hal.goal import BodyTwist, Home, JointPositions  # noqa: E402

from so_arm101_actuator.transport import SOArm101Transport, make_hal_actuator  # noqa: E402

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


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


def test_capabilities_are_empty():
    t, _ = _make_transport()
    assert t.capabilities == frozenset()
    for goal in (JointPositions({"shoulder_pan": 0.0}), Home(), BodyTwist()):
        assert not t.supports(goal)


@pytest.mark.parametrize("goal", [
    JointPositions({"shoulder_pan": 0.0}),
    Home(),
    JointPositions({"shoulder_pan": 99.0}),
    JointPositions({"nonexistent": 0.0}),
    BodyTwist(linear=(1.0, 0.0, 0.0)),
], ids=["joint_positions", "home", "out_of_range", "unknown_joint", "body_twist"])
def test_set_goal_refuses_every_goal_before_touching_the_bus(goal):
    """Valid or not, every goal gets the same refusal, and the protocol is never
    called: not a write, not a read."""
    t, proto = _make_transport()
    with pytest.raises(TransportError) as ei:
        t.set_goal(goal)
    assert ei.value.code is TransportErrorCode.UNSUPPORTED_GOAL
    assert ei.value.detail["capabilities"] == []
    assert ei.value.detail["goal_kind"] == goal.kind.value
    assert proto.method_calls == []


def test_read_state_returns_positions_and_temps():
    t, _ = _make_transport()
    st = t.read_state()
    assert set(st.positions) == set(JOINTS)
    assert st.temperatures_c["shoulder_pan"] == 30
    assert st.connected is True
    assert st.estopped is False


def test_estop_holds_and_latches():
    t, proto = _make_transport()
    t.estop()
    assert t._estopped is True
    assert t.read_state().estopped is True
    # held every joint: re-command what it holds (6 joints)
    assert proto.set_position.call_count == len(JOINTS)
    t.clear_estop()
    assert t._estopped is False
    assert t.read_state().estopped is False


def test_the_stop_is_the_wrapped_actuators_own_latch():
    """One latch, never two copies that can disagree: a stop through the adapter
    refuses the gateway path's motion, and a stop through the gateway path shows
    here."""
    t, _ = _make_transport()
    actuator = t._actuator

    t.estop()
    refused = actuator.execute(envelope={"tool_name": "arm.home", "tool_args": {}},
                               manifest_path="", tier="actuate", config={})
    assert refused.outcome_kind == "denied"
    assert refused.telemetry["deny"] == "estop_latched"

    t.clear_estop()
    assert not actuator._latched()
    stopped = actuator.execute(envelope={"tool_name": "arm.estop", "tool_args": {}},
                               manifest_path="", tier="read", config={})
    assert stopped.success is True
    assert t._estopped is True


# -- end-to-end through the gateway bridge ---------------------------------
@pytest.mark.parametrize("tool_name,tool_args", [
    ("move", {"joint_positions": {"shoulder_pan": 0.0}}),
    ("home", {}),
])
def test_through_transport_actuator_motion_is_refused(tool_name, tool_args):
    t, proto = _make_transport()
    a = TransportActuator(t)
    out = a.execute(
        envelope={"tool_name": tool_name, "tool_args": tool_args},
        manifest_path=Path("ROBOT.md"),
        tier="actuate",
        config={},
    )
    assert out.success is False
    assert out.outcome_kind == "error"
    assert out.telemetry["error_code"] == "unsupported_goal"
    proto.set_position.assert_not_called()


def test_through_transport_actuator_estop_holds_and_latches():
    t, proto = _make_transport()
    a = TransportActuator(t)
    out = a.execute(
        envelope={"tool_name": "estop", "tool_args": {}},
        manifest_path=Path("ROBOT.md"),
        tier="read",
        config={},
    )
    assert out.success is True
    assert out.telemetry["estopped"] is True
    assert t._estopped is True
    assert proto.set_position.call_count == len(JOINTS)


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
