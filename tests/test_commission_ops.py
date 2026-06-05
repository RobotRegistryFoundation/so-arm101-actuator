"""TDD (B1): commissioning actuator ops — set_torque, raw_tick_move, commission_probe,
paced_move — all hardware-free via a mock SCSProtocol.

SAFETY-CRITICAL invariants under test:
  * commission_probe ABORTS on a position-plateau stall and never grinds a joint into a
    hard mechanical stop (declared endpoints may be wrong — that's the premise).
  * paced_move ISSUES-AND-PACES a fixed number of micro-steps; it must NEVER
    poll-to-tolerance (Bob's joints sag and never reach tolerance — that's why slow_move
    exists).
"""

from __future__ import annotations

from pathlib import Path
from unittest.mock import MagicMock

import pytest

from so_arm101_actuator.actuator import SOArm101Actuator
from so_arm101_actuator.errors import OutOfRangeError

_MP = Path(
    "/tmp/none.md"
)  # nonexistent: resolution must fall back to defaults, not crash


def _perfect_actuator(start: dict[int, int] | None = None):
    """Mock SCSProtocol modeling a perfect servo: present == last commanded tick."""
    proto = MagicMock()
    state = dict(start or {})

    def fake_set_position(motor_id, ticks):
        state[motor_id] = ticks

    def fake_read_position(motor_id):
        return state.get(motor_id, 2048)

    proto.set_position.side_effect = fake_set_position
    proto.read_position.side_effect = fake_read_position
    return SOArm101Actuator(protocol=proto), proto


def _stalling_actuator(start: dict[int, int], phys_limit: dict[int, int]):
    """Mock servo that CANNOT advance past phys_limit[motor_id] (increasing direction) —
    models a joint meeting a mechanical stop: present plateaus regardless of goal."""
    proto = MagicMock()
    state = dict(start)

    def fake_set_position(motor_id, ticks):
        lim = phys_limit.get(motor_id)
        state[motor_id] = lim if (lim is not None and ticks > lim) else ticks

    def fake_read_position(motor_id):
        return state.get(motor_id, 2048)

    proto.set_position.side_effect = fake_set_position
    proto.read_position.side_effect = fake_read_position
    return SOArm101Actuator(protocol=proto), proto


def _frozen_actuator(start: dict[int, int]):
    """Mock servo that never moves (unpowered / no response): present stays at start."""
    proto = MagicMock()
    state = dict(start)
    proto.set_position.side_effect = lambda motor_id, ticks: None
    proto.read_position.side_effect = lambda motor_id: state.get(motor_id, 2048)
    return SOArm101Actuator(protocol=proto), proto


def _commanded(proto) -> list[int]:
    return [c.kwargs["ticks"] for c in proto.set_position.call_args_list]


# --- set_torque -----------------------------------------------------------


def test_set_torque_all_disables_every_joint():
    a, proto = _perfect_actuator()
    r = a.set_torque(motor_ids="all", enable=False)
    assert proto.set_torque.call_count == 6
    assert all(c.kwargs["enable"] is False for c in proto.set_torque.call_args_list)
    assert set(r["motor_ids"]) == {1, 2, 3, 4, 5, 6}
    assert r["enabled"] is False


def test_set_torque_specific_motor():
    a, proto = _perfect_actuator()
    a.set_torque(motor_ids=[6], enable=True)
    proto.set_torque.assert_called_once_with(motor_id=6, enable=True)


# --- raw_tick_move (ports grip_to) ---------------------------------------


def test_raw_tick_move_ramps_to_target_and_reports_present():
    a, proto = _perfect_actuator(start={6: 1697})
    r = a.raw_tick_move(motor_id=6, ticks=1529, ramp=6, settle_s=0.0, step_pause_s=0.0)
    assert r["present_tick"] == 1529
    cmds = _commanded(proto)
    assert len(cmds) == 6  # ramped in 6 micro-steps (gentle, no slam)
    assert cmds[-1] == 1529  # final micro-step reaches the goal
    assert cmds[0] != 1529  # not a single jump


# --- commission_probe (SAFETY) -------------------------------------------


def test_commission_probe_reached_is_pass():
    a, proto = _perfect_actuator(start={3: 2048})
    r = a.commission_probe(
        joint_id="elbow_flex",
        motor_id=3,
        direction=1,
        target_ticks=2200,
        step=8,
        settle_s=0.0,
        step_pause_s=0.0,
    )
    assert r["moved"] is True
    assert r["reached"] is True
    assert r["aborted_on_stall"] is False
    assert abs(r["present_tick"] - 2200) <= 12


def test_commission_probe_stall_aborts_without_grinding():
    # Declared target 2400 is wrong; the joint physically stops at 2120.
    a, proto = _stalling_actuator(start={3: 2048}, phys_limit={3: 2120})
    r = a.commission_probe(
        joint_id="elbow_flex",
        motor_id=3,
        direction=1,
        target_ticks=2400,
        step=8,
        stall_steps=3,
        settle_s=0.0,
        step_pause_s=0.0,
    )
    assert r["aborted_on_stall"] is True
    assert r["reached"] is False
    assert r["moved"] is True
    assert abs(r["present_tick"] - 2120) <= 12
    # SAFETY: it stopped probing right after the stall — never commanded near 2400.
    assert max(_commanded(proto)) < 2200


def test_commission_probe_stall_aborts_decreasing_direction():
    # Symmetric to the increasing case: probing DOWNWARD (direction=-1) must also abort on a
    # plateau and never grind — commissioning probes BOTH ways, so pin the -1 path off-hardware.
    proto = MagicMock()
    state = {3: 2048}
    floor = 1980  # joint physically stops here; declared target 1700 is unreachable

    def fake_set_position(motor_id, ticks):
        state[motor_id] = floor if ticks < floor else ticks

    def fake_read_position(motor_id):
        return state.get(motor_id, 2048)

    proto.set_position.side_effect = fake_set_position
    proto.read_position.side_effect = fake_read_position
    a = SOArm101Actuator(protocol=proto)

    r = a.commission_probe(
        joint_id="elbow_flex",
        motor_id=3,
        direction=-1,
        target_ticks=1700,
        step=8,
        stall_steps=3,
        settle_s=0.0,
        step_pause_s=0.0,
    )
    assert r["aborted_on_stall"] is True
    assert r["reached"] is False
    assert r["moved"] is True
    assert abs(r["present_tick"] - floor) <= 12
    # SAFETY: stopped probing near the floor — never commanded DOWN near the bad target 1700.
    assert min(_commanded(proto)) > 1900


def test_commission_probe_no_move_is_fail():
    # Commanded but nothing moved => FAIL (moved=False), distinct from a real stall.
    a, proto = _frozen_actuator(start={3: 2048})
    r = a.commission_probe(
        joint_id="elbow_flex",
        motor_id=3,
        direction=1,
        target_ticks=2400,
        step=8,
        stall_steps=3,
        settle_s=0.0,
        step_pause_s=0.0,
    )
    assert r["moved"] is False
    assert r["reached"] is False
    assert r["aborted_on_stall"] is True


# --- paced_move (issue-and-pace, NOT poll-to-tolerance) -------------------


def test_paced_move_issues_fixed_microsteps_no_tolerance_poll():
    a, proto = _perfect_actuator(start={1: 2048, 2: 2048})
    r = a.paced_move(
        joint_positions={"shoulder_pan": 0.1, "shoulder_lift": 0.1},
        steps=5,
        pace_s=0.0,
        step_timeout_s=0.0,
    )
    # Deterministic: 5 micro-steps x 2 joints. A poll-to-tolerance loop would vary with
    # how fast (if ever) sagging joints reach tolerance.
    assert proto.set_position.call_count == 5 * 2
    assert abs(r["final_positions"]["shoulder_pan"] - 0.1) < 0.02
    assert abs(r["final_positions"]["shoulder_lift"] - 0.1) < 0.02


def test_paced_move_rejects_out_of_range():
    a, proto = _perfect_actuator(start={1: 2048})
    with pytest.raises(OutOfRangeError):
        a.paced_move(
            joint_positions={"shoulder_pan": 99.0},
            steps=3,
            pace_s=0.0,
            step_timeout_s=0.0,
        )


def test_paced_move_does_not_poll_to_tolerance_on_a_frozen_joint():
    # A per-step tolerance-poll would hang/spin on a joint that never reaches target — the
    # exact failure paced_move exists to avoid. Issue-and-pace reads ONLY start + final
    # (2 reads for 1 joint), never once-or-more per micro-step.
    a, proto = _frozen_actuator(start={1: 2048})
    r = a.paced_move(
        joint_positions={"shoulder_pan": 0.1}, steps=5, pace_s=0.0, step_timeout_s=0.0
    )
    assert "final_positions" in r  # completed (did not hang)
    assert (
        proto.read_position.call_count == 2
    )  # start + final; a poll loop would be >> 2


# --- execute() dispatch + outcome_kind -----------------------------------


def test_execute_dispatches_set_torque_as_commissioned():
    a, _ = _perfect_actuator()
    out = a.execute(
        envelope={
            "tool_name": "set_torque",
            "tool_args": {"motor_ids": "all", "enable": False},
        },
        manifest_path=_MP,
        tier="commission",
        config={},
    )
    assert out.success is True
    assert out.outcome_kind == "commissioned"


def test_execute_dispatches_commission_probe_as_commissioned():
    a, _ = _perfect_actuator(start={3: 2048})
    out = a.execute(
        envelope={
            "tool_name": "commission_probe",
            "tool_args": {
                "joint_id": "elbow_flex",
                "motor_id": 3,
                "direction": 1,
                "target_ticks": 2100,
                "step": 8,
                "settle_s": 0.0,
                "step_pause_s": 0.0,
            },
        },
        manifest_path=_MP,
        tier="commission",
        config={},
    )
    assert out.success is True
    assert out.outcome_kind == "commissioned"
    assert out.telemetry["reached"] is True


def test_execute_dispatches_raw_tick_move_as_commissioned():
    a, _ = _perfect_actuator(start={6: 1697})
    out = a.execute(
        envelope={
            "tool_name": "raw_tick_move",
            "tool_args": {
                "motor_id": 6,
                "ticks": 1529,
                "ramp": 4,
                "settle_s": 0.0,
                "step_pause_s": 0.0,
            },
        },
        manifest_path=_MP,
        tier="commission",
        config={},
    )
    assert out.success is True
    assert out.outcome_kind == "commissioned"
    assert out.telemetry["present_tick"] == 1529


def test_execute_dispatches_paced_move_as_executed():
    a, _ = _perfect_actuator(start={1: 2048})
    out = a.execute(
        envelope={
            "tool_name": "paced_move",
            "tool_args": {
                "joint_positions": {"shoulder_pan": 0.05},
                "steps": 4,
                "pace_s": 0.0,
                "step_timeout_s": 0.0,
            },
        },
        manifest_path=_MP,
        tier="op",
        config={},
    )
    assert out.success is True
    assert out.outcome_kind == "executed"


def test_new_ops_declared_in_capabilities():
    a, _ = _perfect_actuator()
    for op in ("set_torque", "raw_tick_move", "commission_probe", "paced_move"):
        assert op in a.capabilities
