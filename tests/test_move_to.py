"""arm.move_to and arm.state, end to end through execute().

Every test here runs against a simulated servo bus (a MagicMock protocol that
remembers what it was told). Nothing opens a serial port, and the only motion
that happens is arithmetic.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from unittest.mock import MagicMock

import pytest

from so_arm101_actuator import config, kinematics as kin
from so_arm101_actuator.actuator import (
    DEFAULT_SPEED,
    MAX_WAYPOINTS,
    SOArm101Actuator,
)
from so_arm101_actuator.errors import DeniedError
from tests.conftest import FIXTURE_MANIFEST

M = FIXTURE_MANIFEST
MANIFEST = Path(M)

#: A target the tool-down solve can actually answer on this geometry.
TARGET = {"x_mm": 150.0, "y_mm": 0.0, "z_mm": 50.0}

#: This rig's MEASURED envelope stops the wrist at +0.41 rad, and pointing the
#: tool straight down at tabletop reach needs about +1.47. Widening the three
#: joints is what an operator does after re-measuring their own arm, and it is
#: the only way any tool-down target executes here — see
#: `test_bobs_measured_envelope_refuses_the_whole_tabletop` for the proof that
#: it is not a quirk of one point.
WIDE_RANGES = json.dumps({
    "shoulder_lift": [-1.45, 1.00],
    "elbow_flex": [-0.19, 1.50],
    "wrist_flex": [-0.93, 1.50],
})


def _actuator(*, positions: dict[int, int] | None = None) -> tuple[SOArm101Actuator, MagicMock]:
    """An actuator over a perfect simulated servo: whatever it is told to be,
    it reads back as. Same shape as the mock in test_actuator.py."""
    proto = MagicMock()
    state: dict[int, int] = {i: 2048 for i in range(1, 7)}
    state.update(positions or {})

    proto.set_position.side_effect = lambda motor_id, ticks: state.__setitem__(motor_id, ticks)
    proto.read_position.side_effect = lambda motor_id: state.get(motor_id, 2048)
    proto.read_temperature.return_value = 30
    actuator = SOArm101Actuator(protocol=proto)
    actuator._apply_manifest(M)
    return actuator, proto


def _invoke(actuator: SOArm101Actuator, tool_name: str, tool_args: dict | None = None,
            *, tier: str = "actuate"):
    return actuator.execute(
        envelope={"tool_name": tool_name, "tool_args": tool_args or {}},
        manifest_path=MANIFEST,
        tier=tier,
        config={},
    )


# --------------------------------------------------------------------------- #
# It moves
# --------------------------------------------------------------------------- #

def test_move_to_lands_the_tip_on_the_requested_point(monkeypatch):
    monkeypatch.setenv("SO_ARM101_SAFE_RANGE_RAD", WIDE_RANGES)
    actuator, _ = _actuator()
    outcome = _invoke(actuator, "arm.move_to", TARGET)

    assert outcome.success, outcome.error_message
    assert outcome.outcome_kind == "executed"
    telemetry = outcome.telemetry
    assert telemetry["reached"] is True
    assert telemetry["eef_mm"]["x"] == pytest.approx(150.0, abs=0.5)
    assert telemetry["eef_mm"]["y"] == pytest.approx(0.0, abs=0.5)
    assert telemetry["eef_mm"]["z"] == pytest.approx(50.0, abs=0.5)
    assert telemetry["error_mm"] < 1.0
    assert telemetry["ik_provider"] == kin.IK_PROVIDER_ID


def test_the_telemetry_carries_the_four_keys_a_harness_reads(monkeypatch):
    """The contract an external evaluation harness is written against. Extra
    keys may be added; these four may not move or be renamed."""
    monkeypatch.setenv("SO_ARM101_SAFE_RANGE_RAD", WIDE_RANGES)
    actuator, _ = _actuator()
    telemetry = _invoke(actuator, "arm.move_to", TARGET).telemetry

    assert set(telemetry) >= {"reached", "final_positions", "eef_mm", "elapsed_s"}
    assert isinstance(telemetry["reached"], bool)
    assert set(telemetry["eef_mm"]) == {"x", "y", "z"}
    assert telemetry["elapsed_s"] >= 0.0
    assert set(telemetry["final_positions"]) == {
        "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"}
    for angle in telemetry["final_positions"].values():
        assert isinstance(angle, float)


def test_wrist_roll_is_held_where_it_was_not_driven_somewhere_new(monkeypatch):
    """A Cartesian target says nothing about the roll of the tool. Choosing one
    would spin whatever is in the gripper for no reason at all."""
    monkeypatch.setenv("SO_ARM101_SAFE_RANGE_RAD", WIDE_RANGES)
    # wrist_roll's zero is tick 2188; park it 200 ticks away from that.
    actuator, proto = _actuator(positions={5: 1988})
    before = actuator._read_joint("wrist_roll")

    telemetry = _invoke(actuator, "arm.move_to", TARGET).telemetry

    assert telemetry["final_positions"]["wrist_roll"] == pytest.approx(before)


def test_the_gripper_is_never_commanded_by_a_cartesian_move(monkeypatch):
    """Moving somewhere is not the same as opening or closing on something."""
    monkeypatch.setenv("SO_ARM101_SAFE_RANGE_RAD", WIDE_RANGES)
    actuator, proto = _actuator()
    _invoke(actuator, "arm.move_to", TARGET)

    gripper_id = config.JOINTS["gripper"]["motor_id"]
    commanded = {call.kwargs["motor_id"] for call in proto.set_position.call_args_list}
    assert gripper_id not in commanded


# --------------------------------------------------------------------------- #
# Speed
# --------------------------------------------------------------------------- #

def test_full_speed_is_one_command_per_joint(monkeypatch):
    """1.0 is exactly what arm.home and arm.reach have always done."""
    monkeypatch.setenv("SO_ARM101_SAFE_RANGE_RAD", WIDE_RANGES)
    actuator, proto = _actuator()
    telemetry = _invoke(actuator, "arm.move_to", {**TARGET, "speed": 1.0}).telemetry

    assert telemetry["waypoints"] == 1
    assert proto.set_position.call_count == 5  # one per moved joint, once


def test_a_slower_speed_paces_the_motion_through_waypoints(monkeypatch):
    """The bus takes Goal_Position and nothing else, so "slower" is spelled as
    more, smaller commands along the same straight line."""
    monkeypatch.setenv("SO_ARM101_SAFE_RANGE_RAD", WIDE_RANGES)
    actuator, proto = _actuator()
    telemetry = _invoke(actuator, "arm.move_to", {**TARGET, "speed": 0.25}).telemetry

    assert telemetry["waypoints"] == 4
    assert proto.set_position.call_count == 5 * 4
    # It still arrives: pacing changes the path, not the destination.
    assert telemetry["error_mm"] < 1.0


def test_waypoints_are_capped_so_a_tiny_speed_is_not_a_hang(monkeypatch):
    monkeypatch.setenv("SO_ARM101_SAFE_RANGE_RAD", WIDE_RANGES)
    actuator, _ = _actuator()
    telemetry = _invoke(actuator, "arm.move_to", {**TARGET, "speed": 0.001}).telemetry
    assert telemetry["waypoints"] == MAX_WAYPOINTS


def test_the_default_speed_is_recorded_in_the_receipt(monkeypatch):
    monkeypatch.setenv("SO_ARM101_SAFE_RANGE_RAD", WIDE_RANGES)
    actuator, _ = _actuator()
    telemetry = _invoke(actuator, "arm.move_to", TARGET).telemetry
    assert telemetry["speed"] == DEFAULT_SPEED


@pytest.mark.parametrize("speed", [0, 0.0, -0.5, 1.5, "fast", None, True])
def test_a_speed_outside_the_scale_is_refused_not_clamped(speed):
    actuator, proto = _actuator()
    outcome = _invoke(actuator, "arm.move_to", {**TARGET, "speed": speed})

    assert outcome.outcome_kind == "denied"
    assert outcome.telemetry["deny"] == "bad_args"
    proto.set_position.assert_not_called()


# --------------------------------------------------------------------------- #
# It refuses — and the refusal is a decision, not a fault
# --------------------------------------------------------------------------- #

def test_a_target_outside_the_declared_workspace_is_denied_with_the_axis_named():
    actuator, proto = _actuator()
    outcome = _invoke(actuator, "arm.move_to", {"x_mm": 500.0, "y_mm": 0.0, "z_mm": 50.0})

    assert outcome.success is False
    assert outcome.outcome_kind == "denied"
    assert outcome.telemetry["deny"] == "out_of_workspace"
    assert "x=" in outcome.telemetry["reason"]
    proto.set_position.assert_not_called()


def test_a_target_beyond_the_arms_reach_is_denied():
    """Inside the declared box on every axis, still outside what the links span
    — the two bounds are different facts and both are enforced."""
    actuator, proto = _actuator()
    outcome = _invoke(actuator, "arm.move_to", {"x_mm": 330.0, "y_mm": 330.0, "z_mm": 240.0})

    assert outcome.outcome_kind == "denied"
    assert outcome.telemetry["deny"] == "unreachable"
    proto.set_position.assert_not_called()


def test_a_pose_outside_this_rigs_measured_envelope_is_denied_not_clamped():
    """The default (un-widened) safe range on this geometry. The nearest legal
    pose would put the tip somewhere else entirely, so there is no clamping to
    be done — only a refusal that says which joint and by how much."""
    actuator, proto = _actuator()
    outcome = _invoke(actuator, "arm.move_to", TARGET)

    assert outcome.outcome_kind == "denied"
    assert outcome.telemetry["deny"] == "unsafe_pose"
    # Names the first joint that fails and the range it fell outside, so the
    # refusal can be argued with rather than merely accepted.
    assert "shoulder_lift" in outcome.telemetry["reason"]
    assert "-0.90" in outcome.telemetry["reason"]
    assert "SO_ARM101_SAFE_RANGE_RAD" in outcome.telemetry["reason"]
    proto.set_position.assert_not_called()


def test_an_arm_parked_outside_its_envelope_is_told_to_home_first(monkeypatch):
    """Both ends of a straight line must be inside the box for the whole line to
    be. When the START is the problem, a Cartesian move cannot fix it."""
    monkeypatch.setenv("SO_ARM101_SAFE_RANGE_RAD", WIDE_RANGES)
    # elbow_flex's zero is tick 2047 and its safe floor is -0.19 rad; park it
    # 400 ticks below zero (~-0.61 rad), well outside.
    actuator, proto = _actuator(positions={3: 1647})
    outcome = _invoke(actuator, "arm.move_to", TARGET)

    assert outcome.outcome_kind == "denied"
    assert outcome.telemetry["deny"] == "unsafe_start"
    assert "arm.home" in outcome.telemetry["reason"]
    proto.set_position.assert_not_called()


def test_a_solve_outside_the_declared_limits_is_denied():
    actuator, proto = _actuator()
    outcome = _invoke(actuator, "arm.move_to", {"x_mm": 100.0, "y_mm": 50.0, "z_mm": 20.0})

    assert outcome.outcome_kind == "denied"
    assert outcome.telemetry["deny"] == "joint_limits"
    proto.set_position.assert_not_called()


@pytest.mark.parametrize("args", [
    {},
    {"x_mm": 150.0, "y_mm": 0.0},
    {"x_mm": 150.0, "y_mm": 0.0, "z_mm": "50"},
    {"x_mm": 150.0, "y_mm": 0.0, "z_mm": float("nan")},
    {"x_mm": 150.0, "y_mm": 0.0, "z_mm": 50.0, "x": 9.0},
])
def test_a_malformed_request_is_denied_before_the_bus_is_touched(args):
    actuator, proto = _actuator()
    outcome = _invoke(actuator, "arm.move_to", args)

    assert outcome.outcome_kind == "denied"
    assert outcome.telemetry["deny"] == "bad_args"
    proto.set_position.assert_not_called()
    proto.read_position.assert_not_called()


def test_a_deny_carries_both_a_sentence_and_a_code():
    """The gateway signs `telemetry` into the receipt and puts `error_message`
    on the wire, so a refusal must be readable both ways."""
    actuator, _ = _actuator()
    outcome = _invoke(actuator, "arm.move_to", TARGET)

    assert outcome.error_message.startswith("unsafe_pose: ")
    assert outcome.telemetry == {"deny": "unsafe_pose",
                                 "reason": outcome.error_message.split(": ", 1)[1]}


def test_bobs_measured_envelope_refuses_the_whole_tabletop():
    """The finding this tool is built on top of, pinned so it cannot be
    forgotten: on this arm's MEASURED envelope, pointing the tool straight down
    is not achievable ANYWHERE in the declared workspace. The wrist tops out at
    +0.41 rad and a vertical tool at tabletop reach needs roughly +1.47.

    `arm.move_to` therefore denies every point on this rig as shipped, and that
    is the correct answer rather than a bug — `arm.reach_point`, which measures
    instead of solving and imposes no tool orientation, is the tool that gets
    there. An operator who has re-measured their own arm widens
    SO_ARM101_SAFE_RANGE_RAD and this tool starts working.

    Sampled coarsely here; a 25,480-point sweep of the same box found zero
    reachable poses too.
    """
    safe = config.resolve_safe_range_rad()
    solved_but_unsafe = 0
    for x in range(-200, 341, 60):
        for y in range(-340, 341, 60):
            for z in range(0, 251, 40):
                solution = kin.solve_tool_down((float(x), float(y), float(z)), M)
                if not solution.ok:
                    continue
                bad = [j for j, a in solution.joints.items()
                       if j in safe and not safe[j][0] <= a <= safe[j][1]]
                assert bad, f"({x},{y},{z}) solved inside the measured envelope"
                solved_but_unsafe += 1
    assert solved_but_unsafe > 0, "the sweep never reached the safe-range check"


# --------------------------------------------------------------------------- #
# arm.state
# --------------------------------------------------------------------------- #

def test_state_reports_every_joint_the_tip_and_the_tool():
    actuator, _ = _actuator()
    outcome = _invoke(actuator, "arm.state", tier="read")

    assert outcome.success
    assert outcome.outcome_kind == "executed"
    telemetry = outcome.telemetry
    assert set(telemetry) == {"joint_positions_rad", "eef_mm", "tool"}
    assert set(telemetry["joint_positions_rad"]) == set(config.JOINTS)
    assert set(telemetry["eef_mm"]) == {"x", "y", "z"}
    assert telemetry["tool"] == "gripper"


def test_state_moves_nothing():
    actuator, proto = _actuator()
    _invoke(actuator, "arm.state", tier="read")
    proto.set_position.assert_not_called()


def test_state_agrees_with_forward_kinematics():
    actuator, _ = _actuator(positions={1: 2100, 2: 2150, 3: 2000, 4: 2060, 5: 2188})
    telemetry = _invoke(actuator, "arm.state", tier="read").telemetry

    expected = kin.tip_position_mm(telemetry["joint_positions_rad"], M)
    assert telemetry["eef_mm"]["x"] == pytest.approx(expected[0])
    assert telemetry["eef_mm"]["y"] == pytest.approx(expected[1])
    assert telemetry["eef_mm"]["z"] == pytest.approx(expected[2])


def test_state_after_a_move_reports_where_the_move_put_it(monkeypatch):
    monkeypatch.setenv("SO_ARM101_SAFE_RANGE_RAD", WIDE_RANGES)
    actuator, _ = _actuator()
    moved = _invoke(actuator, "arm.move_to", TARGET).telemetry
    read_back = _invoke(actuator, "arm.state", tier="read").telemetry

    assert read_back["eef_mm"]["x"] == pytest.approx(moved["eef_mm"]["x"], abs=0.5)
    assert read_back["eef_mm"]["z"] == pytest.approx(moved["eef_mm"]["z"], abs=0.5)


def test_state_still_reports_the_joints_when_there_is_no_geometry(tmp_path):
    """A read that refuses to say where the joints are is useless precisely
    when it is most needed — during a bring-up with an incomplete manifest."""
    bare = tmp_path / "ROBOT.md"
    bare.write_text("---\nmetadata:\n  robot_name: nothing\n---\n\n# nothing\n")
    actuator, _ = _actuator()
    outcome = actuator.execute(
        envelope={"tool_name": "arm.state", "tool_args": {}},
        manifest_path=bare, tier="read", config={})

    assert outcome.success
    assert outcome.telemetry["eef_mm"] is None
    assert outcome.telemetry["joint_positions_rad"]
    assert outcome.telemetry["tool"] is None


# --------------------------------------------------------------------------- #
# Policy
# --------------------------------------------------------------------------- #

def test_a_read_tier_caller_may_look_but_not_move():
    """arm.state sits in status.report's tier class; arm.move_to sits in
    arm.reach's. The driver re-checks both against the TOOL, because the
    gateway's scope gate keys off a string the caller supplies."""
    actuator, proto = _actuator()

    assert _invoke(actuator, "arm.state", tier="read").success is True

    denied = _invoke(actuator, "arm.move_to", TARGET, tier="read")
    assert denied.success is False
    assert "read" in denied.error_message
    proto.set_position.assert_not_called()


def test_an_anon_caller_may_do_neither():
    actuator, _ = _actuator()
    assert _invoke(actuator, "arm.state", tier="anon").success is False
    assert _invoke(actuator, "arm.move_to", TARGET, tier="anon").success is False


def test_both_tools_are_declared_as_implemented():
    """The console builds the robot's advertised capability surface from this
    set. A tool the driver runs but does not declare is invisible to clients."""
    from so_arm101_actuator.actuator import IMPLEMENTED_CAPABILITIES

    assert "arm.move_to" in IMPLEMENTED_CAPABILITIES
    assert "arm.state" in IMPLEMENTED_CAPABILITIES


def test_denied_error_carries_a_code_and_a_detail():
    exc = DeniedError("unreachable", "too far")
    assert exc.code == "unreachable"
    assert exc.detail == "too far"
    assert str(exc) == "unreachable: too far"
