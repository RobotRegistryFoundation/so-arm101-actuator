"""The fixes for what EV-03 found when it ran this driver behind the gateway.

EV-03 (October 2026) replaced the model with a hostile command generator and ran
the real gateway and this driver against a simulated servo bus with bob's
calibration and gravity sag. Configured to do real work, the tip went up to
13 mm below the declared floor, nothing bounded the speed (the servos ran at
270 deg/s, the tip at 1.6 m/s), arm.reach_point steered to points below the
floor, a repeated arm.estop walked the arm down under gravity, and a second
process could drive the servos while the gateway held the port. Each section
below pins one fix.
"""

from __future__ import annotations

import math
import os
from pathlib import Path
from unittest.mock import MagicMock

import pytest

from so_arm101_actuator import actuator as act_mod
from so_arm101_actuator import config, motion
from so_arm101_actuator.actuator import ADOPT_REFERENCE_TICKS, SOArm101Actuator
from so_arm101_actuator.errors import DeniedError
from so_arm101_actuator.kinematics import Box
from tests.conftest import FIXTURE_MANIFEST

M = FIXTURE_MANIFEST


# --------------------------------------------------------------------------- #
# The path, not just its ends (motion.check_line)
# --------------------------------------------------------------------------- #

class _LineChain:
    """A stand-in chain whose tip x follows a table as joint "j" goes 0 -> 1, so
    a test can draw exactly the path it wants to check."""

    def __init__(self, xs: list[float]):
        self.xs = xs

    def points(self, q):
        s = q["j"] * (len(self.xs) - 1)             # extrapolates past either end
        i = min(max(int(math.floor(s)), 0), len(self.xs) - 2)
        x = self.xs[i] + (self.xs[i + 1] - self.xs[i]) * (s - i)
        far = (0.0, 0.0, 100.0)
        return {"tip": (x, 0.0, 100.0), "wrist": far, "elbow": far}


BOX = Box((-500.0, -500.0, 0.0), (340.0, 500.0, 500.0))


def _check(xs, margin=10.0, offset=None):
    return motion.check_line(_LineChain(xs), BOX, {"j": 0.0}, {"j": 1.0},
                             offset=offset or {"j": 0.0}, margin_mm=margin, samples=200)


def test_ends_inside_with_a_middle_that_crosses_the_face_is_refused():
    with pytest.raises(DeniedError) as exc:
        _check([300.0, 345.0, 300.0])
    assert exc.value.code == "path_leaves_workspace"
    assert "x<=340" in exc.value.detail


def test_a_path_that_keeps_the_margin_passes_and_reports_how_close_it_came():
    clearance, closest = _check([300.0, 329.0, 300.0])
    assert clearance == pytest.approx(11.0, abs=0.01)
    assert closest["point"] == "tip" and closest["face"] == "x<=340"


def test_inside_the_box_but_inside_the_margin_is_refused():
    with pytest.raises(DeniedError):
        _check([300.0, 335.0, 300.0])


def test_an_arm_outside_its_envelope_may_come_back_in():
    """Pushed out, or parked badly: arm.home must still be able to bring it in."""
    _check([345.0, 346.5, 300.0])        # dips 1.5 mm further out on the way, ends inside


def test_but_never_walks_further_out_than_the_recovery_allowance():
    with pytest.raises(DeniedError):
        _check([345.0, 345.0 + motion.RECOVERY_DIP_MM + 1.0, 300.0])


def test_a_move_out_of_the_margin_band_may_approach_the_face_but_never_cross_it():
    dip = motion.RECOVERY_DIP_MM
    _check([335.0, 335.0 + dip - 0.5, 300.0])   # starts 5 mm inside, dips within the allowance
    with pytest.raises(DeniedError):
        _check([335.0, 335.0 + dip + 0.5, 300.0])   # a bigger dip: refused
    with pytest.raises(DeniedError):
        _check([339.0, 340.5, 300.0])    # past the face: refused even though it ends inside


def test_a_move_that_stays_in_the_band_may_not_come_any_closer():
    _check([335.0, 334.0])               # farther from the face: fine
    with pytest.raises(DeniedError):
        _check([335.0, 336.0])           # closer: refused


def test_the_predicted_series_catches_what_the_commanded_line_does_not():
    """The arm sits `offset` from its reference (sag). The commanded line keeps
    the margin; the arm, offset by the same amount, would not."""
    _check([300.0, 325.0])
    with pytest.raises(DeniedError) as exc:
        _check([300.0, 325.0], offset={"j": 0.4})       # the arm sits 10 mm further out: x = 335
    assert "predicted" in exc.value.detail


# --------------------------------------------------------------------------- #
# Pacing (motion.pace_line)
# --------------------------------------------------------------------------- #

def test_pacing_spreads_a_move_so_the_tool_never_outruns_the_limit():
    chain = _LineChain([0.0, 300.0])                    # a 300 mm straight tool path
    setpoints, step, duration, path = motion.pace_line(
        chain, {"j": 0.0}, {"j": 1.0}, samples=200, speed=1.0, tool_mps=0.25, joint_dps=None)
    assert path == pytest.approx(300.0)
    assert duration == pytest.approx(0.3 / (0.25 * motion.PACE_FRACTION))
    assert step <= motion.PACE_PERIOD_S + 1e-12
    xs = [chain.points(p)["tip"][0] for p in setpoints]
    steps = [b - a for a, b in zip([0.0, *xs[:-1]], xs, strict=True)]
    assert max(steps) / 1000.0 / step <= 0.25 * motion.PACE_FRACTION * (1 + 1e-9)
    assert setpoints[-1] == {"j": 1.0}


def test_a_move_that_would_outlast_the_ceiling_is_refused_as_too_slow():
    with pytest.raises(DeniedError) as exc:
        motion.pace_line(_LineChain([0.0, 300.0]), {"j": 0.0}, {"j": 1.0}, samples=200,
                         speed=0.001, tool_mps=0.25, joint_dps=None)
    assert exc.value.code == "too_slow"


@pytest.mark.parametrize("speed", [5e-324, 1e-309])
def test_a_subnormal_speed_is_refused_not_an_overflow(speed):
    with pytest.raises(DeniedError) as exc:
        motion.pace_line(_LineChain([0.0, 300.0]), {"j": 0.0}, {"j": 1.0}, samples=200,
                         speed=speed, tool_mps=0.25, joint_dps=None)
    assert exc.value.code == "too_slow"


# --------------------------------------------------------------------------- #
# A sagging servo, for the stop and the reference
# --------------------------------------------------------------------------- #

class _SaggingBus:
    """Every joint reads `sag` ticks BELOW whatever goal it was last sent, the
    way a loaded joint does; the goal register reports the goal."""

    def __init__(self, start: dict[int, int] | None = None, sag: int = 12):
        self.goal = {i: 2048 for i in range(1, 7)}
        self.goal.update(start or {})
        self.sag = sag
        self.writes: list[tuple[int, int]] = []

    def set_position(self, motor_id, ticks):
        self.goal[motor_id] = ticks
        self.writes.append((motor_id, ticks))

    def read_position(self, motor_id):
        return self.goal[motor_id] - self.sag

    def read_goal_position(self, motor_id):
        return self.goal[motor_id]

    def read_temperature(self, motor_id):
        return 30


def _estop(actuator, tier="read"):
    return actuator.execute(envelope={"tool_name": "arm.estop", "tool_args": {}},
                            manifest_path="", tier=tier, config={})


def test_repeating_the_stop_never_walks_a_sagging_arm_down():
    """The ratchet: each stop used to send the READING as the goal, and under
    gravity the reading is below the goal, so every stop lowered the arm by the
    sag (9 cm in 8 s of stops 0.2 s apart in EV-03's simulation)."""
    bus = _SaggingBus()
    actuator = SOArm101Actuator(protocol=bus)
    for _ in range(50):
        assert _estop(actuator).success
    lift = config.JOINTS["shoulder_lift"]["motor_id"]
    sent = {ticks for motor, ticks in bus.writes if motor == lift}
    assert sent == {2048}            # the goal it was holding, every time
    assert bus.goal[lift] == 2048


def test_the_first_stop_holds_the_goal_a_joint_is_holding_not_its_sag():
    bus = _SaggingBus(sag=25)
    actuator = SOArm101Actuator(protocol=bus)
    outcome = _estop(actuator)
    held = outcome.telemetry["held_positions"]
    for joint, rad in held.items():
        assert config.rad_to_ticks(joint, rad) == 2048


def test_a_joint_far_from_its_goal_is_stopped_where_it_is():
    """Further than ADOPT_REFERENCE_TICKS from the goal, the joint is moving or
    was pushed, and holding the goal would mean finishing the move."""
    bus = _SaggingBus(sag=ADOPT_REFERENCE_TICKS + 60)
    actuator = SOArm101Actuator(protocol=bus)
    _estop(actuator)
    lift = config.JOINTS["shoulder_lift"]["motor_id"]
    assert bus.goal[lift] == 2048 - ADOPT_REFERENCE_TICKS - 60


def test_clearing_the_stop_lets_the_next_one_choose_again(fake_clock):
    bus = _SaggingBus()
    actuator = SOArm101Actuator(protocol=bus)
    _estop(actuator)
    actuator.execute(envelope={"tool_name": "arm.estop.clear", "tool_args": {}},
                     manifest_path="", tier="commission", config={})
    assert actuator._held is None


def test_moves_are_planned_from_the_goal_being_held_not_from_the_sag(fake_clock):
    bus = _SaggingBus(sag=12)
    actuator = SOArm101Actuator(protocol=bus)
    start, offset = actuator._planning_start(["shoulder_lift"])
    assert config.rad_to_ticks("shoulder_lift", start["shoulder_lift"]) == 2048
    assert offset["shoulder_lift"] == pytest.approx(config.ticks_to_rad("shoulder_lift", 2036)
                                                    - config.ticks_to_rad("shoulder_lift", 2048))


# --------------------------------------------------------------------------- #
# The stop interrupts a paced move
# --------------------------------------------------------------------------- #

def test_a_stop_during_a_paced_move_ends_the_stream_at_the_next_setpoint(fake_clock):
    """A paced move holds the bus lock for its whole stream. The stop latches
    BEFORE it waits for that lock, and the stream checks the latch before every
    setpoint, so a move in progress ends within one pace period."""
    state = {i: 2048 for i in range(1, 7)}
    proto = MagicMock()
    proto.read_position.side_effect = lambda motor_id: state[motor_id]
    proto.read_goal_position.side_effect = lambda motor_id: state[motor_id]
    actuator = SOArm101Actuator(protocol=proto)
    actuator._apply_manifest(M)
    transport = actuator._estop_transport(port="/dev/null", baud=1_000_000)

    def write(motor_id, ticks):
        state[motor_id] = ticks
        if proto.set_position.call_count == 25:      # the stop arrives mid-stream
            transport._estopped = True

    proto.set_position.side_effect = write
    result = actuator._move_checked({"shoulder_pan": 0.6})

    assert result["motion"]["stopped_by_estop"] is True
    assert result["reached"] is False
    assert proto.set_position.call_count < result["motion"]["setpoints"]


# --------------------------------------------------------------------------- #
# arm.reach_point
# --------------------------------------------------------------------------- #

def _reach(actuator, target, tier="actuate"):
    return actuator.execute(envelope={"tool_name": "arm.reach_point",
                                      "tool_args": {"target_mm": target}},
                            manifest_path=Path(M), tier=tier, config={})


@pytest.mark.parametrize("target", [[250.0, 0.0, -60.0], [300.0, 0.0, -100.0], [200.0, 0.0, 4.0]])
def test_reach_point_refuses_targets_outside_the_workspace_and_its_margin(target):
    """It checked reach only, so it steered the tip to z = -101.7 mm."""
    proto = MagicMock()
    proto.read_position.return_value = 2048
    actuator = SOArm101Actuator(protocol=proto)
    outcome = _reach(actuator, target)
    assert outcome.outcome_kind == "denied"
    assert outcome.telemetry["deny"] == "out_of_workspace"
    proto.set_position.assert_not_called()


def test_reach_point_on_a_fresh_gateway_opens_the_bus_instead_of_failing():
    """Its branch never opened the port, so its first call was a 500."""
    actuator = SOArm101Actuator(protocol=None)
    opened = []

    def ensure(*, port, baud):
        assert act_mod._BUS_LOCK.locked()            # under the same lock as every motion
        opened.append((port, baud))
        actuator._protocol = MagicMock(read_position=MagicMock(return_value=2048))

    actuator._ensure_protocol = ensure
    outcome = _reach(actuator, [250.0, 0.0, -60.0])
    assert opened == [("/dev/ttyACM0", 1_000_000)]
    assert outcome.outcome_kind == "denied"          # a decision, not an AttributeError


@pytest.mark.parametrize("args", [{}, {"target_mm": [1, 2]}, {"target_mm": ["a", 0, 0]},
                                  {"target_mm": [200, 0, 50], "tolerance_mm": -1}])
def test_reach_point_with_bad_arguments_is_a_refusal(args):
    actuator = SOArm101Actuator(protocol=MagicMock())
    outcome = actuator.execute(envelope={"tool_name": "arm.reach_point", "tool_args": args},
                               manifest_path=Path(M), tier="actuate", config={})
    assert outcome.outcome_kind == "denied"
    assert outcome.telemetry["deny"] == "bad_args"


# --------------------------------------------------------------------------- #
# Joint moves the gateway can ask for are checked too
# --------------------------------------------------------------------------- #

def test_a_joint_move_whose_path_leaves_the_workspace_is_refused(fake_clock):
    """arm.home, arm.reach and a bare `move` go through the same check as
    arm.move_to. Here: from a pose inside the box, swing the shoulder until the
    tip would pass below the floor."""
    state = {i: 2048 for i in range(1, 7)}
    state.update({2: 1800, 3: 2300})                  # the manifest's ready pose
    proto = MagicMock()
    proto.read_position.side_effect = lambda motor_id: state[motor_id]
    proto.set_position.side_effect = lambda motor_id, ticks: state.__setitem__(motor_id, ticks)
    actuator = SOArm101Actuator(protocol=proto)
    outcome = actuator.execute(
        envelope={"tool_name": "move", "tool_args": {"joint_positions": {"shoulder_lift": 1.5}}},
        manifest_path=Path(M), tier="actuate", config={})
    assert outcome.outcome_kind == "denied"
    assert outcome.telemetry["deny"] == "path_leaves_workspace"
    proto.set_position.assert_not_called()


def test_a_bare_move_takes_only_the_arguments_it_documents():
    actuator = SOArm101Actuator(protocol=MagicMock())
    outcome = actuator.execute(
        envelope={"tool_name": "move", "tool_args": {"joint_positions": {"shoulder_pan": 0.1},
                                                     "hold": ["gripper"]}},
        manifest_path=Path(M), tier="actuate", config={})
    assert outcome.outcome_kind == "denied"
    assert outcome.telemetry["deny"] == "bad_args"


# --------------------------------------------------------------------------- #
# The bus, held exclusively
# --------------------------------------------------------------------------- #

def test_claim_opens_the_bus_now_with_the_gateways_config():
    actuator = SOArm101Actuator(protocol=None)
    seen = []
    actuator._ensure_protocol = lambda *, port, baud: seen.append((port, baud))
    actuator.claim({"port": "/dev/serial/by-id/arm", "baud": 500_000})
    assert seen == [("/dev/serial/by-id/arm", 500_000)]


def test_dropping_the_protocol_releases_the_handle():
    handle = MagicMock(is_open=True)
    actuator = SOArm101Actuator(protocol=MagicMock(_serial=handle))
    actuator.close()
    handle.close.assert_called_once()
    assert actuator._protocol is None


@pytest.fixture
def pty_port():
    import pty

    pytest.importorskip("serial")
    controller, worker = pty.openpty()
    name = os.ttyname(worker)
    yield name
    os.close(controller)
    os.close(worker)


def test_the_bus_is_opened_exclusively(pty_port):
    """A second opener that asks for the lock is refused: the gateway's own
    second instance, or any tool that opens the port the same way."""
    import serial

    first = act_mod._open_serial(pty_port, 1_000_000)
    try:
        with pytest.raises((serial.SerialException, OSError)):
            act_mod._open_serial(pty_port, 1_000_000)
    finally:
        act_mod._release_serial(first)
    again = act_mod._open_serial(pty_port, 1_000_000)   # released: it can be had again
    act_mod._release_serial(again)


@pytest.mark.skipif(os.geteuid() == 0, reason="TIOCEXCL does not bind a process with CAP_SYS_ADMIN")
def test_a_plain_open_by_another_process_is_refused_while_the_gateway_holds_it(pty_port):
    """LeRobot takes no lock at all; TIOCEXCL is what refuses its open(2)."""
    import errno

    held = act_mod._open_serial(pty_port, 1_000_000)
    try:
        with pytest.raises(OSError) as exc:
            os.close(os.open(pty_port, os.O_RDWR | os.O_NOCTTY))
        assert exc.value.errno == errno.EBUSY
    finally:
        act_mod._release_serial(held)
