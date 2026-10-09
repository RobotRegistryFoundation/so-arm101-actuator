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


def test_a_move_out_of_the_margin_band_keeps_half_the_margin():
    """Starting 7.6 mm from the face (bob's taught ready pose), a move that ends
    well inside may come to 5 mm (half the 10 mm margin) and no closer: the
    margin is for model error, and leaving the band does not remove it."""
    _check([332.4, 334.9, 300.0])        # 5.1 mm at its closest
    with pytest.raises(DeniedError):
        _check([332.4, 335.5, 300.0])    # 4.5 mm: refused
    # Starting closer than half the margin, it may come START_DIP_MM closer at
    # most (the first millimetre of a line out of a pose), and never past the face.
    _check([336.0, 336.0 + motion.START_DIP_MM - 0.1, 300.0])
    with pytest.raises(DeniedError):
        _check([336.0, 336.0 + motion.START_DIP_MM + 0.5, 300.0])
    with pytest.raises(DeniedError):
        _check([339.5, 340.2, 300.0])


def test_recovering_from_one_face_does_not_license_another():
    """Clearance is kept per face: an arm past x = 340 that is coming back in
    may not use that as cover to approach the floor."""

    class _TwoFace:
        def points(self, q):
            j = q["j"]
            x = 345.0 - 45.0 * j                    # from 5 mm past x = 340 to 40 mm inside
            z = 20.0 - 60.0 * j * (1 - j) * 4 / 3   # dips toward the floor in the middle
            far = (0.0, 0.0, 100.0)
            return {"tip": (x, 0.0, z), "wrist": far, "elbow": far}

    with pytest.raises(DeniedError) as exc:
        motion.check_line(_TwoFace(), BOX, {"j": 0.0}, {"j": 1.0}, offset={"j": 0.0},
                          margin_mm=10.0, samples=200)
    assert "z>=0" in exc.value.detail


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


# --------------------------------------------------------------------------- #
# Found by the independent review of the first version of these fixes
# --------------------------------------------------------------------------- #

def test_a_command_while_the_arm_is_busy_is_refused_not_queued(monkeypatch):
    """Requests queued behind a long move each held a gateway thread, and a burst
    of polls left the stop without one: a 25 s move ran to the end with a stop
    sent at 5 s. Now a request waits BUS_WAIT_S for the bus, then is refused."""
    monkeypatch.setattr(act_mod, "BUS_WAIT_S", 0.05)
    actuator = SOArm101Actuator(protocol=MagicMock(read_position=MagicMock(return_value=2048)))
    with act_mod._BUS_LOCK:                                  # a move is holding the bus
        outcome = actuator.execute(envelope={"tool_name": "arm.state", "tool_args": {}},
                                   manifest_path="", tier="read", config={})
    assert outcome.outcome_kind == "denied"
    assert outcome.telemetry["deny"] == "busy"


def test_the_stop_latches_even_while_another_request_holds_the_bus(monkeypatch):
    monkeypatch.setattr(act_mod, "STOP_BUS_WAIT_S", 0.05)
    actuator = SOArm101Actuator(protocol=_SaggingBus())
    with act_mod._BUS_LOCK:
        outcome = _estop(actuator)
    assert actuator._latched()                   # motion is refused from here on
    assert outcome.telemetry["estopped"] is True


def test_a_repeated_stop_does_not_drive_a_joint_back_to_a_goal_it_has_lost():
    """After a stop, a power cycle resets a servo's goal and the arm drops.
    Re-sending the old hold would drive it back at top speed (2.4 m/s in sim);
    holding where it reads would lower the goal by the sag. It holds the goal
    the servo now holds, which moves nothing."""
    bus = _SaggingBus()
    actuator = SOArm101Actuator(protocol=bus)
    _estop(actuator)
    lift = config.JOINTS["shoulder_lift"]["motor_id"]
    bus.goal[lift] = 1500                      # the goal reset; the joint now reads 1488
    bus.writes.clear()
    _estop(actuator)
    assert (lift, 2048) not in bus.writes
    assert (lift, 1500) in bus.writes


def test_a_reset_inside_the_adoption_window_is_not_driven_back_either():
    """The first version re-sent the old hold whenever the joint read within
    100 ticks of it, so a 50-tick reset still lifted the arm 40 mm unpaced."""
    bus = _SaggingBus()
    actuator = SOArm101Actuator(protocol=bus)
    _estop(actuator)
    lift = config.JOINTS["shoulder_lift"]["motor_id"]
    bus.goal[lift] = 2048 - 50
    bus.writes.clear()
    _estop(actuator)
    assert (lift, 2048) not in bus.writes
    assert (lift, 2048 - 50) in bus.writes


def test_repeated_stops_never_ratchet_an_arm_whose_sag_passes_the_adoption_window():
    """Found in review: a joint sagging more than ADOPT_REFERENCE_TICKS was
    re-anchored at its reading on every repeat, and five times bob's sag walked
    the tip from z = 112 to -103 mm in ten stops. The first stop may hold it
    where it reads (it cannot tell a heavy sag from a move); no later one moves
    the goal again."""
    bus = _SaggingBus(sag=ADOPT_REFERENCE_TICKS + 60)
    actuator = SOArm101Actuator(protocol=bus)
    lift = config.JOINTS["shoulder_lift"]["motor_id"]
    _estop(actuator)
    first = bus.goal[lift]
    for _ in range(50):
        assert _estop(actuator).success
    assert {ticks for motor, ticks in bus.writes if motor == lift} == {first}


def test_without_a_goal_register_repeated_stops_still_never_ratchet():
    class _NoRegister(_SaggingBus):
        read_goal_position = None             # a bus that cannot report it

    bus = _NoRegister(sag=ADOPT_REFERENCE_TICKS + 60)
    actuator = SOArm101Actuator(protocol=bus)
    lift = config.JOINTS["shoulder_lift"]["motor_id"]
    _estop(actuator)
    first = bus.goal[lift]
    for _ in range(20):
        _estop(actuator)
    assert bus.goal[lift] == first


def test_a_move_after_a_goal_reset_plans_from_the_register_not_the_stale_copy(fake_clock):
    """Found in review: planning from the copy of the last goal sent, after a
    brown-out reset the servo's goal 50 ticks away, gave a zero-length plan
    whose one setpoint jumped the arm back at 1.5 m/s."""
    bus = _SaggingBus()
    actuator = SOArm101Actuator(protocol=bus)
    lift = config.JOINTS["shoulder_lift"]["motor_id"]
    actuator._write_goal("shoulder_lift", 2048)          # what this process last sent
    bus.goal[lift] = 2048 - 50                            # the reset
    start, offset = actuator._planning_start(["shoulder_lift"])
    assert config.rad_to_ticks("shoulder_lift", start["shoulder_lift"]) == 2048 - 50


def test_a_stall_makes_the_move_longer_never_faster(fake_clock):
    """Setpoints used to be due at fixed times, so after a stall the overdue
    ones went out back to back (0.55 m/s against a 0.25 m/s limit)."""
    state = {i: 2048 for i in range(1, 7)}
    state.update({2: 1800, 3: 2300})
    writes: list[float] = []
    proto = MagicMock()
    proto.read_position.side_effect = lambda motor_id: state[motor_id]

    def write(motor_id, ticks):
        state[motor_id] = ticks
        if motor_id == 1:
            writes.append(fake_clock.now)
            if len(writes) == 20:
                fake_clock.now += 0.3                       # the bus stalls once
    proto.set_position.side_effect = write
    actuator = SOArm101Actuator(protocol=proto)
    actuator._apply_manifest(M)
    result = actuator._move_checked({"shoulder_pan": 0.5})
    step = result["motion"]["pace_period_s"]
    gaps = [b - a for a, b in zip(writes, writes[1:], strict=False)]
    assert min(gaps) >= step - 1e-4      # telemetry rounds the period to 0.1 ms
    assert max(gaps) >= 0.3 - 1e-6       # the stall itself is in there, and nothing made up for it


def test_joints_that_do_not_move_the_tool_are_paced_too(fake_clock):
    """With no joint limit declared, a wrist_roll move used to be one command at
    the servo's top speed: it does not move the tool point, so the tool limit
    never bound it."""
    state = {i: 2048 for i in range(1, 7)}
    state.update({2: 1800, 3: 2300})
    proto = MagicMock()
    proto.read_position.side_effect = lambda motor_id: state[motor_id]
    proto.set_position.side_effect = lambda motor_id, ticks: state.__setitem__(motor_id, ticks)
    actuator = SOArm101Actuator(protocol=proto)
    actuator._apply_manifest(M)
    result = actuator._move_checked({"wrist_roll": 1.6})
    assert result["motion"]["joint_speed_limit_dps"] == pytest.approx(90.0)
    assert result["motion"]["setpoints"] > 20


def _ready_arm(ticks: dict[int, int] | None = None):
    state = {i: 2048 for i in range(1, 7)}
    state.update({2: 1800, 3: 2300, 6: 1700})
    state.update(ticks or {})
    proto = MagicMock()
    proto.read_position.side_effect = lambda motor_id: state[motor_id]
    proto.set_position.side_effect = lambda motor_id, ticks: state.__setitem__(motor_id, ticks)
    return state, proto


def test_arm_home_brings_the_arm_home_though_the_taught_pose_is_inside_the_margin(
        fake_clock, monkeypatch):
    """bob's taught ready pose is 7.6 mm from x = 340, inside the 10 mm margin.
    The first version refused arm.home from anywhere but home itself."""
    monkeypatch.setenv("SO_ARM101_SAFE_RANGE_RAD", (
        '{"shoulder_lift": [-1.45, 1.0], "elbow_flex": [-0.19, 1.5], "wrist_flex": [-0.93, 1.5]}'))
    state, proto = _ready_arm()
    actuator = SOArm101Actuator(protocol=proto)
    away = actuator.execute(envelope={"tool_name": "arm.move_to",
                                      "tool_args": {"x_mm": 150.0, "y_mm": 0.0, "z_mm": 50.0}},
                            manifest_path=Path(M), tier="actuate", config={})
    assert away.outcome_kind == "executed", away.error_message
    home = actuator.execute(envelope={"tool_name": "arm.home", "tool_args": {}},
                            manifest_path=Path(M), tier="actuate", config={})
    assert home.outcome_kind == "executed", home.error_message
    # It may come as close as the taught pose itself (7.6 mm) or half the
    # margin (5 mm), whichever is closer, less START_DIP_MM: 4 mm, no closer.
    assert home.telemetry["motion"]["path_min_clearance_mm"] >= 4.0


def test_a_taught_pose_outside_the_workspace_is_refused_with_the_fix_named(tmp_path, fake_clock):
    text = Path(M).read_text()
    manifest = tmp_path / "ROBOT.md"
    # Every joint at mid-travel puts this arm's tip 13 mm past x = 340.
    manifest.write_text(text.replace("shoulder_lift: 1800", "shoulder_lift: 2048")
                        .replace("elbow_flex: 2300", "elbow_flex: 2048"))
    state, proto = _ready_arm()
    actuator = SOArm101Actuator(protocol=proto)
    outcome = actuator.execute(envelope={"tool_name": "arm.home", "tool_args": {}},
                               manifest_path=manifest, tier="actuate", config={})
    assert outcome.outcome_kind == "denied"
    assert "taught pose" in outcome.telemetry["reason"]
    proto.set_position.assert_not_called()


def test_a_joint_parked_past_its_configured_range_can_still_be_brought_home(fake_clock):
    """bob's firmware lets shoulder_lift reach -1.578 rad against a configured
    -1.5. Setpoints between that start and the range used to fail conversion
    half-way through a write."""
    state, proto = _ready_arm({2: 1205})
    actuator = SOArm101Actuator(protocol=proto)
    actuator._apply_manifest(M)
    assert config.ticks_to_rad("shoulder_lift", 1205) < config.JOINTS["shoulder_lift"]["min_rad"]
    outcome = actuator.execute(envelope={"tool_name": "arm.home", "tool_args": {}},
                               manifest_path=Path(M), tier="actuate", config={})
    assert outcome.outcome_kind == "executed", outcome.error_message


def test_a_manifest_whose_geometry_cannot_be_read_refuses_motion(tmp_path, fake_clock):
    """It used to leave the workspace and chain resolving to nothing, so a
    shoulder swing into the floor executed unchecked."""
    text = Path(M).read_text()
    manifest = tmp_path / "ROBOT.md"
    manifest.write_text(text.replace("  solver:\n", "  solver: [broken]\n  solver_was:\n", 1))
    state, proto = _ready_arm()
    actuator = SOArm101Actuator(protocol=proto)
    outcome = actuator.execute(envelope={"tool_name": "arm.home", "tool_args": {}},
                               manifest_path=manifest, tier="actuate", config={})
    assert outcome.outcome_kind == "denied"
    assert outcome.telemetry["deny"] == "manifest_unreadable"
    proto.set_position.assert_not_called()


def test_a_reach_point_step_the_workspace_refuses_is_a_signed_refusal(monkeypatch):
    proto = MagicMock(read_position=MagicMock(return_value=2048))
    actuator = SOArm101Actuator(protocol=proto)

    def refuse(*args, **kwargs):
        raise DeniedError("path_leaves_workspace", "the tip would come 3 mm from z>=0")

    monkeypatch.setattr(actuator, "_move_checked", refuse)
    outcome = _reach(actuator, [200.0, 0.0, 120.0])
    assert outcome.outcome_kind == "denied"
    assert outcome.telemetry["deny"] == "path_leaves_workspace"


def test_the_bare_move_to_alias_is_parsed_like_arm_move_to():
    """It skipped argument parsing: a string coordinate and timeout_s=3600 went
    through, and the bus was held for an hour."""
    actuator = SOArm101Actuator(protocol=MagicMock())
    outcome = actuator.execute(
        envelope={"tool_name": "move_to",
                  "tool_args": {"x_mm": "150", "y_mm": 0.0, "z_mm": 50.0, "timeout_s": 3600}},
        manifest_path=Path(M), tier="actuate", config={})
    assert outcome.outcome_kind == "denied"
    assert outcome.telemetry["deny"] == "bad_args"


# --------------------------------------------------------------------------- #
# Edges: a typo must not remove the margin; a servo that cannot say is unknown
# --------------------------------------------------------------------------- #

@pytest.mark.parametrize("raw", ["ten", "", "-1", "nan", "inf"])
def test_a_margin_override_that_is_not_a_distance_is_refused_not_read_as_zero(monkeypatch, raw):
    monkeypatch.setenv("SO_ARM101_WORKSPACE_MARGIN_MM", raw)
    with pytest.raises(ValueError, match="SO_ARM101_WORKSPACE_MARGIN_MM"):
        act_mod.resolve_workspace_margin_mm()


def test_a_bad_margin_override_moves_nothing(monkeypatch, fake_clock):
    monkeypatch.setenv("SO_ARM101_WORKSPACE_MARGIN_MM", "ten")
    state, proto = _ready_arm()
    actuator = SOArm101Actuator(protocol=proto)
    outcome = actuator.execute(envelope={"tool_name": "arm.home", "tool_args": {}},
                               manifest_path=Path(M), tier="actuate", config={})
    assert outcome.outcome_kind != "executed"
    assert "SO_ARM101_WORKSPACE_MARGIN_MM" in (outcome.error_message or "")
    proto.set_position.assert_not_called()


def test_the_margin_override_narrows_or_widens_the_default(monkeypatch):
    monkeypatch.delenv("SO_ARM101_WORKSPACE_MARGIN_MM", raising=False)
    assert act_mod.resolve_workspace_margin_mm() == act_mod.WORKSPACE_MARGIN_MM
    monkeypatch.setenv("SO_ARM101_WORKSPACE_MARGIN_MM", "5")
    assert act_mod.resolve_workspace_margin_mm() == 5.0


@pytest.mark.parametrize("reply", [True, -1, 4096, 2048.0, None])
def test_a_goal_register_reply_that_is_not_a_position_is_unknown(reply):
    proto = MagicMock()
    proto.read_goal_position.return_value = reply
    assert SOArm101Actuator(protocol=proto)._read_goal_register("shoulder_lift") is None


def test_a_goal_register_read_that_fails_is_retried_once_then_raises():
    """Found in the third review: a failed read used to read as "cannot say",
    and the callers fell back to a copy a reset may have made stale."""
    proto = MagicMock()
    proto.read_goal_position.side_effect = [OSError("garbled"), 1990]
    assert SOArm101Actuator(protocol=proto)._read_goal_register("shoulder_lift") == 1990
    proto.read_goal_position.side_effect = OSError("no reply")
    with pytest.raises(OSError):
        SOArm101Actuator(protocol=proto)._read_goal_register("shoulder_lift")


def test_a_bus_without_a_goal_register_plans_from_where_the_arm_reads():
    proto = MagicMock(spec=["read_position", "set_position", "read_temperature"])
    proto.read_position.return_value = 2000
    actuator = SOArm101Actuator(protocol=proto)
    assert actuator._read_goal_register("shoulder_lift") is None
    start, offset = actuator._planning_start(["shoulder_lift"])
    assert config.rad_to_ticks("shoulder_lift", start["shoulder_lift"]) == 2000
    assert offset["shoulder_lift"] == 0.0


@pytest.mark.parametrize("value", [float("nan"), float("inf"), True, "0.1"])
def test_a_joint_target_that_is_not_a_finite_number_never_reaches_the_bus(value, fake_clock):
    state, proto = _ready_arm()
    actuator = SOArm101Actuator(protocol=proto)
    actuator._apply_manifest(M)
    with pytest.raises(ValueError):
        actuator._move_checked({"shoulder_pan": value}, speed=1.0, timeout_s=5.0)
    proto.set_position.assert_not_called()


def test_a_joint_target_past_its_configured_range_never_reaches_the_bus(fake_clock):
    state, proto = _ready_arm()
    actuator = SOArm101Actuator(protocol=proto)
    actuator._apply_manifest(M)
    beyond = config.JOINTS["shoulder_pan"]["max_rad"] + 0.1
    with pytest.raises(ValueError):
        actuator._move_checked({"shoulder_pan": beyond}, speed=1.0, timeout_s=5.0)
    proto.set_position.assert_not_called()


# --------------------------------------------------------------------------- #
# Found by the second review
# --------------------------------------------------------------------------- #

def test_while_a_stop_waits_for_the_bus_every_other_request_gives_way(monkeypatch):
    """Python's lock is not first-come-first-served: under 120 looping clients
    the stop's hold went out 4 s after it latched. Now a request that has not
    got the bus while a stop waits is refused at once, not after BUS_WAIT_S."""
    import threading
    import time as real_time

    monkeypatch.setattr(act_mod, "BUS_WAIT_S", 2.0)
    monkeypatch.setattr(act_mod, "STOP_BUS_WAIT_S", 5.0)
    bus = _SaggingBus()
    actuator = SOArm101Actuator(protocol=bus)
    done: list = []
    act_mod._BUS_LOCK.acquire()                         # a move is holding the bus
    try:
        stopper = threading.Thread(target=lambda: done.append(_estop(actuator)))
        stopper.start()
        deadline = real_time.monotonic() + 2.0
        while not act_mod._STOPS_WAITING and real_time.monotonic() < deadline:
            real_time.sleep(0.001)
        assert act_mod._STOPS_WAITING == 1
        started = real_time.monotonic()
        poll = actuator.execute(envelope={"tool_name": "arm.state", "tool_args": {}},
                                manifest_path="", tier="read", config={})
        assert real_time.monotonic() - started < 0.5      # not BUS_WAIT_S
        assert poll.telemetry["deny"] == "busy"
    finally:
        act_mod._BUS_LOCK.release()
    stopper.join(timeout=5.0)
    assert done and done[0].success
    assert act_mod._STOPS_WAITING == 0


def test_a_request_already_waiting_when_a_stop_arrives_hands_the_bus_back(monkeypatch):
    import threading
    import time as real_time

    outcome: list = []

    def waiter():
        try:
            with act_mod._bus(5.0):
                outcome.append("took the bus")
        except DeniedError as exc:
            outcome.append(exc.code)

    act_mod._BUS_LOCK.acquire()                          # a move holds the bus
    thread = threading.Thread(target=waiter)
    thread.start()
    real_time.sleep(0.05)                                # the poll is now waiting for it
    monkeypatch.setattr(act_mod, "_STOPS_WAITING", 1)    # and then a stop arrives
    act_mod._BUS_LOCK.release()
    thread.join(timeout=5.0)
    assert outcome == ["busy"]
    assert act_mod._BUS_LOCK.acquire(blocking=False)     # handed straight back
    act_mod._BUS_LOCK.release()


def test_a_request_arriving_while_a_stop_waits_is_refused_without_waiting(monkeypatch):
    monkeypatch.setattr(act_mod, "_STOPS_WAITING", 1)
    with pytest.raises(DeniedError) as exc, act_mod._bus(5.0):
        pass
    assert exc.value.code == "busy"


@pytest.mark.parametrize("broken", [
    "---\nmetadata:\n\trobot_name: tab\n---\n",           # a stray tab: YAML refuses it
    "no frontmatter at all\n",
    "---\nmetadata: {robot_name: never-closed}\n",
    "---\n- a\n- list\n---\n",
])
def test_a_manifest_that_does_not_parse_refuses_motion(tmp_path, fake_clock, broken):
    """Found in review: frontmatter YAML could not parse read as an empty
    manifest, which declares no workspace and no chain, and a shoulder swing
    executed with no path check at all."""
    manifest = tmp_path / "ROBOT.md"
    manifest.write_text(broken)
    state, proto = _ready_arm()
    actuator = SOArm101Actuator(protocol=proto)
    outcome = actuator.execute(envelope={"tool_name": "arm.home", "tool_args": {}},
                               manifest_path=manifest, tier="actuate", config={})
    assert outcome.outcome_kind == "denied"
    assert outcome.telemetry["deny"] == "manifest_unreadable"
    proto.set_position.assert_not_called()


def test_a_manifest_broken_after_it_was_read_refuses_motion_too(tmp_path, fake_clock):
    """The geometry is applied once per path, but the file can change under a
    running gateway (re-signed): motion asks again every time."""
    manifest = tmp_path / "ROBOT.md"
    manifest.write_text(Path(M).read_text())
    state, proto = _ready_arm()
    actuator = SOArm101Actuator(protocol=proto)
    actuator._apply_manifest(str(manifest))
    broken = Path(M).read_text().replace("metadata:\n", "metadata:\n\t", 1)
    manifest.write_text(broken)
    os.utime(manifest, (1, 1))                            # a different mtime, whatever the clock
    outcome = actuator.execute(envelope={"tool_name": "arm.home", "tool_args": {}},
                               manifest_path=manifest, tier="actuate", config={})
    assert outcome.telemetry.get("deny") == "manifest_unreadable", outcome
    proto.set_position.assert_not_called()


def test_reads_still_answer_from_a_manifest_that_does_not_parse(tmp_path):
    manifest = tmp_path / "ROBOT.md"
    manifest.write_text("---\nmetadata:\n\trobot_name: tab\n---\n")
    state, proto = _ready_arm()
    actuator = SOArm101Actuator(protocol=proto)
    outcome = actuator.execute(envelope={"tool_name": "arm.state", "tool_args": {}},
                               manifest_path=manifest, tier="read", config={})
    assert outcome.outcome_kind == "executed"


def test_a_reach_point_refused_after_earlier_steps_says_the_arm_moved(monkeypatch, fake_clock):
    """Found in review: the deny said "Nothing was moved." after 45 goal writes."""
    proto = MagicMock(read_position=MagicMock(return_value=2048))
    actuator = SOArm101Actuator(protocol=proto)
    calls = {"n": 0}

    def step(pose, **kwargs):
        calls["n"] += 1
        if calls["n"] <= 2:
            return {"reached": True, "final_positions": dict(pose), "max_error_rad": 0.0,
                    "motion": {"stopped_by_estop": False}}
        raise DeniedError("path_leaves_workspace", "the tip would come 3 mm from z>=0. Nothing was moved.")

    monkeypatch.setattr(actuator, "_move_checked", step)
    monkeypatch.setattr(actuator, "_settle", lambda pose: None)
    outcome = _reach(actuator, [200.0, 0.0, 120.0])
    assert outcome.outcome_kind == "denied"
    reason = outcome.telemetry["reason"]
    assert "Nothing was moved" not in reason
    assert "did move the arm" in reason


def test_only_the_taught_pose_gets_the_taught_allowance(monkeypatch):
    actuator = SOArm101Actuator(protocol=MagicMock(read_position=MagicMock(return_value=2048)))
    seen: dict = {}

    def record(joint_positions, **kwargs):
        seen[len(seen)] = kwargs.get("taught_goal")
        return {"reached": True, "final_positions": dict(joint_positions), "max_error_rad": 0.0,
                "elapsed_s": 0.0, "motion": {"stopped_by_estop": False}}

    monkeypatch.setattr(actuator, "_move_checked", record)
    for tool in ("arm.home", "arm.reach"):
        actuator.execute(envelope={"tool_name": tool, "tool_args": {}},
                         manifest_path=Path(M), tier="actuate", config={})
    assert seen == {0: True, 1: False}


def test_a_step_off_the_joint_range_cannot_be_converted():
    with pytest.raises(ValueError):
        SOArm101Actuator._ticks_on_path("shoulder_pan", 10.0, 0.0)


def test_settle_returns_once_the_arm_reads_still(fake_clock):
    proto = MagicMock(read_position=MagicMock(return_value=2100))
    actuator = SOArm101Actuator(protocol=proto)
    actuator._settle({"shoulder_pan": 0.0})                 # still, though not on target
    assert proto.read_position.call_count >= 2


def test_arrival_is_polled_until_the_arm_gets_there(fake_clock):
    class _Lagging(_SaggingBus):
        """Reaches each goal only after a few rounds of reads."""

        def __init__(self):
            super().__init__(sag=0)
            self.reads = 0

        def set_position(self, motor_id, ticks):
            super().set_position(motor_id, ticks)
            self.reads = 0

        def read_position(self, motor_id):
            self.reads += 1
            return self.goal[motor_id] if self.reads > 3 * 6 else self.goal[motor_id] - 200

    bus = _Lagging()
    actuator = SOArm101Actuator(protocol=bus)
    result = actuator._move_checked({"shoulder_pan": 0.05}, speed=1.0, timeout_s=5.0)
    assert result["reached"] is True


def test_releasing_a_handle_is_best_effort():
    class _Gone:
        is_open = False

        def close(self):
            raise OSError("already gone")

    act_mod._release_serial(None)
    act_mod._release_serial(_Gone())                        # neither raises


# --------------------------------------------------------------------------- #
# Found by the third review
# --------------------------------------------------------------------------- #

class _DeafRegisterBus(_SaggingBus):
    """Goal_Position reads fail: a servo that never answers them."""

    def read_goal_position(self, motor_id):
        raise OSError("no reply")


def test_a_repeated_stop_with_a_failing_register_never_jumps_to_a_lost_goal():
    """With the register unreadable, a repeated stop after a 350-tick reset
    re-sent the old goal: the tip rose 250 mm at 1.1 m/s, out of the box."""
    bus = _DeafRegisterBus()
    actuator = SOArm101Actuator(protocol=bus)
    assert _estop(actuator).success
    lift = config.JOINTS["shoulder_lift"]["motor_id"]
    first = bus.goal[lift]
    bus.goal[lift] = first - 350                 # the reset; the joint reads 362 ticks from the hold
    bus.writes.clear()
    assert _estop(actuator).success              # the stop itself does not fail on the read
    assert (lift, first) not in bus.writes
    assert (lift, first - 350 - bus.sag) in bus.writes      # held where it reads


def test_a_repeated_stop_with_a_failing_register_keeps_a_hold_within_the_window():
    bus = _DeafRegisterBus(sag=12)
    actuator = SOArm101Actuator(protocol=bus)
    _estop(actuator)
    lift = config.JOINTS["shoulder_lift"]["motor_id"]
    first = bus.goal[lift]
    for _ in range(20):
        _estop(actuator)
    assert bus.goal[lift] == first               # no ratchet within the window


def test_a_move_with_a_failing_register_moves_nothing(fake_clock):
    bus = _DeafRegisterBus()
    actuator = SOArm101Actuator(protocol=bus)
    outcome = actuator.execute(envelope={"tool_name": "move",
                                         "tool_args": {"joint_positions": {"shoulder_pan": 0.1}}},
                               manifest_path="", tier="actuate", config={})
    assert outcome.outcome_kind == "error"
    assert bus.writes == []


_Z = "      z:\n        - 0\n        - 250\n"


@pytest.mark.parametrize("broken", [
    (_Z, "      z:\n        - 0\n"),
    (_Z, "      z: '0..250'\n"),
    (_Z, "      z:\n        - 250\n        - 0\n"),
    (_Z, "      z:\n        - .nan\n        - 250\n"),
    (_Z, "      zz:\n        - 0\n        - 250\n"),
    ("  workspace:\n", "  workspce:\n"),
    ("    bounds_mm:\n", "    bounds:\n"),
])
def test_a_workspace_declared_wrongly_refuses_motion(tmp_path, fake_clock, broken):
    """Found in the third review: a manifest that parses but declares its
    workspace wrongly read as "no limits", and a joint move took the tip 111 to
    302 mm below the floor."""
    old, new = broken
    text = Path(M).read_text()
    assert old in text, old
    manifest = tmp_path / "ROBOT.md"
    manifest.write_text(text.replace(old, new, 1))
    state, proto = _ready_arm()
    actuator = SOArm101Actuator(protocol=proto)
    outcome = actuator.execute(envelope={"tool_name": "arm.home", "tool_args": {}},
                               manifest_path=manifest, tier="actuate", config={})
    assert outcome.telemetry.get("deny") == "manifest_unreadable", outcome
    proto.set_position.assert_not_called()


def test_the_fixture_workspace_is_declared_correctly():
    from so_arm101_actuator import kinematics as kin

    assert kin.geometry_problem(M) is None


def test_a_line_planned_after_the_manifest_broke_is_refused(tmp_path, fake_clock):
    """The door checks once per request; reach_point plans up to 25 lines."""
    manifest = tmp_path / "ROBOT.md"
    manifest.write_text(Path(M).read_text())
    state, proto = _ready_arm()
    actuator = SOArm101Actuator(protocol=proto)
    actuator._apply_manifest(str(manifest))
    manifest.write_text(Path(M).read_text().replace(_Z, "      z:\n        - 0\n", 1))
    os.utime(manifest, (2, 2))
    with pytest.raises(DeniedError) as exc:
        actuator._move_checked({"shoulder_pan": 0.1}, speed=1.0, timeout_s=5.0)
    assert exc.value.code == "manifest_unreadable"
    proto.set_position.assert_not_called()


def test_a_manifest_re_signed_in_place_is_reloaded(tmp_path):
    """Found in the third review: the joint zeros stayed as first read while the
    box and chain moved on, so motion was checked on a model 26 degrees off."""
    text = Path(M).read_text()
    marker = "zero_pose_steps: "
    assert marker in text
    manifest = tmp_path / "ROBOT.md"
    manifest.write_text(text)
    actuator = SOArm101Actuator(protocol=MagicMock())
    actuator._apply_manifest(str(manifest))
    before = {j: spec["tick_at_zero_rad"] for j, spec in config.JOINTS.items()}
    assert marker + "2230" in text                        # shoulder_lift's zero
    manifest.write_text(text.replace(marker + "2230", marker + "2530", 1))
    os.utime(manifest, (3, 3))
    actuator._apply_manifest(str(manifest))
    after = {j: spec["tick_at_zero_rad"] for j, spec in config.JOINTS.items()}
    assert after != before


def test_only_the_stop_that_latches_goes_first(monkeypatch):
    """Found in the third review: with every stop given priority, four looping
    read-tier stop clients locked out every arm.estop.clear and state read."""
    seen: list[bool] = []
    real = act_mod._bus

    def spy(timeout_s, *, stop=False):
        seen.append(stop)
        return real(timeout_s, stop=stop)

    monkeypatch.setattr(act_mod, "_bus", spy)
    actuator = SOArm101Actuator(protocol=_SaggingBus())
    _estop(actuator)
    _estop(actuator)
    _estop(actuator)
    assert seen == [True, False, False]


@pytest.mark.parametrize("raw", ["many", "-1", "1001", "2.5"])
def test_an_adoption_window_override_that_is_not_in_range_is_refused(monkeypatch, raw):
    monkeypatch.setenv("SO_ARM101_ADOPT_REFERENCE_TICKS", raw)
    with pytest.raises(ValueError, match="SO_ARM101_ADOPT_REFERENCE_TICKS"):
        act_mod.resolve_adopt_ticks()


def test_the_adoption_window_can_be_widened_for_a_heavier_arm(monkeypatch):
    """At five times bob's sag (112 ticks) each fresh choice lowers the arm; a
    window of 150 keeps the goal instead."""
    monkeypatch.setenv("SO_ARM101_ADOPT_REFERENCE_TICKS", "150")
    bus = _SaggingBus(sag=112)
    actuator = SOArm101Actuator(protocol=bus)
    lift = config.JOINTS["shoulder_lift"]["motor_id"]
    _estop(actuator)
    assert bus.goal[lift] == 2048
