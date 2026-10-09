"""Regression tests for what the EV-03 hostile-model test found (8 October 2026).

Each test names the finding it pins down. The software-in-the-loop run that
found them drove this driver through the real robot-md-gateway with a fuzzer
in the model's seat; these tests reproduce each one with a mocked bus.
"""

from __future__ import annotations

import errno
import math
import os
import subprocess
import sys
import threading
import time
from pathlib import Path
from unittest.mock import MagicMock

import pytest

from so_arm101_actuator import actuator as actuator_module
from so_arm101_actuator import config, motion
from so_arm101_actuator.actuator import SOArm101Actuator
from tests.conftest import FIXTURE_MANIFEST, manifest_with

M = FIXTURE_MANIFEST

#: The fixture's taught `ready` pose, in ticks by motor id.
READY = {1: 2048, 2: 1800, 3: 2300, 4: 2048, 5: 2048, 6: 1700}

#: Wide enough for tool-down targets (see test_move_to.WIDE_RANGES).
WIDE_RANGES = ('{"shoulder_lift": [-1.45, 1.00], "elbow_flex": [-0.19, 1.50],'
               ' "wrist_flex": [-0.93, 1.50]}')


def _bus(start: dict[int, int] | None = None, *, sag: dict[int, int] | None = None):
    """A servo bus that remembers goals. ``sag`` makes a motor settle that many
    ticks below its goal, as a gravity-loaded joint does."""
    proto = MagicMock()
    goal = dict(start or READY)
    sag = sag or {}
    proto.set_position.side_effect = lambda motor_id, ticks: goal.__setitem__(motor_id, ticks)
    proto.read_position.side_effect = lambda motor_id: goal.get(motor_id, 2048) - sag.get(motor_id, 0)
    proto.read_temperature.return_value = 30
    return proto, goal


def _actuator(proto, manifest: str = M) -> SOArm101Actuator:
    actuator = SOArm101Actuator(protocol=proto)
    actuator._apply_manifest(manifest)
    return actuator


def _invoke(actuator, tool_name, tool_args=None, *, tier="actuate", scope=None, manifest=M):
    envelope = {"tool_name": tool_name, "tool_args": tool_args or {}}
    if scope is not None:
        envelope["scope"] = scope
    return actuator.execute(envelope=envelope, manifest_path=Path(manifest), tier=tier, config={})


# --------------------------------------------------------------------------- #
# Finding: an envelope whose scope says OBSERVE still executed arm.move_to
# --------------------------------------------------------------------------- #

@pytest.mark.parametrize("tool,args", [
    ("arm.move_to", {"x_mm": 150.0, "y_mm": 0.0, "z_mm": 50.0}),
    ("arm.home", {}),
    ("arm.reach_point", {"target_mm": [200.0, 0.0, 100.0]}),
])
def test_a_motion_tool_under_an_observe_scope_is_refused(tool, args, monkeypatch, fake_clock):
    monkeypatch.setenv("SO_ARM101_SAFE_RANGE_RAD", WIDE_RANGES)
    proto, _ = _bus()
    outcome = _invoke(_actuator(proto), tool, args, scope="OBSERVE")

    assert outcome.outcome_kind == "denied"
    assert outcome.telemetry["deny"] == "scope_mismatch"
    proto.set_position.assert_not_called()


def test_the_same_move_under_manipulate_executes(monkeypatch, fake_clock):
    monkeypatch.setenv("SO_ARM101_SAFE_RANGE_RAD", WIDE_RANGES)
    proto, _ = _bus()
    outcome = _invoke(_actuator(proto), "arm.move_to",
                      {"x_mm": 150.0, "y_mm": 0.0, "z_mm": 50.0}, scope="MANIPULATE")
    assert outcome.outcome_kind == "executed", outcome.error_message


def test_no_scope_can_refuse_a_stop(fake_clock):
    proto, _ = _bus()
    outcome = _invoke(_actuator(proto), "arm.estop", tier="read", scope="OBSERVE")
    assert outcome.outcome_kind == "executed"
    assert outcome.telemetry["estopped"] is True


# --------------------------------------------------------------------------- #
# Finding: only the endpoints were checked; the path between them was not
# --------------------------------------------------------------------------- #

def test_a_path_that_swings_out_of_the_workspace_is_refused_before_moving(
        monkeypatch, tmp_path, fake_clock):
    """From the ready pose the joint-space line to this target swings the tip
    forward ~5 mm before it comes back. With the x face 2.6 mm in front of the
    tip, both ends are inside and the path is not."""
    monkeypatch.setenv("SO_ARM101_SAFE_RANGE_RAD", WIDE_RANGES)
    tight = manifest_with(tmp_path, workspace={"x": [-200, 335], "y": [-340, 340],
                                               "z": [0, 250]})
    proto, _ = _bus()
    actuator = _actuator(proto, tight)
    outcome = _invoke(actuator, "arm.move_to", {"x_mm": 191.2, "y_mm": -89.2, "z_mm": 18.0},
                      manifest=tight)

    assert outcome.outcome_kind == "denied", outcome
    assert outcome.telemetry["deny"] == "path_leaves_workspace"
    assert "outside the declared workspace" in outcome.telemetry["reason"]
    proto.set_position.assert_not_called()


def test_check_line_proves_the_whole_line_not_just_samples():
    """The bound between samples is what makes the check cover the path: with
    the lever-arm bound, no point between two checked samples can be farther
    than the sample spacing from them."""
    config.apply_manifest_calibration(M)
    chain = motion.Chain(M)
    box = motion.workspace_box(M)
    start = {j: config.ticks_to_rad(j, t) for j, t in zip(motion.ARM_JOINTS, [2048, 1800, 2300,
                                                                             2048, 2048])}
    end = dict(start, shoulder_pan=start["shoulder_pan"] - 0.5,
               wrist_flex=start["wrist_flex"] + 1.4)
    check = motion.check_line(start, end, chain=chain, box=box)
    # Brute force at 0.01 mm-equivalent resolution agrees with the verdict.
    worst = min(motion.clearance_mm(chain.tip_mm(
        {j: start[j] + (end[j] - start[j]) * k / 20000 for j in start}), box)
        for k in range(20001))
    assert check.ok == (worst >= 0)
    assert check.min_clearance_mm <= worst + 1e-6


def test_the_lever_bound_holds_in_every_configuration():
    import random

    chain = motion.Chain(M)
    lever = chain.lever_mm()
    rng = random.Random(7)
    for _ in range(5000):
        q = {j: rng.uniform(-2.0, 2.0) for j in motion.ARM_JOINTS}
        d = {j: rng.uniform(-0.2, 0.2) for j in motion.ARM_JOINTS}
        moved = math.dist(chain.tip_mm(q), chain.tip_mm({j: q[j] + d[j] for j in q}))
        assert moved <= motion.tip_bound_mm(d, lever) + 1e-9


def test_a_start_outside_is_allowed_a_path_that_never_gets_worse():
    """An arm already past a face (a taught pose on the edge, a joint that
    settled past it) must be able to come back in, and only by getting better."""
    config.apply_manifest_calibration(M)
    chain = motion.Chain(M)
    box = motion.workspace_box(M)
    stretched = {j: config.ticks_to_rad(j, 2048) for j in motion.ARM_JOINTS}  # 13 mm past x
    assert motion.clearance_mm(chain.tip_mm(stretched), box) < 0
    back = dict(stretched, shoulder_lift=stretched["shoulder_lift"] - 0.3)
    assert motion.check_line(stretched, back, chain=chain, box=box).ok
    further = dict(stretched, shoulder_lift=stretched["shoulder_lift"] + 0.1)
    assert not motion.check_line(stretched, further, chain=chain, box=box).ok


def test_the_margin_keeps_the_commanded_path_inside_by_that_much(monkeypatch, fake_clock):
    monkeypatch.setenv("SO_ARM101_SAFE_RANGE_RAD", WIDE_RANGES)
    monkeypatch.setenv(motion.MARGIN_ENV, "20")
    proto, _ = _bus()
    outcome = _invoke(_actuator(proto), "arm.move_to", {"x_mm": 200.0, "y_mm": 0.0, "z_mm": 10.0})
    assert outcome.outcome_kind == "denied"
    assert outcome.telemetry["deny"] == "path_leaves_workspace"
    assert "margin" in outcome.telemetry["reason"]
    proto.set_position.assert_not_called()


@pytest.mark.parametrize("raw", ["-1", "nan", "inf", "five"])
def test_a_bad_margin_is_an_error_not_zero(monkeypatch, raw):
    monkeypatch.setenv(motion.MARGIN_ENV, raw)
    with pytest.raises(ValueError):
        motion.margin_mm()


# --------------------------------------------------------------------------- #
# Finding: speed was not bounded (one direct command = full servo slew)
# --------------------------------------------------------------------------- #

@pytest.mark.parametrize("declared,expected", [
    ({"max_joint_velocity_dps": 90}, 90.0),
    ({}, motion.DEFAULT_MAX_JOINT_DPS),
    ({"max_joint_velocity_dps": float("nan")}, motion.DEFAULT_MAX_JOINT_DPS),
    ({"max_joint_velocity_dps": -5}, motion.DEFAULT_MAX_JOINT_DPS),
    ({"max_joint_velocity_dps": True}, motion.DEFAULT_MAX_JOINT_DPS),
])
def test_the_joint_rate_comes_from_the_manifest_or_a_conservative_default(
        tmp_path, declared, expected):
    path = manifest_with(tmp_path, safety={**declared,
                                           "estop": {"software": True, "response_ms": 100}})
    limits = motion.motion_limits(path)
    assert math.degrees(limits.joint_rad_s) == pytest.approx(expected)


def test_a_declared_tool_speed_bounds_every_step(tmp_path):
    path = manifest_with(tmp_path, safety={"max_joint_velocity_dps": 180,
                                           "max_linear_velocity_ms": 0.1,
                                           "estop": {"software": True, "response_ms": 100}})
    limits = motion.motion_limits(path)
    assert limits.tool_mm_s == pytest.approx(100.0)
    chain = motion.Chain(path)
    lever = chain.lever_mm()
    start = {j: 0.0 for j in motion.ARM_JOINTS}
    end = dict(start, shoulder_pan=1.0, shoulder_lift=-0.5)
    plan = motion.plan_line(start, end, speed=1.0, limits=limits, lever=lever)
    previous = start
    for step in plan.steps:
        delta = {j: step[j] - previous[j] for j in step}
        assert motion.tip_bound_mm(delta, lever) <= 100.0 * plan.period_s + 1e-6
        previous = step


# --------------------------------------------------------------------------- #
# Finding: arm.reach_point checked reach, not the declared workspace; and its
# first call on a fresh gateway was an HTTP 500 (the port was never opened)
# --------------------------------------------------------------------------- #

@pytest.mark.parametrize("target", [[250.0, 0.0, -60.0], [300.0, 0.0, -100.0],
                                    [120.0, -200.0, -80.0], [200.0, 0.0, math.nan]])
def test_reach_point_refuses_a_target_outside_the_declared_workspace(target, fake_clock):
    proto, _ = _bus()
    outcome = _invoke(_actuator(proto), "arm.reach_point", {"target_mm": target})
    assert outcome.outcome_kind == "denied"
    assert outcome.telemetry["deny"] in {"out_of_workspace", "bad_args"}
    proto.set_position.assert_not_called()


def test_reach_point_opens_the_bus_itself_on_a_fresh_actuator(monkeypatch, fake_clock):
    proto, _ = _bus()
    opened = []

    def fake_open(port, baud, timeout=0.1):
        opened.append(port)
        return MagicMock()

    monkeypatch.setattr(actuator_module, "_open_serial", fake_open)
    monkeypatch.setattr("so_arm101_actuator.protocol.SCSProtocol", lambda serial: proto)
    actuator = SOArm101Actuator()          # no protocol: the gateway entry-point path
    actuator._apply_manifest(M)
    outcome = _invoke(actuator, "arm.reach_point", {"target_mm": [250.0, 30.0, 150.0]})

    assert opened == ["/dev/ttyACM0"]
    assert "AttributeError" not in (outcome.error_message or "")


# --------------------------------------------------------------------------- #
# Finding: repeated arm.estop ratcheted a sagging arm downward
# --------------------------------------------------------------------------- #

def test_a_repeated_stop_resends_the_first_hold_instead_of_reanchoring(fake_clock):
    """shoulder_lift settles 10 ticks below whatever goal it is given. Each
    stop used to send the reading as the new goal, so each one sank it 10 more."""
    proto, goal = _bus(sag={2: 10})
    actuator = _actuator(proto)

    first = _invoke(actuator, "arm.estop", tier="read")
    held_after_first = dict(goal)
    for _ in range(50):
        again = _invoke(actuator, "arm.estop", tier="read")
        assert again.telemetry["hold_reasserted"] is True

    assert first.telemetry["hold_reasserted"] is False
    assert goal == held_after_first            # 50 more stops, no further anchor
    assert goal[2] == READY[2] - 10            # the one unavoidable sag step


def test_clearing_the_latch_lets_the_next_stop_take_a_fresh_hold(fake_clock):
    proto, goal = _bus(sag={2: 10})
    actuator = _actuator(proto)
    _invoke(actuator, "arm.estop", tier="read")
    assert _invoke(actuator, "arm.estop.clear", tier="commission").outcome_kind == "executed"
    second = _invoke(actuator, "arm.estop", tier="read")
    assert second.telemetry["hold_reasserted"] is False
    assert goal[2] == READY[2] - 20


# --------------------------------------------------------------------------- #
# Finding (review): a stop waited for the move it was meant to stop
# --------------------------------------------------------------------------- #

def test_a_stop_interrupts_a_move_in_progress(monkeypatch):
    """The move holds the bus lock for its whole duration. The stop used to
    queue behind it and take effect only after the motion had finished."""
    monkeypatch.setenv("SO_ARM101_SAFE_RANGE_RAD", WIDE_RANGES)
    proto, goal = _bus()
    actuator = _actuator(proto)
    result: dict = {}

    def slow_move():
        result["outcome"] = _invoke(actuator, "arm.move_to",
                                    {"x_mm": 150.0, "y_mm": 0.0, "z_mm": 50.0, "speed": 0.2})

    mover = threading.Thread(target=slow_move)
    mover.start()
    time.sleep(0.3)                        # the move is under way, a few seconds from done
    writes_before = proto.set_position.call_count
    assert writes_before > 0 and mover.is_alive()

    started = time.monotonic()
    stop = _invoke(actuator, "arm.estop", tier="read")
    stop_took = time.monotonic() - started
    mover.join(timeout=5)

    assert stop.outcome_kind == "executed"
    assert stop_took < 0.1, f"the stop waited {stop_took:.2f} s for the move"
    assert not mover.is_alive()
    assert result["outcome"].telemetry.get("stopped") is True
    # Nothing the move wrote came after the hold.
    held = dict(goal)
    time.sleep(0.1)
    assert goal == held


def test_motion_is_refused_while_the_stop_is_latched(monkeypatch, fake_clock):
    monkeypatch.setenv("SO_ARM101_SAFE_RANGE_RAD", WIDE_RANGES)
    proto, _ = _bus()
    actuator = _actuator(proto)
    _invoke(actuator, "arm.estop", tier="read")
    writes = proto.set_position.call_count
    outcome = _invoke(actuator, "arm.move_to", {"x_mm": 150.0, "y_mm": 0.0, "z_mm": 50.0})
    assert outcome.telemetry["deny"] == "estop_latched"
    assert proto.set_position.call_count == writes


# --------------------------------------------------------------------------- #
# Finding: a second process could open the servo bus and drive the arm
# --------------------------------------------------------------------------- #

@pytest.mark.skipif(not sys.platform.startswith("linux"), reason="Linux pty and TIOCGEXCL")
def test_the_servo_bus_is_claimed_exclusively(tmp_path):
    import fcntl
    import struct

    import serial

    master, slave = os.openpty()
    path = os.ttyname(slave)
    os.close(slave)
    handle = actuator_module._open_serial(path, 1_000_000)
    try:
        # TIOCEXCL is set on the line (TIOCGEXCL reads it back on Linux)...
        flag = fcntl.ioctl(handle.fileno(), 0x80045440, struct.pack("i", 0))
        assert struct.unpack("i", flag)[0] == 1
        # ...a second exclusive opener is refused (flock)...
        other = serial.Serial()
        other.port = path
        other.exclusive = True
        with pytest.raises(serial.SerialException):
            other.open()
        # ...and so is a plain open by any non-root process (EBUSY), which is
        # what LeRobot does. Run that open as `nobody` when the suite is root.
        probe = ("import os, sys\ntry:\n    os.close(os.open(sys.argv[1], os.O_RDWR | os.O_NOCTTY))\n"
                 "    print('opened')\nexcept OSError as e:\n    print(e.errno)\n")
        if os.geteuid() == 0:
            os.chmod(path, 0o666)
            # The base interpreter, not a venv's: `nobody` may not reach the venv.
            python = getattr(sys, "_base_executable", sys.executable)
            out = subprocess.run([python, "-I", "-c", probe, path], user=65534, cwd="/",
                                 capture_output=True, text=True, timeout=20).stdout.strip()
        else:
            out = subprocess.run([sys.executable, "-c", probe, path],
                                 capture_output=True, text=True, timeout=20).stdout.strip()
        assert out == str(errno.EBUSY)
    finally:
        handle.close()
        os.close(master)


def test_failing_to_claim_the_bus_fails_the_open(tmp_path):
    """A handle that cannot take TIOCEXCL (here: not a tty at all) is closed and
    the open fails. A bus this driver cannot hold alone is not one it uses."""
    regular = open(tmp_path / "not-a-tty", "w+b")  # noqa: SIM115 - closed by the code under test

    class Handle:
        port = str(tmp_path / "not-a-tty")
        closed = False

        def fileno(self):
            return regular.fileno()

        def close(self):
            Handle.closed = True
            regular.close()

    with pytest.raises(OSError, match="exclusive use"):
        actuator_module._claim_tty(Handle())
    assert Handle.closed


def test_a_bus_error_closes_the_handle_before_dropping_it(fake_clock):
    """The handle holds TIOCEXCL: dropping it without closing would make the
    re-open after a USB hiccup fail with EBUSY."""
    proto, _ = _bus()
    proto.read_position.side_effect = OSError("link down")
    proto._serial = MagicMock()
    actuator = _actuator(proto)
    outcome = _invoke(actuator, "arm.state", tier="read")
    assert outcome.outcome_kind == "error"
    assert "link down" in outcome.error_message
    proto._serial.close.assert_called_once()
    assert actuator._protocol is None


def test_arm_reach_is_paced_too(fake_clock):
    proto, goal = _bus()
    actuator = _actuator(proto)
    outcome = _invoke(actuator, "arm.reach")
    assert outcome.outcome_kind == "executed", outcome.error_message
    assert proto.set_position.call_count > 6          # several paced steps
    assert _invoke(actuator, "arm.reach", {"target": "elsewhere"}).outcome_kind == "error"


@pytest.mark.parametrize("args", [{"target_mm": [1.0, 2.0]}, {"target_mm": "250,0,100"},
                                  {"target_mm": [250.0, "x", 100.0]}, {}])
def test_reach_point_rejects_a_malformed_target_before_the_bus(args, fake_clock):
    proto, _ = _bus()
    outcome = _invoke(_actuator(proto), "arm.reach_point", args)
    assert outcome.outcome_kind in {"error", "denied"}
    proto.set_position.assert_not_called()


def test_an_unexpected_driver_exception_becomes_an_error_outcome(fake_clock):
    proto, _ = _bus()
    proto.read_position.side_effect = RuntimeError("firmware said no")
    outcome = _invoke(_actuator(proto), "arm.state", tier="read")
    assert outcome.outcome_kind == "error"
    assert "RuntimeError" in outcome.error_message
