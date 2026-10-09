"""SO-ARM101 Actuator Protocol implementation. RPN-000000000002."""

from __future__ import annotations

import contextlib
import math
import threading
import time
from pathlib import Path
from typing import TypedDict

from robot_md_gateway.actuator import ActuatorOutcome

from so_arm101_actuator import config
from so_arm101_actuator import config as _config_module
from so_arm101_actuator import kinematics as kin
from so_arm101_actuator import motion
from so_arm101_actuator.errors import (
    DeniedError,
    MotionStopped,
    UnknownJointError,
    OutOfRangeError,
    ActuatorTimeoutError,
)


#: Default `speed` when the caller does not say: the full declared rate
#: (``safety.max_joint_velocity_dps``, and ``max_linear_velocity_ms`` for the
#: tip when declared). Until 0.4.0, 1.0 meant one direct position command,
#: which an STS3215 runs at its own top speed (270 deg/s in the EV-03
#: simulation, against 180 declared); every move is now paced. See motion.py.
DEFAULT_SPEED = 1.0

#: Envelope scopes that may carry a motion tool. A motion tool named in an
#: envelope whose scope says OBSERVE is refused: the signer said "observe",
#: and the scope is part of what was signed.
ACTUATION_SCOPES: frozenset[str] = frozenset({
    "MANIPULATE", "NAVIGATE", "ACTUATE", "EXECUTE", "COMMISSION",
})


class MoveResult(TypedDict):
    reached: bool
    final_positions: dict[str, float]
    elapsed_s: float
    #: Worst per-joint distance from target in the RETURNED snapshot. Present so
    #: a caller can tell "missed by a hair" from "never left the parking spot" —
    #: a bare False says a move failed without saying by how much, which reads as
    #: a broken robot when the arm is a hundredth of a radian off.
    max_error_rad: float


class ActuatorState(TypedDict):
    positions: dict[str, float]
    motor_temps_c: dict[str, float]
    timestamp_s: float


class MoveToResult(TypedDict):
    """Telemetry for one Cartesian move. The four keys an evaluation harness
    reads — ``reached``, ``final_positions``, ``eef_mm``, ``elapsed_s`` — come
    first; the rest exist so a receipt can be argued with rather than believed.
    """

    reached: bool
    final_positions: dict[str, float]
    eef_mm: dict[str, float]
    elapsed_s: float
    #: Worst per-joint miss against the commanded pose, from the same snapshot
    #: `final_positions` reports (see the note in `move`).
    max_error_rad: float
    #: Distance from the tip to the requested point, by forward kinematics on
    #: the pose that was actually reached. `reached` is a JOINT-space verdict;
    #: this is the Cartesian one, and they can disagree.
    error_mm: float
    target_mm: dict[str, float]
    speed: float
    #: Paced steps commanded, one per control period (motion.CONTROL_PERIOD_S).
    waypoints: int
    ik_provider: str
    #: How long the paced part of the move was planned to take.
    paced_s: float
    #: The joint rate the steps were sized for: ``speed`` times the declared limit.
    joint_speed_limit_dps: float
    #: Least clearance from the declared workspace's faces anywhere on the
    #: commanded path (None when the manifest declares no workspace).
    path_min_clearance_mm: float | None


class ArmState(TypedDict):
    """Read-only pose snapshot. `eef_mm` is None only when the manifest
    declares no chain to walk — the joint angles are still reported, because a
    read that refuses to say where the joints are is useless exactly when it is
    most needed.
    """

    joint_positions_rad: dict[str, float]
    eef_mm: dict[str, float] | None
    tool: str | None


def _open_serial(port: str, baud: int, timeout: float = 0.1):
    """Open the servo bus WITHOUT asserting DTR/RTS.

    pyserial raises both on open, which is exactly esptool's
    reset-into-download-mode sequence on an ESP32 native-USB CDC. If a port is
    ever mis-identified — and a LoRa dev board next to a robot arm is the common
    case, not the edge case — opening it should fail harmlessly rather than
    reboot the other device.
    """
    import serial

    handle = serial.Serial()
    handle.port = port
    handle.baudrate = baud
    handle.timeout = timeout
    handle.dtr = False
    handle.rts = False
    # flock(LOCK_EX | LOCK_NB): a second process that also asks for an
    # exclusive lock is refused.
    handle.exclusive = True
    handle.open()
    _claim_tty(handle)
    return handle


def _claim_tty(handle) -> None:  # noqa: ANN001 - a pyserial handle
    """Make this process the only one that can open the servo bus (TIOCEXCL).

    flock is advisory: LeRobot, a terminal or any script that simply opens
    the port never asks for it, and EV-03 showed a second process under the
    same uid writing Goal_Position packets onto the bus while the gateway held
    it open. TIOCEXCL makes every later open(2) of the tty fail with EBUSY
    until this handle is closed (root excepted). Failing to take it fails the
    open: a bus shared with an unknown writer is not one this driver can vouch
    for.
    """
    try:
        import fcntl
        import termios
    except ImportError:  # not POSIX: nothing to claim with
        return
    try:
        fcntl.ioctl(handle.fileno(), termios.TIOCEXCL)
    except OSError as exc:
        with contextlib.suppress(Exception):
            handle.close()
        raise OSError(
            f"could not take exclusive use of {handle.port} (TIOCEXCL): {exc}") from exc


#: One servo bus, one caller at a time. The gateway serves /v1/invoke from a
#: threadpool, so two overlapping requests previously interleaved reads and
#: writes on the same unlocked pyserial handle and BOTH returned HTTP 500
#: (reproduced 6/6). Mutual exclusion has to live here, at the device owner —
#: rate-limiting one client cannot help when a second client exists.
_BUS_LOCK = threading.Lock()

#: ROBOT.md capability names this driver can actually execute. Declared-but-
#: unimplemented capabilities (arm.pick / arm.place need the vision rig and a
#: calibrated gripper) are deliberately absent: allowing one through policy
#: would turn a clean signed DENY into a confusing actuator 500.
IMPLEMENTED_CAPABILITIES: frozenset[str] = frozenset({
    "arm.home", "arm.reach", "status.report", "arm.reach_point",
    "arm.move_to", "arm.state",
    # The stop was written and tested in transport.py long before anything could
    # reach it: it appeared in no capability list, no tier table and no dispatch
    # branch, so the deployed allowlist for this arm had no stop tool of any
    # kind. Declaring it here is what makes it invocable.
    "arm.estop", "arm.estop.clear",
})

#: The honest safety note that travels with the stop, copied verbatim from
#: ``transport.py``'s module docstring. It is stated on the tool and returned in
#: the telemetry the gateway signs, so nobody reads "e-stop" on a receipt and
#: infers a hardware interlock that does not exist here.
ESTOP_SAFETY_NOTE = (
    "SAFETY: ``estop()`` is a best-effort SOFTWARE hold (command each joint to "
    "its current encoder reading so motion stops) - NOT a hardware e-stop. The "
    "SCS bus exposes no torque-off here, and software cannot guarantee the arm "
    "physically stopped."
)

#: Tool descriptions for the two stop tools. Both carry the SAFETY note above,
#: because the place a reader meets the word "e-stop" is the place the caveat
#: has to be.
TOOL_DESCRIPTIONS: dict[str, str] = {
    "arm.estop": (
        "Stop the arm and REFUSE further motion until explicitly cleared. "
        + ESTOP_SAFETY_NOTE
    ),
    "arm.estop.clear": (
        "Release the latched software e-stop so motion can be commanded again. "
        + ESTOP_SAFETY_NOTE
    ),
}

#: Tools that command physical motion, and tools that stop it. Declared rather
#: than inferred from the name, so the gateway can check at startup that an
#: allowlist carrying motion also carries a stop (``allowlist_has_no_stop``).
MOTION_CAPABILITIES: frozenset[str] = frozenset({
    "arm.home", "arm.reach", "arm.reach_point", "arm.move_to",
    "move", "home", "move_to",
})
STOP_CAPABILITIES: frozenset[str] = frozenset({"arm.estop"})


#: Minimum caller tiers per tool, enforced inside ``execute`` as defense in
#: depth. The gateway's tier gate keys off the envelope's self-declared
#: ``scope``, which the caller controls; these bindings key off the TOOL, which
#: the operator controls via the allowlist. ``anon`` appears nowhere: an
#: unauthenticated caller can neither move the arm nor read the servo bus.
REQUIRED_TIERS: dict[str, frozenset[str]] = {
    "arm.home": frozenset({"actuate", "commission"}),
    "arm.reach": frozenset({"actuate", "commission"}),
    "move": frozenset({"actuate", "commission"}),
    "home": frozenset({"actuate", "commission"}),
    "status.report": frozenset({"read", "actuate", "commission"}),
    "read_state": frozenset({"read", "actuate", "commission"}),
    "arm.reach_point": frozenset({"actuate", "commission"}),
    # Cartesian motion is motion: same tier class as arm.reach.
    "arm.move_to": frozenset({"actuate", "commission"}),
    "move_to": frozenset({"actuate", "commission"}),
    # Reading a pose moves nothing: same tier class as status.report.
    "arm.state": frozenset({"read", "actuate", "commission"}),
    "state": frozenset({"read", "actuate", "commission"}),
    # Stopping is the one thing no tier may be refused. The runtime process
    # holds only the read bearer, and a stop it cannot reach is not a stop.
    "arm.estop": frozenset({"read", "actuate", "commission"}),
    # Clearing the latch is the opposite act: it makes the arm movable again,
    # so it sits behind the same bearer as bring-up motion. Read tier can stop
    # this arm and cannot start it.
    "arm.estop.clear": frozenset({"commission"}),
}


def _denied(exc: DeniedError) -> ActuatorOutcome:
    """Turn a refusal into the outcome the gateway signs and returns as a 403.

    The structured form rides in ``telemetry`` as well as in the message. The
    gateway hashes telemetry into the signed receipt either way, so a harness
    reading ``deny``/``reason`` gets a machine-readable refusal that is bound to
    the same signature the human-readable one is.
    """
    return ActuatorOutcome(
        success=False,
        outcome_kind="denied",
        error_message=str(exc),
        telemetry={"deny": exc.code, "reason": exc.detail},
    )


def _parse_move_to_args(tool_args: dict) -> dict:
    """Validate `arm.move_to`'s wire arguments into keyword arguments.

    Strict on purpose. A missing coordinate is not zero, an unknown argument is
    not ignorable (a caller who wrote ``z`` instead of ``z_mm`` means to move
    somewhere, and silently dropping it moves the arm somewhere else), and a
    string "150" is a client that has not decided what its numbers are.
    """
    coords: dict[str, float] = {}
    for name in ("x_mm", "y_mm", "z_mm"):
        if name not in tool_args:
            raise DeniedError(
                "bad_args",
                f"arm.move_to needs x_mm, y_mm and z_mm (millimetres in the arm's "
                f"base frame: z up, x forward); {name} is missing")
        value = tool_args[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise DeniedError(
                "bad_args", f"{name} must be a number, got {value!r}")
        if not math.isfinite(float(value)):
            raise DeniedError("bad_args", f"{name} must be finite, got {value!r}")
        coords[name] = float(value)

    unknown = sorted(set(tool_args) - {"x_mm", "y_mm", "z_mm", "speed"})
    if unknown:
        raise DeniedError(
            "bad_args",
            f"arm.move_to does not take {', '.join(unknown)}; it accepts x_mm, "
            f"y_mm, z_mm and an optional speed")

    # Validated here, before the serial port is opened. A subnormal such as
    # 5e-324 is below the floor and refused; it used to reach ceil(1/speed),
    # overflow, and come back as an HTTP 500.
    speed = motion.validate_speed(tool_args.get("speed", DEFAULT_SPEED))
    return {**coords, "speed": speed}


class SOArm101Actuator:
    """RobotRegistryFoundation/so-arm101-actuator v0.1.0 — RPN-000000000002.

    Constructed with an `SCSProtocol` (or compatible mock for tests). The
    default factory in `from_default_port` opens `/dev/ttyACM0` at 1 Mbps.
    """

    name = "so-arm101"
    description = "SO-ARM101 6-DOF + gripper Actuator Protocol driver. RPN-000000000002."
    config_schema: dict = {}

    capabilities = ("move", "home", "read_state", "move_to", "state",
                    "arm.estop", "arm.estop.clear")

    #: Read off the instance by the gateway's startup invariant. An allowlist
    #: that carries any of `motion_capabilities` and none of `stop_capabilities`
    #: is logged as `allowlist_has_no_stop`.
    motion_capabilities = MOTION_CAPABILITIES
    stop_capabilities = STOP_CAPABILITIES

    def __init__(
        self,
        protocol=None,  # noqa: ANN001 — duck-typed
        *,
        home_pose_rad: dict[str, float] | None = None,
        move_tolerance_rad: float | None = None,
    ) -> None:
        """Create an actuator.

        Args:
            protocol: An ``SCSProtocol`` instance (or compatible mock). When
                ``None`` (the default), the gateway entry-point path, the
                protocol is opened lazily on the first ``execute()`` call via
                ``_ensure_protocol()``.
            home_pose_rad: Optional dict of {joint: rad} to override the
                default home pose (from environment SO_ARM101_HOME_POSE_RAD or
                config.HOME_POSE_RAD). Partial dicts merge with defaults,
                with this kwarg taking precedence per joint.
            move_tolerance_rad: Optional float to override the default move
                tolerance (from environment SO_ARM101_MOVE_TOLERANCE_RAD or
                config.MOVE_TOLERANCE_RAD). Kwarg takes precedence over env.
        """
        # The manifest is authoritative for THIS robot's geometry; the module
        # constants are only a fallback for a bench with no manifest. Without
        # this the gripper's zero is assumed to be 2048 when it is really 1539,
        # which puts every reading outside the joint's own range and gets it
        # excluded from motion as if the hardware were faulty.
        #: True only when the manifest supplied this joint's real zero. The
        #: fallback constant (2048) is wrong for the gripper on this arm, so
        #: motion stays disabled rather than trusting a guess.
        self._gripper_calibrated = False
        self._manifest_applied = None
        try:
            config.apply_manifest_calibration()
            self._gripper_calibrated = config.gripper_geometry_known()
        except Exception:
            # Geometry we cannot read is not a reason to refuse to run; the
            # constants remain a workable fallback.
            pass
        #: A caller's explicit pose outranks anything read from a manifest, and
        #: must survive the re-read that happens when execute() supplies one.
        self._home_pose_override = dict(home_pose_rad) if home_pose_rad else None
        env_pose = config.resolve_home_pose_rad()
        if self._home_pose_override:
            # kwarg merges on top of env-resolved pose (kwarg wins per joint)
            env_pose = {**env_pose, **self._home_pose_override}
        self.home_pose_rad: dict[str, float] = env_pose

        env_tolerance = config.resolve_move_tolerance_rad()
        if move_tolerance_rad is not None:
            self.move_tolerance_rad: float = move_tolerance_rad
        else:
            self.move_tolerance_rad = env_tolerance

        self._protocol = protocol

        #: Set by arm.estop BEFORE it waits for the bus. A move in progress holds
        #: the bus lock until it finishes, so a stop that queued for the lock
        #: used to take effect only after the motion it was meant to stop. Every
        #: motion loop checks this between bus writes and abandons the move.
        self._stop_requested = threading.Event()

    @classmethod
    def from_default_port(cls, port: str = "/dev/ttyACM0", baud: int = 1_000_000) -> "SOArm101Actuator":
        import serial
        from so_arm101_actuator.protocol import SCSProtocol
        ser = _open_serial(port, baud)
        return cls(protocol=SCSProtocol(serial=ser))

    def _ensure_protocol(self, *, port: str = "/dev/ttyACM0", baud: int = 1_000_000) -> None:
        """Open the serial port if no protocol was injected at construction.

        Raises ``IOError`` (subclass of ``OSError``) if the device is
        unavailable; callers catch this and convert it to an error
        ``ActuatorOutcome``.
        """
        if self._protocol is not None:
            return
        import serial
        from so_arm101_actuator.protocol import SCSProtocol
        self._protocol = SCSProtocol(
            serial=_open_serial(port, baud)
        )

    def _check_stop(self) -> None:
        if self._stop_requested.is_set():
            raise MotionStopped(
                "arm.estop arrived during this move; the arm is held where the stop "
                "found it")

    def _pause(self, seconds: float) -> None:
        """Sleep, checking for a stop at least once per control period, so no
        wait in a motion loop can hold a stop back for longer than that."""
        end = time.monotonic() + seconds
        while True:
            self._check_stop()
            remaining = end - time.monotonic()
            if remaining <= 0:
                return
            time.sleep(min(remaining, motion.CONTROL_PERIOD_S))

    @staticmethod
    def _validate_joints(joint_positions: dict[str, float]) -> None:
        for joint in joint_positions:
            if joint not in config.JOINTS:
                raise UnknownJointError(joint)
        for joint, rad in joint_positions.items():
            spec = config.JOINTS[joint]
            if not (spec["min_rad"] <= rad <= spec["max_rad"]):
                raise OutOfRangeError(f"{joint}={rad:.3f} outside [{spec['min_rad']}, {spec['max_rad']}]")

    def move(self, joint_positions: dict[str, float], *, timeout_s: float = 5.0) -> MoveResult:
        """Write each joint's goal once and wait for it to arrive.

        RAW: one Goal_Position per joint, run at the servos' own top speed, with
        no pacing and no workspace check. Nothing the gateway serves calls this
        to start a motion; ``move_paced`` (every tool) calls it only for the
        last, already-paced step. A Python caller who wants the declared limits
        held calls ``move_paced``.
        """
        # Validate before any wire traffic.
        self._validate_joints(joint_positions)
        self._check_stop()

        # Issue commands.
        start = time.monotonic()
        for joint, rad in joint_positions.items():
            self._protocol.set_position(
                motor_id=config.JOINTS[joint]["motor_id"],
                ticks=config.rad_to_ticks(joint, rad),
            )

        # Poll until within tolerance or timeout.
        reached = False
        while time.monotonic() - start < timeout_s:
            self._check_stop()
            current = {j: self._read_joint(j) for j in joint_positions}
            if all(abs(current[j] - joint_positions[j]) <= self.move_tolerance_rad for j in joint_positions):
                reached = True
                break
            time.sleep(0.02)

        # Snapshot final state for *all* commanded joints.
        final = {j: self._read_joint(j) for j in joint_positions}

        # Recompute against the snapshot actually being RETURNED, rather than
        # trusting the loop's verdict.
        #
        # The loop polls, then this reads again. An arm that settled in the gap
        # between the last poll and this read produced a receipt asserting
        # reached=False while final_positions showed every joint on target — the
        # same receipt disagreeing with itself. Observed on real hardware: a
        # wrist_flex move reported failure, and the next command read the joint
        # sitting 0.03 rad from where it had been asked to go, well inside a 0.05
        # tolerance.
        #
        # That matters more than it sounds. These receipts are signed evidence,
        # and a saved capability replaying them would look broken on every run.
        errors = {j: abs(final[j] - joint_positions[j]) for j in joint_positions}
        max_error = max(errors.values()) if errors else 0.0
        reached = max_error <= self.move_tolerance_rad

        return MoveResult(
            reached=reached,
            final_positions=final,
            elapsed_s=time.monotonic() - start,
            max_error_rad=round(max_error, 5),
        )

    def home(self, *, timeout_s: float = 10.0, speed: float = DEFAULT_SPEED) -> MoveResult:
        """Move all joints to the resolved home pose (env/kwarg-overridable),
        paced and path-checked like every other motion."""
        result, _plan, _check = self._move_paced(self.home_pose_rad, speed=speed,
                                                 timeout_s=timeout_s)
        return result

    def move_paced(self, joint_positions: dict[str, float], *, speed: float = DEFAULT_SPEED,
                   timeout_s: float = 5.0) -> MoveResult:
        """Move named joints to target radians at no more than the declared rates,
        along a path checked against the declared workspace before anything moves.

        Raises ``DeniedError`` (``bad_args``, ``too_slow``,
        ``path_leaves_workspace``) with nothing moved, and ``MotionStopped`` if
        ``arm.estop`` arrives part-way.
        """
        result, _plan, _check = self._move_paced(joint_positions, speed=speed,
                                                 timeout_s=timeout_s)
        return result

    def _move_paced(self, target: dict[str, float], *, speed: float = DEFAULT_SPEED,
                    timeout_s: float = 5.0, current: dict[str, float] | None = None,
                    ) -> tuple[MoveResult, motion.Plan, motion.PathCheck | None]:
        """Check the whole path, then walk it one control period at a time.

        ``current`` lets a caller that has just read the joints (``move_to``)
        hand them over instead of reading them twice.
        """
        self._validate_joints(target)
        speed = motion.validate_speed(speed)
        self._check_stop()
        manifest = self._manifest_applied

        joints = sorted(set(target) | set(motion.ARM_JOINTS), key=list(config.JOINTS).index)
        now = dict(current or {})
        for joint in joints:
            if joint not in now and (joint in target or joint in motion.ARM_JOINTS):
                now[joint] = self._read_joint(joint)

        chain: motion.Chain | None
        try:
            chain = motion.Chain(manifest)
        except ValueError:
            chain = None  # no declared geometry: nothing to check a path against
        lever = chain.lever_mm() if chain is not None else {}

        check: motion.PathCheck | None = None
        box = motion.workspace_box(manifest) if chain is not None else None
        if box is not None:
            check = motion.check_line(now, {**now, **target}, chain=chain, box=box,
                                      margin=motion.margin_mm())
            if not check.ok:
                raise DeniedError("path_leaves_workspace", check.detail)

        start_pose = {j: now[j] for j in target}
        plan = motion.plan_line(start_pose, target, speed=speed,
                                limits=motion.motion_limits(manifest), lever=lever)
        started = time.monotonic()
        for index, step in enumerate(plan.steps[:-1], start=1):
            self._check_stop()
            for joint, rad in step.items():
                self._protocol.set_position(motor_id=config.JOINTS[joint]["motor_id"],
                                            ticks=config.rad_to_ticks(joint, rad))
            # Sleep to the end of this step's slot, not for a fixed period, so
            # the bus time does not add up into a slower move than planned (and
            # never into a faster one).
            remaining = started + index * plan.period_s - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
        # The last step is the target itself: move() writes it and waits for the
        # joints to arrive, which is the receipt every caller already reads.
        result = self.move(target, timeout_s=timeout_s)
        return result, plan, check

    def _apply_manifest(self, manifest_path) -> None:
        """Re-read geometry from a specific manifest, at most once per path.

        Idempotent and cheap after the first call. Failure is not fatal: geometry
        we cannot read leaves the generic constants in place, and the gripper
        stays out of motion because its fallback zero is wrong for this arm.
        """
        key = str(manifest_path) if manifest_path else ""
        if not key or key == getattr(self, "_manifest_applied", None):
            return
        try:
            _config_module.apply_manifest_calibration(key)
            self._gripper_calibrated = _config_module.gripper_geometry_known(key)
            # The taught pose is stored in TICKS, so correcting the zeros changes
            # which radians mean that pose. Recompute, keeping any caller override.
            pose = _config_module.resolve_home_pose_rad(key)
            if self._home_pose_override:
                pose = {**pose, **self._home_pose_override}
            self.home_pose_rad = pose
            self._manifest_applied = key
        except Exception:
            pass

    def reach_point(self, target_mm, *, tolerance_mm: float = 5.0,
                    max_iterations: int = 25) -> dict:
        """Move the gripper tip to a point, by measuring rather than solving.

        Each iteration reads where the tip actually is, takes one BOUNDED step
        that shrinks the error, and looks again. No inverse kinematics: the
        analytic solver on this arm constrains the tool axis to vertical, which
        it cannot achieve at tabletop reach, so it solves nothing anywhere in the
        declared workspace.

        Measuring instead of solving also means the loop corrects for a model
        that disagrees with the hardware — and this robot's does, reporting a tip
        20 cm below its own base. A solver would confidently command that error;
        a servo loop converges anyway, because it steers on the encoders.

        Stops early when progress stalls. An arm pressed against something it
        cannot move keeps reporting the same error, and continuing to command
        into it is how a servo is cooked.
        """
        from so_arm101_actuator import kinematics as kin

        target = tuple(float(v) for v in target_mm)
        if not all(math.isfinite(v) for v in target):
            raise DeniedError("bad_args", f"target_mm must be finite, got {list(target_mm)!r}")
        # The declared envelope first, as move_to does. This used to check only
        # that the links could span the point, so a target 100 mm below the
        # declared floor was walked to (EV-03: tip at z = -101.7 mm).
        inside, why = kin.within_workspace(target, self._manifest_applied)
        if not inside:
            raise DeniedError("out_of_workspace", why)
        ok, why = kin.reachable(target, self._manifest_applied)
        if not ok:
            raise OutOfRangeError(why)

        history = []
        best_error = None
        stalled = 0
        warm_started = False

        # Warm start. Damped least squares walks downhill from wherever the arm
        # is, and on this arm the downhill path to a reachable target very often
        # runs into a joint's safe limit and pins there, 10-30 mm short, which
        # then looks exactly like a blocked arm. So first jump to the pose the
        # arm's own geometry says is nearest the target INSIDE the safe envelope
        # (a table lookup), and let the loop close the last few millimetres.
        # Measured 2026-09-09: 0 of 27 easel points reached cold; every target
        # within the envelope reached warm.
        current = {j: self._read_joint(j) for j in config.JOINTS if j != "gripper"}
        _, error_now = kin.reach_step(current, target, manifest_path=self._manifest_applied)
        try:
            warm, predicted = kin.nearest_safe_pose(target, config.SAFE_RANGE_RAD,
                                                    self._manifest_applied)
        except Exception:  # a manifest the table cannot read: servo cold, as before
            warm, predicted = None, None
        # Only for a real jump: a short step from a settled pose converges in one
        # to three iterations, while a warm start may swing the arm to a quite
        # different configuration for the same tip (slow, and hard on the servos).
        WARM_START_MIN_MM = 40.0
        if warm is not None and error_now > WARM_START_MIN_MM and predicted + tolerance_mm < error_now:
            safe = {}
            for joint, value in warm.items():
                lo, hi = config.SAFE_RANGE_RAD.get(joint, (-3.14, 3.14))
                safe[joint] = max(lo, min(hi, value))
            try:
                # Paced and path-checked like every other motion: the jump to a
                # table pose is the longest move this loop makes.
                self._move_paced(safe, timeout_s=3.0)
            except DeniedError:
                pass  # the jump would leave the workspace: servo cold, as before
            else:
                self._settle(safe)
                warm_started = True

        # Static-error compensation. These servos hold a static error under
        # load (measured 2026-09-09: shoulder_lift sits 0.03-0.04 rad from where
        # it was sent and ignores corrections smaller than that), so a geometric
        # step below that size produces no motion at all and the loop parks
        # 10-17 mm short. Add the offset the joint showed on the LAST command
        # (command minus where it settled) to the next one. Not accumulated: an
        # integral wound up over several unanswered steps overshoots by the
        # whole sum the moment the joint does answer (measured: 6 mm -> 16 mm).
        # Bounded, and still clamped to the safe range below, so no joint is
        # ever asked past its measured limits.
        bias = {j: 0.0 for j in kin.REACH_JOINTS}
        BIAS_CAP_RAD = 0.05
        last_command: dict[str, float] | None = None
        pinned: set = set()
        best_pose: dict[str, float] | None = None

        def _give_up(step: int, error: float, current: dict, why: str) -> dict:
            # Walk back to the closest pose this loop measured before reporting
            # the miss, so a caller that reads the arm afterwards finds it at
            # its best, not wherever a diverging step left it.
            if best_pose is not None and best_error is not None and best_error < error - 1.0:
                back = {j: best_pose[j] for j in kin.REACH_JOINTS}
                try:
                    self._move_paced(back, timeout_s=3.0)
                except DeniedError:
                    pass  # no checked way back: stay where the loop stopped
                else:
                    self._settle(back)
                current = {j: self._read_joint(j) for j in config.JOINTS if j != "gripper"}
                error = kin.reach_step(current, target, manifest_path=self._manifest_applied)[1]
            return {"arrived": False, "error_mm": round(error, 2),
                    "iterations": step, "error_history": history,
                    "warm_started": warm_started, "stopped_because": why,
                    "final_positions": current}

        for step in range(max_iterations):
            current = {j: self._read_joint(j) for j in config.JOINTS if j != "gripper"}
            if last_command is not None:
                for joint in bias:
                    # A joint whose command was clamped at its safe limit did not
                    # "fail to answer": it was never asked. Winding its bias up
                    # only pushes the others off course (measured: 12 mm -> 64 mm
                    # in four steps at a sheet corner). No bias for pinned joints.
                    if joint in last_command and joint not in pinned:
                        unanswered = last_command[joint] - current[joint]
                        bias[joint] = max(-BIAS_CAP_RAD, min(BIAS_CAP_RAD, unanswered))
                    else:
                        bias[joint] = 0.0
            proposed, error = kin.reach_step(current, target,
                                             manifest_path=self._manifest_applied)
            history.append(round(error, 2))

            if error <= tolerance_mm:
                return {"arrived": True, "error_mm": round(error, 2),
                        "iterations": step, "error_history": history,
                        "warm_started": warm_started,
                        "final_positions": current}

            if best_error is not None and error > 1.5 * best_error + 5.0:
                return _give_up(step, error, current,
                                "diverging — a joint at its safe limit cannot follow this "
                                "step; the arm is back at the closest point it reached")

            # Stalling is measured RELATIVELY against the best error so far, not
            # step to step: the damped solver overshoots on the way in, and a
            # step that is worse than the last but better than the best is
            # still progress. Gradient descent converges asymptotically, so the
            # improvement per step shrinks as it closes in — an absolute
            # threshold declares a healthy loop "stuck" precisely when it is
            # nearly there. This loop reached 10mm from 325mm and was then
            # called blocked.
            if best_error is not None and (best_error - error) < max(0.05, best_error * 0.02):
                stalled += 1
                if stalled >= 4:
                    return _give_up(step, error, current,
                                    "stopped getting closer — the arm may be blocked, or this "
                                    "point may not be reachable at this approach angle")
            else:
                stalled = 0
                best_error = error
                best_pose = dict(current)

            # Every commanded angle stays inside the joint's own safe range; the
            # servo loop must not be able to walk the arm past its limits.
            safe = {}
            pinned = set()
            for joint, value in proposed.items():
                lo, hi = config.SAFE_RANGE_RAD.get(joint, (-3.14, 3.14))
                wanted = value + bias.get(joint, 0.0)
                safe[joint] = max(lo, min(hi, wanted))
                if safe[joint] != wanted:
                    pinned.add(joint)
            last_command = dict(safe)
            try:
                self._move_paced(safe, timeout_s=3.0)
            except DeniedError as exc:
                return _give_up(step, error, current,
                                f"the next step was refused ({exc.code}): {exc.detail}")
            # move() returns as soon as every joint is within move_tolerance_rad
            # (0.05 rad, 16 mm at the tip) of its target, which is BEFORE a small
            # step has visibly happened. Reading the pose then shows no progress,
            # three such reads count as a stall, and a healthy loop is abandoned
            # 10-30 mm from its target. Observed 2026-09-09: 27 of 27 reachable
            # easel points "stopped getting closer" in under two seconds. So wait
            # for the joints to actually stop moving before measuring again.
            self._settle(safe)

        current = {j: self._read_joint(j) for j in config.JOINTS if j != "gripper"}
        final_error = kin.reach_step(current, target,
                                     manifest_path=self._manifest_applied)[1]
        return _give_up(max_iterations, final_error, current, "ran out of iterations")

    # ----------------------------------------------------------------- #
    # Cartesian control (arm.move_to / arm.state)
    # ----------------------------------------------------------------- #

    def move_to(self, *, x_mm: float, y_mm: float, z_mm: float,
                speed: float = DEFAULT_SPEED,
                timeout_s: float = 6.0) -> MoveToResult:
        """Put the tool tip at (x, y, z) in the arm's BASE frame, tool down.

        Distinct from :meth:`reach_point` on purpose, and both are worth having:

          * ``reach_point`` MEASURES — it steps toward the target reading the
            encoders each time, imposes nothing on tool orientation, and gets
            there at whatever tilt the arm can manage. It is how you touch a
            thing.
          * ``move_to`` SOLVES — one analytic answer, tool vertical, checked
            against the limits before anything moves, and refused outright if
            the answer is not one this arm may hold. It is how an evaluation
            harness asks for a pose it can reason about afterwards.

        Every refusal is a :class:`DeniedError`, which the gateway turns into a
        signed 403. Nothing is clamped: a target that needs a joint past its
        limit is not quietly turned into the nearest legal pose, because that
        pose puts the tip somewhere the caller did not ask for and the receipt
        would say it succeeded.

        The refusals, in the order they are made — cheapest and most decisive
        first, so a bad target costs no bus traffic at all:

        ``out_of_workspace``   outside the manifest's declared envelope
        ``unreachable``        the links do not span it, tool vertical
        ``joint_limits``       solvable, but outside the DECLARED limits
        ``frame_disagreement`` the solve and forward kinematics disagree
        ``unsafe_pose``        inside declared limits, outside this rig's
                               MEASURED envelope (config.SAFE_RANGE_RAD)
        ``unsafe_start``       the arm is parked outside that envelope, so no
                               straight line from here stays inside it
        ``path_leaves_workspace`` the tip's path from here to there leaves the
                               declared envelope, though both ends are inside
        ``too_slow``           at this speed the move would outlast
                               motion.MAX_MOVE_S

        ``speed`` in [0.01, 1] is a fraction of the declared rates
        (``safety.max_joint_velocity_dps``; ``max_linear_velocity_ms`` for the
        tip when declared). The servo bus takes Goal_Position and nothing else —
        there is no velocity register in ``protocol.py`` — and a servo runs to
        a goal at its own top speed, so speed is spelled as time: one small step
        along the straight joint-space line per control period, sized so no
        joint outruns the rate. See motion.py.
        """
        manifest = self._manifest_applied
        target = (float(x_mm), float(y_mm), float(z_mm))
        started = time.monotonic()

        # 1. The declared envelope. Checked first because it is the operator's
        #    statement about where this robot is allowed to be, and it is a
        #    lookup rather than a solve.
        inside, why = kin.within_workspace(target, manifest)
        if not inside:
            raise DeniedError("out_of_workspace", why)

        # 2. Physics. The tip cannot be farther from the base than the links
        #    are long, whatever the manifest's box says.
        ok, why = kin.reachable(target, manifest)
        if not ok:
            raise DeniedError("unreachable", why)

        # 3. The solve itself, from the provider the manifest names.
        solution = kin.solve_tool_down(target, manifest)
        if not solution.ok:
            raise DeniedError(solution.code, solution.detail)

        # 4. This rig's MEASURED envelope, which is tighter than the declared
        #    one and is the check that actually protects the servos. Separate
        #    from the solver's own limit check so the receipt can say which of
        #    the two facts refused: a design limit, or this arm's real stops.
        safe = config.resolve_safe_range_rad()
        for joint, angle in solution.joints.items():
            span = safe.get(joint)
            if span is None:
                continue
            lo, hi = span
            if not (lo <= angle <= hi):
                raise DeniedError(
                    "unsafe_pose",
                    f"the tool-down solution needs {joint} at {angle:+.3f} rad, "
                    f"outside the measured safe range [{lo:+.2f}, {hi:+.2f}] on "
                    f"this arm. Nothing was moved and nothing was clamped: the "
                    f"clamped pose would put the tip somewhere else entirely. "
                    f"An operator who has re-measured this joint can widen it "
                    f"with SO_ARM101_SAFE_RANGE_RAD.")

        # Everything above is arithmetic. Only now does the bus get touched.
        current = {joint: self._read_joint(joint)
                   for joint in config.JOINTS if joint != "gripper"}

        # 5. Both ends of the path must sit inside the safe box, or the straight
        #    line between them does not either. The target end is already
        #    checked; this is the start, and it fails only when the arm is
        #    already parked somewhere it should not be — in which case the way
        #    out is arm.home, not a Cartesian move through the bad region.
        for joint, angle in current.items():
            span = safe.get(joint)
            if span is None:
                continue
            lo, hi = span
            if not (lo <= angle <= hi):
                raise DeniedError(
                    "unsafe_start",
                    f"the arm is parked with {joint} at {angle:+.3f} rad, outside "
                    f"its measured safe range [{lo:+.2f}, {hi:+.2f}] — no straight "
                    f"path from here stays inside the envelope. Run arm.home first.")

        # wrist_roll is HELD, not solved: it turns about the tool axis, which is
        # the axis every remaining link offset lies along, so it moves the tip by
        # exactly nothing. Commanding it to some fresh value would spin whatever
        # is in the gripper for no reason.
        pose = dict(solution.joints)
        pose["wrist_roll"] = current["wrist_roll"]

        # 6. The PATH, not just its ends. The straight joint-space line stays
        #    inside the joint safe box (that box is convex), but the declared
        #    workspace is a box in Cartesian space and the tip's path along that
        #    line is curved: between two allowed targets it went 11-13 mm below
        #    the floor in EV-03. _move_paced checks every point of the line
        #    before anything moves (``path_leaves_workspace``), then walks it no
        #    faster than the declared rates allow.
        result, plan, check = self._move_paced(pose, speed=speed, timeout_s=timeout_s,
                                               current=current)
        final = result["final_positions"]
        tip = kin.tip_position_mm(final, manifest)

        return MoveToResult(
            reached=result["reached"],
            final_positions=final,
            eef_mm={"x": tip[0], "y": tip[1], "z": tip[2]},
            elapsed_s=time.monotonic() - started,
            max_error_rad=result["max_error_rad"],
            error_mm=round(math.dist(tip, target), 2),
            target_mm={"x": target[0], "y": target[1], "z": target[2]},
            speed=float(speed),
            waypoints=len(plan.steps),
            ik_provider=solution.provider,
            paced_s=round(plan.duration_s, 3),
            joint_speed_limit_dps=round(math.degrees(plan.joint_rad_s), 2),
            path_min_clearance_mm=check.min_clearance_mm if check is not None else None,
        )

    def state(self) -> ArmState:
        """Where every joint is, where that puts the tip, and what is on it.

        Read-only in the strongest sense: it issues position reads and nothing
        else, so it is safe to call at any tier that may observe the robot.
        """
        positions = {joint: self._read_joint(joint) for joint in config.JOINTS}
        eef: dict[str, float] | None = None
        try:
            tip = kin.tip_position_mm(positions, self._manifest_applied)
            eef = {"x": tip[0], "y": tip[1], "z": tip[2]}
        except ValueError:
            # No declared chain: we cannot say where the tip is. Say that,
            # rather than refusing to report the joint angles we DO know.
            eef = None
        return ArmState(
            joint_positions_rad=positions,
            eef_mm=eef,
            tool=kin.declared_tool(self._manifest_applied),
        )

    def _home_pose_for_motion(self) -> dict:
        """The taught home pose, including the gripper only if we know its zero.

        The gripper used to be excluded unconditionally, and that was right at
        the time: this module assumed a tick zero of 2048 while the manifest
        declares 1539, so every radian sent to it meant something else. Reading
        the manifest fixed the conversion, so the exclusion now only applies
        when the manifest could not be read at all — in which case the fallback
        constant is wrong again and the old caution still holds.
        """
        pose = dict(self.home_pose_rad)
        if not self._gripper_calibrated:
            pose.pop("gripper", None)
        return pose

    def read_state(self) -> ActuatorState:
        """Read all joint positions and motor temperatures (best-effort).

        Returns a snapshot of the current actuator state with positions in
        radians and temperatures in celsius. If a motor's temperature sensor
        fails to read, that motor is silently omitted from motor_temps_c.
        """
        positions: dict[str, float] = {}
        temps: dict[str, float] = {}
        for joint in config.JOINTS:
            positions[joint] = self._read_joint(joint)
            try:
                temps[joint] = float(self._protocol.read_temperature(
                    motor_id=config.JOINTS[joint]["motor_id"],
                ))
            except (IOError, OSError):
                # Best-effort — joints without a temp sensor are simply omitted.
                pass
        return ActuatorState(
            positions=positions,
            motor_temps_c=temps,
            timestamp_s=time.monotonic(),
        )

    def _settle(self, commanded: dict[str, float], *, still_rad: float = 0.003,
                min_dwell_s: float = 0.35, max_wait_s: float = 1.0, poll_s: float = 0.06) -> None:
        """Block until the commanded joints stop moving (or ``max_wait_s`` passes).

        Waits ``min_dwell_s`` first: a loaded joint creeps at a few milliradians
        per poll, which reads as "still" while it is plainly not there yet
        (measured 2026-09-09: reading after 40 ms and stepping again left the
        loop oscillating; reading after 350 ms converged in four steps). Then
        two consecutive reads within ``still_rad`` of each other count as still.
        Read-only: it never commands anything, so it cannot push a joint anywhere.
        """
        self._pause(min_dwell_s)
        deadline = time.monotonic() + max_wait_s
        last = {j: self._read_joint(j) for j in commanded}
        while time.monotonic() < deadline:
            self._pause(poll_s)
            now = {j: self._read_joint(j) for j in commanded}
            on_target = all(abs(now[j] - commanded[j]) <= still_rad for j in commanded)
            still = all(abs(now[j] - last[j]) <= still_rad for j in commanded)
            if on_target or still:
                return
            last = now

    def _read_joint(self, joint: str) -> float:
        ticks = self._protocol.read_position(motor_id=config.JOINTS[joint]["motor_id"])
        return config.ticks_to_rad(joint, ticks)

    def _estop_transport(self, *, port: str, baud: int):
        """The latch that owns this arm's stop, built once and kept.

        The latch lives in ``SOArm101Transport`` (written, tested, and until now
        unreachable). It is wrapped around THIS actuator instance rather than a
        fresh one, so the two share a single ``_protocol`` and a single serial
        handle - a second transport would mean a second open port and a latch
        that the motion path never consults. Built lazily and cached, because
        the latch state has to survive from the stop to the next request.
        """
        transport = getattr(self, "_transport", None)
        if transport is None:
            from so_arm101_actuator.transport import SOArm101Transport

            transport = SOArm101Transport(actuator=self, port=port, baud=baud)
            self._transport = transport
        return transport

    def execute(
        self,
        *,
        envelope: dict,
        manifest_path: Path,
        tier: str,
        config: dict,
    ) -> ActuatorOutcome:
        """Dispatch an RCAN INVOKE envelope to the appropriate internal method.

        Maps ``envelope["tool_name"]`` to ``move`` / ``home`` / ``read_state``.
        Unknown capabilities and actuator exceptions are both converted to an
        error ``ActuatorOutcome`` so the gateway audit chain always receives a
        structured result.
        """
        # The manifest that authorized THIS call is the authoritative source for
        # this robot's geometry. __init__ also tries, but only via $ROBOT_MANIFEST,
        # and nothing in the deployed services sets it — so relying on __init__
        # alone left the correction inert everywhere except a shell that happened
        # to export it. The gateway always knows the manifest; use it.
        self._apply_manifest(manifest_path)

        tool_name = envelope.get("tool_name")
        tool_args = envelope.get("tool_args", {}) or {}

        # Tier is re-checked HERE against the tool, not against the envelope's
        # self-declared `scope`. The gateway's two gates are independent —
        # check_tier sees only (tier, scope) and check_tool only (tool_name,
        # allowlist) — so an envelope claiming scope="OBSERVE" while naming a
        # motion tool passes both. Binding tier to the TOOL is the check that
        # cannot be talked out of by envelope contents.
        required = REQUIRED_TIERS.get(tool_name)
        if required is not None and tier not in required:
            return ActuatorOutcome(
                success=False,
                outcome_kind="error",
                error_message=(
                    f"tier {tier!r} may not invoke {tool_name!r} "
                    f"(requires one of {sorted(required)})"
                ),
            )

        # And the scope the envelope declares must cover the tool. The tier
        # check above binds the CREDENTIAL to the tool; this binds what the
        # signer said they were doing. An actuate-tier envelope that says
        # scope OBSERVE used to execute arm.move_to (EV-03): the signed scope
        # meant nothing. Stops are exempt: no scope may refuse a stop.
        scope = envelope.get("scope")
        if (tool_name in MOTION_CAPABILITIES and scope is not None
                and str(scope).upper() not in ACTUATION_SCOPES):
            return _denied(DeniedError(
                "scope_mismatch",
                f"{tool_name!r} moves the arm, and this envelope's scope is "
                f"{scope!r}; motion needs one of {sorted(ACTUATION_SCOPES)}"))

        try:
            port = str((config or {}).get("port", "/dev/ttyACM0"))
            baud = int((config or {}).get("baud", 1_000_000))
        except (TypeError, ValueError) as exc:
            return ActuatorOutcome(
                success=False,
                outcome_kind="error",
                error_message=f"bad actuator config: {exc}",
            )

        # The stop, and the refusal that turns it into a stop rather than a
        # pause. Both sit AHEAD of the capability mapping below: a latched arm
        # must not reach the bus at all, and a stop must not depend on any of
        # the geometry the motion tools need to be correct.
        #
        # SAFETY: ``estop()`` is a best-effort SOFTWARE hold (command each joint
        # to its current encoder reading so motion stops) - NOT a hardware
        # e-stop. The SCS bus exposes no torque-off here, and software cannot
        # guarantee the arm physically stopped.
        if tool_name == "arm.estop":
            transport = self._estop_transport(port=port, baud=baud)
            # Latch, and signal any move in progress, BEFORE waiting for the
            # bus. A move holds _BUS_LOCK until it ends, so a stop that queued
            # for the lock first used to take effect only after the motion it
            # was meant to stop. The move checks the signal between bus writes
            # and gives the lock up within one control period.
            transport.request_stop()
            held: dict[str, float] = {}
            try:
                # Same lock every move takes: the hold writes a setpoint per
                # joint, and a read interleaved into that sequence corrupts both.
                with _BUS_LOCK:
                    transport.estop()
                    for joint in _config_module.JOINTS:
                        try:
                            held[joint] = self._read_joint(joint)
                        except Exception:  # noqa: BLE001 - evidence, not control flow
                            pass
            except Exception as exc:  # noqa: BLE001 - the latch is already set
                # estop() latches BEFORE it touches the bus, so a link that is
                # down leaves this arm refusing motion rather than movable. The
                # failure is reported; the stop still stands.
                return ActuatorOutcome(
                    success=False,
                    outcome_kind="error",
                    telemetry={"estopped": True, "safety_note": ESTOP_SAFETY_NOTE},
                    error_message=f"{type(exc).__name__}: {exc}",
                )
            return ActuatorOutcome(
                success=True,
                outcome_kind="executed",
                telemetry={
                    "estopped": True,
                    "held_positions": held,
                    # True: this stop re-sent the hold the first stop of this
                    # latch set, rather than re-anchoring on a sagged reading.
                    "hold_reasserted": bool(transport.last_estop_reasserted),
                    "safety_note": ESTOP_SAFETY_NOTE,
                },
            )
        if tool_name == "arm.estop.clear":
            transport = self._estop_transport(port=port, baud=baud)
            transport.clear_estop()
            return ActuatorOutcome(
                success=True,
                outcome_kind="executed",
                telemetry={
                    "estopped": False,
                    "safety_note": ESTOP_SAFETY_NOTE,
                    "note": "cleared; the arm holds its pose until commanded",
                },
            )
        if tool_name in MOTION_CAPABILITIES:
            # `_estopped` is the transport's own latch - read, never copied. A
            # second copy of this flag is a second thing that can be stale, and
            # the one in transport.py is the one ``set_goal`` already obeys.
            if self._estop_transport(port=port, baud=baud)._estopped:
                return _denied(DeniedError(
                    "estop_latched",
                    f"arm is e-stopped; {tool_name!r} is refused until "
                    "arm.estop.clear is invoked at the commission tier",
                ))

        # ROBOT.md / iOS capability names -> RAP methods. arm.pick / arm.place
        # stay unmapped: they need the vision rig, and the gateway deny-lists
        # them via ROBOT_MD_TOOL_ALLOWLIST so clients get a signed DENY instead.
        #
        # Every motion below goes through move_paced (or move_to, which calls
        # the same primitive): paced at the declared rates, path checked against
        # the declared workspace. arm.home and arm.reach used to be one direct
        # Goal_Position per joint, at the servos' own top speed.
        if tool_name == "arm.home":
            pose = self._home_pose_for_motion()
            tool_name, tool_args = "move_paced", {"joint_positions": pose, "timeout_s": 10.0}
        elif tool_name == "arm.reach_point":
            target = tool_args.get("target_mm")
            if not isinstance(target, (list, tuple)) or len(target) != 3:
                return ActuatorOutcome(
                    success=False, outcome_kind="error",
                    error_message="arm.reach_point needs target_mm as [x, y, z]")
            try:
                tolerance = float(tool_args.get("tolerance_mm", 5.0))
                coords = [float(v) for v in target]
            except (TypeError, ValueError) as exc:
                return _denied(DeniedError("bad_args", f"arm.reach_point: {exc}"))
            outcome = self._on_bus(port, baud, self.reach_point, coords,
                                   tolerance_mm=tolerance)
            if isinstance(outcome, ActuatorOutcome):
                return outcome
            telemetry = outcome
            return ActuatorOutcome(
                success=bool(telemetry.get("arrived")),
                outcome_kind="executed" if telemetry.get("arrived") else "error",
                telemetry=telemetry,
                error_message=(
                    f"{telemetry.get('stopped_because')} "
                    f"(final error {telemetry.get('error_mm')} mm, "
                    f"history {telemetry.get('error_history')}, "
                    f"warm_started={telemetry.get('warm_started')})"))
        elif tool_name == "arm.move_to":
            # Argument shape is settled BEFORE the bus is opened: a malformed
            # request should not cost a serial handle, and it must come back as
            # a deny rather than as a driver crash.
            try:
                tool_args = _parse_move_to_args(tool_args)
            except DeniedError as exc:
                return _denied(exc)
            tool_name = "move_to"
        elif tool_name == "arm.state":
            tool_name, tool_args = "state", {}
        elif tool_name == "status.report":
            tool_name, tool_args = "read_state", {}
        elif tool_name == "arm.reach":
            target = tool_args.get("target", "ready")
            if target != "ready":
                return ActuatorOutcome(
                    success=False,
                    outcome_kind="error",
                    error_message=f"unknown reach target: {target!r}",
                )
            reach_pose = self._home_pose_for_motion()
            reach_pose["shoulder_pan"] = reach_pose.get("shoulder_pan", 0.0) + 0.35
            tool_name, tool_args = "move_paced", {"joint_positions": reach_pose}
        method = {
            # The bare "move" verb is paced and checked too: it is reachable
            # through the gateway whenever an operator allowlists it.
            "move": self.move_paced,
            "move_paced": self.move_paced,
            "home": self.home,
            "read_state": self.read_state,
            "move_to": self.move_to,
            "state": self.state,
        }.get(tool_name)
        if method is None:
            return ActuatorOutcome(
                success=False,
                outcome_kind="error",
                error_message=f"unknown capability: {tool_name!r}",
            )
        result = self._on_bus(port, baud, method, **tool_args)
        if isinstance(result, ActuatorOutcome):
            return result
        return ActuatorOutcome(
            success=True,
            outcome_kind="executed",
            telemetry=dict(result) if isinstance(result, dict) else {"result": result},
        )

    def _on_bus(self, port: str, baud: int, method, *args, **kwargs):  # noqa: ANN001, ANN202
        """Run one driver operation on the bus, or return the outcome it ended in.

        Held across open AND the whole operation: a move polls the bus
        repeatedly until it converges, and a read slipped in between those polls
        corrupts both. Every tool goes through here, arm.reach_point included
        (it used to run without the lock, and without opening the port: its
        first call on a fresh gateway was an HTTP 500, AttributeError).
        """
        try:
            with _BUS_LOCK:
                self._ensure_protocol(port=port, baud=baud)
                return method(*args, **kwargs)
        except DeniedError as exc:
            # A refusal is a decision. It leaves the bus untouched and reaches
            # the caller as a signed 403, not as a 500 that reads like a fault.
            return _denied(exc)
        except MotionStopped as exc:
            # The move was allowed and had started; arm.estop ended it. Not a
            # refusal and not a fault, and the receipt has to say which.
            return ActuatorOutcome(
                success=False,
                outcome_kind="error",
                telemetry={"stopped": True, "reason": "arm.estop"},
                error_message=f"stopped: {exc}",
            )
        except (OSError, IOError) as exc:
            # The serial handle is held for the process lifetime, so a USB
            # replug leaves a dead fd that would fail every later invoke. Close
            # and drop it so the next invoke re-opens by-id instead of needing a
            # restart. Closed explicitly: the handle holds TIOCEXCL, and a
            # dropped-but-open handle would make that re-open fail with EBUSY.
            self._close_protocol()
            return ActuatorOutcome(
                success=False,
                outcome_kind="error",
                error_message=f"{type(exc).__name__}: {exc}",
            )
        except Exception as exc:  # noqa: BLE001 — actuator code is operator-supplied; exceptions become outcomes
            return ActuatorOutcome(
                success=False,
                outcome_kind="error",
                error_message=f"{type(exc).__name__}: {exc}",
            )

    def _close_protocol(self) -> None:
        serial_handle = getattr(self._protocol, "_serial", None)
        if serial_handle is not None:
            with contextlib.suppress(Exception):
                serial_handle.close()
        self._protocol = None
