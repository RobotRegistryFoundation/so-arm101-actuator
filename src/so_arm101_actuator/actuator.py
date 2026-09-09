"""SO-ARM101 Actuator Protocol implementation. RPN-000000000002."""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path
from typing import TypedDict

from robot_md_gateway.actuator import ActuatorOutcome

from so_arm101_actuator import config
from so_arm101_actuator import config as _config_module
from so_arm101_actuator import kinematics as kin
from so_arm101_actuator.errors import (
    DeniedError,
    UnknownJointError,
    OutOfRangeError,
    ActuatorTimeoutError,
)


#: Default `speed` for arm.move_to when the caller does not say. 1.0 is one
#: direct position command, which is exactly what arm.home and arm.reach have
#: always done — a slower default would quietly change how every existing
#: motion on this arm behaves.
DEFAULT_SPEED = 1.0

#: Most interpolation waypoints a single move_to will command. The bus is
#: position-only, so "slower" is spelled as "more, smaller commands"; past a
#: dozen the extra round-trips cost more time than the motion they pace.
MAX_WAYPOINTS = 10

#: How long an intermediate waypoint is given to settle. Deliberately short:
#: waypoints are a pacing device, not targets to converge on, and the final
#: command is the one that has to arrive.
WAYPOINT_TIMEOUT_S = 1.5


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
    waypoints: int
    ik_provider: str


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
    handle.open()
    return handle


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
})


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

    speed = tool_args.get("speed", DEFAULT_SPEED)
    # Validated here as well as in _waypoint_count so a malformed speed is
    # refused before the serial port is opened.
    SOArm101Actuator._waypoint_count(speed)
    return {**coords, "speed": float(speed)}


class SOArm101Actuator:
    """RobotRegistryFoundation/so-arm101-actuator v0.1.0 — RPN-000000000002.

    Constructed with an `SCSProtocol` (or compatible mock for tests). The
    default factory in `from_default_port` opens `/dev/ttyACM0` at 1 Mbps.
    """

    name = "so-arm101"
    description = "SO-ARM101 6-DOF + gripper Actuator Protocol driver. RPN-000000000002."
    config_schema: dict = {}

    capabilities = ("move", "home", "read_state", "move_to", "state")

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

    def move(self, joint_positions: dict[str, float], *, timeout_s: float = 5.0) -> MoveResult:
        # Validate before any wire traffic.
        for joint in joint_positions:
            if joint not in config.JOINTS:
                raise UnknownJointError(joint)
        for joint, rad in joint_positions.items():
            spec = config.JOINTS[joint]
            if not (spec["min_rad"] <= rad <= spec["max_rad"]):
                raise OutOfRangeError(f"{joint}={rad:.3f} outside [{spec['min_rad']}, {spec['max_rad']}]")

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

    def home(self, *, timeout_s: float = 10.0) -> MoveResult:
        """Move all joints to the resolved home pose (env/kwarg-overridable)."""
        return self.move(self.home_pose_rad, timeout_s=timeout_s)

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
            self.move(safe, timeout_s=3.0)
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
                self.move(back, timeout_s=3.0)
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
            self.move(safe, timeout_s=3.0)
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

        ``speed`` (0, 1] paces the motion. The servo bus this driver owns takes
        Goal_Position and nothing else — there is no velocity register in
        ``protocol.py`` and adding one is a hardware change, not a software one
        — so speed is spelled as joint-space interpolation: ``ceil(1/speed)``
        waypoints along the straight line from here to there. 1.0 is one direct
        command, exactly what every other motion on this arm already does.
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

        waypoints = self._waypoint_count(speed)
        for index in range(1, waypoints):
            fraction = index / waypoints
            step = {joint: current[joint] + (pose[joint] - current[joint]) * fraction
                    for joint in pose}
            # Intermediate poses are a pacing device, not destinations — their
            # `reached` verdict is deliberately ignored. Both endpoints are
            # inside the safe box and the box is convex, so every point on this
            # line is too.
            self.move(step, timeout_s=WAYPOINT_TIMEOUT_S)

        result = self.move(pose, timeout_s=timeout_s)
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
            waypoints=waypoints,
            ik_provider=solution.provider,
        )

    @staticmethod
    def _waypoint_count(speed: float) -> int:
        """How many commands to split the path into for this speed.

        Validated, never clamped: a speed of 0 means "do not move", which is not
        a slower move but a different request, and 1.5 is a caller who thinks
        this scale means something it does not.
        """
        if isinstance(speed, bool) or not isinstance(speed, (int, float)):
            raise DeniedError("bad_args", f"speed must be a number, got {speed!r}")
        value = float(speed)
        if not (0.0 < value <= 1.0):
            raise DeniedError(
                "bad_args",
                f"speed must be greater than 0 and at most 1, got {value}")
        if value >= 1.0:
            return 1
        return min(MAX_WAYPOINTS, math.ceil(1.0 / value))

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
        time.sleep(min_dwell_s)
        deadline = time.monotonic() + max_wait_s
        last = {j: self._read_joint(j) for j in commanded}
        while time.monotonic() < deadline:
            time.sleep(poll_s)
            now = {j: self._read_joint(j) for j in commanded}
            on_target = all(abs(now[j] - commanded[j]) <= still_rad for j in commanded)
            still = all(abs(now[j] - last[j]) <= still_rad for j in commanded)
            if on_target or still:
                return
            last = now

    def _read_joint(self, joint: str) -> float:
        ticks = self._protocol.read_position(motor_id=config.JOINTS[joint]["motor_id"])
        return config.ticks_to_rad(joint, ticks)

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

        # ROBOT.md / iOS capability names -> RAP methods. arm.pick / arm.place
        # stay unmapped: they need the vision rig, and the gateway deny-lists
        # them via ROBOT_MD_TOOL_ALLOWLIST so clients get a signed DENY instead.
        if tool_name == "arm.home":
            pose = self._home_pose_for_motion()
            tool_name, tool_args = "move", {"joint_positions": pose}
        elif tool_name == "arm.reach_point":
            target = tool_args.get("target_mm")
            if not target or len(target) != 3:
                return ActuatorOutcome(
                    success=False, outcome_kind="error",
                    error_message="arm.reach_point needs target_mm as [x, y, z]")
            try:
                telemetry = self.reach_point(
                    target,
                    tolerance_mm=float(tool_args.get("tolerance_mm", 5.0)))
            except Exception as exc:
                return ActuatorOutcome(success=False, outcome_kind="error",
                                       error_message=f"{type(exc).__name__}: {exc}")
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
            tool_name, tool_args = "move", {"joint_positions": reach_pose}
        method = {
            "move": self.move,
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
        try:
            port = (config or {}).get("port", "/dev/ttyACM0")
            baud = int((config or {}).get("baud", 1_000_000))
            # Held across open AND the whole operation: a move polls the bus
            # repeatedly until it converges, and a read slipped in between
            # those polls corrupts both.
            with _BUS_LOCK:
                self._ensure_protocol(port=port, baud=baud)
                result = method(**tool_args)
        except DeniedError as exc:
            # A refusal is a decision. It leaves the bus untouched and reaches
            # the caller as a signed 403, not as a 500 that reads like a fault.
            return _denied(exc)
        except (OSError, IOError) as exc:
            # The serial handle is held for the process lifetime, so a USB
            # replug leaves a dead fd that would fail every later invoke. Drop
            # it so the next invoke re-opens by-id instead of needing a restart.
            self._protocol = None
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
        return ActuatorOutcome(
            success=True,
            outcome_kind="executed",
            telemetry=dict(result) if isinstance(result, dict) else {"result": result},
        )
