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
    UnknownJointError,
    OutOfRangeError,
    ActuatorTimeoutError,
)


#: Default `speed` for every motion the gateway can ask for: the fraction of the
#: declared speed limits a move may use (see :func:`kinematics.declared_speed_limits`).
#: 1.0 used to mean "one direct position command", which let the servos run at
#: their own top speed (270 deg/s, the tip at 1.6 m/s, in EV-03's simulation).
#: It now means "as fast as the manifest allows, and no faster".
DEFAULT_SPEED = 1.0

#: How far inside the declared workspace every checked point of a move must stay,
#: in millimetres (or no closer to a face than it started). It covers what the
#: path check cannot see: friction changing the arm's offset from its setpoint
#: while it moves (bob's shoulder settles 2.5 to 27 encoder ticks past its goal
#: depending on which way it arrived, several millimetres at the tip), servo lag,
#: and tick rounding. Override with SO_ARM101_WORKSPACE_MARGIN_MM.
WORKSPACE_MARGIN_MM = 10.0

#: A joint within this many encoder ticks of the goal it was last sent is
#: HOLDING that goal; the difference is gravity sag and friction. Moves are
#: planned from that goal, and a stop re-sends it. Planning from, or holding at,
#: the READING instead lowers the goal by the sag every time: the "sag ratchet"
#: that took a gate out of its envelope in under six seconds on bob, and that
#: walked this arm down 9 cm in 8 s under repeated stops. Bob's measured sag is
#: up to 27 ticks unloaded; 100 (8.8 degrees) leaves room for a payload that
#: triples it. Adopting the goal adds no motion of its own (the servo is already
#: driving to it), so the bound only has to separate holding from a joint that
#: was moved a long way by something else (a power cycle that reset the goal, a
#: collapse), where the reading is the only honest start.
ADOPT_REFERENCE_TICKS = 100

#: How long a request waits for the servo bus before it is refused as busy. A
#: paced move holds the bus for its whole stream, and every request queued
#: behind it holds one of the gateway's worker threads; with no bound, a burst
#: of polls took every thread and the stop could not get one to latch on (a
#: 25 s move ran to the end with a stop sent at 5 s). Commands are refused, not
#: queued, while the arm is busy.
BUS_WAIT_S = 1.0

#: How long the stop waits for the bus to hold the arm once it has latched. A
#: paced move sees the latch and lets go within one pace period.
STOP_BUS_WAIT_S = 5.0

#: The joints the kinematic chain walks; the gripper is not one of them.
ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")


def resolve_workspace_margin_mm() -> float:
    """WORKSPACE_MARGIN_MM, or SO_ARM101_WORKSPACE_MARGIN_MM (a non-negative
    number of millimetres). An unparseable or negative override is refused
    loudly rather than read as zero: a typo must not remove the margin."""
    import os

    raw = os.environ.get("SO_ARM101_WORKSPACE_MARGIN_MM")
    if raw is None:
        return WORKSPACE_MARGIN_MM
    try:
        value = float(raw)
    except ValueError as exc:
        raise ValueError(f"SO_ARM101_WORKSPACE_MARGIN_MM: invalid number {raw!r}") from exc
    if not math.isfinite(value) or value < 0:
        raise ValueError(f"SO_ARM101_WORKSPACE_MARGIN_MM: must be >= 0, got {raw!r}")
    return value


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
    #: Setpoints the move was streamed as, `pace_period_s` apart at most.
    waypoints: int
    ik_provider: str
    #: How the move was paced and checked; see `_motion_telemetry`.
    motion: dict


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
    """Open the servo bus WITHOUT asserting DTR/RTS, and EXCLUSIVELY.

    pyserial raises both on open, which is exactly esptool's
    reset-into-download-mode sequence on an ESP32 native-USB CDC. If a port is
    ever mis-identified — and a LoRa dev board next to a robot arm is the common
    case, not the edge case — opening it should fail harmlessly rather than
    reboot the other device.

    Exclusive, because the gateway is only an enforcement layer if nothing else
    can talk to the servos. On bob, LeRobot drove the arm while the gateway was
    running: any process of the same user could open the port and its
    Goal_Position packets reached the bus. Two locks, because they stop
    different things:

      * pyserial ``exclusive=True`` takes an ``flock``. It refuses a second
        opener that also asks for one, and nothing else.
      * ``TIOCEXCL`` makes the kernel refuse every further ``open(2)`` of this
        tty with ``EBUSY`` for any process without CAP_SYS_ADMIN, LeRobot
        included, which takes no lock at all.

    Both are released when the handle is closed (:func:`_release_serial`).
    Neither binds root, and neither evicts a process that opened the port
    BEFORE this one: that is why the gateway claims the bus at startup
    (:meth:`SOArm101Actuator.claim`).
    """
    import serial

    handle = serial.Serial()
    handle.port = port
    handle.baudrate = baud
    handle.timeout = timeout
    handle.dtr = False
    handle.rts = False
    handle.exclusive = True
    handle.open()
    try:
        _set_tty_exclusive(handle, True)
    except OSError:
        handle.close()
        raise
    return handle


def _set_tty_exclusive(handle, exclusive: bool) -> None:  # noqa: ANN001 — a serial.Serial
    """TIOCEXCL / TIOCNXCL on the handle's descriptor. A no-op where termios has
    neither (Windows), because there is no equivalent to fall back to."""
    try:
        import fcntl
        import termios
    except ImportError:
        return
    request = getattr(termios, "TIOCEXCL" if exclusive else "TIOCNXCL", None)
    if request is not None:
        fcntl.ioctl(handle.fileno(), request)


def _release_serial(handle) -> None:  # noqa: ANN001 — a serial.Serial or None
    """Give the bus back: drop TIOCEXCL, then close (which also drops the flock).
    Best effort and idempotent: releasing a handle that is already gone is fine."""
    if handle is None:
        return
    try:
        if getattr(handle, "is_open", True):
            _set_tty_exclusive(handle, False)
    except Exception:  # noqa: BLE001 — no descriptor (a stand-in handle), or already closed
        pass
    try:
        handle.close()
    except Exception:  # noqa: BLE001 — a close that fails leaves nothing more to do
        pass


#: One servo bus, one caller at a time. The gateway serves /v1/invoke from a
#: threadpool, so two overlapping requests previously interleaved reads and
#: writes on the same unlocked pyserial handle and BOTH returned HTTP 500
#: (reproduced 6/6). Mutual exclusion has to live here, at the device owner —
#: rate-limiting one client cannot help when a second client exists.
_BUS_LOCK = threading.Lock()

#: Guards the lazy construction of the stop's latch (``_estop_transport``): two
#: requests building it at once could latch one copy and consult the other.
_TRANSPORT_LOCK = threading.Lock()


@contextlib.contextmanager
def _bus(timeout_s: float):
    """Hold the servo bus, or refuse as busy after ``timeout_s``."""
    if not _BUS_LOCK.acquire(timeout=timeout_s):
        raise DeniedError(
            "busy",
            "the arm is carrying out another command, and commands are refused rather than "
            "queued while it does. Try again when it has finished.")
    try:
        yield
    finally:
        _BUS_LOCK.release()

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

    speed = tool_args.get("speed", DEFAULT_SPEED)
    # Validated here, before the serial port is opened, so a malformed speed
    # costs nothing; a subnormal one is refused as too slow once the move's
    # length is known.
    return {**coords, "speed": motion.validate_speed(speed)}


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
        #: Encoder ticks last commanded to each joint: the pose this actuator is
        #: holding the arm at, which under load is not where the encoders say the
        #: arm is. See ADOPT_REFERENCE_TICKS.
        self._reference: dict[str, int] = {}
        #: The ticks a latched stop is holding, chosen once per latch so that a
        #: repeated stop re-sends them instead of re-reading a sagged pose.
        self._held: dict[str, int] | None = None
        #: (manifest path, why) when that manifest's geometry could not be read.
        self._manifest_error: tuple[str, str] | None = None

    @classmethod
    def from_default_port(cls, port: str = "/dev/ttyACM0", baud: int = 1_000_000) -> "SOArm101Actuator":
        import serial
        from so_arm101_actuator.protocol import SCSProtocol
        ser = _open_serial(port, baud)
        return cls(protocol=SCSProtocol(serial=ser))

    def claim(self, config: dict | None = None) -> None:  # noqa: A002 — the gateway's name
        """Claim the servo bus now, exclusively, and keep it until :meth:`close`.

        The gateway calls this at startup with the actuator's config (``port``,
        ``baud``) so the bus is held from the moment the service runs, not from
        its first request: a bus nobody holds is a bus anybody can open, and
        the exclusive open cannot evict a process that got there first. Raises
        OSError (EBUSY when someone else holds the port).
        """
        cfg = config or {}
        port = str(cfg.get("port", "/dev/ttyACM0"))
        baud = int(cfg.get("baud", 1_000_000))
        with _BUS_LOCK:
            self._ensure_protocol(port=port, baud=baud)

    def close(self) -> None:
        """Release the servo bus (TIOCNXCL, flock, close). Idempotent."""
        with _BUS_LOCK:
            self._drop_protocol()

    def _drop_protocol(self) -> None:
        """Forget the protocol AND release its handle. Dropping the object
        without closing it would leak the descriptor, and with it the exclusive
        claim, so the re-open after a USB replug would fail with EBUSY against
        this very process."""
        proto, self._protocol = self._protocol, None
        _release_serial(getattr(proto, "_serial", None))

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
            self._write_goal(joint, config.rad_to_ticks(joint, rad))

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

    # ----------------------------------------------------------------- #
    # Checked, paced motion: every motion the gateway can ask for
    # ----------------------------------------------------------------- #

    def _write_goal(self, joint: str, ticks: int) -> None:
        """Command one joint and remember what it was told (the reference)."""
        self._protocol.set_position(motor_id=config.JOINTS[joint]["motor_id"], ticks=ticks)
        self._reference[joint] = int(ticks)

    def _latched(self) -> bool:
        """True while the software stop is latched. Read off the transport that
        owns the latch, never copied: a second copy is a second thing to go stale."""
        transport = getattr(self, "_transport", None)
        return bool(transport is not None and transport._estopped)

    def _planning_start(self, joints) -> tuple[dict[str, float], dict[str, float]]:  # noqa: ANN001
        """Where a move starts, and how far the arm is from it, in radians.

        A joint within ADOPT_REFERENCE_TICKS of the goal it was last sent is
        holding that goal, and the move starts there; the reading's distance
        from it (sag, friction) is the offset the path check adds to predict
        where the arm will really be. Any other joint starts where it reads,
        with no offset.
        """
        start: dict[str, float] = {}
        offset: dict[str, float] = {}
        for joint in joints:
            present = int(self._protocol.read_position(motor_id=config.JOINTS[joint]["motor_id"]))
            held = self._holding_ticks(joint, present)
            start[joint] = config.ticks_to_rad(joint, held)
            offset[joint] = config.ticks_to_rad(joint, present) - start[joint]
        return start, offset

    def _holding_ticks(self, joint: str, present: int) -> int:
        """The goal this joint is holding, if it is holding one; else where it reads.

        The goal is the one this actuator last sent, or, before it has sent any
        (a fresh gateway), the servo's own Goal_Position register.
        """
        ref = self._reference.get(joint)
        if ref is None:
            ref = self._read_goal_register(joint)
        return ref if ref is not None and abs(present - ref) <= ADOPT_REFERENCE_TICKS else present

    def _read_goal_register(self, joint: str) -> int | None:
        """Goal_Position as the servo reports it, or None when it cannot say."""
        reader = getattr(self._protocol, "read_goal_position", None)
        if reader is None:
            return None
        try:
            value = reader(motor_id=config.JOINTS[joint]["motor_id"])
        except Exception:  # noqa: BLE001 — no answer means "do not know", never a fault
            return None
        if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 4095:
            return None
        return value

    def _chain(self) -> kin.Chain | None:
        try:
            return kin.Chain(self._manifest_applied)
        except ValueError:
            return None

    def _move_checked(self, joint_positions: dict[str, float], *, speed: float = DEFAULT_SPEED,
                      timeout_s: float = 5.0, hold: tuple[str, ...] = (),
                      taught_goal: bool = False) -> dict:
        """Move these joints along a checked, paced line, then wait for arrival.

        The path from the pose the arm is being held at to the goal is checked
        against the declared workspace (:func:`motion.check_line`) and paced
        under the declared speed limits (:func:`motion.pace_line`) before
        anything is sent; a refusal is a DeniedError and nothing moves. Joints
        that are not in ``joint_positions`` are not commanded, but where they
        are still counts toward the path check. Joints in ``hold`` are
        commanded to stay where the move starts (their reference, or their
        reading when they are not holding one) and are reported like the rest.

        ``taught_goal`` is for a pose the operator taught (arm.home, arm.reach),
        which may sit inside the workspace margin; see :func:`motion.check_line`.

        Every setpoint is converted to ticks before the first one is sent, so a
        conversion that fails cannot leave a move half-written. A joint that
        starts outside its configured range may pass through values between
        that start and the range on its way back in, and nowhere else.

        The stream stops between setpoints if the software stop latches, and
        the result says so.
        """
        for joint in joint_positions:
            if joint not in config.JOINTS:
                raise UnknownJointError(joint)
        for joint, rad in joint_positions.items():
            spec = config.JOINTS[joint]
            if not isinstance(rad, (int, float)) or isinstance(rad, bool) or not math.isfinite(rad):
                raise OutOfRangeError(f"{joint}={rad!r} is not a finite number")
            if not (spec["min_rad"] <= rad <= spec["max_rad"]):
                raise OutOfRangeError(f"{joint}={rad:.3f} outside [{spec['min_rad']}, {spec['max_rad']}]")
        speed = motion.validate_speed(speed)

        manifest = self._manifest_applied
        joints = list(dict.fromkeys([*ARM_JOINTS, *joint_positions, *hold]))
        start, offset = self._planning_start(joints)
        goal = {**start, **{j: float(v) for j, v in joint_positions.items()}}
        plan = motion.plan(self._chain(), kin.workspace_box(manifest), start, goal,
                           offset=offset, margin_mm=resolve_workspace_margin_mm(), speed=speed,
                           limits=kin.declared_speed_limits(manifest), taught_goal=taught_goal)

        commanded = {**{j: start[j] for j in hold}, **{j: goal[j] for j in joint_positions}}
        ticks_plan = [[(j, self._ticks_on_path(j, setpoint[j], start[j])) for j in commanded]
                      for setpoint in plan.setpoints]
        started = time.monotonic()
        completed = self._stream(ticks_plan, plan.step_s)
        result = self._await_arrival(commanded, started=started, timeout_s=timeout_s,
                                     interrupted=not completed)
        result["motion"] = self._motion_telemetry(plan, offset, completed)
        return result

    @staticmethod
    def _ticks_on_path(joint: str, rad: float, start_rad: float) -> int:
        """Ticks for a setpoint on a checked line. Inside the joint's configured
        range, or between an out-of-range start and that range; nowhere else."""
        spec = config.JOINTS[joint]
        low, high = min(spec["min_rad"], start_rad), max(spec["max_rad"], start_rad)
        if not (low - 1e-9 <= rad <= high + 1e-9):
            raise OutOfRangeError(
                f"{joint}={rad:.3f} outside [{spec['min_rad']}, {spec['max_rad']}]")
        ticks = int(round(spec["tick_at_zero_rad"] + rad * spec["ticks_per_rad"]))
        return max(0, min(4095, ticks))

    def _stream(self, ticks_plan: list[list[tuple[str, int]]], step_s: float) -> bool:
        """Send each setpoint at least ``step_s`` after the one before. False if
        the software stop latched before the last one went out.

        Paced from the previous write, never against a schedule: after a stall
        (a servo that misses a status packet costs a 0.1 s read timeout) a
        schedule sends the overdue setpoints back to back, which ran the tool at
        0.55 m/s against a 0.25 m/s limit. Late setpoints make the move longer,
        never faster.
        """
        for setpoint in ticks_plan:
            if self._latched():
                return False
            sent = time.monotonic()
            for joint, ticks in setpoint:
                self._write_goal(joint, ticks)
            wait = sent + step_s - time.monotonic()
            if wait > 0:
                time.sleep(wait)
        return True

    def _await_arrival(self, joint_positions: dict[str, float], *, started: float, timeout_s: float,
                       interrupted: bool) -> dict:
        """Poll until within tolerance or timeout, then report from ONE snapshot
        (see the note in :meth:`move` on receipts that disagree with themselves)."""
        deadline = time.monotonic() + timeout_s
        while not interrupted and time.monotonic() < deadline and not self._latched():
            current = {j: self._read_joint(j) for j in joint_positions}
            if all(abs(current[j] - joint_positions[j]) <= self.move_tolerance_rad
                   for j in joint_positions):
                break
            time.sleep(0.02)
        final = {j: self._read_joint(j) for j in joint_positions}
        errors = {j: abs(final[j] - joint_positions[j]) for j in joint_positions}
        max_error = max(errors.values()) if errors else 0.0
        return {
            "reached": (not interrupted) and max_error <= self.move_tolerance_rad,
            "final_positions": final,
            "elapsed_s": time.monotonic() - started,
            "max_error_rad": round(max_error, 5),
        }

    @staticmethod
    def _motion_telemetry(plan: motion.Plan, offset: dict[str, float], completed: bool) -> dict:
        """What the receipt says about how a move was paced and checked."""
        return {
            "setpoints": len(plan.setpoints),
            "pace_period_s": round(plan.step_s, 4),
            "planned_duration_s": round(plan.duration_s, 3),
            "tool_speed_limit_mps": plan.tool_speed_limit_mps,
            "joint_speed_limit_dps": plan.joint_speed_limit_dps,
            "tool_path_mm": None if plan.tool_path_mm is None else round(plan.tool_path_mm, 1),
            "path_min_clearance_mm": plan.min_clearance_mm,
            "path_closest": plan.closest,
            "workspace_margin_mm": resolve_workspace_margin_mm(),
            "start_offset_rad": {j: round(v, 4) for j, v in offset.items() if v},
            "stopped_by_estop": not completed,
            "notes": plan.notes,
        }

    def _hold_pose(self) -> dict[str, int]:
        """Hold every joint, and return the ticks held.

        The first hold after the stop latches chooses the goals: the goal a
        joint is holding when it is holding one (within ADOPT_REFERENCE_TICKS;
        see :meth:`_holding_ticks`), otherwise where it reads. Every later hold
        re-sends exactly those ticks. Re-reading instead walks a loaded arm down
        by its sag on every stop (the ratchet: 9 cm in 8 s of repeated stops in
        EV-03's simulation).
        """
        if self._held is None:
            held: dict[str, int] = {}
            for joint, spec in config.JOINTS.items():
                present = int(self._protocol.read_position(motor_id=spec["motor_id"]))
                held[joint] = self._holding_ticks(joint, present)
            self._held = held
        else:
            # A held goal is re-sent only while the joint is still holding it.
            # A joint now far from it (a power cycle reset its goal and the arm
            # dropped) is held where it is: re-sending the old goal would drive
            # it back at the servo's top speed (2.4 m/s at the tip, in sim).
            for joint, spec in config.JOINTS.items():
                present = int(self._protocol.read_position(motor_id=spec["motor_id"]))
                if abs(present - self._held[joint]) > ADOPT_REFERENCE_TICKS:
                    self._held[joint] = present
        for joint, ticks in self._held.items():
            self._write_goal(joint, ticks)
        return dict(self._held)

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
            self._manifest_error = None
        except Exception as exc:  # noqa: BLE001 — recorded; motion refuses on it (see execute)
            # Not fatal for reads. For motion it is: with the geometry unread the
            # workspace, the chain and the limits resolve to nothing, and every
            # move would go unchecked.
            self._manifest_error = (key, f"{type(exc).__name__}: {exc}")

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
            raise DeniedError("bad_args", f"target_mm must be three finite numbers, got {target_mm!r}")
        ok, why = kin.reachable(target, self._manifest_applied)
        if not ok:
            raise OutOfRangeError(why)
        # The declared workspace, with the same margin every step's path is held
        # to. Reach used to be the only check here, so the loop would steer the
        # tip to any point the links could span, through the floor included.
        box = kin.workspace_box(self._manifest_applied)
        margin = resolve_workspace_margin_mm()
        if box is not None and box.clearance(target) < margin:
            raise DeniedError(
                "out_of_workspace",
                f"{list(target)} is {box.clearance(target):.1f} mm inside the declared workspace "
                f"(face {box.nearest_face(target)}); a target must be at least {margin:g} mm inside.")

        refused: dict[str, str] = {}

        def _step(pose: dict[str, float]) -> str | None:
            """One checked, paced move. Returns why it could not be made, or None."""
            try:
                result = self._move_checked(pose, timeout_s=3.0)
            except DeniedError as exc:
                refused.setdefault("code", exc.code)
                return f"the next step was refused: {exc.detail}"
            except OutOfRangeError as exc:
                refused.setdefault("code", "joint_limits")
                return f"the next step was refused: {exc}"
            if result["motion"]["stopped_by_estop"] or self._latched():
                return "stopped by arm.estop"
            return None

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
            # A warm start whose path the workspace check refuses is simply not
            # taken: the loop below servos from where the arm is instead.
            why = _step(safe)
            if why == "stopped by arm.estop":
                current = {j: self._read_joint(j) for j in config.JOINTS if j != "gripper"}
                return {"arrived": False, "error_mm": round(error_now, 2), "iterations": 0,
                        "error_history": history, "warm_started": False, "stopped_because": why,
                        "final_positions": current}
            if why is None:
                self._settle(safe)
                warm_started = True
            else:
                refused.clear()   # a warm start that is refused is simply not taken

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
            if (best_pose is not None and best_error is not None and best_error < error - 1.0
                    and not self._latched()):
                back = {j: best_pose[j] for j in kin.REACH_JOINTS}
                if _step(back) is None:
                    self._settle(back)
                current = {j: self._read_joint(j) for j in config.JOINTS if j != "gripper"}
                error = kin.reach_step(current, target, manifest_path=self._manifest_applied)[1]
            out = {"arrived": False, "error_mm": round(error, 2),
                   "iterations": step, "error_history": history,
                   "warm_started": warm_started, "stopped_because": why,
                   "final_positions": current}
            if why.startswith("the next step was refused") and "code" in refused:
                out["refused"] = refused["code"]
            return out

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
            why = _step(safe)
            if why is not None:
                return _give_up(step, error, current, why)
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

        ``path_leaves_workspace`` the line from here to there passes within
                               the margin of a face (or past it)
        ``too_slow``           `speed` so small the move would outlast
                               motion.MAX_MOVE_S

        ``speed`` (0, 1] is the fraction of the declared speed limits the move
        may use: the tool point's (``safety.max_linear_velocity_ms``, or 0.25
        m/s when the manifest declares none) and each joint's
        (``safety.max_joint_velocity_dps``). The servo bus this driver owns takes
        Goal_Position and nothing else, so a speed limit is a stream of small
        setpoints (motion.PACE_PERIOD_S apart), each sized so the tool cannot be
        asked to move faster than the limit.
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

        # 6 and 7. The PATH, not just its ends: the joint-space line from the
        #    pose the arm is held at to this solution is checked against the
        #    declared workspace at the elbow, wrist and tip, as commanded and as
        #    the arm will really be (``path_leaves_workspace``), and paced under
        #    the declared speed limits (``too_slow`` when `speed` is so small the
        #    move would outlast motion.MAX_MOVE_S). Both endpoints inside the box
        #    used to be taken as proof the path was too, the target was checked
        #    only as commanded (not as the sagging arm would really sit), and the
        #    move was one full-slew jump: EV-03 found the tip up to 13 mm below
        #    the floor in simulation.
        #
        # wrist_roll is HELD, not solved: it turns about the tool axis, which is
        # the axis every remaining link offset lies along, so it moves the tip by
        # exactly nothing. Commanding it to some fresh value would spin whatever
        # is in the gripper for no reason.
        result = self._move_checked(dict(solution.joints), speed=speed, timeout_s=timeout_s,
                                    hold=("wrist_roll",))
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
            waypoints=result["motion"]["setpoints"],
            ik_provider=solution.provider,
            motion=result["motion"],
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
        time.sleep(min_dwell_s)
        deadline = time.monotonic() + max_wait_s
        last = {j: self._read_joint(j) for j in commanded}
        while time.monotonic() < deadline and not self._latched():
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
            with _TRANSPORT_LOCK:
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
            # LATCH FIRST, outside the bus lock. A paced move holds that lock for
            # its whole stream and checks this latch before every setpoint, so
            # setting it here is what stops a move in progress within one pace
            # period. Waiting for the lock first would let the move finish.
            transport._estopped = True
            held: dict[str, float] = {}
            try:
                # Same lock every move takes: the hold writes a setpoint per
                # joint, and a read interleaved into that sequence corrupts both.
                # A move in progress lets go within one pace period of the latch.
                with _bus(STOP_BUS_WAIT_S):
                    transport.estop()
                    held = {joint: _config_module.ticks_to_rad(joint, ticks)
                            for joint, ticks in (self._held or {}).items()}
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
                    # The goals the stop is holding. Repeating the stop re-sends
                    # exactly these; it never re-reads a sagged pose.
                    "held_positions": held,
                    "safety_note": ESTOP_SAFETY_NOTE,
                },
            )
        if tool_name == "arm.estop.clear":
            transport = self._estop_transport(port=port, baud=baud)
            try:
                # Under the bus lock, so a clear cannot interleave with a stop
                # that is still choosing what to hold.
                with _bus(STOP_BUS_WAIT_S):
                    transport.clear_estop()
            except DeniedError as exc:
                return _denied(exc)
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
            # Fail closed on geometry. A manifest whose geometry could not be
            # read leaves the workspace, the chain and the limits resolving to
            # nothing, and the move would go unchecked.
            key = str(manifest_path) if manifest_path else ""
            if key and self._manifest_error is not None and self._manifest_error[0] == key:
                return _denied(DeniedError(
                    "manifest_unreadable",
                    f"this robot's geometry could not be read from {key} "
                    f"({self._manifest_error[1]}), so no motion can be checked against it"))

        # ROBOT.md / iOS capability names -> RAP methods. arm.pick / arm.place
        # stay unmapped: they need the vision rig, and the gateway deny-lists
        # them via ROBOT_MD_TOOL_ALLOWLIST so clients get a signed DENY instead.
        taught = False
        if tool_name in ("arm.home", "home"):
            pose = self._home_pose_for_motion()
            tool_name, tool_args, taught = "move", {"joint_positions": pose}, True
        elif tool_name == "arm.reach_point":
            target = tool_args.get("target_mm")
            if (not isinstance(target, (list, tuple)) or len(target) != 3
                    or any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in target)):
                return _denied(DeniedError(
                    "bad_args", "arm.reach_point needs target_mm as [x, y, z], three numbers"))
            tolerance = tool_args.get("tolerance_mm", 5.0)
            if (isinstance(tolerance, bool) or not isinstance(tolerance, (int, float))
                    or not math.isfinite(tolerance) or tolerance <= 0):
                return _denied(DeniedError(
                    "bad_args", f"tolerance_mm must be a positive number, got {tolerance!r}"))
            # The warm-start table is arithmetic (seconds of it on a Pi, the
            # first time). Build it before taking the bus, not while holding it.
            try:
                kin._safe_table(self._manifest_applied, config.SAFE_RANGE_RAD)
            except Exception:  # noqa: BLE001 — reach_point falls back to a cold start
                pass
            try:
                # The same lock and lazy open as every other motion. reach_point
                # used to run outside both: on a fresh gateway (no protocol open
                # yet) it failed with a 500, and it could interleave its bus
                # traffic with another request's.
                with _bus(BUS_WAIT_S):
                    try:
                        self._ensure_protocol(port=port, baud=baud)
                        telemetry = self.reach_point(target, tolerance_mm=float(tolerance))
                    except (OSError, IOError) as exc:
                        self._drop_protocol()       # under the lock, never beside another request
                        return ActuatorOutcome(success=False, outcome_kind="error",
                                               error_message=f"{type(exc).__name__}: {exc}")
            except DeniedError as exc:
                return _denied(exc)
            except OutOfRangeError as exc:
                return _denied(DeniedError("unreachable", str(exc)))
            except Exception as exc:  # noqa: BLE001 — exceptions become outcomes
                return ActuatorOutcome(success=False, outcome_kind="error",
                                       error_message=f"{type(exc).__name__}: {exc}")
            if telemetry.get("refused"):
                # A step the workspace check refused is a decision, like any
                # other refusal: a signed 403 that carries how far it got.
                return ActuatorOutcome(
                    success=False, outcome_kind="denied",
                    telemetry={**telemetry, "deny": telemetry["refused"],
                               "reason": telemetry.get("stopped_because")},
                    error_message=f"{telemetry['refused']}: {telemetry.get('stopped_because')}")
            return ActuatorOutcome(
                success=bool(telemetry.get("arrived")),
                outcome_kind="executed" if telemetry.get("arrived") else "error",
                telemetry=telemetry,
                error_message=(
                    f"{telemetry.get('stopped_because')} "
                    f"(final error {telemetry.get('error_mm')} mm, "
                    f"history {telemetry.get('error_history')}, "
                    f"warm_started={telemetry.get('warm_started')})"))
        elif tool_name in ("arm.move_to", "move_to"):
            # Argument shape is settled BEFORE the bus is opened: a malformed
            # request should not cost a serial handle, and it must come back as
            # a deny rather than as a driver crash. The bare `move_to` alias is
            # parsed the same way; it used to skip this and take any timeout_s.
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
            tool_name, tool_args, taught = "move", {"joint_positions": reach_pose}, True
        if tool_name == "move":
            # Every joint move the gateway can ask for (arm.home, arm.reach, a
            # bare `move`) is checked against the workspace and paced, exactly
            # like arm.move_to. `move()` itself stays the raw primitive it was.
            unknown = sorted(set(tool_args) - {"joint_positions", "speed", "timeout_s"})
            joints = tool_args.get("joint_positions")
            timeout = tool_args.get("timeout_s", 10.0)
            if (unknown or not isinstance(joints, dict) or isinstance(timeout, bool)
                    or not isinstance(timeout, (int, float)) or not 0 < timeout <= 60):
                return _denied(DeniedError(
                    "bad_args", "move takes joint_positions ({joint: radians}), an optional speed "
                                "and an optional timeout_s (0 to 60 s)"
                                + (f"; not {', '.join(unknown)}" if unknown else "")))
            tool_args = {"joint_positions": joints,
                         "speed": tool_args.get("speed", DEFAULT_SPEED),
                         "timeout_s": float(timeout),
                         "taught_goal": taught}
        method = {
            "move": self._move_checked,
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
            # Held across open AND the whole operation: a move polls the bus
            # repeatedly until it converges, and a read slipped in between
            # those polls corrupts both. Waited for only briefly (BUS_WAIT_S):
            # a request queued behind a long move holds a gateway thread the
            # stop may need.
            with _bus(BUS_WAIT_S):
                try:
                    self._ensure_protocol(port=port, baud=baud)
                    result = method(**tool_args)
                except (OSError, IOError) as exc:
                    # The serial handle is held for the process lifetime, so a
                    # USB replug leaves a dead fd that would fail every later
                    # invoke. Drop it (and release its exclusive claim) so the
                    # next invoke re-opens by-id instead of needing a restart,
                    # and do it while still holding the bus: dropping it after
                    # letting go closed the handle under the next request.
                    self._drop_protocol()
                    return ActuatorOutcome(
                        success=False,
                        outcome_kind="error",
                        error_message=f"{type(exc).__name__}: {exc}",
                    )
        except DeniedError as exc:
            # A refusal is a decision. It leaves the bus untouched and reaches
            # the caller as a signed 403, not as a 500 that reads like a fault.
            return _denied(exc)
        except UnknownJointError as exc:
            return _denied(DeniedError("unknown_joint", f"no joint named {exc.args[0]!r} on this arm"))
        except OutOfRangeError as exc:
            return _denied(DeniedError("joint_limits", str(exc)))
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
