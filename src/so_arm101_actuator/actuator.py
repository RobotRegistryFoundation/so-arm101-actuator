"""SO-ARM101 Actuator Protocol implementation. RPN-000000000002."""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import TypedDict

from robot_md_gateway.actuator import ActuatorOutcome

from so_arm101_actuator import config
from so_arm101_actuator import config as _config_module
from so_arm101_actuator.errors import (
    UnknownJointError,
    OutOfRangeError,
    ActuatorTimeoutError,
)


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
    "arm.home", "arm.reach", "status.report",
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
}


class SOArm101Actuator:
    """RobotRegistryFoundation/so-arm101-actuator v0.1.0 — RPN-000000000002.

    Constructed with an `SCSProtocol` (or compatible mock for tests). The
    default factory in `from_default_port` opens `/dev/ttyACM0` at 1 Mbps.
    """

    name = "so-arm101"
    description = "SO-ARM101 6-DOF + gripper Actuator Protocol driver. RPN-000000000002."
    config_schema: dict = {}

    capabilities = ("move", "home", "read_state")

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
