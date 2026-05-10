"""SO-ARM101 Actuator Protocol implementation. RPN-000000000002."""

from __future__ import annotations

import time
from pathlib import Path
from typing import TypedDict

from robot_md_gateway.actuator import ActuatorOutcome

from so_arm101_actuator import config
from so_arm101_actuator.errors import (
    UnknownJointError,
    OutOfRangeError,
    ActuatorTimeoutError,
)


class MoveResult(TypedDict):
    reached: bool
    final_positions: dict[str, float]
    elapsed_s: float


class ActuatorState(TypedDict):
    positions: dict[str, float]
    motor_temps_c: dict[str, float]
    timestamp_s: float


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
        """
        env_pose = config.resolve_home_pose_rad()
        if home_pose_rad is not None:
            # kwarg merges on top of env-resolved pose (kwarg wins per joint)
            env_pose = {**env_pose, **home_pose_rad}
        self.home_pose_rad: dict[str, float] = env_pose
        self._protocol = protocol

    @classmethod
    def from_default_port(cls, port: str = "/dev/ttyACM0", baud: int = 1_000_000) -> "SOArm101Actuator":
        import serial
        from so_arm101_actuator.protocol import SCSProtocol
        ser = serial.Serial(port=port, baudrate=baud, timeout=0.1)
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
            serial=serial.Serial(port=port, baudrate=baud, timeout=0.1)
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
            if all(abs(current[j] - joint_positions[j]) <= config.MOVE_TOLERANCE_RAD for j in joint_positions):
                reached = True
                break
            time.sleep(0.02)

        # Snapshot final state for *all* commanded joints.
        final = {j: self._read_joint(j) for j in joint_positions}
        return MoveResult(
            reached=reached,
            final_positions=final,
            elapsed_s=time.monotonic() - start,
        )

    def home(self, *, timeout_s: float = 10.0) -> MoveResult:
        """Move all joints to config.HOME_POSE_RAD."""
        return self.move(config.HOME_POSE_RAD, timeout_s=timeout_s)

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
        tool_name = envelope.get("tool_name")
        tool_args = envelope.get("tool_args", {}) or {}
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
            self._ensure_protocol(port=port, baud=baud)
            result = method(**tool_args)
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
