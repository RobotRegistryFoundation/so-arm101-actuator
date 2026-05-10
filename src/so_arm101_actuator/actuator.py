"""SO-ARM101 Actuator Protocol implementation. RPN-000000000002."""

from __future__ import annotations

import time
from typing import TypedDict

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

    capabilities = ("move", "home", "read_state")

    def __init__(self, protocol) -> None:  # noqa: ANN001 — duck-typed
        self._protocol = protocol

    @classmethod
    def from_default_port(cls, port: str = "/dev/ttyACM0", baud: int = 1_000_000) -> "SOArm101Actuator":
        import serial
        from so_arm101_actuator.protocol import SCSProtocol
        ser = serial.Serial(port=port, baudrate=baud, timeout=0.1)
        return cls(protocol=SCSProtocol(serial=ser))

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

    def _read_joint(self, joint: str) -> float:
        ticks = self._protocol.read_position(motor_id=config.JOINTS[joint]["motor_id"])
        return config.ticks_to_rad(joint, ticks)
