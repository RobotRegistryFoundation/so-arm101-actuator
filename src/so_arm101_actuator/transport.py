"""castor-hal Transport adapter for the SO-ARM101 — the first reference adapter.

This wraps the existing, tested ``SOArm101Actuator`` (serial bus servos via
``SCSProtocol``) behind the universal ``castor_hal.Transport`` seam. It is the
proof that the HAL pattern works against a REAL driver: the trust pipeline above
it (gateway tier/confidence/HiTL gating, signing, RCAN → Atlas) is identical to
what a tractor (CAN/J1939, ISOBUS) will use — only THIS file changes per machine.

It is purely ADDITIVE: the original ``SOArm101Actuator`` and its entry point are
unchanged. To run the arm through the HAL, wrap this in
``castor_hal.TransportActuator`` (see ``make_hal_actuator``).

SAFETY: ``estop()`` is a best-effort SOFTWARE hold (command each joint to its
current encoder reading so motion stops) — NOT a hardware e-stop. The SCS bus
exposes no torque-off here, and software cannot guarantee the arm physically
stopped. See ``castor_hal.transport`` for the binding safety note.
"""

from __future__ import annotations

import time

from castor_hal.errors import TransportError, TransportErrorCode
from castor_hal.goal import GoalKind
from castor_hal.state import TransportState
from castor_hal.transport import GoalResult, Transport

from so_arm101_actuator import config
from so_arm101_actuator.actuator import SOArm101Actuator
from so_arm101_actuator.errors import OutOfRangeError, UnknownJointError


class SOArm101Transport(Transport):
    """SO-ARM101 (6-DOF + gripper) as a castor-hal Transport. RPN-000000000002."""

    capabilities = frozenset({GoalKind.JOINT_POSITIONS, GoalKind.HOME})
    name = "so-arm101"
    description = "SO-ARM101 serial-bus-servo transport (RPN-000000000002)"

    def __init__(
        self,
        actuator: SOArm101Actuator | None = None,
        *,
        protocol=None,  # noqa: ANN001 — duck-typed SCSProtocol (or mock)
        port: str = "/dev/ttyACM0",
        baud: int = 1_000_000,
        move_timeout_s: float = 5.0,
        home_timeout_s: float = 10.0,
        home_pose_rad: dict[str, float] | None = None,
        move_tolerance_rad: float | None = None,
    ) -> None:
        self._actuator = actuator or SOArm101Actuator(
            protocol=protocol,
            home_pose_rad=home_pose_rad,
            move_tolerance_rad=move_tolerance_rad,
        )
        self._port = port
        self._baud = baud
        self._move_timeout_s = move_timeout_s
        self._home_timeout_s = home_timeout_s
        self._estopped = False

    @classmethod
    def from_config(cls, config_dict: dict | None) -> "SOArm101Transport":
        """Build from the gateway's execute-time config dict (the entry-point
        path): reads ``port`` / ``baud`` (and optional timeouts)."""
        cfg = config_dict or {}
        return cls(
            port=str(cfg.get("port", "/dev/ttyACM0")),
            baud=int(cfg.get("baud", 1_000_000)),
            move_timeout_s=float(cfg.get("move_timeout_s", 5.0)),
            home_timeout_s=float(cfg.get("home_timeout_s", 10.0)),
        )

    # -- lifecycle ---------------------------------------------------------
    def open(self) -> None:
        try:
            self._actuator._ensure_protocol(port=self._port, baud=self._baud)
        except (IOError, OSError) as exc:
            raise TransportError(
                TransportErrorCode.NOT_CONNECTED,
                f"cannot open SO-ARM101 on {self._port}: {exc}",
                detail={"port": self._port, "baud": self._baud},
            ) from exc

    def close(self) -> None:
        proto = getattr(self._actuator, "_protocol", None)
        ser = getattr(proto, "_serial", None)
        if ser is not None:
            try:
                ser.close()
            except Exception:  # noqa: BLE001 — close is best-effort
                pass

    # -- actuation / sensing ----------------------------------------------
    def set_goal(self, goal) -> GoalResult:
        self.ensure_supported(goal)
        if self._estopped:
            raise TransportError(
                TransportErrorCode.ESTOPPED,
                "transport is e-stopped; clear it before commanding motion",
            )
        try:
            if goal.kind is GoalKind.HOME:
                result = self._actuator.home(timeout_s=self._home_timeout_s)
            else:  # JOINT_POSITIONS (the only other accepted kind)
                result = self._actuator.move(goal.positions, timeout_s=self._move_timeout_s)
        except UnknownJointError as exc:
            raise TransportError(
                TransportErrorCode.UNKNOWN_TARGET, f"unknown joint: {exc}",
                detail={"joints": sorted(config.JOINTS)},
            ) from exc
        except OutOfRangeError as exc:
            raise TransportError(TransportErrorCode.OUT_OF_RANGE, str(exc)) from exc
        except (IOError, OSError) as exc:
            raise TransportError(TransportErrorCode.IO_ERROR, str(exc)) from exc

        state = TransportState(
            connected=True,
            estopped=self._estopped,
            positions=dict(result["final_positions"]),
            unit="rad",
            timestamp_s=time.monotonic(),
        )
        return GoalResult(
            reached=bool(result["reached"]),
            state=state,
            detail={"elapsed_s": result["elapsed_s"], "final_positions": dict(result["final_positions"])},
        )

    def read_state(self) -> TransportState:
        try:
            st = self._actuator.read_state()
        except (IOError, OSError) as exc:
            raise TransportError(TransportErrorCode.IO_ERROR, str(exc)) from exc
        return TransportState(
            connected=True,
            estopped=self._estopped,
            positions=dict(st["positions"]),
            temperatures_c=dict(st["motor_temps_c"]),
            unit="rad",
            timestamp_s=st["timestamp_s"],
        )

    def estop(self) -> None:
        """Best-effort SOFTWARE hold (NOT a hardware e-stop — see module note).
        Commands each joint to its current encoder reading so motion stops, then
        latches the e-stop flag (cleared via ``clear_estop``).

        FAIL-SAFE: the flag is latched FIRST, before the bus hold is attempted —
        so if the link is down (``open`` raises) or the hold cannot be sent, the
        transport stays e-stopped and refuses new motion (``clear_estop`` +
        retry to recover), rather than silently accepting motion after a stop we
        couldn't confirm. We fail toward stopped, never toward movable."""
        self._estopped = True
        self.open()
        proto = self._actuator._protocol
        try:
            for spec in config.JOINTS.values():
                ticks = proto.read_position(motor_id=spec["motor_id"])
                proto.set_position(motor_id=spec["motor_id"], ticks=ticks)
        except (IOError, OSError) as exc:
            raise TransportError(
                TransportErrorCode.IO_ERROR, f"e-stop could not command hold: {exc}"
            ) from exc

    def clear_estop(self) -> None:
        """Release the latched software e-stop so motion can be commanded again."""
        self._estopped = False


def make_hal_actuator(config_dict: dict | None = None):
    """Convenience: an SO-ARM101 wrapped as a gateway actuator through the HAL.

    Returns a ``castor_hal.TransportActuator`` that builds an ``SOArm101Transport``
    lazily from the gateway config on first ``execute``. A future entry point can
    point at this once the HAL path is hardware-validated; the existing
    ``so-arm101`` entry point (``SOArm101Actuator``) is left untouched.
    """
    from castor_hal.actuator import TransportActuator

    return TransportActuator(
        factory=SOArm101Transport.from_config,
        name="so-arm101",
        description=SOArm101Transport.description,
    )
