"""castor-hal Transport adapter for the SO-ARM101: read and stop only.

This wraps the existing, tested ``SOArm101Actuator`` (serial bus servos via
``SCSProtocol``) behind the universal ``castor_hal.Transport`` seam, so the arm
can be read and stopped through ``castor_hal.TransportActuator`` (see
``make_hal_actuator``). It is not the entry point robot-md-gateway uses: that is
``SOArm101Actuator`` itself, which needs nothing from castor-hal. This module
does, and castor-hal is an optional extra: ``pip install
"so-arm101-actuator[hal]"``.

It moves nothing. ``set_goal`` used to drive the actuator's raw ``move()`` and
``home()``: one Goal_Position jump per joint, not checked against the declared
workspace, not paced, and outside the bus lock, so everything EV-03 fixed on the
gateway path (October 2026) was bypassed here. It now accepts no goals:
``capabilities`` is empty and every ``set_goal`` is refused as
``unsupported_goal`` before the bus is touched. Sending it through the checked,
paced path instead would need the manifest that path checks against, which the
``Transport`` seam does not carry.

The stop is the wrapped actuator's own latch (``_estop_transport``), shared, so
this adapter and the gateway path always agree on whether the arm is stopped.

SAFETY: ``estop()`` is a best-effort SOFTWARE hold (each joint is sent a goal
to hold: the goal it was holding, or where it reads if it was moving; a repeated
stop re-sends the same goals) — NOT a hardware e-stop. The SCS bus exposes no
torque-off here, and software cannot guarantee the arm physically stopped. See
``castor_hal.transport`` for the binding safety note.
"""

from __future__ import annotations

try:
    from castor_hal.errors import TransportError, TransportErrorCode
    from castor_hal.state import TransportState
    from castor_hal.transport import GoalResult, Transport
except ModuleNotFoundError as exc:  # castor-hal is the optional `hal` extra
    if exc.name != "castor_hal":
        raise
    raise ModuleNotFoundError(
        "so_arm101_actuator.transport is the castor-hal adapter, and castor-hal is "
        "not installed. It is an optional extra: pip install 'so-arm101-actuator[hal]' "
        "(castor-hal is not on PyPI yet; install it from "
        "https://github.com/RobotRegistryFoundation/castor-hal first). The "
        "robot-md-gateway entry point, SOArm101Actuator, does not need it.",
        name="castor_hal",
    ) from exc

from so_arm101_actuator.actuator import SOArm101Actuator, _StopLatch


class SOArm101Transport(Transport):
    """SO-ARM101 (6-DOF + gripper) as a castor-hal Transport, read and stop only.
    RPN-000000000002."""

    #: No goal kinds: motion through this adapter is fenced off (module note).
    capabilities = frozenset()
    name = "so-arm101"
    description = "SO-ARM101 serial-bus-servo transport, read and stop only (RPN-000000000002)"

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
        # Accepted so existing callers and from_config keep working; nothing
        # moves through this adapter, so nothing reads them.
        self._move_timeout_s = move_timeout_s
        self._home_timeout_s = home_timeout_s

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

    def _latch(self) -> _StopLatch:
        """The wrapped actuator's own stop latch, built on first use."""
        return self._actuator._estop_transport(port=self._port, baud=self._baud)

    @property
    def _estopped(self) -> bool:
        """Whether the wrapped actuator's stop is latched. Read, never copied."""
        return bool(self._latch()._estopped)

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
        """Refused, for every goal, before the bus is touched: this adapter
        moves nothing (see the module note). Move the arm through
        robot-md-gateway's ``so-arm101`` entry point, which checks and paces
        every motion."""
        kind = getattr(goal, "kind", None)
        raise TransportError(
            TransportErrorCode.UNSUPPORTED_GOAL,
            "the SO-ARM101 castor-hal adapter accepts no goals: its motion path drove "
            "the raw move() (not checked against the workspace, not paced, outside the "
            "bus lock), so it is disabled. Move the arm through robot-md-gateway's "
            "so-arm101 entry point, which checks and paces every motion.",
            detail={
                "transport": self.name,
                "goal_kind": getattr(kind, "value", str(kind)),
                "capabilities": [],
            },
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
        Holds each joint (``SOArm101Actuator._hold_pose``) so motion stops, and
        latches the wrapped actuator's own stop (cleared via ``clear_estop``),
        so the gateway path refuses motion too.

        What is held is chosen once per latch: the goal a joint was last sent
        when it is holding that goal, otherwise where it reads. A repeated stop
        re-sends the same goals. It used to re-read the encoders every time, and
        under gravity each reading is the sagged pose, so every repeat lowered
        the goal by the sag: 9 cm in 8 s of stops 0.2 s apart (EV-03,
        simulation, October 2026).

        FAIL-SAFE: the latch is set FIRST, before the bus hold is attempted —
        so if the link is down (``open`` raises) or the hold cannot be sent, the
        arm stays e-stopped and refuses new motion (``clear_estop`` + retry to
        recover), rather than silently accepting motion after a stop we
        couldn't confirm. We fail toward stopped, never toward movable."""
        self._latch()._estopped = True
        self.open()
        try:
            self._actuator._hold_pose()
        except (IOError, OSError) as exc:
            raise TransportError(
                TransportErrorCode.IO_ERROR, f"e-stop could not command hold: {exc}"
            ) from exc

    def clear_estop(self) -> None:
        """Release the latched software e-stop (the wrapped actuator's own) so
        motion can be commanded again through the gateway path. The arm keeps
        holding its pose; the next stop chooses its hold afresh."""
        self._latch().clear_estop()


def make_hal_actuator(config_dict: dict | None = None):
    """Convenience: an SO-ARM101 wrapped as a gateway actuator through the HAL,
    for reading and stopping only (it refuses every goal; see the module note).

    Returns a ``castor_hal.TransportActuator`` that builds an ``SOArm101Transport``
    lazily from the gateway config on first ``execute``. No entry point uses it;
    the ``so-arm101`` entry point (``SOArm101Actuator``) is the one that moves
    the arm.
    """
    from castor_hal.actuator import TransportActuator

    return TransportActuator(
        factory=SOArm101Transport.from_config,
        name="so-arm101",
        description=SOArm101Transport.description,
    )
