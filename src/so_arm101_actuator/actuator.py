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


# Tool_names whose audit outcome is recorded as "commissioned" rather than "executed".
_COMMISSION_OUTCOME_TOOLS = frozenset(
    {"set_torque", "raw_tick_move", "commission_probe"}
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
    description = (
        "SO-ARM101 6-DOF + gripper Actuator Protocol driver. RPN-000000000002."
    )
    config_schema: dict = {}

    capabilities = (
        "move",
        "home",
        "read_state",
        # Commissioning + productized-motion ops (route through the gateway behind the
        # `commission` tier; the gateway gates tool_name via its tool_allowlist, not this tuple).
        "set_torque",
        "raw_tick_move",
        "commission_probe",
        "paced_move",
    )

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
        env_pose = config.resolve_home_pose_rad()
        if home_pose_rad is not None:
            # kwarg merges on top of env-resolved pose (kwarg wins per joint)
            env_pose = {**env_pose, **home_pose_rad}
        self.home_pose_rad: dict[str, float] = env_pose

        env_tolerance = config.resolve_move_tolerance_rad()
        if move_tolerance_rad is not None:
            self.move_tolerance_rad: float = move_tolerance_rad
        else:
            self.move_tolerance_rad = env_tolerance

        self._protocol = protocol
        # Manifest-resolved rad↔tick table (A2). Defaults to the shipped JOINTS until
        # execute() resolves it from the live manifest on first call (cached).
        self._joints: dict = {name: dict(spec) for name, spec in config.JOINTS.items()}
        self._joints_resolved = False

    def _resolve_joints(self, manifest_path: Path) -> None:
        """Refresh `self._joints` from the manifest's commissioned endpoints, once.

        Best-effort: any failure (missing/unparseable manifest, no commissioned fields)
        leaves the shipped defaults in place — calibration resolution must never block
        actuation.
        """
        if self._joints_resolved:
            return
        try:
            resolved = config.resolve_joints_from_manifest(manifest_path)
            if resolved:
                self._joints = resolved
        except Exception:  # noqa: BLE001 — never let resolution block the actuation path
            pass
        self._joints_resolved = True

    @classmethod
    def from_default_port(
        cls, port: str = "/dev/ttyACM0", baud: int = 1_000_000
    ) -> "SOArm101Actuator":
        import serial
        from so_arm101_actuator.protocol import SCSProtocol

        ser = serial.Serial(port=port, baudrate=baud, timeout=0.1)
        return cls(protocol=SCSProtocol(serial=ser))

    def _ensure_protocol(
        self, *, port: str = "/dev/ttyACM0", baud: int = 1_000_000
    ) -> None:
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

    def _validate_rad_targets(self, joint_positions: dict[str, float]) -> None:
        """Reject unknown joints / out-of-range radians before any wire traffic.

        Uses the manifest-resolved table (`self._joints`), so mechanical limits are the
        commissioned ones when present.
        """
        for joint in joint_positions:
            if joint not in self._joints:
                raise UnknownJointError(joint)
        for joint, rad in joint_positions.items():
            spec = self._joints[joint]
            if not (spec["min_rad"] <= rad <= spec["max_rad"]):
                raise OutOfRangeError(
                    f"{joint}={rad:.3f} outside [{spec['min_rad']}, {spec['max_rad']}]"
                )

    def move(
        self, joint_positions: dict[str, float], *, timeout_s: float = 5.0
    ) -> MoveResult:
        # Validate before any wire traffic.
        self._validate_rad_targets(joint_positions)

        # Issue commands.
        start = time.monotonic()
        for joint, rad in joint_positions.items():
            self._protocol.set_position(
                motor_id=self._joints[joint]["motor_id"],
                ticks=config.rad_to_ticks_spec(self._joints[joint], rad),
            )

        # Poll until within tolerance or timeout.
        reached = False
        while time.monotonic() - start < timeout_s:
            current = {j: self._read_joint(j) for j in joint_positions}
            if all(
                abs(current[j] - joint_positions[j]) <= self.move_tolerance_rad
                for j in joint_positions
            ):
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
        """Move all joints to the resolved home pose (env/kwarg-overridable)."""
        return self.move(self.home_pose_rad, timeout_s=timeout_s)

    def read_state(self) -> ActuatorState:
        """Read all joint positions and motor temperatures (best-effort).

        Returns a snapshot of the current actuator state with positions in
        radians and temperatures in celsius. If a motor's temperature sensor
        fails to read, that motor is silently omitted from motor_temps_c.
        """
        positions: dict[str, float] = {}
        temps: dict[str, float] = {}
        for joint in self._joints:
            positions[joint] = self._read_joint(joint)
            try:
                temps[joint] = float(
                    self._protocol.read_temperature(
                        motor_id=self._joints[joint]["motor_id"],
                    )
                )
            except (IOError, OSError):
                # Best-effort — joints without a temp sensor are simply omitted.
                pass
        return ActuatorState(
            positions=positions,
            motor_temps_c=temps,
            timestamp_s=time.monotonic(),
        )

    def _read_joint(self, joint: str) -> float:
        ticks = self._protocol.read_position(motor_id=self._joints[joint]["motor_id"])
        return config.ticks_to_rad_spec(self._joints[joint], ticks)

    # ------------------------------------------------------------------
    # Commissioning + productized-motion ops (B1). All route through the
    # gateway's existing execute() path — no second serial port, no handoff.
    # ------------------------------------------------------------------

    def set_torque(self, *, motor_ids="all", enable: bool = True) -> dict:
        """Enable/disable Torque_Enable on one, several, or all joints.

        ``motor_ids``: ``"all"`` (every configured joint), a single int, or a list of ints.
        Torque-off is the teach path's hand-pose step — done here (through the gateway) so
        safety stays on, instead of stopping the service.
        """
        if motor_ids == "all":
            ids = [spec["motor_id"] for spec in self._joints.values()]
        elif isinstance(motor_ids, int):
            ids = [motor_ids]
        else:
            ids = list(motor_ids)
        for mid in ids:
            self._protocol.set_torque(motor_id=mid, enable=enable)
        return {"motor_ids": ids, "enabled": enable}

    def raw_tick_move(
        self,
        *,
        motor_id: int,
        ticks: int,
        ramp: int = 6,
        settle_s: float = 0.7,
        step_pause_s: float = 0.08,
    ) -> dict:
        """Ramp `motor_id` from its present tick to `ticks` in `ramp` micro-steps, then
        settle, and report the present tick (ports castor-dash `grip.grip_to`).

        Ramping keeps the move gentle (no slam) and lets a clamp stall cleanly — this is
        the raw-tick gripper grasp that finally held the brick (stall 1529). Bypasses the
        rad interface deliberately, driving the same bus the actuator already holds.
        """
        cur = int(self._protocol.read_position(motor_id=motor_id))
        for s in range(1, ramp + 1):
            t = int(round(cur + (ticks - cur) * s / ramp))
            t = max(0, min(4095, t))
            self._protocol.set_position(motor_id=motor_id, ticks=t)
            if step_pause_s:
                time.sleep(step_pause_s)
        if settle_s:
            time.sleep(settle_s)
        present = int(self._protocol.read_position(motor_id=motor_id))
        return {"present_tick": present, "commanded_tick": ticks}

    def commission_probe(
        self,
        *,
        joint_id: str,
        motor_id: int,
        direction: int,
        target_ticks: int,
        current_limit: float | None = None,
        step: int = 8,
        stall_steps: int = 3,
        reach_margin: int = 10,
        max_probe_steps: int = 512,
        settle_s: float = 0.5,
        step_pause_s: float = 0.05,
    ) -> dict:
        """Reality-check one joint's travel toward `target_ticks`, incrementally and safely.

        Declared endpoints may be WRONG (that's the whole premise), so this NEVER slams a
        far Goal_Position and lets the servo grind. It steps `step` ticks at a time in
        `direction` (±1), reads present after each, and ABORTS as soon as present plateaus
        for `stall_steps` consecutive steps (a mechanical stop, or — for the gripper — a
        real grasp). Classification (FAIL/WARN/PASS) is left to the CLI; this returns the
        raw facts.

        Note on `current_limit`: the SCS protocol layer exposes no current/torque-limit
        register, so physical protection comes from this incremental-step + plateau-abort
        loop plus the servo's own internal current limit — not a register write. The arg is
        accepted for forward-compat and recorded; it is advisory here.

        Returns ``{start_tick, commanded_tick, present_tick, moved, reached, aborted_on_stall}``.
        """
        sign = 1 if direction >= 0 else -1
        advance_eps = max(2, step // 4)
        start = int(self._protocol.read_position(motor_id=motor_id))
        present = start
        commanded = start
        stalls = 0
        reached = False
        aborted = False

        def _at_target(p: int) -> bool:
            return (
                p >= target_ticks - reach_margin
                if sign > 0
                else p <= target_ticks + reach_margin
            )

        for _ in range(max_probe_steps):
            if _at_target(present):
                reached = True
                break
            prev = present
            commanded = max(0, min(4095, commanded + sign * step))
            self._protocol.set_position(motor_id=motor_id, ticks=commanded)
            if step_pause_s:
                time.sleep(step_pause_s)
            present = int(self._protocol.read_position(motor_id=motor_id))
            if (present - prev) * sign < advance_eps:
                stalls += 1
                if stalls >= stall_steps:
                    aborted = True
                    break
            else:
                stalls = 0
            if commanded in (
                0,
                4095,
            ):  # hit the tick rail without reaching → stop probing
                break

        if settle_s:
            time.sleep(settle_s)
        present = int(self._protocol.read_position(motor_id=motor_id))
        moved = abs(present - start) > advance_eps
        if not reached and not aborted:
            aborted = True  # ran out of probe budget without reaching → treat as a stall, fail-safe
        return {
            "start_tick": start,
            "commanded_tick": commanded,
            "present_tick": present,
            "moved": moved,
            "reached": reached and moved,
            "aborted_on_stall": aborted and not reached,
        }

    def paced_move(
        self,
        *,
        joint_positions: dict[str, float],
        steps: int = 14,
        step_timeout_s: float = 0.04,
        pace_s: float = 0.07,
    ) -> dict:
        """Issue-and-pace interpolated motion to `joint_positions` (radians).

        Ports castor-dash `vision_autocal.slow_move`: interpolate current→target in `steps`
        micro-steps, ISSUE each step (set_position) and pace with a sleep — it must NEVER
        poll-to-tolerance: Bob's joints sag and never reach tolerance, so a poll loop stalls
        ~2 s/step. Use this for all productized motion (sweeps, replay, pick); the grasp is a
        separate raw_tick_move close. `step_timeout_s` is accepted for contract-compat with
        the gateway tool signature but is a no-op in this direct-issue path.
        """
        self._validate_rad_targets(joint_positions)
        cur = {j: self._read_joint(j) for j in joint_positions}
        for s in range(1, steps + 1):
            f = s / steps
            for joint, target in joint_positions.items():
                rad = cur[joint] + (target - cur[joint]) * f
                self._protocol.set_position(
                    motor_id=self._joints[joint]["motor_id"],
                    ticks=config.rad_to_ticks_spec(self._joints[joint], rad),
                )
            if pace_s:
                time.sleep(pace_s)
        final = {j: self._read_joint(j) for j in joint_positions}
        return {"final_positions": final}

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
            "set_torque": self.set_torque,
            "raw_tick_move": self.raw_tick_move,
            "commission_probe": self.commission_probe,
            "paced_move": self.paced_move,
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
            # Make calibration manifest-authoritative (A2): resolve the rad↔tick table
            # from the live manifest's commissioned endpoints once, before any conversion.
            self._resolve_joints(manifest_path)
            result = method(**tool_args)
        except Exception as exc:  # noqa: BLE001 — actuator code is operator-supplied; exceptions become outcomes
            return ActuatorOutcome(
                success=False,
                outcome_kind="error",
                error_message=f"{type(exc).__name__}: {exc}",
            )
        # Commissioning ops (set_torque / raw_tick_move / commission_probe) are recorded
        # as a distinct audit kind; move/home/read_state/paced_move stay "executed".
        outcome_kind = (
            "commissioned" if tool_name in _COMMISSION_OUTCOME_TOOLS else "executed"
        )
        return ActuatorOutcome(
            success=True,
            outcome_kind=outcome_kind,
            telemetry=dict(result) if isinstance(result, dict) else {"result": result},
        )
