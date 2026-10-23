"""The exact bytes a client sends and gets back for arm.move_to / arm.state.

Every other test in this repo calls the driver directly. This one puts the real
robot-md-gateway in front of it — real manifest-provenance check, real tier
gate, real tool allowlist, real signed receipt — because the documented wire
format is a promise to clients, and a promise nothing exercises is a guess.

Still no hardware: the servo bus is the same MagicMock the rest of the suite
uses, and the manifest is signed at test time with a throwaway key.
"""

from __future__ import annotations

import base64
import threading
import time
from pathlib import Path
from unittest.mock import MagicMock

import pytest

from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
from fastapi.testclient import TestClient

from robot_md_gateway.cert.audit import AuditChain
from robot_md_gateway.cert.policy import ToolAllowlist
from robot_md_gateway.cert.safety import GatewayState, SafetyMonitor
from robot_md_gateway.receiver import make_app

from so_arm101_actuator import config as arm_config
from so_arm101_actuator.actuator import SOArm101Actuator
from tests.conftest import FIXTURE_MANIFEST

KID = "so-arm101-test-manifest"
BEARER = "test-actuate-token"
READ_BEARER = "test-read-token"
COMMISSION_BEARER = "test-commission-token"

#: Bob's live policy, in the shape the operator writes it. arm.state sits with
#: status.report; arm.move_to sits with arm.reach.
TOOL_TIERS = {
    "arm.move_to": frozenset({"actuate", "commission"}),
    "arm.state": frozenset({"read", "actuate", "commission"}),
    "status.report": frozenset({"read", "actuate", "commission"}),
    # Anyone who can reach this arm may stop it; only the bring-up bearer may
    # make it movable again.
    "arm.estop": frozenset({"read", "actuate", "commission"}),
    "arm.estop.clear": frozenset({"commission"}),
}


@pytest.fixture
def signed_manifest(tmp_path) -> tuple[Path, bytes]:
    """The fixture geometry, signed the way a real ROBOT.md is.

    The signed body is everything before the footer's leading newline — the
    canonicalization `robot_md_gateway.manifest_provenance` verifies against.
    """
    key = Ed25519PrivateKey.generate()
    body = Path(FIXTURE_MANIFEST).read_text()
    signature = base64.b64encode(key.sign(body.encode("utf-8"))).decode("ascii")
    path = tmp_path / "ROBOT.md"
    path.write_text(f"{body}\n<!-- ROBOT-MD-SIG kid={KID} sig={signature} -->\n")
    pub_pem = key.public_key().public_bytes(
        encoding=serialization.Encoding.PEM,
        format=serialization.PublicFormat.SubjectPublicKeyInfo,
    )
    return path, pub_pem


@pytest.fixture
def client(signed_manifest):
    path, pub_pem = signed_manifest

    class _Resolver:
        def resolve_public_key_pem(self, kid: str) -> bytes | None:
            return pub_pem if kid == KID else None

    proto = MagicMock()
    state = {i: 2048 for i in range(1, 7)}
    proto.set_position.side_effect = lambda motor_id, ticks: state.__setitem__(motor_id, ticks)
    proto.read_position.side_effect = lambda motor_id: state.get(motor_id, 2048)
    proto.read_temperature.return_value = 30

    app = make_app(
        resolver=_Resolver(),
        tool_allowlist=ToolAllowlist(allowed_tools=(
            "arm.home", "arm.reach", "arm.move_to", "arm.state", "status.report",
            "arm.estop", "arm.estop.clear")),
        tool_tier_requirements=TOOL_TIERS,
        bearer_tiers={BEARER: "actuate", READ_BEARER: "read",
                      COMMISSION_BEARER: "commission"},
        actuator=SOArm101Actuator(protocol=proto),
    )
    with TestClient(app) as http:
        yield http, str(path)


def _envelope(tool_name: str, tool_args: dict, manifest: str, *,
              scope: str = "MANIPULATE") -> dict:
    return {
        "msg_id": "eval-0001",
        "type": "rcan/v1/invoke",
        "ruri": "rcan://RRN-000000000000/arm",
        "scope": scope,
        "tool_name": tool_name,
        "tool_args": tool_args,
        "manifest_path": manifest,
    }


def test_arm_move_to_on_the_wire(client, monkeypatch, fake_clock):
    """The documented request and response for a move that executes."""
    monkeypatch.setenv("SO_ARM101_SAFE_RANGE_RAD",
                       '{"shoulder_lift": [-1.45, 1.0], "elbow_flex": [-0.19, 1.5],'
                       ' "wrist_flex": [-0.93, 1.5]}')
    http, manifest = client

    response = http.post(
        "/v1/invoke",
        json=_envelope("arm.move_to",
                       {"x_mm": 150.0, "y_mm": 0.0, "z_mm": 50.0, "speed": 0.5},
                       manifest),
        headers={"Authorization": f"Bearer {BEARER}"},
    )

    assert response.status_code == 200, response.text
    body = response.json()
    assert body["ok"] is True
    assert body["manifest_kid"] == KID
    assert body["scope"] == "MANIPULATE"
    assert body["tool_name"] == "arm.move_to"
    assert body["actuator_name"] == "so-arm101"
    assert body["outcome_kind"] == "executed"

    telemetry = body["telemetry"]
    assert telemetry["reached"] is True
    assert telemetry["eef_mm"]["x"] == pytest.approx(150.0, abs=0.5)
    assert telemetry["eef_mm"]["z"] == pytest.approx(50.0, abs=0.5)
    assert telemetry["speed"] == 0.5
    # Paced: a stream of setpoints under half the declared tool limit, and the
    # receipt says how the move was paced and how close its path came to a face.
    assert telemetry["waypoints"] == telemetry["motion"]["setpoints"] > 1
    assert telemetry["motion"]["tool_speed_limit_mps"] == pytest.approx(0.125)
    # This rig starts parked with every servo at mid-travel, which puts the tip
    # past x = 340; the move is allowed because it ends well inside and never
    # gets more than motion.RECOVERY_DIP_MM further out on the way.
    assert telemetry["motion"]["path_min_clearance_mm"] is not None
    assert telemetry["motion"]["path_closest"]["point"] in ("tip", "wrist", "elbow")
    assert telemetry["ik_provider"] == "inhouse-so-arm101"
    assert set(telemetry["final_positions"]) == {
        "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"}

    # Unattested here only because this test app configures no signing identity;
    # on a deployed gateway this carries the Ed25519 receipt.
    assert body["attestation"] in ("attested", "unattested")
    assert "envelope_signature" in body


def test_arm_state_on_the_wire(client):
    """The documented request and response for the read-only pose snapshot,
    from a READ-tier bearer under OBSERVE — the same class as status.report."""
    http, manifest = client

    response = http.post(
        "/v1/invoke",
        json=_envelope("arm.state", {}, manifest, scope="OBSERVE"),
        headers={"Authorization": f"Bearer {READ_BEARER}"},
    )

    assert response.status_code == 200, response.text
    body = response.json()
    assert body["ok"] is True
    assert body["tool_name"] == "arm.state"
    assert body["outcome_kind"] == "executed"

    telemetry = body["telemetry"]
    assert set(telemetry) == {"joint_positions_rad", "eef_mm", "tool"}
    assert set(telemetry["joint_positions_rad"]) == {
        "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex",
        "wrist_roll", "gripper"}
    assert set(telemetry["eef_mm"]) == {"x", "y", "z"}
    assert telemetry["tool"] == "gripper"


def test_an_unreachable_target_comes_back_as_a_signed_403(client):
    """The refusal path a harness must handle: HTTP 403, `deny:
    actuator_policy` from the gateway, and the driver's own code nested under
    `telemetry`. Nothing moved."""
    http, manifest = client

    response = http.post(
        "/v1/invoke",
        json=_envelope("arm.move_to", {"x_mm": 500.0, "y_mm": 0.0, "z_mm": 50.0},
                       manifest),
        headers={"Authorization": f"Bearer {BEARER}"},
    )

    assert response.status_code == 403
    detail = response.json()["detail"]
    assert detail["deny"] == "actuator_policy"
    assert detail["actuator_name"] == "so-arm101"
    assert detail["telemetry"]["deny"] == "out_of_workspace"
    assert detail["reason"].startswith("out_of_workspace: ")
    assert "attestation" in detail


def test_a_read_bearer_cannot_move_the_arm(client):
    """Two independent gates say no: the gateway's scope/tier gate, and the
    driver's own tool/tier binding behind it."""
    http, manifest = client

    response = http.post(
        "/v1/invoke",
        json=_envelope("arm.move_to", {"x_mm": 150.0, "y_mm": 0.0, "z_mm": 50.0},
                       manifest),
        headers={"Authorization": f"Bearer {READ_BEARER}"},
    )

    assert response.status_code == 403
    assert response.json()["detail"]["deny"] in ("tier_policy", "tool_tier")


def test_an_unauthenticated_caller_cannot_read_the_pose(client):
    """anon is not a tier that observes this robot. The driver refuses it even
    if an operator's allowlist ever stopped doing so."""
    http, manifest = client

    response = http.post(
        "/v1/invoke",
        json=_envelope("arm.state", {}, manifest, scope="OBSERVE"),
    )

    assert response.status_code in (403, 500)


def test_arm_estop_on_the_wire_from_a_read_bearer(client):
    """The stop, over HTTP, from the least-privileged bearer there is.

    This is the shape the runtime process is in: it holds the READ bearer and
    nothing else, and a stop it cannot reach is not a stop. Before this change
    `arm.estop` was in no capability list, no tier table and no dispatch branch,
    so this request came back as a signed DENY.
    """
    http, manifest = client

    response = http.post(
        "/v1/invoke",
        json=_envelope("arm.estop", {}, manifest, scope="SAFETY"),
        headers={"Authorization": f"Bearer {READ_BEARER}"},
    )

    assert response.status_code == 200, response.text
    body = response.json()
    assert body["ok"] is True
    assert body["tool_name"] == "arm.estop"
    assert body["outcome_kind"] == "executed"
    assert body["telemetry"]["estopped"] is True
    # The receipt carries the caveat, so nobody reads "e-stop" on a signed
    # record and infers a hardware interlock this arm does not have.
    assert "NOT a hardware e-stop" in body["telemetry"]["safety_note"]
    assert body["attestation"] in ("attested", "unattested")


def test_motion_after_the_estop_is_a_signed_403(client):
    """The latch, seen from the wire: motion comes back as a signed refusal
    carrying `estop_latched`, not as a 500 that reads like a broken robot."""
    http, manifest = client

    stop = http.post(
        "/v1/invoke",
        json=_envelope("arm.estop", {}, manifest, scope="SAFETY"),
        headers={"Authorization": f"Bearer {READ_BEARER}"},
    )
    assert stop.status_code == 200, stop.text

    response = http.post(
        "/v1/invoke",
        json=_envelope("arm.move_to", {"x_mm": 150.0, "y_mm": 0.0, "z_mm": 50.0},
                       manifest),
        headers={"Authorization": f"Bearer {BEARER}"},
    )

    assert response.status_code == 403
    detail = response.json()["detail"]
    assert detail["deny"] == "actuator_policy"
    assert detail["telemetry"]["deny"] == "estop_latched"


def test_clearing_the_estop_needs_the_commission_bearer(client):
    """Read tier stops this arm and cannot start it: the clear is refused with
    the read bearer, and the arm stays latched through the refusal."""
    http, manifest = client
    http.post("/v1/invoke",
              json=_envelope("arm.estop", {}, manifest, scope="SAFETY"),
              headers={"Authorization": f"Bearer {READ_BEARER}"})

    refused = http.post(
        "/v1/invoke",
        json=_envelope("arm.estop.clear", {}, manifest, scope="SAFETY"),
        headers={"Authorization": f"Bearer {READ_BEARER}"},
    )
    assert refused.status_code == 403, refused.text

    still_latched = http.post(
        "/v1/invoke",
        json=_envelope("arm.move_to", {"x_mm": 150.0, "y_mm": 0.0, "z_mm": 50.0},
                       manifest),
        headers={"Authorization": f"Bearer {BEARER}"},
    )
    assert still_latched.status_code == 403
    assert still_latched.json()["detail"]["telemetry"]["deny"] == "estop_latched"

    cleared = http.post(
        "/v1/invoke",
        json=_envelope("arm.estop.clear", {}, manifest, scope="SAFETY"),
        headers={"Authorization": f"Bearer {COMMISSION_BEARER}"},
    )
    assert cleared.status_code == 200, cleared.text
    assert cleared.json()["telemetry"]["estopped"] is False


# --------------------------------------------------------------------------- #
# Refusals and stops behind the gateway's safety layer
# (review of robot-md-gateway#35: Q4, P1 and P2)
# --------------------------------------------------------------------------- #

#: The measured safe ranges widened the way an operator does for a tool-down
#: move: the same values test_arm_move_to_on_the_wire sets.
WIDE_SAFE_RANGES = ('{"shoulder_lift": [-1.45, 1.0], "elbow_flex": [-0.19, 1.5],'
                    ' "wrist_flex": [-0.93, 1.5]}')

TARGET = {"x_mm": 150.0, "y_mm": 0.0, "z_mm": 50.0}


def _logging_bus(*, moving_after: int | None = None):
    """A simulated servo bus: each servo reads back the goal it was last sent,
    as position and as its Goal_Position register. Every write is logged as
    (motor_id, ticks). `moving` is set once `moving_after` writes have gone
    out, so a test can act while a paced move is streaming."""
    proto = MagicMock()
    state = {i: 2048 for i in range(1, 7)}
    writes: list[tuple[int, int]] = []
    moving = threading.Event()

    def write(motor_id, ticks):
        state[motor_id] = ticks
        writes.append((motor_id, ticks))
        if moving_after is not None and len(writes) >= moving_after:
            moving.set()

    proto.set_position.side_effect = write
    proto.read_position.side_effect = lambda motor_id: state[motor_id]
    proto.read_goal_position.side_effect = lambda motor_id: state[motor_id]
    proto.read_temperature.return_value = 30
    return proto, state, writes, moving


def _gateway(signed_manifest, proto, *, chain: AuditChain,
             monitor: SafetyMonitor | None = None):
    """The real gateway in front of this driver, with NO per-tool tier
    requirements: an operator who allowlists the stop and binds no tier to it.
    Whatever tier refusal happens is the driver's own."""
    path, pub_pem = signed_manifest

    class _Resolver:
        def resolve_public_key_pem(self, kid: str) -> bytes | None:
            return pub_pem if kid == KID else None

    arm = SOArm101Actuator(protocol=proto)
    app = make_app(
        resolver=_Resolver(),
        tool_allowlist=ToolAllowlist(allowed_tools=(
            "arm.move_to", "arm.reach", "arm.state", "arm.estop", "arm.estop.clear")),
        tool_tier_requirements={},
        bearer_tiers={BEARER: "actuate", READ_BEARER: "read",
                      COMMISSION_BEARER: "commission"},
        audit_chain=chain,
        actuator=arm,
        **({"safety_monitor": monitor} if monitor is not None else {}),
    )
    return app, arm, str(path)


def _motors(held: dict[str, int]) -> list[tuple[int, int]]:
    """A hold as the (motor_id, ticks) writes it is sent as."""
    return sorted((arm_config.JOINTS[joint]["motor_id"], ticks) for joint, ticks in held.items())


def _gateway_stops(chain: AuditChain) -> list:
    return [e for e in chain.entries if e.decision_reason.startswith("safety_stop:")]


def _wait_for(predicate, timeout: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


def _skip_without_the_safety_watchdog() -> None:
    """robot-md-gateway#35 added the watchdog that sends each actuator its
    declared stop when the gateway trips, with /v1/safety/heartbeat. A gateway
    without that route refuses requests while tripped and stops nothing, so
    there is no trip stop to put this driver behind."""

    class _NoKeys:
        def resolve_public_key_pem(self, kid: str) -> bytes | None:
            return None

    routes = {getattr(route, "path", None) for route in make_app(resolver=_NoKeys()).routes}
    if "/v1/safety/heartbeat" not in routes:
        pytest.skip("this robot-md-gateway does not stop its actuators on a trip "
                    "(no /v1/safety/heartbeat; robot-md-gateway#35)")


@pytest.mark.parametrize("tool_name,token,scope", [
    ("arm.estop", None, "SAFETY"),
    ("arm.estop.clear", READ_BEARER, "SAFETY"),
    ("arm.estop.clear", BEARER, "SAFETY"),
    ("arm.state", None, "OBSERVE"),
], ids=["anonymous-stop", "read-clear", "actuate-clear", "anonymous-read"])
def test_a_driver_tier_refusal_is_a_signed_403_recorded_like_its_other_refusals(
        signed_manifest, tool_name, token, scope):
    """The driver's own tier binding saying no, seen from the wire. No tier is
    bound to these tools at the gateway, so the request reaches the driver,
    except an anonymous declared stop on robot-md-gateway#35 from 3020f7f,
    which the gateway refuses itself (below). The driver's refusal of that one
    is pinned by test_actuator.py's arm.estop-anon case.

    This used to come back as HTTP 500, with the audit chain recording
    decision "allow", reason "ok" and outcome "error" (review of
    robot-md-gateway#35, P2). An anonymous caller still may not stop the arm,
    and a read or actuate bearer still may not clear it; what changed is that
    the refusal is a decision, recorded the way the driver's other refusals
    are."""
    proto, _, writes, _ = _logging_bus()
    chain = AuditChain()
    app, arm, manifest = _gateway(signed_manifest, proto, chain=chain)
    headers = {"Authorization": f"Bearer {token}"} if token else {}
    with TestClient(app) as http:
        # A refusal this driver has always returned as `denied`, for reference.
        # It is refused before the bus is touched, like the tier refusal.
        reference = http.post("/v1/invoke", headers={"Authorization": f"Bearer {BEARER}"},
                              json=_envelope("arm.move_to", {**TARGET, "x_mm": 500.0}, manifest))
        assert reference.status_code == 403, reference.text
        assert reference.json()["detail"]["telemetry"]["deny"] == "out_of_workspace"
        workspace = chain.entries[-1]
        response = http.post("/v1/invoke", headers=headers,
                             json=_envelope(tool_name, {}, manifest, scope=scope))

    assert response.status_code == 403, response.text
    detail = response.json()["detail"]
    assert "attestation" in detail
    entry = chain.entries[-1]
    if detail["deny"] == "tier_policy":
        # robot-md-gateway#35 from 3020f7f refuses an anonymous DECLARED STOP
        # itself, before dispatch, so the driver is never asked. Still a signed
        # 403 recorded as a deny, with nothing on the bus and nothing latched.
        assert (tool_name, token) == ("arm.estop", None)
        assert entry.decision == "deny" and entry.actuator_outcome_kind is None
        assert proto.method_calls == [] and writes == [] and arm._latched() is False
        return
    assert detail["deny"] == "actuator_policy"
    assert detail["actuator_name"] == "so-arm101"
    assert detail["telemetry"]["deny"] == "tool_tier"
    assert detail["reason"].startswith("tool_tier: ")
    assert entry.actuator_outcome_kind == "denied"
    # Recorded exactly as the out_of_workspace refusal was. robot-md-gateway#35
    # records a driver's refusal as a deny; gateways before it (0.5.0a9 on
    # PyPI) still sign the 403 but write decision "allow", reason "ok" for
    # every driver refusal, this one included.
    assert entry.decision == workspace.decision
    if workspace.decision == "deny":
        assert entry.decision_reason.startswith("actuator_policy: tool_tier: ")
    # Refused before the bus: nothing read, nothing written, nothing latched.
    assert proto.method_calls == []
    assert writes == []
    assert arm._latched() is False


def test_an_unknown_reach_target_is_a_signed_403_recorded_like_its_other_refusals(
        signed_manifest):
    """arm.reach with a target it does not know, from an actuate bearer. It
    used to come back as HTTP 500, recorded as an allowed call that failed
    (review of so-arm101-actuator#9, finding 2)."""
    proto, _, writes, _ = _logging_bus()
    chain = AuditChain()
    app, arm, manifest = _gateway(signed_manifest, proto, chain=chain)
    actuate = {"Authorization": f"Bearer {BEARER}"}
    with TestClient(app) as http:
        reference = http.post("/v1/invoke", headers=actuate,
                              json=_envelope("arm.move_to", {**TARGET, "x_mm": 500.0}, manifest))
        assert reference.status_code == 403, reference.text
        workspace = chain.entries[-1]
        response = http.post("/v1/invoke", headers=actuate,
                             json=_envelope("arm.reach", {"target": "nowhere"}, manifest))

    assert response.status_code == 403, response.text
    detail = response.json()["detail"]
    assert detail["deny"] == "actuator_policy"
    assert detail["telemetry"]["deny"] == "bad_args"
    assert "attestation" in detail
    entry = chain.entries[-1]
    assert entry.actuator_outcome_kind == "denied"
    assert entry.decision == workspace.decision          # a deny from robot-md-gateway#35 on
    assert proto.method_calls == [] and writes == []
    assert arm._latched() is False


def test_a_trip_ends_a_paced_move_and_the_real_driver_holds(signed_manifest, monkeypatch):
    """robot-md-gateway#35's safe-stop tests use a simulated arm. This is the
    same scenario with this driver behind the gateway, over a simulated bus.

    The heartbeat goes stale while arm.move_to is streaming. The gateway sends
    arm.estop itself (no caller; the read tier). The move ends at its next
    setpoint, the hold goes out after it, and motion stays refused: by the
    gateway while it is tripped, and by the driver's own latch.

    Real time, not the fake clock: the trip comes from the gateway's watchdog
    thread, so the move has to be paced on the same clock the watchdog reads.
    """
    _skip_without_the_safety_watchdog()
    monkeypatch.setenv("SO_ARM101_SAFE_RANGE_RAD", WIDE_SAFE_RANGES)
    proto, state, writes, moving = _logging_bus(moving_after=10)    # two setpoints in
    chain = AuditChain()
    # READY explicitly, and heartbeats sent straight to the monitor: this test
    # is about the stop. Who may send a heartbeat, and the state a gateway
    # boots in, are both changes the review asks robot-md-gateway to make.
    monitor = SafetyMonitor(state=GatewayState.READY, heartbeat_staleness_s=1.0)
    app, arm, manifest = _gateway(signed_manifest, proto, chain=chain, monitor=monitor)
    move = _envelope("arm.move_to", {**TARGET, "speed": 0.1}, manifest)
    actuate = {"Authorization": f"Bearer {BEARER}"}

    with TestClient(app) as http:
        supervised = threading.Event()

        def supervise():
            # The supervising client is alive until the move is under way,
            # then its link drops.
            while not (moving.is_set() or supervised.is_set()):
                monitor.on_heartbeat()
                time.sleep(0.05)

        threading.Thread(target=supervise, daemon=True).start()
        result: dict = {}
        mover = threading.Thread(
            target=lambda: result.update(r=http.post("/v1/invoke", json=move, headers=actuate)))
        mover.start()
        try:
            assert moving.wait(10), "the move never started streaming"
            mover.join(timeout=60)
        finally:
            supervised.set()
        assert not mover.is_alive(), "nothing ended the move"

        response = result["r"]
        assert response.status_code == 200, response.text
        moved = response.json()["telemetry"]
        assert moved["motion"]["stopped_by_estop"] is True
        assert moved["reached"] is False
        assert monitor.state is GatewayState.SAFE_STOP

        # The gateway's own stop, once, executed by this driver. It is recorded
        # after the driver returns, so its record means the hold has gone out.
        assert _wait_for(lambda: _gateway_stops(chain)), "the gateway recorded no stop"
        [stop] = _gateway_stops(chain)
        assert stop.decision == "allow"
        assert "sent arm.estop to so-arm101" in stop.decision_reason
        assert stop.actuator_outcome_kind == "executed"
        assert (stop.caller, stop.tier) == (None, None)

        # The hold went out after the move's last setpoint: every joint once,
        # the gripper included (arm.move_to never commands it), at the goals
        # the latch chose. Nothing has written to the bus since.
        assert arm._latched() is True
        held = dict(arm._held)
        assert set(held) == set(arm_config.JOINTS)
        assert sorted(writes[-6:]) == _motors(held)
        assert sorted(state.items()) == _motors(held)
        hold_writes = len(writes)
        time.sleep(0.3)
        assert len(writes) == hold_writes

        # Ended by the stop, not by running out of setpoints: at speed 0.1 the
        # move was planned as hundreds of them (53 of 704 go out in a normal
        # run). Counted in setpoints streamed, not seconds: a process that
        # stalls streams nothing while it is stalled, so only a stop that comes
        # late to a live process can fail this.
        setpoint_writes = writes[:-len(arm_config.JOINTS)]          # all but the hold
        per = len({motor for motor, _ in setpoint_writes})          # joints per setpoint
        streamed = len(setpoint_writes) / per
        assert streamed < moved["motion"]["setpoints"] / 2, (streamed, moved["motion"])

        # A new move is refused by the tripped gateway, and recorded as a deny.
        refused = http.post("/v1/invoke", json=move, headers=actuate)
        assert refused.status_code == 403, refused.text
        assert refused.json()["detail"]["deny"] == "safety_state"
        assert chain.entries[-1].decision == "deny"

    # And by the driver's own latch, which does not depend on the gateway's
    # state: a move that reaches the driver (once the gateway is cleared, or
    # with no gateway at all) is refused until arm.estop.clear.
    own = arm.execute(envelope=move, manifest_path=Path(manifest), tier="actuate", config={})
    assert own.outcome_kind == "denied"
    assert own.telemetry["deny"] == "estop_latched"
    assert len(writes) == hold_writes


def test_on_a_tripped_gateway_a_bearer_stop_holds_and_an_anonymous_one_is_a_deny(
        signed_manifest):
    """After a trip, with nothing moving.

    A read-bearer arm.estop passes the tripped gateway and re-sends exactly
    the goals the gateway's stop chose: a repeated stop does not ratchet.

    An anonymous arm.estop is a signed 403 recorded as a deny, with nothing on
    the bus. Up to robot-md-gateway#35 at 74bff6e it passes the gateway (no
    tier is bound to it there) and the driver refuses it; it used to be a 500
    recorded as allow/ok, tripped or not (review of robot-md-gateway#35, P2).
    From 3020f7f the gateway refuses it itself, before dispatch."""
    _skip_without_the_safety_watchdog()
    proto, _, writes, _ = _logging_bus()
    chain = AuditChain()
    monitor = SafetyMonitor(state=GatewayState.READY, heartbeat_staleness_s=0.3)
    app, arm, manifest = _gateway(signed_manifest, proto, chain=chain, monitor=monitor)
    stop = _envelope("arm.estop", {}, manifest, scope="SAFETY")

    with TestClient(app) as http:
        assert _wait_for(lambda: _gateway_stops(chain)), "the gateway never tripped"
        assert monitor.state is GatewayState.SAFE_STOP
        held = dict(arm._held)
        before = len(writes)

        operator = http.post("/v1/invoke", json=stop,
                             headers={"Authorization": f"Bearer {READ_BEARER}"})
        assert operator.status_code == 200, operator.text
        assert operator.json()["telemetry"]["estopped"] is True
        assert arm._held == held
        assert sorted(writes[before:]) == _motors(held)

        after = len(writes)
        anonymous = http.post("/v1/invoke", json=stop)
        assert anonymous.status_code == 403, anonymous.text
        detail = anonymous.json()["detail"]
        entry = chain.entries[-1]
        assert entry.decision == "deny"
        # The driver's refusal, or (robot-md-gateway#35 from 3020f7f) the
        # gateway's own, before dispatch.
        assert detail["deny"] in ("actuator_policy", "tier_policy")
        if detail["deny"] == "actuator_policy":
            assert detail["telemetry"]["deny"] == "tool_tier"
            assert entry.actuator_outcome_kind == "denied"
        assert len(writes) == after
