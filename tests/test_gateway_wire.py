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
from pathlib import Path
from unittest.mock import MagicMock

import pytest

from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
from fastapi.testclient import TestClient

from robot_md_gateway.cert.policy import ToolAllowlist
from robot_md_gateway.receiver import make_app

from so_arm101_actuator.actuator import SOArm101Actuator
from tests.conftest import FIXTURE_MANIFEST

KID = "so-arm101-test-manifest"
BEARER = "test-actuate-token"
READ_BEARER = "test-read-token"

#: Bob's live policy, in the shape the operator writes it. arm.state sits with
#: status.report; arm.move_to sits with arm.reach.
TOOL_TIERS = {
    "arm.move_to": frozenset({"actuate", "commission"}),
    "arm.state": frozenset({"read", "actuate", "commission"}),
    "status.report": frozenset({"read", "actuate", "commission"}),
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
            "arm.home", "arm.reach", "arm.move_to", "arm.state", "status.report")),
        tool_tier_requirements=TOOL_TIERS,
        bearer_tiers={BEARER: "actuate", READ_BEARER: "read"},
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


def test_arm_move_to_on_the_wire(client, monkeypatch):
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
    assert telemetry["waypoints"] == 2
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
