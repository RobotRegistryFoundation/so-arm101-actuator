"""Tests for operator-overridable HOME_POSE_RAD + SAFE_RANGE_RAD env vars."""

import json
import os

import pytest

from so_arm101_actuator import config


def test_resolve_home_pose_rad_no_override(monkeypatch):
    monkeypatch.delenv("SO_ARM101_HOME_POSE_RAD", raising=False)
    pose = config.resolve_home_pose_rad()
    assert pose == config.HOME_POSE_RAD
    assert pose["shoulder_lift"] == pytest.approx(0.10)


def test_resolve_home_pose_rad_partial_env_merges_with_defaults(monkeypatch):
    monkeypatch.setenv("SO_ARM101_HOME_POSE_RAD", json.dumps({"shoulder_pan": 0.05}))
    pose = config.resolve_home_pose_rad()
    assert pose["shoulder_pan"] == pytest.approx(0.05)
    assert pose["shoulder_lift"] == pytest.approx(0.10)  # default preserved


def test_resolve_home_pose_rad_invalid_json_raises(monkeypatch):
    monkeypatch.setenv("SO_ARM101_HOME_POSE_RAD", "{not-json}")
    with pytest.raises(ValueError, match="SO_ARM101_HOME_POSE_RAD"):
        config.resolve_home_pose_rad()


def test_resolve_home_pose_rad_unknown_joint_raises(monkeypatch):
    monkeypatch.setenv("SO_ARM101_HOME_POSE_RAD", json.dumps({"bogus_joint": 0.0}))
    with pytest.raises(ValueError, match="bogus_joint"):
        config.resolve_home_pose_rad()


def test_safe_range_default_inside_mechanical_limits():
    safe = config.SAFE_RANGE_RAD
    for joint, (lo, hi) in safe.items():
        spec = config.JOINTS[joint]
        assert spec["min_rad"] <= lo, f"{joint}: safe min {lo} below mech min {spec['min_rad']}"
        assert hi <= spec["max_rad"], f"{joint}: safe max {hi} above mech max {spec['max_rad']}"


def test_resolve_safe_range_rad_no_override(monkeypatch):
    monkeypatch.delenv("SO_ARM101_SAFE_RANGE_RAD", raising=False)
    safe = config.resolve_safe_range_rad()
    assert safe == config.SAFE_RANGE_RAD


def test_resolve_safe_range_rad_partial_override(monkeypatch):
    import json
    monkeypatch.setenv(
        "SO_ARM101_SAFE_RANGE_RAD",
        json.dumps({"shoulder_pan": [-0.5, 0.5]}),
    )
    safe = config.resolve_safe_range_rad()
    assert safe["shoulder_pan"] == (-0.5, 0.5)
    assert safe["wrist_roll"] == config.SAFE_RANGE_RAD["wrist_roll"]


def test_resolve_safe_range_rad_invalid_min_ge_max_raises(monkeypatch):
    import json
    monkeypatch.setenv("SO_ARM101_SAFE_RANGE_RAD", json.dumps({"shoulder_pan": [0.5, 0.5]}))
    with pytest.raises(ValueError, match="shoulder_pan"):
        config.resolve_safe_range_rad()


def test_resolve_safe_range_rad_outside_mechanical_raises(monkeypatch):
    import json
    monkeypatch.setenv("SO_ARM101_SAFE_RANGE_RAD", json.dumps({"shoulder_pan": [-99.0, 99.0]}))
    with pytest.raises(ValueError, match="mechanical"):
        config.resolve_safe_range_rad()


def test_actuator_uses_default_home_pose(monkeypatch):
    monkeypatch.delenv("SO_ARM101_HOME_POSE_RAD", raising=False)
    from so_arm101_actuator.actuator import SOArm101Actuator
    a = SOArm101Actuator(protocol=None)
    assert a.home_pose_rad["shoulder_lift"] == pytest.approx(0.10)


def test_actuator_kwarg_overrides_env(monkeypatch):
    import json
    monkeypatch.setenv("SO_ARM101_HOME_POSE_RAD", json.dumps({"shoulder_pan": 0.05}))
    from so_arm101_actuator.actuator import SOArm101Actuator
    a = SOArm101Actuator(protocol=None, home_pose_rad={"shoulder_pan": 0.20})
    assert a.home_pose_rad["shoulder_pan"] == pytest.approx(0.20)
    # other joints fall back to defaults (env partial override is overridden by kwarg, kwarg is the same partial form)
    assert a.home_pose_rad["shoulder_lift"] == pytest.approx(0.10)


def test_resolve_move_tolerance_rad_no_override(monkeypatch):
    monkeypatch.delenv("SO_ARM101_MOVE_TOLERANCE_RAD", raising=False)
    assert config.resolve_move_tolerance_rad() == pytest.approx(0.02)


def test_resolve_move_tolerance_rad_env_override(monkeypatch):
    monkeypatch.setenv("SO_ARM101_MOVE_TOLERANCE_RAD", "0.07")
    assert config.resolve_move_tolerance_rad() == pytest.approx(0.07)


def test_resolve_move_tolerance_rad_invalid_raises(monkeypatch):
    monkeypatch.setenv("SO_ARM101_MOVE_TOLERANCE_RAD", "not-a-float")
    with pytest.raises(ValueError, match="SO_ARM101_MOVE_TOLERANCE_RAD"):
        config.resolve_move_tolerance_rad()


def test_resolve_move_tolerance_rad_negative_raises(monkeypatch):
    monkeypatch.setenv("SO_ARM101_MOVE_TOLERANCE_RAD", "-0.01")
    with pytest.raises(ValueError, match="positive"):
        config.resolve_move_tolerance_rad()


def test_actuator_uses_env_move_tolerance(monkeypatch):
    monkeypatch.setenv("SO_ARM101_MOVE_TOLERANCE_RAD", "0.07")
    from so_arm101_actuator.actuator import SOArm101Actuator
    a = SOArm101Actuator(protocol=None)
    assert a.move_tolerance_rad == pytest.approx(0.07)


def test_actuator_kwarg_overrides_env_tolerance(monkeypatch):
    monkeypatch.setenv("SO_ARM101_MOVE_TOLERANCE_RAD", "0.07")
    from so_arm101_actuator.actuator import SOArm101Actuator
    a = SOArm101Actuator(protocol=None, move_tolerance_rad=0.15)
    assert a.move_tolerance_rad == pytest.approx(0.15)


def test_actuator_home_uses_resolved_pose(monkeypatch):
    """home() must use self.home_pose_rad (env/kwarg-resolved), not the module-level HOME_POSE_RAD."""
    import json
    monkeypatch.setenv("SO_ARM101_HOME_POSE_RAD", json.dumps({"shoulder_pan": 0.13}))
    from so_arm101_actuator import config
    from so_arm101_actuator.actuator import SOArm101Actuator

    class FakeProtocol:
        def __init__(self):
            self.last_ticks: dict[int, int] = {}

        def set_position(self, motor_id: int, ticks: int) -> None:
            self.last_ticks[motor_id] = ticks

        def read_position(self, motor_id: int) -> int:
            # Return whatever was last written so move()'s polling loop sees "reached" immediately.
            return self.last_ticks.get(motor_id, 2048)

    fake = FakeProtocol()
    a = SOArm101Actuator(protocol=fake)
    a.home()

    # shoulder_pan is motor_id=1; env overrides it to 0.13 rad.
    assert 1 in fake.last_ticks, "home() did not call set_position — protocol interface mismatch?"
    actual_rad = config.ticks_to_rad("shoulder_pan", fake.last_ticks[1])
    assert actual_rad == pytest.approx(0.13, abs=0.01), (
        f"home() used unresolved HOME_POSE_RAD (got {actual_rad:.4f} rad) "
        f"instead of self.home_pose_rad (env-overridden shoulder_pan=0.13)"
    )
