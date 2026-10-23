"""castor-hal is optional: the robot-md-gateway entry point runs without it.

0.3.0 could not be installed from PyPI while it required castor-hal, which is
not on PyPI. Making castor-hal an extra was not enough on its own: the stop's
latch lived in the castor-hal adapter, so with castor-hal absent `arm.estop`
raised ModuleNotFoundError before it latched, and so did every motion, which
consults the latch first.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

from tests.conftest import FIXTURE_MANIFEST

ROOT = Path(__file__).resolve().parent.parent

#: Runs in a fresh interpreter that cannot import castor-hal, installed or not.
#: Prints "ok" when every check has passed.
_WITHOUT_CASTOR_HAL = r'''
import importlib.abc
import sys


class _NoCastorHal(importlib.abc.MetaPathFinder):
    def find_spec(self, name, path=None, target=None):
        if name == "castor_hal" or name.startswith("castor_hal."):
            raise ModuleNotFoundError(f"No module named {name!r}", name=name)
        return None


sys.meta_path.insert(0, _NoCastorHal())

from importlib.metadata import entry_points
from unittest.mock import MagicMock

(ep,) = [e for e in entry_points(group="robot_md_gateway.actuators") if e.name == "so-arm101"]
cls = ep.load()                                         # what robot-md-gateway does

import so_arm101_actuator.actuator as act


class _Clock:                                           # pacing without the waiting
    now = 1000.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += max(0.0, seconds)


act.time = _Clock()

state = {i: 2048 for i in range(1, 7)}
proto = MagicMock()
proto.read_position.side_effect = lambda motor_id: state[motor_id]
proto.read_goal_position.side_effect = lambda motor_id: state[motor_id]
proto.read_temperature.return_value = 30
proto.set_position.side_effect = lambda motor_id, ticks: state.__setitem__(motor_id, ticks)
arm = cls(protocol=proto)


def call(tool, tier, args=None, manifest=""):
    return arm.execute(envelope={"tool_name": tool, "tool_args": args or {}},
                       manifest_path=manifest, tier=tier, config={})


out = call("arm.estop", "read")
assert out.success and out.telemetry["estopped"] is True, out
assert proto.set_position.call_count == 6, proto.set_position.call_count     # every joint held
out = call("arm.home", "actuate")
assert out.outcome_kind == "denied" and out.telemetry["deny"] == "estop_latched", out
assert call("arm.estop.clear", "commission").success
out = call("move", "actuate", {"joint_positions": {"shoulder_pan": 0.3}}, manifest=sys.argv[1])
assert out.success, out
motion = out.telemetry["motion"]
assert motion["setpoints"] > 1, motion                                       # paced
assert motion["path_min_clearance_mm"] is not None, motion                   # checked
assert not [m for m in sys.modules if m == "castor_hal" or m.startswith("castor_hal.")]

try:
    import so_arm101_actuator.transport  # noqa: F401
except ModuleNotFoundError as exc:
    assert "git+https://github.com/RobotRegistryFoundation/castor-hal" in str(exc), exc
else:
    raise AssertionError("the castor-hal adapter imported without castor-hal")
print("ok")
'''


def test_the_gateway_entry_point_runs_without_castor_hal():
    """Load the entry point the way robot-md-gateway does, in an interpreter
    that cannot import castor-hal, then stop, refuse, clear, and make one
    checked, paced move."""
    run = subprocess.run([sys.executable, "-c", _WITHOUT_CASTOR_HAL, FIXTURE_MANIFEST],
                         capture_output=True, text=True, timeout=120, cwd=ROOT, check=False)
    assert run.returncode == 0 and run.stdout.strip().endswith("ok"), run.stderr or run.stdout


def test_castor_hal_is_neither_a_dependency_nor_an_extra():
    """castor-hal is not on PyPI. Naming it, even in an extra, would let whoever
    registers the name there install code onto the machine that drives the arm."""
    tomllib = pytest.importorskip("tomllib")            # Python 3.11+
    project = tomllib.loads((ROOT / "pyproject.toml").read_text())["project"]
    named = list(project["dependencies"])
    for extra in project.get("optional-dependencies", {}).values():
        named += extra
    assert not [d for d in named if d.lower().replace("_", "-").startswith("castor-hal")]
