# so-arm101-actuator

[![PyPI](https://img.shields.io/pypi/v/so-arm101-actuator.svg)](https://pypi.org/project/so-arm101-actuator/)

**Actuator Protocol driver for the SO-ARM101 arm.**
RPN-000000000002 · `pip install so-arm101-actuator`

```python
from so_arm101_actuator import SOArm101Actuator

actuator = SOArm101Actuator.from_default_port()  # /dev/ttyACM0 @ 1 Mbps
actuator.home()
actuator.move({"shoulder_pan": 0.3})
print(actuator.read_state())
```

## Capabilities

- `move(joint_positions, *, timeout_s=5.0)` — drive named joints to target radians; blocks until in tolerance or timeout.
- `home(*, timeout_s=10.0)` — move to the configured zero pose.
- `read_state()` — snapshot positions + best-effort motor temperatures.

## What this is

This is the **first real actuator driver** registered against the
post-2026-05-09-reset RobotRegistryFoundation. It bridges Hugging Face /
The Robot Studio's SO-ARM101 hardware to `robot-md-gateway`'s Actuator
Protocol via entry-point dispatch.

For a step-by-step install + register + run walkthrough, see the
[robot-md cookbook beat 8](https://robotmd.dev/cookbook/beat-8/) (will land
in Phase 3 of the roadmap).

For one operator's full setup story, see the
[Bob case study](https://robotmd.dev/case-studies/) (also Phase 3).

## Examples

```bash
python -m so_arm101_actuator.examples.wave
python -m so_arm101_actuator.examples.move_to_home
python -m so_arm101_actuator.examples.read_state
```

## Architecture

Two layers:

- `protocol.py` — pure SCS/STS wire protocol on a `serial.Serial`-like.
- `actuator.py` — Actuator Protocol implementation; depends only on `protocol.py`.

This separation lets you swap the protocol layer (alternative servo lib, simulator)
without touching capability code.

## Hardware tests

```bash
SO_ARM101_HARDWARE=1 pytest tests/test_hardware.py -v
```

Skipped by default. Run on real hardware only.

## Operator overrides (v0.2.0+)

Two env vars allow operator-specific tuning without code changes. Both are JSON, partially merged with defaults.

```bash
# Override HOME_POSE_RAD for one joint (e.g., gravity-load on shoulder_lift)
export SO_ARM101_HOME_POSE_RAD='{"shoulder_lift": 0.10}'

# Tighten SAFE_RANGE_RAD for shoulder_pan
export SO_ARM101_SAFE_RANGE_RAD='{"shoulder_pan": [-0.5, 0.5]}'
```

`HOME_POSE_RAD` can also be set via the `home_pose_rad=` constructor kwarg (kwarg overrides env).

## Sweep CLI (v0.2.0+)

A developer-facing range-of-motion test that drives all 6 joints with random poses and prints a live status table.

```bash
# 100 iterations, default seed
python -m so_arm101_actuator.sweep

# Dry-run (no hardware): print 10 planned poses to JSONL
python -m so_arm101_actuator.sweep --dry-run --iterations 10 --seed 42 --out /tmp/poses.jsonl
```

The sweep CLI is for hands-on hardware bring-up + demos. Cert evidence (`bob.local/FULL-SWEEP-100`) does NOT come from this CLI; it comes from `opencastor-ops/scripts/hil/` orchestrating pre-signed envelopes through `robot-md-gateway`.

## Calibration (v0.2.1+)

The default `SAFE_RANGE_RAD` and `MOVE_TOLERANCE_RAD` values reflect calibration measurements taken on a specific SO-ARM101 rig (Bob) on 2026-05-10. Different physical rigs will have different mechanical stops and steady-state precision. Operators should:

```bash
# Override SAFE_RANGE_RAD per joint after measuring your own rig's reachable range
export SO_ARM101_SAFE_RANGE_RAD='{"elbow_flex": [-0.30, 0.95]}'

# Loosen MOVE_TOLERANCE_RAD if your arm has larger steady-state error (gravity-loaded joints)
export SO_ARM101_MOVE_TOLERANCE_RAD=0.05
```

See the sweep CLI (`python -m so_arm101_actuator.sweep --iterations 5 --dry-run`) to generate a sample pose list for your rig.

## License

Apache-2.0.
