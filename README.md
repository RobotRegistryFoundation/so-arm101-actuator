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
- `move_to(*, x_mm, y_mm, z_mm, speed=1.0)` — put the tool tip on a point in the
  arm's base frame, tool pointing straight down (v0.3.0+).
- `state()` — where the joints are, where that puts the tip, and what is on the
  end of the arm (v0.3.0+).
- `reach_point(target_mm, *, tolerance_mm=5.0)` — get to a point by measuring
  and stepping rather than solving, at whatever tilt the arm can manage.

### RCAN tool names

The gateway invokes these by their ROBOT.md capability names:

| Tool | Scope | Tiers | Driver method |
|---|---|---|---|
| `arm.home` | MANIPULATE | actuate, commission | `home` |
| `arm.reach` | MANIPULATE | actuate, commission | `move` (to the taught pose) |
| `arm.reach_point` | MANIPULATE | actuate, commission | `reach_point` |
| `arm.move_to` | MANIPULATE | actuate, commission | `move_to` |
| `arm.state` | OBSERVE | read, actuate, commission | `state` |
| `status.report` | OBSERVE | read, actuate, commission | `read_state` |

## Cartesian control (v0.3.0+)

### `arm.move_to` — solve, or refuse

Request to `POST /v1/invoke`:

```json
{
  "msg_id": "eval-0001",
  "type": "rcan/v1/invoke",
  "ruri": "rcan://RRN-000000000011/arm",
  "scope": "MANIPULATE",
  "tool_name": "arm.move_to",
  "tool_args": {"x_mm": 150.0, "y_mm": 0.0, "z_mm": 50.0, "speed": 0.5},
  "manifest_path": "/home/you/robot/ROBOT.md"
}
```

with `Authorization: Bearer <actuate-tier token>`. `speed` is optional and in
(0, 1]; `x_mm`/`y_mm`/`z_mm` are millimetres in the arm's **base frame**
(`physics.solver.base_frame`: z up, x forward) and name the **tool tip**, not
the wrist flange.

Response (`200`):

```json
{
  "ok": true,
  "manifest_kid": "bob-manifest-2026",
  "scope": "MANIPULATE",
  "tool_name": "arm.move_to",
  "actuator_name": "so-arm101",
  "outcome_kind": "executed",
  "telemetry": {
    "reached": true,
    "final_positions": {
      "shoulder_pan": 0.0, "shoulder_lift": -1.363708920430335,
      "elbow_flex": 1.4634176716429017, "wrist_flex": 1.4710875755823298,
      "wrist_roll": -0.21475731030398976
    },
    "eef_mm": {"x": 150.08, "y": 0.0, "z": 49.89},
    "elapsed_s": 0.0338,
    "max_error_rad": 0.00076,
    "error_mm": 0.14,
    "target_mm": {"x": 150.0, "y": 0.0, "z": 50.0},
    "speed": 0.5,
    "waypoints": 2,
    "ik_provider": "inhouse-so-arm101"
  },
  "attestation": "attested",
  "outcome": {"...": "the Ed25519-signed receipt"},
  "envelope_signature": {"kid": "...", "alg": "Ed25519", "sig": "..."}
}
```

`reached` is a joint-space verdict; `error_mm` is the Cartesian one, computed by
forward kinematics on the pose actually reached. They can disagree, and when
they do the second is the one that answers "did it get there".

**Nothing is ever clamped.** A target the arm cannot hold comes back as a signed
`403` with no motion at all:

```json
{
  "detail": {
    "deny": "actuator_policy",
    "reason": "out_of_workspace: x=500mm is outside the declared workspace (-200 to 340mm)",
    "actuator_name": "so-arm101",
    "telemetry": {
      "deny": "out_of_workspace",
      "reason": "x=500mm is outside the declared workspace (-200 to 340mm)"
    },
    "attestation": "attested",
    "envelope_signature": {"kid": "...", "alg": "Ed25519", "sig": "..."}
  }
}
```

`detail.telemetry.deny` is the stable code to branch on. `detail.reason` is the
same thing as a sentence, and may be reworded:

| `deny` | Meaning |
|---|---|
| `bad_args` | missing/non-numeric coordinate, unknown argument, or `speed` outside (0, 1] |
| `out_of_workspace` | outside `physics.workspace.bounds_mm` |
| `unreachable` | the links do not span it with the tool vertical |
| `joint_limits` | solvable, but outside a joint's declared `limits_deg` |
| `frame_disagreement` | the solve and forward kinematics disagree about where the pose puts the tip |
| `unsafe_pose` | inside the declared limits, outside this rig's **measured** `SAFE_RANGE_RAD` |
| `unsafe_start` | the arm is parked outside that envelope — run `arm.home` first |
| `ik_provider_mismatch` | the manifest names a solver this driver does not implement |
| `no_kinematics` | the manifest declares no chain to solve against |

#### The straight-down constraint is a real limit, not a formality

`arm.move_to` solves with the **in-house `inhouse-so-arm101` provider** the
manifest names — the same closed form that ships in the robot-md CLI as
`robot_md.kinematics.Kinematics.ik_reach`, ported here because the authoring CLI
is not installed on a robot. `tests/test_kinematics_ik.py` asserts the two agree
exactly whenever robot-md *is* importable.

That solver pins the tool axis to vertical, and on the SO-ARM101 this driver was
written against, that is geometrically incompatible with the arm's measured
envelope: the wrist tops out around **+0.41 rad** and a vertical tool at
tabletop reach needs roughly **+1.47 rad**. A sweep of 25,480 points across that
robot's declared workspace found **zero** poses that solve inside the measured
ranges. On such a rig `arm.move_to` denies every target with `unsafe_pose` — and
that is the correct answer, not a defect:

- To **touch a point** at whatever tilt the arm can manage, use
  `arm.reach_point`, which measures and steps instead of solving and imposes no
  tool orientation at all.
- To make `arm.move_to` usable, an operator who has re-measured their own arm
  widens the envelope: `SO_ARM101_SAFE_RANGE_RAD='{"wrist_flex": [-0.93, 1.50]}'`.
  That is a deliberate act with a servo-latch cost, which is exactly why the
  driver will not do it on your behalf.

### `arm.state` — read-only

```json
{
  "msg_id": "eval-0002",
  "type": "rcan/v1/invoke",
  "ruri": "rcan://RRN-000000000011/arm",
  "scope": "OBSERVE",
  "tool_name": "arm.state",
  "tool_args": {},
  "manifest_path": "/home/you/robot/ROBOT.md"
}
```

with `Authorization: Bearer <read-tier token or better>`. Response (`200`):

```json
{
  "ok": true,
  "manifest_kid": "bob-manifest-2026",
  "scope": "OBSERVE",
  "tool_name": "arm.state",
  "actuator_name": "so-arm101",
  "outcome_kind": "executed",
  "telemetry": {
    "joint_positions_rad": {
      "shoulder_pan": 0.0, "shoulder_lift": -1.363708920430335,
      "elbow_flex": 1.4634176716429017, "wrist_flex": 1.4710875755823298,
      "wrist_roll": -0.21475731030398976, "gripper": 0.7807962210337913
    },
    "eef_mm": {"x": 150.08, "y": 0.0, "z": 49.89},
    "tool": "gripper"
  },
  "attestation": "attested",
  "envelope_signature": {"kid": "...", "alg": "Ed25519", "sig": "..."}
}
```

`tool` is `"pen"`, `"gripper"`, or `null`, resolved most-specific-first from
`SO_ARM101_TOOL` (an operator who swapped the end effector by hand), then
`physics.solver.tool` in the manifest, then `"gripper"` when the manifest
declares a gripper joint and a tip offset. `null` means nothing described it —
never a guess. `SO_ARM101_TOOL=none` says the flange is bare.

`eef_mm` is `null` only when the manifest declares no kinematic chain; the joint
angles are still reported, because a read that refuses during a bring-up is
useless exactly when it is most needed.

### Enabling them on a deployed robot

Both tools are declared in `IMPLEMENTED_CAPABILITIES`, which is what a console
builds a robot's advertised capability surface from. The gateway will not
dispatch them until the operator also adds them to its policy:

```bash
ROBOT_MD_TOOL_ALLOWLIST="...,arm.move_to,arm.state"
ROBOT_MD_TOOL_MIN_TIER="...,arm.move_to:actuate|commission,arm.state:read|actuate|commission"
```

The gateway reads that file at start, so the change lands on its next restart.
Listing them in the signed `ROBOT.md`'s `capabilities:` as well is a separate,
deliberate operator act — re-signing a robot's root-of-trust document is not a
side effect of a driver update.

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
