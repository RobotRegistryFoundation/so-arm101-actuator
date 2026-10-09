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
(0, 1]: the fraction of the declared speed limits the move may use (see
[Motion is checked and paced](#motion-is-checked-and-paced-v040)).
`x_mm`/`y_mm`/`z_mm` are millimetres in the arm's **base frame**
(`physics.solver.base_frame`: z up, x forward) and name the **tool tip**, not
the wrist flange. The envelope's `scope` must be an actuation scope
(`MANIPULATE`, `ACTUATE`, `EXECUTE`, ...): robot-md-gateway refuses a motion
tool under `OBSERVE`.

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
    "elapsed_s": 2.649,
    "max_error_rad": 0.00076,
    "error_mm": 0.14,
    "target_mm": {"x": 150.0, "y": 0.0, "z": 50.0},
    "speed": 0.5,
    "waypoints": 133,
    "ik_provider": "inhouse-so-arm101",
    "motion": {
      "setpoints": 133,
      "pace_period_s": 0.0199,
      "planned_duration_s": 2.649,
      "tool_speed_limit_mps": 0.125,
      "joint_speed_limit_dps": null,
      "tool_path_mm": 264.9,
      "path_min_clearance_mm": 7.05,
      "path_closest": {"point": "tip", "series": "commanded", "fraction_of_path": 0.047,
                       "face": "x<=340", "at_mm": [333.0, 38.1, 193.2]},
      "workspace_margin_mm": 10.0,
      "start_offset_rad": {},
      "stopped_by_estop": false,
      "notes": []
    }
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
| `bad_args` | missing/non-numeric coordinate, unknown argument, or `speed` not a finite number in (0, 1] |
| `out_of_workspace` | outside `physics.workspace.bounds_mm` |
| `path_leaves_workspace` | the line from where the arm is held to the target comes within the margin of a face, or past it (see below) |
| `too_slow` | `speed` so small the move would take longer than 60 s |
| `busy` | another command is running; commands are refused, not queued |
| `manifest_unreadable` | the manifest's geometry could not be read, so nothing can be checked |
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

## Motion is checked and paced (v0.4.0)

Every motion robot-md-gateway can ask this driver for (`arm.move_to`,
`arm.home`, `arm.reach`, `arm.reach_point`'s steps and a bare `move`) is one
straight line in joint space, from the pose the arm is being held at to the
goal, and that line is checked and paced before anything is sent.

**The whole path is checked, not its ends.** The line is sampled every
0.005 rad (about 2 mm at full reach), and the elbow, the wrist and the tip are
checked against `physics.workspace.bounds_mm` at every sample, face by face,
twice: as commanded, and as the arm will really be (the commanded pose plus the
offset the arm shows now: gravity sag, friction). Each must stay
`SO_ARM101_WORKSPACE_MARGIN_MM` (default **10 mm**) from every face. That margin
covers what neither series can see: the offset changing with the pose (bob's
shoulder settles 2.5 to 27 encoder ticks past its goal depending on which way it
arrived, several millimetres at the tip), servo lag and tick rounding.

Starting closer than the margin, a point may come no closer to that face, unless
the move takes it a full margin away, in which case it keeps half the margin
on the way (or, starting closer than that, may come 1 mm closer than it started,
never past the face as planned: the first millimetre of almost any line out of
a pose points the tip a little the wrong way). An arm already past a face
(pushed out, or parked badly) may go up to 2 mm further past that face, and no
other, on a move that ends a full margin inside. The pose the operator taught
(`arm.home`) may sit inside the margin; the path to it may come as close as the
pose itself or half the margin, whichever is closer, less 1 mm, and the pose
itself must be inside the box. A refusal is `path_leaves_workspace` with no
motion.

Inside the margin band the plan is all there is, and the plan can be beaten:
when a move starts, each joint's friction flips to the other side of its goal,
which the prediction (measured at rest) cannot see. In the review's probes
(simulation, bob's sag and friction fit, a stiff servo), 4 of 41 moves out of
bob's ready pose, which sits 1.6 mm from x = 340 once it sags, crossed that face
by up to 0.8 mm. Do not teach a pose inside the margin: from a full margin
inside, the same error stays inside the box.

The margin has a cost: no target closer than 10 mm to a face is accepted, as
commanded or as predicted. The prediction carries today's offset unchanged to
the goal, and the real offset depends on the pose and on which way the arm
arrives, so it can be wrong in either direction; the margin is there for the
direction that matters. In EV-03's simulation (bob's sag model) it erred the
safe way at the low stations of a tabletop task: carried down from the station
30 mm above, the offset put the tip 3 mm above the table at a station commanded
10 mm up, where the simulated arm settles 10.3 mm up, so those stations were
refused at a 10 mm margin and at a 5 mm one. And bob's taught ready pose sits
7.6 mm from the x = 340 face (1.6 mm as predicted), where many lines out of it
start by swinging the tip 1 to 3 mm toward that face, so from ready some
targets are refused until a move that swings the other way gets the arm out.
An operator whose work needs the tool nearer a face can declare the face where
the tool may really go, or lower the margin after measuring their own arm's
offsets.

**Speed is bounded.** The servos take Goal_Position and nothing else, so a
speed limit is a stream of setpoints, at most 20 ms apart, sized so that
neither the tool point nor any joint is asked to move faster than the declared
limits: `safety.max_tool_velocity_ms` for the tool point (**0.25 m/s** when the
manifest declares none, ISO 10218-1's reduced speed; the schema's
`max_linear_velocity_ms`, documented for wheeled bases, can only lower it) and
`safety.max_joint_velocity_dps` for each joint (**90 deg/s** when undeclared, so
wrist_roll and the gripper, which do not move the tool point, are paced too).
The stream is planned at 80 % of the limits, and each setpoint goes out at least
one period after the last: after a stall the move takes longer, it never
catches up. `speed` scales both limits.
`speed` 1.0 (the default) used to mean one direct command, which ran the servos
at their own top speed (270 deg/s and 1.6 m/s at the tip in EV-03's simulation);
it now means "as fast as the manifest allows".

**Moves start from the goal being held.** A joint within 100 encoder ticks of
the goal it was last sent (or, on a fresh gateway, of its own Goal_Position
register) is holding that goal, and the move starts there; planning from the
sagged reading instead lowers the arm by its sag on every command. 100 ticks is
almost four times bob's measured sag, room for a payload. A joint that starts
outside its configured range may pass between that start and the range on its
way back in; every setpoint is converted before the first is sent.

**The stop interrupts, and holds without walking the arm down.** `arm.estop`
latches before it waits for the bus, and a paced move checks the latch before
every setpoint, so a move in progress ends within one period. While a stop waits
for the bus, every other request is refused as `busy` at once (one that gets the
bus first hands it straight back), and otherwise a request waits at most 1 s for
the bus and is then refused: commands are not queued behind a running move.
(robot-md-gateway runs stop tools on worker threads of their own, so a burst of
other requests cannot hold the stop back before it gets here either.)

The hold picks each joint's goal once per latch: the goal its servo is holding
(read back from the servo's Goal_Position register), when the joint reads within
100 ticks of it, so the stop moves nothing; otherwise where the joint is (it was
moving, or it was pushed, or it is stalled against something). A repeated stop
re-sends those goals for as long as the servo still holds them, however far the
arm has sagged from them, and chooses again only for a joint whose servo no
longer does (a brown-out reset its goal to wherever it fell), from what that
servo holds now. The stop used to re-read the encoders every time, and under
gravity every repeat lowered the arm by its sag: 9 cm in 8 s of stops 0.2 s
apart in simulation, and again, in review, for an arm sagging more than the 100
ticks. Moves are planned from the same register for the same reason: planned
from a stale copy, the first setpoint after a brown-out re-commanded the old
goal in one unpaced jump.

One cost remains for an arm whose sag passes 100 ticks (about 9 degrees; five
times bob's): the first stop cannot tell that sag from a stall, holds the joint
where it reads, and the arm drops once by its sag (59 mm at the tip in
simulation) before it holds. The register cannot tell the two apart; a load or
speed reading could.

**`arm.reach_point` respects the workspace.** Targets less than the margin
inside the box are refused (`out_of_workspace`), and every step it takes is a
checked, paced move; a step the check refuses ends the call as a signed refusal
that says how far it got. It used to check reach only, and steered the tip to
z = -101.7 mm. It now runs under the bus lock and opens the port like every
other motion; its first call on a fresh gateway used to be an HTTP 500.

**The bus is held exclusively.** The port is opened with an `flock`
(pyserial `exclusive=True`) and `TIOCEXCL`, so any other process's `open(2)`
fails with `EBUSY` while the gateway holds it. LeRobot takes no lock, and
drove bob's arm with the gateway running. robot-md-gateway calls `claim()` at
startup so the port is held from the start. Neither lock binds root, and
neither evicts a process that opened the port first.

Refusals of joint moves are now decisions, not faults: an unknown joint or a
value outside a joint's limits comes back as a signed `403`
(`unknown_joint`, `joint_limits`) rather than a `500`. A manifest whose geometry
cannot be read refuses every motion (`manifest_unreadable`) instead of letting
it through unchecked.

**Not covered:** the castor-hal `Transport` path (`transport.py`,
`make_hal_actuator`) still drives the raw `move()`: unpaced, unchecked and
outside the bus lock. It is not the entry point the gateway uses; switching to
it would bypass everything above.

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
