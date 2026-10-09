# Changelog

All notable changes to `so-arm101-actuator`. Format loosely follows
[Keep a Changelog](https://keepachangelog.com/); versions follow semver.

Entries before 0.3.0 are reconstructed from the git history — this file did not
exist while they shipped.

## [Unreleased]

What EV-03 found when it ran this driver behind robot-md-gateway against a
simulated servo bus with bob's calibration and gravity sag (October 2026), and
the fixes. Configured to do real work, the tip went up to 13 mm below the
declared floor: targets on the floor face were accepted, the sagging arm rested
below them, and full-slew jumps overshot on the way. The driver's own receipts
put the tip outside the workspace after 270 to 450 moves per ten-minute run, and
nothing bounded speed.

### Changed (behaviour)

- **Every gateway motion is a checked, paced line** (`motion.py`). `arm.move_to`,
  `arm.home`, `arm.reach`, `arm.reach_point`'s steps and a bare `move` go from the
  pose the arm is held at to the goal along one joint-space line that is
  checked against `physics.workspace.bounds_mm` at the elbow, wrist and tip,
  as commanded and as the arm will really be (commanded + its current offset),
  with a margin (`SO_ARM101_WORKSPACE_MARGIN_MM`, default 10 mm) kept per face
  (a move leaving the band keeps half of it, or comes at most 1 mm closer and
  never past the face, as planned, when it starts nearer than that; an arm past
  a face may go 2 mm further on that face only, on its way back in; the taught
  pose can be reached though it is inside the margin), and streamed as setpoints at least 20 ms apart
  (never catching up after a stall) under the declared speed limits. New deny
  codes: `path_leaves_workspace`, `too_slow`, `busy`, `manifest_unreadable`.
  **No target closer than the margin to a face is accepted.**
- **`speed` means the fraction of the declared limits.** 1.0 (the default) used
  to be one direct Goal_Position command, which ran the servos at their own top
  speed; it is now the manifest's limits: `safety.max_tool_velocity_ms` for the
  tool point (0.25 m/s when undeclared; `max_linear_velocity_ms` can only lower
  it) and `safety.max_joint_velocity_dps` per joint (90 deg/s when undeclared).
  Moves take longer. The `waypoints` telemetry key is now the number
  of setpoints streamed; a `motion` block reports the pacing, the limits, how
  close the path came to a face and whether a stop ended it.
- **`arm.estop` interrupts a move and no longer ratchets.** It latches before it
  waits for the bus lock, a paced move checks the latch before every setpoint,
  every other request gives way while the latching stop waits for the bus, and
  otherwise waits at most 1 s before it is refused as `busy`. The hold re-sends the goals
  it chose for this latch while the servos still hold them, instead of
  re-reading the sagged encoders on every repeat (which walked the arm down
  9 cm in 8 s in simulation); a joint whose servo no longer holds them (a reset)
  is chosen again from what it holds. Each goal is the one the servo's own
  Goal_Position register reports, when the joint reads within 100 ticks of it,
  not its sag. An arm sagging more than 100 ticks drops by its sag on each
  fresh choice (see below).
- **Moves plan from the servo's Goal_Position register**, not from a copy of
  the last goal sent: after a brown-out reset a goal, the stale copy made the
  first setpoint an unpaced jump back (1.5 m/s at the tip in simulation).
- **`arm.reach_point`** refuses targets less than the margin inside the
  workspace (`out_of_workspace`; it checked reach only and steered to
  z = -101.7 mm), takes only checked, paced steps, runs under the bus lock and
  opens the port itself (its first call on a fresh gateway was an HTTP 500).
- **The servo bus is opened exclusively** (pyserial `exclusive=True` plus
  `TIOCEXCL`), so another process's `open(2)` fails with `EBUSY` while the
  driver holds it. New `claim(config)` and `close()`; robot-md-gateway calls
  `claim()` at startup. A dropped handle is now released, not leaked.
- A subnormal `speed` (5e-324) is refused as `too_slow` instead of raising
  `OverflowError` (an HTTP 500).
- An unknown joint or a joint value outside its limits is a signed refusal
  (`unknown_joint`, `joint_limits`), not a 500. A bare `move` takes only
  `joint_positions`, `speed` and `timeout_s`; the bare `move_to` alias is parsed
  like `arm.move_to`.
- A manifest whose geometry cannot be read refuses motion
  (`manifest_unreadable`) instead of leaving every check resolving to nothing:
  a path that cannot be read, no YAML frontmatter, frontmatter that does not
  parse (one stray tab) or is not a mapping. Asked on every motion, so a
  manifest that changes under a running gateway is caught too. Reads still
  answer.
- `arm.reach` (a pose derived from the taught one) no longer gets the taught
  pose's allowance for sitting inside the margin. A refused `arm.reach_point`
  step no longer says "Nothing was moved." when earlier steps did move.
- A joint parked outside its configured range can be moved back in; every
  setpoint is converted before the first is sent.
- The serial handle is dropped under the bus lock, and the stop's latch is built
  under its own lock.
- Not covered: the castor-hal `Transport` path still drives the raw `move()`.
- Found by a third review, and fixed: a Goal_Position read that fails is
  retried once and then fails the move (it used to fall back to the stale
  copy); the stop then holds a joint within the window of its chosen goal and
  any other where it reads. A workspace that parses but is malformed (`z: [0]`,
  a misspelled key, low above high) refuses motion, checked before every
  planned line. Only the stop that latches gets bus priority (four looping stop
  clients used to lock out every clear). A manifest re-signed in place is
  re-read, joint zeros included. `SO_ARM101_ADOPT_REFERENCE_TICKS` moves the
  100-tick window. Documented, not fixed: a joint stalled within the window
  keeps pushing through a stop, and an arm sagging past it drops by its sag on
  every fresh choice (each latch, each clear, each move start).
- The cost, in the same simulation: no excursion in 9 hostile ten-minute runs,
  but with the declared joint limits a benign tabletop task ran 64 of its 93
  moves (its stations 10 mm above the table were refused: the predicted series
  carries the offset from the station above, which over-predicts the sag there
  by about 7 mm), and a task working 4 to 18 mm above the table ran none from
  bob's ready pose (it sits in the margin band of x = 340, and each line out of
  it swung toward that face).

### Added

- `kinematics.Chain` (fast forward kinematics for the elbow, wrist and tip),
  `kinematics.Box` / `workspace_box`, `kinematics.declared_speed_limits`.
- `protocol.SCSProtocol.read_goal_position`.

## [0.3.0]

### Added

- **`arm.move_to`** — Cartesian control. Puts the tool tip on a point in the
  arm's base frame with the tool pointing straight down, solved with the
  `inhouse-so-arm101` provider the manifest names. Refuses anything it cannot
  hold with a structured, signed deny and **no motion**; never clamps to the
  nearest legal pose, because that pose puts the tip somewhere the caller did
  not ask for and the receipt would still say it succeeded. Deny codes:
  `bad_args`, `out_of_workspace`, `unreachable`, `joint_limits`,
  `frame_disagreement`, `unsafe_pose`, `unsafe_start`, `ik_provider_mismatch`,
  `no_kinematics`. Telemetry: `reached`, `final_positions`, `eef_mm`,
  `elapsed_s`, plus `max_error_rad`, `error_mm`, `target_mm`, `speed`,
  `waypoints`, `ik_provider`.
- **`arm.state`** — read-only pose snapshot: `joint_positions_rad`, `eef_mm`,
  `tool`. Same tier class as `status.report` (read, actuate, commission); issues
  position reads and nothing else.
- `kinematics.solve_tool_down` — the analytic tool-down solve, a faithful port
  of `robot_md.kinematics.Kinematics.ik_reach`. Ported rather than imported
  because robot-md is an authoring CLI and is not installed on a robot;
  `tests/test_kinematics_ik.py` asserts the two agree exactly across a grid of
  targets whenever robot-md *is* importable, so the copy cannot drift.
- `kinematics.declared_tool` — what is on the end of the arm, resolved from
  `SO_ARM101_TOOL`, then the manifest, then a declared gripper. `None` when
  nothing described it; naming a tool nobody declared would be a claim.
- `kinematics.joint_limits_rad`, `kinematics.tip_offset_from_manifest`,
  `kinematics.frontmatter` — the manifest reads the above are built from.
- `errors.DeniedError` — a refusal carrying a stable `code` and a human
  `detail`. Converted to `outcome_kind="denied"`, which the gateway returns as a
  signed 403 rather than a 500 that reads like a broken robot.
- `SO_ARM101_TOOL` environment override.
- `tests/fixtures/so_arm101.robot.md` — a real SO-ARM101's geometry, checked in
  so the suite stops depending on one operator's home directory.

### Changed

- **`speed` on a position-only bus.** `protocol.py` writes Goal_Position and
  nothing else — there is no velocity register, and adding one is a hardware
  decision, not a software one. `speed` is therefore spelled as joint-space
  interpolation: `ceil(1/speed)` waypoints along the straight line to the
  target, capped at 10. `speed: 1.0` (the default) is one direct command,
  exactly what `arm.home` and `arm.reach` have always done.
- Manifest frontmatter is now parsed once and cached by mtime. The closed-loop
  reach evaluates forward kinematics nine times per step, each of which re-read
  and re-parsed the whole YAML document; a workspace sweep was spending minutes
  in the parser. Keyed on mtime, so a re-signed manifest lands without a
  restart.
- `tip_position_mm` and `max_reach_mm` read the tip offset from the manifest
  instead of the module constant, so a rig that swapped its gripper for a pen is
  described by editing its manifest.
- `capabilities` gained `move_to` and `state`. The three original verbs keep
  their positions.
- The test suite restores `config.JOINTS` and `config.SAFE_RANGE_RAD` after
  every test. `apply_manifest_calibration` mutates them in place by design, so
  the first test to read a manifest was silently re-zeroing every joint for
  every test after it, and results depended on collection order.

### Known limitation (not a defect)

The in-house solver pins the tool axis to vertical. On the SO-ARM101 this driver
was written against, that is geometrically incompatible with the arm's measured
envelope — the wrist tops out near +0.41 rad and a vertical tool at tabletop
reach needs roughly +1.47. A sweep of 25,480 points across that robot's declared
workspace found **zero** poses that solve inside the measured ranges, so
`arm.move_to` denies every target on that rig as shipped.

`arm.reach_point` is the tool that gets to a point there: it measures and steps
rather than solving, and imposes no tool orientation. An operator who has
re-measured their own arm can widen `SO_ARM101_SAFE_RANGE_RAD` and `arm.move_to`
starts working; the driver will not widen it on their behalf, because the cost
of being wrong is a latched servo.

## [0.2.3]

- Closed-loop reaching (`arm.reach_point` / `reach_point`): get to a coordinate
  by measuring and stepping, because the analytic IK cannot solve inside this
  arm's limits.
- Forward kinematics (`kinematics.tip_position_mm`): where the tip is, so a tap
  on a phone screen can become a coordinate in the arm's frame.
- `reached` derived from the snapshot the receipt actually returns, so a receipt
  can no longer disagree with itself.
- Gripper driven from the manifest on the path the gateway actually uses.
- This robot's geometry read from its manifest instead of assumed.
- Servo-bus access serialized; implemented capabilities declared.
- Per-tool tier enforcement inside `execute()`; serial reopened after an I/O
  error.
- ROBOT.md capability names mapped to driver methods (`arm.home`, `arm.reach`,
  `status.report`).
- SO-ARM101 as the first castor-hal Transport adapter.

## [0.2.1]

- `SAFE_RANGE_RAD` and `MOVE_TOLERANCE_RAD` calibrated against a real rig, with
  `SO_ARM101_SAFE_RANGE_RAD` / `SO_ARM101_MOVE_TOLERANCE_RAD` overrides for
  operators whose arms measure differently.

## [0.2.0]

- Operator overrides: `SO_ARM101_HOME_POSE_RAD`, `SO_ARM101_SAFE_RANGE_RAD`.
- Sweep CLI for hands-on range-of-motion bring-up.

## [0.1.0]

- First release. `move` / `home` / `read_state` over a vendored SCS/Feetech wire
  protocol, dispatched by `robot-md-gateway` through the
  `robot_md_gateway.actuators` entry-point group. RPN-000000000002.
