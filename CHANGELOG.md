# Changelog

All notable changes to `so-arm101-actuator`. Format loosely follows
[Keep a Changelog](https://keepachangelog.com/); versions follow semver.

Entries before 0.3.0 are reconstructed from the git history — this file did not
exist while they shipped.

## [0.4.0]

Found by the EV-03 hostile-model test: a fuzzer in the model's seat, sending signed
`arm.move_to` envelopes through the real robot-md-gateway to this driver, with only the
serial port simulated (bob's calibration and gravity sag), 8 October 2026. Results and
the harness are in the EV-03 test kit.

### Changed (behaviour)

- **Every motion is paced.** `speed` is now a fraction of the declared rates
  (`safety.max_joint_velocity_dps`; `safety.max_linear_velocity_ms` for the tool tip
  when declared), walked one step per 20 ms control period. `speed: 1.0` used to be one
  direct Goal_Position per joint, which the servos run at their own top speed: EV-03
  measured 270 °/s against 180 declared, and the tip at up to 1.6 m/s. A manifest that
  declares no joint rate gets 30 °/s (robot-md's first-motion default). `arm.home`,
  `arm.reach`, the `move` and `home` verbs and every `arm.reach_point` step are paced
  the same way. `speed` must be in [0.01, 1]; a move that would take longer than 30 s is
  refused (`too_slow`).
- **The path is checked before anything moves.** The tip's path along the joint-space
  line is checked against `physics.workspace.bounds_mm`, sampled at ≤ 1 mm of tip travel
  by a lever-arm bound with the segments between samples bounded too, and refused with
  `path_leaves_workspace` when it leaves. Both ends were checked before; between two
  allowed targets the tip went 11–13 mm below the declared floor. An arm that starts
  outside may only move in ways that do not make it worse.
  `SO_ARM101_WORKSPACE_MARGIN_MM` keeps the commanded path that far inside every face
  (default 0); the check is on the commanded path, and a loaded servo settles short of
  its goal.
- **Loaded joints are held on target.** A position servo settles short of its goal under
  load (bob: shoulder_lift 27 ticks, elbow_flex 19, wrist_flex 12, lifted into place), so
  a path checked as commanded can still leave the arm below the floor. Once a paced move
  comes to rest, each joint's goal is offset by the error it settled with: bounded
  (0.05 rad), clamped to the safe range, paced, a few rounds, each joint keeping its best
  offset; a joint that stops improving is not pushed harder. The sag this shows (goal
  offset per mm/rad of the tip's drop rate) is fed forward into later moves, so the arm
  does not ride below its path while moving either. In the EV-03 simulation of bob's
  sag the tip's error after a move fell from 11–13 mm to under 1 mm, and the worst
  excursion below the declared floor from 13.6 mm to 4.6 mm (mid-move, onto targets on
  the floor plane itself). reach_point keeps its own correction.
- **`arm.reach_point` checks the declared workspace** (it checked reach only, and walked
  to z = −101.7 mm), runs under the bus lock, and opens the port itself (its first call
  on a fresh gateway was an HTTP 500, AttributeError).
- **Scope is enforced for motion tools.** An envelope that names a motion tool under a
  non-actuation scope (OBSERVE, say) is refused with `scope_mismatch`; it used to
  execute. No scope can refuse `arm.estop`.
- **A stop interrupts the move it is meant to stop.** `arm.estop` latches and signals
  before it waits for the bus; every motion loop checks between bus writes and gives the
  bus up within one control period. The stop used to queue behind the move's bus lock and
  take effect only after the motion had finished. An interrupted move returns
  `outcome_kind: "error"` with `telemetry.stopped: true`.
- **A repeated stop re-sends the first hold.** Each `arm.estop` used to send the
  encoder reading as the new goal; a loaded joint settles below its goal, so repeated
  stops ratcheted the arm down (EV-03, simulated: z 185 → −227 mm over 200 read-tier
  stops). The first stop of a latch takes the reading; repeats re-send it
  (`telemetry.hold_reasserted`); `arm.estop.clear` releases it.
- **The servo bus is held exclusively** (`exclusive=True` plus `TIOCEXCL`): any other
  non-root process that opens the port gets `EBUSY`. A port that cannot be claimed is
  not used. On a bus error the handle is closed before it is dropped, so the re-open is
  not refused by our own claim.
- Subnormal speeds (5e-324, 1e-309) are a `bad_args` deny; they overflowed
  `ceil(1/speed)` and came back as HTTP 500.

### Added

- `move_paced()` and `motion.py` (limits from the manifest, the lever-arm bound, the
  planner and the path check).
- `MoveToResult`: `paced_s`, `joint_speed_limit_dps`, `path_min_clearance_mm`.
  `waypoints` now counts paced steps.
- `tests/test_ev03_findings.py`: one or more regression tests per finding.

### Not changed

- `move()` stays raw (full servo speed, no workspace check). The sweep CLI uses it;
  nothing the gateway serves starts a motion with it. The castor-hal transport's
  `JOINT_POSITIONS` goal still calls it.
- The first stop on a loaded arm still lets each joint settle by its static error once
  (one sag step, a few mm at the tip in simulation). Check it on the arm.
- The sag model is learned online and is a model: on a different arm, payload or pose
  range it can be wrong, which is what the bound and the per-joint best are for. It
  needs checking on the arm.

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
