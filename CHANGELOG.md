# Changelog

All notable changes to `so-arm101-actuator`. Format loosely follows
[Keep a Changelog](https://keepachangelog.com/); versions follow semver.

Entries before 0.3.0 are reconstructed from the git history — this file did not
exist while they shipped.

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
