---
robot-md-version: 1
package: so-arm101-actuator
version: 0.1.0
rpn: RPN-000000000002
hardware-class: so-arm101
capabilities:
  - move
  - home
  - read_state
entry-point: so_arm101_actuator.actuator:SOArm101Actuator
gateway-min: 0.5.0a1
license: Apache-2.0
homepage: https://github.com/RobotRegistryFoundation/so-arm101-actuator
cookbook: https://robotmd.dev/cookbook/beat-8/
---

# SO-ARM101 Actuator

Real Actuator Protocol driver for the SO-ARM101 (Hugging Face / The Robot
Studio's 5-DOF + gripper arm). Wraps the SCS/STS servo wire protocol in a
clean two-layer package.

## Capabilities

- `move(joint_positions: dict[str, float])` — blocking; reaches each joint within tolerance or times out.
- `home()` — moves to configured home pose.
- `read_state()` — snapshots positions + best-effort motor temperatures.

## Default wiring

USB-C from servo bus to host, default port `/dev/ttyACM0` at 1 Mbps. Override
via `SO_ARM101_PORT` and `SO_ARM101_BAUD` environment variables or
constructor arguments.

## See also

- Cookbook beat 8: install + register + run end-to-end.
- Bob case study at robotmd.dev/case-studies/ once it's published.
