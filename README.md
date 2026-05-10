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

## License

Apache-2.0.
