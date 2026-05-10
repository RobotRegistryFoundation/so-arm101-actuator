"""Wave hello: shoulder_pan oscillates ±0.3 rad three times.

Run:
    python -m so_arm101_actuator.examples.wave
"""

from __future__ import annotations

import time

from so_arm101_actuator import SOArm101Actuator


def main() -> None:
    actuator = SOArm101Actuator.from_default_port()
    actuator.home()
    for _ in range(3):
        actuator.move({"shoulder_pan": 0.3}, timeout_s=2.0)
        actuator.move({"shoulder_pan": -0.3}, timeout_s=2.0)
        time.sleep(0.1)
    actuator.home()


if __name__ == "__main__":
    main()
