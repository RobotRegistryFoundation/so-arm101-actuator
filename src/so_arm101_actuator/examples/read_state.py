"""Print current joint positions + temperatures.

Run:
    python -m so_arm101_actuator.examples.read_state
"""

from __future__ import annotations

from so_arm101_actuator import SOArm101Actuator


def main() -> None:
    actuator = SOArm101Actuator.from_default_port()
    state = actuator.read_state()
    for joint, rad in state["positions"].items():
        temp = state["motor_temps_c"].get(joint, "—")
        print(f"{joint:18s} {rad:+.3f} rad   {temp:>4} C")
    print(f"\ntimestamp: {state['timestamp_s']:.2f}")


if __name__ == "__main__":
    main()
