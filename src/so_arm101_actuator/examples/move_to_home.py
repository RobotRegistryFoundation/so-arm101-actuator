"""Send the arm to the configured home pose.

Run:
    python -m so_arm101_actuator.examples.move_to_home
"""

from __future__ import annotations

from so_arm101_actuator import SOArm101Actuator


def main() -> None:
    actuator = SOArm101Actuator.from_default_port()
    result = actuator.home()
    print(f"reached={result['reached']} elapsed={result['elapsed_s']:.2f}s")
    print(f"final positions (rad): {result['final_positions']}")


if __name__ == "__main__":
    main()
