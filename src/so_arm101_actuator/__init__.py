"""SO-ARM101 Actuator Protocol driver. RPN-000000000002."""

__version__ = "0.2.2"
__rpn__ = "RPN-000000000002"

from so_arm101_actuator.actuator import SOArm101Actuator

__all__ = ["SOArm101Actuator", "__version__", "__rpn__"]
