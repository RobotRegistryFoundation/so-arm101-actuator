"""Error hierarchy for so_arm101_actuator."""


class UnknownJointError(KeyError):
    """Raised when a joint name is not in config.JOINTS."""


class OutOfRangeError(ValueError):
    """Raised when a target value is outside the configured joint limits."""


class ProtocolError(IOError):
    """Raised on SCS protocol errors (checksum, framing, serial I/O)."""


class ActuatorTimeoutError(TimeoutError):
    """Raised when a blocking actuator call exceeds its timeout."""
