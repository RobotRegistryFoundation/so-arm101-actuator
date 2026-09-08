"""Error hierarchy for so_arm101_actuator."""


class UnknownJointError(KeyError):
    """Raised when a joint name is not in config.JOINTS."""


class OutOfRangeError(ValueError):
    """Raised when a target value is outside the configured joint limits."""


class ProtocolError(IOError):
    """Raised on SCS protocol errors (checksum, framing, serial I/O)."""


class ActuatorTimeoutError(TimeoutError):
    """Raised when a blocking actuator call exceeds its timeout."""


class DeniedError(Exception):
    """A refusal the caller must read as a DECISION, not as a fault.

    The gateway turns an ``outcome_kind="denied"`` into a signed 403 the client
    can keep, while anything else becomes a 500 that reads as a broken robot.
    Refusing to drive into a joint limit is not the robot breaking, so it is
    raised as this and converted there.

    ``code`` is stable and machine-readable ("unreachable", "unsafe_pose", ...);
    ``detail`` is the sentence a person reads.
    """

    def __init__(self, code: str, detail: str) -> None:
        self.code = code
        self.detail = detail
        super().__init__(f"{code}: {detail}")
