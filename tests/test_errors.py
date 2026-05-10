"""Tests for the error class hierarchy."""

import pytest

from so_arm101_actuator.errors import (
    UnknownJointError,
    OutOfRangeError,
    ProtocolError,
    ActuatorTimeoutError,
)


def test_unknown_joint_error_is_keyerror():
    with pytest.raises(KeyError):
        raise UnknownJointError("foo")


def test_out_of_range_error_is_valueerror():
    with pytest.raises(ValueError):
        raise OutOfRangeError("too high")


def test_protocol_error_is_ioerror():
    with pytest.raises(IOError):
        raise ProtocolError("checksum mismatch")


def test_actuator_timeout_is_timeouterror():
    with pytest.raises(TimeoutError):
        raise ActuatorTimeoutError("did not reach target")
