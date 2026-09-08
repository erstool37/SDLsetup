"""The circulator package's public surface. Everything callable from outside is here.

A caller reaching past this module into ``codec``, ``link``, ``safety`` or
``circulator`` is a signal, not a shortcut: either this surface is missing
something and should gain it deliberately, or the caller is about to
reimplement a bound that already exists.

    from tools import circulator

    dev = circulator.Circulator.from_config("configs/config.yaml")
    dev.write_setpoint(dev.bound(24.774))     # dry-run plan by default

:class:`FakeSerial` is re-exported because tests are legitimate callers and it
is the ONLY transport they may use -- see :mod:`tools.circulator.link` for why
no test opens a real port.
"""
from __future__ import annotations

from .circulator import (
    CONFIG_SECTION,
    DEFAULT_BAUDRATE,
    DEFAULT_BOOT_SETTLE_S,
    DEFAULT_BYTESIZE,
    DEFAULT_PARITY,
    DEFAULT_STOPBITS,
    DEFAULT_TIMEOUT_S,
    DEFAULT_UNIT_ID,
    INVENTORY_PATH,
    Circulator,
    CirculatorSettings,
)
from .codec import (
    SETPOINT_ADDRESS,
    SETPOINT_COUNT,
    decode_setpoint,
    describe_frame,
    encode_setpoint,
)
from .link import FakeSerial, SerialLink, WriteResult
from .node import CirculatorNode
from .safety import (
    COMMAND_MAX_C,
    COMMAND_MIN_C,
    ActuationNotAllowed,
    BoundedSetpoint,
    CirculatorError,
    CommandLimits,
    SafetyError,
)

__all__ = [
    "COMMAND_MAX_C",
    "COMMAND_MIN_C",
    "CONFIG_SECTION",
    "DEFAULT_BAUDRATE",
    "DEFAULT_BOOT_SETTLE_S",
    "DEFAULT_BYTESIZE",
    "DEFAULT_PARITY",
    "DEFAULT_STOPBITS",
    "DEFAULT_TIMEOUT_S",
    "DEFAULT_UNIT_ID",
    "INVENTORY_PATH",
    "SETPOINT_ADDRESS",
    "SETPOINT_COUNT",
    "ActuationNotAllowed",
    "BoundedSetpoint",
    "Circulator",
    "CirculatorError",
    "CirculatorNode",
    "CirculatorSettings",
    "CommandLimits",
    "FakeSerial",
    "SafetyError",
    "SerialLink",
    "WriteResult",
    "decode_setpoint",
    "describe_frame",
    "encode_setpoint",
]
