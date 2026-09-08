"""Custom MCU circulator -- temperature SETPOINT ONLY, write-only, no read-back.

Common entry points
===================
Most callers need only:

* :class:`Circulator` (``from_config``, ``bound``, ``write_setpoint``) -- the bath.
* :class:`CirculatorSettings` -- its resolved settings and command bound.
* :class:`FakeSerial` -- the only transport a test may use.

Do not drive the bath by hand for a run. ``scripts/environment/start_kinetics.py``
is THE full-run entry point and ``scripts/environment/check_environment.py`` the
status read; both reach the bath through the relay's pre-declared fail-safe.


.. danger::

   **OPENING THE SERIAL PORT HARDWARE-RESETS THE MICROCONTROLLER.** The board
   is reached through an FTDI FT232R bridge whose DTR line is capacitively
   coupled to /RESET, and the OS asserts DTR on open, before any byte is sent.
   There is no read-only identify query. Opening the port is treated here as an
   actuating operation: gated by ``allow_actuation``, counted, and logged as a
   reset. Use ``port_present()`` -- ``stat`` only -- to ask whether the device
   node exists.

This device has **no vendor, model, manual, firmware source, schematic or
register map**. ``RW-3`` in the code this replaces was a MATLAB cell-divider
comment, not a model number. Exactly **one transaction** is established:
4 holding registers at 0-based address **980** carrying an IEEE-754 **float64**
Celsius setpoint. Everything else -- mixing, flow, RPM, status, alarms -- is
``MEANING UNKNOWN``, and there is **no read-back of any kind**.

    from tools import circulator

    dev = circulator.Circulator.from_config("configs/config.yaml")
    print(dev.port_present())                  # opens nothing
    dev.write_setpoint(dev.bound(24.774))      # dry-run plan by default

Layers, lowest to highest:

``codec``       the one verified encoding: float64, byteorder LITTLE, wordorder
                LITTLE. **NOT the PLC's encoding** (that one is float32,
                byteorder BIG, wordorder LITTLE)
``safety``      CommandLimits + BoundedSetpoint -- the STRICT (0, 30) C bound,
                enforced by type; both endpoints refused. INFERRED from the prior
                system's logs, whose maximum coincides with a saturation
                artifact; this board has no known clamp of its own
``link``        the only transport; opening it is actuation, and every write is
                checked against the FC16 echo
``circulator``  Circulator + CirculatorSettings -- what orchestration calls
``node``        Lab node wrapper for the dashboard (thin; never raises into the bus)
``api``         the public surface; import from here

This package **reports**. It never decides: no retry, no fallback setpoint, no
clamp. A refused bound raises; a failed write returns ``ok=False`` with the
observed echo, and the phase script chooses what to do.
"""
from . import circulator, codec, link, safety
from .api import (
    COMMAND_MAX_C,
    COMMAND_MIN_C,
    CONFIG_SECTION,
    DEFAULT_TIMEOUT_S,
    INVENTORY_PATH,
    SETPOINT_ADDRESS,
    SETPOINT_COUNT,
    ActuationNotAllowed,
    BoundedSetpoint,
    Circulator,
    CirculatorError,
    CirculatorNode,
    CirculatorSettings,
    CommandLimits,
    FakeSerial,
    SafetyError,
    WriteResult,
    decode_setpoint,
    describe_frame,
    encode_setpoint,
)

__all__ = [
    "COMMAND_MAX_C",
    "COMMAND_MIN_C",
    "CONFIG_SECTION",
    "DEFAULT_TIMEOUT_S",
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
    "WriteResult",
    "circulator",
    "codec",
    "decode_setpoint",
    "describe_frame",
    "encode_setpoint",
    "link",
    "safety",
]
