"""The environment package's public surface. Everything callable from outside is here.

A caller reaching past this module into ``registers``, ``reading``, ``safety``,
``plc`` or ``relay`` is a signal, not a shortcut: either this surface is missing
something and should gain it deliberately, or the caller is about to reimplement
a bound that already exists.

    from tools import environment

    print(environment.provenance())                  # what is measured, what is inferred
    reading = environment.read("configs/config.yaml")
    print(reading.temp_filtered_c, reading.temp_sp_c)

    environment.set_temperature(24.0, "configs/config.yaml", dry_run=True)

Read and actuate are separated on purpose
=========================================

The read side (:func:`read`, :func:`diagnostics`, :func:`pid_enabled`,
:func:`provenance`, :func:`latest_published`) touches nothing that changes state.
The actuating side (:func:`set_temperature`, :func:`set_humidity`,
:func:`enable_pid`, :func:`disable_pid`, :class:`Relay`) is gated by
``environment.allow_actuation`` in ``configs/config.yaml`` and defaults to a
printed dry-run plan.

Every setpoint on the actuating side goes through
:meth:`SetpointLimits.validate` before :meth:`PlcClient.write_float32`, so an
unbounded float cannot reach the wire -- that is enforced by the
:class:`BoundedSetpoint` type, not by these functions being careful.

**The rate limiter is NOT applied by these helpers.**
:class:`~.safety.RateLimiter` needs the previous write's value and time, which a
stateless one-shot helper does not have; inventing "no previous write" on every
call would make the interval rule vacuous while looking enforced. So the step and
interval caps are the phase script's to apply, with the history it holds. A run
that writes setpoints repeatedly must construct a :class:`RateLimiter` and call
``check()`` itself.

Sessions are short by design
============================

Each helper opens one Modbus session, does one operation, and closes it. There is
no cached module-level session, because **the CLICK accepts at most three
concurrent Modbus TCP clients** and refuses the fourth -- a lingering session
held by a convenience function is a socket a run cannot have. A script doing more
than one operation should hold its own :class:`PlcClient` (or
:class:`Environment`, the same class under a name that reads better in
orchestration code) and pass it as ``client=``.

Two ``SafetyError`` classes, and how this module resolves the clash
==================================================================

:mod:`tools.environment.safety` and :mod:`tools.circulator.safety` each define a
``SafetyError``, and they are **unrelated classes** with different bases
(``PlcError`` and ``CirculatorError``). Catching one does not catch the other,
and that is deliberate: a refused PLC setpoint and a refused bath command are
different faults on different instruments.

This module is the environment package's surface, so the bare name
:class:`SafetyError` is the **PLC** one. The circulator's is re-exported beside
it as :class:`CirculatorSafetyError`, never as a bare ``SafetyError``, so no
importer can pick up the wrong one by accident. :meth:`Relay.step` can raise
either -- the PLC's from a read-back mismatch, the circulator's from an
out-of-bound DF9 -- so a phase script driving the relay catches both:

    from tools.environment import api as env
    try:
        record = relay.step()
    except (env.SafetyError, env.CirculatorSafetyError) as exc:
        ...
"""
from __future__ import annotations

from typing import Any

#: The circulator's guard type is imported here under a name that cannot be
#: confused with the PLC's. See the module docstring -- they are unrelated
#: classes with different bases, and catching one does not catch the other.
from ..circulator.api import SafetyError as CirculatorSafetyError
from .node import EnvironmentNode
from .plc import (
    CONFIRMED,
    DEFAULT_DEVICE_ID,
    DEFAULT_HOST,
    DEFAULT_PORT,
    FAILED,
    MAX_CONCURRENT_CLIENTS,
    NATIVE_DIAGNOSTICS,
    PLANNED,
    FakePlc,
    PlcClient,
    PlcSettings,
    WriteResult,
    latest_published,
    publish_last_reading,
)
from .reading import ChannelQuality, EnvironmentReading, decode_block
from .registers import (
    BLOCK_COUNT,
    BY_FIELD,
    C1_COIL,
    DF_BASE,
    REGISTERS,
    UNKNOWN_DF,
    WRITABLE,
    CoilSpec,
    Provenance,
    RegisterSpec,
    df_address,
    f32_to_regs,
    provenance_table,
    regs_to_f32,
)
from .relay import Relay, RelayPolicy, RelayRecord
from .safety import (
    ActuationNotAllowed,
    BoundedSetpoint,
    CommsLost,
    PlcError,
    RateLimited,
    RateLimiter,
    SafetyError,
    SetpointLimits,
)

#: ``PlcClient`` under the name orchestration code reads better with. The same
#: class, not a subclass: one client, two names, no second implementation.
Environment = PlcClient

#: Config section this package resolves from.
CONFIG_SECTION = "environment"


# ---------------------------------------------------------------------------
# read-only helpers
# ---------------------------------------------------------------------------
def _session(config: Any, client: PlcClient | None,
             **overrides: Any) -> tuple[PlcClient, bool]:
    """``(client, owned)``. ``owned`` means this call must close it again."""
    if client is not None:
        return client, False
    return PlcClient(PlcSettings.from_config(config, **overrides)), True


def read(config: Any = None, *, client: PlcClient | None = None,
         publish: bool = False, **overrides: Any) -> EnvironmentReading:
    """One block read of DF1..DF23 plus coil C1, decoded.

    A transport fault comes back as ``read_ok=False`` with **no channels** -- it
    does not raise and does not retry, because both would be decisions. Read
    ``client.last_error`` for the fault text; the reading type carries none.

    ``publish=True`` also writes the reading where the dashboard reads it, so a
    run's own session is the only Modbus client on the wire.

    **``read_ok`` is not a sufficient gate.** A SHORT block still reports
    ``read_ok=True``, so check ``partial`` and the presence of the specific
    channel you are about to use. :class:`Relay` does exactly that.
    """
    session, owned = _session(config, client, **overrides)
    try:
        session.connect()
        record = session.read_block()
        if publish:
            session.publish_last_reading(record)
        return record
    finally:
        if owned:
            session.close()


def pid_enabled(config: Any = None, *, client: PlcClient | None = None,
                **overrides: Any) -> bool | None:
    """Coil C1, the PID auto/manual flag. ``None`` when it could not be read.

    ``None`` is not ``False``: "we do not know whether the loop is closed" and
    "the loop is open" are different facts.
    """
    session, owned = _session(config, client, **overrides)
    try:
        session.connect()
        return session.read_pid_enabled()
    finally:
        if owned:
            session.close()


def diagnostics(config: Any = None, *, client: PlcClient | None = None,
                **overrides: Any) -> dict[str, Any]:
    """Connection facts, plus the native SC/SD bits we cannot read yet.

    Every ``SC90``-``SC95`` and ``SD41`` entry reports ``unavailable`` with the
    reason: the bits exist and their nicknames are known, **their Modbus
    addresses are not**. They are not guessed -- an invented address reads some
    other register and reports it as a link flag.
    """
    session, owned = _session(config, client, **overrides)
    try:
        return session.diagnostics()
    finally:
        if owned:
            session.close()


def provenance() -> str:
    """A printable table of every register, its provenance, and whether it was
    verified against live hardware.

    Print it in any run that records environment data: the address formula is
    THIRD_PARTY, not vendor-certified, and an operator reading a number off this
    layer must never mistake an inference for a measurement.
    """
    return provenance_table()


# ---------------------------------------------------------------------------
# actuating helpers
# ---------------------------------------------------------------------------
def _write_setpoint(field: str, value: float, config: Any,
                    client: PlcClient | None, dry_run: bool,
                    **overrides: Any) -> WriteResult:
    session, owned = _session(config, client, **overrides)
    try:
        # validate() is the ONLY producer of a BoundedSetpoint, and
        # write_float32 takes nothing else. So the bound is enforced by type
        # here, not by this function remembering to check.
        sp = session.settings.limits.validate(field, value)
        if not dry_run:
            session.connect()
        return session.write_float32(sp, dry_run=dry_run)
    finally:
        if owned:
            session.close()


def set_temperature(value_c: float, config: Any = None, *,
                    client: PlcClient | None = None, dry_run: bool = False,
                    **overrides: Any) -> WriteResult:
    """Write ``temp_sp_c`` (DF5) as one FC16 of two registers, then read it back.

    Refused outside the **strict** open interval the config declares -- 0.0 and
    30.0 are both rejected, inherited from the MATLAB GUI this layer replaces.
    Refused, never clamped: a setpoint quietly moved to the nearest legal value
    makes the run record disagree with the chamber.

    ``dry_run=True`` returns the frame it would have sent as
    ``outcome="planned"`` -- and therefore ``ok=False``, because nothing was
    written -- constructing no transport at all.
    """
    return _write_setpoint("temp_sp_c", value_c, config, client, dry_run,
                           **overrides)


def set_humidity(value_pct: float, config: Any = None, *,
                 client: PlcClient | None = None, dry_run: bool = False,
                 **overrides: Any) -> WriteResult:
    """Write ``rh_sp_pct`` (DF6). Same bound discipline as :func:`set_temperature`.

    The PLC actuates humidity itself, as a PWM duty cycle. Nothing about it
    transits this host, so there is no humidity relay.
    """
    return _write_setpoint("rh_sp_pct", value_pct, config, client, dry_run,
                           **overrides)


def enable_pid(config: Any = None, *, client: PlcClient | None = None,
               dry_run: bool = False, **overrides: Any) -> WriteResult:
    """Set coil C1, handing temperature and humidity control to the ladder loop.

    **Requires ``allow_actuation``.** Deliberately asymmetric with
    :func:`disable_pid`, and deliberately **not** a dashboard command: enabling
    a PID loop is an actuation that must be made on purpose.
    """
    session, owned = _session(config, client, **overrides)
    try:
        if not dry_run:
            session.connect()
        return session.set_pid(True, dry_run=dry_run)
    finally:
        if owned:
            session.close()


def disable_pid(config: Any = None, *, client: PlcClient | None = None,
                dry_run: bool = False, **overrides: Any) -> WriteResult:
    """Clear coil C1. **Permitted with ``allow_actuation`` false.**

    Turning the loop off is the safe direction and it is what the abort path
    does, so gating it behind the same flag as a setpoint write would mean a
    supervisor that may not write is also a supervisor that cannot stop.
    """
    session, owned = _session(config, client, **overrides)
    try:
        if not dry_run:
            session.connect()
        return session.set_pid(False, dry_run=dry_run)
    finally:
        if owned:
            session.close()


__all__ = [
    "BLOCK_COUNT",
    "BY_FIELD",
    "C1_COIL",
    "CONFIG_SECTION",
    "CONFIRMED",
    "DEFAULT_DEVICE_ID",
    "DEFAULT_HOST",
    "DEFAULT_PORT",
    "DF_BASE",
    "FAILED",
    "MAX_CONCURRENT_CLIENTS",
    "NATIVE_DIAGNOSTICS",
    "PLANNED",
    "REGISTERS",
    "UNKNOWN_DF",
    "WRITABLE",
    "ActuationNotAllowed",
    "BoundedSetpoint",
    "ChannelQuality",
    "CirculatorSafetyError",
    "CoilSpec",
    "CommsLost",
    "Environment",
    "EnvironmentNode",
    "EnvironmentReading",
    "FakePlc",
    "PlcClient",
    "PlcError",
    "PlcSettings",
    "Provenance",
    "RateLimited",
    "RateLimiter",
    "RegisterSpec",
    "Relay",
    "RelayPolicy",
    "RelayRecord",
    "SafetyError",
    "SetpointLimits",
    "WriteResult",
    "decode_block",
    "df_address",
    "diagnostics",
    "disable_pid",
    "enable_pid",
    "f32_to_regs",
    "latest_published",
    "pid_enabled",
    "provenance",
    "provenance_table",
    "publish_last_reading",
    "read",
    "regs_to_f32",
    "set_humidity",
    "set_temperature",
]
