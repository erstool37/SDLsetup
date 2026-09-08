"""Modbus TCP I/O for the CLICK PLC. One block read, one guarded write.

This is the only module in the environment package that touches a socket. It
reports what the PLC said and refuses what the guards refuse; it decides
nothing -- no retry, no substitution, no clamping, no choice of what to do
about a bad reading.

The four defects it exists to prevent
=====================================

Each one was live in the code this replaces, and each has a named test in
``dev/tests/test_environment_safety.py``.

1. **A float32 setpoint written as ONE register.** The prior code called
   ``write_register(28681, high_word_only)`` -- the high half of a float32,
   leaving the low word at whatever the PLC held before. Demonstrated live:
   writing ``25.3`` read back as ``25.3587``, and ``93.7`` as ``93.9349``. It
   appeared to work only because every setpoint ever used in practice (25.0,
   90, 92, 95, 87) happens to have a zero low word. :meth:`PlcClient.write_float32`
   emits exactly one FC16 of exactly two registers, and **this module exposes
   no API capable of a single-register float write at all** -- there is no
   ``write_register`` method to reach for.

2. **No write verification.** Every write here is read back and compared
   word-for-word, and a mismatch raises :class:`~.safety.SafetyError`.

3. **A silent read failure that still actuated.** The prior read was guarded by
   ``if not result.isError():`` with no else -- on an error response the value
   stayed ``NaN`` and downstream code forwarded it anyway. Here a failed read
   returns a reading with ``read_ok=False`` and **no channels at all**: nothing
   is substituted, and the caller cannot mistake absence for a measurement.

4. **No plausibility check.** Range facts come from the quality flags in
   :mod:`tools.environment.reading`; this module adds no verdict of its own.

A write reports one of three outcomes and never a bare boolean
=============================================================

:class:`WriteResult` carries ``outcome`` in ``{"planned", "confirmed",
"failed"}`` and derives ``ok`` from it, so ``ok`` is True only for a write that
was sent *and* verified. A dry run is ``planned`` with ``ok=False``: a frame was
composed and nothing left the process, and a caller checking only ``ok`` must
not be able to read a plan as a completed write.

The three-socket ceiling
========================

The CLICK accepts **at most 3 concurrent Modbus TCP clients** and refuses a 4th
with a Modbus error -- vendor-confirmed in both the CLICK and CLICK PLUS
manuals. That is why :func:`publish_last_reading` exists: the dashboard renders
the state a run publishes to a small JSON file instead of holding a session of
its own.

Native diagnostics that we cannot read yet
==========================================

The CLICK has built-in comms diagnostics -- ``SC90`` Port_1_Ready_Flag,
``SC91`` Port_1_Error_Flag, ``SC92`` Port_1_Clients_Limit, ``SC93``
Port_1_IP_Resolved, ``SC94`` Port_1_Link_Flag, ``SC95`` Port_1_100MBIT_Flag,
and ``SD41`` _Port1_No_Comm_Time (a 0..32767 s counter that resets on every
received message and increments once per second, i.e. a built-in comms-loss
detector). **Their Modbus addresses are not known**, and this module ships them
as ``None`` rather than guessing: a guessed address reads some other register
and reports it as a link flag. :meth:`PlcClient.diagnostics` says so out loud.

The transport is injected
=========================

``PlcClient(settings, transport=fake)`` takes anything with the four keyword
signatures pymodbus 3.15 uses. ``pymodbus`` itself is imported **inside** the
method that builds the real client, so this package imports with no vendor SDK
present -- asserted by a test.
"""
from __future__ import annotations

import dataclasses
import datetime
import json
import os
import time
from pathlib import Path
from typing import Any

from .. import config as _config
from . import reading as _reading
from . import registers
from .safety import (
    ActuationNotAllowed,
    BoundedSetpoint,
    CommsLost,
    RateLimiter,
    SafetyError,
    SetpointLimits,
)

#: Static link-local address from the PLC project file. DHCP is off on the
#: CLICK, so it does not move.
DEFAULT_HOST = "169.254.33.33"
DEFAULT_PORT = 502
#: **INFERENCE, not documented.** The prior code never set a unit id, so this is
#: the pymodbus library default rather than a value read off the PLC.
#: TODO(operator): confirm the CLICK's Modbus device id in its project file.
DEFAULT_DEVICE_ID = 1
DEFAULT_TIMEOUT_S = 2.0
#: Zero on purpose. A retry is a decision, and this layer does not make any --
#: a failed read is reported to the caller, which decides whether to ask again.
DEFAULT_RETRIES = 0

#: Vendor-confirmed in the CLICK and CLICK PLUS manuals.
MAX_CONCURRENT_CLIENTS = 3

#: Where a run publishes its last reading for the dashboard. Module-level so a
#: test can redirect it, exactly as ``tools.occupancy.OCCUPANCY_DIR`` is.
PUBLISH_DIR = Path.home() / ".sdl_lab" / "environment"
PUBLISH_NAME = "last_reading.json"

#: The CLICK's own comms diagnostics. Every address is ``None``: the bits exist
#: and their nicknames are known, their Modbus addresses are not.
NATIVE_DIAGNOSTICS: tuple[tuple[str, str, int | None], ...] = (
    ("SC90", "Port_1_Ready_Flag", None),
    ("SC91", "Port_1_Error_Flag", None),
    ("SC92", "Port_1_Clients_Limit", None),
    ("SC93", "Port_1_IP_Resolved", None),
    ("SC94", "Port_1_Link_Flag", None),
    ("SC95", "Port_1_100MBIT_Flag", None),
    ("SD41", "_Port1_No_Comm_Time", None),
)

_DIAG_TODO = ("TODO(operator): confirm via CLICK Address Picker with "
              '"Display MODBUS Address" checked')


def _publish_path() -> Path:
    """Resolved at call time so reassigning :data:`PUBLISH_DIR` takes effect."""
    return PUBLISH_DIR / PUBLISH_NAME


def publish_last_reading(record: _reading.EnvironmentReading) -> Path:
    """Write ``record`` to :data:`PUBLISH_DIR` atomically; return the path.

    The dashboard reads this instead of opening its own Modbus session, because
    the CLICK only has :data:`MAX_CONCURRENT_CLIENTS` sockets to spend. Written
    to a temp file in the same directory and moved with :func:`os.replace`, so
    a reader never sees a half-written record.
    """
    path = _publish_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = record.as_dict()
    payload["published_unix_s"] = time.time()
    payload["published_monotonic_s"] = time.monotonic()
    temp = path.with_name(path.name + ".%d.tmp" % os.getpid())
    temp.write_text(json.dumps(payload, indent=1), encoding="utf-8")
    os.replace(temp, path)
    return path


def latest_published() -> dict[str, Any] | None:
    """The last published reading plus ``age_s``, or ``None`` if there is none.

    ``age_s`` is computed on read rather than stored, so a stale file cannot
    claim to be fresh. It is a fact, not a verdict: whether that age is too old
    is the caller's call (``environment.plc_stale_s`` in ``config.yaml``).
    """
    path = _publish_path()
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return None
    except (OSError, json.JSONDecodeError) as exc:
        raise CommsLost("%s: unreadable published reading (%s)" % (path, exc)) from exc
    if not isinstance(payload, dict):
        raise CommsLost("%s: published reading is not a JSON object" % path)
    published = payload.get("published_unix_s")
    payload["age_s"] = (time.time() - float(published)
                        if isinstance(published, (int, float)) else None)
    return payload


# ---------------------------------------------------------------------------
# settings
# ---------------------------------------------------------------------------

#: Keys resolved from the ``environment`` section, with their cast. Named after
#: the config keys so there is no translation layer to drift.
_NUMERIC_KEYS = (
    ("port", DEFAULT_PORT, int),
    ("device_id", DEFAULT_DEVICE_ID, int),
    ("timeout_s", DEFAULT_TIMEOUT_S, float),
    ("retries", DEFAULT_RETRIES, int),
    ("temp_sp_min_c", None, float),
    ("temp_sp_max_c", None, float),
    ("rh_sp_min_pct", None, float),
    ("rh_sp_max_pct", None, float),
    ("max_setpoint_step_c", None, float),
    ("max_setpoint_step_pct", None, float),
    ("min_setpoint_interval_s", None, float),
    ("relay_period_s", 1.0, float),
    ("plc_stale_s", 5.0, float),
)
_BOOL_KEYS = (
    ("allow_actuation", False),
    ("dashboard_poll_plc", False),
)


@dataclasses.dataclass(frozen=True)
class PlcSettings:
    """Resolved runtime configuration for the PLC link, with provenance.

    ``sources`` records which layer each value came from, and
    :meth:`describe` prints it. A safety limit whose provenance is unknown is
    not a safety limit -- print this in any run that writes a setpoint.
    """

    host: str = DEFAULT_HOST
    port: int = DEFAULT_PORT
    device_id: int = DEFAULT_DEVICE_ID
    timeout_s: float = DEFAULT_TIMEOUT_S
    retries: int = DEFAULT_RETRIES
    allow_actuation: bool = False
    dashboard_poll_plc: bool = False
    # Setpoint bounds. Named after the config keys; interpreted as an OPEN
    # interval -- see the module docstring of tools.environment.safety.
    temp_sp_min_c: float = SetpointLimits().temp_min_c
    temp_sp_max_c: float = SetpointLimits().temp_max_c
    rh_sp_min_pct: float = SetpointLimits().rh_min_pct
    rh_sp_max_pct: float = SetpointLimits().rh_max_pct
    max_setpoint_step_c: float = RateLimiter().max_step_c
    max_setpoint_step_pct: float = RateLimiter().max_step_pct
    min_setpoint_interval_s: float = RateLimiter().min_interval_s
    # Supervisor cadence and staleness. Resolved here so the whole
    # `environment` section has exactly one resolution path.
    relay_period_s: float = 1.0
    plc_stale_s: float = 5.0
    sources: dict = dataclasses.field(default_factory=dict)

    def __post_init__(self) -> None:
        if not isinstance(self.host, str) or not self.host.strip():
            raise _config.ConfigError(
                "environment.host must be a non-empty string, got %r" % (self.host,))
        for name in ("port", "device_id"):
            value = getattr(self, name)
            if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                raise _config.ConfigError(
                    "environment.%s must be a non-negative integer, got %r"
                    % (name, value))
        if self.timeout_s <= 0.0:
            raise _config.ConfigError(
                "environment.timeout_s must be positive, got %r" % (self.timeout_s,))
        if self.retries < 0:
            raise _config.ConfigError(
                "environment.retries must not be negative, got %r" % (self.retries,))
        if self.plc_stale_s <= 0.0 or self.relay_period_s <= 0.0:
            raise _config.ConfigError(
                "environment.relay_period_s and plc_stale_s must be positive, got "
                "%r and %r" % (self.relay_period_s, self.plc_stale_s))
        if self.plc_stale_s < self.relay_period_s:
            raise _config.ConfigError(
                "environment.plc_stale_s (%r) is below relay_period_s (%r); the "
                "watchdog would fire before the loop has had a chance to read"
                % (self.plc_stale_s, self.relay_period_s))
        # Bounds and step caps are validated by the types that own them, so
        # there is exactly one place the ceilings are written down.
        try:
            _ = self.limits
            _ = self.rate_limiter
        except SafetyError as exc:
            raise _config.ConfigError("environment: %s" % exc) from exc

    @property
    def limits(self) -> SetpointLimits:
        """The setpoint window. Raises if config tried to widen a ceiling."""
        return SetpointLimits(
            temp_min_c=self.temp_sp_min_c,
            temp_max_c=self.temp_sp_max_c,
            rh_min_pct=self.rh_sp_min_pct,
            rh_max_pct=self.rh_sp_max_pct,
        )

    @property
    def rate_limiter(self) -> RateLimiter:
        return RateLimiter(
            max_step_c=self.max_setpoint_step_c,
            max_step_pct=self.max_setpoint_step_pct,
            min_interval_s=self.min_setpoint_interval_s,
        )

    @classmethod
    def from_config(cls, config: Any = None, **overrides: Any) -> PlcSettings:
        """Resolve through the precedence contract in :mod:`tools.config`.

        ``overrides`` are CLI-flag level (highest precedence); passing ``None``
        for one means "not specified", so a flag that was not given never
        shadows the file. A key absent everywhere yields the code default; a
        key **present but unusable raises**, and the two are never collapsed.

        There is no ``environment.json`` hardware inventory: the CLICK's
        register map is code (:mod:`tools.environment.registers`), not data.
        """
        lab = _config.LabConfig.load(config)
        section = lab.section("environment")
        _reject_empty_keys(section)

        resolved: dict[str, Any] = {}
        sources: dict[str, str] = {}

        def take(key: str, default: Any, cast: type | None) -> None:
            raw = section.get(key)
            # This file's parser reads YAML 1.1 booleans, so a bare `N`, `no`
            # or `on` becomes a bool. int(False) is 0 and str(False) is
            # "False" -- both plausible-looking and both wrong, so a bool
            # standing where a number or a host belongs is refused here rather
            # than cast.
            if cast is not bool and isinstance(raw, bool):
                raise _config.ConfigError(
                    "environment.%s: %r is a boolean, not a %s. This file's "
                    "parser reads YAML 1.1 booleans, so bare y/n/on/off/N "
                    "become True/False -- quote the value if you meant the "
                    "string." % (key, raw, "value" if cast is None else cast.__name__))
            item = _config.resolve(
                key, argument=overrides.get(key), config=section,
                default=default, inventory_name="environment.json", cast=cast,
            )
            if item.value is not None or default is not None:
                resolved[key] = item.value
            sources[key] = item.source

        for key, default, cast in _NUMERIC_KEYS:
            take(key, default, cast)
        for key, default in _BOOL_KEYS:
            take(key, default, bool)
        take("host", DEFAULT_HOST, None)
        if not isinstance(resolved.get("host"), str):
            raise _config.ConfigError(
                "environment.host must be a string, got %r from %s"
                % (resolved.get("host"), sources.get("host")))

        return cls(sources=sources, **resolved)

    def describe(self) -> str:
        """Every value with the config layer it came from.

        Printed by any run that writes a setpoint: a safety limit whose
        provenance is unknown is not a safety limit.
        """
        lines = ["environment PLC settings (host=%s:%d device_id=%d, "
                 "allow_actuation=%r)" % (self.host, self.port, self.device_id,
                                          self.allow_actuation)]
        for field in dataclasses.fields(self):
            if field.name == "sources":
                continue
            value = getattr(self, field.name)
            lines.append("  %-24s %-14r [%s]"
                         % (field.name, value, self.sources.get(field.name, "-")))
        lines.append("  " + self.limits.describe())
        lines.append("  " + self.rate_limiter.describe())
        return "\n".join(lines)


def _reject_empty_keys(section: dict[str, Any]) -> None:
    """An empty config key parses to ``{}``, not ``None`` and not ``""``.

    The repo's stdlib parser reads an empty value as the opening of a nested
    mapping, and ``{}`` is falsy -- so a naive ``if value:`` reads an
    intentionally-empty key as absent, which is exactly the absent/unusable
    collapse this layer must not perform. Every ``environment`` key is typed,
    so an empty one is present-and-unusable and raises here with a message
    naming the trap, rather than surfacing as a confusing cast failure.
    """
    typed = {key for key, _, _ in _NUMERIC_KEYS}
    typed.update(key for key, _ in _BOOL_KEYS)
    typed.add("host")
    for key in sorted(typed & set(section)):
        if isinstance(section[key], dict):
            raise _config.ConfigError(
                "environment.%s is present but has no value. An empty key "
                "parses to an empty mapping in this file's parser, not to "
                "null -- either give it a value or delete the line so the "
                "code default applies." % key)


# ---------------------------------------------------------------------------
# results
# ---------------------------------------------------------------------------

#: A frame was composed and nothing was sent. Not a success.
PLANNED = "planned"
#: Sent, and the read-back matched word-for-word. The only success.
CONFIRMED = "confirmed"
#: Attempted and not verified. Says nothing about what the PLC now holds.
FAILED = "failed"

_OUTCOMES = (PLANNED, CONFIRMED, FAILED)


@dataclasses.dataclass(frozen=True)
class WriteResult:
    """What one write actually did. Data, not a verdict.

    ``outcome`` is the whole truth and :attr:`ok` is derived from it:

    ``planned``     a dry run. A frame was composed; nothing left the process.
    ``confirmed``   sent, and the read-back verified word-for-word.
    ``failed``      attempted and not verified.

    **:attr:`ok` is False for a perfectly successful dry run, and that is not a
    bug.** ``ok`` means "the PLC is holding this value", which a dry run never
    establishes -- so a phase script that checks only ``ok`` reads a plan as
    what it is instead of as a completed write. That misuse was the reason for
    making the state explicit rather than documenting it.

    :attr:`dry_run` is a derived property for the same reason: two independent
    sources of truth for one fact drift, and then one of them is wrong.
    """

    outcome: str
    address: int
    values: tuple[int, ...]
    readback: tuple[int, ...] | None = None
    error: str | None = None
    field: str | None = None
    value: float | bool | None = None

    def __post_init__(self) -> None:
        if self.outcome not in _OUTCOMES:
            raise ValueError("outcome must be one of %r, got %r"
                             % (list(_OUTCOMES), self.outcome))

    @property
    def ok(self) -> bool:
        """True only for a write that was sent AND verified."""
        return self.outcome == CONFIRMED

    @property
    def dry_run(self) -> bool:
        return self.outcome == PLANNED

    def as_dict(self) -> dict[str, Any]:
        """A JSON-serialisable record of this write, matching the circulator's.

        Built explicitly and **not** with ``dataclasses.asdict``, which would
        drop :attr:`ok` and :attr:`dry_run` -- they are properties. A record
        that lost them would show a plan and a confirmed write as the same
        thing, which is the exact confusion ``outcome`` exists to remove.

        The shape deliberately mirrors
        :meth:`tools.circulator.api.WriteResult.as_dict` where the two devices
        have the same fact to report -- ``outcome``, ``ok``, ``dry_run``,
        ``address``, ``values``, ``error`` -- so a relay record carrying one of
        each can be read without a per-device key map. It diverges only where
        the devices differ: this one has a real ``readback`` (the CLICK is read
        straight back and compared), and it names the ``field`` and ``value``
        it wrote, neither of which the circulator can report.
        """
        return {
            "outcome": self.outcome,
            "ok": self.ok,
            "dry_run": self.dry_run,
            "address": self.address,
            "values": list(self.values),
            "readback": None if self.readback is None else list(self.readback),
            "error": self.error,
            "field": self.field,
            "value": self.value,
        }

    def describe(self) -> str:
        head = {PLANNED: "DRY-RUN", CONFIRMED: "CONFIRMED",
                FAILED: "FAILED"}[self.outcome]
        words = " ".join("0x%04X" % word for word in self.values)
        line = "%s %s=%r -> addr %d [%s]" % (head, self.field, self.value,
                                             self.address, words)
        if self.error:
            line += "  error: %s" % self.error
        return line


def _unverified(result: WriteResult, message: str) -> SafetyError:
    """A :class:`SafetyError` carrying the failed :class:`WriteResult`.

    **Why the PLC raises where the circulator returns.** ``tools.circulator``
    reports an echo mismatch as ``outcome="failed"`` without raising, because
    that device offers no read-back at all and the caller is the only thing
    that can decide what an unverified write means. Here a mismatch means a
    setpoint was written into a *live control loop* and we cannot say what the
    loop is now acting on -- there is no safe way to report that and continue,
    so it raises. Same field, different escalation, for a stated reason.
    """
    error = SafetyError(message)
    error.result = result
    return error


# ---------------------------------------------------------------------------
# the client
# ---------------------------------------------------------------------------

class PlcClient:
    """One Modbus TCP session with the CLICK. Reports; decides nothing.

    ``transport`` is injected. ``None`` means "build the real
    :class:`pymodbus.client.ModbusTcpClient` when first needed", and the import
    happens inside that method so this package loads with no vendor SDK and no
    hardware present.
    """

    def __init__(self, settings: PlcSettings, transport: Any = None) -> None:
        self.settings = settings
        self.transport = transport
        self._owns_transport = transport is None
        #: The last transport fault, verbatim. Read it when a reading came back
        #: with ``read_ok=False`` -- the reading type carries no error field.
        self.last_error: str | None = None

    # -- session --------------------------------------------------------
    def _ensure_transport(self) -> Any:
        if self.transport is None:
            # Imported HERE, never at module level: the package must import on
            # a machine with no pymodbus and no PLC.
            from pymodbus.client import ModbusTcpClient

            self.transport = ModbusTcpClient(
                self.settings.host,
                port=self.settings.port,
                timeout=self.settings.timeout_s,
                retries=self.settings.retries,
            )
        return self.transport

    def connect(self) -> bool:
        """Open the socket. Returns whether it opened; never raises for a
        refused connection -- an unreachable PLC is a fact to report."""
        try:
            opened = bool(self._ensure_transport().connect())
        except Exception as exc:                                    # noqa: BLE001
            self.last_error = "connect: %s: %s" % (type(exc).__name__, exc)
            return False
        if not opened:
            self.last_error = "connect: refused by %s:%d (the CLICK accepts at " \
                "most %d concurrent Modbus TCP clients)" % (
                    self.settings.host, self.settings.port, MAX_CONCURRENT_CLIENTS)
        return opened

    def close(self) -> None:
        if self.transport is None:
            return
        try:
            self.transport.close()
        except Exception as exc:                                    # noqa: BLE001
            self.last_error = "close: %s: %s" % (type(exc).__name__, exc)
        if self._owns_transport:
            self.transport = None

    def __enter__(self) -> PlcClient:
        self.connect()
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()

    # -- reads ----------------------------------------------------------
    @staticmethod
    def _now() -> tuple[str, float]:
        return (datetime.datetime.now(datetime.timezone.utc).isoformat(),
                time.monotonic())

    def read_block(self) -> _reading.EnvironmentReading:
        """One block read of DF1..DF23, decoded.

        A transport fault or an error response returns a reading with
        ``read_ok=False`` and **no channels**; it does not raise, and it does
        not retry -- both would be decisions. Nothing is substituted for a
        value that did not arrive, which is the whole difference from the code
        this replaces.
        """
        t_utc, monotonic_s = self._now()
        words = self._read_registers(registers.DF_BASE, registers.BLOCK_COUNT)
        if words is None:
            # decode_block(()) yields read_ok=False with empty channels. Reused
            # rather than hand-building a reading, so there is one decoder.
            return _reading.decode_block((), t_utc=t_utc, monotonic_s=monotonic_s,
                                         pid_enabled=None)
        return _reading.decode_block(words, t_utc=t_utc, monotonic_s=monotonic_s,
                                     pid_enabled=self.read_pid_enabled())

    def _read_registers(self, address: int, count: int) -> list[int] | None:
        """Raw holding-register read. ``None`` on any fault, never a retry."""
        try:
            result = self._ensure_transport().read_holding_registers(
                address, count=count, device_id=self.settings.device_id)
        except Exception as exc:                                    # noqa: BLE001
            self.last_error = "read_holding_registers(%d, count=%d): %s: %s" % (
                address, count, type(exc).__name__, exc)
            return None
        if result is None or self._is_error(result):
            self.last_error = "read_holding_registers(%d, count=%d): %r" % (
                address, count, result)
            return None
        words = getattr(result, "registers", None)
        if words is None:
            self.last_error = "read_holding_registers(%d, count=%d): response " \
                "carried no registers: %r" % (address, count, result)
            return None
        return [int(word) for word in words]

    def read_pid_enabled(self) -> bool | None:
        """Coil C1, the PID auto/manual flag. ``None`` when it could not be read.

        ``None`` is not ``False``: "we do not know whether the loop is closed"
        and "the loop is open" are different facts, and conflating them is how
        a supervisor decides to act on a state it never observed.
        """
        try:
            result = self._ensure_transport().read_coils(
                registers.C1_COIL, count=1, device_id=self.settings.device_id)
        except Exception as exc:                                    # noqa: BLE001
            self.last_error = "read_coils(%d): %s: %s" % (
                registers.C1_COIL, type(exc).__name__, exc)
            return None
        if result is None or self._is_error(result):
            self.last_error = "read_coils(%d): %r" % (registers.C1_COIL, result)
            return None
        bits = getattr(result, "bits", None)
        if not bits:
            self.last_error = "read_coils(%d): response carried no bits: %r" % (
                registers.C1_COIL, result)
            return None
        return bool(bits[0])

    @staticmethod
    def _is_error(result: Any) -> bool:
        checker = getattr(result, "isError", None)
        return bool(checker()) if callable(checker) else False

    # -- writes ---------------------------------------------------------
    def write_float32(self, sp: BoundedSetpoint, *,
                      dry_run: bool = False) -> WriteResult:
        """Write one bounded setpoint as a single FC16 of exactly two registers.

        ``sp`` must be a :class:`~.safety.BoundedSetpoint`. A plain ``float`` is
        refused **by type**, before any value check: the type is the evidence
        that a bound ran, and the code this replaces had none.

        ``dry_run=True`` returns the frame it would have sent, as
        ``outcome="planned"`` with ``ok=False``, and constructs no transport at
        all -- so a plan can be printed on a machine with no PLC. Nothing
        succeeded, so nothing reports ok. Otherwise ``allow_actuation`` must be set, or
        :class:`~.safety.ActuationNotAllowed` is raised. (Those are separate
        knobs on purpose: ``dry_run`` is "show me", the gate is "you may".)

        The two written words are read straight back and compared word-for-word;
        a mismatch, or a read-back that could not be obtained, raises
        :class:`~.safety.SafetyError`. An unverified write is not a write.
        """
        if not isinstance(sp, BoundedSetpoint):
            raise SafetyError(
                "write_float32() takes a BoundedSetpoint, not %s. Produce one "
                "with SetpointLimits.validate(field, value) -- passing a bare "
                "number would put an unbounded setpoint on the wire, which is "
                "the defect this type exists to prevent." % type(sp).__name__)

        # F2b: the token proves *a* bound ran, not that it ran against THIS
        # client's limits or targets a canonical register. Re-validate at the
        # sink: the value against the receiving client's current SetpointLimits
        # (raises if outside), and the token's spec against the canonical
        # WRITABLE spec for its field (a token whose spec is not the canonical
        # one is refused). A token validated under a wider range or redirected to
        # a non-canonical register is not trusted blindly across the boundary.
        self.settings.limits.validate(sp.field, sp.value)
        canonical = registers.WRITABLE.get(sp.field)
        if canonical is None or sp.spec is not canonical:
            raise SafetyError(
                "write_float32(): the setpoint token for %r does not carry the "
                "canonical WRITABLE register spec for that field (got spec %r). A "
                "token whose spec was redirected is refused at the sink." %
                (sp.field, sp.spec))

        low_word, high_word = registers.f32_to_regs(sp.value)
        words = (low_word, high_word)
        address = sp.spec.address

        if dry_run:
            return WriteResult(outcome=PLANNED, address=address, values=words,
                               field=sp.field, value=sp.value)
        if not self.settings.allow_actuation:
            raise ActuationNotAllowed(
                "refusing to write %s=%g: environment.allow_actuation is "
                "false. Pass dry_run=True to see the frame."
                % (sp.field, sp.value))

        try:
            result = self._ensure_transport().write_registers(
                address, list(words), device_id=self.settings.device_id)
        except Exception as exc:                                    # noqa: BLE001
            error = "write_registers(%d, %r): %s: %s" % (
                address, list(words), type(exc).__name__, exc)
            self.last_error = error
            return WriteResult(outcome=FAILED, address=address, values=words,
                               error=error, field=sp.field, value=sp.value)
        if result is None or self._is_error(result):
            error = "write_registers(%d, %r): %r" % (address, list(words), result)
            self.last_error = error
            return WriteResult(outcome=FAILED, address=address, values=words,
                               error=error, field=sp.field, value=sp.value)

        readback = self._read_registers(address, 2)
        if readback is None:
            raise _unverified(
                WriteResult(outcome=FAILED, address=address, values=words,
                            error="read-back unobtainable: %s" % self.last_error,
                            field=sp.field, value=sp.value),
                "wrote %s=%g to %d but could not read it back (%s). An "
                "unverified write is not a write: the PLC may hold this value, "
                "the previous one, or half of each."
                % (sp.field, sp.value, address, self.last_error))
        if tuple(readback) != words:
            raise _unverified(
                WriteResult(outcome=FAILED, address=address, values=words,
                            readback=tuple(readback), error="read-back mismatch",
                            field=sp.field, value=sp.value),
                "read-back mismatch at %d for %s=%g: wrote [0x%04X 0x%04X], "
                "read [%s] (decodes to %r). This is the half-word failure mode: "
                "the PLC is not holding what was commanded."
                % (address, sp.field, sp.value, words[0], words[1],
                   " ".join("0x%04X" % w for w in readback),
                   registers.regs_to_f32(readback[0], readback[1])
                   if len(readback) >= 2 else None))
        return WriteResult(outcome=CONFIRMED, address=address, values=words,
                           readback=tuple(readback), field=sp.field,
                           value=sp.value)

    def set_pid(self, enabled: bool, *, dry_run: bool = False) -> WriteResult:
        """Write coil C1, the PID auto/manual flag.

        **Disabling is permitted with ``allow_actuation`` false; enabling is
        not.** The asymmetry is deliberate and it is not a convenience: turning
        the loop off is the safe direction and it is what the abort path does,
        so gating it behind the same flag as a setpoint write would mean a
        supervisor that may not write is also a supervisor that cannot stop.
        Enabling hands control of the chamber to the ladder loop and therefore
        needs the gate.

        The coil is read straight back. A definite mismatch raises -- being
        wrong about whether the loop is closed is the state this method exists
        to establish. A read-back that could not be obtained is reported as
        ``ok=False`` with the error, not raised, so an abort path is not itself
        aborted by a dead socket.
        """
        want = bool(enabled)
        if dry_run:
            return WriteResult(outcome=PLANNED, address=registers.C1_COIL,
                               values=(1 if want else 0,),
                               field=registers.COIL_SPEC.field, value=want)
        if want and not self.settings.allow_actuation:
            raise ActuationNotAllowed(
                "refusing to ENABLE the PID loop: environment.allow_actuation "
                "is false. Disabling is always allowed; enabling is not.")

        try:
            result = self._ensure_transport().write_coil(
                registers.C1_COIL, want, device_id=self.settings.device_id)
        except Exception as exc:                                    # noqa: BLE001
            error = "write_coil(%d, %r): %s: %s" % (
                registers.C1_COIL, want, type(exc).__name__, exc)
            self.last_error = error
            return WriteResult(outcome=FAILED, address=registers.C1_COIL,
                               values=(1 if want else 0,), error=error,
                               field=registers.COIL_SPEC.field, value=want)
        if result is None or self._is_error(result):
            error = "write_coil(%d, %r): %r" % (registers.C1_COIL, want, result)
            self.last_error = error
            return WriteResult(outcome=FAILED, address=registers.C1_COIL,
                               values=(1 if want else 0,), error=error,
                               field=registers.COIL_SPEC.field, value=want)

        readback = self.read_pid_enabled()
        if readback is None:
            return WriteResult(
                outcome=FAILED, address=registers.C1_COIL,
                values=(1 if want else 0,),
                error="commanded PID %s but could not read C1 back (%s)" % (
                    "on" if want else "off", self.last_error),
                field=registers.COIL_SPEC.field, value=want)
        if readback is not want:
            raise _unverified(
                WriteResult(outcome=FAILED, address=registers.C1_COIL,
                            values=(1 if want else 0,),
                            readback=(1 if readback else 0,),
                            error="read-back mismatch",
                            field=registers.COIL_SPEC.field, value=want),
                "read-back mismatch on C1: commanded PID %s, coil reads %s"
                % ("on" if want else "off", "on" if readback else "off"))
        return WriteResult(outcome=CONFIRMED, address=registers.C1_COIL,
                           values=(1 if want else 0,),
                           readback=(1 if readback else 0,),
                           field=registers.COIL_SPEC.field, value=want)

    # -- diagnostics and publishing -------------------------------------
    def diagnostics(self) -> dict[str, Any]:
        """Connection facts, plus the native diagnostics we cannot read yet.

        Every SC/SD bit is reported ``unavailable`` with the reason. They are
        not guessed: an invented address reads some other register and reports
        it as a link flag, which is worse than reporting nothing.
        """
        native = {
            bit: {
                "nickname": nickname,
                "modbus_address": address,
                "status": "unavailable: address unknown",
                "todo": _DIAG_TODO,
            }
            for bit, nickname, address in NATIVE_DIAGNOSTICS
        }
        transport = self.transport
        connected = getattr(transport, "connected", None)
        return {
            "host": self.settings.host,
            "port": self.settings.port,
            "device_id": self.settings.device_id,
            "device_id_provenance": "INFERENCE -- the prior code never set a "
                                    "unit id, so this is the pymodbus default",
            "timeout_s": self.settings.timeout_s,
            "retries": self.settings.retries,
            "transport": type(transport).__name__ if transport is not None else None,
            "transport_connected": connected,
            "max_concurrent_clients": MAX_CONCURRENT_CLIENTS,
            "max_concurrent_clients_provenance": "VENDOR -- CLICK and CLICK "
                                                 "PLUS manuals",
            "authentication": "none -- the PLC project has [UserAccounts] "
                              "Disable=1",
            "last_error": self.last_error,
            "native_diagnostics": native,
        }

    def publish_last_reading(self, record: _reading.EnvironmentReading) -> Path:
        """Publish ``record`` for the dashboard. See :func:`publish_last_reading`."""
        return publish_last_reading(record)

    @staticmethod
    def latest_published() -> dict[str, Any] | None:
        """Read the published reading. See :func:`latest_published`."""
        return latest_published()

    def __repr__(self) -> str:
        return "PlcClient(%s:%d device_id=%d transport=%s)" % (
            self.settings.host, self.settings.port, self.settings.device_id,
            type(self.transport).__name__ if self.transport is not None else "None")


# ---------------------------------------------------------------------------
# the fake
# ---------------------------------------------------------------------------

class _FakeResponse:
    """Enough of a pymodbus response for :class:`PlcClient` to read."""

    def __init__(self, *, registers_: list[int] | None = None,
                 bits: list[bool] | None = None, error: bool = False) -> None:
        self.registers = registers_ if registers_ is not None else []
        self.bits = bits if bits is not None else []
        self._error = error

    def isError(self) -> bool:                                      # noqa: N802
        """pymodbus spells it this way; the name is theirs, not ours."""
        return self._error

    def __repr__(self) -> str:
        return "_FakeResponse(error=%r, registers=%r, bits=%r)" % (
            self._error, self.registers, self.bits)


class FakePlc:
    """An in-memory CLICK: register bank, coil bank, and fault injection.

    It implements the four pymodbus 3.15 methods :class:`PlcClient` actually
    calls, with **the same keyword signatures** (``count=``, ``device_id=``) --
    so a call that would fail against the real library fails here too. That is
    the point: a fake with looser signatures verifies nothing.

    Seed it from ``{df_number: float}``. Faults:

    ``fail_connect``      :func:`connect` returns False.
    ``raise_on``          method names that raise :class:`OSError`.
    ``error_on``          method names that return an error response.
    ``short_registers``   truncate every register read to this many words.
    """

    def __init__(self, values: dict[int, float] | None = None, *,
                 coils: dict[int, bool] | None = None,
                 fail_connect: bool = False,
                 raise_on: set[str] | None = None,
                 error_on: set[str] | None = None,
                 short_registers: int | None = None) -> None:
        self.registers: dict[int, int] = {}
        for df, value in (values or {}).items():
            low_word, high_word = registers.f32_to_regs(value)
            address = registers.df_address(df)
            self.registers[address] = low_word
            self.registers[address + 1] = high_word
        self.coils: dict[int, bool] = dict(coils or {})
        self.fail_connect = fail_connect
        self.raise_on = frozenset(raise_on or ())
        self.error_on = frozenset(error_on or ())
        self.short_registers = short_registers
        self.connected = False
        #: Every call as ``(method, kwargs)``, so a test can assert that a
        #: write happened exactly once with exactly two words.
        self.calls: list[tuple[str, dict[str, Any]]] = []

    # -- introspection for tests ----------------------------------------
    def calls_of(self, method: str) -> list[tuple[str, dict[str, Any]]]:
        return [call for call in self.calls if call[0] == method]

    def count(self, method: str) -> int:
        return len(self.calls_of(method))

    def value_at(self, df: int) -> float:
        """Decode DF ``df`` out of the bank, through the real codec."""
        address = registers.df_address(df)
        return registers.regs_to_f32(self.registers.get(address, 0),
                                     self.registers.get(address + 1, 0))

    def _record(self, method: str, **kwargs: Any) -> None:
        self.calls.append((method, kwargs))
        if method in self.raise_on:
            raise OSError("FakePlc: injected transport fault on %s" % method)

    # -- the pymodbus surface -------------------------------------------
    def connect(self) -> bool:
        self._record("connect")
        self.connected = not self.fail_connect
        return self.connected

    def close(self) -> None:
        self._record("close")
        self.connected = False

    def read_holding_registers(self, address: int, *, count: int = 1,
                               device_id: int = 1,
                               no_response_expected: bool = False) -> _FakeResponse:
        self._record("read_holding_registers", address=address, count=count,
                     device_id=device_id)
        if "read_holding_registers" in self.error_on:
            return _FakeResponse(error=True)
        if self.short_registers is not None:
            count = min(count, self.short_registers)
        return _FakeResponse(registers_=[self.registers.get(address + offset, 0)
                                         for offset in range(count)])

    def write_registers(self, address: int, values: list[int], *,
                        device_id: int = 1,
                        no_response_expected: bool = False) -> _FakeResponse:
        self._record("write_registers", address=address, values=list(values),
                     device_id=device_id)
        if "write_registers" in self.error_on:
            return _FakeResponse(error=True)
        for offset, word in enumerate(values):
            self.registers[address + offset] = int(word) & 0xFFFF
        return _FakeResponse()

    def read_coils(self, address: int, *, count: int = 1, device_id: int = 1,
                   no_response_expected: bool = False) -> _FakeResponse:
        self._record("read_coils", address=address, count=count,
                     device_id=device_id)
        if "read_coils" in self.error_on:
            return _FakeResponse(error=True)
        return _FakeResponse(bits=[bool(self.coils.get(address + offset, False))
                                   for offset in range(count)])

    def write_coil(self, address: int, value: bool, *, device_id: int = 1,
                   no_response_expected: bool = False) -> _FakeResponse:
        self._record("write_coil", address=address, value=bool(value),
                     device_id=device_id)
        if "write_coil" in self.error_on:
            return _FakeResponse(error=True)
        self.coils[address] = bool(value)
        return _FakeResponse()


__all__ = [
    "CONFIRMED",
    "DEFAULT_DEVICE_ID",
    "DEFAULT_HOST",
    "DEFAULT_PORT",
    "FAILED",
    "MAX_CONCURRENT_CLIENTS",
    "NATIVE_DIAGNOSTICS",
    "PLANNED",
    "PUBLISH_DIR",
    "FakePlc",
    "PlcClient",
    "PlcSettings",
    "WriteResult",
    "latest_published",
    "publish_last_reading",
]
