"""The circulator's main API -- what an orchestration script calls.

    from tools import circulator

    dev = circulator.Circulator.from_config("configs/config.yaml")
    print(dev.settings.describe())
    print(dev.port_present())

    sp = dev.bound(24.774)              # raises unless 0.0 < v < 30.0 (strict)
    result = dev.write_setpoint(sp)     # dry-run plan unless allow_actuation
    if result.outcome == "failed":      # NOT `not result.ok` -- a plan is ok=False too
        ...                             # the SCRIPT decides what to do

Three things this layer refuses to guess
========================================

**Whether the port may be opened.** ``allow_actuation`` defaults to False and a
dry run is the default. On this device that gate is not a convenience: the
FT232R bridge asserts DTR on open and DTR is coupled to /RESET, so **opening
the port hardware-resets the microcontroller.** A dry run therefore does not
merely skip the write -- it never constructs a transport and never opens
anything. See :mod:`tools.circulator.link`.

**What a setpoint is allowed to be.** Only a
:class:`~tools.circulator.safety.BoundedSetpoint` reaches
:meth:`Circulator.write_setpoint`, and that type exists only as the output of
:meth:`~tools.circulator.safety.CommandLimits.validate`. A bare ``float``
cannot reach the wire. The bound is 0.0-30.0 C inclusive, **INFERRED** from the
prior system's observed output envelope -- this custom board's own protections,
if it has any, are unknown.

**Whether a write landed.** :class:`~tools.circulator.link.WriteResult` carries
the FC16 echo. The prior code assigned the return value and never read it.

What this layer does NOT do
===========================

It does not decide. There is no retry, no fallback setpoint, no clamp, and no
"good enough". A refused bound raises; a failed write comes back as
``outcome="failed"`` with the observed echo, and the phase script chooses.

It also cannot read anything back. **This device has no read-back** -- no
temperature, no status, no alarm. The prior code's only read was commented out.
Do not add one on the assumption that a controller must have one.
"""
from __future__ import annotations

import dataclasses
from pathlib import Path
from typing import Any

from .. import config as _config
from .codec import SETPOINT_ADDRESS, SETPOINT_COUNT, encode_setpoint
from .link import SerialLink, WriteResult
from .safety import (
    COMMAND_MAX_C,
    COMMAND_MIN_C,
    ActuationNotAllowed,
    BoundedSetpoint,
    CirculatorError,
    CommandLimits,
    SafetyError,
)

#: Hardware inventory shipped beside this module.
INVENTORY_PATH = Path(__file__).resolve().parent / "circulator.json"

#: Section name in ``configs/config.yaml``.
CONFIG_SECTION = "circulator"

#: Transport defaults, verbatim from the prior working code:
#: ``ModbusSerialClient('COM3', baudrate=9600, parity='N', stopbits=1,
#: bytesize=8, timeout=1)``. The port itself is NOT carried over -- ``COM3`` was
#: a Windows name and this runs under WSL.
DEFAULT_BAUDRATE = 9600
DEFAULT_PARITY = "N"
DEFAULT_STOPBITS = 1
DEFAULT_BYTESIZE = 8
DEFAULT_TIMEOUT_S = 1.0

#: Modbus slave id. INFERENCE, not a documented fact: neither the Python nor the
#: MATLAB version ever specified one, so both used the client library's default
#: of 1. No device ever confirmed it.
DEFAULT_UNIT_ID = 1

#: Seconds to wait after an open before any frame is permitted, because the open
#: reset the MCU and the firmware has to boot. NOT measured on this board -- no
#: firmware source or schematic exists to derive it from, and the port has not
#: been opened. 3.0 s is the conventional generous allowance for an AVR-class
#: bootloader plus application start.
#: TODO(operator): measure the real boot time once the bridge is attached, and
#: replace this with the observed value plus margin.
DEFAULT_BOOT_SETTLE_S = 3.0

# Mark/space parity is deliberately EXCLUDED: this board has only ever run
# "N", and neither the framing nor pymodbus RTU behaviour under M/S can be
# verified here. A narrower set turns an unverifiable setting into a refusal.
_PARITIES = ("N", "E", "O")
_BYTESIZES = (5, 6, 7, 8)
_STOPBITS = (1, 1.5, 2)


@dataclasses.dataclass(frozen=True)
class CirculatorSettings:
    """Everything the circulator needs, with the layer each value came from.

    ``sources`` is what a run logs so a later reader can tell whether a limit
    came from a flag, from ``config.yaml``, from ``circulator.json``, or from
    the code default. A safety limit whose provenance is unknown is not a
    safety limit.

    Cross-field validation happens here, in ``__post_init__``, and not at the
    call sites -- so there is no way to hold an invalid settings object.
    """

    port: str | None = None
    baudrate: int = DEFAULT_BAUDRATE
    parity: str = DEFAULT_PARITY
    stopbits: float = DEFAULT_STOPBITS
    bytesize: int = DEFAULT_BYTESIZE
    timeout_s: float = DEFAULT_TIMEOUT_S
    unit_id: int = DEFAULT_UNIT_ID
    allow_actuation: bool = False
    boot_settle_s: float = DEFAULT_BOOT_SETTLE_S
    command_min_c: float = COMMAND_MIN_C
    command_max_c: float = COMMAND_MAX_C
    sources: dict = dataclasses.field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.port is not None and not str(self.port).strip():
            raise CirculatorError(
                "port is an empty string. Write null (or leave it out) to mean "
                "'no port configured'; an empty string reads as a real path.")
        if not isinstance(self.baudrate, int) or isinstance(self.baudrate, bool) \
                or self.baudrate <= 0:
            raise CirculatorError(f"baudrate must be a positive int, got {self.baudrate!r}")
        if self.parity not in _PARITIES:
            # An UNQUOTED `parity: N` in config.yaml parses to boolean False --
            # tools.config._parse_scalar accepts the YAML 1.1 false-words
            # {false, no, off, n}. Verified: _parse_scalar("N") -> False. So this
            # check is load-bearing, not decorative: False reaching the client
            # would be a baffling framing failure rather than a clear refusal.
            raise CirculatorError(
                f"parity must be one of {_PARITIES}, got {self.parity!r}. If this is "
                f"False, config.yaml wrote an UNQUOTED N -- this file's parser reads "
                f"YAML 1.1 booleans, so it must be written parity: \"N\".")
        if self.stopbits not in _STOPBITS:
            raise CirculatorError(
                f"stopbits must be one of {_STOPBITS}, got {self.stopbits!r}")
        if self.bytesize not in _BYTESIZES:
            raise CirculatorError(
                f"bytesize must be one of {_BYTESIZES}, got {self.bytesize!r}")
        if not isinstance(self.timeout_s, (int, float)) or self.timeout_s <= 0:
            raise CirculatorError(
                f"timeout_s must be a positive number, got {self.timeout_s!r}")
        if not isinstance(self.unit_id, int) or isinstance(self.unit_id, bool) \
                or not 0 <= self.unit_id <= 247:
            raise CirculatorError(
                f"unit_id must be a Modbus slave id in 0..247, got {self.unit_id!r}")
        if not isinstance(self.boot_settle_s, (int, float)) or self.boot_settle_s < 0:
            raise CirculatorError(
                f"boot_settle_s must be a non-negative number, got {self.boot_settle_s!r}")
        if not isinstance(self.allow_actuation, bool):
            raise CirculatorError(
                f"allow_actuation must be a bool, got {self.allow_actuation!r}; "
                f"never truthiness -- every non-empty string is truthy, so a typo "
                f"would switch actuation ON")
        # Raises if the pair is inverted, non-finite, or WIDER than the code
        # ceiling. Config may tighten this range, never loosen it.
        CommandLimits(min_c=float(self.command_min_c), max_c=float(self.command_max_c))

    # -- construction -----------------------------------------------------
    @classmethod
    def from_config(cls, config: Any = None, **overrides: Any) -> CirculatorSettings:
        """Resolve settings through the precedence contract in :mod:`tools.config`.

        ``overrides`` are CLI-flag level (highest precedence). Passing None for
        an override means "not specified", so a flag that was not given never
        shadows the config file. A value present but unparseable RAISES -- it
        never falls through to the default, because a typo that quietly selects
        a permissive limit is the failure the layering exists to prevent.
        """
        lab = _config.LabConfig.load(config)
        section = lab.section(CONFIG_SECTION)
        inventory = _config.load_inventory(INVENTORY_PATH)
        serial_inv = inventory.get("serial", {}) if isinstance(inventory, dict) else {}

        resolved: dict[str, Any] = {}
        sources: dict[str, str] = {}

        def scalar(where: str, mapping: dict, key: str) -> Any:
            """One layer's value for ``key``, with the parser's blank-key case handled.

            ``tools.config``'s parser turns a bare ``key:`` with nothing after
            it into an empty MAPPING, not None -- it cannot tell "null" from
            "a nested block follows". Left alone, ``resolve(cast=str)`` then
            casts ``{}`` to the literal string ``'{}'``, and for ``port`` that is
            a device path. Observed here: ``configs/config.yaml`` writes
            ``port:`` blank on purpose to mean "not set yet", and the resolved
            port came back as ``'{}'``.

            So an EMPTY mapping is read as absent, which is what the operator
            meant. A NON-empty mapping where a scalar belongs is a malformed
            file and raises -- never a silent fallthrough.
            """
            value = mapping.get(key)
            if isinstance(value, dict):
                if value:
                    raise _config.ConfigError(
                        f"{where}: {CONFIG_SECTION}.{key} is a nested block, but a "
                        f"single value is expected here (got keys "
                        f"{sorted(value)}); refusing to guess which one is meant")
                return None
            return value

        def take(key: str, default: Any, cast: type | None = None,
                 inv: dict | None = None) -> None:
            inv_map = inventory if inv is None else inv
            item = _config.resolve(
                key, argument=overrides.get(key),
                config={key: scalar("config.yaml", section, key)},
                inventory={key: scalar("circulator.json", inv_map, key)},
                default=default, inventory_name="circulator.json", cast=cast,
            )
            resolved[key] = item.value
            sources[key] = item.source

        take("port", None, str)
        take("baudrate", DEFAULT_BAUDRATE, int, inv=serial_inv)
        take("parity", DEFAULT_PARITY, str, inv=serial_inv)
        take("stopbits", DEFAULT_STOPBITS, int, inv=serial_inv)
        take("bytesize", DEFAULT_BYTESIZE, int, inv=serial_inv)
        take("timeout_s", DEFAULT_TIMEOUT_S, float, inv=serial_inv)
        take("unit_id", DEFAULT_UNIT_ID, int)
        take("allow_actuation", False, bool)
        take("boot_settle_s", DEFAULT_BOOT_SETTLE_S, float)
        take("command_min_c", COMMAND_MIN_C, float)
        take("command_max_c", COMMAND_MAX_C, float)

        try:
            return cls(sources=sources, **resolved)
        except CirculatorError as exc:
            # Surface a bad FILE as a ConfigError, so a caller distinguishes
            # "the configuration is wrong" from "this call is wrong".
            raise _config.ConfigError(
                f"{CONFIG_SECTION}: {exc} (sources: "
                f"{', '.join('%s=%s' % kv for kv in sorted(sources.items()))})"
            ) from exc

    # -- reporting --------------------------------------------------------
    @property
    def limits(self) -> CommandLimits:
        """The command bound this settings object authorises."""
        return CommandLimits(min_c=float(self.command_min_c),
                             max_c=float(self.command_max_c))

    def describe(self) -> str:
        lines = [f"circulator settings (port={self.port!r}, "
                 f"allow_actuation={self.allow_actuation})"]
        for field in dataclasses.fields(self):
            if field.name == "sources":
                continue
            value = getattr(self, field.name)
            lines.append("  %-16s %-24r [%s]"
                         % (field.name, value, self.sources.get(field.name, "-")))
        lines.append("  " + self.limits.describe())
        return "\n".join(lines)


class Circulator:
    """The circulator facade. Reports and refuses; it never decides.

    Dry run (``allow_actuation=False``, the default) is a plan: the frame is
    encoded and returned, no transport is constructed, and the port is never
    opened -- which on this device means the MCU is never reset.
    """

    def __init__(self, settings: CirculatorSettings | None = None,
                 transport: Any = None, *,
                 log: Any = None, clock: Any = None, sleep: Any = None) -> None:
        self.settings = settings if settings is not None else CirculatorSettings()
        self._link = SerialLink(self.settings, transport, log=log,
                                clock=clock, sleep=sleep)

    @classmethod
    def from_config(cls, config: Any = None, **overrides: Any) -> Circulator:
        transport = overrides.pop("transport", None)
        log = overrides.pop("log", None)
        return cls(CirculatorSettings.from_config(config, **overrides),
                   transport, log=log)

    # -- observation ------------------------------------------------------
    @property
    def link(self) -> SerialLink:
        """The transport session. Reading it never builds or opens anything.

        H8: this hands out the GUARDED :class:`~.link.SerialLink` wrapper, whose
        write path takes only a canonical codec-built frame (H3) and whose raw
        vendor client is now private (``SerialLink._transport``). The token/frame
        guards prevent ACCIDENTAL misuse, not a determined caller using private
        names or ``object.__setattr__``. (This accessor is not itself renamed
        private, because the dashboard node reads ``link.is_open`` through it.)
        """
        return self._link

    @property
    def limits(self) -> CommandLimits:
        return self.settings.limits

    @property
    def open_count(self) -> int:
        """Ports opened in this session -- i.e. **MCU reset events**."""
        return self._link.open_count

    def port_present(self) -> dict:
        """Does the device node exist? ``stat`` only -- the port is never opened."""
        return self._link.port_present()

    # -- the bound --------------------------------------------------------
    def bound(self, value_c: float) -> BoundedSetpoint:
        """Validate a Celsius setpoint against this run's limits. Raises; never clamps."""
        return self.limits.validate(value_c)

    # -- actuation --------------------------------------------------------
    def open(self, settle: bool = True) -> dict:
        """Open the port. **Resets the MCU.** Refused unless allow_actuation is set."""
        return self._link.open(settle=settle)

    def close(self) -> None:
        self._link.close()

    def write_setpoint(self, sp: BoundedSetpoint) -> WriteResult:
        """Write the setpoint register block. Accepts ONLY a bounded setpoint.

        Dry run returns the frame it would have sent as
        ``outcome="planned"`` -- and therefore ``ok=False``, because nothing was
        written -- having constructed no transport and opened nothing.
        """
        if not isinstance(sp, BoundedSetpoint):
            raise SafetyError(
                f"write_setpoint requires a BoundedSetpoint, got "
                f"{type(sp).__name__} {sp!r}. Call .bound(value) first: this bound "
                f"is the ONLY temperature limit that exists for this device, which "
                f"is a custom board with no documented clamp of its own."
            )
        # F2b: the token proves *a* bound ran, not that it ran against THIS
        # client's limits. A token validated under a wider or different range
        # (e.g. (0, 30) while this run's bound is (10, 30)) must not be trusted
        # blindly across the boundary -- re-validate its value against the
        # receiving client's own bound, which raises if it is outside.
        # H4: snapshot the token's value ONCE, then use only the local. A
        # subclass with a stateful __getattribute__ could otherwise pass the
        # re-validation below and hand a DIFFERENT value to the encoder.
        value_c = sp.value_c
        self.limits.validate(value_c)
        values = encode_setpoint(value_c)
        if not self.settings.allow_actuation:
            # outcome="planned", so `ok` is False: nothing was written, and
            # that is the honest answer. See WriteResult's docstring.
            return WriteResult(
                outcome=WriteResult.PLANNED, address=SETPOINT_ADDRESS,
                values=values,
            )
        # F2c: the raw wire takes ONLY a private fixed-frame object, built solely
        # by the setpoint codec at the canonical address/count -- never an
        # arbitrary (address, values) pair.
        return self._link.write_registers(self._link.encode_frame(value_c))

    def plan_setpoint(self, sp: BoundedSetpoint) -> dict:
        """What a write would send, without sending it. Never opens the port."""
        if not isinstance(sp, BoundedSetpoint):
            raise SafetyError("plan_setpoint requires a BoundedSetpoint")
        return {"value_c": sp.value_c, "address": SETPOINT_ADDRESS,
                "count": SETPOINT_COUNT, "values": encode_setpoint(sp.value_c),
                "unit_id": self.settings.unit_id,
                "unit_id_provenance": "inference (library default; never specified "
                                      "in prior code)"}

    def __enter__(self) -> Circulator:
        if self.settings.allow_actuation:
            self._link.open()
        return self

    def __exit__(self, *exc_info: object) -> None:
        self._link.close()


__all__ = [
    "CONFIG_SECTION",
    "DEFAULT_BAUDRATE",
    "DEFAULT_BOOT_SETTLE_S",
    "DEFAULT_BYTESIZE",
    "DEFAULT_PARITY",
    "DEFAULT_STOPBITS",
    "DEFAULT_TIMEOUT_S",
    "DEFAULT_UNIT_ID",
    "INVENTORY_PATH",
    "ActuationNotAllowed",
    "Circulator",
    "CirculatorSettings",
]
