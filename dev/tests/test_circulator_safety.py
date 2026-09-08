#!/usr/bin/env python3
"""The circulator's command bound, and the token that makes it unskippable.

No hardware, no serial port, no network. Every object here is built from
numbers.

Why this bound is entirely ours: the device is a custom MCU board with no
vendor, no manual, no firmware source and no schematic. Whether it has an
over-temperature cutoff or a setpoint clamp of its own is UNKNOWABLE, so the
only bound that exists is the one in this layer.

    python dev/tests/test_circulator_safety.py
"""
from __future__ import annotations

import dataclasses
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from tools import config as _config  # noqa: E402
from tools.circulator.circulator import Circulator, CirculatorSettings  # noqa: E402
from tools.circulator.safety import (  # noqa: E402
    COMMAND_MAX_C,
    COMMAND_MIN_C,
    ActuationNotAllowed,
    BoundedSetpoint,
    CirculatorError,
    CommandLimits,
    SafetyError,
)

fails = 0


def ok(condition: bool, message: str, detail: str = "") -> None:
    global fails
    print("[test] %s: %s%s" % ("PASS" if condition else "FAIL", message,
                               f"  ({detail})" if detail else ""))
    if not condition:
        fails += 1


def blocked(fn, message: str, exc: type = SafetyError) -> None:
    global fails
    try:
        value = fn()
    except exc as caught:
        print("[test] PASS: %s  -> %s" % (message, str(caught)[:70]))
        return
    except Exception as other:
        print("[test] FAIL: %s -- raised %s, not %s"
              % (message, type(other).__name__, exc.__name__))
        fails += 1
        return
    print("[test] FAIL: %s -- nothing raised, returned %r" % (message, value))
    fails += 1


LIMITS = CommandLimits()

print("--- the bound and where it came from ---")
ok((COMMAND_MIN_C, COMMAND_MAX_C) == (0.0, 30.0),
   "the code ceiling is 0.0 to 30.0 C (INFERRED from the prior system's logs)")
ok((LIMITS.min_c, LIMITS.max_c) == (0.0, 30.0), "and is the default CommandLimits")
ok(dataclasses.is_dataclass(LIMITS) and LIMITS.__dataclass_params__.frozen,
   "CommandLimits is frozen -- a run cannot edit its own bound mid-run")

print("\n--- the interval is OPEN: strictly inside is accepted ---")
for value in (0.001, 4.0, 15.0, 24.774, 29.999):
    sp = LIMITS.validate(value)
    ok(isinstance(sp, BoundedSetpoint) and sp.value_c == value,
       "validate accepts %r" % value)

print("\n--- BOTH ENDPOINTS ARE REFUSED (strict, matching the MATLAB original) ---")
# 30.000 is exactly what a pinned, failing controller emits: the 2026-01-11 log
# ends with the output saturated at 30.000 while the measurement read 11.4 C.
# So the envelope maximum is partly an artifact, and is the worst candidate for
# a legal command. The PLC-side guard is strict for the same reason -- neither
# device is the odd one out at its limits.
blocked(lambda: LIMITS.validate(30.0), "exactly 30.0 is refused (saturation artifact)")
blocked(lambda: LIMITS.validate(0.0), "exactly 0.0 is refused")
blocked(lambda: LIMITS.validate(-0.0), "and so is negative zero")

print("\n--- and outside them too; it never clamps ---")
blocked(lambda: LIMITS.validate(30.001), "30.001 is refused, not clamped to 30.0")
blocked(lambda: LIMITS.validate(-0.001), "-0.001 is refused, not clamped to 0.0")
blocked(lambda: LIMITS.validate(50.0), "50.0 is refused")
blocked(lambda: LIMITS.validate(-40.0), "-40.0 is refused")
blocked(lambda: LIMITS.validate(float("nan")), "NaN is refused")
blocked(lambda: LIMITS.validate(float("inf")), "+inf is refused")
blocked(lambda: LIMITS.validate(float("-inf")), "-inf is refused")
blocked(lambda: LIMITS.validate("25"), "a string is refused rather than coerced")
blocked(lambda: LIMITS.validate(None), "None is refused")

print("\n--- the token: a BoundedSetpoint is evidence a guard ran ---")
blocked(lambda: BoundedSetpoint(25.0, LIMITS),
        "BoundedSetpoint(25.0, limits) cannot be built directly")
blocked(lambda: BoundedSetpoint(object(), 25.0, LIMITS),
        "nor with a token of our own making")
sp = LIMITS.validate(25.0)
ok(sp.limits is LIMITS, "the token carries the limits that approved it")
ok("25" in repr(sp), "and says what it is", repr(sp))

print("\n--- a tightened bound is honoured; the token remembers which one ---")
tight = CommandLimits(min_c=4.0, max_c=20.0)
ok(tight.validate(4.001).value_c == 4.001, "a tightened range accepts just inside itself")
blocked(lambda: tight.validate(4.0), "but not its own endpoint -- strict there too")
blocked(lambda: tight.validate(25.0),
        "25.0 is refused under a 4-20 C run bound even though the ceiling allows it")
ok(CommandLimits(min_c=0.0, max_c=30.0).max_c == 30.0,
   "a RANGE of exactly the ceiling is legal, even though its endpoints are not")
blocked(lambda: CommandLimits(min_c=10.0, max_c=10.0),
        "an EMPTY open interval is refused: it would accept nothing at all")

print("\n--- but config may only TIGHTEN the range, never widen it ---")
blocked(lambda: CommandLimits(min_c=0.0, max_c=45.0),
        "max_c=45.0 is refused: it would widen past the 30.0 C code ceiling")
blocked(lambda: CommandLimits(min_c=-10.0, max_c=30.0),
        "min_c=-10.0 is refused: it would widen past the 0.0 C code floor")
blocked(lambda: CommandLimits(min_c=20.0, max_c=10.0), "an inverted range is refused")
blocked(lambda: CommandLimits(min_c=0.0, max_c=float("nan")), "a NaN bound is refused")
blocked(lambda: CirculatorSettings(command_max_c=45.0),
        "CirculatorSettings cannot widen the bound either", CirculatorError)
blocked(lambda: CirculatorSettings.from_config({"circulator": {"command_max_c": 45.0}}),
        "and config.yaml cannot widen it", _config.ConfigError)
tightened = CirculatorSettings.from_config({"circulator": {"command_max_c": 20.0}})
ok(tightened.command_max_c == 20.0, "config.yaml CAN tighten it")
ok(tightened.sources.get("command_max_c") == "config.yaml",
   "and the settings record which layer that came from",
   tightened.sources.get("command_max_c"))

print("\n--- an unparseable value raises; it never falls through to a default ---")
blocked(lambda: CirculatorSettings.from_config({"circulator": {"command_max_c": "warm"}}),
        "command_max_c: warm is an error, not a silent 30.0", _config.ConfigError)
blocked(lambda: CirculatorSettings.from_config({"circulator": {"allow_actuation": "flase"}}),
        "a typo'd actuation gate is an error, not True", _config.ConfigError)

print("\n--- an unbounded float cannot reach the wire ---")
dev = Circulator(CirculatorSettings(port="/dev/null-not-opened", allow_actuation=False))
blocked(lambda: dev.write_setpoint(25.0), "write_setpoint refuses a plain float")
blocked(lambda: dev.write_setpoint("25.0"), "write_setpoint refuses a string")
blocked(lambda: dev.write_setpoint(None), "write_setpoint refuses None")
result = dev.write_setpoint(dev.bound(25.0))
ok(result.outcome == "planned" and result.values == [0, 0, 0, 0x3940],
   "and accepts the token, describing the frame it would have sent")
ok(result.ok is False, "a plan reports ok=False: nothing was written")
blocked(lambda: dev.bound(45.0), "Circulator.bound applies the same bound")
blocked(lambda: dev.bound(30.0), "including the strict upper endpoint")

print("\n--- the actuation gate is its own refusal, not a safety violation ---")
blocked(lambda: dev.open(), "open() is refused while allow_actuation is off",
        ActuationNotAllowed)
ok(issubclass(ActuationNotAllowed, CirculatorError) and issubclass(SafetyError, CirculatorError),
   "both refusals are CirculatorError, so one except clause catches the pair")

print("\n--- a bare `key:` means null, and must never become the string {} ---")
# tools.config's parser turns `port:` with nothing after it into an empty
# MAPPING, and resolve(cast=str) will happily cast that to the literal path
# "{}". configs/config.yaml writes `port:` blank on purpose, so this was live.
blank = CirculatorSettings.from_config({"circulator": {"port": {}, "baudrate": 9600}})
ok(blank.port is None, "a blank port resolves to None, not %r" % (blank.port,))
ok(blank.sources.get("port") == "default", "and is reported as coming from the default")
ok(CirculatorSettings.from_config().port is None,
   "the live configs/config.yaml resolves port to None as its comment intends")
blocked(lambda: CirculatorSettings.from_config({"circulator": {"port": {"nested": 1}}}),
        "a real nested block where a scalar belongs raises", _config.ConfigError)
blocked(lambda: CirculatorSettings(port="   "),
        "an all-whitespace port is refused rather than opened", CirculatorError)
ok(CirculatorSettings.from_config().sources.get("port") == "default",
   "and safe_setpoint_c, also blank in that section, is simply not ours to read")

print("\n--- an UNQUOTED parity: N parses to False, and must be refused ---")
# tools.config._parse_scalar accepts the YAML 1.1 false-words {false,no,off,n},
# so a bare N in config.yaml becomes boolean False. Verified in the parser.
ok(_config._parse_scalar("N") is False,
   "the repo parser really does read a bare N as False -- so this check is load-bearing")
blocked(lambda: CirculatorSettings(parity=False),
        "parity=False is refused, not coerced", CirculatorError)
blocked(lambda: CirculatorSettings(parity="X"), "an unknown parity letter is refused",
        CirculatorError)
blocked(lambda: CirculatorSettings(parity="M"),
        "mark parity is refused: never used on this board, unverifiable here",
        CirculatorError)
ok(CirculatorSettings(parity="N").parity == "N", "the quoted N is accepted")

print("\n%s" % ("ALL PASS" if fails == 0 else "%d FAILURE(S)" % fails))
sys.exit(1 if fails else 0)
