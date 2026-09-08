#!/usr/bin/env python3
"""Regression (Codex review A, F2a): a validated setpoint token must be IMMUTABLE.

The defect: BoundedSetpoint used __slots__ but allowed attribute assignment, so
    sp = circulator.bound(15.0)
    sp.value_c = 35.0
    circulator.write_setpoint(sp)   # encodes and sends 35.0
reached the wire past the only bound the device has. The PLC token likewise let
value / field / spec be reassigned, including redirecting spec to a read-only or
fabricated register.

Run: python3 dev/tests/test_bounded_setpoint_immutable.py
"""
from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

from tools.circulator.safety import CommandLimits  # noqa: E402
from tools.environment import registers as _reg  # noqa: E402
from tools.environment.safety import SetpointLimits  # noqa: E402

FAILURES: list[str] = []


def ok(cond: bool, msg: str) -> None:
    print(f"[test] {'ok' if cond else 'FAIL'}: {msg}")
    if not cond:
        FAILURES.append(msg)


def blocks(fn, what: str) -> None:
    """A mutation attempt must raise, not silently succeed."""
    try:
        fn()
    except Exception:
        ok(True, what)
        return
    ok(False, what)


def main() -> int:
    # -- circulator token -------------------------------------------------
    sp = CommandLimits(min_c=10.0, max_c=30.0).validate(15.0)
    ok(sp.value_c == 15.0, "circulator token carries the validated value")
    blocks(lambda: setattr(sp, "value_c", 35.0),
           "circulator token refuses value_c reassignment (the 35 C wire bypass)")
    blocks(lambda: setattr(sp, "limits", CommandLimits(min_c=0.0, max_c=30.0)),
           "circulator token refuses limits reassignment")
    blocks(lambda: delattr(sp, "value_c"),
           "circulator token refuses attribute deletion")
    ok(sp.value_c == 15.0, "circulator token value is unchanged after the attempts")

    # -- PLC token --------------------------------------------------------
    lim = SetpointLimits()
    tsp = lim.validate("temp_sp_c", 20.0)
    ok(tsp.value == 20.0, "PLC token carries the validated value")
    blocks(lambda: setattr(tsp, "value", 40.0),
           "PLC token refuses value reassignment")
    blocks(lambda: setattr(tsp, "field", "rh_sp_pct"),
           "PLC token refuses field reassignment")
    # redirecting spec to a read-only register was an explicit review scenario
    ro_spec = _reg.BY_FIELD["temp_filtered_c"]  # rw == "R"
    blocks(lambda: setattr(tsp, "spec", ro_spec),
           "PLC token refuses spec redirection to a read-only register")
    ok(tsp.value == 20.0 and tsp.field == "temp_sp_c",
       "PLC token value and field are unchanged after the attempts")

    print()
    if FAILURES:
        print(f"[test] {len(FAILURES)} FAILURE(S)")
        return 1
    print("[test] ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
