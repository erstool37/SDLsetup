#!/usr/bin/env python3
"""Regression: the declared safe setpoint must be held to the bound the abort
path will actually use.

The defect this locks out (found 2026-09-07, after the live PLC read):
``RelayPolicy.__post_init__`` validated ``safe_setpoint_c`` against
``CommandLimits()`` -- the *code ceiling* (0, 30) -- while ``configs/config.yaml``
tightens the floor to 10.0, the PLC's **measured** ``DF21 Temp Output LB``. So a
safe setpoint in (0, 10] constructed cleanly and was then refused by
``circulator.bound()`` *during an abort*, which is the one moment a fail-safe
must not fail.

Run: python3 dev/tests/test_relay_safe_setpoint_bound.py
"""
from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

from tools.circulator.api import CirculatorSettings  # noqa: E402
from tools.environment.relay import RelayPolicy  # noqa: E402

FAILURES: list[str] = []


def ok(cond: bool, msg: str) -> None:
    if cond:
        print(f"[test] ok: {msg}")
    else:
        print(f"[test] FAIL: {msg}")
        FAILURES.append(msg)


def refused(value: float) -> bool:
    try:
        RelayPolicy.from_config(safe_setpoint_c=value)
    except Exception:
        return True
    return False


def main() -> int:
    circ = CirculatorSettings.from_config()
    lo, hi = circ.limits.min_c, circ.limits.max_c
    print(f"[test] resolved circulator bound: ({lo}, {hi}) exclusive")

    # The premise. If config stops tightening the floor this test is vacuous,
    # so assert the premise rather than silently testing nothing.
    ok(lo > 0.0,
       f"config tightens the floor above the code ceiling's 0.0 (got {lo}) -- "
       f"without this the regression cannot be expressed")

    # The defect itself: strictly inside the code ceiling, outside the resolved bound.
    for value in (0.5, 5.0, lo - 0.1):
        ok(refused(value),
           f"safe_setpoint_c={value} is refused at CONSTRUCTION "
           f"(inside code ceiling, outside resolved bound)")

    # The bound is open at both ends, matching the circulator's own semantics.
    ok(refused(lo), f"safe_setpoint_c={lo} (the floor itself) is refused -- open interval")
    ok(refused(hi), f"safe_setpoint_c={hi} (the ceiling itself) is refused -- open interval")
    ok(refused(hi + 1.0), f"safe_setpoint_c={hi + 1.0} above the bound is refused")

    # A usable value still works, and reports the bound it was held to.
    mid = (lo + hi) / 2.0
    try:
        policy = RelayPolicy.from_config(safe_setpoint_c=mid)
    except Exception as exc:  # pragma: no cover - would be a real break
        ok(False, f"safe_setpoint_c={mid} should construct, raised {exc!r}")
        policy = None
    if policy is not None:
        ok(policy.safe_setpoint_c == mid, f"safe_setpoint_c={mid} constructs")
        ok(policy.effective_limits.min_c == lo and policy.effective_limits.max_c == hi,
           "effective_limits is the RESOLVED bound, not the code ceiling")
        ok(str(lo) in policy.describe() or f"{lo:g}" in policy.describe(),
           "describe() reports the bound actually in force")

    # The error must tell the operator what to do, not just that it failed.
    try:
        RelayPolicy.from_config(safe_setpoint_c=5.0)
        ok(False, "5.0 should have raised")
    except Exception as exc:
        text = str(exc)
        ok("safe_setpoint_c" in text, "the error names the config key")
        ok(f"{lo:g}" in text and f"{hi:g}" in text,
           "the error quotes the RESOLVED range, not the code ceiling")

    # F8: the resolved bound cannot be dropped by normal dataclass operations.
    import dataclasses as _dc
    good = RelayPolicy.from_config(safe_setpoint_c=mid)
    try:
        _dc.replace(good, safe_setpoint_c=5.0, command_limits=None)
        ok(False, "replace(command_limits=None) must raise, not fall back to (0,30)")
    except Exception:
        ok(True, "replace(safe_setpoint_c=5.0, command_limits=None) raises (no wide fallback)")
    try:
        RelayPolicy(safe_setpoint_c=20.0)
        ok(False, "direct RelayPolicy without command_limits must raise")
    except Exception:
        ok(True, "direct RelayPolicy without command_limits raises (bound is required)")

    print()
    if FAILURES:
        print(f"[test] {len(FAILURES)} FAILURE(S)")
        return 1
    print("[test] ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
