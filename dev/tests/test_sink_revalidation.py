#!/usr/bin/env python3
"""F2b: a bound token is re-validated at the SINK, not trusted blindly across a boundary.

A ``BoundedSetpoint`` proves that *a* bound ran -- not that it ran against the
limits of the client that finally puts it on the wire. A token validated under a
wider ``(0, 30)`` range must not be accepted by a client whose own bound is the
tightened ``(10, 30)``, nor may a token whose spec was redirected reach the PLC's
canonical setpoint register. Both sinks re-check.

No hardware, no serial port, no socket: every transport here is a fake.

    python dev/tests/test_sink_revalidation.py
"""
from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from tools.circulator.api import (  # noqa: E402
    Circulator,
    CirculatorSettings,
    CommandLimits,
    FakeSerial,
)
from tools.circulator.api import SafetyError as CircSafetyError  # noqa: E402
from tools.environment import registers  # noqa: E402
from tools.environment.plc import FakePlc, PlcClient, PlcSettings  # noqa: E402
from tools.environment.safety import SafetyError, SetpointLimits  # noqa: E402

fails = 0


def ok(condition: bool, message: str, detail: str = "") -> None:
    global fails
    print("[test] %s: %s%s" % ("PASS" if condition else "FAIL", message,
                               "  (%s)" % detail if detail else ""))
    if not condition:
        fails += 1


def raises(kind, call, message: str):
    try:
        call()
    except kind as exc:
        ok(True, message, "%s: %s" % (type(exc).__name__, str(exc)[:60]))
        return exc
    except Exception as exc:                                          # noqa: BLE001
        ok(False, message, "raised %s instead: %s" % (type(exc).__name__, exc))
        return None
    ok(False, message, "did not raise")
    return None


print("--- CIRCULATOR SINK: a token from a WIDER bound is refused by a tighter client ---")
# A token validated under the code ceiling (0, 30).
wide_token = CommandLimits(0.0, 30.0).validate(5.0)
ok(wide_token.value_c == 5.0, "5.0 C is a valid token under (0, 30)")

# A run whose resolved bound is the tightened (10, 30) -- the real rig's bound.
tight = Circulator(
    CirculatorSettings(port="/tmp/sdl-fake", allow_actuation=False,
                       command_min_c=10.0, command_max_c=30.0),
    transport=FakeSerial())
ok((tight.limits.min_c, tight.limits.max_c) == (10.0, 30.0),
   "the receiving client's own bound is (10, 30)")
raises(CircSafetyError, lambda: tight.write_setpoint(wide_token),
       "write_setpoint REFUSES the (0,30) token whose value 5.0 is outside (10,30)")

# The refusal happens even on a dry run (before the planned/actuate branch).
ok(tight.settings.allow_actuation is False, "and this client is a dry run -- the check is upstream of that")

# A token inside the sink's own bound still works.
good = tight.write_setpoint(tight.limits.validate(15.0))
ok(good.outcome == "planned" and good.values,
   "a token INSIDE the sink's bound is accepted (planned here)")

print("\n--- PLC SINK: a token from a WIDER bound is refused by a tighter client ---")
# temp bounds tighten UP from the code floor 0.0 -> a (10, 30) run bound.
wide_sp = SetpointLimits().validate("temp_sp_c", 5.0)          # valid under (0, 30)
ok(wide_sp.value == 5.0, "5.0 C is a valid PLC token under (0, 30)")
client = PlcClient(PlcSettings(allow_actuation=True, temp_sp_min_c=10.0),
                   FakePlc({5: 87.3}))
ok(client.settings.limits.temp_min_c == 10.0, "the client's own setpoint floor is 10.0")
raises(SafetyError, lambda: client.write_float32(wide_sp),
       "write_float32 REFUSES the (0,30) token whose value 5.0 is outside this client's (10,30)")

# A genuine token inside the sink's bound carries the canonical WRITABLE spec and
# is accepted through re-validation (the write itself is confirmed by the fake).
in_bound = client.settings.limits.validate("temp_sp_c", 20.0)
ok(in_bound.spec is registers.WRITABLE["temp_sp_c"],
   "a genuine token carries the canonical WRITABLE spec, so the sink's spec check passes")
res = client.write_float32(in_bound)
ok(res.outcome == "confirmed",
   "and a token inside the sink's bound with the canonical spec writes fine", res.outcome)

print("\n%s" % ("ALL PASS" if fails == 0 else "%d FAILURE(S)" % fails))
sys.exit(1 if fails else 0)
