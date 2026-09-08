#!/usr/bin/env python3
"""Hardening E: the LAST pass on the real operational path (P1-P7).

Closes the remaining coerce-instead-of-validate gaps on the paths a real run
actually takes -- a malformed register payload, the relay's read_ok/pid_enabled
gate, and plain construction of the limit/settings dataclasses -- plus the two
PLC protocol-address range checks. The dependency-injection seams (a hostile
duck-typed settings object, a NaN clock, a fake transport) are documented as
TRUSTED in the package READMEs and deliberately NOT chased here.

No hardware, no socket, no serial port (opening the circulator's port
hardware-resets its MCU, so no test may). Everything is numbers and fakes.

    python dev/tests/test_hardening_e.py
"""
from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from tools import config as _config  # noqa: E402
from tools.circulator.circulator import Circulator, CirculatorSettings  # noqa: E402
from tools.circulator.link import FakeSerial  # noqa: E402
from tools.circulator.safety import CirculatorError  # noqa: E402
from tools.environment import registers  # noqa: E402
from tools.environment.plc import FakePlc, PlcClient, PlcSettings  # noqa: E402
from tools.environment.reading import EnvironmentReading  # noqa: E402
from tools.environment.relay import (  # noqa: E402
    SOURCE_CHANNEL,
    Relay,
    RelayPolicy,
)
from tools.environment.safety import (  # noqa: E402
    ActuationNotAllowed,
    RateLimiter,
    SafetyError,
    SetpointLimits,
)

fails = 0


def ok(condition, message, detail=""):
    global fails
    print("[test] %s: %s%s" % ("PASS" if condition else "FAIL", message,
                               "  (%s)" % detail if detail else ""))
    if not condition:
        fails += 1


def raises(exc, fn, message):
    global fails
    try:
        fn()
    except exc as caught:
        print("[test] PASS: %s  -> %s" % (message, str(caught)[:60]))
        return
    except Exception as other:  # noqa: BLE001
        print("[test] FAIL: %s -- raised %s instead: %s"
              % (message, type(other).__name__, str(other)[:50]))
        fails += 1
        return
    print("[test] FAIL: %s -- nothing raised" % message)
    fails += 1


C1 = registers.C1_COIL
GOOD_ROW = {5: 25.000, 6: 95.000, 9: 24.774, 10: 25.340, 18: 24.866, 19: 95.180}
SAFE_C = 15.0
N = registers.BLOCK_COUNT


class Clock:
    def __init__(self, t=1000.0):
        self.t = float(t)

    def __call__(self):
        return self.t


def build():
    fake = FakePlc(dict(GOOD_ROW), coils={C1: True})
    client = PlcClient(PlcSettings(allow_actuation=True), fake)
    line = FakeSerial()
    device = Circulator(
        CirculatorSettings(port="/tmp/sdl-fake-circulator-e",
                           allow_actuation=True, boot_settle_s=0.0),
        line)
    device.open(settle=False)
    relay = Relay(client, device,
                  RelayPolicy(safe_setpoint_c=SAFE_C, command_limits=device.limits),
                  clock=Clock())
    return relay, fake, line


class _RegFake:
    """A transport whose holding-register read returns a caller-chosen payload,
    so a malformed word reaches _read_registers. Coils read clean True."""

    def __init__(self, words):
        self._words = list(words)

    def connect(self):
        return True

    def read_holding_registers(self, address, *, count=1, device_id=1,
                               no_response_expected=False):
        return type("R", (), {"registers": list(self._words),
                              "isError": lambda s: False})()

    def read_coils(self, address, *, count=1, device_id=1,
                   no_response_expected=False):
        return type("R", (), {"bits": [True], "isError": lambda s: False})()


def _reg_read_ok(words):
    return PlcClient(PlcSettings(), _RegFake(words)).read_block().read_ok


def _reading(read_ok=True, pid_enabled=True):
    return EnvironmentReading(
        t_utc="t", monotonic_s=0.0, read_ok=read_ok, partial=False,
        raw_registers=(), channels={SOURCE_CHANNEL: 24.5}, quality={},
        unknown_channels={}, pid_enabled=pid_enabled)


# ---------------------------------------------------------------------------
print("--- P1: a malformed register word FAILS the read, never coerced ---")
ok(_reg_read_ok([0] * N) is True,
   "a clean all-zero payload of the right length reads OK (control)")
ok(_reg_read_ok(["123"] + [0] * (N - 1)) is False,
   "a string word -> read_ok False (int('123') would have coerced it)")
ok(_reg_read_ok([123.9] + [0] * (N - 1)) is False,
   "a float word -> read_ok False (int(123.9) would have truncated it)")
ok(_reg_read_ok([True] + [0] * (N - 1)) is False,
   "a bool word -> read_ok False (bool is an int subclass; True would read as 1)")
ok(_reg_read_ok([-1] + [0] * (N - 1)) is False,
   "a negative word -> read_ok False (outside the 0..0xFFFF register range)")
ok(_reg_read_ok([0x10000] + [0] * (N - 1)) is False,
   "a word above 0xFFFF -> read_ok False (not a 16-bit register value)")

print("\n--- P2: the relay forwards only on read_ok / pid_enabled IS True ---")
ok(Relay._why_unusable(_reading(read_ok=True), 24.5) is None,
   "read_ok exactly True with a finite value is forwardable (control)")
ok(Relay._why_unusable(_reading(read_ok="true"), 24.5) is not None,
   "read_ok='true' (truthy string) is refused -- not exactly True")
ok(Relay._why_unusable(_reading(read_ok=1), 24.5) is not None,
   "read_ok=1 (truthy int) is refused -- not exactly True")

relay2, _, _ = build()
relay2.env.read_block = lambda: _reading(read_ok=True, pid_enabled=1)
rec = relay2.step()
ok(rec.forwarded is False,
   "pid_enabled=1 (truthy, not exactly True) is NOT forwarded -- treated as off")

print("\n--- P3: SetpointLimits bounds must be real finite numbers ---")
ok(isinstance(SetpointLimits(), SetpointLimits), "the default limits build (control)")
raises(SafetyError, lambda: SetpointLimits(temp_min_c=True),
       "SetpointLimits(temp_min_c=True) raises -- bool is not a real bound")
raises(SafetyError, lambda: SetpointLimits(rh_max_pct=True),
       "SetpointLimits(rh_max_pct=True) raises")

print("\n--- P4: RateLimiter fields, and the allow_large_step OVERRIDE gate ---")
raises(SafetyError, lambda: RateLimiter(max_step_c=True),
       "RateLimiter(max_step_c=True) raises -- bool is not a real cap")
rl = RateLimiter()
# The real gap: "false" is truthy, so the OLD gate bypassed the step cap.
raises(SafetyError,
       lambda: rl.check("temp_sp_c", 100.0, now=10.0, last_value=0.0,
                        last_time=None, allow_large_step="false"),
       "allow_large_step='false' RAISES rather than bypassing the step cap")
# The real override still works, and the cap still refuses without it.
ok(rl.check("temp_sp_c", 100.0, now=10.0, last_value=0.0, last_time=None,
            allow_large_step=True) is None,
   "allow_large_step=True (a real bool) still suppresses the cap")
raises(SafetyError,
       lambda: rl.check("temp_sp_c", 100.0, now=10.0, last_value=0.0,
                        last_time=None, allow_large_step=False),
       "a 100 C step with allow_large_step=False is still refused (cap 5 C)")

print("\n--- P5: CirculatorSettings command bounds must be real numbers ---")
ok(isinstance(CirculatorSettings(), CirculatorSettings),
   "default circulator settings build (control)")
raises(CirculatorError, lambda: CirculatorSettings(command_max_c=True),
       "CirculatorSettings(command_max_c=True) raises -- float(True) would coerce")
raises(CirculatorError, lambda: CirculatorSettings(command_min_c="5"),
       "CirculatorSettings(command_min_c='5') raises -- float('5') would coerce")

print("\n--- P6: rearm() requires a REAL bool actuation flag ---")
relay6, _, _ = build()
relay6.safe(reason="latch for P6")
ok(relay6.safe_mode is True
   and relay6._last_safe.quality.get("safe_confirmed") is True,
   "precondition: latched and CONFIRMED, so rearm reaches the actuation gate")


class _DuckSettings:
    allow_actuation = "false"


class _DuckCirc:
    settings = _DuckSettings()


relay6.circulator = _DuckCirc()
raises(ActuationNotAllowed, relay6.rearm,
       "rearm() refuses a duck-typed allow_actuation='false' (truthy) not authorizes")

print("\n--- P7: PLC protocol-address ranges ---")
ok(isinstance(PlcSettings(), PlcSettings), "default PLC settings build (control)")
raises(_config.ConfigError, lambda: PlcSettings(port=0),
       "port=0 raises -- below the 1..65535 TCP range")
raises(_config.ConfigError, lambda: PlcSettings(port=70000),
       "port=70000 raises -- above 65535")
raises(_config.ConfigError, lambda: PlcSettings(device_id=0),
       "device_id=0 raises -- below the Modbus 1..247 range")
raises(_config.ConfigError, lambda: PlcSettings(device_id=248),
       "device_id=248 raises -- above 247")

print("\n%s" % ("ALL PASS" if fails == 0 else "%d FAILURE(S)" % fails))
sys.exit(1 if fails else 0)
