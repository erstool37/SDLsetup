#!/usr/bin/env python3
"""Hardening D: close the "coerce instead of validate" class across the
environment and circulator fail-safe layers (S1-S7).

Every prior round kept finding the same root defect in new methods: a
wrong-typed value COERCED (``bool(x)``, truthiness, ``int(float)``) instead of
VALIDATED. This suite pins each named instance and the sibling behaviour, so the
pattern cannot come back unnoticed.

No hardware, no socket, no serial port (opening the circulator's port
hardware-resets its MCU, so no test may). Everything is built from numbers and
in-memory fakes.

    python dev/tests/test_hardening_d.py
"""
from __future__ import annotations

import datetime
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

import tools.environment.relay as relay_module  # noqa: E402
from tools import config as _config  # noqa: E402
from tools.circulator.circulator import Circulator, CirculatorSettings  # noqa: E402
from tools.circulator.link import FakeSerial  # noqa: E402
from tools.environment import registers  # noqa: E402
from tools.environment.plc import FakePlc, PlcClient, PlcSettings  # noqa: E402
from tools.environment.relay import Relay, RelayPolicy, RelayRecord  # noqa: E402
from tools.environment.safety import SafetyError as PlcSafetyError  # noqa: E402

fails = 0


def ok(condition: bool, message: str, detail: str = "") -> None:
    global fails
    print("[test] %s: %s%s" % ("PASS" if condition else "FAIL", message,
                               "  (%s)" % detail if detail else ""))
    if not condition:
        fails += 1


def raises(exc, fn, message: str) -> None:
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


class Clock:
    def __init__(self, t: float = 1000.0) -> None:
        self.t = float(t)

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += float(dt)


def build(*, pid_on=True, circ_actuation=True, opened=True):
    fake = FakePlc(dict(GOOD_ROW), coils={C1: bool(pid_on)})
    client = PlcClient(PlcSettings(allow_actuation=True), fake)
    line = FakeSerial()
    device = Circulator(
        CirculatorSettings(port="/tmp/sdl-fake-circulator",
                           allow_actuation=circ_actuation, boot_settle_s=0.0),
        line)
    if opened and circ_actuation:
        device.open(settle=False)
    relay = Relay(client, device,
                  RelayPolicy(safe_setpoint_c=SAFE_C, command_limits=device.limits),
                  clock=Clock())
    return relay, fake, line


class _CoilFake:
    """Enough of a transport for read_pid_enabled(): returns ONE coil bit whose
    type the test controls, so a malformed non-bool bit can be injected."""

    def __init__(self, bit) -> None:
        self._bit = bit
        self.connected = False

    def connect(self) -> bool:
        self.connected = True
        return True

    def read_coils(self, address, *, count=1, device_id=1,
                   no_response_expected=False):
        bit = self._bit
        return type("R", (), {"bits": [bit], "isError": lambda s: False})()


class _RegFake:
    """A transport whose holding-register read returns a payload the test
    controls -- so a non-integer/inf word can reach _read_registers' int()."""

    def __init__(self, words) -> None:
        self._words = list(words)
        self.connected = False

    def connect(self) -> bool:
        self.connected = True
        return True

    def read_holding_registers(self, address, *, count=1, device_id=1,
                               no_response_expected=False):
        return type("R", (), {"registers": list(self._words),
                              "isError": lambda s: False})()

    def read_coils(self, address, *, count=1, device_id=1,
                   no_response_expected=False):
        return type("R", (), {"bits": [False], "isError": lambda s: False})()


# ---------------------------------------------------------------------------
print("--- S1: PlcClient.set_pid() validates `enabled`, never coerces it ---")
safe = PlcClient(PlcSettings(allow_actuation=False), FakePlc(coils={C1: False}))
raises(PlcSafetyError, lambda: safe.set_pid("false", dry_run=True),
       "set_pid('false') is REFUSED -- bool('false') is True and would ENABLE C1")
raises(PlcSafetyError, lambda: safe.set_pid(1, dry_run=True),
       "set_pid(1) is refused: an int is not a bool")
raises(PlcSafetyError, lambda: safe.set_pid(0, dry_run=True),
       "set_pid(0) is refused too (0 is falsy but not a bool)")
raises(PlcSafetyError, lambda: safe.set_pid(True, dry_run="yes"),
       "set_pid(dry_run='yes') is refused: dry_run must be a real bool")
ok(safe.set_pid(False, dry_run=True).value is False,
   "a genuine set_pid(False) still plans, value=False")
ok(safe.set_pid(True, dry_run=True).value is True,
   "a genuine set_pid(True) still plans, value=True")

print("\n--- S2: every numeric PlcSettings field is required finite ---")
raises(_config.ConfigError, lambda: PlcSettings(timeout_s=float("nan")),
       "timeout_s=NaN refused (NaN<=0 is False, so a bare bound missed it)")
raises(_config.ConfigError, lambda: PlcSettings(timeout_s=float("inf")),
       "timeout_s=inf refused")
raises(_config.ConfigError, lambda: PlcSettings(relay_period_s=float("nan")),
       "relay_period_s=NaN refused")
raises(_config.ConfigError, lambda: PlcSettings(plc_stale_s=float("inf")),
       "plc_stale_s=inf refused")
raises(_config.ConfigError, lambda: PlcSettings(retries=float("nan")),
       "retries=NaN refused (a non-int retries had no isinstance guard)")
ok(PlcSettings(timeout_s=2.0, relay_period_s=1.0, plc_stale_s=5.0).timeout_s == 2.0,
   "finite defaults still construct")

print("\n--- S3: read_pid_enabled() requires a GENUINE bool coil bit ---")
ok(PlcClient(PlcSettings(), _CoilFake(True)).read_pid_enabled() is True,
   "a genuine True reads True")
ok(PlcClient(PlcSettings(), _CoilFake(False)).read_pid_enabled() is False,
   "a genuine False reads False")
ok(PlcClient(PlcSettings(), _CoilFake(1)).read_pid_enabled() is True,
   "the int 1 reads True")
ok(PlcClient(PlcSettings(), _CoilFake(0)).read_pid_enabled() is False,
   "the int 0 reads False")
ok(PlcClient(PlcSettings(), _CoilFake("YES")).read_pid_enabled() is None,
   "a malformed truthy string reads None (unknown), NOT coerced to True (fails OPEN)")
ok(PlcClient(PlcSettings(), _CoilFake(2)).read_pid_enabled() is None,
   "a stray int 2 reads None, not True")

print("\n--- S5: a malformed register payload FAILS the read closed, never escapes ---")
_rf = PlcClient(PlcSettings(), _RegFake([float("inf")] * 4))
got = "escaped"
try:
    got = _rf._read_registers(registers.DF_BASE, 4)
except OverflowError:
    got = "overflow-escaped"
ok(got is None,
   "int(inf) -> None (failed read), never an uncaught OverflowError past the guard",
   repr(got))
block = PlcClient(
    PlcSettings(), _RegFake([float("inf")] * registers.BLOCK_COUNT)).read_block()
ok(block.read_ok is False,
   "read_block() with an inf word returns read_ok=False, not a raise")

print("\n--- S4: a FOREIGN .record on an exception does NOT skip safe() ---")
relay, fake, line = build()


def _boom_read():
    exc = RuntimeError("injected upstream failure")
    exc.record = {"foreign": True}  # an unrelated .record attribute
    raise exc


relay.env.read_block = _boom_read
caught = None
try:
    relay.step()
except RuntimeError as e:
    caught = e
ok(caught is not None, "the foreign exception still propagates out of step()")
ok(relay.safe_mode is True,
   "safe() RAN and latched -- a foreign .record no longer short-circuits the abort")
ok(isinstance(getattr(caught, "record", None), RelayRecord),
   "step() replaced the foreign .record with OUR RelayRecord")
ok(getattr(caught, "_relay_safe_ran", False) is True,
   "and stamped the private safe-ran marker")
ok(fake.coils.get(C1) is False, "the PID was actually disabled by the abort")

print("\n--- S6: a failing timestamp must NOT skip the safe write + PID disable ---")


class _BoomDatetime:
    timezone = datetime.timezone

    class datetime:  # noqa: N801 - mirrors the stdlib attribute path
        @staticmethod
        def now(tz=None):
            raise RuntimeError("clock exploded")


relay2, fake2, line2 = build()
_real_dt = relay_module.datetime
rec = None
try:
    relay_module.datetime = _BoomDatetime
    rec = relay2.safe(reason="S6 timestamp explodes")
finally:
    relay_module.datetime = _real_dt
ok(rec is not None, "safe() returned a record and did not raise despite the clock")
ok(relay2.safe_mode is True, "safe() still latched")
ok(len(line2.frames) >= 1,
   "the safe bath write was STILL attempted (a timestamp failure did not skip it)")
ok(fake2.coils.get(C1) is False,
   "and the PID disable STILL ran")

print("\n--- S7: rearm() obtains its timestamp BEFORE clearing the latch ---")
relay3, fake3, line3 = build()
relay3.safe(reason="latch for the rearm test")
ok(relay3.safe_mode is True
   and relay3._last_safe.quality.get("safe_confirmed") is True,
   "precondition: latched and CONFIRMED so rearm can proceed")
_real_dt = relay_module.datetime
did_raise = False
try:
    relay_module.datetime = _BoomDatetime
    relay3.rearm()
except Exception:  # noqa: BLE001
    did_raise = True
finally:
    relay_module.datetime = _real_dt
ok(did_raise is True, "rearm() raised because its timestamp failed")
ok(relay3.safe_mode is True,
   "the latch is INTACT (fail closed) -- forwarding was NOT re-enabled before the raise")

print("\n%s" % ("ALL PASS" if fails == 0 else "%d FAILURE(S)" % fails))
sys.exit(1 if fails else 0)
