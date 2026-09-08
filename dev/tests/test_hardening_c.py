#!/usr/bin/env python3
"""Codex re-verify C: close every fail-safe gap reachable by ORDINARY code.

Ordinary code = normal attribute access, plain construction, public method
calls, ``dataclasses.replace``. What remains after this pass is only the
genuinely-unpreventable class -- ``object.__setattr__``, private-name access,
``BaseException`` -- which is documented, not chased.

Every object here is built from numbers and fakes. No socket, no serial port
(opening the circulator's port hardware-resets its MCU, so no test may).

    python dev/tests/test_hardening_c.py
"""
from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from tools import config as _config  # noqa: E402
from tools.circulator.circulator import Circulator, CirculatorSettings  # noqa: E402
from tools.circulator.link import FakeSerial, SerialLink, WriteResult  # noqa: E402
from tools.circulator.safety import (  # noqa: E402
    CirculatorError,
    CommandLimits,
)
from tools.circulator.safety import SafetyError as CircSafetyError  # noqa: E402
from tools.environment import registers  # noqa: E402
from tools.environment.plc import FakePlc, PlcClient, PlcSettings  # noqa: E402
from tools.environment.relay import Relay, RelayPolicy  # noqa: E402
from tools.environment.safety import (  # noqa: E402
    CommsLost,
    FailSafeIncomplete,
)
from tools.environment.safety import SafetyError as PlcSafetyError  # noqa: E402

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


class Clock:
    def __init__(self, t: float = 1000.0) -> None:
        self.t = float(t)

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += float(dt)


GOOD_ROW = {5: 25.000, 6: 95.000, 9: 24.774, 10: 25.340, 18: 24.866, 19: 95.180}
SAFE_C = 15.0
C1 = registers.C1_COIL


def poke(fake: FakePlc, df: int, value: float) -> None:
    low, high = registers.f32_to_regs(value)
    address = registers.df_address(df)
    fake.registers[address] = low
    fake.registers[address + 1] = high


def build(*, clock=None, policy=None, plc=None, serial=None, pid_on=True,
          circ_actuation=True, opened=True):
    fake = FakePlc(dict(GOOD_ROW), coils={C1: bool(pid_on)}, **(plc or {}))
    client = PlcClient(PlcSettings(allow_actuation=True), fake)
    line = FakeSerial(**(serial or {}))
    device = Circulator(
        CirculatorSettings(port="/tmp/sdl-fake-circulator",
                           allow_actuation=circ_actuation, boot_settle_s=0.0),
        line)
    if opened and circ_actuation:
        device.open(settle=False)
    relay = Relay(client, device,
                  RelayPolicy(safe_setpoint_c=SAFE_C, command_limits=device.limits,
                              **(policy or {})),
                  clock=clock or Clock())
    return relay, fake, line


def circ_live(**kw) -> CirculatorSettings:
    base = {"port": "/tmp/sdl-fake-circulator", "allow_actuation": True,
            "boot_settle_s": 0.0}
    base.update(kw)
    return CirculatorSettings(**base)


def last_frame_c(line: FakeSerial):
    if not line.frames:
        return None
    from tools.circulator.codec import decode_setpoint
    return decode_setpoint(line.frames[-1]["values"])


# ---------------------------------------------------------------------------
print("\n--- G1: EPS no longer widens the command ceiling ---")
raises(CircSafetyError, lambda: CommandLimits(max_c=30.0000005),
       "CommandLimits(max_c=30.0000005) is refused -- a 5e-7 overshoot is not slop")
raises(CircSafetyError, lambda: CommandLimits(min_c=-0.0000005),
       "and a 5e-7 UNDERshoot of the floor is refused too")
ok(CommandLimits(min_c=0.0, max_c=30.0).max_c == 30.0,
   "a RANGE of exactly the ceiling is still legal (exact literals, no EPS needed)")
_resolved = CirculatorSettings.from_config({"circulator": {"command_min_c": 10.0}}).limits
ok((_resolved.min_c, _resolved.max_c) == (10.0, 30.0),
   "the resolved (10,30) run bound still constructs", "%r" % ((_resolved.min_c, _resolved.max_c),))
ok(_resolved.validate(20.0).value_c == 20.0, "and validates a value strictly inside it")
raises(CircSafetyError, lambda: _resolved.validate(30.00000025),
       "a tiny overshoot of the value ceiling is still refused by validate() (strict)")

print("\n--- G2: relay.safe_mode is a read-only property; assignment cannot clear the latch ---")
relay, _f, _l = build()
raises(AttributeError, lambda: setattr(relay, "safe_mode", False),
       "relay.safe_mode = False raises (no setter) -- cannot bypass rearm()")
raises(AttributeError, lambda: setattr(relay, "safe_mode", True),
       "and relay.safe_mode = True raises too")
relay.safe(reason="latch it")
ok(relay.safe_mode is True, "safe() still latches, via the backing field _safe_mode")
raises(AttributeError, lambda: setattr(relay, "safe_mode", False),
       "still cannot be cleared by ordinary assignment while latched")

print("\n--- G3: a truthy non-bool actuation gate is refused at construction ---")
raises(_config.ConfigError, lambda: PlcSettings(allow_actuation="false"),
       "PlcSettings(allow_actuation='false') raises -- a truthy string does not open the gate")
raises(_config.ConfigError, lambda: PlcSettings(dashboard_poll_plc="yes"),
       "PlcSettings(dashboard_poll_plc='yes') raises too")
raises(CirculatorError, lambda: CirculatorSettings(allow_actuation="false"),
       "CirculatorSettings(allow_actuation='false') raises (already held; re-asserted)")
ok(PlcSettings(allow_actuation=True).allow_actuation is True,
   "a real bool still constructs")

print("\n--- G4: NaN / wrong-type circulator settings are refused ---")
raises(CirculatorError, lambda: CirculatorSettings(boot_settle_s=float("nan")),
       "boot_settle_s=NaN is refused -- NaN<0 is False, so the old check let it through")
raises(CirculatorError, lambda: CirculatorSettings(timeout_s=float("nan")),
       "timeout_s=NaN is refused")
raises(CirculatorError, lambda: CirculatorSettings(timeout_s=float("inf")),
       "timeout_s=+inf is refused")
raises(CirculatorError, lambda: CirculatorSettings(stopbits=True),
       "stopbits=True is refused -- True==1 must not sneak past `in _STOPBITS`")
raises(CirculatorError, lambda: CirculatorSettings(boot_settle_s=True),
       "boot_settle_s=True is refused (a bool is not a duration)")
ok(CirculatorSettings(boot_settle_s=3.0, timeout_s=1.0, stopbits=1).stopbits == 1,
   "valid finite settings still construct")

print("\n--- G5: a PID-off safe write that keeps failing escalates past circ_stale_s ---")
clock = Clock()
relay, fake, line = build(clock=clock, pid_on=False, policy={"circ_stale_s": 4.0},
                          serial={"error_response": True})
first = relay.step()
ok(first.forwarded is False, "PID off: DF9 is not forwarded")
ok(first.write is not None and first.write.get("outcome") == "failed",
   "the safe-setpoint write to the bath FAILED", str(first.write and first.write.get("outcome")))
ok(relay.safe_mode is False, "one failed PID-off safe write is not yet staleness")
clock.advance(4.5)
exc = raises(CommsLost, relay.step,
             "a PERSISTENT PID-off safe-write failure escalates past circ_stale_s (G5)")
ok(relay.safe_mode is True, "and the relay latched safe mode instead of retrying forever")
ok(exc is not None and "circ_stale_s" in str(exc), "the error names circ_stale_s", str(exc)[:70])

print("\n--- G5b: a PID-off safe write that CONFIRMS still does not escalate ---")
relay, fake, line = build(pid_on=False, policy={"circ_stale_s": 4.0})
relay.step()
relay.step()
ok(relay.safe_mode is False, "a confirmed PID-off safe write never trips the watchdog")
ok(len(line.frames) == 1, "and it is written exactly once per episode")

print("\n--- G6: the RESOLVED command bound is enforced at the wire, not just at bound() ---")
with SerialLink(circ_live(), transport=FakeSerial()) as link:
    raises(CircSafetyError, lambda: link.write_registers(SerialLink.encode_frame(35.0)),
           "a 35 C frame is refused at write_registers -- past the code ceiling")
# The killer case: a value INSIDE the code ceiling (0,30) but OUTSIDE this
# device's tightened (10,30) bound. encode_frame builds a shape-valid frame; the
# sink re-decodes and re-validates against ITS OWN limits.
with SerialLink(circ_live(command_min_c=10.0), transport=FakeSerial()) as link:
    raises(CircSafetyError, lambda: link.write_registers(SerialLink.encode_frame(5.0)),
           "5 C is refused at the wire under a (10,30) device, though 5 is inside (0,30)")
    good = link.write_registers(SerialLink.encode_frame(20.0))
    ok(good.outcome == "confirmed", "20 C, inside the resolved bound, still writes")
dev = Circulator(circ_live(), transport=FakeSerial())
dev.open(settle=False)
res = dev.write_setpoint(dev.bound(20.0))
ok(res.outcome == "confirmed", "and the guarded Circulator.write_setpoint(bound(20)) still confirms")
dev.close()

print("\n--- G7: step() is self-safe against a bad clock and a raising read ---")
relay, fake, line = build(clock=lambda: float("nan"))
exc = raises(PlcSafetyError, relay.step, "a NaN clock inside step() raises")
ok(relay.safe_mode is True, "and the relay went SAFE (G7: self-safe, not reliant on the finally)")
ok(exc is not None and getattr(exc, "record", None) is not None,
   "the raised exception carries the fail-safe record")
ok(last_frame_c(line) == SAFE_C, "the safe setpoint was written", "%r" % last_frame_c(line))

relay, fake, line = build()


def _boom_read():
    raise RuntimeError("read_block exploded")


relay.env.read_block = _boom_read
exc = raises(RuntimeError, relay.step, "a read_block that raises propagates")
ok(relay.safe_mode is True, "but the relay went SAFE first (G7)")
ok(exc is not None and getattr(exc, "record", None) is not None,
   "and the exception carries the fail-safe record")
ok(fake.coils.get(C1) is False, "and C1 was driven off by the fail-safe")

print("\n--- G7: a malformed register payload is a failed read, never an exception ---")


class _BadResp:
    registers = ["x", "y", "z", "w"]

    def isError(self):  # noqa: N802 - vendor spelling
        return False


class _BadTransport:
    def connect(self):
        return True

    def read_holding_registers(self, address, *, count=1, device_id=1,
                               no_response_expected=False):
        return _BadResp()

    def read_coils(self, address, *, count=1, device_id=1,
                   no_response_expected=False):
        return _BadResp()

    def close(self):
        pass


try:
    reading = PlcClient(PlcSettings(), _BadTransport()).read_block()
    ok(reading.read_ok is False,
       "a non-integer register -> read_ok False (the int() conversion is inside the read guard)")
except Exception as exc:                                             # noqa: BLE001
    ok(False, "read_block must not raise on a malformed payload",
       "%s: %s" % (type(exc).__name__, exc))

print("\n--- H6: safe() never raises for an ordinary Exception; BaseException is documented ---")


class _BoomAsDict:
    outcome = "confirmed"
    ok = True
    error = None

    def as_dict(self):
        raise RuntimeError("as_dict boom")


class _CircBoom:
    class settings:
        allow_actuation = True

    def bound(self, v):
        return CommandLimits().validate(v)

    def write_setpoint(self, sp):
        return _BoomAsDict()


class _EnvStub:
    def set_pid(self, enabled, *, dry_run=False):
        return WriteResult(outcome="confirmed", address=980, values=[0, 0, 0, 0],
                           echo_address=980, echo_count=4)


boom = Relay(_EnvStub(), _CircBoom(),
             RelayPolicy(safe_setpoint_c=SAFE_C, command_limits=CommandLimits()))
try:
    brec = boom.safe(reason="H6: circulator write result as_dict() raises")
    ok(brec.quality.get("safe_confirmed") is False,
       "safe() returns a NOT-confirmed record when the write result's as_dict() raises")
    ok(boom.safe_mode is True, "and still latches safe mode via _safe_mode")
except Exception as exc:                                             # noqa: BLE001
    ok(False, "safe() must NOT raise when a result's as_dict() raises",
       "%s: %s" % (type(exc).__name__, exc))
ok(isinstance(Relay.safe.__doc__, str) and "BaseException" in Relay.safe.__doc__,
   "safe() documents that BaseException is intentionally NOT caught")

print("\n--- H7: a clean context exit with an unconfirmed fail-safe raises (depends on H6) ---")
clean, _cf, _cl = build(circ_actuation=False, opened=False)


def _clean_exit():
    with clean:
        pass


raises(FailSafeIncomplete, _clean_exit,
       "a CLEAN exit whose exit fail-safe did not confirm raises FailSafeIncomplete (H7)")

exc_relay, _ef, _el = build(circ_actuation=False, opened=False)
got = "none"
try:
    with exc_relay:
        raise RuntimeError("phase script blew up")
except FailSafeIncomplete:
    got = "failsafe"
except RuntimeError:
    got = "runtime"
ok(got == "runtime",
   "an EXCEPTION exit preserves the original error, never masks it with FailSafeIncomplete (H7)",
   got)

print("\n%s" % ("ALL PASS" if fails == 0 else "%d FAILURE(S)" % fails))
sys.exit(1 if fails else 0)
