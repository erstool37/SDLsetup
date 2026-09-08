#!/usr/bin/env python3
"""Codex re-verify B (H1-H8): residual fail-safe gaps closed after round-1 hardening.

Every assertion runs against ``FakePlc`` and ``FakeSerial``. No socket is
opened and no serial port is touched -- opening the circulator's port
hardware-resets its MCU, so no test in this repo may.

    python dev/tests/test_reverify_b.py
"""
from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from tools.circulator.api import CirculatorError  # noqa: E402
from tools.circulator.api import SafetyError as CircSafetyError  # noqa: E402
from tools.circulator.circulator import Circulator, CirculatorSettings  # noqa: E402
from tools.circulator.codec import SETPOINT_ADDRESS, encode_setpoint  # noqa: E402
from tools.circulator.link import (  # noqa: E402
    _FRAME_TOKEN,
    FakeSerial,
    SerialLink,
    WriteResult,
    _SetpointFrame,
)
from tools.circulator.safety import (  # noqa: E402
    BoundedSetpoint as CircBounded,
)
from tools.circulator.safety import CommandLimits  # noqa: E402
from tools.environment import registers  # noqa: E402
from tools.environment.plc import FakePlc, PlcClient, PlcSettings  # noqa: E402
from tools.environment.relay import Relay, RelayPolicy  # noqa: E402
from tools.environment.safety import (  # noqa: E402
    ActuationNotAllowed,
    CommsLost,
    FailSafeIncomplete,
    SetpointLimits,  # noqa: E402
)
from tools.environment.safety import BoundedSetpoint as PlcBounded  # noqa: E402

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


def last_frame_c(line: FakeSerial):
    if not line.frames:
        return None
    from tools.circulator.codec import decode_setpoint
    return decode_setpoint(line.frames[-1]["values"])


# ---------------------------------------------------------------------------
print("\n--- H1: a PERSISTENT unreadable PID coil fires the fail-safe ---")
clock = Clock()
relay, fake, line = build(clock=clock, policy={"plc_stale_s": 5.0},
                          plc={"error_on": {"read_coils"}})
first = relay.step()
ok(first.reading["pid_enabled"] is None, "the coil is unreadable -> pid_enabled is None")
ok(first.forwarded is False, "not forwarded on the first unknown-coil step")
ok(relay.safe_mode is False, "and NOT yet safe, within plc_stale_s")
ok(line.frames == [], "and nothing was written to the bath yet")
clock.advance(3.0)
relay.step()
ok(relay.safe_mode is False, "still within plc_stale_s at 3 s -> still not safe")
clock.advance(2.001)
exc = raises(CommsLost, relay.step,
             "beyond plc_stale_s a persistent None coil-read forces the fail-safe (H1)")
ok(relay.safe_mode is True, "the relay latched safe mode -- 'unknown' failed closed")
ok(last_frame_c(line) == SAFE_C, "the SAFE setpoint was written, not the last command",
   "%r" % last_frame_c(line))
ok(fake.coils.get(C1) is False, "and C1 was driven off")

print("\n--- H2: with the PID loop OFF the bath is driven to safe, DF9 not forwarded ---")
relay, fake, line = build(pid_on=False)
rec = relay.step()
ok(rec.forwarded is False, "C1 off -> DF9 is NOT forwarded")
ok(last_frame_c(line) == SAFE_C, "the bath is driven to the safe setpoint (H2)",
   "%r" % last_frame_c(line))
ok(len(line.frames) == 1, "exactly one frame -- the safe setpoint", str(len(line.frames)))
ok(relay.safe_mode is False, "this is NOT latched safe mode: a clean PID-off is recoverable")
relay.step()
ok(len(line.frames) == 1, "a second PID-off step does NOT re-drive: once per episode (H2)")
fake.coils[C1] = True
poke(fake, 9, 24.0)
back = relay.step()
ok(back.forwarded is True, "with C1 turned back on, forwarding resumes")

print("\n--- H3: _SetpointFrame is immutable and _check_frame enforces the shape ---")
frame = SerialLink.encode_frame(25.0)
ok(isinstance(frame.values, tuple), "encode_frame stores values as a TUPLE (H3)",
   type(frame.values).__name__)
ok(frame.address == SETPOINT_ADDRESS and frame.values == (0, 0, 0, 0x3940),
   "and the canonical 4-register frame at address 980", "%r" % (frame.values,))
raises(CircSafetyError, lambda: setattr(frame, "address", 12),
       "frame.address cannot be reassigned (H3)")
raises(CircSafetyError, lambda: setattr(frame, "values", (1, 2, 3, 4)),
       "frame.values cannot be reassigned (H3)")


def _live():
    return CirculatorSettings(port="/tmp/sdl-fake", allow_actuation=True,
                              boot_settle_s=0.0)


wrong_addr = _SetpointFrame(_FRAME_TOKEN, 12, (0, 0, 0, 0))
with SerialLink(_live(), transport=FakeSerial()) as ln:
    raises(CirculatorError, lambda: ln.write_registers(wrong_addr),
           "a frame at a non-980 address is refused at the sink (H3)")
one_reg = _SetpointFrame(_FRAME_TOKEN, SETPOINT_ADDRESS, (0x3940,))
with SerialLink(_live(), transport=FakeSerial()) as ln:
    raises(CirculatorError, lambda: ln.write_registers(one_reg),
           "a one-register frame is refused at the sink (H3)")
with SerialLink(_live(), transport=FakeSerial()) as ln:
    good = ln.write_registers(SerialLink.encode_frame(25.0))
ok(good.outcome == "confirmed", "the canonical frame still writes fine", good.outcome)

print("\n--- H4: sinks snapshot the token value once (stateful subclass) ---")


class _StatefulCirc(CircBounded):
    """A BoundedSetpoint whose value_c changes on each read (adversarial)."""

    def __init__(self, seq, limits):
        object.__setattr__(self, "_seq", list(seq))
        object.__setattr__(self, "_i", 0)
        object.__setattr__(self, "limits", limits)

    def __getattribute__(self, name):
        if name == "value_c":
            seq = object.__getattribute__(self, "_seq")
            i = object.__getattribute__(self, "_i")
            object.__setattr__(self, "_i", i + 1)
            return seq[min(i, len(seq) - 1)]
        return object.__getattribute__(self, name)


dev = Circulator(CirculatorSettings(port="/tmp/x", allow_actuation=False),
                 transport=FakeSerial())
res = dev.write_setpoint(_StatefulCirc([20.0, 25.0], dev.limits))
ok(res.values == encode_setpoint(20.0),
   "the CIRCULATOR sink encodes the FIRST read (snapshot), not a later value (H4)",
   "%r vs %r" % (res.values, encode_setpoint(20.0)))


class _StatefulPlc(PlcBounded):
    def __init__(self, seq, field, spec, limits):
        object.__setattr__(self, "_seq", list(seq))
        object.__setattr__(self, "_i", 0)
        object.__setattr__(self, "field", field)
        object.__setattr__(self, "spec", spec)
        object.__setattr__(self, "limits", limits)

    def __getattribute__(self, name):
        if name == "value":
            seq = object.__getattribute__(self, "_seq")
            i = object.__getattribute__(self, "_i")
            object.__setattr__(self, "_i", i + 1)
            return seq[min(i, len(seq) - 1)]
        return object.__getattribute__(self, name)


spec = registers.WRITABLE["temp_sp_c"]
plc_sp = _StatefulPlc([20.0, 25.0], "temp_sp_c", spec, SetpointLimits())
pclient = PlcClient(PlcSettings(allow_actuation=True), FakePlc({5: 20.0}))
pres = pclient.write_float32(plc_sp, dry_run=True)
ok(tuple(pres.values) == tuple(registers.f32_to_regs(20.0)),
   "the PLC sink encodes the FIRST read (snapshot), not a later value (H4)",
   "%r vs %r" % (pres.values, registers.f32_to_regs(20.0)))

print("\n--- H5: rearm() requires a CONFIRMED, latched fail-safe ---")
# Opened, actuation ON, but the bath WRITE fails -> unconfirmed while the
# actuation gate is open, so ONLY the new confirmed-gate can refuse the rearm.
relay, fake, line = build(serial={"error_response": True})
relay.safe(reason="latch, unconfirmed")
ok(relay.safe_mode is True, "the relay is latched")
ok(relay._last_safe.quality["safe_confirmed"] is False,
   "the safe was NOT confirmed (the bath write failed)")
raises(ActuationNotAllowed, relay.rearm,
       "rearm() after an UNCONFIRMED safe is refused, even with actuation ON (H5)")
ok(relay.safe_mode is True, "and safe mode stays latched after the refusal")

relay2, _f2, _l2 = build()
raises(ActuationNotAllowed, relay2.rearm,
       "rearm() when NOT latched is refused -- there is nothing to rearm (H5)")

relay3, fake3, line3 = build()
relay3.safe(reason="latch, confirmed")
ok(relay3._last_safe.quality["safe_confirmed"] is True, "this safe WAS confirmed")
event = relay3.rearm()
ok(relay3.safe_mode is False and event["rearm_count"] == 1,
   "rearm() after a CONFIRMED safe succeeds and records that it ran (H5)")

print("\n--- H6: safe() is totally non-raising, even when a result's as_dict() raises ---")


class _BoomResult:
    outcome = "confirmed"
    ok = True
    error = None

    def as_dict(self):
        raise RuntimeError("as_dict boom")


class _StubEnv:
    def set_pid(self, enabled, *, dry_run=False):
        return _BoomResult()


class _StubCirc:
    class settings:
        allow_actuation = True

    def bound(self, v):
        return CommandLimits().validate(v)

    def write_setpoint(self, sp):
        return WriteResult(outcome="confirmed", address=SETPOINT_ADDRESS,
                           values=[0, 0, 0, 0], echo_address=SETPOINT_ADDRESS,
                           echo_count=4)


boom_relay = Relay(_StubEnv(), _StubCirc(),
                   RelayPolicy(safe_setpoint_c=SAFE_C, command_limits=CommandLimits()))
raised = False
brec = None
try:
    brec = boom_relay.safe(reason="H6 injected as_dict failure")
except Exception as boom_exc:                                         # noqa: BLE001
    raised = True
    print("[test] FAIL: safe() raised %s: %s" % (type(boom_exc).__name__, boom_exc))
    fails += 1
ok(raised is False, "safe() does NOT raise when a result's as_dict() raises (H6)")
ok(brec is not None and brec.quality["safe_confirmed"] is False,
   "and records the fail-safe as NOT confirmed")
ok(brec is not None and brec.quality["pid_disable"]["outcome"] == "failed",
   "the PID-disable normalisation failure is recorded as failed, not propagated")

print("\n--- H7: a clean context exit with an unconfirmed fail-safe raises ---")
clean_relay, _cf, _cl = build(circ_actuation=False, opened=False)


def _clean_exit():
    with clean_relay:
        pass


raises(FailSafeIncomplete, _clean_exit,
       "a CLEAN exit whose fail-safe did not confirm raises FailSafeIncomplete (H7)")

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
   "an EXCEPTION exit preserves the original error, never masks it with "
   "FailSafeIncomplete (H7)", got)

print("\n--- H8: raw transports are off the public surface ---")
inj = FakePlc({5: 20.0})
pc = PlcClient(PlcSettings(), inj)
ok(not hasattr(pc, "transport"),
   "PlcClient.transport is gone from the public surface (H8)")
ok(pc._transport is inj, "the injected transport is stored privately as _transport")

sl = SerialLink(CirculatorSettings(port="/tmp/x", allow_actuation=True,
                                    boot_settle_s=0.0), transport=FakeSerial())
ok(not hasattr(sl, "transport"),
   "SerialLink.transport property is gone from the public surface (H8)")
ok(sl._transport is not None, "the raw client is reachable only via private _transport")

circd = Circulator(CirculatorSettings(port="/tmp/x", allow_actuation=False),
                   transport=FakeSerial())
ok(circd.link is not None and not hasattr(circd.link, "transport"),
   "Circulator.link stays (the node reads is_open) but its raw transport is private (H8)")

print("\n%s" % ("ALL PASS" if fails == 0 else "%d FAILURE(S)" % fails))
sys.exit(1 if fails else 0)
