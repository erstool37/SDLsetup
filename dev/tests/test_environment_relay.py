#!/usr/bin/env python3
"""The PLC -> circulator relay: what it forwards, what it refuses, and when it goes safe.

Every assertion here runs against ``FakePlc`` and ``FakeSerial``. No socket is
opened, no serial port is touched -- which matters more than usual on this rig,
because opening the circulator's port hardware-resets its microcontroller.

The test this file exists for
============================

``EnvironmentReading.read_ok`` is **not** sufficient to gate a forward on.
``tools.environment.reading.decode_block`` sets ``read_ok=True`` for a *short*
block: some pairs arrived, so those decoded. A reading missing the exact channel
the relay forwards therefore still reports ``read_ok=True``.

Two cases below pin that down, and both are written so they FAIL if the guard is
ever "simplified" to a bare ``if reading.read_ok``:

* ``short_registers=18`` -- DF1..DF9 arrive. ``read_ok=True``, ``partial=True``,
  and ``temp_pid_output_c`` **is present**. So neither ``read_ok`` alone nor
  channel-presence alone is the guard; ``partial`` is what refuses this one.
* ``short_registers=10`` -- DF1..DF5 arrive. ``read_ok=True`` and the channel is
  **absent**, so a guard that checked only ``partial`` on a complete-looking
  block would still have to look the channel up.

    python dev/tests/test_environment_relay.py
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from tools import config as _config  # noqa: E402
from tools.circulator.api import (  # noqa: E402
    Circulator,
    CirculatorError,  # noqa: E402
    CirculatorSettings,
    FakeSerial,
    decode_setpoint,
)
from tools.circulator.api import SafetyError as CircSafetyError  # noqa: E402
from tools.environment import registers  # noqa: E402
from tools.environment.plc import FakePlc, PlcClient, PlcSettings  # noqa: E402
from tools.environment.relay import (  # noqa: E402
    COOPERATIVE_WATCHDOG_RESIDUAL,
    Relay,
    RelayPolicy,
)
from tools.environment.safety import (  # noqa: E402
    ActuationNotAllowed,
    CommsLost,
    FailSafeIncomplete,
    SafetyError,
)

fails = 0


def ok(condition: bool, message: str, detail: str = "") -> None:
    global fails
    print("[test] %s: %s%s" % ("PASS" if condition else "FAIL", message,
                               "  (%s)" % detail if detail else ""))
    if not condition:
        fails += 1


def raises(kind, call, message: str):
    """Assert ``call()`` raises ``kind``; return the exception (or None)."""
    try:
        call()
    except kind as exc:
        ok(True, message, "%s: %s" % (type(exc).__name__, str(exc)[:70]))
        return exc
    except Exception as exc:                                          # noqa: BLE001
        ok(False, message, "raised %s instead: %s" % (type(exc).__name__, exc))
        return None
    ok(False, message, "did not raise")
    return None


class Clock:
    """An injected monotonic clock. The watchdog is asserted, never slept through."""

    def __init__(self, t: float = 1000.0) -> None:
        self.t = float(t)

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += float(dt)


#: One real logged row (2025-07-26 15:52:32), by DF number, so the expected
#: values are measurements rather than round numbers chosen to be easy.
GOOD_ROW = {5: 25.000, 6: 95.000, 9: 24.774, 10: 25.340, 18: 24.866, 19: 95.180}

#: 24.774 is not representable in float32; this is what the wire actually holds.
DF9_EXACT = registers.regs_to_f32(*registers.f32_to_regs(24.774))

SAFE_C = 15.0
DF9 = registers.df_address(9)


def poke(fake: FakePlc, df: int, value: float) -> None:
    """Seed one DF through the real codec, after construction."""
    low, high = registers.f32_to_regs(value)
    address = registers.df_address(df)
    fake.registers[address] = low
    fake.registers[address + 1] = high


def build(values: dict | None = None, *, clock: Clock | None = None,
          policy: dict | None = None, plc: dict | None = None,
          serial: dict | None = None, pid_on: bool = True):
    """A wired relay over two fakes. Returns ``(relay, fake_plc, fake_serial)``."""
    fake = FakePlc(dict(GOOD_ROW if values is None else values),
                   coils={registers.C1_COIL: bool(pid_on)}, **(plc or {}))
    client = PlcClient(PlcSettings(allow_actuation=True), fake)

    line = FakeSerial(**(serial or {}))
    device = Circulator(
        CirculatorSettings(port="/tmp/sdl-fake-circulator", allow_actuation=True,
                           boot_settle_s=0.0),
        line)
    device.open(settle=False)

    return (Relay(client, device,
                  RelayPolicy(safe_setpoint_c=SAFE_C, command_limits=device.limits,
                              **(policy or {})),
                  clock=clock or Clock()),
            fake, line)


def last_frame_c(line: FakeSerial) -> float | None:
    """The Celsius value of the last frame that actually left the host."""
    if not line.frames:
        return None
    return decode_setpoint(line.frames[-1]["values"])


# ---------------------------------------------------------------------------
print("\n--- a normal step forwards DF9 verbatim ---")
relay, fake, line = build()
record = relay.step()
ok(record.forwarded is True, "a complete, finite reading is forwarded")
ok(record.source_channel == "temp_pid_output_c", "from the PID-output channel")
ok(record.value_c == DF9_EXACT, "the forwarded value is BIT-IDENTICAL to DF9",
   "%.17g vs %.17g" % (record.value_c, DF9_EXACT))
ok(record.reading["channels"]["temp_pid_output_c"] == record.value_c,
   "and identical to what the reading itself carried -- nothing was rounded")
ok(last_frame_c(line) == DF9_EXACT,
   "and the frame on the wire decodes back to exactly that value",
   "%.17g" % (last_frame_c(line) or float("nan")))
ok(record.write is not None and record.write["outcome"] == "confirmed",
   "the write is confirmed against the FC16 echo",
   str(record.write and record.write.get("error")))
ok(record.safe_mode is False and record.reason is None, "not safe mode, no refusal reason")
ok(len(line.frames) == 1, "exactly one frame was sent", str(len(line.frames)))

print("\n--- THE CROSS-UNIT HAZARD: read_ok=True on a short block ---")
relay, fake, line = build(plc={"short_registers": 18})
record = relay.step()
ok(record.reading["read_ok"] is True,
   "the short block STILL reports read_ok=True -- this is the trap")
ok(record.reading["partial"] is True, "and partial=True is the flag that catches it")
ok("temp_pid_output_c" in record.reading["channels"],
   "the channel IS present here, so presence alone is not the guard either")
ok(record.forwarded is False,
   "NOT forwarded. A bare `if reading.read_ok` guard would have forwarded it")
ok(record.value_c is None, "and no value is recorded as forwarded")
ok(record.reason is not None and "partial" in record.reason.lower(),
   "the reason names the partial block", str(record.reason))
ok(line.frames == [], "nothing reached the circulator", str(line.frames))

print("\n--- and a short block that drops the channel entirely ---")
relay, fake, line = build(plc={"short_registers": 10})
record = relay.step()
ok(record.reading["read_ok"] is True, "read_ok is True again")
ok("temp_pid_output_c" not in record.reading["channels"], "but DF9 never arrived")
ok(record.forwarded is False, "NOT forwarded")
ok(line.frames == [], "nothing reached the circulator")

print("\n--- a non-finite DF9 is not forwarded ---")
relay, fake, line = build()
# Poked as raw words: f32_to_regs REFUSES to encode a NaN, so the bank cannot
# be seeded with one through the codec. 0x7FC00000 is a float32 quiet NaN.
fake.registers[DF9] = 0x0000
fake.registers[DF9 + 1] = 0x7FC0
record = relay.step()
ok(record.reading["partial"] is False, "the block is complete")
ok(math.isnan(record.reading["channels"]["temp_pid_output_c"]),
   "and DF9 decodes to NaN, unsubstituted")
ok(record.forwarded is False, "NOT forwarded")
ok(record.reason is not None and "finite" in record.reason.lower(),
   "the reason names finiteness", str(record.reason))
ok(line.frames == [], "nothing reached the circulator -- NaN encodes without error")

print("\n--- an out-of-bound DF9 raises out of the circulator guard, and goes safe ---")
for bad, why in ((35.0, "above the bound"),
                 (30.0, "exactly the upper endpoint (the bound is STRICT)"),
                 (0.0, "exactly the lower endpoint (also STRICT)")):
    relay, fake, line = build()
    poke(fake, 9, bad)
    exc = raises(CircSafetyError, relay.step, "DF9=%g raises: %s" % (bad, why))
    ok(relay.safe_mode is True, "  and the relay is in safe mode")
    ok(last_frame_c(line) == SAFE_C, "  the safe setpoint %g C was written" % SAFE_C,
       "%r" % last_frame_c(line))
    ok(fake.coils.get(registers.C1_COIL) is False, "  and the PID coil C1 is off")
    ok(getattr(exc, "record", None) is not None,
       "  the exception carries the safe-mode record")

print("\n--- a transient PLC fault SHORTER than plc_stale_s does not trip safe mode ---")
clock = Clock()
relay, fake, line = build(clock=clock, policy={"plc_stale_s": 5.0})
fake.error_on = frozenset({"read_holding_registers"})
first = relay.step()
ok(first.forwarded is False and first.reading["read_ok"] is False,
   "the failed read is reported, not raised")
clock.advance(3.0)
second = relay.step()
ok(second.forwarded is False, "still bad 3 s later")
ok(relay.safe_mode is False, "but 3 s < 5 s, so NO safe mode")
ok(line.frames == [], "and nothing was written to the bath")
fake.error_on = frozenset()
clock.advance(1.0)
third = relay.step()
ok(third.forwarded is True, "the recovered read forwards again")
ok(relay.safe_mode is False, "and the relay never went safe")

print("\n--- a PLC fault BEYOND plc_stale_s: safe setpoint, PID off, CommsLost ---")
clock = Clock()
relay, fake, line = build(clock=clock, policy={"plc_stale_s": 5.0},
                          plc={"error_on": {"read_holding_registers"}})
relay.step()
clock.advance(5.0)
ok(relay.step() is not None and relay.safe_mode is False,
   "at exactly plc_stale_s the watchdog has not fired yet (strictly greater)")
clock.advance(0.001)
exc = raises(CommsLost, relay.step, "beyond plc_stale_s it raises CommsLost")
ok(relay.safe_mode is True, "the relay is in safe mode")
ok(last_frame_c(line) == SAFE_C, "the safe setpoint was written -- NOT the last value",
   "%r" % last_frame_c(line))
ok(fake.coils.get(registers.C1_COIL) is False, "and the PID coil C1 is off")
ok(len(line.frames) == 1 and last_frame_c(line) == SAFE_C,
   "exactly ONE frame was ever sent, and it is the safe setpoint -- no stale "
   "value was re-sent at any point", "%d frame(s)" % len(line.frames))

print("\n--- a circulator write failure beyond circ_stale_s: PID off, CommsLost ---")
clock = Clock()
relay, fake, line = build(clock=clock, policy={"circ_stale_s": 4.0},
                          serial={"error_response": True})
first = relay.step()
ok(first.write is not None and first.write["outcome"] == "failed",
   "the echo is judged and the write reports failed", str(first.write))
ok(first.forwarded is True,
   "the value WAS handed to the circulator -- the outcome says it did not land")
ok(relay.safe_mode is False, "one failure is not yet staleness")
clock.advance(4.5)
exc = raises(CommsLost, relay.step, "beyond circ_stale_s it raises CommsLost")
ok(fake.coils.get(registers.C1_COIL) is False, "the PID coil C1 is off")
ok(exc is not None and "circ_stale_s" in str(exc), "and the error names the key")

print("\n--- __exit__ goes safe EVEN WHILE an exception propagates ---")
relay, fake, line = build()
try:
    with relay as active:
        ok(active is relay, "the context manager yields the relay")
        raise RuntimeError("a phase script blew up mid-loop")
except RuntimeError as exc:
    ok(str(exc) == "a phase script blew up mid-loop",
       "the original exception is NOT swallowed")
else:
    ok(False, "the original exception is NOT swallowed", "nothing propagated")
ok(last_frame_c(line) == SAFE_C, "the safe setpoint was written on the way out",
   "%r" % last_frame_c(line))
ok(fake.coils.get(registers.C1_COIL) is False,
   "and C1 was written False -- the one real interlock the prior system had")

print("\n--- a large step is FLAGGED and still forwarded UNSHAPED ---")
relay, fake, line = build(policy={"large_step_c": 5.0})
poke(fake, 9, 10.0)
small = relay.step()
ok(small.quality["large_step"] is False, "the first step has no predecessor to jump from")
poke(fake, 9, 20.0)
big = relay.step()
ok(big.quality["large_step"] is True, "a 10 C jump is flagged")
ok(big.quality["step_c"] == 10.0, "and the step size is recorded", "%r" % big.quality["step_c"])
ok(big.forwarded is True, "and it is STILL forwarded")
ok(big.value_c == 20.0, "unshaped: the forwarded value equals the input exactly",
   "%.17g" % big.value_c)
ok(last_frame_c(line) == 20.0, "and so does the frame on the wire",
   "%.17g" % (last_frame_c(line) or float("nan")))

print("\n--- RelayPolicy refuses to default the safe setpoint ---")
raises(TypeError, RelayPolicy, "RelayPolicy() with no safe_setpoint_c is a TypeError")
exc = raises(_config.ConfigError, lambda: RelayPolicy(safe_setpoint_c={}),
             "safe_setpoint_c={} (the empty-key parse) raises")
ok(exc is not None and "circulator.safe_setpoint_c" in str(exc),
   "and the message names the config key", str(exc)[:90])
ok(exc is not None and "configs/config.yaml" in str(exc),
   "and the file to edit")
exc = raises(_config.ConfigError,
             lambda: RelayPolicy.from_config({"circulator": {"safe_setpoint_c": {}}}),
             "from_config with an empty key raises")
ok(exc is not None and "circulator.safe_setpoint_c" in str(exc), "naming the key")
exc = raises(_config.ConfigError, lambda: RelayPolicy.from_config({"circulator": {}}),
             "from_config with the key absent raises too")
ok(exc is not None and "circulator.safe_setpoint_c" in str(exc), "naming the key")
raises(_config.ConfigError,
       lambda: RelayPolicy.from_config({"circulator": {"safe_setpoint_c": 30.0}}),
       "a safe setpoint on the strict endpoint is refused at construction")
raises(_config.ConfigError,
       lambda: RelayPolicy.from_config({"circulator": {"safe_setpoint_c": 99.0}}),
       "and one outside the command bound is refused too")
good = RelayPolicy.from_config({
    "circulator": {"safe_setpoint_c": 14.5, "circ_stale_s": 4.0},
    "environment": {"plc_stale_s": 6.0, "relay_period_s": 1.0},
})
ok(good.safe_setpoint_c == 14.5, "a declared safe setpoint resolves", "%r" % good.safe_setpoint_c)
ok(good.circ_stale_s == 4.0, "so does circ_stale_s", "%r" % good.circ_stale_s)
ok(good.plc_stale_s == 6.0, "and plc_stale_s comes from the environment section",
   "%r" % good.plc_stale_s)
ok("configs/config.yaml" in good.describe() or "config.yaml" in good.describe(),
   "describe() names where the values came from")
residual = Relay.HONEST_RESIDUAL
ok(isinstance(residual, str) and "cannot be delivered" in residual.lower()
   and "serial" in residual.lower(),
   "the honest residual is a string constant for run.note(), and it says the "
   "safe setpoint cannot be delivered over a dead serial link")

print("\n--- RelayRecord.as_dict() is JSON-serialisable ---")
relay, fake, line = build()
record = relay.step()
payload = record.as_dict()
try:
    text = json.dumps(payload, allow_nan=False)
    back = json.loads(text)
    ok(back["value_c"] == DF9_EXACT, "json round-trip preserves the forwarded value")
    ok(back["source_channel"] == "temp_pid_output_c", "and the source channel")
    ok(back["write"]["outcome"] == "confirmed", "and the write outcome")
    ok(back["quality"]["large_step"] is False, "and the quality flags")
except (TypeError, ValueError) as exc:
    ok(False, "json.dumps(record.as_dict()) round-trips", str(exc))

print("\n--- F9: a non-finite or backward clock cannot silently disable a watchdog ---")
relay, _f9a, _l9a = build(clock=lambda: float("nan"))
raises(SafetyError, relay.step, "a NaN clock is refused, not silently ignored")

_seq = iter([1000.0, 990.0, 990.0])
relay, _f9b, _l9b = build(clock=lambda: next(_seq))
relay.step()  # t=1000, good read, forwards normally
raises(SafetyError, relay.step, "a backward clock jump is refused")

def unopened_relay(*, circ_actuation: bool, pid_on: bool = True,
                   serial: dict | None = None, policy: dict | None = None):
    """A relay whose circulator is NOT opened, for the exception/gate paths."""
    fk = FakePlc(dict(GOOD_ROW), coils={registers.C1_COIL: bool(pid_on)})
    cl = PlcClient(PlcSettings(allow_actuation=True), fk)
    ln = FakeSerial(**(serial or {}))
    dv = Circulator(
        CirculatorSettings(port="/tmp/sdl-fake-circulator",
                           allow_actuation=circ_actuation, boot_settle_s=0.0), ln)
    rl = Relay(cl, dv, RelayPolicy(safe_setpoint_c=SAFE_C, command_limits=dv.limits,
                                   **(policy or {})))
    return rl, fk, ln, dv


print("\n--- H2: with the PID loop OFF, the bath is DRIVEN TO SAFE (not forwarded) ---")
relay, fake, line = build(pid_on=False)
rec = relay.step()
ok(rec.forwarded is False, "C1 off -> DF9 is NOT forwarded (a disabled loop's output is not a command)")
ok(rec.reason is not None and "OFF" in rec.reason.upper() and "C1" in rec.reason,
   "the reason names the PID coil being off", str(rec.reason)[:70])
ok(last_frame_c(line) == SAFE_C,
   "the bath is driven to the declared safe setpoint, not left on its last command (H2)",
   "%r" % last_frame_c(line))
ok(len(line.frames) == 1, "exactly one frame -- the safe setpoint", str(len(line.frames)))
ok(relay.safe_mode is False, "this is NOT latched safe mode -- a clean PID-off is recoverable")
rec2 = relay.step()
ok(len(line.frames) == 1, "a second PID-off step does NOT re-drive: once per episode (H2)")
ok(rec2.forwarded is False, "still not forwarding while C1 is off")

print("\n--- F1: with the PID coil UNREADABLE (None), DF9 is not forwarded ---")
relay, fake, line = build(plc={"error_on": {"read_coils"}})
rec = relay.step()
ok(rec.reading["pid_enabled"] is None, "C1 could not be read -> pid_enabled is None")
ok(rec.forwarded is False, "an unknown loop state is NOT forwarded ('unknown' is not 'closed')")
ok(line.frames == [], "and nothing reached the bath")

print("\n--- F1: safe mode is LATCHED; step() keeps refusing until rearm() ---")
relay, fake, line = build()
relay.safe(reason="latch it for the test")
ok(relay.safe_mode is True, "safe() latched safe mode")
frames_before = len(line.frames)
poke(fake, 9, 24.0)                       # a perfectly good, in-bound reading
rec = relay.step()
ok(rec.forwarded is False and "LATCH" in (rec.reason or "").upper(),
   "a subsequent step REFUSES while latched -- even with a good reading", str(rec.reason)[:60])
ok(len(line.frames) == frames_before, "and forwards nothing new: a recovered PLC does not silently resume")
event = relay.rearm()
ok(relay.safe_mode is False, "rearm() clears the latch")
ok(event["action"] == "rearm" and event["rearm_count"] == 1 and event["was_safe_mode"] is True,
   "and records that it ran", str(event)[:70])
# rearm() deliberately does NOT re-enable the PID (safe() disabled C1); a real
# resume also needs the operator to re-enable the loop, so simulate that here.
rec = relay.step()
ok(rec.forwarded is False and rec.reading["pid_enabled"] is False,
   "even after rearm, a step with C1 still OFF does not forward -- rearm re-enables no PID")
fake.coils[registers.C1_COIL] = True      # operator re-enables the loop deliberately
rec = relay.step()
ok(rec.forwarded is True, "with the latch cleared AND the PID re-enabled, forwarding resumes")

print("\n--- F1: rearm() requires circulator actuation to be allowed ---")
relay, fake, line, dev = unopened_relay(circ_actuation=False)
relay.safe(reason="latch it")
ok(relay.safe_mode is True, "the relay is latched safe")
raises(ActuationNotAllowed, relay.rearm,
       "rearm() is refused while circulator.allow_actuation is off")
ok(relay.safe_mode is True, "and safe mode stays latched after the refusal")

print("\n--- F3: a dry-run PLANNED write is not a healthy delivery ---")
relay, fake, line, dev = unopened_relay(circ_actuation=False)   # every write is planned
rec = relay.step()
ok(rec.write is not None and rec.write["outcome"] == "planned", "the write is a PLAN")
ok(rec.forwarded is False, "and forwarded is False -- a plan delivered nothing")
ok(line.frames == [], "nothing was sent")
poke(fake, 9, 5.0)                        # a big jump from GOOD_ROW's DF9
rec2 = relay.step()
ok(rec2.quality["previous_value_c"] is None,
   "a plan establishes NO baseline -- previous_value_c is still None")
ok(rec2.quality["large_step"] is False,
   "so a jump after only plans is not flagged against a phantom baseline")

print("\n--- F4: a write-path exception still runs the fail-safe ---")
# allow_actuation on, but the link is never opened -> write_registers raises
# OUTSIDE the old bound()-only try. The whole sequence must be guarded.
relay, fake, line, dev = unopened_relay(circ_actuation=True)
exc = raises(CirculatorError, relay.step, "a write on an unopened link raises")
ok(exc is not None and getattr(exc, "record", None) is not None,
   "and it carries the fail-safe record -- safe() ran (F4)")
ok(relay.safe_mode is True, "the relay went to safe mode on the exception")
ok(fake.coils.get(registers.C1_COIL) is False, "and C1 was driven off by the fail-safe")

print("\n--- F5: an unconfirmed fail-safe raises FailSafeIncomplete, not a false success ---")
clock = Clock()
relay, fake, line = build(clock=clock, policy={"plc_stale_s": 5.0},
                          plc={"error_on": {"read_holding_registers"}},
                          serial={"error_response": True})   # the SAFE write will fail
relay.step()
clock.advance(5.001)
exc = raises(FailSafeIncomplete, relay.step,
             "beyond plc_stale_s with a dead bath link -> FailSafeIncomplete")
ok(isinstance(exc, CommsLost), "which is a CommsLost subclass, so existing handlers still catch it")
ok(exc is not None and "INCOMPLETE" in str(exc) and "ATTEMPTED" in str(exc),
   "and the message says ATTEMPTED, never 'was written'", str(exc)[:80])
ok(exc is not None and getattr(exc, "record", None) is not None
   and exc.record.quality["safe_confirmed"] is False,
   "the attached record shows safe_confirmed=False")
ok(fake.coils.get(registers.C1_COIL) is False,
   "C1 was still driven off (the PID-disable path is separate from the bath write)")

print("\n--- F6: the circulator write watchdog ATTEMPTS the safe setpoint ---")
clock = Clock()
relay, fake, line = build(clock=clock, policy={"circ_stale_s": 4.0},
                          serial={"error_response": True})
relay.step()                               # DF9 write fails, frame recorded
n_first = len(line.frames)
clock.advance(4.5)
exc = raises(CommsLost, relay.step, "beyond circ_stale_s the watchdog fires")
ok(fake.coils.get(registers.C1_COIL) is False, "C1 was disabled")
ok(last_frame_c(line) == SAFE_C,
   "and the SAFE setpoint was ATTEMPTED -- the last frame on the wire is it, not the last DF9",
   "%r" % last_frame_c(line))
# The watchdog step forwards DF9 first (a frame), THEN the watchdog attempts the
# safe setpoint (a second frame): two frames in the one step, the last SAFE_C.
ok(len(line.frames) == n_first + 2,
   "the watchdog step attempted the DF9 forward AND then the safe setpoint",
   "%d then %d" % (n_first, len(line.frames)))

print("\n--- F7: the cooperative-watchdog residual is a recordable constant ---")
ok(isinstance(COOPERATIVE_WATCHDOG_RESIDUAL, str)
   and "SD41" in COOPERATIVE_WATCHDOG_RESIDUAL
   and "cooperative" in COOPERATIVE_WATCHDOG_RESIDUAL.lower(),
   "COOPERATIVE_WATCHDOG_RESIDUAL names SD41 and the cooperative limit")
ok(Relay.COOPERATIVE_WATCHDOG_RESIDUAL == COOPERATIVE_WATCHDOG_RESIDUAL,
   "and is reachable on the Relay class for a phase script")
_readme = (REPO / "tools" / "environment" / "README.md").read_text(encoding="utf-8")
ok("What this layer cannot protect against" in _readme and "SD41" in _readme,
   "and the README documents what the layer cannot protect against")

print("\n%s" % ("ALL PASS" if fails == 0 else "%d FAILURE(S)" % fails))
sys.exit(1 if fails else 0)
