#!/usr/bin/env python3
"""Guards for the CLICK PLC layer. No hardware, no socket, no vendor SDK.

Every test here drives :class:`tools.environment.plc.FakePlc`, an in-memory
register bank with the same keyword signatures as the four ``pymodbus`` 3.15
calls the client actually makes. The PLC on this rig was unreachable when this
was written (ARP FAILED) and nothing here needs it.

The assertions that matter most are the defect classes that were live in the
code this layer replaces:

* a float32 setpoint written as ONE register carrying only the high word,
  leaving the low word stale -- observed live to turn 25.3 into 25.3587 and
  93.7 into 93.9349, and to *look* correct for every setpoint whose low word
  happens to be zero (25.0, 90, 92, 95, 87);
* a read whose error response was skipped, leaving the value NaN while
  downstream code forwarded it anyway;
* setpoint bounds that existed in a MATLAB GUI and were dropped by the Python
  port -- including the strictness of the interval, which a min/max pair in
  ``config.yaml`` cannot express on its own.

    python dev/tests/test_environment_safety.py
"""
from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from tools import config as _config  # noqa: E402
from tools.environment import plc as plc_module  # noqa: E402
from tools.environment import reading as _reading  # noqa: E402
from tools.environment import registers  # noqa: E402
from tools.environment.plc import (  # noqa: E402
    CONFIRMED,
    FAILED,
    PLANNED,
    FakePlc,
    PlcClient,
    PlcSettings,
    WriteResult,
)
from tools.environment.safety import (  # noqa: E402
    ActuationNotAllowed,
    BoundedSetpoint,
    PlcError,
    RateLimited,
    RateLimiter,
    SafetyError,
    SetpointLimits,
)

fails = 0

TEMP_SPEC = registers.BY_FIELD["temp_sp_c"]
TEMP_ADDR = TEMP_SPEC.address          # DF5 -> 28672 + 2*(5-1) = 28680


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
        print("[test] PASS: %s  -> %s" % (message, str(caught)[:66]))
        return
    except Exception as other:
        print("[test] FAIL: %s -- raised %s instead: %s"
              % (message, type(other).__name__, str(other)[:50]))
        fails += 1
        return
    print("[test] FAIL: %s -- nothing raised" % message)
    fails += 1


def settings(**overrides) -> PlcSettings:
    return PlcSettings.from_config({}, **overrides)


LIMITS = SetpointLimits()

print("--- setpoint bounds: the MATLAB GUI's STRICT open intervals ---")
sp = LIMITS.validate("temp_sp_c", 15.0)
ok(isinstance(sp, BoundedSetpoint), "15.0 C validates")
ok(sp.field == "temp_sp_c" and sp.value == 15.0, "the token carries field and value")
ok(sp.spec is TEMP_SPEC, "and the RegisterSpec it will be written to")
ok(sp.limits is LIMITS, "and the limits that approved it")
ok(sp.address == TEMP_ADDR, "its address comes from the register table")
raises(SafetyError, lambda: LIMITS.validate("temp_sp_c", 0.0),
       "temp exactly 0.0 C is refused (STRICT, per the MATLAB source)")
raises(SafetyError, lambda: LIMITS.validate("temp_sp_c", 30.0),
       "temp exactly 30.0 C is refused (STRICT)")
raises(SafetyError, lambda: LIMITS.validate("temp_sp_c", -0.1), "temp -0.1 C")
raises(SafetyError, lambda: LIMITS.validate("temp_sp_c", 30.1), "temp 30.1 C")
raises(SafetyError, lambda: LIMITS.validate("temp_sp_c", float("nan")), "temp NaN")
raises(SafetyError, lambda: LIMITS.validate("temp_sp_c", float("inf")), "temp inf")
raises(SafetyError, lambda: LIMITS.validate("temp_sp_c", float("-inf")), "temp -inf")
raises(SafetyError, lambda: LIMITS.validate("temp_sp_c", "25"),
       "a string setpoint")
ok(LIMITS.validate("rh_sp_pct", 95.0).value == 95.0, "95 %RH validates")
raises(SafetyError, lambda: LIMITS.validate("rh_sp_pct", 0.0), "RH exactly 0 % (STRICT)")
raises(SafetyError, lambda: LIMITS.validate("rh_sp_pct", 100.0), "RH exactly 100 % (STRICT)")
raises(SafetyError, lambda: LIMITS.validate("nonsuch_sp", 5.0), "an unknown field")
raises(SafetyError, lambda: LIMITS.validate("temp_kp", 5.0),
       "temp_kp -- a read-only-by-policy register")
raises(SafetyError, lambda: LIMITS.validate("temp_filtered_c", 5.0),
       "a sensor channel is not a setpoint")
raises(SafetyError, lambda: SetpointLimits(temp_max_c=40.0),
       "limits that WIDEN the 30 C code ceiling")
raises(SafetyError, lambda: SetpointLimits(rh_min_pct=-5.0),
       "limits that widen the 0 %RH floor")
raises(SafetyError, lambda: SetpointLimits(temp_min_c=20.0, temp_max_c=10.0),
       "limits whose min is above their max")
ok(SetpointLimits(temp_max_c=25.0).temp_max_c == 25.0, "but tightening to 25 C is allowed")
ok(len(registers.WRITABLE) == 2, "only two registers are writable at all",
   "%s" % sorted(registers.WRITABLE))

print("\n--- the token type: a bounded setpoint cannot be forged ---")
raises((SafetyError, TypeError),
       lambda: BoundedSetpoint(object(), "temp_sp_c", 15.0, TEMP_SPEC, LIMITS),
       "BoundedSetpoint() with a foreign token")
raises((SafetyError, TypeError), lambda: BoundedSetpoint("temp_sp_c", 15.0),
       "BoundedSetpoint() constructed positionally")

print("\n--- write_float32 takes the token, not a number ---")
fake = FakePlc({5: 87.3})
client = PlcClient(settings(allow_actuation=True), transport=fake)
raises(SafetyError, lambda: client.write_float32(25.3),
       "a plain float is refused BY TYPE, before any value check")
ok(fake.count("write_registers") == 0, "and nothing was written")
raises(SafetyError, lambda: client.write_float32(15.0),
       "an in-range plain float is refused too -- the guard is the type")

print("\n--- the half-float write CANNOT recur ---")
fake = FakePlc({5: 87.3})
stale_lo = fake.registers[TEMP_ADDR]
client = PlcClient(settings(allow_actuation=True), transport=fake)
result = client.write_float32(LIMITS.validate("temp_sp_c", 25.3))
writes = fake.calls_of("write_registers")
ok(result.outcome == CONFIRMED, "the write reports outcome=confirmed", result.outcome)
ok(result.ok is True, "and ok is True -- sent AND verified")
ok(len(writes) == 1, "exactly ONE write_registers call", "got %d" % len(writes))
sent = writes[0][1] if writes else {"values": None, "address": None, "device_id": None}
ok(sent["values"] is not None and len(sent["values"]) == 2,
   "carrying exactly TWO registers", "%r" % (sent["values"],))
ok(sent["address"] == TEMP_ADDR, "at DF5's own address %d" % TEMP_ADDR)
ok(sent["device_id"] == 1, "with device_id=1 (pymodbus 3.15 spelling, not slave=)")
ok(fake.count("write_coil") == 0, "and no coil was touched")
decoded = fake.value_at(5)
ok(abs(decoded - 25.3) < 1e-4, "the bank decodes to 25.3", "%r" % decoded)
half = registers.regs_to_f32(stale_lo, fake.registers[TEMP_ADDR + 1])
ok(abs(half - 25.3) > 1e-3,
   "and the OLD high-word-only write would have decoded to something else",
   "%.4f from the stale low word 0x%04X" % (half, stale_lo))
ok("write_register" not in dir(PlcClient),
   "PlcClient exposes no single-register write at all")
ok(not [n for n in dir(PlcClient) if "single" in n.lower()],
   "and nothing named *single*")
ok(result.readback == tuple(sent["values"]),
   "the WriteResult carries the words it read back")
ok(result.field == "temp_sp_c" and result.value == 25.3,
   "and what it was asked to write")

print("\n--- every write is read back and compared word-for-word ---")


class CorruptingFake(FakePlc):
    """Stores a different low word than it was handed. Nothing else changes."""

    def write_registers(self, address, values, *, device_id=1,
                        no_response_expected=False):
        return super().write_registers(address, [values[0] ^ 0x0001, values[1]],
                                       device_id=device_id,
                                       no_response_expected=no_response_expected)


corrupt = CorruptingFake({5: 87.3})
raises(SafetyError,
       lambda: PlcClient(settings(allow_actuation=True), transport=corrupt)
       .write_float32(LIMITS.validate("temp_sp_c", 25.3)),
       "a read-back mismatch raises")
try:
    PlcClient(settings(allow_actuation=True), transport=CorruptingFake({5: 87.3})
              ).write_float32(LIMITS.validate("temp_sp_c", 25.3))
    ok(False, "the mismatch raise carries a failed WriteResult")
except SafetyError as caught:
    carried = getattr(caught, "result", None)
    ok(carried is not None and carried.outcome == FAILED,
       "the mismatch raise carries a WriteResult with outcome=failed",
       getattr(carried, "outcome", None))
    ok(carried is not None and carried.ok is False, "whose ok is False")
err_fake = FakePlc({5: 87.3}, error_on={"read_holding_registers"})
raises(SafetyError,
       lambda: PlcClient(settings(allow_actuation=True), transport=err_fake)
       .write_float32(LIMITS.validate("temp_sp_c", 25.3)),
       "a write whose read-back could not be obtained raises")
write_err = FakePlc({5: 87.3}, error_on={"write_registers"})
res = PlcClient(settings(allow_actuation=True), transport=write_err).write_float32(
    LIMITS.validate("temp_sp_c", 25.3))
ok(res.outcome == FAILED and res.error,
   "an error response to the write reports outcome=failed", res.outcome)
ok(res.ok is False, "and ok is False")
raises(ValueError, lambda: WriteResult(outcome="fine", address=0, values=()),
       "an outcome outside the three states is refused")

print("\n--- the actuation gate ---")
gated = PlcClient(settings(allow_actuation=False), transport=FakePlc({5: 25.0}))
raises(ActuationNotAllowed,
       lambda: gated.write_float32(LIMITS.validate("temp_sp_c", 25.3)),
       "a setpoint write with allow_actuation=False")
dry = PlcClient(settings(allow_actuation=False), transport=None)
frame = dry.write_float32(LIMITS.validate("temp_sp_c", 25.3), dry_run=True)
ok(frame.outcome == PLANNED and frame.address == TEMP_ADDR,
   "dry-run describes the frame it would have sent", frame.outcome)
ok(frame.ok is False,
   "and ok is False -- a plan is not a write, so it cannot read as one")
ok(frame.dry_run is True, "dry_run is derived from the outcome, not stored beside it")
ok(len(frame.values) == 2, "the dry-run frame is two registers")
ok(dry.transport is None, "and constructed no transport at all")
ok("DRY-RUN" in frame.describe(), "and says so when printed")

print("\n--- PID coil: disabling is the safe direction, and is always allowed ---")
coil_fake = FakePlc({}, coils={registers.C1_COIL: True})
safe = PlcClient(settings(allow_actuation=False), transport=coil_fake)
off = safe.set_pid(False)
ok(off.outcome == CONFIRMED, "set_pid(False) succeeds with allow_actuation=False",
   off.outcome)
ok(off.ok is True, "and its read-back confirmed it")
plan = safe.set_pid(True, dry_run=True)
ok(plan.outcome == PLANNED and plan.ok is False,
   "set_pid(True, dry_run=True) plans without the gate and without claiming ok")
ok(coil_fake.coils[registers.C1_COIL] is False, "and the coil is actually cleared")
ok(coil_fake.calls_of("write_coil")[0][1]["address"] == registers.C1_COIL,
   "written to C1 at %d" % registers.C1_COIL)
raises(ActuationNotAllowed, lambda: safe.set_pid(True),
       "set_pid(True) needs the gate")
armed = PlcClient(settings(allow_actuation=True),
                  transport=FakePlc({}, coils={registers.C1_COIL: False}))
ok(armed.set_pid(True).outcome == CONFIRMED,
   "set_pid(True) succeeds once the gate is open")
coil_err = PlcClient(settings(allow_actuation=False),
                     transport=FakePlc({}, coils={registers.C1_COIL: True},
                                       error_on={"write_coil"}))
ok(coil_err.set_pid(False).outcome == FAILED,
   "a coil write that errored reports outcome=failed, not ok")

print("\n--- a failed read carries read_ok=False and fabricates nothing ---")
boom = FakePlc({5: 25.0}, raise_on={"read_holding_registers"})
r = PlcClient(settings(), transport=boom).read_block()
ok(r.read_ok is False, "a transport exception -> read_ok=False, no exception raised")
ok(r.channels == {}, "and NO channel values at all", "%r" % r.channels)
ok(r.temp_filtered_c is None, "the accessor is None -- not NaN, not 0.0, not stale")
ok(r.pid_enabled is None, "the coil is unknown, not assumed off")
ok(boom.count("read_holding_registers") == 1, "and it was NOT retried")
errored = FakePlc({5: 25.0}, error_on={"read_holding_registers"})
r2 = PlcClient(settings(), transport=errored).read_block()
ok(r2.read_ok is False and r2.channels == {},
   "an error RESPONSE is treated the same way")
lost = PlcClient(settings(), transport=FakePlc({}, raise_on={"read_coils"}))
ok(lost.read_pid_enabled() is None, "read_pid_enabled() -> None on failure")
ok(lost.last_error is not None, "and the fault text is kept, not swallowed")
ok(PlcClient(settings(), transport=FakePlc({}, fail_connect=True)).connect() is False,
   "a refused connect is reported, not raised")

print("\n--- a short block is partial, not an error ---")
short = FakePlc({1: 95.18, 2: 24.866, 5: 25.0}, short_registers=10)
r3 = PlcClient(settings(), transport=short).read_block()
ok(r3.partial is True, "partial=True")
ok(r3.read_ok is True, "read_ok stays True -- some pairs did arrive")
ok("temp_sp_c" in r3.channels and "temp_filtered_c" not in r3.channels,
   "only the pairs that arrived are present, the rest are absent")

print("\n--- the rate limiter, on an injected clock ---")
rl = RateLimiter()
raises(RateLimited,
       lambda: rl.check("temp_sp_c", 26.0, now=100.0, last_value=20.0, last_time=None),
       "a 6 C step (cap 5 C)")
rl.check("temp_sp_c", 26.0, now=100.0, last_value=20.0, last_time=None,
         allow_large_step=True)
ok(True, "the same step passes with allow_large_step=True")
rl.check("temp_sp_c", 24.0, now=100.0, last_value=20.0, last_time=None)
ok(True, "a 4 C step passes")
raises(RateLimited,
       lambda: rl.check("temp_sp_c", 21.0, now=103.0, last_value=20.0, last_time=100.0),
       "two writes 3 s apart (min interval 10 s)")
rl.check("temp_sp_c", 21.0, now=115.0, last_value=20.0, last_time=100.0)
ok(True, "15 s apart passes")
raises(RateLimited,
       lambda: rl.check("rh_sp_pct", 55.0, now=100.0, last_value=40.0, last_time=None),
       "a 15 %RH step (cap 10 %)")
raises(TypeError,
       lambda: rl.check("temp_sp_c", 21.0, last_value=20.0, last_time=None),
       "check() has no default clock -- `now` is required")
raises(RateLimited,
       lambda: rl.check("temp_sp_c", 21.0, now=90.0, last_value=20.0, last_time=100.0),
       "a clock that ran backwards is refused, not read as a long wait")
raises(RateLimited,
       lambda: rl.check("temp_sp_c", 26.0, now=103.0, last_value=20.0,
                        last_time=100.0, allow_large_step=True),
       "allow_large_step waives the STEP cap only, never the interval")
rl.check("temp_sp_c", 21.0, now=100.0, last_value=None, last_time=None)
ok(True, "the first write of a session has no step to compare against")
ok(issubclass(RateLimited, PlcError) and issubclass(SafetyError, PlcError),
   "every exception roots at PlcError")
ok(issubclass(ActuationNotAllowed, SafetyError), "and the gate is a safety error")

print("\n--- settings: precedence, provenance, and no laxer fallback ---")
base = PlcSettings.from_config({})
ok(base.host == "169.254.33.33" and base.port == 502, "code defaults")
ok(base.device_id == 1, "device_id defaults to 1 (INFERENCE -- see plc.py)")
ok(base.allow_actuation is False, "actuation is off unless asked for")
ok(base.retries == 0, "retries=0 -- a retry is a decision this layer never makes")
ok(base.sources["host"] == "default", "and sources names the layer")
ok("[default]" in base.describe(), "describe() prints the source layer")
ok(base.limits.temp_max_c == 30.0, "the limits come from the settings")
tight = PlcSettings.from_config({"environment": {"temp_sp_max_c": 25.0}})
ok(tight.temp_sp_max_c == 25.0, "config.yaml may TIGHTEN a ceiling")
ok(tight.limits.temp_max_c == 25.0, "and the tightened bound reaches SetpointLimits")
ok(tight.sources["temp_sp_max_c"] == "config.yaml", "and the source says so")
ok("config.yaml" in tight.describe(), "describe() shows it came from the file")
raises(SafetyError, lambda: tight.limits.validate("temp_sp_c", 27.0),
       "27 C is refused under the tightened ceiling")
raises(_config.ConfigError,
       lambda: PlcSettings.from_config({"environment": {"temp_sp_max_c": 40.0}}),
       "config.yaml may NOT widen the 30 C ceiling")
raises(_config.ConfigError,
       lambda: PlcSettings.from_config({"environment": {"rh_sp_min_pct": -5.0}}),
       "nor the 0 %RH floor")
raises(_config.ConfigError,
       lambda: PlcSettings.from_config({"environment": {"timeout_s": "soonish"}}),
       "an unparseable value raises -- never falls through to the default")
raises(_config.ConfigError,
       lambda: PlcSettings.from_config({"environment": {"allow_actuation": "flase"}}),
       "a typo-d boolean gate raises rather than reading as True")
ok(PlcSettings.from_config({"environment": {}}).host == "169.254.33.33",
   "an ABSENT key yields the code default")
raises(_config.ConfigError,
       lambda: PlcSettings.from_config({"environment": {"temp_sp_min_c": 20.0,
                                                        "temp_sp_max_c": 10.0}}),
       "min above max is cross-field validated")
raises(_config.ConfigError,
       lambda: PlcSettings.from_config({"environment": {"plc_stale_s": 0.5,
                                                        "relay_period_s": 1.0}}),
       "a watchdog shorter than the loop period is cross-field validated")
ok(PlcSettings.from_config({}, allow_actuation=True).sources["allow_actuation"]
   == "argument", "an override outranks the file and says so")

print("\n--- this parser's two traps: an empty key, and coerced scalars ---")
raises(_config.ConfigError,
       lambda: PlcSettings.from_config({"environment": {"timeout_s": {}}}),
       "an EMPTY key parses to {} -- present and unusable, so it raises")
ok(PlcSettings.from_config({"environment": {}}).timeout_s == 2.0,
   "while a genuinely absent key still yields the code default")
raises(_config.ConfigError,
       lambda: PlcSettings.from_config({"environment": {"host": False}}),
       "a bare N/no/off parses to False -- refused, not str()-ed into a hostname")
raises(_config.ConfigError,
       lambda: PlcSettings.from_config({"environment": {"port": False}}),
       "and not int()-ed into port 0")
raises(_config.ConfigError,
       lambda: PlcSettings.from_config({"environment": {"device_id": True}}),
       "nor into device_id 1, which would look entirely plausible")

print("\n--- the real configs/config.yaml resolves, and is not laxer than code ---")
live = PlcSettings.from_config(None)
ok(live.host == "169.254.33.33", "the repo config resolves", live.host)
ok(live.allow_actuation is False, "and ships with actuation OFF")
ok(live.limits.temp_max_c <= 30.0 and live.limits.rh_max_pct <= 100.0,
   "and never loosens the code ceilings")
ok(live.sources["allow_actuation"] == "config.yaml",
   "the gate's provenance is the file, not a default")

print("\n--- diagnostics: the SC/SD bits are unavailable, not guessed ---")
diag = PlcClient(settings(), transport=FakePlc({})).diagnostics()
ok(diag["max_concurrent_clients"] == 3, "the 3-client ceiling is reported")
native = diag["native_diagnostics"]
ok(len(native) == 7, "all seven native bits are listed", "%d" % len(native))
ok(all(entry["modbus_address"] is None for entry in native.values()),
   "every native diagnostic address is None -- none is guessed")
ok(all("unavailable" in entry["status"] for entry in native.values()),
   "and each says unavailable")
ok("TODO(operator)" in native["SD41"]["todo"],
   "with the operator step that would resolve it")
ok("INFERENCE" in diag["device_id_provenance"], "device_id is marked as inference")

print("\n--- the dashboard reads a published file, not a Modbus session ---")
tmp = Path(tempfile.mkdtemp(prefix="env-publish-"))
plc_module.PUBLISH_DIR = tmp / "nested"
words: list[int] = []
seed = {5: 25.0, 6: 95.0, 18: 24.866}
for spec in registers.REGISTERS:
    lo, hi = registers.f32_to_regs(seed.get(spec.df, 1.0))
    words.extend((lo, hi))
sample = _reading.decode_block(words, t_utc="2025-07-26T15:52:32+00:00",
                               monotonic_s=1.0, pid_enabled=True)
published = PlcClient(settings(), transport=FakePlc({})).publish_last_reading(sample)
ok(published.exists(), "publish_last_reading() created the file and its directory")
back = plc_module.latest_published()
ok(back is not None, "latest_published() reads it back")
ok(abs(back["channels"]["temp_sp_c"] - 25.0) < 1e-4, "the setpoint round-trips")
ok(abs(back["channels"]["temp_filtered_c"] - 24.866) < 1e-4, "so does the sensor")
ok(back["pid_enabled"] is True, "and the coil state")
ok(0.0 <= back["age_s"] < 60.0, "and it reports a plausible age_s",
   "%r" % back.get("age_s"))
ok(not list(plc_module.PUBLISH_DIR.glob("*.tmp*")),
   "the atomic temp file was replaced, not left behind")
raw = json.loads(published.read_text(encoding="utf-8"))
ok("age_s" not in raw, "age_s is computed on read, not frozen into the file")
time.sleep(0.05)
ok(plc_module.latest_published()["age_s"] > 0.0, "age_s advances")
plc_module.PUBLISH_DIR = tmp / "empty"
ok(plc_module.latest_published() is None, "no published file -> None, not a fake reading")

print("\n--- no vendor SDK at import time ---")
probe = subprocess.run(
    [sys.executable, "-c",
     "import sys; sys.path.insert(0, %r); import tools.environment.plc; "
     "print('pymodbus' in sys.modules)" % str(REPO)],
    capture_output=True, text=True, timeout=90)
ok(probe.returncode == 0 and probe.stdout.strip() == "False",
   "importing tools.environment.plc does not import pymodbus",
   (probe.stdout + probe.stderr).strip()[:66])

print("\n%s" % ("ALL PASS" if fails == 0 else "%d FAILURE(S)" % fails))
sys.exit(1 if fails else 0)
