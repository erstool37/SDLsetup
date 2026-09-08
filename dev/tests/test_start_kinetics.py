#!/usr/bin/env python3
"""start_kinetics / check_environment: dry-run planning, guarded refusal, and the
reused fail-safe -- all against fakes, no socket, no serial port.

Opening the circulator's port hardware-resets its MCU, so like every other test
on this rig this one touches neither the wire nor the port. The live PLC/dry-run
checks belong to the verification report, not to the suite.

    python dev/tests/test_start_kinetics.py
"""
from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from scripts.environment.check_environment import check_environment  # noqa: E402
from scripts.environment.start_kinetics import start_kinetics  # noqa: E402
from tools.circulator.api import (  # noqa: E402
    Circulator,
    CirculatorSettings,
    FakeSerial,
    decode_setpoint,  # noqa: E402
)
from tools.environment import registers  # noqa: E402
from tools.environment.plc import FakePlc, PlcClient, PlcSettings  # noqa: E402
from tools.environment.relay import Relay, RelayPolicy  # noqa: E402
from tools.environment.safety import CommsLost, FailSafeIncomplete, SafetyError  # noqa: E402

fails = 0


def ok(condition: bool, message: str, detail: str = "") -> None:
    global fails
    print("[test] %s: %s%s" % ("PASS" if condition else "FAIL", message,
                               "  (%s)" % detail if detail else ""))
    if not condition:
        fails += 1


#: One real logged row plus the raw channels check_environment reports.
ROW = {1: 95.0, 2: 25.0, 5: 25.0, 6: 95.0, 9: 24.774, 10: 25.340,
       18: 24.866, 19: 95.180}
SAFE_C = 15.0
DF9 = registers.df_address(9)


class Clock:
    def __init__(self, t: float = 1000.0) -> None:
        self.t = float(t)

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += float(dt)


def fake_client(pid_on: bool = True) -> PlcClient:
    fake = FakePlc(dict(ROW), coils={registers.C1_COIL: bool(pid_on)})
    return PlcClient(PlcSettings(allow_actuation=False), fake)


def poke(fake: FakePlc, df: int, value: float) -> None:
    low, high = registers.f32_to_regs(value)
    address = registers.df_address(df)
    fake.registers[address] = low
    fake.registers[address + 1] = high


# ---------------------------------------------------------------------------
print("\n--- an out-of-range setpoint is REFUSED through the guarded path (dry run) ---")
for bad_temp in (99.0, 30.0, 0.0, -1.0):
    try:
        start_kinetics(temp_c=bad_temp, _client=fake_client())
        ok(False, "temp_c=%g should refuse" % bad_temp, "did not raise")
    except SafetyError as exc:
        ok(True, "temp_c=%g refused" % bad_temp, str(exc)[:60])
    except Exception as exc:  # noqa: BLE001
        ok(False, "temp_c=%g refused with the wrong type" % bad_temp,
           "%s: %s" % (type(exc).__name__, exc))
try:
    start_kinetics(rh_pct=101.0, _client=fake_client())
    ok(False, "rh_pct=101 should refuse", "did not raise")
except SafetyError:
    ok(True, "rh_pct=101 refused too")


print("\n--- a normal dry run returns executed=False and produces a plan ---")
summary = start_kinetics(temp_c=25.0, rh_pct=93.0, hours=72.0,
                         _client=fake_client())
ok(summary["executed"] is False, "executed is False", repr(summary["executed"]))
ok(summary["ended"] == "planned", "ended == 'planned'", summary["ended"])
ok(summary["iterations"] == 0, "no iterations ran in a dry run")
ok(summary["run_dir"] and Path(summary["run_dir"]).exists(),
   "a Run folder was created", str(summary["run_dir"]))
ok((Path(summary["run_dir"]) / "manifest.json").exists(),
   "the run wrote a manifest")
ok(isinstance(summary["final_reading"], dict)
   and summary["final_reading"]["channels"].get("temp_pid_output_c") is not None,
   "the read-only current-state read populated final_reading")
ok(any("DRY RUN" in n for n in summary["notes"]),
   "the notes record that nothing was written")


print("\n--- execute=True is REFUSED while the config gates are shut ---")
# The shipped config has both allow_actuation false, so this is the real path.
summary = start_kinetics(temp_c=25.0, execute=True, _client=fake_client())
ok(summary["executed"] is False, "not executed with a gate shut")
ok(summary["run_dir"] is None, "no run was created -- refused before any I/O")
ok("refused" in summary["ended"], "ended names the refusal", summary["ended"])
ok(any("allow_actuation" in n for n in summary["notes"]),
   "the refusal names the shut gate")


print("\n--- check_environment returns a zero-actuation status read ---")
state = check_environment(_client=fake_client(pid_on=True))
ok(abs(state["temp_filtered_c"] - 24.866) < 1e-3, "temp filtered reported",
   repr(state["temp_filtered_c"]))
ok(abs(state["temp_raw_c"] - 25.0) < 1e-3, "temp raw reported",
   repr(state["temp_raw_c"]))
ok(abs(state["rh_filtered_pct"] - 95.18) < 1e-2, "rh filtered reported",
   repr(state["rh_filtered_pct"]))
ok(abs(state["rh_raw_pct"] - 95.0) < 1e-3, "rh raw reported")
ok(abs(state["temp_sp_c"] - 25.0) < 1e-3, "temp setpoint reported")
ok(abs(state["rh_sp_pct"] - 95.0) < 1e-3, "rh setpoint reported")
ok(state["pid_enabled"] is True, "PID state reported", repr(state["pid_enabled"]))
ok("UNKNOWN" in state["note"], "the unknown-channel note is present")
state_off = check_environment(_client=fake_client(pid_on=False))
ok(state_off["pid_enabled"] is False, "PID-off state reported")


print("\n--- CONTROL PATH: one forward, then a simulated comms-loss fail-safe ---")
# This is the machinery start_kinetics's execute path drives through
# hold_environment: a Relay over the same two fakes. Deterministic clock, so the
# watchdog is asserted rather than slept through.
clock = Clock()
fake = FakePlc(dict(ROW), coils={registers.C1_COIL: True})
plc = PlcClient(PlcSettings(allow_actuation=True), fake)
line = FakeSerial()
bath = Circulator(CirculatorSettings(port="/tmp/sdl-fake-circulator",
                                     allow_actuation=True, boot_settle_s=0.0), line)
bath.open(settle=False)
relay = Relay(plc, bath,
              RelayPolicy(safe_setpoint_c=SAFE_C, command_limits=bath.limits),
              clock=clock)

record = relay.step()
df9 = registers.regs_to_f32(*registers.f32_to_regs(24.774))
ok(record.forwarded is True, "a good reading is forwarded")
ok(record.write["outcome"] == "confirmed", "the write is confirmed")
ok(line.frames and abs(decode_setpoint(line.frames[-1]["values"]) - df9) < 1e-6,
   "DF9 reached the bath on the wire", str(len(line.frames)))

# Now the PLC goes silent: seed a NaN DF9 (a failed upstream read) and hold it
# past plc_stale_s. The relay must fire its watchdog, run the pre-declared
# fail-safe, latch safe mode, and put the SAFE setpoint on the wire.
fake.registers[DF9] = 0x0000
fake.registers[DF9 + 1] = 0x7FC0  # float32 quiet NaN
fired = None
for _ in range(10):
    clock.advance(1.0)
    try:
        relay.step()
    except CommsLost as exc:  # FailSafeIncomplete is a CommsLost subclass
        fired = exc
        break
ok(fired is not None, "the staleness watchdog fired CommsLost")
ok(relay.safe_mode is True, "safe mode is LATCHED after the fail-safe")
ok(line.frames and abs(decode_setpoint(line.frames[-1]["values"]) - SAFE_C) < 1e-6,
   "the SAFE setpoint is the last thing on the wire", str(len(line.frames)))
ok(not isinstance(fired, FailSafeIncomplete),
   "the fail-safe CONFIRMED against the fake echo (plain CommsLost, not INCOMPLETE)")


print()
if fails:
    print("[test] %d FAILURE(S)" % fails)
    sys.exit(1)
print("[test] ALL PASS")
