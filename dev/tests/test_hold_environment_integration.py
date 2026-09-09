#!/usr/bin/env python3
"""hold_environment.run() end-to-end over FakePlc + FakeSerial -- the path no
prior test drove.

Every earlier test wired a ``Relay`` directly; none drove ``hold_environment.run``,
so two defects a live run exposed slipped through:

* **D1 (logger):** the circulator link logs ``_emit(message, level)`` (two args)
  through the callback ``Run.logger`` hands it, which used to take one -- the run
  crashed with a ``TypeError`` at the port open.
* **D2 (setup order):** the PID was enabled BEFORE the port open, and the port
  open crashed, so the fail-safe (relay built only afterwards) never ran and the
  PLC's C1 was left ENABLED.

No socket is opened and the circulator's serial port is never touched (opening it
resets the MCU) -- ``run`` is driven through its test-only ``_plc_transport`` /
``_circ_transport`` seam over the two fakes every other test on this rig uses.

    python dev/tests/test_hold_environment_integration.py
"""
from __future__ import annotations

import os
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

# Run data goes to a throwaway dir, not the repo's dataset/ tree.
os.environ["SDL_DATA_ROOT"] = tempfile.mkdtemp(prefix="sdl-holdenv-int-")

from scripts.environment import hold_environment  # noqa: E402
from tools.circulator.api import FakeSerial, decode_setpoint  # noqa: E402
from tools.environment import registers  # noqa: E402
from tools.environment.plc import FakePlc  # noqa: E402

fails = 0


def ok(condition: bool, message: str, detail: str = "") -> None:
    global fails
    print("[test] %s: %s%s" % ("PASS" if condition else "FAIL", message,
                               "  (%s)" % detail if detail else ""))
    if not condition:
        fails += 1


C1 = registers.C1_COIL
DF9_C = registers.regs_to_f32(*registers.f32_to_regs(24.774))
SAFE_C = 15.0

#: A gate-open config with the fakes injected; both actuation gates true so a
#: bare --execute is not refused, boot_settle 0 so no wait, and a declared safe
#: setpoint the fail-safe can write.
CONFIG = {
    "environment": {"allow_actuation": True, "relay_period_s": 0.02},
    "circulator": {"allow_actuation": True, "safe_setpoint_c": SAFE_C,
                   "port": "/tmp/sdl-fake-circulator", "boot_settle_s": 0.0},
}

#: One real logged row, seeded through the codec by FakePlc.
ROW = {1: 95.0, 2: 25.0, 5: 25.0, 6: 95.0, 9: 24.774, 10: 25.340,
       18: 24.866, 19: 95.180}


def args_for(extra: list[str]):
    base = ["--duration", "0.06", "--period", "0.02", "--config", "IGNORED"]
    parsed = hold_environment.build_parser().parse_args(base + extra)
    parsed.config = CONFIG  # a dict config; from_config/LabConfig.load accept one
    return parsed


def run(extra, plc, line):
    """Drive run() with fakes; return (exit_code_or_None, raised_or_None)."""
    try:
        return hold_environment.run(args_for(extra), _plc_transport=plc,
                                    _circ_transport=line), None
    except BaseException as exc:                                      # noqa: BLE001
        return None, exc


# ---------------------------------------------------------------------------
print("\n--- D1: a live run reaches the circulator open() and its log call "
      "does NOT raise ---")
plc = FakePlc(dict(ROW), coils={C1: False})
line = FakeSerial()
code, raised = run(["--temp-sp", "24.0", "--execute"], plc, line)
ok(raised is None,
   "run() completed without the logger TypeError at the port open (D1)",
   repr(raised))
ok(line.open_count >= 1,
   "the circulator port was opened -- open() calls _emit(msg, level) BEFORE "
   "connect(), so this only reaches 1 if the two-arg log callback worked",
   "open_count=%d" % line.open_count)
ok(isinstance(code, int), "run() returned an int exit code", repr(code))


print("\n--- D2: a setup-phase crash (port open fails) leaves C1 DISABLED ---")
# fail_open makes bath_device.open() raise AFTER the setpoints are written. With
# the OLD order the PID was enabled before that crash and no relay existed to
# turn it off; with the fix the relay is armed first and the PID is enabled only
# AFTER a successful open, so the fail-safe disables C1 (or it was never on).
plc = FakePlc(dict(ROW), coils={C1: False})
line = FakeSerial(fail_open=True)
code, raised = run(["--temp-sp", "24.0", "--enable-pid", "--execute"], plc, line)
ok(raised is not None,
   "the port-open failure surfaces -- run() did not silently succeed",
   repr(raised))
ok(plc.coils.get(C1) is False,
   "after the crash the PLC's C1 (PID) is DISABLED: the fail-safe ran (D2)",
   "C1=%r" % plc.coils.get(C1))


print("\n--- a normal short run forwards a setpoint and ends with C1 disabled ---")
plc = FakePlc(dict(ROW), coils={C1: False})
line = FakeSerial()
code, raised = run(["--temp-sp", "24.0", "--enable-pid", "--execute"], plc, line)
ok(raised is None, "the run completed", repr(raised))
ok(isinstance(code, int), "and returned an int exit code", repr(code))
forwarded = [f for f in line.frames
             if abs(decode_setpoint(f["values"]) - DF9_C) < 1e-6]
ok(len(forwarded) >= 1,
   "at least one DF9 value was forwarded to the bath while the PID was on",
   "%d frame(s) total" % len(line.frames))
ok(plc.coils.get(C1) is False,
   "the run ends with C1 disabled -- finally: relay.safe() ran on the clean exit",
   "C1=%r" % plc.coils.get(C1))
ok(line.frames and abs(decode_setpoint(line.frames[-1]["values"]) - SAFE_C) < 1e-6,
   "and the last frame on the wire is the declared safe setpoint",
   "%r" % (decode_setpoint(line.frames[-1]["values"]) if line.frames else None))


print("\n%s" % ("ALL PASS" if fails == 0 else "%d FAILURE(S)" % fails))
sys.exit(1 if fails else 0)
