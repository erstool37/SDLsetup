#!/usr/bin/env python3
"""The arm and the UV-Vis carrier can never move at the same time.

The carrier slides out into the arm's workspace. They are different
instruments, on different interfaces, driven by different processes -- so the
only thing standing between them is this interlock. No hardware is touched:
every claim here is a file in a temp directory.

    python dev/tests/test_occupancy.py
"""
from __future__ import annotations

import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from tools import occupancy  # noqa: E402

failures = 0


def ok(condition: bool, message: str, detail: str = "") -> None:
    global failures
    print("[test] %s: %s%s" % ("PASS" if condition else "FAIL", message,
                               f"  ({detail})" if detail else ""))
    if not condition:
        failures += 1


def blocked(fn, message: str) -> None:
    global failures
    try:
        fn()
    except occupancy.Busy as exc:
        print("[test] PASS: %s  -> %s" % (message, str(exc)[:74]))
        return
    print("[test] FAIL: %s -- nothing raised" % message)
    failures += 1


tmp = tempfile.mkdtemp(prefix="occupancy-test-")
occupancy.OCCUPANCY_DIR = Path(tmp)

print("--- the conflict this exists for ---")
ok("uv_vis" in occupancy.CONFLICTS["arm"], "arm conflicts with uv_vis")
ok("arm" in occupancy.CONFLICTS["uv_vis"], "uv_vis conflicts with arm (both directions)")
ok(occupancy.CONFLICTS["microscope"] == frozenset(),
   "the cameras conflict with nothing -- they do not move")

print("\n--- a claim blocks the instrument it shares space with ---")
with occupancy.claim("arm", doing="A1 focus sweep"):
    blocked(lambda: occupancy.require_free("uv_vis"),
            "the carrier is refused while the arm is moving")
    blocked(lambda: occupancy.require_free("arm"),
            "a second process is refused the arm")
    occupancy.require_free("microscope")
    ok(True, "the cameras are still allowed -- they do not share space")
    held = occupancy.current("arm")
    ok(held is not None and held.doing == "A1 focus sweep",
       "the panel can see what the arm is doing", held.doing if held else "-")

print("\n--- and releases on the way out ---")
occupancy.require_free("uv_vis")
ok(True, "the carrier is allowed again once the arm finishes")
ok(occupancy.current("arm") is None, "no claim is left behind")

print("\n--- it releases on failure too, or an abort would lock the rig ---")
try:
    with occupancy.claim("arm", doing="run that fails"):
        raise RuntimeError("boom")
except RuntimeError:
    pass
ok(occupancy.current("arm") is None, "an exception still released the claim")
occupancy.require_free("uv_vis")
ok(True, "the carrier is not stranded by a crashed arm run")

print("\n--- the reverse direction ---")
with occupancy.claim("uv_vis", doing="carrier plate_out"):
    blocked(lambda: occupancy.require_free("arm"),
            "the arm is refused while the carrier is out")

print("\n--- a claim from a dead process must not lock the rig forever ---")
dead = subprocess.run([sys.executable, "-c", "import os; print(os.getpid())"],
                      capture_output=True, text=True)
dead_pid = int(dead.stdout.strip())
(Path(tmp) / "arm.json").write_text(
    '{"doing": "crashed run", "pid": %d, "since": %f, "phase": "t"}' % (dead_pid, time.time()))
ok(occupancy.current("arm") is None,
   "a claim whose process is gone is cleared automatically", f"pid {dead_pid}")
occupancy.require_free("uv_vis")
ok(True, "and the rig is usable again without deleting a file by hand")

print("\n--- a LIVE claim is never cleared as stale ---")
(Path(tmp) / "arm.json").write_text(
    '{"doing": "real run", "pid": %d, "since": %f, "phase": "t"}' % (os.getpid(), time.time()))
ok(occupancy.current("arm") is not None, "our own live claim survives")
blocked(lambda: occupancy.require_free("uv_vis"), "and still blocks the carrier")
(Path(tmp) / "arm.json").unlink()

print("\n--- claiming refuses BEFORE starting anything ---")
with occupancy.claim("arm", doing="holding"):
    started = []
    try:
        with occupancy.claim("uv_vis", doing="should never start"):
            started.append(True)
    except occupancy.Busy:
        pass
    ok(not started, "the blocked claim's body never ran")

print("\n--- what the panel renders ---")
with occupancy.claim("arm", doing="A1 sweep"):
    state = occupancy.status()
ok(state["arm"]["busy"] is True, "arm reports busy while claimed")
ok(state["uv_vis"]["busy"] is False, "uv_vis reports idle")
ok("age_s" in state["arm"] and "pid" in state["arm"],
   "the panel gets how long and which process")
ok(occupancy.status()["arm"]["busy"] is False, "and idle again afterwards")

print("\n--- the PLC and the circulator are registered, and self-exclude ---")
ok("environment" in occupancy.CONFLICTS, "environment is in the policy table")
ok("circulator" in occupancy.CONFLICTS, "circulator is in the policy table")
ok(occupancy.CONFLICTS["environment"] == frozenset(),
   "environment conflicts with nothing else -- the PLC does not move")
ok(occupancy.CONFLICTS["circulator"] == frozenset(),
   "circulator conflicts with nothing else -- the bath does not move")

print("\n--- but each still excludes ITSELF, which is the point ---")
# The PLC refuses a 4th concurrent Modbus TCP client, so a second supervisor
# session is how a run loses its connection.
with occupancy.claim("environment", doing="setpoint relay"):
    blocked(lambda: occupancy.require_free("environment"),
            "a second PLC session is refused (only 3 TCP clients exist)")
    occupancy.require_free("circulator")
    ok(True, "the circulator is unaffected -- different interface")
    occupancy.require_free("arm")
    ok(True, "and the arm is unaffected -- no shared bench space")

# Opening the circulator's serial port asserts DTR, which is wired to /RESET:
# a second opener reboots the MCU under the first.
with occupancy.claim("circulator", doing="bath setpoint write"):
    blocked(lambda: occupancy.require_free("circulator"),
            "a second port open is refused (opening resets the MCU)")
    occupancy.require_free("environment")
    ok(True, "the PLC is unaffected by the circulator being busy")
    occupancy.require_free("arm")
    ok(True, "and so is the arm")

print("\n--- and neither blocks, nor is blocked by, the arm ---")
with occupancy.claim("arm", doing="plate transport"):
    occupancy.require_free("environment")
    occupancy.require_free("circulator")
    ok(True, "a moving arm does not stop the enclosure or the bath")
ok(occupancy.current("environment") is None and occupancy.current("circulator") is None,
   "no claim left behind for either")

print("\n%s" % ("ALL PASS" if failures == 0 else "%d FAILURE(S)" % failures))
sys.exit(1 if failures else 0)
