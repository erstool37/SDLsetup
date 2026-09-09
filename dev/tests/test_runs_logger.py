#!/usr/bin/env python3
"""D1: the callable ``Run.logger(device)`` returns must accept a level argument.

A device's ``log=`` callback is called by that device with ``(message, level)``
-- the circulator link's ``_emit`` does exactly that. The prior one-argument
lambda crashed a real run with::

    TypeError: Run.logger.<locals>.<lambda>() takes 1 positional argument but 2
    were given

So the callable must accept ``(message)`` AND ``(message, level)``, and the
level must reach the transcript. Runs against a temp directory; no hardware.

    python dev/tests/test_runs_logger.py
"""
from __future__ import annotations

import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from tools import runs  # noqa: E402

fails = 0


def ok(condition: bool, message: str, detail: str = "") -> None:
    global fails
    print("[test] %s: %s%s" % ("PASS" if condition else "FAIL", message,
                               "  (%s)" % detail if detail else ""))
    if not condition:
        fails += 1


tmp = Path(tempfile.mkdtemp(prefix="sdl-runs-logger-"))
run = runs.Run.create("logger-test", config=None, root=tmp,
                      description="D1 logger-signature regression")
log = run.logger("circulator")

print("\n--- the callable accepts (message) ---")
raised = None
try:
    log("plain info message")
except BaseException as exc:                                          # noqa: BLE001
    raised = exc
ok(raised is None, "logger()(message) does not raise", repr(raised))

print("\n--- the callable accepts (message, level) -- the D1 crash ---")
raised = None
try:
    log("OPENING /dev/ttyUSB0 -- RESETS THE MCU", "warn")
except BaseException as exc:                                          # noqa: BLE001
    raised = exc
ok(raised is None,
   "logger()(message, level) does not raise (the exact live-run crash)",
   repr(raised))

print("\n--- the level reaches the device transcript ---")
device_log = run.path / "devices" / "circulator" / "circulator.log"
run.close()
text = device_log.read_text(encoding="utf-8") if device_log.exists() else ""
ok("plain info message" in text, "the info line was written", device_log.name)
ok("WARN: OPENING" in text,
   "the warn line was written AT WARN level, not silently downgraded",
   repr([ln for ln in text.splitlines() if "OPENING" in ln][:1]))

print("\n%s" % ("ALL PASS" if fails == 0 else "%d FAILURE(S)" % fails))
sys.exit(1 if fails else 0)
