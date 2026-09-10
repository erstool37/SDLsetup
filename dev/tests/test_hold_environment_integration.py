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

It also covers the three things added on 2026-09-09, all of which live in this
loop and nowhere else:

* **the operator stop path** -- a background job inherits SIGINT ignored, so
  neither Ctrl-C nor ``kill -INT`` could stop a live hold, and ``kill -TERM``
  would skip the ``finally`` that disables the PID;
* **publishing every step** -- ``publish_last_reading`` was called only by the
  read helpers, so the dashboard showed a stale chamber for the whole of a run;
* **consuming dashboard commands** -- setpoints and PID on/off applied mid-run
  through ``tools.environment.channel``, each through the same guards a CLI flag
  goes through.

No socket is opened and the circulator's serial port is never touched (opening it
resets the MCU) -- ``run`` is driven through its test-only ``_plc_transport`` /
``_circ_transport`` seam over the two fakes every other test on this rig uses,
under the ``no_hardware`` tripwire that makes a real transport unconstructible.

    python dev/tests/test_hold_environment_integration.py
"""
from __future__ import annotations

import json
import os
import signal
import sys
import time
from pathlib import Path

import no_hardware  # MUST be first: it blocks every real transport

from scripts.environment import hold_environment
from tools import runs
from tools.circulator.api import FakeSerial, decode_setpoint
from tools.environment import channel, registers
from tools.environment.plc import FakePlc, latest_published

fails = 0


def ok(condition: bool, message: str, detail: str = "") -> None:
    global fails
    print("[test] %s: %s%s" % ("PASS" if condition else "FAIL", message,
                               "  (%s)" % detail if detail else ""))
    if not condition:
        fails += 1


def coil_writes_true(plc) -> int:
    """How many times C1 was driven TRUE. Zero is the safe answer."""
    return len([call for call in plc.calls_of("write_coil")
                if call[1]["value"] is True])


C1 = registers.C1_COIL
DF5 = registers.df_address(5)
DF6 = registers.df_address(6)
DF9_C = registers.regs_to_f32(*registers.f32_to_regs(24.774))
SAFE_C = 15.0

#: The float32 words for 95.0 %RH, low word first -- what DF6/DF6+1 must hold
#: after a dashboard humidity command. Written as literals so the test does not
#: re-derive the value through the same codec it is checking.
RH95_WORDS = [0x0000, 0x42BE]

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

#: The same row with DF6 (RH SP) seeded AWAY from 95.0, so a command asking for
#: 95.0 is a real change on the wire. With ROW itself the register already holds
#: 95.0's words and the assertion would pass without anything being written --
#: a check that cannot fail.
ROW_RH80 = {**ROW, 6: 80.0}


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


def latest_manifest() -> dict:
    """The newest run manifest under the throwaway data root."""
    paths = sorted(no_hardware.DATA_ROOT.rglob("manifest.json"),
                   key=lambda path: path.stat().st_mtime)
    return json.loads(paths[-1].read_text(encoding="utf-8")) if paths else {}


def command_run(name, args, extra=("--execute",), *, row=None, coils=None,
                now=None):
    """Enqueue one command, drive a short live run, and report what happened.

    Returns ``(fake_plc, fake_serial, exit_code, raised, published)``.
    """
    plc = FakePlc(dict(row if row is not None else ROW),
                  coils={C1: False} if coils is None else coils)
    line = FakeSerial()
    if now is None:
        channel.enqueue(name, args, source="test")
    else:
        channel.enqueue(name, args, source="test", now=now)
    code, raised = run(list(extra), plc, line)
    return plc, line, code, raised, latest_published()


def last_command(published) -> dict:
    return (published or {}).get("last_command") or {}


# ---------------------------------------------------------------------------
print("\n--- the hardware tripwire is armed before any run() call ---")
try:
    no_hardware.selftest()
    ok(True, "no_hardware blocks a real Modbus client and a raw TCP socket",
       no_hardware.describe())
except AssertionError as exc:
    ok(False, "TRIPWIRE IS NOT ARMED -- stopping", str(exc))
    sys.exit(1)


# ---------------------------------------------------------------------------
print("\n--- B1 unit: the stop handlers survive an INHERITED SIG_IGN ---")
# The defect exactly: a run launched as a background job from a non-interactive
# bash inherits SIGINT = SIG_IGN, and CPython leaves an INHERITED SIG_IGN alone
# -- it installs default_int_handler only over the inherited default. Observed
# live 2026-09-09: getsignal(SIGINT) was SIG_IGN and kill -INT was swallowed.
# So reproduce that starting state rather than testing from a clean one.
signal.signal(signal.SIGINT, signal.SIG_IGN)
signal.signal(signal.SIGTERM, signal.SIG_DFL)
ok(signal.getsignal(signal.SIGINT) is signal.SIG_IGN,
   "starting state reproduced: SIGINT is SIG_IGN, so kill -INT is swallowed and "
   "Python raises no KeyboardInterrupt")
previous = hold_environment._install_operator_stop_handlers()
ok(signal.getsignal(signal.SIGINT) is signal.default_int_handler,
   "after install SIGINT raises KeyboardInterrupt again, DESPITE having been "
   "inherited as SIG_IGN", repr(signal.getsignal(signal.SIGINT)))
term_handler = signal.getsignal(signal.SIGTERM)
ok(callable(term_handler),
   "and SIGTERM has a Python handler instead of the default disposition, which "
   "would kill the process without unwinding -- skipping the finally that "
   "disables the PID", repr(term_handler))
raised = None
try:
    term_handler(signal.SIGTERM, None)
except KeyboardInterrupt as exc:
    raised = exc
ok(isinstance(raised, KeyboardInterrupt),
   "invoking the SIGTERM handler raises KeyboardInterrupt -- the SAME path "
   "Ctrl-C takes, so it lands in _hold's except and then the fail-safe",
   repr(raised))
hold_environment._restore_stop_handlers(previous)
ok(signal.getsignal(signal.SIGINT) is signal.SIG_IGN,
   "restore put the inherited SIG_IGN back: run() does not leak its handlers to "
   "whatever called it")
signal.signal(signal.SIGINT, signal.default_int_handler)


# ---------------------------------------------------------------------------
print("\n--- B1 integration: a stop mid-loop ends ABORTED with the fail-safe run ---")


class StopOnSecondStep(hold_environment.env.Relay):
    """A relay whose 2nd ``step()`` is the operator's stop signal arriving.

    SUBCLASSED, not faked. The thing under test is the REAL ``Relay.safe()``
    running out of the ``finally``; a fake with its own ``safe()`` would assert
    nothing about the interlock.
    """

    steps = 0

    def step(self):
        type(self).steps += 1
        if type(self).steps >= 2:
            raise KeyboardInterrupt("test: a stop signal arrived on the 2nd step")
        return super().step()


plc = FakePlc(dict(ROW), coils={C1: True})
line = FakeSerial()
StopOnSecondStep.steps = 0
real_relay_class = hold_environment.env.Relay
hold_environment.env.Relay = StopOnSecondStep
try:
    code, raised = run(["--temp-sp", "24.0", "--execute"], plc, line)
finally:
    hold_environment.env.Relay = real_relay_class
ok(raised is None,
   "the KeyboardInterrupt was HANDLED, not propagated -- _hold caught it",
   repr(raised))
ok(code == 130, "run() returned 130, the interrupted exit code", repr(code))
ok(StopOnSecondStep.steps == 2,
   "the loop stopped ON the interrupted step and did not come round again",
   "steps=%d" % StopOnSecondStep.steps)
ok(plc.coils.get(C1) is False,
   "C1 (PID Auto) is DISABLED: the finally fail-safe ran on the abort path",
   "C1=%r" % plc.coils.get(C1))
ok(line.frames and abs(decode_setpoint(line.frames[-1]["values"]) - SAFE_C) < 1e-6,
   "and the declared safe setpoint is the last thing on the wire",
   "%r" % (decode_setpoint(line.frames[-1]["values"]) if line.frames else None))
manifest = latest_manifest()
ok(manifest.get("status") == runs.STATUS_ABORTED,
   "the run manifest records the run as %r, not as a clean finish"
   % runs.STATUS_ABORTED, repr(manifest.get("status")))


# ---------------------------------------------------------------------------
print("\n--- B2: the loop publishes EVERY step, where the dashboard reads ---")
plc = FakePlc(dict(ROW), coils={C1: False})
line = FakeSerial()
code, raised = run(["--execute"], plc, line)
published = latest_published()
ok(published is not None,
   "the loop published a reading at all. Before this, publish_last_reading was "
   "called only by the read helpers, so a 30-minute hold showed the chamber as "
   "of whenever someone last ran a read command")
ok("pid_enabled" in (published or {}),
   "the published record carries pid_enabled", repr((published or {}).get("pid_enabled")))
channels = (published or {}).get("channels") or {}
ok(abs(channels.get("rh_sp_pct", -1) - 95.0) < 1e-3,
   "and the decoded channels -- re-decoded from the record's own raw_registers, "
   "not re-read from the PLC", repr(channels.get("rh_sp_pct")))
ok(abs(channels.get("temp_pid_output_c", -1) - DF9_C) < 1e-6,
   "including DF9, byte-identical to what the relay forwarded: the published "
   "reading IS the reading the loop acted on", repr(channels.get("temp_pid_output_c")))
controller = (published or {}).get("controller") or {}
ok(controller.get("pid") == os.getpid(),
   "controller.pid names THIS process, so the dashboard can say who is holding "
   "the chamber and the operator knows what to signal", repr(controller.get("pid")))
ok(controller.get("run_dir") and Path(controller["run_dir"]).exists(),
   "controller.run_dir points at a run folder that exists",
   str(controller.get("run_dir")))
ok(isinstance(controller.get("started_unix_s"), float),
   "controller.started_unix_s is wall time, which means something in another "
   "process", repr(controller.get("started_unix_s")))
ok(controller.get("valve_mode") == "unavailable" and "Y001" in controller.get("valve_note", ""),
   "valve_mode is reported UNAVAILABLE with the reason, not invented as a state",
   repr(controller.get("valve_mode")))
age = (published or {}).get("age_s")
ok(isinstance(age, float) and age < 30.0,
   "age_s is small -- the record was written by the loop just now, not left "
   "over from a read", repr(age))
ok((published or {}).get("last_command") is None,
   "last_command is None when no command was issued, not an empty dict",
   repr((published or {}).get("last_command")))
readings = plc.count("read_holding_registers")
ok(readings <= plc.count("read_coils") + 1,
   "publishing added no extra block read: the reading is re-decoded from the "
   "relay's record, so the CLICK's three sockets are not spent twice per period",
   "read_holding_registers=%d read_coils=%d" % (readings, plc.count("read_coils")))


# ---------------------------------------------------------------------------
print("\n--- B3: a humidity setpoint command reaches DF6 as ONE FC16 of TWO words ---")
plc, line, code, raised, published = command_run("setpoint", {"rh_pct": 95.0},
                                                 row=ROW_RH80)
ok(raised is None, "the run completed", repr(raised))
writes = [call[1] for call in plc.calls_of("write_registers")
          if call[1]["address"] == DF6]
ok(len(writes) == 1 and writes[0]["values"] == RH95_WORDS,
   "exactly one write to DF6 (%d), of exactly two words %r -- the single-register "
   "float write is the defect this layer exists to prevent" % (DF6, RH95_WORDS),
   repr(writes))
ok([plc.registers.get(DF6), plc.registers.get(DF6 + 1)] == RH95_WORDS,
   "and DF6/%d hold them afterwards" % (DF6 + 1),
   repr([plc.registers.get(DF6), plc.registers.get(DF6 + 1)]))
ok(last_command(published).get("outcome") == channel.OUTCOME_CONFIRMED,
   "last_command reports CONFIRMED -- written AND read back word for word",
   repr(last_command(published)))
ok(last_command(published).get("name") == "setpoint",
   "and names the command it is about")
ok("--rh-sp" in last_command(published).get("detail", ""),
   "and its detail names the CLI flag whose path it took, because it is the "
   "same path", last_command(published).get("detail", "")[:60])


print("\n--- B3: pid enabled=False drives C1 False, asserted on the CALL LOG ---")
plc, line, code, raised, published = command_run("pid", {"enabled": False},
                                                 coils={C1: True})
coil_writes = [call[1] for call in plc.calls_of("write_coil")]
ok(coil_writes and coil_writes[0]["address"] == C1
   and coil_writes[0]["value"] is False,
   "the FIRST coil write of the run is the command's C1=False. Asserted on the "
   "call log deliberately: the finally fail-safe ALSO drives C1 False, so the "
   "end state alone would prove nothing about the command", repr(coil_writes[:2]))
ok(plc.coils.get(C1) is False, "and C1 ends disabled")
ok(last_command(published).get("outcome") == channel.OUTCOME_CONFIRMED,
   "last_command reports CONFIRMED", repr(last_command(published).get("detail", "")[:60]))


print("\n--- B3: a valve command is REFUSED, and says why ---")
plc, line, code, raised, published = command_run("valve", {"mode": "open"})
ok(last_command(published).get("outcome") == channel.OUTCOME_REFUSED,
   "outcome is refused -- accepted-and-ignored is how an operator comes to "
   "believe a valve moved", repr(last_command(published).get("outcome")))
ok("Y001" in last_command(published).get("detail", ""),
   "and the detail names the unverified coil rather than a generic refusal",
   last_command(published).get("detail", "")[:70])
ok(not plc.calls_of("write_registers"),
   "nothing was written for it", repr(plc.calls_of("write_registers")))


print("\n--- B3: a STALE command is never applied ---")
plc, line, code, raised, published = command_run(
    "setpoint", {"rh_pct": 20.0}, row=ROW_RH80,
    now=lambda: time.time() - (channel.STALE_AFTER_S + 30.0))
ok(last_command(published).get("outcome") == channel.OUTCOME_STALE,
   "outcome is stale, its own outcome and not a refusal: the operator should "
   "see that it EXPIRED, not that something rejected the value",
   repr(last_command(published).get("outcome")))
ok(not plc.calls_of("write_registers"),
   "NOTHING reached the wire -- an intent queued before a restart or a long "
   "stall must not fire afterwards", repr(plc.calls_of("write_registers")))
ok(abs(plc.value_at(6) - 80.0) < 1e-3,
   "and DF6 still holds what it held", repr(plc.value_at(6)))


print("\n--- B3: an out-of-bound setpoint is REFUSED, and the loop carries on ---")
plc, line, code, raised, published = command_run("setpoint", {"rh_pct": 150.0},
                                                 row=ROW_RH80)
ok(raised is None, "the run completed -- a bad command must not end a hold",
   repr(raised))
ok(last_command(published).get("outcome") == channel.OUTCOME_REFUSED,
   "outcome is refused", repr(last_command(published).get("outcome")))
ok("rh_sp_pct" in last_command(published).get("detail", ""),
   "the detail names the bound that refused it, not just 'invalid'",
   last_command(published).get("detail", "")[:70])
ok(not plc.calls_of("write_registers"),
   "and nothing reached the wire", repr(plc.calls_of("write_registers")))
ok(abs(plc.value_at(6) - 80.0) < 1e-3, "DF6 unchanged", repr(plc.value_at(6)))


print("\n--- B3: the rate limiter sees the CLI's own write ---")
# --temp-sp writes DF5 at the start of the run; a dashboard temperature command
# arriving milliseconds later is inside min_setpoint_interval_s (10 s) and must
# be refused. This only works because _write_setpoints seeds the same history
# the command path checks -- otherwise the first command of every run would be
# treated as "no previous write" and land on top of the CLI's.
plc, line, code, raised, published = command_run(
    "setpoint", {"temp_c": 25.0}, extra=("--temp-sp", "24.0", "--execute"))
ok(last_command(published).get("outcome") == channel.OUTCOME_REFUSED,
   "outcome is refused", repr(last_command(published).get("outcome")))
ok("RateLimited" in last_command(published).get("detail", ""),
   "and it was the RATE limiter, not the bound -- 25 C is perfectly legal",
   last_command(published).get("detail", "")[:70])
ok(abs(plc.value_at(5) - 24.0) < 1e-3,
   "DF5 still holds the CLI's 24.0, not the command's 25.0", repr(plc.value_at(5)))


print("\n--- B3: a string bool never gets as far as the channel file ---")
try:
    channel.enqueue("pid", {"enabled": "false"})
    ok(False, 'enqueue accepted the string "false" -- it is TRUTHY, so coercing '
              "it anywhere downstream would ENABLE the PID")
except channel.ChannelError as exc:
    ok(True, 'enqueue REFUSES the string "false" for pid.enabled, so it never '
             "reaches a file, let alone the wire", str(exc)[:60])
ok(not (channel.channel_dir() / channel.COMMAND_NAME).exists(),
   "and no command file was left behind by the refused enqueue")


print("\n--- B3: a hand-written malformed command.json refuses and is CLEARED ---")
path = channel.channel_dir() / channel.COMMAND_NAME
path.write_text('{"seq": 99, "issued_unix_s": %f, "name": "pid", '
                '"args": {"enabled": "false"}, "source": "hand"}' % time.time(),
                encoding="utf-8")
plc = FakePlc(dict(ROW), coils={C1: False})
line = FakeSerial()
code, raised = run(["--execute"], plc, line)
published = latest_published()
ok(raised is None, "the loop survived an unreadable command file", repr(raised))
ok(last_command(published).get("outcome") == channel.OUTCOME_REFUSED,
   "it is reported as refused", repr(last_command(published).get("outcome")))
ok(last_command(published).get("name") == "UNREADABLE",
   "named UNREADABLE with seq None, rather than claiming a seq that could not "
   "be parsed", repr(last_command(published).get("seq")))
ok(not path.exists(),
   "and the file was REMOVED. take() raises before it unlinks, so leaving it "
   "would re-refuse the same file every period for the rest of the run")
ok(coil_writes_true(plc) == 0,
   'the PID was never enabled by it -- "false" is truthy and this is the whole '
   "reason the channel refuses a non-bool", repr(plc.calls_of("write_coil")))


print("\n%s" % ("ALL PASS" if fails == 0 else "%d FAILURE(S)" % fails))
sys.exit(1 if fails else 0)
