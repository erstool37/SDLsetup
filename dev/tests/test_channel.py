#!/usr/bin/env python3
"""tools/environment/channel.py -- the dashboard->controller command file.

This module is the ONLY thing standing between a dashboard click and a guarded
write, and everything it does is shape validation and an atomic file move. So
the interesting cases are the malformed ones: a bool where a float belongs
(``bool`` is an ``int`` subclass, so ``float(True)`` is 1.0 and looks like a
setpoint), the string ``"false"`` for a bool (truthy, and it would ENABLE the
PID), a NaN (every comparison against it is False, so a bound-only check passes
it), and an unconsumed command that is still sitting there after a restart.

No hardware and no live state: ``no_hardware`` moves ``plc.PUBLISH_DIR`` -- which
is what ``channel_dir()`` resolves -- into a throwaway directory, so this never
touches the file the running dashboard reads.

    python dev/tests/test_channel.py
"""
from __future__ import annotations

import json
import sys
import time

import no_hardware  # noqa: F401  -- MUST be first: it redirects PUBLISH_DIR

from tools.environment import channel  # noqa: E402

fails = 0


def ok(condition: bool, message: str, detail: str = "") -> None:
    global fails
    print("[test] %s: %s%s" % ("PASS" if condition else "FAIL", message,
                               "  (%s)" % detail if detail else ""))
    if not condition:
        fails += 1


def refuses(name, args, why: str) -> None:
    """validate() must raise ChannelError -- not coerce, not pass."""
    try:
        channel.validate(name, args)
    except channel.ChannelError as exc:
        ok(True, "REFUSED %s %r -- %s" % (name, args, why), str(exc)[:70])
    except Exception as exc:  # noqa: BLE001
        ok(False, "%s %r refused with the WRONG type" % (name, args),
           "%s: %s" % (type(exc).__name__, exc))
    else:
        ok(False, "%s %r was ACCEPTED and should not be (%s)" % (name, args, why))


class Clock:
    """An injected clock, so staleness is asserted rather than slept through."""

    def __init__(self, t: float = 1000.0) -> None:
        self.t = float(t)

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += float(dt)


print("\n--- validate() refuses every malformed shape ---")
refuses("setpoint", {"temp_c": True},
        "bool is an int subclass, so float(True)=1.0 would look like a setpoint")
refuses("setpoint", {"rh_pct": False}, "same, and False would read as 0 %RH")
refuses("setpoint", {"temp_c": float("nan")},
        "every comparison with NaN is False, so a bound-only check passes it")
refuses("setpoint", {"rh_pct": float("inf")}, "inf is not a setpoint")
refuses("setpoint", {"temp_c": "24"}, "a string is not a number here")
refuses("setpoint", {}, "at least one of temp_c/rh_pct is required")
refuses("setpoint", {"temp_c": 24.0, "rh": 90.0},
        "an unexpected key is a typo, and a silently dropped key is a setpoint "
        "the operator believes was applied")
refuses("pid", {"enabled": "false"},
        "the string 'false' is TRUTHY -- coercing it would ENABLE the PID")
refuses("pid", {"enabled": 1}, "the int 1 is not a real bool")
refuses("pid", {"enabled": None}, "absent is not False")
refuses("pid", {}, "enabled is required")
refuses("valve", {"mode": "half"}, "not one of %s" % (channel.VALVE_MODES,))
refuses("valve", {"mode": True}, "a bool is not a valve mode")
refuses("purge", {}, "unknown command name")
refuses("setpoint", ["temp_c", 24.0], "args must be a mapping")

print("\n--- validate() accepts and NORMALISES the good shapes ---")
clean = channel.validate("setpoint", {"temp_c": 24, "rh_pct": None})
ok(clean == {"temp_c": 24.0}, "an int becomes a float and an explicit None is "
                              "dropped, not written as a setpoint", repr(clean))
ok(channel.validate("pid", {"enabled": False}) == {"enabled": False},
   "pid enabled=False survives")
ok(channel.validate("valve", {"mode": "auto"}) == {"mode": "auto"},
   "a valid valve mode survives")


print("\n--- enqueue -> peek -> take round-trips, and take() does not replay ---")
path = channel.channel_dir() / channel.COMMAND_NAME
first = channel.enqueue("setpoint", {"rh_pct": 95.0}, source="test")
ok(path.exists(), "enqueue wrote %s" % channel.COMMAND_NAME, str(path))
seen = channel.peek()
ok(seen is not None and seen.seq == first.seq and seen.args == {"rh_pct": 95.0},
   "peek returns the pending command without consuming it", repr(seen))
ok(path.exists(), "and the file is still there after peek -- peek is not take")
taken = channel.take()
ok(taken is not None and taken.seq == first.seq,
   "take returns the same command", repr(taken))
ok(not path.exists(),
   "the file is GONE once take() returned, so a crash between take and apply "
   "cannot replay the command on restart")
ok(channel.take() is None, "a second take finds nothing pending")
ok(channel.peek() is None, "and neither does peek")


print("\n--- a second enqueue OVERWRITES an unconsumed first; seq is monotonic ---")
one = channel.enqueue("setpoint", {"temp_c": 24.0})
two = channel.enqueue("pid", {"enabled": False})
pending = channel.take()
ok(pending is not None and pending.name == "pid",
   "the LATER command is the one waiting -- the operator's latest intent wins",
   repr(pending and pending.name))
ok(channel.take() is None,
   "and the overwritten one is not queued behind it: there is exactly one slot")
ok(two.seq == one.seq + 1, "seq incremented across the two enqueues",
   "%d -> %d" % (one.seq, two.seq))
three = channel.enqueue("valve", {"mode": "auto"})
ok(three.seq > two.seq, "and again for a third, even though one was consumed",
   "%d -> %d" % (two.seq, three.seq))
channel.take()


print("\n--- is_stale, against an injected clock ---")
clock = Clock()
fresh = channel.enqueue("pid", {"enabled": False}, now=clock)
ok(channel.is_stale(fresh, now=clock) is False,
   "a command taken at the instant it was issued is not stale")
clock.advance(channel.STALE_AFTER_S - 0.5)
ok(channel.is_stale(fresh, now=clock) is False,
   "nor is one taken just inside STALE_AFTER_S=%g" % channel.STALE_AFTER_S)
clock.advance(1.0)
ok(channel.is_stale(fresh, now=clock) is True,
   "past STALE_AFTER_S it IS stale -- a queued intent must not fire after a "
   "restart or a long stall")
backwards = Clock(fresh.issued_unix_s - 10.0)
ok(channel.is_stale(fresh, now=backwards) is True,
   "a NEGATIVE age is stale too: a command from the future means the clock is "
   "unreliable, and an unknown age is not a fresh one")
channel.take()


print("\n--- outcome_record carries the command and refuses an unknown outcome ---")
cmd = channel.enqueue("setpoint", {"temp_c": 24.5}, source="dashboard")
record = channel.outcome_record(cmd, channel.OUTCOME_CONFIRMED, "written and read back")
ok(record["seq"] == cmd.seq and record["name"] == "setpoint",
   "the record carries the command it is about")
ok(record["args"] == {"temp_c": 24.5}, "and its args", repr(record["args"]))
ok(record["outcome"] == "confirmed" and record["detail"] == "written and read back",
   "and the outcome and detail")
ok(isinstance(record["finished_unix_s"], float),
   "and when it finished", repr(record.get("finished_unix_s")))
ok(record["source"] == "dashboard", "and who asked")
for bad in ("ok", "CONFIRMED", "done", "", None, True):
    try:
        channel.outcome_record(cmd, bad, "x")
        ok(False, "outcome %r should be refused" % (bad,))
    except channel.ChannelError:
        ok(True, "outcome %r refused -- the dashboard renders these, so an "
                 "unknown one is a blank pill, not an error" % (bad,))
channel.take()


print("\n--- a hand-written command.json that is malformed raises on take() ---")
# The dashboard cannot produce these -- enqueue validates first -- but an
# operator with an editor can, and so can an older dashboard build.
for text, why in (
        ('{"seq": 1, "issued_unix_s": 1.0, "name": "pid", '
         '"args": {"enabled": "false"}, "source": "hand"}',
         'the string "false" would ENABLE the PID if it were coerced'),
        ('{"seq": 1, "issued_unix_s": 1.0, "name": "purge", "args": {}}',
         "an unknown command name"),
        ('{"name": "pid", "args": {"enabled": false}}', "no seq, no timestamp"),
        ('not json at all', "not JSON"),
        ('[1, 2, 3]', "JSON, but not an object")):
    path.write_text(text, encoding="utf-8")
    try:
        channel.take()
        ok(False, "a malformed command.json was ACCEPTED (%s)" % why)
    except channel.ChannelError as exc:
        ok(True, "malformed command.json raises ChannelError -- %s" % why,
           str(exc)[-60:])
    except Exception as exc:  # noqa: BLE001
        ok(False, "malformed command.json raised the WRONG type (%s)" % why,
           "%s: %s" % (type(exc).__name__, exc))
    path.unlink(missing_ok=True)


print("\n--- the file itself: atomic, JSON, and no temp files left behind ---")
made = channel.enqueue("setpoint", {"temp_c": 24.0, "rh_pct": 93.0})
payload = json.loads(path.read_text(encoding="utf-8"))
ok(set(payload) == {"seq", "issued_unix_s", "name", "args", "source"},
   "the on-disk keys are exactly the Command fields", repr(sorted(payload)))
ok(payload["args"] == {"temp_c": 24.0, "rh_pct": 93.0},
   "both setpoint keys survive the round trip")
ok(abs(payload["issued_unix_s"] - time.time()) < 30.0,
   "issued_unix_s is wall time, which is what the consumer ages it against")
leftovers = sorted(p.name for p in channel.channel_dir().glob("*.tmp"))
ok(not leftovers, "no .tmp files left in the channel directory", repr(leftovers))
ok(made.seq > 0, "seq is positive")
channel.take()


print("\n%s" % ("ALL PASS" if fails == 0 else "%d FAILURE(S)" % fails))
sys.exit(1 if fails else 0)
