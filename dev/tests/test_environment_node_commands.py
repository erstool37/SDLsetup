#!/usr/bin/env python3
"""The environment node's command routing: ask the running loop, or write directly.

No hardware, no socket, no vendor SDK, and nothing that could reach one. The
node's four wire paths -- ``client``, ``_transient_read``, ``_write`` and
``_set_pid`` -- are replaced before any command runs; in the *running* case the
first two raise :class:`AssertionError` on touch, so a routing mistake that sent
a command down the direct path fails here instead of contending for the CLICK's
third Modbus socket with a live run.

``plc.PUBLISH_DIR`` is redirected into a temporary directory, which is also
where :mod:`tools.environment.channel` puts ``command.json`` -- so this test
neither reads a real run's published reading nor queues a command a real
controller would then apply.

The defect class it guards is the one this routing exists for: while the
supervising loop holds the ``environment`` claim the dashboard *cannot* open a
session, so a command has to become a request. A version that kept the direct
path would raise ``occupancy.Busy`` at every click -- and one that kept the
direct path and somehow got a socket would write behind the loop's back.

    python dev/tests/test_environment_node_commands.py
"""
from __future__ import annotations

import json
import sys
import tempfile
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from dashboard import page as page_module  # noqa: E402
from tools import occupancy  # noqa: E402
from tools.environment import channel  # noqa: E402
from tools.environment import plc as plc_module  # noqa: E402
from tools.environment.node import EnvironmentNode  # noqa: E402
from tools.environment.plc import PlcSettings  # noqa: E402

fails = 0


def ok(condition: bool, message: str, detail: str = "") -> None:
    global fails
    print("[test] %s: %s%s" % ("PASS" if condition else "FAIL", message,
                               "  (%s)" % detail if detail else ""))
    if not condition:
        fails += 1


tmp = Path(tempfile.mkdtemp(prefix="sdl-node-cmd-"))
plc_module.PUBLISH_DIR = tmp

FAKE_CLAIM = occupancy.Claim(device="environment", doing="hold_environment",
                             pid=4242, since=time.time())
running = False


def fake_blockers(device):
    """Stands in for the real claim file, so no process has to be invented."""
    return [FAKE_CLAIM] if running else []


occupancy.blockers = fake_blockers


def node(*, allow_actuation: bool = True) -> EnvironmentNode:
    """A node whose every wire path is replaced. ``writes``/``pids`` record the
    direct ones; while a loop is running, touching one is the failure."""
    n = EnvironmentNode(settings=PlcSettings(allow_actuation=allow_actuation))
    n.writes = []
    n.pids = []

    def no_wire(*_a, **_k):
        raise AssertionError("must not reach the wire")

    def write(field, target):
        if running:
            no_wire()
        n.writes.append((field, target))
        return {"outcome": "confirmed", "ok": True, "field": field, "value": target}

    def set_pid(enabled):
        if running:
            no_wire()
        n.pids.append(enabled)
        return {"outcome": "confirmed", "ok": True, "pid_enabled": enabled}

    n.client = no_wire
    n._transient_read = no_wire
    n._write = write
    n._set_pid = set_pid
    return n


def clear_channel() -> None:
    for name in (channel.COMMAND_NAME, channel.SEQ_NAME):
        (tmp / name).unlink(missing_ok=True)


print("--- a controller is running: every command becomes a request ---")
running = True
clear_channel()
n = node()
res = n.command("set_setpoint", rh_pct=95)
ok(res.get("queued") is True and res.get("ok") is True,
   "set_setpoint(rh_pct=95) is queued, not written", repr(res))
ok(isinstance(res.get("seq"), int), "and the reply carries the seq to watch",
   repr(res.get("seq")))
pending = channel.peek()
ok(pending is not None and pending.name == "setpoint"
   and pending.args == {"rh_pct": 95.0},
   "the pending command is setpoint {'rh_pct': 95.0}",
   repr(None if pending is None else pending.args))
ok(n.writes == [] and n.pids == [], "nothing took the direct path",
   repr(n.writes + n.pids))

res = n.command("set_temperature", target_c=25.0)
ok(res.get("queued") is True and channel.peek().args == {"temp_c": 25.0},
   "set_temperature routes through the channel too, keyed temp_c",
   repr(channel.peek().args))

n.command("set_setpoint", temp_c=25.0, rh_pct=93.0)
ok(channel.peek().args == {"temp_c": 25.0, "rh_pct": 93.0},
   "both keys travel as ONE setpoint command", repr(channel.peek().args))

res = n.command("enable_pid")
ok(res.get("queued") is True and channel.peek().name == "pid"
   and channel.peek().args == {"enabled": True},
   "enable_pid is queued while the loop is watching", repr(res))

res = n.command("disable_pid")
ok(res.get("queued") is True and channel.peek().args == {"enabled": False},
   "disable_pid is queued -- the loop holds the only session", repr(res))

res = n.command("valve", mode="open")
ok(res.get("queued") is True and channel.peek().name == "valve"
   and channel.peek().args == {"mode": "open"},
   "valve(mode='open') is queued", repr(res))

print("\n--- a malformed request is refused, and queues nothing ---")
clear_channel()
n = node()
res = n.command("set_setpoint", rh_pct="95")
ok(res.get("ok") is False and "refused" in res,
   "a string setpoint is refused by the channel's shape check", repr(res))
ok("real number" in res.get("refused", ""), "and the refusal says why",
   res.get("refused", "")[:70])
ok(channel.peek() is None, "nothing was queued")
ok(n.writes == [] and n.pids == [], "and nothing was written")
res = n.command("valve", mode="ajar")
ok(res.get("ok") is False and "valve.mode" in res.get("refused", ""),
   "an unknown valve mode is refused", repr(res)[:80])

print("\n--- no controller: the direct paths, unchanged ---")
running = False
clear_channel()
n = node()
res = n.command("enable_pid")
ok(res.get("ok") is False and "start_kinetics" in res.get("refused", ""),
   "enable_pid is refused, naming what to start first",
   res.get("refused", "")[:70])
ok(channel.peek() is None, "and it queues nothing for a loop that is not there")

res = n.command("disable_pid")
ok(n.pids == [False] and res.get("ok") is True,
   "disable_pid takes the direct _set_pid path -- off is always allowed",
   repr(res))

res = n.command("set_setpoint", temp_c=25.0, rh_pct=93.0)
ok(n.writes == [("temp_sp_c", 25.0), ("rh_sp_pct", 93.0)],
   "set_setpoint writes each present key through _write", repr(n.writes))
ok(set(res.get("results") or {}) == {"temp_sp_c", "rh_sp_pct"},
   "and returns both WriteResults, keyed by field", repr(res)[:90])

n = node()
res = n.command("set_temperature", target_c=25.3)
ok(n.writes == [("temp_sp_c", 25.3)] and res.get("outcome") == "confirmed",
   "set_temperature still writes temp_sp_c directly", repr(res)[:80])

res = n.command("valve", mode="auto")
ok(res.get("ok") is False and "no controller is running" in res.get("refused", ""),
   "valve is refused with no loop to change the mode",
   res.get("refused", "")[:70])

res = n.command("set_setpoint")
ok(res.get("ok") is False and "temp_c and/or rh_pct" in res.get("refused", ""),
   "set_setpoint with neither key refuses rather than guessing one",
   res.get("refused", "")[:70])

print("\n--- status surfaces the controller's blocks ---")
running = True
clear_channel()
(tmp / plc_module.PUBLISH_NAME).write_text(json.dumps({
    "published_unix_s": time.time(),
    "pid_enabled": True,
    "channels": {"temp_filtered_c": 24.9, "rh_filtered_pct": 92.4,
                 "temp_sp_c": 25.0, "rh_sp_pct": 93.0,
                 "temp_pid_output_c": 26.1, "rh_pid_output_pct": 41.5},
    "controller": {"pid": 4242, "run_dir": "/home/lamp/SDLsetup/dataset/x",
                   "started_unix_s": time.time() - 60,
                   "valve_mode": "unavailable",
                   "valve_note": "the ladder owns the valve; no mode register"},
    "last_command": {"seq": 7, "name": "pid", "args": {"enabled": True},
                     "source": "dashboard", "outcome": "confirmed",
                     "detail": "coil read back True",
                     "finished_unix_s": time.time()},
}, indent=1), encoding="utf-8")
channel.enqueue("setpoint", {"rh_pct": 95.0})
st = node().status()
ok(st.get("controller_running") is True,
   "controller_running is the occupancy claim, not a published boast",
   repr(st.get("controller_running")))
ok((st.get("controller") or {}).get("pid") == 4242,
   "the controller block is surfaced whole", repr(st.get("controller")))
ok((st.get("last_command") or {}).get("outcome") == "confirmed",
   "so is last_command", repr(st.get("last_command")))
pc = st.get("pending_command") or {}
ok(pc.get("name") == "setpoint" and pc.get("args") == {"rh_pct": 95.0}
   and isinstance(pc.get("age_s"), float),
   "pending_command reports the unconsumed request and its age", repr(pc))
ok(st.get("pid_enabled") is True and st.get("temp_sp_c") == 25.0
   and st.get("rh_sp_pct") == 93.0 and st.get("temp_pid_output_c") == 26.1
   and st.get("rh_pid_output_pct") == 41.5,
   "and the existing published fields are untouched")
ok(st.get("state") == "online", "a fresh record is online", repr(st.get("state")))
ok(channel.peek() is not None, "status peeked -- it did not consume the request")

(tmp / plc_module.PUBLISH_NAME).write_text(json.dumps({
    "published_unix_s": time.time() - 600, "pid_enabled": True, "channels": {},
    "controller": {"pid": 4242, "run_dir": "/x", "started_unix_s": 0,
                   "valve_mode": "unavailable", "valve_note": ""},
}, indent=1), encoding="utf-8")
st = node().status()
ok(st.get("state") == "offline",
   "a controller block does not make a stale record fresh", repr(st.get("state")))

(tmp / channel.COMMAND_NAME).write_text("{not json", encoding="utf-8")
st = node().status()
ok((st.get("pending_command") or {}).get("error"),
   "an unreadable command.json is reported in the card, not raised",
   repr(st.get("pending_command")))
ok(st.get("controller_running") is True, "and the rest of the card still renders")

print("\n--- the page carries the elements this routing needs ---")
for element in ("cc-ctrl-run", "cc-pid-on", "cc-pid-off", "cc-sp-temp", "cc-sp-rh",
                "cc-sp-apply", "cc-preset-paper", "cc-preset-95", "cc-valve-auto",
                "cc-valve-open", "cc-valve-close", "cc-valve-note", "cc-pending",
                "cc-cmd-msg"):
    ok('id="%s"' % element in page_module.PAGE, "page.PAGE defines #%s" % element)
ok("/command/environment/" in page_module.PAGE,
   "and posts to the environment node's command endpoint")
# A browser dialog blocks the page and cannot be driven from automation, so the
# PID confirm is a two-click arm. Guard the ban, not just today's markup.
for banned in ("window.confirm", "window.alert", "window.prompt"):
    ok(banned not in page_module.PAGE, "no %s in the page" % banned)

print("\n%s" % ("ALL PASS" if fails == 0 else "%d FAILURE(S)" % fails))
sys.exit(1 if fails else 0)
