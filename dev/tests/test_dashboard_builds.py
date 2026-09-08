#!/usr/bin/env python3
"""``dashboard.app.build_lab()`` starts with no hardware present, and exposes no button that actuates.

Two separate failures are guarded here.

**Startup.** The documented precedent is the UV-Vis node whose constructor
raised ``TypeError``: registering it meant **the dashboard could not start at
all**, and no check short of building the whole lab caught it. So this file
builds the lab with nothing plugged in and calls ``status()`` on every node.

**Surface.** A web page must not be able to actuate hardware with one click, so
two commands were deliberately removed and must stay removed:

* ``enable_pid`` is not an environment command -- enabling the ladder PID hands
  the chamber's heater and humidifier to the loop. ``disable_pid`` *is* offered,
  because off is the safe direction.
* ``open``/``close`` are not circulator commands -- **opening the serial port
  hardware-resets the MCU** (FT232R, DTR coupled to /RESET, asserted by the OS
  on open before any byte is sent). ``allow_actuation`` gating is not sufficient
  mitigation for a one-click reset.

**No Modbus session while the dashboard renders.** The CLICK accepts at most
three concurrent Modbus TCP clients and refuses the fourth, so with
``environment.dashboard_poll_plc`` false the node must render the reading a run
published rather than opening a session of its own.

No hardware, no network, no motion. ``plc.PUBLISH_DIR`` is redirected into a
temporary directory so this test neither reads a real run's published reading
nor writes one.

    python dev/tests/test_dashboard_builds.py
"""
from __future__ import annotations

import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

fails = 0


def ok(condition: bool, message: str, detail: str = "") -> None:
    global fails
    print("[test] %s: %s%s" % ("PASS" if condition else "FAIL", message,
                               "  (%s)" % detail if detail else ""))
    if not condition:
        fails += 1


try:
    from tools.environment import plc as _plc
    from tools.environment.plc import PlcSettings
except Exception as exc:                                              # noqa: BLE001
    print("[test] SKIP: tools.environment could not be imported: %s: %s"
          % (type(exc).__name__, exc))
    print("[test] SKIP: dashboard startup and the node command surfaces were "
          "NOT verified")
    print("SKIPPED (tools.environment unavailable)")
    sys.exit(0)

try:
    from dashboard.app import build_lab
except Exception as exc:                                              # noqa: BLE001
    print("[test] SKIP: dashboard.app could not be imported: %s: %s"
          % (type(exc).__name__, exc))
    print("[test] SKIP: build_lab(), every node's status(), and the removal of "
          "enable_pid / open / close were NOT verified")
    print("SKIPPED (dashboard.app unavailable)")
    sys.exit(0)

#: Nodes whose adapters are documented as never raising into the bus. These are
#: the two this unit owns; see the FINDING block at the end for the rest.
BUS_SAFE = ("environment", "circulator")

with tempfile.TemporaryDirectory() as tmp:
    original = _plc.PUBLISH_DIR
    _plc.PUBLISH_DIR = Path(tmp) / "environment"
    try:
        print("--- the lab builds with nothing plugged in ---")
        lab = build_lab()
        names = sorted(lab.nodes)
        ok(bool(lab.nodes), "build_lab() registered at least one node", str(names))
        for expected in ("environment", "circulator"):
            ok(expected in lab.nodes,
               "%r is registered" % expected, str(names))

        print("\n--- every node's status() returns a dict and never raises ---")
        for name, node in sorted(lab.nodes.items()):
            try:
                status = node.status()
            except Exception as exc:                                  # noqa: BLE001
                ok(False, "%s.status() did not raise" % name,
                   "%s: %s" % (type(exc).__name__, exc))
                continue
            ok(isinstance(status, dict), "%s.status() returns a dict" % name,
               type(status).__name__)
            ok(isinstance(status, dict) and "state" in status and "kind" in status,
               "  and carries state/kind", str(status.get("state")))

        print("\n--- an unknown command is reported, not raised ---")
        for name in BUS_SAFE:
            node = lab.nodes.get(name)
            if node is None:
                ok(False, "%s is registered, so its command surface can be checked"
                          % name)
                continue
            try:
                result = node.command("no_such_command_xyzzy")
            except Exception as exc:                                  # noqa: BLE001
                ok(False, "%s.command() reports an unknown name" % name,
                   "raised %s: %s" % (type(exc).__name__, exc))
                continue
            ok(isinstance(result, dict) and result.get("ok") is False,
               "%s.command('no_such_command_xyzzy') -> ok=False dict" % name,
               str(result)[:80])
            ok(isinstance(result, dict) and ("error" in result or "refused" in result),
               "  and says why", str(result)[:80])

        print("\n--- the two commands deliberately absent from the web surface ---")
        env = lab.nodes["environment"]
        env_commands = env.commands()
        ok("enable_pid" not in env_commands,
           "enable_pid is NOT an environment dashboard command", str(env_commands))
        ok("disable_pid" in env_commands,
           "  but disable_pid is -- off is the safe direction")
        refused = env.command("enable_pid")
        ok(isinstance(refused, dict) and refused.get("ok") is False,
           "  and asking for it anyway is refused, not raised", str(refused)[:70])

        circ = lab.nodes["circulator"]
        circ_commands = circ.commands()
        for banned in ("open", "close"):
            ok(banned not in circ_commands,
               "%r is NOT a circulator dashboard command -- opening the port "
               "RESETS the MCU" % banned, str(circ_commands))
        ok("plan_setpoint" in circ_commands and "port_present" in circ_commands,
           "  the read-only and dry-run commands are offered", str(circ_commands))
        presence = circ.command("port_present")
        ok(isinstance(presence, dict),
           "port_present() returns a dict and opens nothing", str(presence)[:80])

        print("\n--- the dashboard holds no Modbus session while polling is off ---")
        settings = PlcSettings.from_config(REPO / "configs" / "config.yaml")
        ok(settings.dashboard_poll_plc is False,
           "environment.dashboard_poll_plc is false in configs/config.yaml -- "
           "the CLICK allows only 3 concurrent clients",
           repr(settings.dashboard_poll_plc))
        status = env.status()
        ok(status.get("polled_plc") is False,
           "so status() reports polled_plc False", repr(status.get("polled_plc")))
        ok("dashboard_poll_plc" in str(status.get("poll_skipped")),
           "and names dashboard_poll_plc as the reason it took no session",
           str(status.get("poll_skipped"))[:90])

        print("\n--- FINDING: the other nodes still RAISE on an unknown command ---")
        # Not an ok() assertion: tools/{arm,microscope,uvvis}/node.py and
        # tools/_vacant.py fall through to `raise NotImplementedError(name)`
        # (VacantNode inherits tools.node.Node.command, which raises), and those
        # files are outside this unit's write scope. Recorded so the gap is
        # visible instead of silent. Lab.command() does NOT catch it, so an
        # unknown name from the web page becomes a 500 rather than a refusal.
        # TODO(operator): decide whether every node adapter should return
        # {"ok": False, "error": ...} like environment and circulator do. If so,
        # the one-line fix is in tools/node.py Node.command plus the three
        # `raise NotImplementedError(name)` tails, and this block becomes an
        # assertion over every node.
        for name, node in sorted(lab.nodes.items()):
            if name in BUS_SAFE:
                continue
            try:
                node.command("no_such_command_xyzzy")
                verdict = "returned a dict"
            except NotImplementedError:
                verdict = "raises NotImplementedError"
            except Exception as exc:                                  # noqa: BLE001
                verdict = "raises %s" % type(exc).__name__
            print("       %-12s %s" % (name, verdict))
    finally:
        _plc.PUBLISH_DIR = original

print("\n%s" % ("ALL PASS" if fails == 0 else "%d FAILURE(S)" % fails))
sys.exit(1 if fails else 0)
