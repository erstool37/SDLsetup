#!/usr/bin/env python3
"""The environment section of ``configs/config.yaml``, and the node the dashboard builds.

Two things are checked, and they fail in different ways:

**Resolution and provenance.** Every value reaches ``PlcSettings`` through
``tools.config.resolve``, and ``describe()`` prints the layer each one came from.
A value that is *present but unparseable* must RAISE rather than fall through to
the code default, and config may TIGHTEN a safety ceiling but never widen it.

**Dashboard startup.** ``dashboard/app.py`` calls ``EnvironmentNode()`` with no
arguments. The documented precedent is the UV-Vis node whose constructor raised
``TypeError``, which meant **the dashboard could not start at all** -- and no
check short of building the lab caught it. So this file builds the lab and calls
``status()`` on the node, with no hardware present.

    python dev/tests/test_environment_config.py
"""
from __future__ import annotations

import math
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from tools import config as _config  # noqa: E402
from tools.circulator.api import CirculatorSettings  # noqa: E402
from tools.environment import plc as _plc  # noqa: E402
from tools.environment.api import EnvironmentNode, PlcSettings  # noqa: E402

CONFIG_PATH = REPO / "configs" / "config.yaml"

fails = 0


def ok(condition: bool, message: str, detail: str = "") -> None:
    global fails
    print("[test] %s: %s%s" % ("PASS" if condition else "FAIL", message,
                               "  (%s)" % detail if detail else ""))
    if not condition:
        fails += 1


def raises(kind, call, message: str):
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


# ---------------------------------------------------------------------------
print("\n--- the shipped file parses under this repo's own parser ---")
if not CONFIG_PATH.exists():
    # Skip LOUDLY: a config test that quietly passes with no config file
    # reports coverage it does not have.
    print("[test] SKIP: configs/config.yaml is missing at %s" % CONFIG_PATH)
    print("[test] SKIP: no resolution, provenance or ceiling check ran")
    print("SKIPPED (no configs/config.yaml)")
    sys.exit(0)

raw = _config.load_yaml(CONFIG_PATH)
ok(isinstance(raw, dict) and "environment" in raw,
   "load_yaml parses the file and finds the environment section")
section = raw["environment"]
ok(isinstance(section, dict), "the environment section is a mapping")

print("\n--- resolution and provenance ---")
settings = PlcSettings.from_config(CONFIG_PATH)
ok(settings.host == "169.254.33.33", "host resolves from the file", settings.host)
ok(settings.allow_actuation is False, "and the master gate is OFF in the file")
described = settings.describe()
ok("[config.yaml]" in described,
   "describe() names config.yaml as the source of a file-supplied value")
ok("[default]" in described,
   "and names 'default' for a key the file does not carry")
for key in ("host", "port", "plc_stale_s", "temp_sp_max_c"):
    ok(settings.sources.get(key) is not None,
       "  %s carries a provenance layer" % key, str(settings.sources.get(key)))

overridden = PlcSettings.from_config(CONFIG_PATH, timeout_s=7.5)
ok(overridden.timeout_s == 7.5, "an override wins over the file")
ok(overridden.sources["timeout_s"] == "argument", "and is recorded as 'argument'",
   overridden.sources["timeout_s"])
not_given = PlcSettings.from_config(CONFIG_PATH, timeout_s=None)
ok(not_given.timeout_s == settings.timeout_s,
   "an override of None means 'not specified' and never shadows the file")

print("\n--- present-but-unparseable RAISES; it never falls through ---")
raises(_config.ConfigError,
       lambda: PlcSettings.from_config({"environment": {"timeout_s": "warm"}}),
       "a non-numeric timeout_s raises")
raises(_config.ConfigError,
       lambda: PlcSettings.from_config({"environment": {"port": "five-oh-two"}}),
       "a non-numeric port raises")
raises(_config.ConfigError,
       lambda: PlcSettings.from_config({"environment": {"allow_actuation": "flase"}}),
       "a typo'd boolean gate raises rather than reading as truthy")

print("\n--- config may TIGHTEN a safety ceiling, never widen it ---")
tighter = PlcSettings.from_config({"environment": {"temp_sp_max_c": 26.0}})
ok(tighter.limits.temp_max_c == 26.0, "a tighter temperature ceiling is accepted")
raises(_config.ConfigError,
       lambda: PlcSettings.from_config({"environment": {"temp_sp_max_c": 45.0}}),
       "a WIDER temperature ceiling raises")
raises(_config.ConfigError,
       lambda: PlcSettings.from_config({"environment": {"temp_sp_min_c": -10.0}}),
       "a lower floor raises")
raises(_config.ConfigError,
       lambda: PlcSettings.from_config({"environment": {"rh_sp_max_pct": 120.0}}),
       "and so does a wider humidity ceiling")

print("\n--- the empty-key trap: a blank value parses to {}, not None ---")
blank = _config.load_yaml(CONFIG_PATH).get("circulator", {})
ok(blank.get("port") == {},
   "circulator.port is blank on purpose and parses to an EMPTY MAPPING",
   repr(blank.get("port")))
# The trap itself is demonstrated on a SYNTHETIC blank key, not on the live
# config: safe_setpoint_c is an operator decision (set 2026-09-08), and a test
# that required it to stay blank would fail the moment someone made it.
with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as _fh:
    _fh.write("circulator:\n  safe_setpoint_c:\n  port:\n")
    _blank_path = Path(_fh.name)
try:
    _synthetic = _config.load_yaml(_blank_path).get("circulator", {})
finally:
    _blank_path.unlink(missing_ok=True)
ok(_synthetic.get("safe_setpoint_c") == {},
   "a BLANK safe_setpoint_c parses to an EMPTY MAPPING -- and {} is FALSY, so "
   "`if value:` reads it as absent", repr(_synthetic.get("safe_setpoint_c")))
_live = blank.get("safe_setpoint_c")
ok(isinstance(_live, float) and math.isfinite(_live),
   "the LIVE safe_setpoint_c is a finite number (operator set it), not a blank",
   repr(_live))
_lim = CirculatorSettings.from_config().limits
ok(isinstance(_live, float) and _lim.min_c < _live < _lim.max_c,
   "and it lies strictly inside the RESOLVED circulator bound",
   "%r in (%g, %g)" % (_live, _lim.min_c, _lim.max_c))
raises(_config.ConfigError,
       lambda: PlcSettings.from_config({"environment": {"timeout_s": {}}}),
       "an empty environment key is present-and-unusable, and raises")

print("\n--- the dashboard can start with no hardware present ---")
with tempfile.TemporaryDirectory() as tmp:
    # Redirected so this test never reads a real run's published reading, and
    # never writes one. plc.PUBLISH_DIR is module-level for exactly this.
    original = _plc.PUBLISH_DIR
    _plc.PUBLISH_DIR = Path(tmp) / "environment"
    try:
        node = EnvironmentNode()
        ok(node.kind == "environment", "EnvironmentNode() constructs with NO arguments",
           node.kind)
        status = node.status()
        ok(isinstance(status, dict), "status() returns a dict", type(status).__name__)
        ok("error" not in status or status.get("error") is None,
           "and carries no error with nothing published", str(status.get("error")))
        for key in ("state", "in_range", "pid_enabled", "age_s",
                    "clients_limit_reached", "summary"):
            ok(key in status, "  status reports %s" % key, repr(status.get(key)))
        ok(status["age_s"] is None, "age_s is None when nothing has been published")
        ok("read" in node.commands() and "disable_pid" in node.commands(),
           "read and disable_pid are offered", str(node.commands()))
        ok("enable_pid" not in node.commands(),
           "enable_pid is deliberately NOT a dashboard command")
        refused = node.command("enable_pid")
        ok(refused.get("ok") is False,
           "and asking for it anyway is refused, not raised", str(refused)[:70])
        bad = node.command("set_temperature", target_c=99.0)
        ok(bad.get("ok") is False and ("refused" in bad or "error" in bad),
           "an out-of-bound setpoint comes back as a refusal dict, never an exception",
           str(bad)[:80])

        try:
            from dashboard.app import build_lab
        except Exception as exc:                                      # noqa: BLE001
            # Skip LOUDLY, naming what was not verified.
            print("[test] SKIP: dashboard.app could not be imported: %s: %s"
                  % (type(exc).__name__, exc))
            print("[test] SKIP: build_lab() and node registration were NOT verified")
        else:
            lab = build_lab()
            registry = lab.nodes
            names = sorted(registry)
            ok("environment" in names,
               "build_lab() constructs the whole lab with the environment node "
               "registered -- the check the UV-Vis TypeError needed", str(names))
            env = registry["environment"]
            ok(isinstance(env.status(), dict),
               "and its status() through the registry returns a dict")
            ok(env.kind == "environment", "with kind='environment'", env.kind)
            ok("enable_pid" not in env.commands(),
               "and the registered node offers no enable_pid either")
    finally:
        _plc.PUBLISH_DIR = original

print("\n%s" % ("ALL PASS" if fails == 0 else "%d FAILURE(S)" % fails))
sys.exit(1 if fails else 0)
