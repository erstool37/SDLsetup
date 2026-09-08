#!/usr/bin/env python3
"""
check_environment.py -- a zero-actuation, one-call status read of the chamber.

WHAT IT DOES
  One READ-ONLY Modbus block read of the CLICK PLC, returned and printed:
  current temperature and humidity (raw AND filtered), both setpoints, whether
  the PID loop is on, and the standing note on what this layer cannot confirm.

    from scripts.environment.check_environment import check_environment
    state = check_environment()
    print(state["temp_filtered_c"], state["rh_filtered_pct"], state["pid_enabled"])

  Or from the shell:

    python scripts/environment/check_environment.py
    python scripts/environment/check_environment.py --json

NO ACTUATION, EVER
  There is no write path in this file -- not a setpoint, not a coil, not a PID
  toggle -- and the circulator's serial port is never opened (opening it
  hardware-resets the MCU). This is the status read; `start_kinetics.py` beside
  it is the full-run entry point. It claims `occupancy("environment")` for the
  read because the CLICK accepts at most three concurrent Modbus TCP clients.

`pid_enabled is None` is not `False`
  ``None`` means the C1 coil could not be read -- "we do not know whether the
  loop is closed" -- which is a different fact from "the loop is open".
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from tools import environment as env  # noqa: E402
from tools import occupancy  # noqa: E402

DEVICE = "environment"

#: What a status read cannot establish, carried in the returned dict so no
#: number is mistaken for something better-attested than it is.
UNKNOWN_NOTE = (
    "READ-ONLY status. Channel identity beyond DF1/DF2 is UNCONFIRMED (this "
    "decodes through tools.environment.registers, which it cannot corroborate). "
    "DF3/DF4 MEANING and the SC90-SC95/SD41 native diagnostics' Modbus ADDRESSES "
    "are UNKNOWN and were not guessed. pid_enabled=None means C1 could not be "
    "read -- 'unknown', not 'off'. Nothing about the circulator is read here.")


def check_environment(config: Any = None, *, _client: Any = None) -> dict:
    """Read the chamber once and return its state. Never actuates.

    ``_client`` is an internal seam for tests to inject a fake-backed
    :class:`tools.environment.api.PlcClient`; production callers leave it
    ``None`` and one short read-only session is opened and closed.
    """
    cfg = config if config is not None else str(REPO / "configs" / "config.yaml")
    settings = env.PlcSettings.from_config(cfg)

    with occupancy.claim(DEVICE, doing="check_environment (read-only)"):
        owned = _client is None
        plc = _client if _client is not None else env.PlcClient(settings)
        try:
            plc.connect()
            reading = plc.read_block()
            last_error = getattr(plc, "last_error", None)
        finally:
            if owned:
                plc.close()

    channels = reading.channels
    return {
        "temp_raw_c": channels.get("temp_raw_c"),
        "temp_filtered_c": reading.temp_filtered_c,
        "rh_raw_pct": channels.get("rh_raw_pct"),
        "rh_filtered_pct": reading.rh_filtered_pct,
        "temp_sp_c": reading.temp_sp_c,
        "rh_sp_pct": reading.rh_sp_pct,
        "pid_enabled": reading.pid_enabled,
        "read_ok": reading.read_ok,
        "partial": reading.partial,
        "unknown_channels": {
            key: value.get("value")
            for key, value in reading.unknown_channels.items()
        },
        "last_error": last_error,
        "t_utc": reading.t_utc,
        "note": UNKNOWN_NOTE,
    }


def _print(state: dict) -> None:
    print("chamber status @ %s  (read_ok=%r partial=%r)"
          % (state["t_utc"], state["read_ok"], state["partial"]))
    print("  temperature : raw=%s C   filtered=%s C   setpoint=%s C"
          % (state["temp_raw_c"], state["temp_filtered_c"], state["temp_sp_c"]))
    print("  humidity    : raw=%s %%RH  filtered=%s %%RH  setpoint=%s %%RH"
          % (state["rh_raw_pct"], state["rh_filtered_pct"], state["rh_sp_pct"]))
    print("  PID (C1)    : %r  (None means NOT READ, which is not 'off')"
          % state["pid_enabled"])
    if state["unknown_channels"]:
        print("  unknown DFs : %s  (MEANING UNKNOWN)" % state["unknown_channels"])
    print("  note        : %s" % state["note"])


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--json", action="store_true",
                    help="print the status as one JSON object on stdout")
    ap.add_argument("--config", default=str(REPO / "configs" / "config.yaml"),
                    help="config file to resolve settings from")
    return ap


def run(args: argparse.Namespace) -> int:
    state = check_environment(config=args.config)
    if args.json:
        print(json.dumps(state, indent=2, default=str))
    else:
        _print(state)
    return 0 if (state["read_ok"] and not state["partial"]) else 1


def main(argv=None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
