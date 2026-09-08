#!/usr/bin/env python3
"""
start_kinetics.py -- one call that starts a full kinetics run on the chamber.

WHAT THIS IS
  The clean, one-call entry point to a full run: it sets the temperature and
  humidity setpoints, enables the PLC's PID loop, and then holds the chamber for
  the requested duration while forwarding the loop's output to the bath. The
  defaults reproduce the 2021 platform paper's demonstrated condition -- air
  temperature 25 C, air humidity 93 %RH, ~72 h.

    from scripts.environment.start_kinetics import start_kinetics
    start_kinetics()                       # dry-run plan, defaults, nothing written
    start_kinetics(temp_c=25, rh_pct=93, hours=72, execute=True)   # live (gated)

  Or from the shell:

    python scripts/environment/start_kinetics.py --temp 25 --rh 93 --hours 72
    python scripts/environment/start_kinetics.py --temp 25 --rh 93 --hours 72 --execute

`scripts/` DECIDES; `tools/` REPORTS
  Setting setpoints, enabling the PID, and running a loop are DECISIONS, so this
  lives in `scripts/environment/`, never in `tools/`. It is the reader's full-run
  entry point; `check_environment.py` beside it is the zero-actuation status read.

REUSE, NOT DUPLICATION (which of the two offered options this took)
  This took the second option from the brief: `start_kinetics` sets the
  setpoints and then CALLS `hold_environment`'s runner. The supervised loop, the
  triple-gated `--execute`, the `finally: relay.safe()` interlock, the
  `COOPERATIVE_WATCHDOG_RESIDUAL` note, and the `CommsLost`/`FailSafeIncomplete`
  handling all live in `hold_environment.py` and are reused verbatim -- the only
  thing added there is `--enable-pid`, because a kinetics run must close the loop
  and a plain hold does not. Nothing about the loop or the fail-safe is copied
  here.

DRY RUN IS THE DEFAULT
  With `execute=False` (the default) nothing is written, no serial port is
  opened (opening it hardware-resets the MCU), and the PID is not enabled. The
  requested setpoints are still validated through the guarded, type-enforced
  path, so an out-of-range `temp_c`/`rh_pct` is REFUSED even in a dry run. One
  READ-ONLY block read is taken to show the chamber's current state in the plan.

EXECUTE IS TRIPLE-GATED
  `execute=True` additionally requires BOTH `environment.allow_actuation` and
  `circulator.allow_actuation` to be true. If either is shut, this REFUSES,
  names the shut gate, and returns without touching hardware -- it never silently
  degrades to a dry run.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from scripts.environment import hold_environment  # noqa: E402
from tools import circulator as circ  # noqa: E402
from tools import environment as env  # noqa: E402
from tools import occupancy, runs  # noqa: E402

DEVICE = "environment"
BATH = "circulator"
PHASE = "start_kinetics"

#: The demonstrated condition from the 2021 platform paper. Defaults, not law.
DEFAULT_TEMP_C = 25.0
DEFAULT_RH_PCT = 93.0
DEFAULT_HOURS = 72.0


def start_kinetics(temp_c: float = DEFAULT_TEMP_C, rh_pct: float = DEFAULT_RH_PCT,
                   hours: float = DEFAULT_HOURS, *, execute: bool = False,
                   config: Any = None, period_s: float | None = None,
                   _client: Any = None) -> dict:
    """Start (or plan) a kinetics run. Returns a summary dict.

    Keys: ``run_dir``, ``executed``, ``iterations``, ``final_reading``,
    ``ended``, ``notes``. ``_client`` is an internal seam for tests to inject a
    fake-backed :class:`tools.environment.api.PlcClient` and never touch a
    socket; production callers leave it ``None``.
    """
    cfg = config if config is not None else str(REPO / "configs" / "config.yaml")

    # Surfaces a bad config (e.g. an unset circulator.safe_setpoint_c) as a
    # refusal to construct, before anything else happens.
    plc_settings = env.PlcSettings.from_config(cfg)
    bath_settings = circ.CirculatorSettings.from_config(cfg)
    policy = env.RelayPolicy.from_config(cfg)

    # Validate the requested setpoints through the GUARDED path -- the same
    # type-enforced SetpointLimits.validate the wire write uses. It raises
    # env.SafetyError outside the strict (0, 30) C / (0, 100) %RH bound, and
    # does so here even in a dry run: a plan for an impossible setpoint is not a
    # plan worth printing.
    plc_settings.limits.validate("temp_sp_c", float(temp_c))
    plc_settings.limits.validate("rh_sp_pct", float(rh_pct))

    period = float(period_s) if period_s is not None else float(plc_settings.relay_period_s)
    if float(hours) <= 0.0:
        raise ValueError("hours must be positive, got %r" % (hours,))
    if period <= 0.0:
        raise ValueError("period_s must be positive, got %r" % (period,))

    if execute:
        return _execute(temp_c, rh_pct, hours, period, cfg,
                        plc_settings, bath_settings)
    return _plan(temp_c, rh_pct, hours, period, cfg,
                 plc_settings, bath_settings, policy, _client)


def _execute(temp_c: float, rh_pct: float, hours: float, period: float,
             cfg: Any, plc_settings: Any, bath_settings: Any) -> dict:
    """Delegate a live run to ``hold_environment`` -- refuse first if a gate is shut.

    The gate check here builds a clean, named refusal for the summary dict; it
    is a pre-check, not a re-implementation. When both gates are open the whole
    supervised run (loop + fail-safe) is `hold_environment`'s.
    """
    shut: list[str] = []
    if not plc_settings.allow_actuation:
        shut.append("environment.allow_actuation is FALSE (from %s)"
                    % plc_settings.sources.get("allow_actuation", "default"))
    if not bath_settings.allow_actuation:
        shut.append("circulator.allow_actuation is FALSE (from %s)"
                    % bath_settings.sources.get("allow_actuation", "default"))
    if shut:
        msg = ("REFUSED: execute=True requires BOTH actuation gates open; shut: "
               "%s. No hardware was touched -- not a socket, not the serial port, "
               "not the PID coil. Open the gate(s) in %s, or run without execute "
               "to plan." % (" and ".join(shut), cfg))
        print(msg, file=sys.stderr)
        return {"run_dir": None, "executed": False, "iterations": 0,
                "final_reading": None,
                "ended": "refused: %s" % " and ".join(shut), "notes": [msg]}

    # Both gates open. Reuse hold_environment's runner verbatim -- its loop, its
    # gate re-check, its finally: relay.safe(), its residual notes -- adding only
    # --enable-pid so the PLC loop is closed for a kinetics run.
    argv = ["--temp-sp", str(float(temp_c)), "--rh-sp", str(float(rh_pct)),
            "--duration", str(float(hours) * 3600.0), "--period", str(float(period)),
            "--enable-pid", "--execute", "--config", str(cfg)]
    args = hold_environment.build_parser().parse_args(argv)
    code = hold_environment.run(args)
    return {"run_dir": None, "executed": code == 0, "iterations": None,
            "final_reading": None,
            "ended": ("delegated to hold_environment (exit=%d); the run manifest "
                      "printed by that runner carries the per-iteration record, "
                      "forwarded/refused counts, and every note" % code),
            "notes": ["live run executed via hold_environment.run -- the shared "
                      "supervised loop and fail-safe were reused, not duplicated"]}


def _plan(temp_c: float, rh_pct: float, hours: float, period: float, cfg: Any,
          plc_settings: Any, bath_settings: Any, policy: Any,
          client: Any) -> dict:
    """The dry-run path: validate, read once (read-only), print a plan, record it."""
    notes = [
        env.Relay.HONEST_RESIDUAL,
        env.COOPERATIVE_WATCHDOG_RESIDUAL,
        "DRY RUN: no setpoint was written, no serial port was opened, the PID "
        "was NOT enabled. Nothing here asserts that a live run would succeed.",
        "channel IDENTITY beyond DF1/DF2 is UNCONFIRMED, and DF3/DF4 MEANING and "
        "the SC90-SC95/SD41 native diagnostics' Modbus ADDRESSES remain UNKNOWN "
        "-- none was guessed.",
    ]
    record_run = runs.Run.create(
        PHASE, config=cfg, argv=sys.argv, devices=(DEVICE, BATH),
        description="plan (dry-run) a kinetics run")
    final_reading: dict | None = None
    try:
        for text in notes:
            record_run.note(text)
        record_run.log(plc_settings.describe(), device=DEVICE)
        record_run.log(bath_settings.describe(), device=BATH)
        record_run.log(policy.describe(), device=DEVICE)
        record_run.log(env.provenance(), device=DEVICE)

        # One READ-ONLY block read to show current state. A read is not a write;
        # it opens no serial port and enables no PID. Best-effort: a failed read
        # does not sink the plan.
        try:
            with occupancy.claim(DEVICE, doing="%s (dry-run read-only)" % PHASE):
                owned = client is None
                plc = client if client is not None else env.PlcClient(plc_settings)
                try:
                    plc.connect()
                    reading = plc.read_block()
                    final_reading = reading.as_dict()
                    record_run.record(DEVICE, "readings", final_reading)
                finally:
                    if owned:
                        plc.close()
        except Exception as exc:  # noqa: BLE001
            record_run.note("the dry-run read-only read did not complete (%s: %s); "
                            "the plan is unaffected -- it describes what WOULD be "
                            "done, not the current state." % (type(exc).__name__, exc))

        plan = _plan_text(temp_c, rh_pct, hours, period, plc_settings,
                          bath_settings, policy, final_reading)
        print(plan)

        record_run.metric("planned_temp_sp_c", float(temp_c))
        record_run.metric("planned_rh_sp_pct", float(rh_pct))
        record_run.metric("planned_hours", float(hours))
        record_run.metric("period_s", float(period))
        record_run.metric("executed", False)
    finally:
        record_run.finish(runs.STATUS_OK)

    return {"run_dir": str(record_run.path), "executed": False, "iterations": 0,
            "final_reading": final_reading, "ended": "planned", "notes": notes}


def _plan_text(temp_c: float, rh_pct: float, hours: float, period: float,
               plc_settings: Any, bath_settings: Any, policy: Any,
               final_reading: dict | None) -> str:
    lines = [
        "",
        "=" * 70,
        "KINETICS RUN PLAN (dry run -- nothing below was written)",
        "=" * 70,
        "  WOULD write temperature setpoint : %.6g C   (DF5)" % float(temp_c),
        "  WOULD write humidity setpoint    : %.6g %%RH  (DF6)" % float(rh_pct),
        "  WOULD enable PID (coil C1)        : yes, once, after the setpoints",
        "  hold duration                     : %.6g h  (%.0f s)"
        % (float(hours), float(hours) * 3600.0),
        "  relay period                      : %.6g s" % float(period),
        "  comms-loss safe setpoint          : %.6g C  (%s)"
        % (float(policy.safe_setpoint_c),
           policy.sources.get("safe_setpoint_c", "-")),
        "",
    ]
    if final_reading is not None:
        ch = final_reading.get("channels", {})
        lines += [
            "  current chamber state (read-only):",
            "    temp filtered : %s C" % ch.get("temp_filtered_c"),
            "    rh filtered   : %s %%RH" % ch.get("rh_filtered_pct"),
            "    temp setpoint : %s C" % ch.get("temp_sp_c"),
            "    rh setpoint   : %s %%RH" % ch.get("rh_sp_pct"),
            "    PID (C1)      : %r" % final_reading.get("pid_enabled"),
            "    read_ok=%r partial=%r"
            % (final_reading.get("read_ok"), final_reading.get("partial")),
            "",
        ]
    else:
        lines += ["  current chamber state: NOT read (see run notes)", ""]
    lines += [plc_settings.describe(), "", bath_settings.describe(), "",
              policy.describe(), ""]
    return "\n".join(lines)


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--temp", type=float, default=DEFAULT_TEMP_C,
                    help="temperature setpoint in degC (default: %(default)s)")
    ap.add_argument("--rh", type=float, default=DEFAULT_RH_PCT,
                    help="humidity setpoint in %%RH (default: %(default)s)")
    ap.add_argument("--hours", type=float, default=DEFAULT_HOURS,
                    help="hold duration in hours (default: %(default)s)")
    ap.add_argument("--period", type=float, default=None,
                    help="relay period in s (default: environment.relay_period_s)")
    ap.add_argument("--execute", action="store_true",
                    help="actuate. Requires BOTH environment.allow_actuation and "
                         "circulator.allow_actuation; a bare --execute against a "
                         "shut gate is REFUSED, never downgraded")
    ap.add_argument("--config", default=str(REPO / "configs" / "config.yaml"),
                    help="config file to resolve settings from")
    return ap


def run(args: argparse.Namespace) -> int:
    try:
        summary = start_kinetics(temp_c=args.temp, rh_pct=args.rh,
                                 hours=args.hours, execute=args.execute,
                                 config=args.config, period_s=args.period)
    except env.SafetyError as exc:
        print("REFUSED -- setpoint out of the guarded bound:\n%s" % exc,
              file=sys.stderr)
        return 2
    print()
    print("summary:")
    for key in ("run_dir", "executed", "iterations", "final_reading", "ended"):
        value = summary.get(key)
        if key == "final_reading" and isinstance(value, dict):
            value = "<reading: %d channels>" % len(value.get("channels", {}))
        print("  %-14s %r" % (key, value))
    if args.execute and not summary["executed"]:
        return 2
    return 0


def main(argv=None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
