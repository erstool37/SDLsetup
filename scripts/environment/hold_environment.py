#!/usr/bin/env python3
"""
hold_environment.py -- hold the chamber at a setpoint, relaying DF9 to the bath.

THIS SCRIPT OWNS WHAT tools/ MAY NOT DECIDE
  `tools/` reports; `scripts/` decides. Everything below is a decision, which is
  why it lives here and not in `tools.environment.relay`:
    * the loop, and its cadence (`--period`)
    * when to stop (`--duration`, `--stop-on-stale`, Ctrl-C)
    * what to do when a read is unusable -- the RETRY/HOLD POLICY, below
    * whether to actuate at all (`--execute`, plus both config gates)
    * the run record

  `Relay.step()` does exactly one read and one forward per call and refuses
  rather than choosing. Its two watchdogs RAISE `CommsLost` after executing the
  pre-declared fail-safe; catching that and deciding whether to continue is
  this file's job.

THE RETRY/HOLD POLICY, STATED
  On an unusable PLC reading, this script does NOTHING and comes round again on
  the next period. It does not re-read immediately, it does not back off, and
  above all it DOES NOT RE-SEND THE LAST VALUE.

  Not re-sending is the whole point. A real log from 2026-01-11 ends with the
  temperature output pinned at exactly 30.000 while the measured temperature
  read 11.4 C -- "hold the last value" there meant "hold a saturated maximum
  for hours". So the bath simply keeps whatever it last received (it has no
  read-back, so nothing else is even knowable) until either a good reading
  arrives or `environment.plc_stale_s` elapses and the relay's watchdog forces
  the declared safe setpoint and disables the PID.

  Immediate retry is deliberately absent too: the CLICK has three Modbus
  sockets, and a tight retry loop against a controller that is already not
  answering spends them. One read per period, and let the watchdog decide.

  `--stop-on-stale` (default: stop) chooses what happens after the watchdog
  fires. `stop` ends the run non-zero -- the safe setpoint is written and the
  PID is off, and continuing to poll a PLC that went silent is not a recovery
  strategy. `continue` keeps polling in safe mode, for the case where an
  operator is watching and wants the loop to resume by itself if comms come
  back; it never leaves safe mode on its own, because re-enabling the PID is an
  actuation nobody asked for.

ACTUATION IS TRIPLE-GATED
  `--execute` AND `environment.allow_actuation` AND `circulator.allow_actuation`
  must all be true. A bare `--execute` against closed gates REFUSES and names
  which gate is shut -- it does not silently degrade to a dry run, because a run
  that thinks it is controlling the chamber and is not is worse than one that
  stopped.

THE finally BLOCK IS THE ONE REAL INTERLOCK
  `relay.safe()` writes the declared safe setpoint and, in its own `finally`,
  drives `C1` False. That mirrors the single interlock the prior system had -- a
  `try/finally` writing `C1 = False` -- and it is the property worth keeping
  from it. `Relay.__exit__` does the same while an exception propagates, and
  this file's `finally` calls it again explicitly so the guarantee does not
  depend on which path exited.

FIRST ADOPTER OF tools.runs.Run
  This and `environment_probe.py` are the first phase scripts in the repo to
  open a `Run`, deliberately: a supervisor loop's value is the record of what it
  forwarded and what it refused, and `Run` writes the manifest on the way out
  INCLUDING on failure -- a run that aborted is the one whose record matters
  most.

    # dry run, nothing leaves the process
    python scripts/environment/hold_environment.py --temp-sp 24.0 --duration 30
    # live, only if both config gates are open
    python scripts/environment/hold_environment.py --temp-sp 24.0 --execute
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from tools import circulator as circ  # noqa: E402
from tools import config as _config  # noqa: E402
from tools import environment as env  # noqa: E402
from tools import occupancy, runs  # noqa: E402

DEVICE = "environment"
BATH = "circulator"
PHASE = "hold_environment"


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--temp-sp", type=float, default=None,
                    help="write this temperature setpoint (DF5) once at start, "
                         "in degC. Refused outside the STRICT open interval "
                         "config declares; omit to leave the PLC's own "
                         "setpoint alone")
    ap.add_argument("--rh-sp", type=float, default=None,
                    help="write this humidity setpoint (DF6) once at start, in "
                         "%%RH. Same discipline. The PLC actuates humidity "
                         "itself; nothing about it transits this host")
    ap.add_argument("--duration", type=float, default=60.0,
                    help="seconds to hold for (default: 60.0)")
    ap.add_argument("--period", type=float, default=None,
                    help="seconds between relay iterations (default: "
                         "environment.relay_period_s)")
    ap.add_argument("--execute", action="store_true",
                    help="actuate. Requires environment.allow_actuation AND "
                         "circulator.allow_actuation as well; a bare --execute "
                         "against a closed gate is REFUSED, not downgraded")
    ap.add_argument("--no-relay", action="store_true",
                    help="do not forward DF9 to the bath. Setpoint writes and "
                         "the read loop still happen; the circulator is never "
                         "opened and its port is never touched")
    ap.add_argument("--safe-setpoint", type=float, default=None,
                    help="override circulator.safe_setpoint_c for this run, in "
                         "degC. There is no code default: a safe setpoint "
                         "nobody declared is not safe")
    ap.add_argument("--stop-on-stale", choices=("stop", "continue"),
                    default="stop",
                    help="what to do after a watchdog fires and the fail-safe "
                         "has run: stop the run non-zero (default), or keep "
                         "polling in safe mode without ever re-enabling the PID")
    ap.add_argument("--enable-pid", action="store_true",
                    help="enable the PLC PID loop (coil C1) ONCE, after the "
                         "setpoint writes and before the hold loop. Live "
                         "only; ignored in a dry run. A plain hold leaves C1 "
                         "untouched -- this is what a full kinetics run adds")
    ap.add_argument("--config", default=str(REPO / "configs" / "config.yaml"),
                    help="config file to resolve settings from")
    return ap


def _gate_refusal(args: argparse.Namespace, plc: env.PlcSettings,
                  bath: circ.CirculatorSettings) -> str | None:
    """Which gate is shut, or ``None`` if actuation is permitted.

    Names the specific key and where it resolved from. A refusal that says only
    "actuation not allowed" sends the operator looking in the wrong file.
    """
    if not args.execute:
        return None                      # dry run was asked for; nothing to gate
    shut: list[str] = []
    if not plc.allow_actuation:
        shut.append("environment.allow_actuation is FALSE (from %s)"
                    % plc.sources.get("allow_actuation", "default"))
    if not args.no_relay and not bath.allow_actuation:
        shut.append("circulator.allow_actuation is FALSE (from %s)"
                    % bath.sources.get("allow_actuation", "default"))
    if not shut:
        return None
    return (
        "--execute was passed, but %s.\n"
        "REFUSING rather than falling back to a dry run: a supervisor that "
        "believes it is controlling the chamber while nothing leaves the "
        "process is worse than one that stopped. Open the gate in %s, or drop "
        "--execute to plan.%s"
        % (" and ".join(shut), args.config,
           "" if args.no_relay else
           "\nNote: --no-relay would drop the circulator gate from this list, "
           "since the bath is then never opened."))


def _check_safe_setpoint(policy: env.RelayPolicy,
                         bath: circ.CirculatorSettings) -> str | None:
    """Is the declared safe setpoint writable under THIS run's command range?

    ``RelayPolicy`` validates ``safe_setpoint_c`` against ``CommandLimits()``,
    the CODE CEILING -- currently ``(0, 30)`` C. ``configs/config.yaml``
    tightens the floor to the measured 10.0 (the PLC's own ``DF21 Temp Output
    LB``, read 2026-09-07), and the policy does not see that tightening. So a
    declared safe setpoint in ``(0, 10]`` constructs fine and is then REFUSED
    during an abort, which is the worst possible moment to discover it.

    Checked here, before the loop starts, against the resolved range. This
    closes the gap for this script only; see tools/circulator/safety.py for the
    standing TODO about the code ceiling itself.
    """
    try:
        bath.limits.validate(float(policy.safe_setpoint_c))
    except circ.SafetyError as exc:
        return (
            "circulator.safe_setpoint_c=%r is not writable under this run's "
            "command range.\n  %s\n  %s\n"
            "RelayPolicy validated it against the CODE CEILING (%g, %g) C, "
            "which is wider than the resolved range -- so it passed there and "
            "would have failed during an abort. Choose a value strictly inside "
            "the resolved range in %s, or pass --safe-setpoint."
            % (policy.safe_setpoint_c, exc, bath.limits.describe(),
               circ.COMMAND_MIN_C, circ.COMMAND_MAX_C, "configs/config.yaml"))
    return None


def _write_setpoints(args: argparse.Namespace, client: env.PlcClient,
                     record_run: runs.Run, live: bool) -> bool:
    """Write DF5/DF6 once, if asked. Returns False if a write was not confirmed.

    Branches on ``outcome``, never on ``ok``: ``ok`` is False for a perfectly
    successful dry run, so ``if not result.ok`` would read every plan as a
    failure.
    """
    all_good = True
    for flag, field, setter, value in (
            ("--temp-sp", "temp_sp_c", env.set_temperature, args.temp_sp),
            ("--rh-sp", "rh_sp_pct", env.set_humidity, args.rh_sp)):
        if value is None:
            continue
        result = setter(value, args.config, client=client, dry_run=not live)
        record_run.record(DEVICE, "setpoint_writes", result.as_dict())
        record_run.log("%s %s: %s" % (flag, field, result.describe()),
                       device=DEVICE)
        print(result.describe())
        if result.outcome == env.FAILED:
            all_good = False
            record_run.note("the %s write FAILED and was not verified: %r. What "
                            "the PLC now holds for %s is UNKNOWN."
                            % (flag, result.error, field))
        elif result.outcome == env.PLANNED:
            record_run.note("the %s write was PLANNED only -- nothing was sent, "
                            "so the PLC still holds whatever it held before."
                            % flag)
    return all_good


def run(args: argparse.Namespace) -> int:
    if args.duration <= 0.0:
        print("--duration must be positive", file=sys.stderr)
        return 2

    # ---- resolve everything, and surface a bad config as a refusal ----------
    try:
        plc_settings = env.PlcSettings.from_config(args.config)
        bath_settings = circ.CirculatorSettings.from_config(args.config)
        overrides = {}
        if args.safe_setpoint is not None:
            overrides["safe_setpoint_c"] = args.safe_setpoint
        # from_config RAISES with an actionable message while safe_setpoint_c is
        # unset. Surfaced verbatim: it names the key, the file, the blank-key
        # parser trap, and the range -- which is more than a summary would.
        policy = env.RelayPolicy.from_config(args.config, **overrides)
    except _config.ConfigError as exc:
        print("REFUSED -- configuration:\n%s" % exc, file=sys.stderr)
        return 2

    period = args.period if args.period is not None else plc_settings.relay_period_s
    if period <= 0.0:
        print("--period must be positive", file=sys.stderr)
        return 2

    gate = _gate_refusal(args, plc_settings, bath_settings)
    if gate is not None:
        print("REFUSED:\n%s" % gate, file=sys.stderr)
        return 2

    if not args.no_relay:
        mismatch = _check_safe_setpoint(policy, bath_settings)
        if mismatch is not None:
            print("REFUSED:\n%s" % mismatch, file=sys.stderr)
            return 2

    live = bool(args.execute)
    devices = (DEVICE,) if args.no_relay else (DEVICE, BATH)

    # NOT the `with` form here, deliberately. Run.__exit__ stamps STATUS_OK
    # whenever no exception escaped, and this script's failures are RETURNED
    # (a watchdog that fired, an unconfirmed setpoint write) rather than raised
    # -- so the context manager would record a failed hold as a clean one. The
    # explicit finally below writes the manifest on every path instead, which is
    # the guarantee the context manager exists to provide.
    record_run = runs.Run.create(
        PHASE, config=args.config, argv=sys.argv, devices=devices,
        description="hold the chamber, relaying DF9 to the bath")
    # Pessimistic defaults: if this function leaves by a path that set neither,
    # the manifest says failed rather than ok.
    status = runs.STATUS_FAILED
    exit_code = 1
    forwarded = refused = 0
    try:
        # -- the honest residual, and what this run does NOT cover ------------
        record_run.note(env.Relay.HONEST_RESIDUAL)
        # F7: every watchdog here is COOPERATIVE -- it fires only from inside
        # Relay.step(), which this loop must keep calling. If this process is
        # SIGKILLed or loses power, nothing fires. Recorded verbatim so the run
        # says what it cannot protect against.
        record_run.note(env.COOPERATIVE_WATCHDOG_RESIDUAL)
        record_run.note(
            "NO PLC-SIDE WATCHDOG IS IN USE. The CLICK's own `SD41` diagnostic "
            "bit would let the ladder notice that this supervisor stopped "
            "talking, but its MODBUS ADDRESS IS UNKNOWN and was not guessed -- "
            "an invented address reads some other register and reports it as a "
            "link flag. So every watchdog here is HOST-SIDE: if this process "
            "dies without running its finally block (SIGKILL, power loss), "
            "nothing on the PLC notices, the ladder keeps running, and the bath "
            "keeps its last setpoint. TODO(operator): establish the SD41 "
            "address, or add a ladder-side heartbeat timeout.")
        record_run.note(
            "channel IDENTITY beyond DF1/DF2 is UNCONFIRMED, so the forwarded "
            "channel is DF9 because the PLC's nickname table says `Tem PID "
            "Output`, not because that was verified independently.")
        if not live:
            record_run.note("DRY RUN: no frame left this process. Nothing here "
                            "asserts that a live run would have succeeded.")
        if args.no_relay:
            record_run.note("--no-relay: DF9 was NOT forwarded and the "
                            "circulator's port was never opened. Nothing is "
                            "asserted about the bath.")

        record_run.log(plc_settings.describe(), device=DEVICE)
        if not args.no_relay:
            # Guarded: logging to a device creates its folder, and a circulator
            # folder in a run that never touched the bath is a misleading record.
            record_run.log(bath_settings.describe(), device=BATH)
        record_run.log(policy.describe(), device=DEVICE)
        record_run.log(env.provenance(), device=DEVICE)
        print(policy.describe())
        print()

        status = runs.STATUS_OK
        exit_code = 0
        bath_device = None
        relay = None

        # One claim for the whole session: the CLICK has three Modbus sockets,
        # and claiming per-iteration would let another client in between two of
        # them.
        with occupancy.claim(DEVICE, doing="%s (live=%r)" % (PHASE, live)):
            client = env.PlcClient(plc_settings)
            try:
                opened = client.connect()
                record_run.log("connect -> %r (last_error=%r)"
                               % (opened, client.last_error), device=DEVICE)

                if not _write_setpoints(args, client, record_run, live):
                    record_run.note("a setpoint write was not confirmed; the "
                                    "hold loop ran anyway, so the PLC may have "
                                    "been holding a different setpoint than "
                                    "this run asked for.")
                    exit_code = 1

                # A kinetics run must CLOSE the loop; a plain hold does not.
                # Enabled once here, in-session, after the setpoints and
                # before the loop. Live only -- a dry run enables no PID.
                if getattr(args, "enable_pid", False):
                    if live:
                        pid_res = env.enable_pid(args.config, client=client)
                        record_run.record(DEVICE, "pid_enable", pid_res.as_dict())
                        record_run.log("enable PID (C1): %s" % pid_res.describe(),
                                       device=DEVICE)
                        print(pid_res.describe())
                        if pid_res.outcome == env.FAILED:
                            record_run.note("--enable-pid FAILED (%r); the PLC "
                                            "loop may not be closed."
                                            % pid_res.error)
                            exit_code = 1
                    else:
                        record_run.note("--enable-pid was requested but this "
                                        "is a DRY RUN; the PID was NOT enabled.")

                if args.no_relay:
                    deadline = time.monotonic() + args.duration
                    while time.monotonic() < deadline:
                        reading = client.read_block()
                        record_run.record(DEVICE, "readings", reading.as_dict())
                        print("read_ok=%r partial=%r temp=%r rh=%r pid=%r"
                              % (reading.read_ok, reading.partial,
                                 reading.temp_filtered_c, reading.rh_filtered_pct,
                                 reading.pid_enabled))
                        time.sleep(period)
                else:
                    bath_device = circ.Circulator(
                        bath_settings, log=record_run.logger(BATH))
                    if live:
                        # Opening the port hardware-RESETS the MCU, so it is
                        # done ONCE, here, explicitly -- never on demand from
                        # inside the loop.
                        record_run.log("opening the serial port -- this RESETS "
                                       "the MCU", device=BATH)
                        record_run.record(BATH, "port", bath_device.open())
                    relay = env.Relay(client, bath_device, policy)
                    forwarded, refused, exit_code, status = _hold(
                        relay, record_run, args, period, exit_code)
            finally:
                # The one real interlock, and it runs on every path: safe
                # setpoint written, C1 driven False in safe()'s own finally.
                if relay is not None:
                    final = relay.safe(reason="hold_environment finished "
                                              "(status=%s)" % status)
                    record_run.record(DEVICE, "relay", final.as_dict())
                    record_run.log(final.describe(), device=DEVICE)
                    print(final.describe())
                elif not args.no_relay and bath_device is not None:
                    record_run.note("no Relay was constructed, so no fail-safe "
                                    "setpoint was written. The bath keeps "
                                    "whatever it last received.")
                if bath_device is not None:
                    bath_device.close()
                client.close()
    except BaseException as exc:
        # Recorded as a fact before it propagates: a run that crashed is the one
        # whose record matters most, and "it failed" without the exception text
        # is not a record.
        status = (runs.STATUS_ABORTED if isinstance(exc, KeyboardInterrupt)
                  else runs.STATUS_FAILED)
        record_run.note("%s: %s: %s" % (status, type(exc).__name__, exc))
        raise
    finally:
        record_run.metric("forwarded", forwarded)
        record_run.metric("refused", refused)
        record_run.metric("live", live)
        record_run.metric("period_s", period)
        print()
        print("run: %s" % record_run.path)
        print("     forwarded=%d refused=%d status=%s" % (forwarded, refused, status))
        record_run.finish(status, exit_code=exit_code)

    return exit_code


def _hold(relay: env.Relay, record_run: runs.Run, args: argparse.Namespace,
          period: float, exit_code: int) -> tuple[int, int, int, str]:
    """The loop. Returns ``(forwarded, refused, exit_code, status)``.

    Every decision in here is this file's: one `step()` per period, no immediate
    retry, no re-send of a stale value, and `--stop-on-stale` deciding what
    happens after a watchdog has already executed the fail-safe.
    """
    forwarded = refused = 0
    status = runs.STATUS_OK
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            try:
                record = relay.step()
            except env.CommsLost as exc:
                # The relay has ALREADY ATTEMPTED the safe setpoint and the PID
                # disable before raising. What happens next is a decision.
                #
                # env.FailSafeIncomplete (a CommsLost subclass) means the
                # fail-safe could NOT be confirmed -- a dead serial link or a
                # gate off -- so the bath may not hold the safe setpoint. It is
                # caught here too and flagged loudly; there is no software
                # recovery for a dead link (see HONEST_RESIDUAL).
                attached = getattr(exc, "record", None)
                if attached is not None:
                    record_run.record(DEVICE, "relay", attached.as_dict())
                if isinstance(exc, env.FailSafeIncomplete):
                    record_run.note("FAIL-SAFE INCOMPLETE: the watchdog fired but "
                                    "the safe setpoint and/or PID disable were not "
                                    "confirmed (%s). The bath state is UNVERIFIED; "
                                    "a person at the rig is required." % exc)
                # Safe mode is now LATCHED (F1): this script does NOT rearm the
                # relay. Re-arming resumes forwarding and is an actuation nobody
                # asked for mid-abort; --stop-on-stale=continue keeps polling in
                # latched safe mode, and only a deliberate operator rearm() plus
                # a PID re-enable would resume control.
                record_run.log("WATCHDOG: %s" % exc, device=DEVICE, level="error")
                print("WATCHDOG: %s" % exc, file=sys.stderr)
                refused += 1
                if args.stop_on_stale == "stop":
                    record_run.note("stopped on a watchdog: the fail-safe ran "
                                    "(safe setpoint written, PID disabled) and "
                                    "--stop-on-stale=stop ended the run. "
                                    "Continuing to poll a silent PLC is not a "
                                    "recovery strategy.")
                    return forwarded, refused, 1, runs.STATUS_FAILED
                record_run.note("--stop-on-stale=continue: still polling in "
                                "SAFE MODE. The PID is NOT re-enabled by this "
                                "script -- that is an actuation an operator "
                                "must make deliberately.")
                exit_code = 1
                time.sleep(period)
                continue
            except (env.SafetyError, env.CirculatorSafetyError) as exc:
                # A refused DF9. The relay went safe before raising; a value
                # outside the command bound is a fault upstream, not something
                # to retry around.
                attached = getattr(exc, "record", None)
                if attached is not None:
                    record_run.record(DEVICE, "relay", attached.as_dict())
                record_run.log("REFUSED: %s" % exc, device=DEVICE, level="error")
                print("REFUSED: %s" % exc, file=sys.stderr)
                record_run.note("a bound refused the forwarded value and the "
                                "fail-safe ran. Not retried: an out-of-bound "
                                "PID output is a fault upstream of this script.")
                return forwarded, refused + 1, 2, runs.STATUS_FAILED

            record_run.record(DEVICE, "relay", record.as_dict())
            if record.forwarded:
                forwarded += 1
            else:
                # Refused, and the watchdog has not fired yet. Do nothing: no
                # re-read, no back-off, and above all no re-send of the last
                # value -- see the RETRY/HOLD POLICY in the module docstring.
                refused += 1
            print(record.describe())
            time.sleep(period)
    except KeyboardInterrupt:
        record_run.log("interrupted by the operator", device=DEVICE, level="warn")
        print("\ninterrupted -- going safe", file=sys.stderr)
        return forwarded, refused, 130, runs.STATUS_ABORTED
    return forwarded, refused, exit_code, status


def main(argv=None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
