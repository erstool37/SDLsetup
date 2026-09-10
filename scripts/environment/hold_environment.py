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
import os
import signal
import sys
import threading
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

#: Manual valve control is NOT wired, and the dashboard is told exactly that
#: rather than shown a switch that does nothing. Y001's Modbus coil address is
#: unverified, and so is the ladder gating that would have to honour it -- an
#: invented coil address writes some other output. So every `valve` command is
#: REFUSED, carrying this text as its reason. A later investigation may
#: establish the address; these are a module constant so that is a one-line
#: change here rather than a search through the loop.
VALVE_MODE = "unavailable"
VALVE_NOTE = ("Y001 coil address and ladder gating unverified; manual valve "
              "control is not wired")

#: The one mapping from a dashboard setpoint key to the CLI flag it mirrors, the
#: field the bound is keyed on, the `args` attribute that flag lands in, and the
#: guarded writer. `_write_setpoints` (the --temp-sp/--rh-sp path) and
#: `_apply_setpoint` (the dashboard path) both iterate THIS, so the two cannot
#: drift apart on which field, which bound, or which writer -- there is one
#: setpoint write path in this file and it is `env.set_temperature` /
#: `env.set_humidity`, whichever asked for it.
SETPOINT_WRITERS = (
    ("temp_c", "--temp-sp", "temp_sp_c", "temp_sp", env.set_temperature),
    ("rh_pct", "--rh-sp", "rh_sp_pct", "rh_sp", env.set_humidity),
)


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
                     record_run: runs.Run, live: bool,
                     history: dict[str, tuple[float, float]]) -> bool:
    """Write DF5/DF6 once, if asked. Returns False if a write was not confirmed.

    Branches on ``outcome``, never on ``ok``: ``ok`` is False for a perfectly
    successful dry run, so ``if not result.ok`` would read every plan as a
    failure.

    A CONFIRMED write is recorded in ``history``, which is the rate limiter's
    memory. That matters because the dashboard can change a setpoint seconds
    later: without seeding it here, the interval rule would treat the first
    dashboard command of a run as "no previous write" and let it land on top of
    the one this function just made.
    """
    all_good = True
    for _key, flag, field, attr, setter in SETPOINT_WRITERS:
        value = getattr(args, attr)
        if value is None:
            continue
        result = setter(value, args.config, client=client, dry_run=not live)
        if result.outcome == env.CONFIRMED:
            history[field] = (float(value), time.time())
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


def _stop_on_signal(signum: int, _frame: object) -> None:
    """Turn a signal into the KeyboardInterrupt this file already handles."""
    raise KeyboardInterrupt("stopped by signal %d" % signum)


def _install_operator_stop_handlers() -> dict:
    """Wire SIGINT and SIGTERM to the loop's existing KeyboardInterrupt path.

    Returns the previous dispositions; hand them to
    :func:`_restore_stop_handlers` in the same ``finally`` that runs the
    fail-safe.

    WHY THIS IS NOT REDUNDANT WITH WHAT CPython ALREADY DOES. A process started
    as a background job from a NON-INTERACTIVE bash inherits SIGINT set to
    ``SIG_IGN``, and CPython leaves an inherited ``SIG_IGN`` alone -- it installs
    ``default_int_handler`` only over the inherited *default*. Observed on a
    live supervised run, 2026-09-09: ``signal.getsignal(SIGINT)`` was
    ``signal.SIG_IGN`` and ``kill -INT`` was swallowed outright, so there was no
    graceful way to stop a 30-minute hold. The obvious next thing to reach for
    is worse: the default SIGTERM disposition kills the process WITHOUT
    unwinding, so the ``finally`` that calls ``relay.safe()`` never runs and the
    PLC is left with C1 (PID Auto) ENABLED and the bath holding whatever setpoint
    it last received.

    So both signals are pointed at the one path that is already correct.
    ``_hold``'s ``except KeyboardInterrupt`` returns ABORTED and the outer
    ``finally`` writes the safe setpoint and drives C1 False -- the same thing
    an operator's Ctrl-C has always done.

    Main thread only: ``signal.signal`` raises ``ValueError`` anywhere else, and
    a run driven from a worker thread should still run rather than refuse. A
    handler that cannot be installed is skipped for the same reason -- the run
    is still bounded by ``--duration``.
    """
    previous: dict = {}
    if threading.current_thread() is not threading.main_thread():
        return previous
    for signum, handler in ((signal.SIGINT, signal.default_int_handler),
                            (signal.SIGTERM, _stop_on_signal)):
        try:
            previous[signum] = signal.signal(signum, handler)
        except (OSError, ValueError):
            continue
    return previous


def _restore_stop_handlers(previous: dict) -> None:
    """Put back exactly what :func:`_install_operator_stop_handlers` replaced."""
    for signum, handler in previous.items():
        try:
            signal.signal(signum, handler)
        except (OSError, ValueError):
            continue


def _controller_block(record_run: runs.Run, started_unix_s: float) -> dict:
    """Who is holding the chamber, for the dashboard's own header.

    ``valve_mode``/``valve_note`` are reported as facts about what this
    controller can do, not as a valve state: see :data:`VALVE_NOTE`.
    """
    return {
        "pid": os.getpid(),
        "run_dir": str(record_run.path),
        "started_unix_s": float(started_unix_s),
        "valve_mode": VALVE_MODE,
        "valve_note": VALVE_NOTE,
    }


def _reading_of(record: env.RelayRecord) -> env.EnvironmentReading:
    """Re-decode the reading the relay just acted on, out of its own record.

    ``RelayRecord`` carries the reading as a dict and ``publish_last_reading``
    takes the object, so one of them has to give. Re-decoding is the cheap side:
    ``raw_registers`` is kept in the record for exactly this ("any channel can
    be re-decoded later from the record without going back to the PLC" --
    ``tools/environment/reading.py``), and ``PlcClient.read_block`` builds a
    FAILED read the same way, as ``decode_block(())``, so the round trip is
    exact on that path too.

    The alternative -- a second ``read_block()`` per period -- costs one of the
    CLICK's three Modbus sockets on every iteration AND publishes a different
    read than the one the loop actually forwarded. That second problem is the
    real one: the dashboard would be showing a reading no decision was made on.
    """
    reading = record.reading
    return env.decode_block(reading.get("raw_registers", ()),
                            t_utc=record.t_utc, monotonic_s=record.monotonic_s,
                            pid_enabled=reading.get("pid_enabled"))


def _publish(client: env.PlcClient, record_run: runs.Run, started_unix_s: float,
             reading: env.EnvironmentReading, last_command: dict | None) -> None:
    """Put this iteration's reading where the dashboard reads it.

    THE LOOP HAS TO DO THIS, AND UNTIL 2026-09-09 NOTHING DID.
    ``publish_last_reading`` was called only by the read helpers in
    ``tools/environment/api.py`` and by the node's own read -- so during a
    supervised run, the one time the dashboard *cannot* open a Modbus session of
    its own (this process holds the ``environment`` claim, and the CLICK has
    three sockets), the panel rendered whatever was left over from the last
    manual read. A 30-minute hold showed a 30-minute-old chamber and no way to
    tell.

    Never fatal. A full disk or a vanished publish directory is logged and the
    hold continues: the run's own record under ``dataset/`` is the authority and
    the published file is a projection of it, so losing the projection is not a
    reason to stop controlling a chamber.
    """
    try:
        client.publish_last_reading(
            reading,
            extra={"controller": _controller_block(record_run, started_unix_s),
                   "last_command": last_command})
    except (OSError, env.SafetyError) as exc:
        record_run.log("publish_last_reading failed (%s: %s); the dashboard will "
                       "keep showing the previous reading"
                       % (type(exc).__name__, exc), device=DEVICE, level="warn")


def _apply_setpoint(cmd: env.Command, client: env.PlcClient,
                    args: argparse.Namespace, record_run: runs.Run,
                    limiter: env.RateLimiter,
                    history: dict[str, tuple[float, float]],
                    live: bool) -> tuple[str, str]:
    """Write the setpoint(s) a command asked for, through the CLI's own path.

    The bound and the FC16-with-read-back come from ``set_temperature`` /
    ``set_humidity`` -- literally the functions ``--temp-sp`` and ``--rh-sp``
    call, reached through :data:`SETPOINT_WRITERS` so the two paths cannot
    diverge.

    The rate limiter is applied HERE because it has to be:
    ``tools/environment/api.py`` says so outright -- it needs the previous
    write's value and time, which a stateless one-shot helper does not have, and
    inventing "no previous write" on every call would make the interval rule
    vacuous while looking enforced. ``history`` is that memory, shared with the
    ``--temp-sp``/``--rh-sp`` writes at the start of the run.

    BOTH keys of a two-key command are bounded and rate-checked before EITHER
    reaches the wire. Otherwise a command whose second value is out of range
    leaves the first one written and still reports refused, and the run record
    then disagrees with the chamber -- which is the failure this whole layer is
    built to avoid.
    """
    now = time.time()
    planned = []
    for key, flag, field, _attr, setter in SETPOINT_WRITERS:
        if key not in cmd.args:
            continue
        value = float(cmd.args[key])
        # The same SetpointLimits.validate the writer re-applies at the sink,
        # called early only so a refusal cannot half-apply a two-key command.
        # The token it returns is deliberately discarded: this is the check, not
        # the write.
        client.settings.limits.validate(field, value)
        last_value, last_time = history.get(field, (None, None))
        limiter.check(field, value, now=now, last_value=last_value,
                      last_time=last_time)
        planned.append((flag, field, setter, value))

    outcomes: list[str] = []
    details: list[str] = []
    for flag, field, setter, value in planned:
        result = setter(value, args.config, client=client, dry_run=not live)
        record_run.record(DEVICE, "setpoint_writes", result.as_dict())
        if result.outcome == env.CONFIRMED:
            history[field] = (value, time.time())
        outcomes.append(result.outcome)
        details.append("%s %s" % (flag, result.describe()))

    detail = " | ".join(details)
    # A two-key command reports BOTH halves either way, and the worst outcome
    # wins: "one of them failed" must not be summarised as confirmed.
    if env.FAILED in outcomes:
        return env.channel.OUTCOME_FAILED, detail
    if outcomes and all(outcome == env.CONFIRMED for outcome in outcomes):
        return env.channel.OUTCOME_CONFIRMED, detail
    return env.channel.OUTCOME_PLANNED, detail


def _apply_pid(cmd: env.Command, client: env.PlcClient, relay: env.Relay | None,
               live: bool) -> tuple[str, str]:
    """Turn the PLC's PID loop on or off. Off is always allowed; on is not.

    Disabling is the safe direction and ``PlcClient.set_pid`` permits it with
    ``allow_actuation`` false, so a dashboard that may not write a setpoint is
    still a dashboard that can stop the loop.

    ENABLING is refused while the relay is latched in safe mode. The latch means
    the pre-declared fail-safe has already executed -- the safe setpoint is on
    the bath and C1 is off -- and re-closing the loop from a dashboard button
    would resume control across a link the watchdog just declared unusable. The
    relay refuses to rearm itself for that reason (F1); this refuses for the
    same one. A run is restarted instead, deliberately, by a person.
    """
    if not cmd.args["enabled"]:
        result = client.set_pid(False, dry_run=not live)
    elif relay is not None and relay.safe_mode:
        return env.channel.OUTCOME_REFUSED, ("relay is latched in safe mode; "
                                             "restart the run to re-arm")
    else:
        result = client.set_pid(True, dry_run=not live)
    return result.outcome, result.describe()


def _dispatch(cmd: env.Command, client: env.PlcClient, relay: env.Relay | None,
              args: argparse.Namespace, record_run: runs.Run,
              limiter: env.RateLimiter,
              history: dict[str, tuple[float, float]],
              live: bool) -> tuple[str, str]:
    """Apply one shape-validated command. Returns ``(outcome, detail)``.

    REFUSED and FAILED are told apart deliberately. Refused means a bound, a
    gate, or a rate limit turned the command down -- a guard did its job, and
    the operator needs to see which one. Failed means it was attempted and did
    not verify, which is a different conversation and a different next step.
    """
    try:
        if cmd.name == "setpoint":
            return _apply_setpoint(cmd, client, args, record_run, limiter,
                                   history, live)
        if cmd.name == "pid":
            return _apply_pid(cmd, client, relay, live)
        # "valve" -- always refused; see VALVE_NOTE. Refused out loud rather
        # than accepted and ignored, because a control that reports success
        # while doing nothing is how an operator comes to believe a valve moved.
        return env.channel.OUTCOME_REFUSED, VALVE_NOTE
    except (env.SafetyError, ValueError) as exc:
        # env.SafetyError covers RateLimited and ActuationNotAllowed, both
        # subclasses of it; ValueError covers channel.ChannelError (a subclass)
        # and a malformed float. All of them are a guard refusing.
        return env.channel.OUTCOME_REFUSED, "%s: %s" % (type(exc).__name__, exc)
    except Exception as exc:                                        # noqa: BLE001
        return env.channel.OUTCOME_FAILED, "%s: %s" % (type(exc).__name__, exc)


def _take_command(client: env.PlcClient, relay: env.Relay | None,
                  args: argparse.Namespace, record_run: runs.Run,
                  limiter: env.RateLimiter,
                  history: dict[str, tuple[float, float]],
                  live: bool) -> dict | None:
    """Consume at most one dashboard command, apply it, and return its record.

    ``None`` means nothing was pending. Otherwise the return value is the
    ``last_command`` block the caller publishes; the contract for it is in
    :mod:`tools.environment.channel`.

    NOTHING A COMMAND DOES MAY END THE HOLD. A bad value, a shut gate, a
    rate-limit refusal, a stale intent, even a hand-written ``command.json``
    that is not valid JSON: each becomes an outcome and the loop comes round
    again. ``BaseException`` is deliberately not caught -- the operator's stop
    signal arrives as a KeyboardInterrupt and must still stop the run.
    """
    try:
        cmd = env.channel.take()
    except (env.channel.ChannelError, OSError) as exc:
        # take() raises BEFORE it unlinks, so an unreadable command.json would
        # be re-read and re-refused every period for the rest of the run.
        # Remove it: nothing in this channel is ever replayed, and the
        # dashboard's next write replaces the file anyway.
        poison = env.channel.channel_dir() / env.channel.COMMAND_NAME
        try:
            poison.unlink()
        except OSError:
            pass
        # log() echoes to stdout by default, so the operator sees this without a
        # second print -- which would put the same refusal on the console twice.
        record_run.log("REFUSED an unreadable dashboard command: %s" % exc,
                       device=DEVICE, level="warn")
        # Reported with seq and name unknown rather than through
        # outcome_record(): there is no Command to build one from, and claiming
        # a seq we could not read would be worse than saying we could not.
        record = {"seq": None, "issued_unix_s": None, "name": "UNREADABLE",
                  "args": {}, "source": "unknown",
                  "outcome": env.channel.OUTCOME_REFUSED, "detail": str(exc),
                  "finished_unix_s": time.time()}
        record_run.record(DEVICE, "commands", record)
        return record
    if cmd is None:
        return None

    if env.channel.is_stale(cmd):
        # Never applied, and reported as its own outcome rather than as a
        # refusal: a queued intent must not fire after a restart or a long
        # stall, and the operator should see that it expired rather than that
        # something rejected it.
        outcome = env.channel.OUTCOME_STALE
        detail = ("issued %.1f s ago, past channel.STALE_AFTER_S=%g; NOT applied"
                  % (time.time() - cmd.issued_unix_s, env.channel.STALE_AFTER_S))
    else:
        outcome, detail = _dispatch(cmd, client, relay, args, record_run,
                                    limiter, history, live)

    record = env.channel.outcome_record(cmd, outcome, detail)
    record_run.record(DEVICE, "commands", record)
    # One line, logged once: Run.log echoes to stdout itself.
    record_run.log("command %s#%d %r -> %s: %s"
                   % (cmd.name, cmd.seq, cmd.args, outcome, detail),
                   device=DEVICE)
    return record


def run(args: argparse.Namespace, *, _plc_transport: object = None,
        _circ_transport: object = None) -> int:
    """Hold the chamber, relaying DF9 to the bath.

    ``_plc_transport`` / ``_circ_transport`` are a TEST-ONLY seam: when given,
    they are injected as the Modbus and serial transports so a test can drive
    this runner end-to-end over FakePlc/FakeSerial without opening a socket or
    the circulator's port (opening that port resets the MCU). Both default to
    ``None`` -- the real ``main()`` path builds real clients.
    """
    # Wall time, not monotonic: it is published for the dashboard to render as
    # "holding since", and a monotonic value means nothing in another process.
    started_unix_s = time.time()

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

    # The rate limiter is THIS SCRIPT's, per tools/environment/api.py, and it is
    # stateful: `history` maps a field to the (value, unix time) of its last
    # CONFIRMED write. It spans the --temp-sp/--rh-sp writes below AND every
    # dashboard setpoint command the loop consumes, so the step and interval
    # caps mean something across both instead of resetting per caller.
    limiter = plc_settings.rate_limiter
    history: dict[str, tuple[float, float]] = {}

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
        record_run.log(limiter.describe(), device=DEVICE)
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
            client = env.PlcClient(plc_settings, _plc_transport)
            # ARM THE INTERLOCK BEFORE ANY ACTUATING CALL. The relay -- and the
            # bath it drives -- are built here, AHEAD of the setpoint writes and
            # the port open, so a crash ANYWHERE in the setup below still reaches
            # `finally: relay.safe()`, which disables the PID. The live run that
            # motivated this enabled the PID and then crashed opening the port;
            # with no relay yet built, the finally had nothing to turn C1 off
            # with, and the PLC was left with its PID ENABLED. Constructing a
            # Circulator/Relay does not actuate: no port is opened and no coil is
            # written until the guarded calls below.
            if not args.no_relay:
                bath_device = circ.Circulator(
                    bath_settings, _circ_transport, log=record_run.logger(BATH))
                relay = env.Relay(client, bath_device, policy)
            # Live path only, and only now that the interlock above exists: from
            # here on a stop signal raises KeyboardInterrupt, which reaches the
            # `finally` below and disables the PID. Restored there too, so a
            # caller that drives run() twice in one process is not left holding
            # this run's handlers.
            stop_handlers = _install_operator_stop_handlers() if live else {}
            try:
                opened = client.connect()
                record_run.log("connect -> %r (last_error=%r)"
                               % (opened, client.last_error), device=DEVICE)

                # (1) setpoints -- the first actuating step in a live run.
                if not _write_setpoints(args, client, record_run, live,
                                        history):
                    record_run.note("a setpoint write was not confirmed; the "
                                    "hold loop ran anyway, so the PLC may have "
                                    "been holding a different setpoint than "
                                    "this run asked for.")
                    exit_code = 1

                # (2) open the bath port. Opening it hardware-RESETS the MCU, so
                # it is done ONCE, here, explicitly -- never on demand from
                # inside the loop.
                if not args.no_relay and live:
                    record_run.log("opening the serial port -- this RESETS "
                                   "the MCU", device=BATH)
                    record_run.record(BATH, "port", bath_device.open())

                # (3)+(4) enable the PID LAST -- only after the setpoints are
                # written, the bath port is open, the relay is built, and a first
                # live PLC read succeeds. A kinetics run must CLOSE the loop; a
                # plain hold does not. Enabling it last means that if any earlier
                # step failed, the PID was never enabled and there is nothing to
                # leave unsafe -- and if it IS enabled, the relay built above is
                # already in place to disable it in `finally`.
                if getattr(args, "enable_pid", False):
                    if not live:
                        record_run.note("--enable-pid was requested but this "
                                        "is a DRY RUN; the PID was NOT enabled.")
                    else:
                        first = client.read_block()
                        record_run.record(DEVICE, "readings", first.as_dict())
                        if first.read_ok is not True:
                            record_run.note(
                                "--enable-pid was requested but the first PLC "
                                "read did not succeed (last_error=%r); the PID "
                                "was NOT enabled, so nothing was left in an "
                                "actuated state." % client.last_error)
                            exit_code = 1
                        else:
                            pid_res = env.enable_pid(args.config, client=client)
                            record_run.record(DEVICE, "pid_enable",
                                              pid_res.as_dict())
                            record_run.log("enable PID (C1): %s"
                                           % pid_res.describe(), device=DEVICE)
                            print(pid_res.describe())
                            if pid_res.outcome == env.FAILED:
                                record_run.note("--enable-pid FAILED (%r); the "
                                                "PLC loop may not be closed."
                                                % pid_res.error)
                                exit_code = 1

                # (5) the loop.
                if args.no_relay:
                    deadline = time.monotonic() + args.duration
                    last_command: dict | None = None
                    while time.monotonic() < deadline:
                        # Dashboard commands are honoured here too: --no-relay
                        # drops the forward to the bath, not the supervisor.
                        outcome = _take_command(client, None, args, record_run,
                                                limiter, history, live)
                        if outcome is not None:
                            last_command = outcome
                        reading = client.read_block()
                        record_run.record(DEVICE, "readings", reading.as_dict())
                        _publish(client, record_run, started_unix_s, reading,
                                 last_command)
                        print("read_ok=%r partial=%r temp=%r rh=%r pid=%r"
                              % (reading.read_ok, reading.partial,
                                 reading.temp_filtered_c, reading.rh_filtered_pct,
                                 reading.pid_enabled))
                        time.sleep(period)
                else:
                    forwarded, refused, exit_code, status = _hold(
                        relay, record_run, args, period, exit_code,
                        client=client, started_unix_s=started_unix_s,
                        live=live, limiter=limiter, history=history)
            finally:
                _restore_stop_handlers(stop_handlers)
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
          period: float, exit_code: int, *, client: env.PlcClient,
          started_unix_s: float, live: bool, limiter: env.RateLimiter,
          history: dict[str, tuple[float, float]]) -> tuple[int, int, int, str]:
    """The loop. Returns ``(forwarded, refused, exit_code, status)``.

    Every decision in here is this file's: one `step()` per period, no immediate
    retry, no re-send of a stale value, and `--stop-on-stale` deciding what
    happens after a watchdog has already executed the fail-safe.

    Three things happen per period, in this order and for these reasons:

    1. **One pending dashboard command is consumed, BEFORE the step.** So a
       setpoint the operator just changed is on the PLC before the reading this
       iteration publishes -- otherwise the panel would show the old setpoint
       for a full period after acknowledging the command, which reads as the
       command having been lost.
    2. **`relay.step()`** -- one read, one forward or one refusal.
    3. **The reading is published**, with who is holding the chamber and what
       became of the last command. This is the only thing that keeps the
       dashboard current during a run: it cannot open a Modbus session of its
       own while this process holds the `environment` claim.
    """
    forwarded = refused = 0
    status = runs.STATUS_OK
    #: The most recent command's outcome record, republished every iteration
    #: until another command replaces it, so the panel keeps showing what
    #: happened rather than blanking a second later. `None` until one arrives.
    last_command: dict | None = None
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            outcome = _take_command(client, relay, args, record_run, limiter,
                                    history, live)
            if outcome is not None:
                last_command = outcome
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
                    # Published as well: a watchdog firing is exactly when the
                    # operator is looking at the panel, and safe_mode reaching
                    # it a period late is a period spent wondering.
                    _publish(client, record_run, started_unix_s,
                             _reading_of(attached), last_command)
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
            _publish(client, record_run, started_unix_s, _reading_of(record),
                     last_command)
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
