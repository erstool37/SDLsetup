#!/usr/bin/env python3
"""
environment_probe.py -- read the CLICK PLC and record everything. READ-ONLY.

WHAT IT DOES
  1. Claim `occupancy("environment")` -- the CLICK accepts at most THREE
     concurrent Modbus TCP clients and refuses the fourth, so two probes or a
     probe beside a running supervisor is how a run loses its connection.
  2. Open one session and read the DF1..DF23 block plus coil C1, `--samples`
     times at `--interval` seconds.
  3. Print each channel with its PLC nickname, value, units and PROVENANCE, so
     no number on screen can be mistaken for something better-attested than it
     is; then `provenance()`, `PlcSettings.describe()` and `diagnostics()`.
  4. Record it all through `tools.runs.Run`, RAW REGISTERS INCLUDED, so any
     value can be re-decoded later without going back to the PLC.

THERE IS NO WRITE PATH IN THIS FILE, NOT EVEN A GATED ONE
  No setpoint write, no coil write, no `set_pid`. The read side of
  `tools.environment.api` is the only thing imported that touches the wire.
  Writing lives in `hold_environment.py`, behind `--execute` and
  `environment.allow_actuation`. A probe that *could* write is a probe someone
  will eventually run with the wrong flag against a live control loop.

FIRST ADOPTER OF tools.runs.Run
  This and `hold_environment.py` are the first phase scripts in the repo to
  open a `Run`. That is a deliberate choice, not an accident of being new: an
  environment probe's whole value is the record it leaves, and the manifest --
  entrypoint, commit, dirty diff, resolved config, environment, notes -- is
  what makes a reading citable six months on. The microscope scripts still
  write their own timestamped directories; migrating them is separate work.

WHAT THIS RUN DOES NOT VERIFY
  Recorded with `run.note()` rather than left to silence, because silence reads
  as "verified" to the next person:
    * channel IDENTITY beyond DF1/DF2. The read goes through the register map,
      so it cannot confirm the map. DF1/DF2 are corroborated only because the
      swapped reading would be 70.8 C, impossible for a room.
    * the SC90-SC95 / SD41 native diagnostics. Their nicknames are known and
      their Modbus addresses are NOT, so they are reported unavailable rather
      than read from a guessed address.
    * anything about the circulator. This script does not open its serial port
      (opening it hardware-resets the MCU) and says nothing about the bath.

    python scripts/environment/environment_probe.py --samples 3
    python scripts/environment/environment_probe.py --json
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from tools import environment as env  # noqa: E402
from tools import occupancy, runs  # noqa: E402

DEVICE = "environment"
PHASE = "environment_probe"

#: What this run did not check. Written into the manifest via run.note().
UNVERIFIED = (
    "channel IDENTITY beyond DF1/DF2 is UNCONFIRMED. This read decodes through "
    "tools.environment.registers, so it cannot corroborate that table -- only "
    "what the registers held. DF1/DF2 are the exception: the swapped reading "
    "would be 21.8 %RH and 70.8 C, and 70.8 C is impossible for this room. "
    "Confirming the rest needs the PLC's own data view, an independent source.",
    "the native SC90-SC95 and SD41 diagnostics were NOT read. Their nicknames "
    "are known; their Modbus addresses are not, and an invented address reads "
    "some other register and reports it as a link flag. diagnostics() reports "
    "each one 'unavailable: address unknown' for that reason.",
    "DF3/DF4 MEANING remains UNKNOWN. They have no nickname in the PLC "
    "project. A reading of exactly 0.000 means 0 mA on a 0-20 mA input, i.e. "
    "nothing is currently wired there -- which is a fact about the wiring, not "
    "an interpretation of the channel.",
    "nothing was verified about the circulator. This script never opens its "
    "serial port -- opening asserts DTR on the FT232R and hardware-RESETS the "
    "MCU -- so it says nothing about the bath or what it last received.",
    "this is a READ-ONLY probe. It asserts nothing about whether a setpoint "
    "write would succeed, and it did not attempt one.",
)


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--samples", type=int, default=1,
                    help="how many block reads to take (default: 1)")
    ap.add_argument("--interval", type=float, default=1.0,
                    help="seconds between samples (default: 1.0)")
    ap.add_argument("--json", action="store_true",
                    help="also print each decoded reading as JSON on stdout")
    ap.add_argument("--config", default=str(REPO / "configs" / "config.yaml"),
                    help="config file to resolve settings from")
    return ap


def _channel_rows(record: env.EnvironmentReading) -> list[tuple[str, ...]]:
    """One row per register, in DF order, carrying provenance beside the value."""
    rows: list[tuple[str, ...]] = [
        ("DF", "PLC nickname", "field", "value", "units", "provenance",
         "verified", "quality")]
    for spec in env.REGISTERS:
        if spec.field is None:
            unknown = record.unknown_channels.get("DF%d" % spec.df)
            value = "-" if unknown is None else "%.6g" % unknown["value"]
            rows.append((str(spec.df), spec.nickname, "(none)", value,
                         "UNKNOWN", str(spec.provenance),
                         "yes" if spec.verified_live else "no",
                         "MEANING UNKNOWN"))
            continue
        if spec.field not in record.channels:
            rows.append((str(spec.df), spec.nickname, spec.field, "ABSENT",
                         spec.units or "-", str(spec.provenance),
                         "yes" if spec.verified_live else "no",
                         "did not arrive"))
            continue
        quality = record.quality[spec.field]
        flags = []
        if not quality.finite:
            flags.append("NOT FINITE")
        if quality.in_valid_range is False:
            flags.append("OUT OF VALID RANGE")
        elif quality.in_valid_range is None:
            flags.append("no range declared")
        rows.append((
            str(spec.df), spec.nickname, spec.field,
            "%.6g" % record.channels[spec.field], spec.units or "-",
            str(spec.provenance), "yes" if spec.verified_live else "no",
            ", ".join(flags) if flags else "in range",
        ))
    # env.registers rather than the api surface: COIL_SPEC is not re-exported
    # by tools/environment/api.py, and the coil's provenance belongs in this
    # table beside every register's. tools.environment.__all__ does export the
    # submodule, so this is not reaching into a private name.
    # TODO(operator): add COIL_SPEC (and MEASURED_OUTPUT_CLAMPS/MEASURED_DATE)
    # to api.py's surface -- api.py was outside this change's write scope.
    coil = env.registers.COIL_SPEC
    rows.append(("C1", coil.nickname, coil.field,
                 repr(record.pid_enabled), "-", str(coil.provenance),
                 "yes" if coil.verified_live else "no",
                 "None means NOT READ, which is not the same as off"))
    return rows


def _table(rows: list[tuple[str, ...]]) -> str:
    widths = [max(len(row[i]) for row in rows) for i in range(len(rows[0]))]
    lines = []
    for index, row in enumerate(rows):
        lines.append("  " + "  ".join(
            cell.ljust(widths[i]) for i, cell in enumerate(row)).rstrip())
        if index == 0:
            lines.append("  " + "  ".join("-" * w for w in widths))
    return "\n".join(lines)


def run(args: argparse.Namespace) -> int:
    if args.samples < 1:
        print("--samples must be at least 1", file=sys.stderr)
        return 2
    if args.interval < 0.0:
        print("--interval must not be negative", file=sys.stderr)
        return 2

    settings = env.PlcSettings.from_config(args.config)

    with runs.Run.create(PHASE, config=args.config, argv=sys.argv,
                         devices=(DEVICE,),
                         description="read-only probe of the CLICK PLC") as record_run:
        for text in UNVERIFIED:
            record_run.note(text)

        record_run.log("READ-ONLY probe. This script has no write path at all.",
                       device=DEVICE)
        record_run.log(settings.describe(), device=DEVICE)
        record_run.log(env.provenance(), device=DEVICE)

        diagnostics: dict = {}
        readings: list[env.EnvironmentReading] = []
        usable = 0

        # One claim around the whole session: claiming per-read would let a
        # second client slip in between two of them, and the CLICK has only
        # three sockets to spend.
        with occupancy.claim(DEVICE, doing="%s (%d read-only samples)"
                                           % (PHASE, args.samples)):
            client = env.PlcClient(settings)
            try:
                opened = client.connect()
                record_run.log("connect -> %r (last_error=%r)"
                               % (opened, client.last_error), device=DEVICE)
                diagnostics = client.diagnostics()

                for index in range(args.samples):
                    if index:
                        time.sleep(args.interval)
                    reading = client.read_block()
                    readings.append(reading)
                    # Raw registers go in, so any channel can be re-decoded
                    # later without re-reading the PLC.
                    record_run.record(DEVICE, "readings", reading.as_dict())

                    # read_ok is NOT a sufficient gate: decode_block sets it
                    # True for a SHORT block too. Usability needs partial and
                    # the specific channel checked as well.
                    complete = reading.read_ok and not reading.partial
                    if complete:
                        usable += 1
                    print()
                    print("sample %d/%d  t_utc=%s  read_ok=%r partial=%r "
                          "words=%d pid_auto=%r"
                          % (index + 1, args.samples, reading.t_utc,
                             reading.read_ok, reading.partial,
                             len(reading.raw_registers), reading.pid_enabled))
                    if not complete:
                        print("  NOT A COMPLETE BLOCK -- read_ok alone is not "
                              "evidence a channel is trustworthy. last_error=%r"
                              % client.last_error)
                    print(_table(_channel_rows(reading)))
                    if args.json:
                        print(json.dumps(reading.as_dict(), indent=2, default=str))
            finally:
                client.close()

        print()
        print(settings.describe())
        print()
        print(env.provenance())
        print("diagnostics:")
        print(json.dumps(diagnostics, indent=2, default=str))

        record_run.write_json(DEVICE, "diagnostics", diagnostics)
        record_run.write_json(DEVICE, "settings", {
            "describe": settings.describe(),
            "sources": dict(settings.sources),
        })
        record_run.write_json(DEVICE, "provenance", {
            "table": env.provenance(),
            "measured_output_clamps": dict(env.registers.MEASURED_OUTPUT_CLAMPS),
            "measured_output_clamps_date": env.registers.MEASURED_DATE,
            "measured_output_clamps_note":
                "the PLC's own clamps on its own PID outputs, read live on that "
                "date. Not limits this layer enforces.",
        })

        record_run.metric("samples_requested", args.samples)
        record_run.metric("samples_complete", usable)
        last = readings[-1] if readings else None
        if last is not None and not last.partial and last.read_ok:
            for field in ("temp_filtered_c", "rh_filtered_pct", "temp_sp_c",
                          "rh_sp_pct", "temp_pid_output_c", "temp_output_lb_c",
                          "temp_output_ub_c"):
                if field in last.channels:
                    record_run.metric("last_%s" % field, last.channels[field])
            record_run.metric("last_pid_auto", last.pid_enabled)
        else:
            record_run.note("no COMPLETE block arrived, so no channel value is "
                            "asserted as a metric. last_error=%r"
                            % (client.last_error,))

        print()
        print("run: %s" % record_run.path)
        print("     %d of %d samples were complete blocks"
              % (usable, args.samples))

    # Nothing was written to the PLC either way, so a failed read is reported
    # as a non-zero exit and nothing else needs undoing.
    return 0 if usable == args.samples else 1


def main(argv=None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
