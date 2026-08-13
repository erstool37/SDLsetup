#!/usr/bin/env python3
"""hunt_a1.py -- walk along X at a pinned Z until the printed mark appears.

    python scripts/microscope/hunt_a1.py                    # plan only
    python scripts/microscope/hunt_a1.py --execute
    python scripts/microscope/hunt_a1.py --execute --span -35 --step 1.5

WHY
---
The rig was physically moved on 2026-08-06 and the operator estimates the plate
is now about 30 mm further along -X. The taught A1 is therefore wrong by roughly
that much, and no anchored envelope can express a 30 mm excursion -- they box XY
to at most 9 mm around the anchor that is itself wrong.

Rather than write an estimate into the taught state and call it calibration,
this walks the estimate out: step along X only, hold Z exactly, look at every
frame with the same trigger the grid search uses, and stop on the first
size-plausible mark. The pose read back THERE is what gets taught -- a
measurement, not an assumption.

SAFETY
------
* Z is PINNED. The envelope is ``Envelope.free`` with z_floor == z_ceiling ==
  the current height, so no commanded pose can change Z by any amount. The
  objective is directly above; X is the only axis that may move.
* Y is held at its current value and re-checked on every readback.
* Every move is verified with ``MoveResult.verify()`` -- this arm can refuse a
  move silently (measured: stopped 29 mm short with no latched fault), and a
  hunt that believes a refused step would report the mark at the wrong X.
* Dry-run by default.
* It teaches NOTHING. It reports the X where the mark was seen; writing that
  into ``workspace.json`` is a separate, explicit step.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from tools.arm.api import Arm, ArmSettings  # noqa: E402
from tools.arm.safety import Envelope, SafetyError  # noqa: E402
from tools.microscope import Microscope, MicroscopeSettings  # noqa: E402
from tools.microscope import marker as _marker  # noqa: E402

_spec = importlib.util.spec_from_file_location(
    "find_target", REPO / "scripts" / "microscope" / "find_target.py")
_ft = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = _ft
_spec.loader.exec_module(_ft)

FRAME_TIMEOUT_S = 25.0


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__.split("WHY")[0].strip(),
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--execute", action="store_true")
    ap.add_argument("--host", default=None)
    ap.add_argument("--span", type=float, default=-35.0,
                    help="total X travel, signed. Negative walks -X "
                         "(default: %(default)s)")
    ap.add_argument("--step", type=float, default=1.5,
                    help="X step, mm. Smaller than the ~2.8 mm field so the "
                         "mark cannot fall between two frames (default: %(default)s)")
    ap.add_argument("--speed", type=float, default=5.0)
    ap.add_argument("--settle", type=float, default=0.4)
    ap.add_argument("--dot-mm", type=float, default=1.0)
    ap.add_argument("--px-per-mm", type=float, default=_ft.TRIGGER_PX_PER_MM)
    ap.add_argument("--frame-ordinal", type=int, default=2)
    ap.add_argument("--out-dir", default=None)
    return ap


def run(args) -> int:
    settings = ArmSettings.from_config(str(REPO / "configs" / "config.yaml"),
                                       host=args.host, live=True)
    arm = Arm(settings)
    here = list(arm.status()["pose"])
    x0, y0, z0 = here[0], here[1], here[2]

    n = max(1, int(abs(args.span) / abs(args.step)))
    sign = 1.0 if args.span >= 0 else -1.0
    xs = [x0 + sign * abs(args.step) * i for i in range(1, n + 1)]

    prior = _marker.MarkPrior(diameter_mm=args.dot_mm, px_per_mm=args.px_per_mm)
    lo, hi = prior.band_px

    print("=" * 72)
    print("hunt A1 along X    mode: %s"
          % ("LIVE -- THE ARM WILL MOVE" if args.execute else "DRY-RUN"))
    print("start       x=%.4f y=%.4f z=%.4f" % (x0, y0, z0))
    print("Z is PINNED at %.4f (free envelope, floor == ceiling)" % z0)
    print("walk        %+.1f mm in %d step(s) of %.2f mm -> x ends at %.4f"
          % (args.span, n, abs(args.step), xs[-1] if xs else x0))
    print("trigger     %.2f mm mark at %.1f px/mm -> accept radius %.0f-%.0f px"
          % (args.dot_mm, args.px_per_mm, lo, hi))
    print("            RED/BLUE stops the hunt; a dark-only hit is logged and the")
    print("            walk continues (dark alone false-positived twice today)")
    print("=" * 72)

    if not args.execute:
        print("\n[dry-run] nothing commanded. Re-run with --execute.")
        return 0

    out = Path(args.out_dir) if args.out_dir else (
        REPO / "dataset" / "captures" / "hunt_a1" / time.strftime("%Y%m%d-%H%M%S"))
    out.mkdir(parents=True, exist_ok=True)

    env = Envelope.free(
        reason=("hunt for the relocated plate: X-only walk at a pinned Z. The "
                "rig moved ~30 mm on 2026-08-06 and no anchored envelope can "
                "express that, because the anchor itself is what is wrong."),
        z_floor_mm=z0, z_ceiling_mm=z0, speed_max_mm_s=max(args.speed, 5.0))
    scope = Microscope(MicroscopeSettings.from_config(
        str(REPO / "configs" / "config.yaml")))

    record = {"start_pose": here, "span_mm": args.span, "step_mm": args.step,
              "px_per_mm": args.px_per_mm, "dot_mm": args.dot_mm,
              "band_px": [lo, hi], "visited": []}
    try:
        with arm.occupied("hunt A1 along X"):
            # Built INSIDE the claim: with_envelope() copies the holder at
            # construction time, so a view made before the claim carries
            # _held=None and its per-move claim then deadlocks against the
            # outer one held by this same process.
            walker = arm.with_envelope(env)
            for i, x in enumerate(xs, 1):
                label = "hunt %d/%d x=%.3f" % (i, len(xs), x)
                try:
                    res = walker.move(x, y0, z0, speed=args.speed, label=label,
                                      takeup=False)
                except SafetyError as exc:
                    print("  [%d/%d] REFUSED: %s" % (i, len(xs), exc))
                    record["visited"].append({"x": x, "status": "refused",
                                              "reason": str(exc)})
                    break

                arrival = res.verify()
                if arrival["moved"] is False:
                    print("  [%d/%d] x=%.3f DID NOT ARRIVE (%s). Stopping: a hunt "
                          "that trusts a refused step reports the mark at the "
                          "wrong X." % (i, len(xs), x, arrival["reason"]))
                    record["visited"].append({"x": x, "status": "not_arrived",
                                              "arrival": arrival})
                    break

                moved_at = time.time()
                time.sleep(args.settle)
                dest = out / ("hunt_%03d_x%+08.3f.jpg" % (i, x))
                frame = scope.grab_frame(dest, after=moved_at,
                                         ordinal=args.frame_ordinal,
                                         timeout_s=FRAME_TIMEOUT_S)
                if not frame.ok:
                    print("  [%d/%d] x=%.3f no frame: %s" % (i, len(xs), x, frame.reason))
                    record["visited"].append({"x": x, "status": "no_frame",
                                              "reason": frame.reason})
                    continue

                # ALL THREE discriminants, not just "dark".
                #
                # The colour ones are far more specific and the rig's own record
                # says so: through the 2026-08-04 run that produced 21 dark
                # false positives in 28 frames, red and blue correctly reported
                # NOTHING. They key on a LOCAL excess of one channel over the
                # max of the other two, which a machined edge, a bore rim or a
                # shadow cannot produce -- whereas all three of those are dark.
                #
                # So a colour sighting stops the hunt on its own. A dark-only
                # sighting is logged and the walk CONTINUES: measured this
                # session, dark alone stopped on a plate edge (x=-8.6) and a
                # well rim (x=-13.1), neither of which was a mark.
                verdict = _marker.classify_dots(frame.path, expect=prior)
                colours = [c for c in (verdict.get("prior_matched") or [])
                           if c in ("red", "blue")]
                dark_hit = _ft.dot_trigger(frame.path, prior)
                entry = {"x": x, "status": "captured", "frame": str(dest),
                         "present": list(verdict.get("present") or []),
                         "prior_matched": list(verdict.get("prior_matched") or []),
                         "dot_trigger": dark_hit}
                record["visited"].append(entry)

                if colours:
                    rec = (verdict.get("colours") or {}).get(colours[0]) or {}
                    print("\n  [COLOUR MARK] x=%.4f  %s dot, margin %s"
                          % (x, colours[0].upper(), rec.get("margin")))
                    print("  frame: %s" % dest)
                    record["found_x"] = x
                    record["found_colour"] = colours[0]
                    record["found_frame"] = str(dest)
                    break

                note = ""
                if dark_hit:
                    note = ("  <- dark candidate r=%.0f via %s (NOT trusted alone; "
                            "continuing)" % (dark_hit["radius_px"], dark_hit["matched_on"]))
                present = verdict.get("present") or []
                print("  [%d/%d] x=%.3f  %s%s"
                      % (i, len(xs), x,
                         ("fired: " + ",".join(present)) if present else "nothing",
                         note))
    finally:
        (out / "hunt.json").write_text(json.dumps(record, indent=1))
        print("\n[record] %s" % (out / "hunt.json"))

    if "found_x" in record:
        final = list(arm.status()["pose"])
        print("\n%s MARK FOUND. Arm is at x=%.4f y=%.4f z=%.4f"
              % (record.get("found_colour", "?").upper(), *final[:3]))
        print("That X is a MEASUREMENT, not yet taught. Nothing was written to")
        print("workspace.json -- teaching A1 is a separate, explicit step.")
        return 0

    print("\nno mark found over %+.1f mm. Widen --span or check the plate." % args.span)
    return 1


def main(argv=None) -> int:
    try:
        return run(build_parser().parse_args(argv))
    except SafetyError as exc:
        print("\nSAFETY: %s" % exc)
        return 6
    except KeyboardInterrupt:
        print("\ninterrupted")
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
