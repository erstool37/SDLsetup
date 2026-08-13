#!/usr/bin/env python3
"""
focus_sweep.py -- Y-axis approach to well A1, then an upward-only Z autofocus sweep.

WHAT IT DOES
  1. (--approach) Drive to A1 the only permitted way: stage 150 mm out along Y,
     level Z and align X out there, then slide straight in along Y at fixed Z.
     Built and validated by tools.arm.approach.
  2. Verify by READBACK that the arm really is at A1.
  3. Step the plate UPWARD in Z only, never more than Z_MAX_RISE_MM above A1,
     grabbing and scoring one microscope frame per step.
  4. Move to the sharpest Z -- approached from below, so backlash is taken out
     the same way at every sample and at the final move.

SAFETY MODEL -- enforced in code and unit-tested, not merely documented
  G1  Z window. Every commanded pose is checked against [a1.z, a1.z+max_rise].
      Out of window RAISES; it is never clamped. Enforced by
      Envelope.check_target (run inside every Arm.move()), built as
      Envelope.anchored(a1, z_max_rise_mm=max_rise, xy_max_mm=0.0,
      orient_tol_deg=0.0) -- see build_sweep_envelope() below.
  G2  Only Z moves during the sweep. X/Y/roll/pitch/yaw are compared against the
      arm's ACTUAL READ-BACK pose, not against the pose the script just built --
      a guard that only inspects its own arithmetic proves nothing. The same
      envelope's xy_max_mm=0.0 / orient_tol_deg=0.0 makes any XY or orientation
      drift raise, target or readback.
  G3  Position precondition. The sweep refuses to start unless readback puts the
      arm at A1 within the envelope's readback slack (0.5 mm / 0.5 deg).
      Envelope.check_readback reads the controller, never the arithmetic.
  G4  Dry-run default. Without --execute nothing connects and nothing moves
      (XArmConnection is a null transport when live=False).
  G5  Preflight. One frame is graded before any motion; too dark, and the arm
      is never even connected.
  G6  Closed loop or stop. The FIRST unusable frame aborts the pass. A blind
      ascent to the ceiling is exactly the case where a mis-measured clearance
      bites, so the routine will not continue without eyes.
  G7  Retreat on ANY failure -- including SIGTERM/SIGHUP, and including a second
      Ctrl-C during the retreat itself. Arm.engaged is set BEFORE every blocking
      send and is never reset, so an interrupt mid-move -- or a later failure
      after the arm has already been raised -- still retreats. Arm.retreat()
      only knows how to lower Z while the ACTIVE envelope is anchored; during
      the Y-approach (an unanchored workspace transit, see CONSOLIDATION NOTE)
      it logs a manual-lower warning instead of moving.
  G8  Controller faults are never auto-cleared on connect. A live fault aborts
      unless --clear-errors is passed deliberately.
  G9  Boundary peaks are a failure, not a result. A focus peak pinned at either
      end of the window means the true focus is outside it; the run exits
      non-zero and refuses to persist anything.
  G10 --save-a1-z writes only after a clean, non-boundary run, and clamps against
      the IMMUTABLE taught baseline, so repeated runs cannot ratchet the ceiling
      upward one focus height at a time.

FRAMES
  Frames come only from the live session's published file, via
  tools.microscope.Microscope.grab_frame(). The camera is never
  opened directly -- the Leica K3C is single-client and the live loop holds it.
  A frame counts for a step only if it was published after the step's settle
  period AND it is the SECOND such frame, because the publisher's mtime is a
  write time, not an exposure time: the first new file may have been exposed
  before the arm arrived. Frames are checked for a complete JPEG end-of-image
  marker and a size that stopped changing, so a half-written file is never
  scored.

SCORING
  Sharpness is scored on the MASKED region by default -- the illuminated sample
  surface plus the alignment mark's edge band, not the whole frame. Full-frame
  scoring is dominated by the unlit vignette and the illumination boundary,
  neither of which changes with focus: measured on a saved sweep, full-frame
  variance-of-Laplacian ranked A1+1.0 mm best, while the masked metric ranked
  A1+3.0 mm best (352.4 vs 98.0 tenengrad) -- the height the operator
  independently picked as visibly sharpest. Pass --full-frame-focus for the old
  whole-frame behaviour, kept reachable for comparison; it is no longer the
  default. See tools.microscope.focus for the mask construction.

CONSOLIDATION NOTE (2026-08)
  This file used to carry its own arm connection, its own frame poller, and its
  own focus scorer -- one of six such copies across the repo. It now calls the
  shared tools.arm.Arm / .safety.Envelope for every move,
  tools.microscope.Microscope for every frame, and
  tools.microscope.autofocus for scan planning and peak choice. Two
  consequences worth knowing:
    * the Y-approach runs under an UNANCHORED (workspace) Envelope, because it
      changes X and orientation, which the sweep's anchored envelope forbids.
      Its safety comes entirely from approach.validate_route(), unchanged; the
      transit envelope only bounds Z to the route's own extent. See G7 above
      for what this means for retreat during that phase.
    * focus scoring moved from a local mean^2-normalised full-frame score to
      the shared masked scorer -- see SCORING above and --full-frame-focus.

USAGE
  python3 scripts/focus_sweep.py                          # dry-run, prints the plan
  python3 scripts/focus_sweep.py --execute --approach      # approach A1, then sweep
  python3 scripts/focus_sweep.py --execute                 # already at A1: sweep only
  python3 scripts/focus_sweep.py --execute --save-a1-z     # also persist the focused Z
  python3 scripts/focus_sweep.py --execute --full-frame-focus   # old whole-frame scoring
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


import argparse
import datetime as dt
import hashlib
import json
import math
import shutil
import signal
import sys
import time
from collections.abc import Sequence
from pathlib import Path
from typing import Any

from tools import config as _config
from tools.arm import Arm, Envelope
from tools.arm import approach as approach_mod
from tools.arm.geometry import WELL_PITCH_MM
from tools.arm.safety import (
    READBACK_SLACK_DEG,
    READBACK_SLACK_MM,
    SPEED_MAX_MM_S,
    Z_MAX_RISE_MM,
)
from tools.microscope import Microscope
from tools.microscope import autofocus as af
from tools.microscope.focus import FocusSample

# ---------------------------------------------------------------------------
# Constants -- operator limits. Do not change without re-measuring the rig.
# ---------------------------------------------------------------------------

#: Readback tolerance for the "are we really at A1" precondition (G3). Named
#: locally so the rest of this file reads the same as before the migration;
#: sourced from the shared envelope defaults so there is one number, not two.
POSITION_TOL_MM = READBACK_SLACK_MM
POSITION_TOL_DEG = READBACK_SLACK_DEG

#: Bounds on operator-supplied numbers. Unbounded CLI values are a safety hole:
#: a huge --speed turns a careful approach into a lunge, and --min-mean 0 turns
#: the brightness gate off entirely while the Z ceiling stays on.
MIN_MEAN_FLOOR = 5.0
MAX_STEPS = 200

#: Frame-grab timeout. Not a CLI flag (never was); pinned here rather than
#: inheriting Microscope's own default (25.0 s) so this file's numeric
#: behaviour does not silently drift if that default ever changes.
FRAME_TIMEOUT_S = 20.0

DEFAULT_OUT_DIR = _config.PROJECT_ROOT / "dataset" / "captures" / "focus_sweep"
ARM_HOST_DEFAULT = "192.168.1.201"

#: Taught workspace key for A1. A separate concept from the envelope's display
#: name ("A1") passed to Envelope.anchored() below.
ANCHOR = "microscope"


class SweepError(RuntimeError):
    """Anything that must stop the sweep."""


class _Terminated(SweepError):
    """SIGTERM / SIGHUP arrived -- treated as an abort so retreat still runs."""


# ---------------------------------------------------------------------------
# (1) The sweep envelope -- G1/G2/G3, byte-for-byte the old check_target /
#     check_readback bound (see the guard-equivalence table in the migration
#     brief: xy_max_mm=0.0 and orient_tol_deg=0.0 on the TARGET check reproduce
#     the old code's exact-match XY/orientation guard; readback_slack_mm/deg
#     reproduce the old POSITION_TOL_MM/DEG allowance on READBACK only).
# ---------------------------------------------------------------------------

def build_sweep_envelope(a1: Sequence[float], max_rise_mm: float) -> Envelope:
    return Envelope.anchored(
        a1, name="A1", z_max_rise_mm=max_rise_mm, xy_max_mm=0.0, orient_tol_deg=0.0,
        readback_slack_mm=POSITION_TOL_MM, readback_slack_deg=POSITION_TOL_DEG,
        speed_max_mm_s=SPEED_MAX_MM_S,
    )


# ---------------------------------------------------------------------------
# (2) The Y-approach -- unchanged route/validation (approach_mod), but every
#     leg now has to reach the arm through a guarded ValidatedMove. Legs 1-4
#     end with A1's orientation and travel through XY the anchored sweep
#     envelope cannot bound (leg 1 keeps whatever X the arm started at), so
#     this phase runs on a FREE (unanchored) envelope, per Arm.free_envelope's
#     documented purpose ("workspace traversal ... a reason and a Z corridor").
#     approach.validate_route() -- run unconditionally inside approach_route()
#     -- remains the actual collision guard for this phase, exactly as before.
# ---------------------------------------------------------------------------

def build_transit(arm: Arm, a1: Sequence[float], standoff_mm: float,
                  args: argparse.Namespace) -> tuple[Arm, list[dict[str, Any]]]:
    """Build (but do not send) the validated Y-approach route and the transit
    Arm view to send it through. Touches the controller only to read the
    current pose, never to move it, so it cannot itself leave the arm engaged.

    The caller MUST reassign its own `arm` to the returned Arm BEFORE calling
    send_approach_legs() -- see the note there for why: engaged bookkeeping
    depends on the caller and send_approach_legs() sharing the same object
    from the first leg onward, not just once every leg has succeeded.
    """
    start = arm.pose() if arm.settings.live else list(a1)
    route = approach_mod.approach_route(
        start, a1, distance_mm=standoff_mm, sign=approach_mod.Y_APPROACH_SIGN,
        transit_mm_s=min(args.speed * 3, SPEED_MAX_MM_S), push_in_mm_s=args.speed)
    print(approach_mod.describe(route, start), flush=True)

    # The transit's only Envelope-enforced bound is Z, sized to exactly what
    # this validated route needs -- not a newly invented tolerance. XY/orient
    # safety is approach.validate_route()'s job, not this envelope's.
    leg_zs = [start[2]] + [wp["pose"][2] for wp in route]
    transit = arm.free_envelope(
        reason="A1 Y-approach transit (focus_sweep --approach); XY and orientation "
               "safety come from approach.validate_route(), not this envelope",
        z_floor_mm=min(leg_zs), z_ceiling_mm=max(leg_zs))
    return transit, route


def send_approach_legs(arm: Arm, route: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Send each leg of an already-built, already-validated route.

    `arm` must already be the transit (free-envelope) Arm the CALLER is
    holding -- move_pose() sets .engaged on THIS object as it sends each leg,
    so if a leg fails partway, the caller's own `arm` reference (the same
    object, not a copy) is already the one retreat() needs; nothing here is
    returned-and-lost the way it would be if the transit Arm were only handed
    back once every leg had already succeeded.
    """
    done: list[dict[str, Any]] = []
    for wp in route:
        print("  [approach] %s" % wp["label"], flush=True)
        # Each leg carries its own orientation, so this is move_pose(), not
        # move(). Called unconditionally -- like run_pass()'s arm.move(), it is
        # a safe no-op transport in dry-run (nothing connects, nothing sends)
        # and .achieved comes back None -- so dry-run still exercises the same
        # Envelope.check_target() the live path does. verify=False skips only
        # the post-move readback re-check: the transit envelope bounds Z alone,
        # and XY/orientation safety for this phase is approach.validate_route()
        # 's job. Engagement is set by the driver before the blocking call, so
        # an abort mid-leg still owes a retreat.
        result = arm.move_pose(wp["pose"], speed=wp["speed_mm_s"], label=wp["label"],
                               verify=False)
        done.append({"label": wp["label"], "target": list(wp["pose"]),
                    "achieved": result.achieved})
    return done


# ---------------------------------------------------------------------------
# (3) One pass of the sweep. G6: the FIRST unusable frame aborts.
# ---------------------------------------------------------------------------

def run_pass(arm: Arm, scope: Microscope, a1: Sequence[float], z_values: list[float],
            out_dir: Path, tag: str, settle_s: float, method: str, min_mean: float,
            masked: bool, ordinal: int) -> tuple[list[dict[str, Any]], list[FocusSample]]:
    samples: list[dict[str, Any]] = []
    focus_samples: list[FocusSample] = []
    for i, z in enumerate(z_values, start=1):
        label = "%s %d/%d" % (tag, i, len(z_values))
        result = arm.move(a1[0], a1[1], z, label=label, takeup=True)
        rec: dict[str, Any] = {"z": round(z, 4), "rise_mm": round(z - a1[2], 4),
                               "pass": tag, "achieved_pose": result.achieved}
        if not arm.settings.live:
            rec.update(usable=False, reason="dry-run: no frame taken")
            samples.append(rec)
            print("        dry-run, no frame", flush=True)
            continue
        time.sleep(settle_s)
        dest = out_dir / ("%s_%02d_z%+07.3f.jpg" % (tag, i, z - a1[2]))
        # Freshness is anchored AFTER the settle, not at arrival: grab_frame()
        # waits for a frame published after THIS call.
        frame = scope.grab_frame(dest, ordinal=ordinal, timeout_s=FRAME_TIMEOUT_S)
        rec.update(ok=frame.ok, path=str(frame.path) if frame.path else None,
                   frame_mtime=frame.published_at, frames_skipped=ordinal - 1,
                   bytes=(dest.stat().st_size if frame.ok else None))
        sample: FocusSample | None = None
        if frame.ok:
            try:
                sample = scope.measure_focus(dest, method=method, min_mean=min_mean,
                                             masked=masked)
            except Exception as exc:
                rec["usable"] = False
                rec["reason"] = "measure_focus failed: %r" % (exc,)
            else:
                rec.update(sample.as_dict())
        else:
            rec["usable"] = False
            rec["reason"] = frame.reason
        samples.append(rec)
        if rec.get("usable"):
            print("        mean=%7.2f  score=%.6g  %s" % (rec["mean"], rec["score"], dest.name),
                 flush=True)
            focus_samples.append(sample)
        else:
            raise SweepError("%s: %s -- stopping rather than climbing blind"
                             % (label, rec.get("reason")))
    return samples, focus_samples


def _file_sha256(path: Path) -> str:
    h = hashlib.sha256()
    h.update(path.read_bytes())
    return h.hexdigest()


# ---------------------------------------------------------------------------
# (4) CLI
# ---------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--execute", action="store_true",
                    help="connect to the arm and move (default: dry-run)")
    ap.add_argument("--approach", action="store_true",
                    help="drive to A1 first via the Y route (standoff -> Z -> X -> Y push-in)")
    ap.add_argument("--host", default=ARM_HOST_DEFAULT)
    ap.add_argument("--max-rise", type=float, default=Z_MAX_RISE_MM)
    ap.add_argument("--coarse-step", type=float, default=1.0)
    ap.add_argument("--fine-step", type=float, default=0.1)
    ap.add_argument("--speed", type=float, default=3.0,
                    help="mm/s for Z steps (max %.0f)" % SPEED_MAX_MM_S)
    ap.add_argument("--settle", type=float, default=0.8)
    ap.add_argument("--frame-ordinal", type=int, default=2,
                    help="use the Nth frame published after settling (default 2; the "
                         "1st may have been exposed before the arm arrived)")
    ap.add_argument("--method", default="laplacian",
                    choices=("laplacian", "tenengrad", "brenner"))
    ap.add_argument("--min-mean", type=float, default=30.0)
    ap.add_argument("--full-frame-focus", action="store_true",
                    help="score focus on the whole frame instead of the masked ROI "
                         "(old behaviour, kept reachable for comparison; masked "
                         "scoring is now the default -- see the module docstring)")
    ap.add_argument("--standoff-mm", type=float, default=None,
                    help="Y standoff for --approach (default: module value, 150 mm)")
    ap.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    ap.add_argument("--clear-errors", action="store_true",
                    help="clear a pre-existing controller fault (inspect the arm first)")
    ap.add_argument("--save-a1-z", action="store_true",
                    help="persist the focused Z into A1 (only after a clean, non-boundary run)")
    return ap


# ---------------------------------------------------------------------------
# (5) The procedure
# ---------------------------------------------------------------------------

def run(args: argparse.Namespace) -> int:
    # -- CLI validation: unbounded numbers are a safety hole -----------------
    def finite(name: str, v: float) -> float:
        if not math.isfinite(v):
            raise SystemExit("%s must be a finite number, got %r" % (name, v))
        return v

    finite("--max-rise", args.max_rise)
    finite("--coarse-step", args.coarse_step)
    finite("--fine-step", args.fine_step)
    finite("--speed", args.speed)
    finite("--min-mean", args.min_mean)
    finite("--settle", args.settle)
    if not (0 < args.max_rise <= Z_MAX_RISE_MM):
        raise SystemExit("--max-rise must be in (0, %.1f]; the ceiling is a hardware "
                         "clearance decision, not a CLI option" % Z_MAX_RISE_MM)
    if not (0 < args.speed <= SPEED_MAX_MM_S):
        raise SystemExit("--speed must be in (0, %.1f] mm/s" % SPEED_MAX_MM_S)
    if args.min_mean < MIN_MEAN_FLOOR:
        raise SystemExit("--min-mean must be >= %.1f; disabling the brightness gate while "
                         "the Z ceiling stays on is not an option" % MIN_MEAN_FLOOR)
    if args.coarse_step <= 0 or args.fine_step <= 0:
        raise SystemExit("step sizes must be positive")
    if args.frame_ordinal < 1:
        raise SystemExit("--frame-ordinal must be >= 1")
    if args.settle < 0:
        raise SystemExit("--settle must be >= 0")
    n_est = int(args.max_rise / args.coarse_step) + int(args.coarse_step / args.fine_step) + 4
    if n_est > MAX_STEPS:
        raise SystemExit("that step configuration implies ~%d moves (cap %d); use larger steps"
                         % (n_est, MAX_STEPS))

    masked = not args.full_frame_focus

    arm = Arm.from_config(live=args.execute, host=args.host, speed_mm_s=args.speed,
                          clear_errors=args.clear_errors, anchor=ANCHOR)
    scope = Microscope.from_config()

    # a1 / a1_loc: the taught anchor and its metadata. Outside the try block,
    # matching the old code -- a missing taught location is not a run failure
    # to recover from, it is a setup error.
    a1 = arm.location_pose(ANCHOR)
    a1_loc = arm.store.require_location(ANCHOR)
    # G10: the immutable baseline the ceiling is measured from, so repeated
    # --save-a1-z runs cannot walk the ceiling upward.
    baseline_z = float(a1_loc.metadata.get("taught_a1_z", a1[2]))
    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    out_dir = args.out_dir / stamp

    plan = af.plan_scan(a1[2], a1[2] + args.max_rise, args.coarse_step, args.fine_step)
    standoff_mm = args.standoff_mm if args.standoff_mm else approach_mod.Y_STANDOFF_MM

    print("A1 pose      : x=%.3f y=%.3f z=%.3f roll=%.3f pitch=%.3f yaw=%.3f" % tuple(a1))
    print("taught A1 z  : %.4f  (ceiling is measured from THIS, not from a saved focus)"
          % baseline_z)
    print("Z window     : %.3f .. %.3f  (upward only, %.2f mm budget, hard cap %.1f)"
          % (a1[2], a1[2] + args.max_rise, args.max_rise, Z_MAX_RISE_MM))
    print("Approach     : %s" % ("Y route, %.0f mm standoff at %sY"
                                 % (standoff_mm,
                                    "+" if approach_mod.Y_APPROACH_SIGN > 0 else "-")
                                 if args.approach else "OFF (arm must already be at A1)"))
    print("Plan         : %s" % plan["description"])
    print("Well pitch   : %.1f mm  (every well is derived from A1; this sweep moves neither "
          "X nor Y)" % WELL_PITCH_MM)
    print("Frames       : %s" % out_dir)
    print("Focus        : %s, %s" % (args.method, "full-frame" if args.full_frame_focus
                                     else "masked (default)"))
    print("Mode         : %s" % ("LIVE -- THE ARM WILL MOVE" if args.execute else "DRY-RUN"))
    print()

    samples: list[dict[str, Any]] = []
    approach_log: list[dict[str, Any]] = []
    chosen: float | None = None
    boundary: Any = False
    status = "dry-run" if not args.execute else "incomplete"

    def _term(signum, _frame):
        raise _Terminated("received signal %d" % signum)

    for sig in (signal.SIGTERM, signal.SIGHUP):
        try:
            signal.signal(sig, _term)
        except (ValueError, OSError):
            pass

    try:
        # -- Preflight: grade a frame BEFORE connecting to the arm -----------
        if args.execute:
            print("[preflight] grading one frame before touching the arm")
            out_dir.mkdir(parents=True, exist_ok=True)
            pre_dest = out_dir / "preflight.jpg"
            pre = scope.grab_frame(pre_dest, ordinal=1, timeout_s=FRAME_TIMEOUT_S)
            if not pre.ok:
                raise SweepError("preflight: %s" % pre.reason)
            graded = scope.measure_focus(pre_dest, method=args.method, min_mean=args.min_mean,
                                         masked=masked)
            if not graded.usable:
                raise SweepError("preflight: %s -- fix the illumination and re-run. "
                                 "The arm has not been connected." % graded.reason)
            print("[preflight] OK: mean=%.2f score=%.6g\n" % (graded.mean, graded.score))

        # -- Approach --------------------------------------------------------
        if args.approach:
            print("[approach] driving to A1 along the Y route")
            # Reassign `arm` to the transit view BEFORE sending a single leg:
            # send_approach_legs() sets .engaged on whatever object it is
            # given, so if this file held onto the old (pre-transit) `arm`
            # until every leg had already succeeded, a leg that failed
            # partway would leave `arm` pointing at an un-engaged object and
            # the except-block's arm.retreat() below would silently no-op.
            arm, route = build_transit(arm, a1, standoff_mm, args)
            approach_log = send_approach_legs(arm, route)
            print()

        # -- Switch to the Z-only sweep envelope ------------------------------
        arm = arm.with_envelope(build_sweep_envelope(a1, args.max_rise))

        # -- G3: prove by readback that we are at A1 -------------------------
        if args.execute:
            actual = arm.pose()
            print("[readback] x=%.3f y=%.3f z=%.3f roll=%.3f pitch=%.3f yaw=%.3f"
                  % tuple(actual))
            arm.envelope.check_readback(actual, where="before the sweep")
            print("[readback] confirmed at A1 within %.2f mm / %.2f deg\n"
                  % (POSITION_TOL_MM, POSITION_TOL_DEG))

        # -- Pass 1: coarse ---------------------------------------------------
        print("[pass 1/2] coarse, %d points @ %.2f mm" % (plan["n_coarse"], args.coarse_step))
        coarse_samples, coarse_focus = run_pass(
            arm, scope, a1, plan["coarse_z"], out_dir, "coarse", args.settle,
            args.method, args.min_mean, masked, args.frame_ordinal)
        samples += coarse_samples

        if args.execute:
            coarse_report = af.scan_report(coarse_focus, plan["coarse_z"])
            peak = coarse_report["best_z_mm"]
            fine_z = af.fine_scan_values(peak, a1[2], a1[2] + args.max_rise,
                                         args.fine_step, plan["fine_half_window"])
            print("\n[pass 2/2] fine, %d points @ %.2f mm around coarse peak z=%.3f"
                  % (len(fine_z), args.fine_step, peak))
            fine_samples, fine_focus = run_pass(
                arm, scope, a1, fine_z, out_dir, "fine", args.settle,
                args.method, args.min_mean, masked, args.frame_ordinal)
            samples += fine_samples
            if len(fine_focus) < 3:
                raise SweepError("only %d usable fine samples; need at least 3 to fit a peak"
                                 % len(fine_focus))

            fine_report = af.scan_report(fine_focus, fine_z)
            best_z = fine_report["best_z_mm"]
            best_score = fine_report["best_score"]
            chosen = af.parabolic_refine(fine_z, [fs.score for fs in fine_focus])
            lo, hi = a1[2], a1[2] + args.max_rise
            if not (lo - 1e-6 <= chosen <= hi + 1e-6):
                raise SweepError("parabolic fit returned z=%.4f outside the window "
                                 "[%.4f, %.4f]" % (chosen, lo, hi))

            # G9: a peak pinned at either end means focus is OUTSIDE the window.
            if best_z >= hi - args.fine_step - 1e-6:
                boundary = "top"
            elif best_z <= lo + args.fine_step + 1e-6:
                boundary = "bottom"

            # cross-pass sanity: the fine winner should beat every coarse sample
            print("\n[result] best coarse   z=%.4f (A1%+.3f) score=%.6g"
                  % (coarse_report["best_z_mm"], coarse_report["best_z_mm"] - a1[2],
                     coarse_report["best_score"]))
            print("[result] best fine     z=%.4f (A1%+.3f) score=%.6g"
                  % (best_z, best_z - a1[2], best_score))
            print("[result] parabolic fit z=%.4f (A1%+.3f)" % (chosen, chosen - a1[2]))
            if best_score < coarse_report["best_score"]:
                print("[result] WARNING: no fine sample beat the best coarse sample; the "
                      "focus curve may be flat or the peak may lie elsewhere", flush=True)

            if boundary:
                raise SweepError(
                    "focus peak is pinned at the %s of the Z window (best z=%.4f, window "
                    "[%.4f, %.4f]). True focus is OUTSIDE the searched range -- this is a "
                    "measurement, not a focus. Re-measure the plate-to-objective clearance "
                    "before widening the budget." % (boundary, best_z, lo, hi))

            arm.move(a1[0], a1[1], chosen, label="move to focus", takeup=True)
            status = "focused"
        else:
            print("\n[dry-run] the fine pass and peak selection need real frames; "
                  "re-run with --execute")

    except (SweepError, KeyboardInterrupt, Exception) as exc:   # noqa: B014 - explicit
        status = "aborted: %s" % (exc if str(exc) else repr(exc))
        print("\n[ABORT] %s" % status, file=sys.stderr, flush=True)
        arm.retreat()
    finally:
        try:
            out_dir.mkdir(parents=True, exist_ok=True)
            (out_dir / "manifest.json").write_text(json.dumps({
                "artifact": "A1 Y-approach + upward-only Z autofocus sweep",
                "produced_at": dt.datetime.now().isoformat(timespec="seconds"),
                "status": status,
                "boundary": boundary,
                "code": str(Path(__file__).resolve()),
                "code_sha256": _file_sha256(Path(__file__).resolve()),
                "entrypoint": " ".join([sys.executable, *sys.argv]),
                "arm_host": args.host,
                "a1_pose": a1,
                "a1_store_path": str(arm.store.path),
                "taught_a1_z_baseline": baseline_z,
                "well_pitch_mm": WELL_PITCH_MM,
                "approach_used": bool(args.approach),
                "approach_standoff_mm": standoff_mm,
                "approach_sign": approach_mod.Y_APPROACH_SIGN,
                "approach_legs": approach_log,
                "z_window_mm": [a1[2], a1[2] + args.max_rise],
                "max_rise_mm": args.max_rise,
                "hard_ceiling_mm": Z_MAX_RISE_MM,
                "coarse_step_mm": args.coarse_step,
                "fine_step_mm": args.fine_step,
                "speed_mm_s": args.speed,
                "backlash_takeup_mm": arm.settings.backlash_takeup_mm,
                "settle_s": args.settle,
                "frame_ordinal": args.frame_ordinal,
                "focus_method": args.method,
                "focus_masked": masked,
                "score_definition": ("%s, %s region" % (
                    args.method, "full frame" if args.full_frame_focus
                    else "masked (lit sample + marker edge)")),
                "brightness_min_mean": args.min_mean,
                "frame_source": str(scope.stream_path()),
                "chosen_z": chosen,
                "chosen_rise_mm": None if chosen is None else round(chosen - a1[2], 4),
                "store_mutated": False,     # rewritten below if --save-a1-z fires
                "samples": samples,
            }, indent=2))
            print("\n[manifest] %s" % (out_dir / "manifest.json"))
        except OSError as exc:
            print("[manifest] WRITE FAILED: %r" % (exc,), file=sys.stderr, flush=True)
        arm.close()

    # -- G10: persist only after a clean, non-boundary run --------------------
    if args.save_a1_z:
        store = arm.store
        if status != "focused" or chosen is None or boundary:
            print("[saved] NOT saving: status=%s boundary=%s -- the calibration anchor is "
                  "only rewritten after a clean run" % (status, boundary), flush=True)
        elif not (baseline_z - 1e-6 <= chosen <= baseline_z + Z_MAX_RISE_MM + 1e-6):
            print("[saved] NOT saving: z=%.4f is outside the immutable taught baseline "
                  "window [%.4f, %.4f]" % (chosen, baseline_z, baseline_z + Z_MAX_RISE_MM),
                  flush=True)
        else:
            backup = store.path.with_suffix(".json.bak.%s" % stamp)
            shutil.copyfile(store.path, backup)
            loc = store.workspace.locations[ANCHOR]
            previous_z = loc.pose.z
            loc.pose.z = round(float(chosen), 6)
            loc.metadata["taught_a1_z"] = baseline_z
            loc.metadata["focused_z_set_at"] = dt.datetime.now().isoformat(timespec="seconds")
            loc.metadata["focused_z_previous"] = previous_z
            loc.metadata["focused_z_manifest"] = str(out_dir / "manifest.json")
            loc.metadata["joint_angles_deg_stale"] = (
                "pose.z was changed by focus_sweep; the recorded joint angles describe the "
                "pose BEFORE that change and no longer correspond to pose")
            store.save()
            m = json.loads((out_dir / "manifest.json").read_text())
            m["store_mutated"] = {"path": str(store.path), "field": "microscope.pose.z",
                                  "from": previous_z, "to": loc.pose.z, "backup": str(backup)}
            (out_dir / "manifest.json").write_text(json.dumps(m, indent=2))
            print("[saved] A1 z %.4f -> %.4f  (backup: %s)" % (previous_z, loc.pose.z, backup))

    return 0 if status in ("focused", "dry-run") else 1


def main(argv=None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
