#!/usr/bin/env python3
"""
align_a1.py -- one command that focuses and centres well A1, iteratively.

    python3 scripts/align_a1.py --execute

WHY IT ITERATES
---------------
Focus and centring are coupled, so doing each once is not enough:

  * Changing Z changes the MAGNIFICATION, so the pixels-per-mm used to convert
    "the mark is 300 px off" into "move 0.35 mm" is only valid at the height it
    was measured at. Measured on this rig: 861 px/mm along arm-X at A1+3.0 mm.
  * Changing XY changes WHICH PART of the sample is under the objective, and a
    plate is neither perfectly flat nor perfectly level, so best focus moves.

So each round is: focus -> re-measure the pixel/mm mapping at that new height ->
centre -> check whether either changed enough to matter. The loop stops when a
round changes Z by less than `--z-tol` and leaves the mark within `--px-tol` of
the image centre.

**The run always ends with a focus pass**, after the final XY move, so the
height that gets stored was measured at the XY that gets stored -- never at the
one before it.

WHAT THIS FIXES, MEASURED TODAY (2026-07-31)
--------------------------------------------
1. FOCUS WAS SCORED ON THE WHOLE FRAME AND PICKED THE WRONG HEIGHT. Most of the
   frame is unlit vignette plus the bright/dark boundary, which dominates a
   variance-of-Laplacian score. Full-frame scoring ranked A1+1.0 mm best; the
   same frames scored on the central ROI ranked A1+3.0 mm best, by 265 vs 78 in
   ROI tenengrad -- a 3.4x separation -- and +3.0 mm was visibly, obviously
   sharper. This module scores an ROI (via the shared masked focus metric,
   `tools.microscope.Microscope.measure_focus(masked=True)`, the
   default). Full-frame scoring stays reachable with --full-frame-focus, purely
   as an escape hatch -- this module has never used it by default.
2. THE MARK'S CENTROID LIED WHEN THE MARK WAS PARTLY UNLIT, and lied in the
   confident direction: it read "centred" while the true centre was 0.59 mm
   away. `marker.detect` reports both a centroid and a circle fit and flags
   their disagreement; this module steers on whichever the flags justify.
3. A FRAME'S mtime IS ITS PUBLISH TIME, NOT ITS EXPOSURE TIME. At a measured
   3.0 s cadence with ~2 s capture, the first frame published after the arm
   settles was probably exposed before it arrived. Every capture here takes the
   SECOND new frame (Microscope.grab_frame's default ordinal).

SAFETY -- the same rules as the single-purpose tools, now enforced by the one
shared guarded path (tools.arm.Envelope + Arm) every procedure uses
--------------------------------------------------------------------------
  S1 Z is confined to [a1.z, a1.z + Z_MAX_RISE_MM] measured from the IMMUTABLE
     taught A1 height, so repeated runs cannot walk the ceiling toward the lens.
  S2 XY is confined to a +/-XY_MAX_MM box around the taught A1.
  S3 Both are checked against READBACK, not against the pose the script just
     built. A guard that only inspects its own arithmetic proves nothing.
  S4 Dry-run default; --execute is required to move.
  S5 Any failure -- exception, Ctrl-C, SIGTERM -- lowers Z to the taught A1
     height, the direction away from the objective.
  S6 No usable frame stops the routine where it stands; it never climbs blind.
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
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from tools import config as _config
from tools.arm import Arm, Envelope
from tools.arm.safety import SPEED_MAX_MM_S, XY_MAX_MM, Z_MAX_RISE_MM
from tools.arm.workspace import WorkspaceStore
from tools.calib import pixel_scale as ps
from tools.microscope import Microscope, marker
from tools.microscope import autofocus as af

# ---------------------------------------------------------------------------
# Limits. Z_MAX_RISE_MM / XY_MAX_MM / SPEED_MAX_MM_S are hardware-clearance
# decisions, not tuning knobs -- imported from the shared guard so this module
# can never drift from the value every other procedure enforces.
# ---------------------------------------------------------------------------

#: Orientation tolerance for the A1 envelope. Applied identically to the
#: commanded target and to readback (no separate slack) -- matching the single
#: guard this module used before the consolidation.
POSITION_TOL_DEG = 0.5

ROI_FRAC = 0.45               # central fraction of the frame used for masked focus
CAL_STEP_MM = 0.50
RED_FALLBACK_TO_DARK = True

DEFAULT_OUT_DIR = _config.PROJECT_ROOT / "dataset" / "captures" / "align_a1"
ARM_HOST_DEFAULT = "192.168.1.201"


class AlignError(RuntimeError):
    pass


class _Terminated(AlignError):
    pass


# ---------------------------------------------------------------------------
# Stage 1: focus at the current XY
# ---------------------------------------------------------------------------

def focus_here(arm: Arm, scope: Microscope, x: float, y: float, out: Path, tag: str,
               args: argparse.Namespace) -> dict[str, Any]:
    """Coarse then fine Z sweep, scored on the lit region. Returns the chosen z."""
    lo = arm.envelope.anchor[2]
    hi = lo + args.max_rise
    plan = af.plan_scan(lo, hi, args.coarse_step, args.fine_step)
    samples: list[dict[str, Any]] = []

    def sweep(zs: list[float], name: str) -> list[dict[str, Any]]:
        got = []
        for i, z in enumerate(zs, 1):
            arm.move(x, y, z, label="%s %s %d/%d" % (tag, name, i, len(zs)), takeup=True)
            if not arm.settings.live:
                got.append({"z": z, "usable": False, "reason": "dry-run"})
                continue
            time.sleep(args.settle)
            f = out / ("%s_%s_%02d_z%+07.3f.jpg" % (tag, name, i, z - lo))
            frame = scope.grab_frame(f, ordinal=args.frame_ordinal)
            if not frame.ok:
                raise AlignError("%s %s %d: %s" % (tag, name, i, frame.reason))
            sample = scope.measure_focus(frame.path, method=args.method,
                                         min_mean=args.min_mean,
                                         masked=not args.full_frame_focus)
            s = sample.as_dict()
            s.update(z=round(z, 4), rise_mm=round(z - lo, 4), frame=str(f), pass_=name)
            if not sample.usable:
                raise AlignError("%s %s %d: %s -- stopping rather than sweeping blind"
                                 % (tag, name, i, sample.reason))
            print("        z=%+.3f  lit mean=%6.2f  score=%9.1f"
                  % (z - lo, sample.mean, sample.score), flush=True)
            got.append(s)
        return got

    print("  [focus] coarse: %d points @ %.2f mm" % (plan["n_coarse"], args.coarse_step))
    samples += sweep(plan["coarse_z"], "coarse")
    if not arm.settings.live:
        return {"z": lo, "samples": samples, "dry_run": True}

    coarse = [s for s in samples if s["pass_"] == "coarse"]
    peak = max(coarse, key=lambda s: s["score"])["z"]
    fine_z = af.fine_scan_values(peak, lo, hi, args.fine_step, plan["fine_half_window"])
    print("  [focus] fine: %d points @ %.2f mm around z=%+.3f"
          % (len(fine_z), args.fine_step, peak - lo))
    samples += sweep(fine_z, "fine")
    fine = [s for s in samples if s["pass_"] == "fine"]
    if len(fine) < 3:
        raise AlignError("only %d usable fine samples; need 3 to fit a peak" % len(fine))

    best = max(fine, key=lambda s: s["score"])
    z_star = af.parabolic_refine([s["z"] for s in fine], [s["score"] for s in fine])
    z_star = min(max(z_star, lo), hi)
    boundary = None
    if best["z"] >= hi - args.fine_step - 1e-6:
        boundary = "top"
    elif best["z"] <= lo + args.fine_step + 1e-6:
        boundary = "bottom"
    if boundary:
        raise AlignError("focus peak is pinned at the %s of the Z window (best z=%+.3f mm, "
                         "window 0..%+.1f mm). True focus is OUTSIDE the searched range; "
                         "re-measure the plate-to-objective clearance before widening it."
                         % (boundary, best["z"] - lo, args.max_rise))
    print("  [focus] best sampled z=%+.4f mm, parabolic fit z=%+.4f mm"
          % (best["z"] - lo, z_star - lo))
    arm.move(x, y, z_star, label="%s move to focus" % tag, takeup=True)
    return {"z": z_star, "best_sampled_z": best["z"], "samples": samples,
            "best_score": best["score"]}


# ---------------------------------------------------------------------------
# Stage 2: measure the pixel/mm mapping at this height
# ---------------------------------------------------------------------------

def calibrate_here(arm: Arm, scope: Microscope, x: float, y: float, z: float, out: Path,
                   tag: str, args: argparse.Namespace) -> dict[str, Any]:
    f0 = out / ("%s_cal_ref.jpg" % tag)
    frame0 = scope.grab_frame(f0, ordinal=args.frame_ordinal)
    if not frame0.ok:
        raise AlignError("calibration reference frame: %s" % frame0.reason)

    shifts = {}
    for axis, (dx, dy) in (("x", (args.cal_step, 0.0)), ("y", (0.0, args.cal_step))):
        # XY-only move at constant Z -- no backlash dip (matches the old
        # GuardedArm.goto default of takeup=False; only the Z-changing focus
        # moves in focus_here() opt into the dip).
        arm.move(x + dx, y + dy, z, label="%s calib +%s" % (tag, axis.upper()), takeup=False)
        time.sleep(args.settle)
        fi = out / ("%s_cal_%s.jpg" % (tag, axis))
        framei = scope.grab_frame(fi, ordinal=args.frame_ordinal)
        if not framei.ok:
            raise AlignError("calibration %s frame: %s" % (axis, framei.reason))
        a = cv2.imread(str(f0), cv2.IMREAD_GRAYSCALE).astype(np.float32)
        b = cv2.imread(str(fi), cv2.IMREAD_GRAYSCALE).astype(np.float32)
        win = cv2.createHanningWindow((a.shape[1], a.shape[0]), cv2.CV_32F)
        (sx, sy), resp = cv2.phaseCorrelate(a, b, win)
        if resp < 0.02:
            raise AlignError("phase correlation for %s is too weak (response %.4f) to trust"
                             % (axis, resp))
        if (sx * sx + sy * sy) ** 0.5 < 5.0:
            raise AlignError("a %.2f mm move in %s shifted the image by <5 px; the arm may "
                             "not have moved" % (args.cal_step, axis))
        shifts[axis] = (float(sx), float(sy), float(resp))
        print("        +%.2f mm %s -> image moved (%+7.1f, %+7.1f) px  (response %.3f)"
              % (args.cal_step, axis.upper(), sx, sy, resp), flush=True)

    arm.move(x, y, z, label="%s back from calibration" % tag, takeup=False)
    J = ps.jacobian_from_moves(shifts["x"][:2], shifts["y"][:2], args.cal_step)
    d = ps.decompose(J)
    print("        %.1f px/mm (arm-X), %.1f px/mm (arm-Y), rotation %.1f deg, %s"
          % (d["px_per_mm_x"], d["px_per_mm_y"], d["rotation_deg"],
             "orientation-reversing" if d["orientation_reversing"] else "orientation-preserving"))
    return {"z": z, "J": J.tolist(), "shifts": shifts, **d}


# ---------------------------------------------------------------------------
# Stage 3: centre the mark
# ---------------------------------------------------------------------------

def centre_here(arm: Arm, scope: Microscope, x: float, y: float, z: float, J: np.ndarray,
                out: Path, tag: str, args: argparse.Namespace) -> dict[str, Any]:
    cur_x, cur_y = x, y
    history = []
    for it in range(1, args.centre_iters + 1):
        time.sleep(args.settle)
        f = out / ("%s_centre_%02d.jpg" % (tag, it))
        frame = scope.grab_frame(f, ordinal=args.frame_ordinal)
        if not frame.ok:
            raise AlignError("centring iteration %d: %s" % (it, frame.reason))
        det = scope.detect_marker(f, mode=args.mode)
        if not det["candidates"] and args.mode == "red" and RED_FALLBACK_TO_DARK:
            print("        no red mark (%s); falling back to the dark detector"
                  % det.get("note", "")[:60], flush=True)
            det = scope.detect_marker(f, mode="dark")
        if not det["candidates"]:
            raise AlignError("no alignment mark found: %s" % det.get("note", "no candidates"))
        cand = det["candidates"][0]
        off, why = marker.best_offset_px(cand)
        err = float(np.hypot(*off))
        print("    [centre %d] offset (%+7.1f, %+7.1f) px = %6.1f px via %s%s"
              % (it, off[0], off[1], err, why,
                 "  [%s]" % cand["quality"] if cand["quality"] != "ok" else ""), flush=True)
        history.append({"iter": it, "frame": str(f), "offset_px": list(off),
                        "error_px": err, "metric": why, "quality": cand["quality"],
                        "disagreement_px": cand.get("disagreement_px"),
                        "xy": [cur_x, cur_y]})
        if err <= args.px_tol:
            print("    [centre %d] within %.0f px -- centred" % (it, args.px_tol), flush=True)
            break
        dx, dy = ps.correction_mm(J, off)
        nx, ny = cur_x + dx, cur_y + dy
        anchor = arm.envelope.anchor
        if max(abs(nx - anchor[0]), abs(ny - anchor[1])) > XY_MAX_MM:
            raise AlignError("the correction would move the plate %+.3f, %+.3f mm from A1, "
                             "outside the +/-%.1f mm box. The mark is further off than this "
                             "routine may chase; re-teach A1 by hand."
                             % (nx - anchor[0], ny - anchor[1], XY_MAX_MM))
        print("        correction (%+.4f, %+.4f) mm" % (dx, dy), flush=True)
        arm.move(nx, ny, z, label="%s centre %d" % (tag, it), takeup=False)
        cur_x, cur_y = nx, ny
    return {"x": cur_x, "y": cur_y, "history": history,
            "final_error_px": history[-1]["error_px"] if history else None}


# ---------------------------------------------------------------------------
# The loop
# ---------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--execute", action="store_true")
    ap.add_argument("--host", default=ARM_HOST_DEFAULT)
    ap.add_argument("--rounds", type=int, default=3, help="max focus+centre rounds")
    ap.add_argument("--max-rise", type=float, default=Z_MAX_RISE_MM)
    ap.add_argument("--coarse-step", type=float, default=1.0)
    ap.add_argument("--fine-step", type=float, default=0.1)
    ap.add_argument("--cal-step", type=float, default=CAL_STEP_MM)
    ap.add_argument("--centre-iters", type=int, default=4)
    ap.add_argument("--px-tol", type=float, default=40.0, help="centred when within this")
    ap.add_argument("--z-tol", type=float, default=0.15, help="converged when |dz| below this")
    ap.add_argument("--speed", type=float, default=3.0)
    ap.add_argument("--settle", type=float, default=0.8)
    ap.add_argument("--frame-ordinal", type=int, default=2)
    ap.add_argument("--method", default="tenengrad",
                    choices=("laplacian", "tenengrad", "brenner"))
    ap.add_argument("--min-mean", type=float, default=30.0)
    ap.add_argument("--mode", default="dark", choices=("dark", "red"))
    ap.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    ap.add_argument("--clear-errors", action="store_true")
    ap.add_argument("--save", action="store_true", help="persist the aligned A1 pose")
    ap.add_argument("--full-frame-focus", action="store_true",
                    help="score focus on the whole frame instead of the masked ROI "
                         "(old behaviour, kept reachable for comparison; this module has "
                         "always defaulted to the masked ROI -- see the module docstring)")
    return ap


def run(args: argparse.Namespace) -> int:
    for nm, v in (("--max-rise", args.max_rise), ("--coarse-step", args.coarse_step),
                  ("--fine-step", args.fine_step), ("--cal-step", args.cal_step),
                  ("--speed", args.speed), ("--px-tol", args.px_tol), ("--z-tol", args.z_tol)):
        if not math.isfinite(v):
            raise SystemExit("%s must be finite, got %r" % (nm, v))
    if not (0 < args.max_rise <= Z_MAX_RISE_MM):
        raise SystemExit("--max-rise must be in (0, %.1f]" % Z_MAX_RISE_MM)
    if not (0 < args.speed <= SPEED_MAX_MM_S):
        raise SystemExit("--speed must be in (0, %.1f]" % SPEED_MAX_MM_S)
    if not (0 < args.cal_step <= 1.0):
        raise SystemExit("--cal-step must be in (0, 1.0] mm")
    if args.min_mean < 5.0:
        raise SystemExit("--min-mean must be >= 5.0")
    if args.rounds < 1 or args.centre_iters < 1:
        raise SystemExit("--rounds and --centre-iters must be >= 1")

    store = WorkspaceStore()
    loc = store.require_location("microscope")
    p = loc.pose
    live = [p.x, p.y, p.z, p.roll, p.pitch, p.yaw]
    # The Z ceiling is measured from the ORIGINAL taught height, never from a
    # previously saved focus height, so repeated runs cannot ratchet it upward.
    taught = list(live)
    taught[2] = float(loc.metadata.get("taught_a1_z", live[2]))

    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    out = args.out_dir / stamp
    out.mkdir(parents=True, exist_ok=True)

    print("A1 (stored)  x=%.4f y=%.4f z=%.4f" % (live[0], live[1], live[2]))
    print("taught A1 z  %.4f   -> Z window %.4f .. %.4f (%.1f mm)"
          % (taught[2], taught[2], taught[2] + args.max_rise, args.max_rise))
    print("XY box       +/-%.1f mm around (%.4f, %.4f)" % (XY_MAX_MM, taught[0], taught[1]))
    print("focus metric %s on the %s"
          % (args.method,
             "whole frame (--full-frame-focus)" if args.full_frame_focus
             else "lit sample + mark edge, growing window from %.0f%%" % (ROI_FRAC * 100)))
    print("mark mode    %s" % args.mode)
    print("output       %s" % out)
    print("mode         %s\n" % ("LIVE -- THE ARM WILL MOVE" if args.execute else "DRY-RUN"))

    # The envelope carries the OPERATOR's ceiling (--max-rise), not the hard
    # constant. --max-rise is validated above to be within (0, Z_MAX_RISE_MM],
    # so this can only ever tighten the box -- but if the envelope used the
    # constant instead, asking for a 3 mm run would still permit 10 mm of rise
    # while the banner claimed 3.
    envelope = Envelope.anchored(
        taught, name="A1", z_max_rise_mm=args.max_rise, xy_max_mm=XY_MAX_MM,
        orient_tol_deg=POSITION_TOL_DEG, readback_slack_mm=0.0, readback_slack_deg=0.0)
    arm = Arm.from_config(live=args.execute, host=args.host, speed_mm_s=args.speed,
                          clear_errors=args.clear_errors, envelope=envelope)
    scope = Microscope.from_config()

    record: dict[str, Any] = {"artifact": "A1 iterative focus + XY centring", "stamp": stamp,
                              "taught_a1": taught, "stored_a1": live, "rounds": [],
                              "full_frame_focus": bool(args.full_frame_focus)}
    status = "dry-run" if not args.execute else "incomplete"
    final_x = final_y = final_z = None
    scale_points: list[tuple[float, Any]] = []

    def _term(signum, _f):
        raise _Terminated("signal %d" % signum)

    for sg in (signal.SIGTERM, signal.SIGHUP):
        try:
            signal.signal(sg, _term)
        except (ValueError, OSError):
            pass

    try:
        if args.execute:
            actual = arm.pose()
            print("[readback] x=%.4f y=%.4f z=%.4f" % tuple(actual[:3]))
            arm.envelope.check_readback(actual, where="starting pose")
            for i, nm in ((0, "x"), (1, "y")):
                if abs(actual[i] - live[i]) > XY_MAX_MM:
                    raise AlignError("the arm is %+.3f mm from A1 in %s; drive it to A1 first"
                                     % (actual[i] - live[i], nm))
            cur_x, cur_y, cur_z = actual[0], actual[1], actual[2]
        else:
            cur_x, cur_y, cur_z = live[0], live[1], live[2]

        prev_z = None
        for rnd in range(1, args.rounds + 1):
            tag = "r%d" % rnd
            print("\n=== round %d/%d ===" % (rnd, args.rounds))
            foc = focus_here(arm, scope, cur_x, cur_y, out, tag, args)
            cur_z = foc["z"]
            if not args.execute:
                record["rounds"].append({"round": rnd, "focus": foc})
                break

            print("  [calib] measuring px/mm at this height")
            cal = calibrate_here(arm, scope, cur_x, cur_y, cur_z, out, tag, args)
            scale_points.append((cur_z, np.array(cal["J"], dtype=float)))

            cen = centre_here(arm, scope, cur_x, cur_y, cur_z, np.array(cal["J"], dtype=float),
                              out, tag, args)
            dx, dy = cen["x"] - cur_x, cen["y"] - cur_y
            cur_x, cur_y = cen["x"], cen["y"]
            dz = 0.0 if prev_z is None else cur_z - prev_z
            prev_z = cur_z
            record["rounds"].append({"round": rnd, "focus": {k: v for k, v in foc.items()
                                                             if k != "samples"},
                                     "focus_samples": foc.get("samples"),
                                     "calibration": cal, "centring": cen,
                                     "dxy_mm": [round(dx, 4), round(dy, 4)],
                                     "dz_mm": round(dz, 4)})
            print("  [round %d] dz=%+.4f mm, dxy=(%+.4f, %+.4f) mm, mark %.1f px off"
                  % (rnd, dz, dx, dy, cen["final_error_px"]))
            if rnd > 1 and abs(dz) <= args.z_tol and cen["final_error_px"] <= args.px_tol \
                    and abs(dx) < 0.02 and abs(dy) < 0.02:
                print("  [round %d] converged" % rnd)
                break

        # -- ALWAYS finish with a focus pass at the final XY -----------------
        if args.execute:
            print("\n=== final focus pass at the aligned XY ===")
            print("    (the stored height must be the one measured AT the stored XY)")
            fin = focus_here(arm, scope, cur_x, cur_y, out, "final", args)
            cur_z = fin["z"]
            record["final_focus"] = {k: v for k, v in fin.items() if k != "samples"}
            record["final_focus_samples"] = fin.get("samples")

            # one last look, for the record -- no move follows it
            time.sleep(args.settle)
            fchk = out / "final_check.jpg"
            gchk = scope.grab_frame(fchk, ordinal=args.frame_ordinal)
            if gchk.ok:
                det = scope.detect_marker(fchk, mode=args.mode)
                if det["candidates"]:
                    c = det["candidates"][0]
                    off, why = marker.best_offset_px(c)
                    record["final_check"] = {"frame": str(fchk), "offset_px": list(off),
                                             "error_px": float(np.hypot(*off)),
                                             "metric": why, "quality": c["quality"],
                                             "disagreement_px": c.get("disagreement_px")}
                    print("[final] mark %.1f px from centre via %s (%s)"
                          % (np.hypot(*off), why, c["quality"]))
                record["final_check_focus"] = scope.measure_focus(
                    fchk, method=args.method, min_mean=args.min_mean,
                    masked=not args.full_frame_focus).as_dict()

            final_x, final_y, final_z = cur_x, cur_y, cur_z
            print("\n[result] A1 aligned: x=%.4f y=%.4f z=%.4f  (taught%+.4f, %+.4f, %+.4f)"
                  % (final_x, final_y, final_z, final_x - taught[0], final_y - taught[1],
                     final_z - taught[2]))
            status = "aligned"

        # -- height model, if we measured at more than one height -------------
        if len({round(z, 3) for z, _ in scale_points}) >= 2:
            model = ps.fit_scale_model([z for z, _ in scale_points],
                                       [J for _, J in scale_points])
            record["pixel_scale_model"] = model
            ps.save(model, out / "pixel_scale_model.json")
            print("[scale] height model fitted from %d heights -> %s"
                  % (len(scale_points), out / "pixel_scale_model.json"))
        elif scale_points:
            record["pixel_scale_single"] = {"z": scale_points[0][0],
                                            "J": scale_points[0][1].tolist()}
            print("[scale] only one height measured; no height model fitted. Run "
                  "pixel_scale_cal.py to build one.")

    except BaseException as exc:
        status = "aborted: %s" % (exc if str(exc) else repr(exc))
        print("\n[ABORT] %s" % status, file=sys.stderr, flush=True)
        arm.retreat()
    finally:
        record["status"] = status
        record["entrypoint"] = " ".join([sys.executable, *sys.argv])
        record["code_sha256"] = hashlib.sha256(
            Path(__file__).resolve().read_bytes()).hexdigest()
        record["limits"] = {"z_max_rise_mm": Z_MAX_RISE_MM, "xy_max_mm": XY_MAX_MM,
                            "roi_frac": ROI_FRAC, "well_pitch_mm": 9.0}
        (out / "manifest.json").write_text(json.dumps(record, indent=2, default=str))
        print("[manifest] %s" % (out / "manifest.json"))
        arm.close()

    if args.save:
        if status != "aligned" or final_z is None:
            print("[saved] NOT saving: status=%s" % status)
        else:
            backup = store.path.with_suffix(".json.bak.%s" % stamp)
            shutil.copyfile(store.path, backup)
            loc_entry = store.workspace.locations["microscope"]
            prev = [loc_entry.pose.x, loc_entry.pose.y, loc_entry.pose.z]
            loc_entry.pose.x, loc_entry.pose.y, loc_entry.pose.z = (
                round(final_x, 6), round(final_y, 6), round(final_z, 6))
            loc_entry.metadata["taught_a1_z"] = taught[2]
            loc_entry.metadata["aligned_at"] = dt.datetime.now().isoformat(timespec="seconds")
            loc_entry.metadata["aligned_previous_pose"] = prev
            loc_entry.metadata["aligned_manifest"] = str(out / "manifest.json")
            loc_entry.metadata["joint_angles_deg_stale"] = ("pose was changed by align_a1; the "
                                                    "recorded joint angles describe the pose "
                                                    "BEFORE alignment")
            store.save()
            print("[saved] A1 (%.4f, %.4f, %.4f) -> (%.4f, %.4f, %.4f)  backup %s"
                  % (*prev, loc_entry.pose.x, loc_entry.pose.y, loc_entry.pose.z, backup))

    return 0 if status in ("aligned", "dry-run") else 1


def main(argv=None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
