#!/usr/bin/env python3
"""
pixel_scale_cal.py -- measure pixels-per-millimetre and direction at several
heights, fit the height model, and store it for everything else to use.

    python3 scripts/pixel_scale_cal.py --execute
    python3 scripts/pixel_scale_cal.py --execute --heights 0,2,4,6,8

WHAT IT MEASURES
  At each height it makes two known moves -- +cal_step along arm X, then along
  arm Y -- and measures how far the IMAGE actually moved, by phase correlation
  over the whole frame. Two measured displacements give the full 2x2 Jacobian
  J(z) mapping millimetres of arm travel to pixels of image travel, including
  the rotation between the camera axes and the arm axes. Nothing about scale,
  sign, or orientation is assumed.

  Phase correlation, not marker tracking: it uses every scratch and speck in the
  field, so it does not care whether the alignment mark is visible, centred, or
  clipped.

WHY SEVERAL HEIGHTS
  Magnification changes with working distance, so px/mm is a function of Z.
  `tools.calib.pixel_scale` fits s(z) = A/(B - z) -- exactly linear in
  1/s, so the fit is a closed-form least squares with no starting guess. Its
  module docstring carries the derivation, the regime where it holds, and what
  goes wrong if you assume a constant scale instead. Three or more heights also
  yield a residual, which is the number that says whether the model is right
  rather than merely fitted.

OUTPUT
  ~/.sdl_lab/robot_arm/pixel_scale.json   (and a copy beside the frames)
  Load it with tools.calib.pixel_scale.load(), then call
  jacobian_at(model, z) for the mapping at any height in range.

SAFETY
  Uses the shared tools.arm.Arm + Envelope guard -- the same guarded
  path every procedure on this rig moves through: Z confined to the taught A1
  height plus the operator's budget, XY confined to a box around A1, every pose
  checked against readback, dry-run by default, and any failure lowers Z to the
  taught A1 height.

  This procedure has never been run on hardware. Only its imports and structure
  were re-pointed at the shared device API during the 2026-07-31 consolidation;
  its measurement logic is untouched.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


import argparse
import datetime as dt
import json
import math
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
from tools.microscope import Microscope

#: Orientation tolerance for the A1 envelope. Same value align_a1.py uses --
#: this procedure shares the identical guard (it used align_a1's GuardedArm,
#: unchanged, before the consolidation).
POSITION_TOL_DEG = 0.5

STORE_PATH = Path.home() / ".sdl_lab" / "robot_arm" / "pixel_scale.json"
DEFAULT_OUT_DIR = _config.PROJECT_ROOT / "dataset" / "captures" / "pixel_scale"


class CalibrationError(RuntimeError):
    pass


def measure_at(arm: Arm, scope: Microscope, x: float, y: float, z: float, out: Path, tag: str,
               args: argparse.Namespace) -> dict[str, Any]:
    """Jacobian at one height. Returns the matrix and its decomposition."""
    arm.move(x, y, z, label="%s settle at height" % tag, takeup=True)
    time.sleep(args.settle)
    f0 = out / ("%s_ref.jpg" % tag)
    frame0 = scope.grab_frame(f0, ordinal=args.frame_ordinal)
    if not frame0.ok:
        raise CalibrationError("%s reference frame: %s" % (tag, frame0.reason))
    focus = scope.measure_focus(frame0.path, method=args.method, min_mean=args.min_mean,
                                masked=not args.full_frame_focus)
    if not focus.usable:
        raise CalibrationError("%s: %s -- the field must be lit and in some focus to correlate"
                               % (tag, focus.reason))

    shifts = {}
    for axis, (dx, dy) in (("x", (args.cal_step, 0.0)), ("y", (0.0, args.cal_step))):
        # XY-only move at constant Z -- no backlash dip (matches the old
        # GuardedArm.goto default of takeup=False).
        arm.move(x + dx, y + dy, z, label="%s +%s" % (tag, axis.upper()), takeup=False)
        time.sleep(args.settle)
        fi = out / ("%s_%s.jpg" % (tag, axis))
        framei = scope.grab_frame(fi, ordinal=args.frame_ordinal)
        if not framei.ok:
            raise CalibrationError("%s %s frame: %s" % (tag, axis, framei.reason))
        a = cv2.imread(str(f0), cv2.IMREAD_GRAYSCALE).astype(np.float32)
        b = cv2.imread(str(fi), cv2.IMREAD_GRAYSCALE).astype(np.float32)
        win = cv2.createHanningWindow((a.shape[1], a.shape[0]), cv2.CV_32F)
        (sx, sy), resp = cv2.phaseCorrelate(a, b, win)
        if resp < args.min_response:
            raise CalibrationError(
                "%s %s: phase-correlation response %.4f is below %.2f; the "
                "frames do not share enough structure to trust the shift"
                % (tag, axis, resp, args.min_response))
        if math.hypot(sx, sy) < 5.0:
            raise CalibrationError(
                "%s %s: a %.2f mm move shifted the image by only %.1f px; the "
                "arm may not have moved" % (tag, axis, args.cal_step, math.hypot(sx, sy)))
        shifts[axis] = {"dx_px": float(sx), "dy_px": float(sy), "response": float(resp)}
        print("      +%.2f mm %s -> (%+8.1f, %+8.1f) px   response %.3f"
              % (args.cal_step, axis.upper(), sx, sy, resp), flush=True)
        arm.move(x, y, z, label="%s back from %s" % (tag, axis.upper()), takeup=False)

    J = ps.jacobian_from_moves((shifts["x"]["dx_px"], shifts["x"]["dy_px"]),
                               (shifts["y"]["dx_px"], shifts["y"]["dy_px"]), args.cal_step)
    d = ps.decompose(J)
    print("      -> %.1f px/mm (X), %.1f px/mm (Y), rotation %+.2f deg, shear %+.2f deg, %s"
          % (d["px_per_mm_x"], d["px_per_mm_y"], d["rotation_deg"], d["shear_deg"],
             "reversing" if d["orientation_reversing"] else "preserving"), flush=True)
    return {"z": z, "rise_mm": round(z - arm.envelope.anchor[2], 4), "J": J.tolist(),
            "shifts": shifts, "focus": focus.as_dict(), **d}


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--execute", action="store_true")
    ap.add_argument("--host", default="192.168.1.201")
    ap.add_argument("--heights", default="0,2,4,6,8",
                    help="comma-separated rises above the taught A1 height, mm")
    ap.add_argument("--cal-step", type=float, default=0.5)
    ap.add_argument("--speed", type=float, default=3.0)
    ap.add_argument("--settle", type=float, default=0.8)
    ap.add_argument("--frame-ordinal", type=int, default=2)
    ap.add_argument("--method", default="tenengrad",
                    choices=("laplacian", "tenengrad", "brenner"))
    ap.add_argument("--min-mean", type=float, default=30.0)
    ap.add_argument("--min-response", type=float, default=0.02)
    ap.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    ap.add_argument("--clear-errors", action="store_true")
    ap.add_argument("--store", type=Path, default=STORE_PATH)
    ap.add_argument("--full-frame-focus", action="store_true",
                    help="score the per-height focus check on the whole frame instead of "
                         "the masked ROI (old behaviour, kept reachable for comparison)")
    return ap


def run(args: argparse.Namespace) -> int:
    try:
        rises = [float(v) for v in args.heights.split(",") if v.strip()]
    except ValueError as exc:
        raise SystemExit("--heights must be comma-separated numbers") from exc
    if len(rises) < 2:
        raise SystemExit("need at least 2 heights to fit a height model")
    if any(not math.isfinite(r) for r in rises):
        raise SystemExit("--heights must all be finite")
    if min(rises) < 0 or max(rises) > Z_MAX_RISE_MM:
        raise SystemExit("every height must be within 0..%.1f mm above the taught A1 height"
                         % Z_MAX_RISE_MM)
    if len(set(round(r, 6) for r in rises)) < 2:
        raise SystemExit("the heights must not all be the same")
    if not (0 < args.cal_step <= 1.0):
        raise SystemExit("--cal-step must be in (0, 1.0] mm")
    if not (0 < args.speed <= SPEED_MAX_MM_S):
        raise SystemExit("--speed must be in (0, %.1f] mm/s" % SPEED_MAX_MM_S)

    store = WorkspaceStore()
    loc = store.require_location("microscope")
    p = loc.pose
    live = [p.x, p.y, p.z, p.roll, p.pitch, p.yaw]
    # Same immutable-baseline logic as align_a1.py: the Z ceiling is measured
    # from the ORIGINAL taught height, never a previously saved focus height.
    taught = list(live)
    taught[2] = float(loc.metadata.get("taught_a1_z", live[2]))

    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    out = args.out_dir / stamp
    out.mkdir(parents=True, exist_ok=True)

    print("taught A1 z : %.4f" % taught[2])
    print("heights     : %s mm above it" % ", ".join("%+.1f" % r for r in rises))
    print("cal step    : %.2f mm per axis" % args.cal_step)
    print("output      : %s" % out)
    print("mode        : %s\n" % ("LIVE -- THE ARM WILL MOVE" if args.execute else "DRY-RUN"))

    envelope = Envelope.anchored(
        taught, name="A1", z_max_rise_mm=Z_MAX_RISE_MM, xy_max_mm=XY_MAX_MM,
        orient_tol_deg=POSITION_TOL_DEG, readback_slack_mm=0.0, readback_slack_deg=0.0)
    arm = Arm.from_config(live=args.execute, host=args.host, speed_mm_s=args.speed,
                          clear_errors=args.clear_errors, envelope=envelope)
    scope = Microscope.from_config()

    points: list[dict[str, Any]] = []
    status = "dry-run" if not args.execute else "incomplete"

    def _term(signum, _f):
        raise CalibrationError("signal %d" % signum)

    for sg in (signal.SIGTERM, signal.SIGHUP):
        try:
            signal.signal(sg, _term)
        except (ValueError, OSError):
            pass

    try:
        if args.execute:
            actual = arm.verify_at_anchor(where="starting pose")
            x, y = actual[0], actual[1]
            print("[readback] x=%.4f y=%.4f z=%.4f -- calibrating at this XY\n"
                  % (actual[0], actual[1], actual[2]))
        else:
            x, y = live[0], live[1]

        for i, rise in enumerate(rises, 1):
            z = taught[2] + rise
            print("[height %d/%d] rise %+.2f mm (z=%.4f)" % (i, len(rises), rise, z))
            if not args.execute:
                arm.move(x, y, z, label="dry-run height %d" % i, takeup=False)
                continue
            points.append(measure_at(arm, scope, x, y, z, out, "h%02d" % i, args))

        if args.execute:
            model = ps.fit_scale_model([pt["z"] for pt in points],
                                       [np.array(pt["J"]) for pt in points])
            model["measured_points"] = points
            model["taught_a1_z"] = taught[2]
            model["cal_step_mm"] = args.cal_step
            model["produced_at"] = dt.datetime.now().isoformat(timespec="seconds")
            model["entrypoint"] = " ".join([sys.executable, *sys.argv])
            ps.save(model, out / "pixel_scale_model.json")
            ps.save(model, args.store)
            status = "calibrated"

            print("\n=== height model ===")
            for key in ("px_per_mm_x", "px_per_mm_y"):
                m = model["axes"][key]
                if m["form"] == "constant":
                    print("  %s: constant %.1f px/mm (telecentric over this range)"
                          % (key, m["s0"]))
                else:
                    print("  %s: s(z) = %.1f / (%.4f - z)   [A, B]  residual %s"
                          % (key, m["A"], m["B"],
                             "n/a" if m["recip_rms_residual"] is None
                             else "%.2e" % m["recip_rms_residual"]))
                print("       measured: %s"
                      % ", ".join("%.1f" % v for v in m["measured"]))
            r = model["rotation"]
            print("  rotation: %.2f deg (spread %.2f deg over the range, slope %+.3f deg/mm)"
                  % (r["mean_deg"], r["spread_deg"], r["slope_deg_per_mm"]))
            if r["spread_deg"] > 1.0:
                print("       NOTE: >1 deg of drift suggests the optical axis is not "
                      "parallel to the travel axis")
            print("\n  stored: %s" % args.store)
            print("  use:    from tools.calib import pixel_scale as ps")
            print("          J = ps.jacobian_at(ps.load(%r), z)" % str(args.store))
    except BaseException as exc:
        status = "aborted: %s" % (exc if str(exc) else repr(exc))
        print("\n[ABORT] %s" % status, file=sys.stderr, flush=True)
        arm.retreat()
    finally:
        (out / "manifest.json").write_text(json.dumps(
            {"artifact": "pixel-scale height calibration", "status": status,
             "stamp": stamp, "taught_a1": taught, "heights_mm": rises,
             "cal_step_mm": args.cal_step, "points": points,
             "entrypoint": " ".join([sys.executable, *sys.argv])},
            indent=2, default=str))
        print("[manifest] %s" % (out / "manifest.json"))
        arm.close()

    return 0 if status in ("calibrated", "dry-run") else 1


def main(argv=None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
