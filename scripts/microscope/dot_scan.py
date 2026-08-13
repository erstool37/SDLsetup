#!/usr/bin/env python3
"""dot_scan.py -- sweep the tray under the objective until the printed dots are found.

    python3 scripts/microscope/dot_scan.py                          # plan only
    python3 scripts/microscope/dot_scan.py --execute
    python3 scripts/microscope/dot_scan.py --execute --radius 12 --overlap 0.35

WHAT THIS IS FOR
----------------
The tray carries three printed dots -- BLACK on A1, RED on B1, BLUE on A2 --
9.0 mm apart. After the arm drives to the taught A1 pose the black dot is often
not under the objective at all, because the tray does not seat identically every
time. Before anything can be centred, *something* has to be found.

Seeing ANY ONE dot is enough. The colour says which well you are over, so the
move back to A1 follows from a single sighting. That is why the scan looks for
all three rather than hunting the black one specifically.

WHY IT MEASURES THE SCALE FIRST
-------------------------------
The step between frames is derived from the field of view, and the field of view
is a function of height: the objective is fixed, so raising the tray magnifies.
The one recorded number -- 861 px/mm -- was measured on 2026-07-31 at A1+3.0 mm,
and the focus plane has since moved to about A1+8.07 mm. Reusing it here would
size the step from a magnification that no longer applies, and a step that is
too big walks straight over a dot without ever seeing it.

So the scan measures the pixel-per-millimetre Jacobian *at the height it is
about to scan at*, by two known small moves and phase correlation, and derives
the step from that. If the measurement fails it says so and falls back to a
deliberately small step rather than guessing.

WHY THE MOVES ARE SMALL
-----------------------
Measured on this rig 2026-08-04: at A1+7.6 mm -- only 0.47 mm below the focus
plane -- the variance of the Laplacian had already collapsed from 364 to 7.6,
and phase correlation between two frames 0.5 mm apart returned a displacement of
0.03 px. Not because the arm had not moved, but because there was no resolvable
structure left to correlate. **The depth of field here is well under half a
millimetre.** The calibration step is therefore kept small enough to stay in
focus and to keep the two frames overlapping heavily.

SAFETY
------
The envelope is anchored on the pose the arm is **already at**, with the Z
budget pinned shut: ``z_min_rise_mm = z_max_rise_mm = 0``, so the permitted Z
window is a single value and no pose this script can build will change the
height. The tray is 8 mm above taught A1 and the whole remaining clearance to
the objective is the operator's asserted 10 mm, so Z is not this routine's to
move. Everything it does is lateral.

Backlash take-up is disabled for the same reason: the dip is a Z move, and a
pinned Z window would refuse it -- correctly, but only after a failed move.

XY is boxed to the search radius plus the calibration step. That box is far
wider than the 2.5 mm closed-loop default, which is deliberate and is the reason
the box is stated explicitly here instead of inherited: a search that cannot
leave 2.5 mm cannot find a dot 6 mm away.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


import argparse  # noqa: E402
import datetime  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402
import time  # noqa: E402

from tools import config as _config  # noqa: E402
from tools.arm import Arm  # noqa: E402
from tools.arm.safety import Envelope, SafetyError  # noqa: E402
from tools.calib import search as _search  # noqa: E402
from tools.microscope import Microscope  # noqa: E402
from tools.microscope import marker as _marker  # noqa: E402

ANCHOR = "microscope"
ARM_HOST_DEFAULT = "192.168.1.201"

#: Known move used to measure px/mm, mm. Small enough to stay inside a depth of
#: field measured at well under 0.5 mm, and to leave the two frames overlapping
#: by ~90%, which is what phase correlation wants.
CAL_STEP_MM = 0.30

#: Below this correlation response the displacement is not trusted. The failed
#: 2026-08-04 attempt returned 0.03 px at a response of 0.13 on frames with no
#: resolvable structure, so response alone is NOT sufficient -- the measured
#: shift is also checked against the shift the move should have produced.
MIN_RESPONSE = 0.05

#: A calibration move must move the image by at least this many pixels to be
#: believed. 0.30 mm at even 100 px/mm is 30 px; anything under this means the
#: frames are not resolving the motion.
MIN_SHIFT_PX = 8.0

#: Fallback field of view, mm, used only when the measurement fails. Small on
#: purpose: an over-small step is slow, an over-large one misses the dot.
FALLBACK_FOV_MM = (1.6, 1.3)

#: Speed for the short lateral hops, mm/s.
SCAN_SPEED_MM_S = 5.0

#: Seconds to let the tray stop ringing before asking for a frame.
SETTLE_S = 0.8

#: Take the SECOND published frame after a move. The first may have been exposed
#: before the move finished -- publishing runs ~3.0 s apart while a capture takes
#: ~2 s. Measured in tools/microscope/api.py; do not lower this to 1.
FRAME_ORDINAL = 2
FRAME_TIMEOUT_S = 30.0


def _now_stamp() -> str:
    return datetime.datetime.now().strftime("%Y%m%d-%H%M%S")


def _fmt_xy(pose) -> str:
    return "x=%9.4f y=%9.4f z=%9.4f" % (pose[0], pose[1], pose[2])


def phase_shift(path_a: Path, path_b: Path):
    """Pixel displacement from frame A to frame B, and the correlation response.

    Uses every scratch and speck in the field rather than tracking a marker, so
    it works on blank tray -- which is the whole point, since during a search
    there is usually no marker in view.
    """
    import cv2
    import numpy as np

    a = cv2.imread(str(path_a), cv2.IMREAD_GRAYSCALE)
    b = cv2.imread(str(path_b), cv2.IMREAD_GRAYSCALE)
    if a is None or b is None:
        raise MeasurementError("could not read %s or %s" % (path_a, path_b))
    if a.shape != b.shape:
        raise MeasurementError("frames differ in size: %s vs %s" % (a.shape, b.shape))
    win = cv2.createHanningWindow((a.shape[1], a.shape[0]), cv2.CV_32F)
    (du, dv), response = cv2.phaseCorrelate(np.float32(a) * win, np.float32(b) * win)
    sharp = float(cv2.Laplacian(a, cv2.CV_64F).var())
    return (float(du), float(dv)), float(response), sharp, (a.shape[1], a.shape[0])


class MeasurementError(RuntimeError):
    """The pixel scale could not be measured. Never guessed around silently."""


def measure_jacobian(arm, scope, here, out_dir, *, cal_step, speed, settle):
    """px/mm along each arm axis, measured HERE, by two known moves.

    Returns ``(fov_mm, detail)``. Raises :class:`MeasurementError` rather than
    returning a number it does not believe -- a wrong scale silently sizes the
    scan step, and an over-large step walks over the dot it was looking for.
    """
    frames = {}

    def hop(dx, dy, tag):
        arm.move(here[0] + dx, here[1] + dy, here[2], speed=speed,
                 label="cal %s" % tag, takeup=False)
        # Gate on the instant the move RETURNED, so the accepted frame is one
        # the publisher itself says began exposing after the tray stopped.
        moved_at = time.time()
        time.sleep(settle)
        dest = out_dir / ("cal_%s.jpg" % tag)
        frame = scope.grab_frame(dest, after=moved_at, ordinal=FRAME_ORDINAL,
                                 timeout_s=FRAME_TIMEOUT_S)
        frames[tag] = frame.require()
        return frames[tag]

    ref = hop(0.0, 0.0, "ref")
    px = {}
    detail = {"cal_step_mm": cal_step, "axes": {}}
    for axis, (dx, dy) in (("x", (cal_step, 0.0)), ("y", (0.0, cal_step))):
        moved = hop(dx, dy, axis)
        (du, dv), response, sharp, size = phase_shift(ref, moved)
        shift = math.hypot(du, dv)
        detail["axes"][axis] = {"du_px": du, "dv_px": dv, "shift_px": shift,
                                "response": response, "ref_var_laplacian": sharp,
                                "frame_px": list(size)}
        if response < MIN_RESPONSE or shift < MIN_SHIFT_PX:
            raise MeasurementError(
                "arm %s: a %.2f mm move shifted the image %.2f px at response %.3f "
                "(need >= %.1f px and >= %.2f). The frames carry no resolvable "
                "structure -- check focus before trusting any scale."
                % (axis, cal_step, shift, response, MIN_SHIFT_PX, MIN_RESPONSE))
        px[axis] = (du / cal_step, dv / cal_step)
        arm.move(here[0], here[1], here[2], speed=speed, label="cal back", takeup=False)

    width_px, height_px = detail["axes"]["x"]["frame_px"]
    # px/mm magnitude along each ARM axis: how far the image travels per mm of
    # arm travel, independent of which image axis it lands on.
    scale_x = math.hypot(*px["x"])
    scale_y = math.hypot(*px["y"])
    if scale_x <= 0 or scale_y <= 0:
        raise MeasurementError("non-positive scale: %r %r" % (scale_x, scale_y))

    fov_x = _extent_mm(px["x"], scale_x, width_px, height_px)
    fov_y = _extent_mm(px["y"], scale_y, width_px, height_px)
    detail.update(px_per_mm_x=scale_x, px_per_mm_y=scale_y,
                  fov_mm=[fov_x, fov_y],
                  note="fov is the frame's extent along each arm axis's own measured "
                       "image direction, not the short frame axis for both")
    return (fov_x, fov_y), detail


def _extent_mm(direction, scale_px_per_mm, width_px, height_px) -> float:
    """How many mm of travel the frame covers along one arm axis.

    The frame is a ``width x height`` rectangle in image space, and an arm axis
    maps onto some direction inside it. The distance you can travel along that
    direction before leaving the frame is set by whichever edge you reach first.

    Doing this properly matters more than it looks. The first version used
    ``min(width, height)`` for BOTH axes -- safe, but on a 3072 x 2048 sensor
    that discards a third of the coverage along whichever arm axis runs the long
    way, and the step derived from it is a third too small. On a 169-point scan
    that is most of the run time, spent re-photographing ground already seen.
    """
    du, dv = direction
    norm = math.hypot(du, dv)
    if norm <= 0:
        return min(width_px, height_px) / scale_px_per_mm
    cu, cv = abs(du) / norm, abs(dv) / norm
    limits = []
    if cu > 1e-9:
        limits.append(width_px / cu)
    if cv > 1e-9:
        limits.append(height_px / cv)
    return min(limits) / scale_px_per_mm


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--execute", action="store_true",
                    help="connect and move (default: dry-run, commands nothing)")
    ap.add_argument("--host", default=ARM_HOST_DEFAULT)
    ap.add_argument("--radius", type=float, default=10.0,
                    help="search half-width from the current XY, mm (default: %(default)s)")
    ap.add_argument("--overlap", type=float, default=0.25,
                    help="fraction of frame overlap between steps (default: %(default)s)")
    ap.add_argument("--step", type=float, default=None,
                    help="override the step, mm. Default: derived from the MEASURED fov")
    ap.add_argument("--cal-step", type=float, default=CAL_STEP_MM)
    ap.add_argument("--speed", type=float, default=SCAN_SPEED_MM_S)
    ap.add_argument("--settle", type=float, default=SETTLE_S)
    ap.add_argument("--stop-on-first", action="store_true",
                    help="stop at the first dot. Default is to sweep the whole box, "
                         "because several sightings are what let the plate frame be solved")
    ap.add_argument("--dot-mm", type=float, default=1.0,
                    help="printed dot diameter in mm; with the measured px/mm this "
                         "gives the expected radius in pixels. 0 disables the size "
                         "prior (NOT advised: on 2026-08-04 the dark discriminant "
                         "fired on 21 of 28 dot-free frames without it).")
    ap.add_argument("--skip-calibration", action="store_true",
                    help="do not measure the scale; use --step or the fallback")
    ap.add_argument("--out-dir", type=Path, default=None)
    return ap


def run(args: argparse.Namespace) -> int:
    out_root = args.out_dir or (_config.PROJECT_ROOT / "dataset" / "captures" / "dot_scan")
    out_dir = out_root / _now_stamp()

    arm = Arm.from_config(live=args.execute, host=args.host, clear_errors=False,
                          anchor=ANCHOR, speed_mm_s=args.speed)
    taught = arm.location_pose(ANCHOR)

    if args.execute:
        here = arm.pose()
    else:
        from tools.arm.driver import ArmError, XArmConnection
        probe = XArmConnection(args.host, live=True, clear_errors=False,
                               log=lambda _m: None)
        try:
            here = probe.read_pose()
        except ArmError as exc:
            raise SystemExit("cannot read the arm's pose: %s" % exc) from None
        finally:
            probe.disconnect()

    rise = here[2] - taught[2]
    box = float(args.radius) + float(args.cal_step) + 0.5

    # Anchored on where the arm ALREADY IS, with the Z budget pinned shut. The
    # permitted window becomes the single value here[2], so nothing this script
    # builds can change the height.
    envelope = Envelope.anchored(
        here, name="scan origin", z_min_rise_mm=0.0, z_max_rise_mm=0.0,
        xy_max_mm=box, orient_tol_deg=0.5, speed_max_mm_s=max(args.speed, 10.0))
    scanning = arm.with_envelope(envelope)

    print("dot_scan -- find the printed dots on the tray")
    print("  taught A1   %s" % _fmt_xy(taught))
    print("  arm is at   %s   (A1 %+.3f mm)" % (_fmt_xy(here), rise))
    print("  Z is PINNED at %.4f -- this routine only moves laterally" % here[2])
    print("  XY box      +/-%.2f mm around the current XY" % box)
    print("  output      %s" % out_dir)
    print()

    if not args.execute:
        fov = tuple(args.step for _ in range(2)) if args.step else FALLBACK_FOV_MM
        plan = _search.scan_plan(fov_mm=fov, max_offset_mm=args.radius,
                                 overlap=args.overlap)
        print("  DRY-RUN. With a %.2f x %.2f mm field and %.0f%% overlap:"
              % (fov[0], fov[1], 100 * args.overlap))
        print("    %d scan points, largest uncovered gap %.3f mm"
              % (len(plan), _search.coverage_gap_mm(plan, fov)))
        print("    est. %.1f min at ~4 s/point" % (len(plan) * 4 / 60.0))
        print()
        print("  Nothing was commanded. Add --execute to scan.")
        return 0

    out_dir.mkdir(parents=True, exist_ok=True)
    scope = Microscope.from_config()
    record = {"started_at": datetime.datetime.now().isoformat(),
              "taught_a1": list(taught), "scan_origin": list(here),
              "rise_above_taught_a1_mm": rise, "radius_mm": args.radius,
              "overlap": args.overlap, "z_pinned_at": here[2]}

    with scanning.occupied("dot_scan"):
        fov, cal = _measure_or_fall_back(scanning, scope, here, out_dir, args, record)

        step = args.step if args.step else min(fov) * (1.0 - args.overlap)
        plan = _search.scan_plan(fov_mm=fov, max_offset_mm=args.radius,
                                 overlap=args.overlap)
        gap = _search.coverage_gap_mm(plan, fov)
        print()
        print("  field of view %.3f x %.3f mm  ->  step %.3f mm" % (fov[0], fov[1], step))
        print("  %d points, largest uncovered gap %.4f mm  (<=0 means no holes)"
              % (len(plan), gap))
        print("  est. %.1f min" % (len(plan) * 4 / 60.0))
        print()
        record.update(fov_mm=list(fov), step_mm=step, n_points=len(plan),
                      coverage_gap_mm=gap, calibration=cal)

        prior, prior_skip = _build_prior(cal, args)
        if prior is None:
            print("  SIZE PRIOR OFF: %s" % prior_skip)
            print("    every dark region inside the lit field will be reported as a "
                  "sighting,")
            print("    including machined features. Treat hits as unconfirmed.")
            record["size_prior"] = {"applied": False, "reason": prior_skip}
        else:
            lo, hi = prior.band_px
            print("  size prior: %.2f mm mark at %.1f px/mm -> radius %.0f px "
                  "(accept %.0f-%.0f)"
                  % (prior.diameter_mm, prior.px_per_mm, prior.radius_px, lo, hi))
            record["size_prior"] = {"applied": True, **prior.as_dict()}
        sightings = _walk(scanning, scope, here, plan, out_dir, args, record,
                          prior=prior)

    record["finished_at"] = datetime.datetime.now().isoformat()
    record["sightings"] = sightings
    (out_dir / "manifest.json").write_text(json.dumps(record, indent=2, default=str))

    print()
    print("=" * 72)
    if not sightings:
        print("NO DOTS FOUND in %d frames over +/-%.1f mm." % (len(plan), args.radius))
        print("  That is a result, not a crash. What it rules out: the black, red and")
        print("  blue dots are not within %.1f mm of where the arm started, at this" % args.radius)
        print("  height, with this illumination.")
        print("  Next: widen with --radius, or check the frames in")
        print("  %s -- if they are blurred, the focus plane moved." % out_dir)
    else:
        print("FOUND %d sighting(s):" % len(sightings))
        for s in sightings:
            print("  %-6s at scan offset (%+.3f, %+.3f) mm   arm (%.4f, %.4f)"
                  % (s["colour"], s["scan_dx_mm"], s["scan_dy_mm"],
                     s["arm_x"], s["arm_y"]))
            print("         offset in frame %+.1f, %+.1f px   margin %.1f   quality %s"
                  % (s["offset_px"][0], s["offset_px"][1], s["margin"], s["quality"]))
            print("         %s" % s["frame"])
    print("manifest: %s" % (out_dir / "manifest.json"))
    return 0 if sightings else 4


def _measure_or_fall_back(arm, scope, here, out_dir, args, record):
    if args.skip_calibration:
        fov = (args.step, args.step) if args.step else FALLBACK_FOV_MM
        print("  scale measurement SKIPPED; using fov %.2f x %.2f mm" % fov)
        return fov, {"skipped": True}
    print("  measuring px/mm here (two %.2f mm moves, phase correlation)"
          % args.cal_step)
    try:
        fov, cal = measure_jacobian(arm, scope, here, out_dir, cal_step=args.cal_step,
                                    speed=args.speed, settle=args.settle)
        print("    px/mm  arm-X %.1f   arm-Y %.1f" % (cal["px_per_mm_x"],
                                                      cal["px_per_mm_y"]))
        return fov, cal
    except (MeasurementError, SafetyError) as exc:
        print("    MEASUREMENT FAILED: %s" % exc)
        fov = (args.step, args.step) if args.step else FALLBACK_FOV_MM
        print("    falling back to a deliberately small field %.2f x %.2f mm; the step"
              % fov)
        print("    is now a guess, so coverage is asserted rather than measured.")
        record["calibration_failed"] = str(exc)
        return fov, {"failed": str(exc)}


def _build_prior(cal, args):
    """The prior, or None with the reason it could not be built.

    Requires a MEASURED scale. `cal` carries "skipped" or "failed" when the
    Jacobian could not be measured, and in that case the fallback field of
    view is an assumption -- deriving an expected radius from it would dress a
    guess up as a size check.
    """
    if not args.dot_mm:
        return None, "disabled by --dot-mm 0"
    if cal.get("skipped") or cal.get("failed"):
        return None, ("no measured px/mm at this height (%s), so the expected "
                      "radius would be a guess"
                      % ("calibration skipped" if cal.get("skipped")
                         else "calibration failed"))
    scale = 0.5 * (float(cal["px_per_mm_x"]) + float(cal["px_per_mm_y"]))
    prior = _marker.MarkPrior(diameter_mm=args.dot_mm, px_per_mm=scale)
    return prior, ""


def _walk(arm, scope, here, plan, out_dir, args, record, prior=None):
    """Drive the plan, classify every frame, and collect what was seen.

    Records EVERY frame's verdict, not only the hits: a scan that found nothing
    is only trustworthy if it can be shown which positions were actually looked
    at and whether those frames were usable.
    """
    sightings = []
    visited = []
    for i, point in enumerate(plan, start=1):
        dx, dy = point.dx_mm, point.dy_mm
        tx, ty = here[0] + dx, here[1] + dy
        try:
            arm.move(tx, ty, here[2], speed=args.speed,
                     label="scan %d/%d" % (i, len(plan)), takeup=False)
        except SafetyError as exc:
            print("  [%3d/%d] (%+.2f,%+.2f) REFUSED: %s" % (i, len(plan), dx, dy, exc))
            visited.append({"i": i, "dx_mm": dx, "dy_mm": dy, "refused": str(exc)})
            continue

        moved_at = time.time()
        time.sleep(args.settle)
        dest = out_dir / ("scan_%03d_dx%+06.2f_dy%+06.2f.jpg" % (i, dx, dy))
        frame = scope.grab_frame(dest, after=moved_at, ordinal=FRAME_ORDINAL,
                                 timeout_s=FRAME_TIMEOUT_S)
        if not frame.ok:
            print("  [%3d/%d] (%+.2f,%+.2f) no frame: %s"
                  % (i, len(plan), dx, dy, frame.reason))
            visited.append({"i": i, "dx_mm": dx, "dy_mm": dy, "frame_error": frame.reason})
            continue

        verdict = _marker.classify_dots(frame.require(), expect=prior)
        # `present` means the discriminant fired; `prior_matched` means it also
        # looked like a dot. Both are recorded: their difference IS the
        # false-positive rate, and it is only measurable if both survive.
        loose = verdict.get("present") or []
        present = (verdict.get("prior_matched") if prior is not None
                   else loose) or []
        entry = {"i": i, "dx_mm": dx, "dy_mm": dy, "frame": str(dest),
                 "present": list(present), "fired": list(loose),
                 "conflicts": verdict.get("conflicts") or []}
        visited.append(entry)

        if present:
            for colour in present:
                rec = (verdict.get("colours") or {}).get(colour) or {}
                sightings.append({
                    "colour": colour, "scan_dx_mm": dx, "scan_dy_mm": dy,
                    "arm_x": tx, "arm_y": ty, "arm_z": here[2],
                    "offset_px": rec.get("offset_px") or [0.0, 0.0],
                    "margin": rec.get("margin", 0.0),
                    "quality": rec.get("quality", "?"),
                    "frame": str(dest),
                })
            print("  [%3d/%d] (%+.2f,%+.2f)  *** %s ***"
                  % (i, len(plan), dx, dy, ", ".join(present).upper()))
            if args.stop_on_first:
                print("  stopping at the first sighting (--stop-on-first)")
                break
        elif i % 10 == 0 or i == 1:
            print("  [%3d/%d] (%+.2f,%+.2f)  nothing" % (i, len(plan), dx, dy))

    record["visited"] = visited
    # Park back at the scan origin so the next routine starts where this one did.
    try:
        arm.move(here[0], here[1], here[2], speed=args.speed, label="back to origin",
                 takeup=False)
    except SafetyError as exc:
        print("  could not return to the scan origin: %s" % exc)
    return sightings


def main(argv=None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
