#!/usr/bin/env python3
"""
xy_center.py -- centre the operator's black dot in well A1 by moving X/Y only.

WHAT IT DOES
  1. Measure the pixel<->mm mapping empirically: make two known small moves
     (+dx in X, then +dy in Y) and measure how far the image actually shifted,
     by phase correlation over the whole frame. This yields the full 2x2 Jacobian
     including any rotation between the arm axes and the camera axes. Nothing is
     assumed about scale, sign, or orientation.
  2. Detect the dark dot and report its centroid -- with quality flags, never a
     decision (surface law: sensors report, they do not decide).
  3. Move so the dot lands on the image centre, re-measure, and repeat until it
     converges or the iteration budget runs out.
  4. Run a small confirmation ring around the result and keep the best point, so
     the answer is checked by sampling and not just by one model inversion.

HARD SAFETY RULES -- enforced through tools.arm's shared guard path
  X1 Z NEVER CHANGES. The safety envelope is built anchored at the Z read back
     at the start of the run, with its rise window collapsed to zero both ways
     (z_min_rise_mm=0, z_max_rise_mm=0), so any target whose Z differs raises.
     Focus is not this routine's business.
  X2 XY is confined to a box of +/- XY_MAX_MM around the A1 anchor. A 96-well
     well is ~6.4 mm across, so 2.5 mm cannot reach the wall. Out of box RAISES;
     it is never clamped.
  X3 Readback after every move; the machine, not the arithmetic, is what gets
     compared against the box (Arm.move() re-reads and re-checks after every
     commanded pose).
  X4 Dry-run default. Without --execute nothing connects and nothing moves.
  X5 Any failure returns the arm to the pose it started from.
  X6 The dot detector reports candidates and flags; this module decides. If it
     reports nothing usable, the run aborts WITHOUT moving anywhere new.

WHY PHASE CORRELATION FOR CALIBRATION AND NOT THE DOT
  The dot is soft-edged and may be clipped by the frame edge, so its centroid is
  a poor displacement reference. Whole-frame phase correlation uses every scratch
  and speck in the field and is unbiased by clipping.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


import argparse
import datetime as dt
import json
import shutil
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

from tools import config as _config
from tools import microscope
from tools.arm import Arm, Envelope
from tools.calib import pixel_scale as ps
from tools.calib.pixel_scale import image_shift_px

#: XY excursion budget around the A1 anchor, mm. A 96-well well is ~6.4 mm
#: across; 2.5 mm cannot reach the wall.
XY_MAX_MM = 2.5

#: Calibration step. Big enough to measure well above correlation noise, small
#: enough to stay deep inside the box.
CAL_STEP_MM = 0.50

#: Convergence: stop when the dot is this close to the image centre.
CONVERGE_PX = 25.0

DEFAULT_OUT_DIR = _config.PROJECT_ROOT / "dataset" / "captures" / "xy_center"
ARM_HOST_DEFAULT = "192.168.1.201"


class XYError(RuntimeError):
    pass


# ---------------------------------------------------------------------------
# Candidate selection -- reports nothing, decides which blob is the dot (X6)
# ---------------------------------------------------------------------------

def _chosen_candidate(det: dict[str, Any]) -> dict[str, Any]:
    """Pick which detected candidate is the operator's dot.

    microscope.marker.detect() already sorts candidates largest-first, so the
    largest is the mark. No usable candidate aborts WITHOUT moving anywhere new.
    """
    if not det["candidates"]:
        raise XYError("no dark blob found in the field. Is the marked well under the lens, "
                      "and is the dot inside the field of view?")
    return det["candidates"][0]


# ---------------------------------------------------------------------------
# X5 -- any failure returns the arm to the pose it started from
# ---------------------------------------------------------------------------

def _restore(a: Arm, start_x: float | None, start_y: float | None,
            z_locked: float | None) -> None:
    """Best-effort XY restore on failure.

    Arm.retreat() only fires automatically through the `with a:` context-manager
    form, which this procedure does not use, and even then it only lowers Z to
    the envelope's anchor height -- a no-op here since Z is locked at the
    anchor already. It does not restore XY, which is the entire failure mode
    this guard protects against, so the restore stays local: it goes through
    the same envelope-guarded a.move() as every other commanded pose in this
    run, so a restore that would itself leave the box still raises rather than
    silently moving.
    """
    if not (a.settings.live and a.engaged and start_x is not None and start_y is not None):
        return
    try:
        print("  [safety] returning to the starting XY", flush=True)
        a.move(start_x, start_y, z_locked, label="restore start XY", takeup=False)
    except BaseException as exc:
        print("  [safety] RESTORE FAILED: %r" % (exc,), file=sys.stderr, flush=True)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--execute", action="store_true")
    ap.add_argument("--host", default=ARM_HOST_DEFAULT)
    ap.add_argument("--speed", type=float, default=3.0)
    ap.add_argument("--settle", type=float, default=0.8)
    ap.add_argument("--cal-step", type=float, default=CAL_STEP_MM)
    ap.add_argument("--iters", type=int, default=3, help="centring iterations (default 3)")
    ap.add_argument("--converge-px", type=float, default=CONVERGE_PX)
    ap.add_argument("--ring", type=float, default=0.15,
                    help="confirmation ring radius in mm (0 disables)")
    ap.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    ap.add_argument("--mode", default="dark", choices=("dark", "red"),
                    help="marker type: black mark (dark) or red mark (red, far more robust)")
    ap.add_argument("--save", action="store_true",
                    help="persist the corrected XY into the A1 anchor")
    return ap


# ---------------------------------------------------------------------------
# The procedure
# ---------------------------------------------------------------------------

def run(args: argparse.Namespace) -> int:
    if not (0 < args.cal_step <= 1.0):
        raise SystemExit("--cal-step must be in (0, 1.0] mm")
    if not (0 < args.speed <= 30.0):
        raise SystemExit("--speed must be in (0, 30] mm/s")
    if args.iters < 1:
        raise SystemExit("--iters must be >= 1")

    a = Arm.from_config(host=args.host, live=args.execute, speed_mm_s=args.speed,
                        anchor="microscope", clear_errors=False)
    anchor = a.location_pose("microscope")

    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    out = args.out_dir / stamp
    out.mkdir(parents=True, exist_ok=True)

    record: dict[str, Any] = {"artifact": "A1 XY dot-centring", "stamp": stamp,
                              "anchor_a1": anchor, "xy_box_mm": XY_MAX_MM,
                              "steps": []}
    status = "dry-run"
    start_x = start_y = None
    z_locked = None

    try:
        if args.execute:
            start = a.pose()
            start_x, start_y = start[0], start[1]
            z_locked = start[2]
            record["start_pose"] = start
            record["z_locked"] = z_locked
            print("start pose : x=%.4f y=%.4f z=%.4f  (A1%+.3f,%+.3f,%+.3f)"
                  % (start[0], start[1], start[2],
                     start[0] - anchor[0], start[1] - anchor[1], start[2] - anchor[2]))
            anchor_z_locked = list(anchor)
            anchor_z_locked[2] = z_locked
            # readback_slack_mm reproduces the retired XYArm.check tolerance of
            # 1e-3 mm on Z exactly. With slack 0 the Z window would be
            # +/-EPS (1e-6 mm), which is finer than the controller reports its
            # own position and would abort a run that is in fact holding Z.
            # XY is unaffected in practice: 2.5 + 0.001 is still the 2.5 mm box.
            envelope = Envelope.anchored(
                anchor_z_locked, name="A1", z_min_rise_mm=0.0, z_max_rise_mm=0.0,
                xy_max_mm=XY_MAX_MM, orient_tol_deg=0.5,
                readback_slack_mm=1e-3, readback_slack_deg=0.0,
            )
            envelope.check_readback(start, where="start pose")
            a = a.with_envelope(envelope)
            print("Z locked at %.4f -- this routine will not change it\n" % z_locked)
        else:
            print("DRY-RUN: would read the current pose, lock Z, and calibrate px/mm "
                  "with two %.2f mm moves inside a +/-%.1f mm box." % (args.cal_step, XY_MAX_MM))
            return 0

        scope = microscope.Microscope.from_config()

        # -- 1. detect the dot where we stand -------------------------------
        f0 = out / "00_start.jpg"
        g0 = scope.grab_frame(f0)
        if not g0.ok:
            raise XYError("start frame: %s" % g0.reason)
        det0 = microscope.marker.detect(f0, mode=args.mode)
        dot0 = _chosen_candidate(det0)
        off0, why0 = microscope.marker.best_offset_px(dot0)
        print("[dot] %d candidate(s); largest at (%.1f, %.1f) px, r~%.0f px, area %.2f%% "
              "of frame, touches_border=%s, quality=%s"
              % (det0["n_candidates"], dot0["centroid_px"][0], dot0["centroid_px"][1],
                 dot0["equiv_radius_px"], dot0["area_frac"] * 100,
                 dot0["touches_frame_border"], dot0["quality"]))
        print("[dot] offset from image centre: %+.1f, %+.1f px via %s\n"
              % (off0[0], off0[1], why0))
        record["steps"].append({"stage": "detect_start", "frame": str(f0),
                                "detection": det0, "chosen": dot0})

        # -- 2. calibrate px <-> mm by two known moves ----------------------
        print("[calib] measuring the pixel/mm mapping with two %.2f mm moves" % args.cal_step)
        base_x, base_y = start_x, start_y

        a.move(base_x + args.cal_step, base_y, z_locked, label="calib +X", takeup=False)
        time.sleep(args.settle)
        fx = out / "01_calib_x.jpg"
        gx = scope.grab_frame(fx)
        if not gx.ok:
            raise XYError("calib +X frame: %s" % gx.reason)
        shift_x = image_shift_px(f0, fx)
        print("      +%.2f mm in X -> image moved (%+.1f, %+.1f) px  (response %.3f)"
              % (args.cal_step, shift_x["dx_px"], shift_x["dy_px"], shift_x["response"]))

        a.move(base_x, base_y + args.cal_step, z_locked, label="calib +Y", takeup=False)
        time.sleep(args.settle)
        fy = out / "02_calib_y.jpg"
        gy_ = scope.grab_frame(fy)
        if not gy_.ok:
            raise XYError("calib +Y frame: %s" % gy_.reason)
        shift_y = image_shift_px(f0, fy)
        print("      +%.2f mm in Y -> image moved (%+.1f, %+.1f) px  (response %.3f)"
              % (args.cal_step, shift_y["dx_px"], shift_y["dy_px"], shift_y["response"]))

        for nm, s in (("X", shift_x), ("Y", shift_y)):
            if s["response"] < 0.02:
                raise XYError("phase correlation for %s is too weak (response %.4f); the "
                              "frames do not share enough structure to trust the mapping"
                              % (nm, s["response"]))
            if (s["dx_px"] ** 2 + s["dy_px"] ** 2) ** 0.5 < 5.0:
                raise XYError("a %.2f mm move in %s shifted the image by less than 5 px; "
                              "either the arm did not move or the scale is far smaller than "
                              "expected -- refusing to invert a degenerate mapping"
                              % (args.cal_step, nm))

        # J maps mm -> px:  [dx_px; dy_px] = J @ [dx_mm; dy_mm].
        # Built by the shared solver so this procedure cannot end up with a
        # laxer singularity threshold than the calibration layer's -- a second
        # copy here previously accepted det down to 1e-6 where the shared code
        # refuses below 1e-3, i.e. it would invert mappings the shared code
        # calls degenerate.
        try:
            J = ps.jacobian_from_moves((shift_x["dx_px"], shift_x["dy_px"]),
                                       (shift_y["dx_px"], shift_y["dy_px"]),
                                       args.cal_step)
        except ps.ScaleError as exc:
            raise XYError(str(exc)) from None
        Jinv = np.linalg.inv(J)
        geom = ps.decompose(J)
        px_per_mm_x = geom["px_per_mm_x"]
        px_per_mm_y = geom["px_per_mm_y"]
        angle = geom["rotation_deg"]
        print("      scale: %.1f px/mm along arm-X, %.1f px/mm along arm-Y; "
              "camera rotated %.1f deg vs arm-X" % (px_per_mm_x, px_per_mm_y, angle))
        fov_mm = det0["width"] / px_per_mm_x if px_per_mm_x else float("nan")
        print("      field of view ~%.2f mm wide (%.0f px)\n" % (fov_mm, det0["width"]))
        record["calibration"] = {
            "cal_step_mm": args.cal_step, "shift_x": shift_x, "shift_y": shift_y,
            "J_mm_to_px": J.tolist(), "px_per_mm_x": px_per_mm_x,
            "px_per_mm_y": px_per_mm_y, "camera_rotation_deg": angle,
            "fov_mm_wide": fov_mm,
        }

        # -- 3. iterate to centre -------------------------------------------
        a.move(base_x, base_y, z_locked, label="back to start XY", takeup=False)
        time.sleep(args.settle)
        cur_x, cur_y = base_x, base_y
        history = []
        for it in range(1, args.iters + 1):
            fi = out / ("10_iter%d.jpg" % it)
            gi = scope.grab_frame(fi)
            if not gi.ok:
                raise XYError("iteration %d frame: %s" % (it, gi.reason))
            det = microscope.marker.detect(fi, mode=args.mode)
            dot = _chosen_candidate(det)
            off, why = microscope.marker.best_offset_px(dot)
            off = np.array(off, dtype=float)
            err = float(np.hypot(*off))
            print("[iter %d] dot at (%.1f, %.1f) px, offset (%+.1f, %+.1f) px = %.1f px via "
                  "%s, touches_border=%s"
                  % (it, dot["centroid_px"][0], dot["centroid_px"][1], off[0], off[1], err,
                     why, dot["touches_frame_border"]))
            history.append({"iter": it, "frame": str(fi), "chosen": dot,
                            "offset_px": off.tolist(), "error_px": err, "metric": why,
                            "pose_xy": [cur_x, cur_y]})
            if err <= args.converge_px and not dot["touches_frame_border"]:
                print("[iter %d] within %.0f px -- converged" % (it, args.converge_px))
                break
            # to move the dot's image by -off, the stage must move by -Jinv @ off
            d_mm = -Jinv @ off
            step = float(np.hypot(*d_mm))
            print("        correction: %+.4f, %+.4f mm (%.3f mm)" % (d_mm[0], d_mm[1], step))
            nx, ny = cur_x + float(d_mm[0]), cur_y + float(d_mm[1])
            if max(abs(nx - anchor[0]), abs(ny - anchor[1])) > XY_MAX_MM:
                raise XYError("the correction would put the plate %+.3f, %+.3f mm from A1, "
                              "outside the +/-%.1f mm box. The dot is further off than this "
                              "routine is allowed to chase -- re-teach A1 by hand."
                              % (nx - anchor[0], ny - anchor[1], XY_MAX_MM))
            a.move(nx, ny, z_locked, label="iter %d correction" % it, takeup=False)
            cur_x, cur_y = nx, ny
            time.sleep(args.settle)
        record["iterations"] = history

        # -- 4. confirmation ring -------------------------------------------
        best = (cur_x, cur_y, None)
        if args.ring > 0:
            print("\n[ring] sampling %.2f mm around the result to confirm it is the best point"
                  % args.ring)
            ring_pts = [(cur_x, cur_y, "centre")]
            for k in range(8):
                th = k * np.pi / 4
                ring_pts.append((cur_x + args.ring * float(np.cos(th)),
                                 cur_y + args.ring * float(np.sin(th)), "ring %d" % (k + 1)))
            results = []
            for j, (rx, ry, lbl) in enumerate(ring_pts):
                if max(abs(rx - anchor[0]), abs(ry - anchor[1])) > XY_MAX_MM:
                    print("      skipping %s: outside the box" % lbl)
                    continue
                a.move(rx, ry, z_locked, label=lbl, takeup=False)
                time.sleep(args.settle)
                fr = out / ("20_ring%02d.jpg" % j)
                gr = scope.grab_frame(fr)
                if not gr.ok:
                    print("      %s: %s -- skipped" % (lbl, gr.reason))
                    continue
                d = microscope.marker.detect(fr, mode=args.mode)
                if not d["candidates"]:
                    print("      %s: no dot -- skipped" % lbl)
                    continue
                c = d["candidates"][0]
                off_c, why_c = microscope.marker.best_offset_px(c)
                e = float(np.hypot(*off_c))
                print("      %-9s offset %6.1f px via %-9s touches_border=%s"
                      % (lbl, e, why_c, c["touches_frame_border"]))
                results.append({"label": lbl, "xy": [rx, ry], "error_px": e,
                                "clipped": c["touches_frame_border"], "frame": str(fr)})
            record["ring"] = results
            usable = [r for r in results if not r["clipped"]] or results
            if usable:
                win = min(usable, key=lambda r: r["error_px"])
                best = (win["xy"][0], win["xy"][1], win["error_px"])
                print("[ring] best: %s at %.1f px" % (win["label"], win["error_px"]))

        a.move(best[0], best[1], z_locked, label="final centred XY", takeup=False)
        final = a.pose()
        record["final_pose"] = final
        record["final_offset_from_a1_mm"] = [round(final[0] - anchor[0], 4),
                                             round(final[1] - anchor[1], 4)]
        print("\n[result] centred at x=%.4f y=%.4f  (A1%+.4f, %+.4f mm)"
              % (final[0], final[1], final[0] - anchor[0], final[1] - anchor[1]))
        status = "centred"

    except BaseException as exc:
        status = "aborted: %s" % (exc if str(exc) else repr(exc))
        print("\n[ABORT] %s" % status, file=sys.stderr, flush=True)
        _restore(a, start_x, start_y, z_locked)
    finally:
        record["status"] = status
        record["entrypoint"] = " ".join([sys.executable, *sys.argv])
        (out / "manifest.json").write_text(json.dumps(record, indent=2, default=str))
        print("[manifest] %s" % (out / "manifest.json"))
        a.close()

    if args.save and status == "centred":
        backup = a.store.path.with_suffix(".json.bak.%s" % stamp)
        shutil.copyfile(a.store.path, backup)
        loc = a.store.workspace.locations["microscope"]
        prev = [loc.pose.x, loc.pose.y]
        loc.pose.x = round(float(record["final_pose"][0]), 6)
        loc.pose.y = round(float(record["final_pose"][1]), 6)
        loc.metadata["xy_centred_at"] = dt.datetime.now().isoformat(timespec="seconds")
        loc.metadata["xy_centred_previous"] = prev
        loc.metadata["xy_centred_manifest"] = str(out / "manifest.json")
        a.store.save()
        print("[saved] A1 xy (%.4f, %.4f) -> (%.4f, %.4f)  backup %s"
              % (prev[0], prev[1], loc.pose.x, loc.pose.y, backup))
    elif args.save:
        print("[saved] NOT saving: status=%s" % status)

    return 0 if status in ("centred", "dry-run") else 1


def main(argv=None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
