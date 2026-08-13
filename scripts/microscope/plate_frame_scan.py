#!/usr/bin/env python3
"""plate_frame_scan.py -- centre the dots at A1, A2 and B1, then solve the plate frame.

    python3 scripts/microscope/plate_frame_scan.py             # plan only
    python3 scripts/microscope/plate_frame_scan.py --execute

WHY
---
Every well is derived as A1 plus an integer number of 9.0 mm steps, and that
arithmetic assumes the grid is square to the arm's X/Y axes. The plate is seated
by hand, so it is not. ``tools/calib/plate_frame.py`` quantifies the cost: 1 deg
of seating error puts a well 117 mm away off by 2.0 mm, most of a well. A1->A12
is 99 mm, so the same 1 deg costs about 1.7 mm there -- the difference between
imaging the dot and imaging the wall.

Three dots are printed for exactly this: BLACK at A1, BLUE at A2 (one COLUMN
step), RED at B1 (one ROW step). Two measured vectors of known true length,
along the plate's own axes, recover the whole 2x2 grid-to-arm map.

WHY THREE AND NOT TWO. The existing ``centering.calibrate_plate`` uses A1 and
A12 -- two points on ONE axis -- and ``tools/arm/geometry.py`` records the known
limitation in its own docstring: two collinear points cannot separate a rotation
from a row-axis error. B1 is the non-collinear third point that closes it.

FLAT-FIELDING, and why it is not optional here
----------------------------------------------
Measured 2026-08-11 on this rig: only 31.7% of the frame is above the dark
threshold. The illumination is a bright central disc with dark corners, so the
dot and the unlit surround are ONE connected component and no threshold
separates them. ``find_dark_dot`` on the raw frame returns either a 6284 px
speck or the whole 4.34e6 px surround, and three ROI sizes disagreed by over
1600 px -- that is not a detection, it is whichever speck the crop included.

Dividing by a heavily blurred copy of the frame removes the illumination profile
and leaves the dot dark *relative to its local surround*. After it, full-frame
and 60%-ROI detection agree: a 1.40 mm disc, offset ~(-100, +50) px. The
correction lives in the CameraPort adapter, so ``center_on_dot`` is untouched
and still sees a plain array.

The dot is HAND-DRAWN (operator, 2026-08-11), so circle-fit quality flags are
meaningless here by construction and centring is centroid-based.

SAFETY
------
One anchored envelope for the whole scan, built on the taught A1: XY confined to
a box that admits the 9 mm well steps plus centring slack, Z FROZEN at the
taught height (z_max_rise_mm=0), orientation frozen. Every pose is validated and
read back through the shared guarded ``Arm``. Dry-run by default.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse  # noqa: E402
import json  # noqa: E402

import numpy as np  # noqa: E402
from PIL import Image, ImageFilter  # noqa: E402

from tools.arm import Arm, ArmSettings, SafetyError  # noqa: E402
from tools.arm.driver import ArmError  # noqa: E402
from tools.arm.safety import Envelope  # noqa: E402
from tools.arm.workspace import WorkspaceStore  # noqa: E402
from tools.calib import centering  # noqa: E402
from tools.calib import pixel_scale as ps  # noqa: E402
from tools.calib import plate_frame as pf  # noqa: E402
from tools.microscope import Microscope, MicroscopeSettings  # noqa: E402

#: Blur radius, px, used to estimate the illumination field. Must be well above
#: the dot radius (measured 600 px) so the dot is not blurred into its own
#: background and erased. 180 px was the value verified on 2026-08-11.
FLAT_FIELD_BLUR_PX = 180

#: XY half-box around A1. Must admit a 9 mm well step plus centring corrections;
#: deliberately not larger, so a runaway correction cannot walk off the plate.
XY_BOX_MM = 14.0

#: Fraction of the frame kept when looking for the dot. Everything outside is
#: flattened to the frame median so it cannot be detected.
#:
#: Measured 2026-08-11 at A2: the well wall throws a large dark ARC against the
#: frame edge that is comparable in area to the dot. Detection flipped between
#: the two, and each correction pushed the real dot further out -- across six
#: iterations it walked 2.02 mm in Y and ended half off the left edge, while
#: the reported residual kept falling because it was measuring the arc. A1 never
#: showed this: its dot sat near centre and converged in one step.
#:
#: 0.62 keeps a region comfortably larger than the 1.4 mm dot at 851 px/mm while
#: excluding the edge arcs.
#: DEFAULT 1.0 = OFF. Measured 2026-08-11: 0.62 made A1 WORSE, not better --
#: residual went from 0.0099 mm (converged in one iteration) to 0.2266 mm (did
#: not converge in eight). The arithmetic says why: the dot is 1.4 mm ~ 1200 px
#: across and a 0.62 keep-region is only 1270 px tall, so the mask was clipping
#: the dot itself and biasing its centroid. Any keep-region large enough to hold
#: the dot plus its off-centre excursion is the whole frame height.
#: Kept as a knob for a smaller dot; it is not usable for this one.
CENTRE_KEEP_FRAC = 1.0

FRAME_TIMEOUT_S = 30.0


class _FlatFieldCamera:
    """CameraPort: newest published frame, illumination-corrected.

    Reports pixels. It makes no decision about what the dot is or where the arm
    should go -- that stays with center_on_dot and this script.
    """

    def __init__(self, scope: Microscope, out_dir: Path, ordinal: int, tag: str = "f",
                 keep_frac: float = CENTRE_KEEP_FRAC):
        self.scope, self.out_dir, self.ordinal = scope, out_dir, ordinal
        self.tag, self.n = tag, 0
        self.keep_frac = float(keep_frac)

    def frame(self) -> np.ndarray:
        self.n += 1
        dest = self.out_dir / f"{self.tag}_{self.n:03d}.jpg"
        got = self.scope.grab_frame(dest, ordinal=self.ordinal, timeout_s=FRAME_TIMEOUT_S)
        if not got.ok:
            raise SystemExit(f"[scan] ABORT: frame grab failed: {got.reason}")
        img = Image.open(dest).convert("L")
        raw = np.asarray(img, dtype=float)
        bg = np.asarray(img.filter(ImageFilter.GaussianBlur(radius=FLAT_FIELD_BLUR_PX)),
                        dtype=float)
        flat = np.clip(raw / np.maximum(bg, 1.0) * 128.0, 0, 255)
        # Border suppression. Dark structures against the frame edge (well wall
        # arcs) are not candidates -- the dot we are centring is by definition
        # the one near the middle, and a competing edge blob makes the loop walk
        # the real dot out of frame instead of into the centre.
        if self.keep_frac < 1.0:
            h, w = flat.shape
            keep = np.zeros_like(flat, dtype=bool)
            y0 = int(h * (1 - self.keep_frac) / 2)
            x0 = int(w * (1 - self.keep_frac) / 2)
            keep[y0:h - y0, x0:w - x0] = True
            flat[~keep] = float(np.median(flat[keep]))
        return flat.astype(np.uint8)


class _GuardedArm:
    """ArmPort backed by the shared guarded Arm. Every move is envelope-checked.

    Z, roll, pitch and yaw are PINNED to the taught anchor rather than echoed
    back from the pose that was just read. Centring is an XY operation; letting
    it re-command a measured Z means feeding settling noise back in as a
    command. The envelope freezes Z to exactly the taught height, so a pose read
    0.7 um high -- which is what the arm actually reports -- fails validation
    and aborts the scan. Measured 2026-08-11: z 191.2095 against a window of
    [191.2088, 191.2088].

    Pinning is the stronger fix than widening the window: it keeps Z frozen at
    zero budget (raising Z drives the plate toward the objective) while making
    the commanded value exactly the one the envelope permits.
    """

    def __init__(self, arm: Arm, anchor: tuple):
        self.arm = arm
        self.anchor = anchor

    def get_pose(self) -> list:
        return list(self.arm.pose())

    def move_cart(self, x, y, z, roll, pitch, yaw, speed) -> None:
        a = self.anchor
        self.arm.move_pose((x, y, a[2], a[3], a[4], a[5]), speed=speed,
                           label="centre", takeup=False)


class _Transform:
    """transform.apply(offset_px) -> (dx_mm, dy_mm), from the measured Jacobian."""

    def __init__(self, J):
        self.J = J

    def apply(self, offset_px):
        return ps.correction_mm(self.J, offset_px)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="centre A1/A2/B1 and solve the plate frame")
    p.add_argument("--execute", action="store_true")
    p.add_argument("--host")
    p.add_argument("--speed", type=float, default=5.0, help="mm/s (default 5)")
    p.add_argument("--settle", type=float, default=1.5)
    p.add_argument("--frame-ordinal", type=int, default=2)
    p.add_argument("--tolerance-mm", type=float, default=0.02)
    p.add_argument("--max-iterations", type=int, default=4)
    p.add_argument("--centre-keep-frac", type=float, default=CENTRE_KEEP_FRAC,
                   help="keep only this central fraction of the frame when "
                        "detecting (1.0 = off, the default). Values below ~0.9 "
                        "clip this rig's 1.4 mm dot and bias its centroid.")
    p.add_argument("--max-correction-mm", type=float, default=1.2,
                   help="cumulative XY travel centring may use at one well "
                        "(default 1.2). The dot should be within a fraction of a "
                        "mm of nominal; walking further means it is chasing "
                        "something else, and 2.02 mm of walk is exactly how the "
                        "2026-08-11 A2 attempt ended up on a well-wall arc.")
    p.add_argument("--scale-json", default="/home/lamp/.sdl_lab/robot_arm/pixel_scale.json")
    p.add_argument("--out-dir", default=None)
    p.add_argument("--store", default="/home/lamp/.sdl_lab/robot_arm/plate_frame.json")
    return p


def run(args) -> int:
    overrides = {"live": args.execute}
    if args.host:
        overrides["host"] = args.host
    settings = ArmSettings.from_config(**overrides)
    store = WorkspaceStore(settings.workspace_path)
    a1p = store.require_location("microscope").pose
    a1 = (float(a1p.x), float(a1p.y), float(a1p.z),
          float(a1p.roll), float(a1p.pitch), float(a1p.yaw))

    cfg = json.load(open("/home/lamp/.sdl_lab/robot_arm/plate_config.json"))
    amap = cfg["axis_map"]
    pitch = float(cfg.get("well_pitch_mm", pf.PITCH_MM))

    def nominal(col_steps: int, row_steps: int) -> tuple[float, float]:
        x, y = a1[0], a1[1]
        dc = pitch * col_steps * float(amap["col_sign"])
        dr = pitch * row_steps * float(amap["row_sign"])
        if amap["col_axis"] == "x":
            x += dc
        else:
            y += dc
        if amap["row_axis"] == "x":
            x += dr
        else:
            y += dr
        return x, y

    targets = [("A1  black", 0, 0), ("A2  blue ", 1, 0), ("B1  red  ", 0, 1)]
    print(f"taught A1   : x={a1[0]:.4f} y={a1[1]:.4f} z={a1[2]:.4f} yaw={a1[5]:.4f}")
    print(f"pitch       : {pitch} mm   axis_map {amap}")
    for name, c, r in targets:
        nx, ny = nominal(c, r)
        print(f"  {name} nominal  x={nx:.4f} y={ny:.4f}")

    model = ps.load(Path(args.scale_json))
    J = ps.jacobian_at(model, a1[2])
    d = ps.decompose(J)
    print(f"jacobian    : {d['px_per_mm_x']:.1f}/{d['px_per_mm_y']:.1f} px/mm, "
          f"rot {d['rotation_deg']:.2f} deg, reversing={d['orientation_reversing']}")

    if not args.execute:
        print("\n[scan] DRY-RUN (no --execute): nothing moved, nothing captured.")
        return 0

    out_dir = Path(args.out_dir or (Path("dataset/captures/plate_frame")
                                    / "run"))
    out_dir.mkdir(parents=True, exist_ok=True)

    robot = Arm(settings, log=lambda m: print(m, flush=True))
    print(f"[scan] connecting to {settings.host} ...", flush=True)
    try:
        robot.connection.connect(arm=True)
    except ArmError as exc:
        raise SystemExit(f"[scan] ABORT: {exc}") from exc

    env = Envelope.anchored(a1, name="A1", xy_max_mm=XY_BOX_MM,
                            z_max_rise_mm=0.0, z_min_rise_mm=0.0,
                            orient_tol_deg=0.5)
    guarded = robot.with_envelope(env)
    scope = Microscope(MicroscopeSettings.from_config())
    transform = _Transform(J)

    measured: dict[str, tuple[float, float]] = {}
    try:
        for name, c, r in targets:
            nx, ny = nominal(c, r)
            print(f"\n[scan] === {name} === driving to nominal x={nx:.4f} y={ny:.4f}")
            guarded.move_pose((nx, ny, a1[2], a1[3], a1[4], a1[5]),
                              speed=args.speed, label=f"goto {name.strip()}",
                              takeup=False)
            cam = _FlatFieldCamera(scope, out_dir, args.frame_ordinal,
                                   tag=name.split()[0], keep_frac=args.centre_keep_frac)
            res = centering.center_on_dot(
                _GuardedArm(guarded, a1), cam, transform,
                tolerance_mm=args.tolerance_mm,
                max_iterations=args.max_iterations,
                max_correction_mm=args.max_correction_mm,
                speed=args.speed)
            pose = list(guarded.pose())
            print(f"[scan] {name} converged={res.converged} "
                  f"residual={res.residual_mm:.4f} mm  reason={res.aborted_reason}")
            print(f"[scan] {name} centred at x={pose[0]:.4f} y={pose[1]:.4f}")
            if not res.converged:
                raise SystemExit(
                    f"[scan] ABORT at {name}: centring did not converge "
                    f"({res.aborted_reason}). Refusing to solve a frame from a dot "
                    f"that is not centred -- the error would propagate into every "
                    f"well position.")
            measured[name.split()[0]] = (pose[0], pose[1])

        frame = pf.solve_plate_frame(
            p_black=measured["A1"], p_red=measured["B1"], p_blue=measured["A2"],
            pitch_mm=pitch, axis_map=amap)

        print("\n=== PLATE FRAME ===")
        print(f"  rotation_deg          {frame.rotation_deg:+.4f}   "
              f"(seating error; 1 deg ~ 1.7 mm at A12)")
        print(f"  scale col / row       {frame.scale_col:.4f} / {frame.scale_row:.4f} "
              f"(1.0 = nominal {pitch} mm)")
        print(f"  non_orthogonality_deg {frame.non_orthogonality_deg:+.4f}")
        print(f"  residual_mm           {frame.residual_mm:.4f}")
        print(f"  quality               {frame.quality}   {frame.reason or ''}")
        a12 = pf.well_offset(frame, row=0, col=11)
        print(f"\n  A12 = A1 + ({a12[0]:+.4f}, {a12[1]:+.4f}) mm  ->  "
              f"x={a1[0]+a12[0]:.4f} y={a1[1]+a12[1]:.4f}")
        nx12, ny12 = nominal(11, 0)
        print(f"  nominal A12 would be   x={nx12:.4f} y={ny12:.4f}   "
              f"(difference {((a1[0]+a12[0]-nx12)**2 + (a1[1]+a12[1]-ny12)**2)**0.5:.4f} mm)")

        Path(args.store).write_text(json.dumps({
            "measured_poses": measured,
            "rotation_deg": frame.rotation_deg,
            "scale_col": frame.scale_col, "scale_row": frame.scale_row,
            "non_orthogonality_deg": frame.non_orthogonality_deg,
            "residual_mm": frame.residual_mm,
            "quality": frame.quality, "reason": frame.reason,
            "matrix": [list(r) for r in np.asarray(frame.matrix).tolist()],
            "taught_a1": list(a1), "pitch_mm": pitch,
            "jacobian_decomposition": d,
            "note": ("dots are HAND-DRAWN; centring is centroid-based and circle-fit "
                     "quality flags do not apply. Frames flat-fielded "
                     f"(gaussian r={FLAT_FIELD_BLUR_PX} px) because only ~32% of the "
                     "raw frame is lit and the dot merges with the vignette."),
        }, indent=1))
        print(f"\n  stored: {args.store}")
    except (ArmError, SafetyError) as exc:
        raise SystemExit(f"[scan] ABORT: {exc}") from exc
    finally:
        robot.close()
        print("[scan] disconnected.")
    return 0


def main(argv: list[str] | None = None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
