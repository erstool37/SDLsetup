#!/usr/bin/env python3
"""column_calibration.py -- measure the plate's true COLUMN axis from A1 and A12.

    python3 scripts/microscope/column_calibration.py             # plan only
    python3 scripts/microscope/column_calibration.py --execute

WHY THIS AND NOT THE THREE-DOT FRAME
------------------------------------
``tools/calib/plate_frame.py`` solves the full 2x2 grid map from three dots --
BLACK at A1, BLUE at A2, RED at B1. Surveyed on this plate 2026-08-11, those
dots are not there: A2 shows a plain well ring and B1 a bubble-filled well, with
no mark in either. Centring at A2 therefore locked onto the well-wall arc at the
frame edge (first requested correction 1.32 mm, past the 1.19 mm half-height of
the field of view) and walked the arm 2 mm across six iterations.

What this plate DOES carry is a black dot at A1 and another at A12 (operator,
confirmed by survey). Those two wells are 11 column steps and 99 mm apart -- the
longest baseline available, and the one where a seating rotation shows up most.

WHAT IT MEASURES, AND WHAT IT CANNOT
------------------------------------
Two points on ONE axis give the column step vector: its direction (the seating
rotation) and its length (the true pitch). That is everything needed to reach
any well in ROW A, which is what the A1..A12 imaging run needs.

It CANNOT separate a rotation from a row-axis error -- ``tools/arm/geometry.py``
records that limitation in its own docstring, and it is exactly why the
three-dot version exists. So the row axis here stays NOMINAL and is written to
the store marked as such. Do not read the row direction out of this file as if
it were measured; it was not. A red or blue dot on a row-neighbour well is what
would close it.

SAFETY
------
Z and orientation FROZEN at the taught A1 (rise budget 0 both ways). XY confined
to a box just large enough to reach column 12. Axis-sequential moves, every leg
validated and read back. Dry-run by default.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402

from tools.arm import Arm, ArmSettings, SafetyError  # noqa: E402
from tools.arm.driver import ArmError  # noqa: E402
from tools.arm.safety import Envelope  # noqa: E402
from tools.arm.workspace import WorkspaceStore  # noqa: E402
from tools.calib import adapters, centering  # noqa: E402
from tools.calib import pixel_scale as ps  # noqa: E402
from tools.microscope import Microscope, MicroscopeSettings  # noqa: E402

XY_BOX_MM = 105.0
COL_STEPS_A1_TO_A12 = 11

#: Sanity gates on the measured column vector. A measurement outside these is
#: reported and REFUSED, not stored -- a wrong grid map silently mis-addresses
#: all 96 wells, which is far worse than no map.
MAX_ROTATION_DEG = 15.0
MAX_PITCH_DEVIATION = 0.05          # +/-5% on the 9.0 mm nominal pitch





def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="measure the plate column axis from A1 and A12")
    p.add_argument("--execute", action="store_true")
    p.add_argument("--host")
    p.add_argument("--speed", type=float, default=5.0)
    p.add_argument("--tolerance-mm", type=float, default=0.05)
    p.add_argument("--max-iterations", type=int, default=8)
    p.add_argument("--max-correction-mm", type=float, default=1.6)
    p.add_argument("--scale-json", default="/home/lamp/.sdl_lab/robot_arm/pixel_scale.json")
    p.add_argument("--out-dir", default="/home/lamp/SDLsetup/dataset/captures/column_cal")
    p.add_argument("--store", default="/home/lamp/.sdl_lab/robot_arm/well_calibration.json")
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
    pitch_nom = float(cfg.get("well_pitch_mm", 9.0))

    def nominal_col(col: int) -> tuple[float, float]:
        x, y = a1[0], a1[1]
        d = pitch_nom * col * float(amap["col_sign"])
        if amap["col_axis"] == "x":
            x += d
        else:
            y += d
        return x, y

    n12 = nominal_col(COL_STEPS_A1_TO_A12)
    print(f"taught A1   : x={a1[0]:.4f} y={a1[1]:.4f} z={a1[2]:.4f}")
    print(f"nominal A12 : x={n12[0]:.4f} y={n12[1]:.4f}   "
          f"({COL_STEPS_A1_TO_A12} steps x {pitch_nom} mm)")

    model = ps.load(Path(args.scale_json))
    J = ps.jacobian_at(model, a1[2])
    d = ps.decompose(J)
    print(f"jacobian    : {d['px_per_mm_x']:.1f}/{d['px_per_mm_y']:.1f} px/mm, "
          f"rot {d['rotation_deg']:.2f} deg, reversing={d['orientation_reversing']}")

    if not args.execute:
        print("\n[cal] DRY-RUN (no --execute): nothing moved.")
        return 0

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    robot = Arm(settings, log=lambda m: print(m, flush=True))
    print(f"[cal] connecting to {settings.host} ...", flush=True)
    try:
        robot.connection.connect(arm=True)
    except ArmError as exc:
        raise SystemExit(f"[cal] ABORT: {exc}") from exc

    env = Envelope.anchored(a1, name="A1", xy_max_mm=XY_BOX_MM,
                            z_max_rise_mm=0.0, z_min_rise_mm=0.0, orient_tol_deg=0.5)
    guarded = robot.with_envelope(env)
    scope = Microscope(MicroscopeSettings.from_config())
    transform = adapters.JacobianTransform(J)

    def centre_at(tag: str, tx: float, ty: float) -> tuple[float, float]:
        print(f"\n[cal] === {tag} === driving to x={tx:.4f} y={ty:.4f}", flush=True)
        adapters.axis_sequential_to(guarded, a1, tx, ty, speed=args.speed,
                                    label=f"to {tag}")
        cam = adapters.FlatFieldCamera(scope, out_dir, ordinal=2, tag=tag)
        res = centering.center_on_dot(
            adapters.PinnedArm(guarded, a1), cam, transform,
            tolerance_mm=args.tolerance_mm, max_iterations=args.max_iterations,
            max_correction_mm=args.max_correction_mm, speed=args.speed)
        pose = list(guarded.pose())
        print(f"[cal] {tag}: converged={res.converged} residual={res.residual_mm:.4f} mm "
              f"reason={res.aborted_reason}")
        print(f"[cal] {tag}: centred x={pose[0]:.4f} y={pose[1]:.4f}")
        if not res.converged:
            raise SystemExit(
                f"[cal] ABORT at {tag}: centring did not converge "
                f"({res.aborted_reason}). A column axis fitted to an uncentred dot "
                f"mis-addresses every well in the row.")
        return pose[0], pose[1]

    try:
        p1 = centre_at("A1", a1[0], a1[1])
        p12 = centre_at("A12", n12[0], n12[1])

        vx, vy = p12[0] - p1[0], p12[1] - p1[1]
        step = (vx / COL_STEPS_A1_TO_A12, vy / COL_STEPS_A1_TO_A12)
        pitch = math.hypot(*step)

        nx, ny = nominal_col(1)
        nstep = (nx - a1[0], ny - a1[1])
        rot = math.degrees(math.atan2(step[1], step[0]) - math.atan2(nstep[1], nstep[0]))
        rot = (rot + 180.0) % 360.0 - 180.0
        dev = pitch / pitch_nom - 1.0

        print("\n=== COLUMN AXIS ===")
        print(f"  A1  centred            x={p1[0]:.4f} y={p1[1]:.4f}")
        print(f"  A12 centred            x={p12[0]:.4f} y={p12[1]:.4f}")
        print(f"  measured column step   ({step[0]:+.4f}, {step[1]:+.4f}) mm")
        print(f"  measured pitch         {pitch:.4f} mm  ({dev * 100:+.2f}% vs {pitch_nom})")
        print(f"  seating rotation       {rot:+.4f} deg")
        print(f"  nominal A12 error      "
              f"{math.hypot(p12[0] - n12[0], p12[1] - n12[1]):.4f} mm "
              f"(how far the old arithmetic was off at A12)")

        bad = []
        if abs(rot) > MAX_ROTATION_DEG:
            bad.append(f"rotation {rot:+.3f} deg exceeds {MAX_ROTATION_DEG}")
        if abs(dev) > MAX_PITCH_DEVIATION:
            bad.append(f"pitch deviates {dev * 100:+.1f}% (limit "
                       f"{MAX_PITCH_DEVIATION * 100:.0f}%)")
        if bad:
            raise SystemExit("[cal] REFUSED, not stored: " + "; ".join(bad) +
                             ". A wrong grid map mis-addresses all 96 wells silently.")

        Path(args.store).write_text(json.dumps({
            "taught_a1": list(a1),
            "a1_centred": list(p1), "a12_centred": list(p12),
            "col_step_mm": list(step), "col_pitch_mm": pitch,
            "col_rotation_deg": rot, "col_pitch_deviation": dev,
            "col_steps_measured_over": COL_STEPS_A1_TO_A12,
            "row_axis": "NOMINAL -- NOT MEASURED",
            "row_step_mm": [0.0, pitch_nom] if amap["row_axis"] == "y"
                           else [pitch_nom, 0.0],
            "jacobian_decomposition": d,
            "method": ("two dots, A1 and A12, 11 column steps apart. Measures the "
                       "COLUMN axis only; two collinear points cannot separate a "
                       "rotation from a row-axis error, so the row axis above is "
                       "nominal and unmeasured. A2/B1 carry no dots on this plate "
                       "(surveyed 2026-08-11)."),
        }, indent=1))
        print(f"\n  stored: {args.store}")
        print("  row axis is NOMINAL and marked as such -- do not read it as measured.")
    except (ArmError, SafetyError) as exc:
        raise SystemExit(f"[cal] ABORT: {exc}") from exc
    finally:
        robot.close()
        print("[cal] disconnected.")
    return 0


def main(argv: list[str] | None = None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
