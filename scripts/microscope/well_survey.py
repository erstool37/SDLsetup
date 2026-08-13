#!/usr/bin/env python3
"""well_survey.py -- drive to nominal wells and photograph them. No centring.

    python3 scripts/microscope/well_survey.py                      # plan only
    python3 scripts/microscope/well_survey.py --execute
    python3 scripts/microscope/well_survey.py --execute --wells A1,A2,B1,A12

WHY
---
Before fitting a plate frame it is worth simply LOOKING at each well. Centring
answers "where exactly is the dot"; it cannot answer "is there a dot here at
all", and on 2026-08-11 that distinction cost several runs: at nominal A2 the
first requested correction was 1.32 mm, past the 1.19 mm half-height of the
field of view, i.e. the detector was locked on the well-wall arc at the frame
edge rather than on any dot. No amount of iterating fixes that.

This does the cheap thing first: go there, take one picture, look.

SAFETY
------
Z and orientation are FROZEN at the taught A1 (``z_max_rise_mm=0``,
``z_min_rise_mm=0``) -- raising Z drives the plate toward the fixed objective,
and nothing here needs to. XY is confined to a box big enough to reach column
12 (99 mm from A1) and no bigger. Moves are axis-sequential per the standing
rule, slow by default, each leg validated and read back.

NOTE the 99 mm traverse to A12 at working height is a path this rig has not run
before. It is a planar move under a fixed objective with Z frozen, but the
gripper travels with the plate, so run it once with eyes on the rig.
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
from tools.microscope import Microscope, MicroscopeSettings  # noqa: E402

FLAT_FIELD_BLUR_PX = 180
FRAME_TIMEOUT_S = 30.0
#: Reach column 12 (11 steps x 9 mm = 99 mm) plus a little slack, and no more.
XY_BOX_MM = 105.0


def parse_well(name: str) -> tuple[int, int]:
    """'A1' -> (row 0, col 0); 'B1' -> (1, 0); 'A12' -> (0, 11)."""
    name = name.strip().upper()
    row = ord(name[0]) - ord("A")
    col = int(name[1:]) - 1
    if not (0 <= row <= 7) or not (0 <= col <= 11):
        raise SystemExit(f"[survey] {name!r} is not a well on a 96-well plate")
    return row, col


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="photograph nominal wells; no centring")
    p.add_argument("--execute", action="store_true")
    p.add_argument("--host")
    p.add_argument("--wells", default="A1,A2,B1,A12")
    p.add_argument("--speed", type=float, default=5.0)
    p.add_argument("--settle", type=float, default=2.0)
    p.add_argument("--frame-ordinal", type=int, default=2)
    p.add_argument("--out-dir", default="/home/lamp/SDLsetup/dataset/captures/well_survey")
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
    pitch = float(cfg.get("well_pitch_mm", 9.0))

    def nominal(row: int, col: int) -> tuple[float, float]:
        x, y = a1[0], a1[1]
        dc = pitch * col * float(amap["col_sign"])
        dr = pitch * row * float(amap["row_sign"])
        if amap["col_axis"] == "x":
            x += dc
        else:
            y += dc
        if amap["row_axis"] == "x":
            x += dr
        else:
            y += dr
        return x, y

    wells = [w for w in (w.strip() for w in args.wells.split(",")) if w]
    plan = [(w, *parse_well(w)) for w in wells]

    print(f"taught A1 : x={a1[0]:.4f} y={a1[1]:.4f} z={a1[2]:.4f} yaw={a1[5]:.4f}")
    print(f"pitch     : {pitch} mm   axis_map {amap}")
    print("plan (Z and orientation FROZEN at the taught A1):")
    for w, r, c in plan:
        nx, ny = nominal(r, c)
        print(f"  {w:<4} row={r} col={c:<2}  x={nx:9.4f} y={ny:9.4f}   "
              f"(A1 {nx - a1[0]:+8.3f}, {ny - a1[1]:+7.3f})")

    if not args.execute:
        print("\n[survey] DRY-RUN (no --execute): nothing moved, nothing captured.")
        return 0

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    robot = Arm(settings, log=lambda m: print(m, flush=True))
    print(f"[survey] connecting to {settings.host} ...", flush=True)
    try:
        robot.connection.connect(arm=True)
    except ArmError as exc:
        raise SystemExit(f"[survey] ABORT: {exc}") from exc

    env = Envelope.anchored(a1, name="A1", xy_max_mm=XY_BOX_MM,
                            z_max_rise_mm=0.0, z_min_rise_mm=0.0,
                            orient_tol_deg=0.5)
    guarded = robot.with_envelope(env)
    scope = Microscope(MicroscopeSettings.from_config())

    import time
    results = []
    try:
        for w, r, c in plan:
            nx, ny = nominal(r, c)
            print(f"\n[survey] === {w} === x={nx:.4f} y={ny:.4f}", flush=True)
            guarded.move_axiswise(nx, ny, a1[2], speed=args.speed,
                                  label=f"to {w}", order="xyz")
            time.sleep(args.settle)
            dest = out_dir / f"{w}.jpg"
            got = scope.grab_frame(dest, ordinal=args.frame_ordinal,
                                   timeout_s=FRAME_TIMEOUT_S)
            if not got.ok:
                print(f"[survey] {w}: frame grab FAILED: {got.reason}")
                results.append((w, None))
                continue
            img = Image.open(dest).convert("L")
            raw = np.asarray(img, dtype=float)
            bg = np.asarray(img.filter(ImageFilter.GaussianBlur(radius=FLAT_FIELD_BLUR_PX)),
                            dtype=float)
            flat = np.clip(raw / np.maximum(bg, 1.0) * 128.0, 0, 255)
            Image.fromarray(flat.astype(np.uint8)).save(out_dir / f"{w}_flat.png")
            dark = float((flat < 90).mean())
            print(f"[survey] {w}: mean={raw.mean():.1f} dark_frac={dark:.3f} -> {dest}")
            results.append((w, dark))
        print("\n=== SURVEY ===")
        for w, d in results:
            print(f"  {w:<4} {'no frame' if d is None else 'dark fraction %.3f' % d}")
        print(f"\n  frames + flat-fielded PNGs in {out_dir}")
    except (ArmError, SafetyError) as exc:
        raise SystemExit(f"[survey] ABORT: {exc}") from exc
    finally:
        robot.close()
        print("[survey] disconnected.")
    return 0


def main(argv: list[str] | None = None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
