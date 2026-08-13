#!/usr/bin/env python3
"""lower_plate.py -- lower the plate by N mm at the current XY. It CANNOT raise.

    python3 scripts/tool_building/lower_plate.py 5.0            # dry-run
    python3 scripts/tool_building/lower_plate.py 5.0 --execute

WHY THIS EXISTS
---------------
``goto_rise.py`` is deliberately upward-only, confined to [a1.z, a1.z+10], and
the sweep envelope has ``z_min_rise_mm=0``. Both are correct: on this rig
raising Z drives the plate TOWARD the fixed objective, so upward is the
dangerous direction and it is the one that is fenced.

That left no way to go the other way. Measured 2026-08-11: a full 35-point
sweep over A1+0..+10 found no focal plane at all (prominence 1.292 against a
1.5 floor; a real peak on this rig scores ~50). If focus is not in the 10 mm
above the taught A1, it is most likely BELOW it -- and nothing could search
there, because the anchor's own height was the floor.

Lowering the taught A1 and re-sweeping upward covers that region. This is the
one step that needs a downward move.

SAFETY MODEL
------------
Downward is the safe direction here, and this tool is built so it is the ONLY
direction available:

  G1  The envelope's CEILING is the arm's own current Z, read from the
      controller. A rise is therefore not merely discouraged, it fails
      validation -- there is no argument value that makes this raise the plate.
  G2  ``drop_mm`` must be finite and > 0. A negative "drop" is rejected rather
      than quietly inverted into a rise.
  G3  The floor is the taught tray height minus the standard settling margin,
      so this cannot drive the plate into the bench either.
  G4  Single-axis Z move through the shared guarded path: envelope validate,
      occupancy claim, and post-move readback, same as every other move.
  G5  Dry-run by default.

It does NOT re-teach anything. Teaching stays an explicit separate step
(``teach_location.py``), so lowering and redefining the anchor cannot be
confused for one operation.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse  # noqa: E402

from scripts.tool_building.pick_place import Z_CORRIDOR_MARGIN_MM  # noqa: E402
from tools.arm import Arm, ArmSettings, SafetyError  # noqa: E402
from tools.arm.driver import ArmError  # noqa: E402
from tools.arm.workspace import WorkspaceStore  # noqa: E402

MAX_DROP_MM = 20.0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="lower the plate by N mm at the current XY (never raises)")
    p.add_argument("drop_mm", type=float, help=f"how far to lower, mm (0..{MAX_DROP_MM})")
    p.add_argument("--execute", action="store_true", help="actually move")
    p.add_argument("--host")
    p.add_argument("--speed", type=float, default=3.0,
                   help="mm/s (default 3, deliberately slow)")
    return p


def run(args: argparse.Namespace) -> int:
    drop = float(args.drop_mm)
    if not (drop > 0.0) or drop != drop or drop > MAX_DROP_MM:
        raise SystemExit(f"[lower] drop_mm must be > 0 and <= {MAX_DROP_MM}, got {drop!r}. "
                         f"This tool only lowers; it has no upward mode.")

    overrides = {"live": args.execute}
    if args.host:
        overrides["host"] = args.host
    settings = ArmSettings.from_config(**overrides)
    store = WorkspaceStore(settings.workspace_path)
    tray_z = float(store.require_location("floor").pose.z)

    if not args.execute:
        print(f"[lower] DRY-RUN: would lower {drop:.3f} mm at the current XY.")
        print("[lower] the ceiling is read from the arm, so this cannot raise.")
        return 0

    robot = Arm(settings, log=lambda m: print(m, flush=True))
    print(f"[lower] connecting to {settings.host} ...", flush=True)
    try:
        robot.connection.connect(arm=True)
    except ArmError as exc:
        raise SystemExit(f"[lower] ABORT: {exc}") from exc

    try:
        here = robot.pose()
        target_z = here[2] - drop
        floor_z = tray_z - Z_CORRIDOR_MARGIN_MM
        print(f"[lower] at z={here[2]:.4f}; target z={target_z:.4f}; "
              f"floor {floor_z:.4f} (taught tray {tray_z:.4f})")
        if target_z < floor_z:
            raise SystemExit(
                f"[lower] ABORT: {drop} mm would reach z={target_z:.4f}, below the "
                f"tray floor {floor_z:.4f}.")

        # G1: ceiling IS where the arm is now. A rise cannot validate.
        guarded = robot.free_envelope(
            reason=(f"lower the plate {drop} mm at fixed XY to search for focus below "
                    f"the taught A1; ceiling pinned to the current pose so this "
                    f"cannot raise the plate toward the objective"),
            z_floor_mm=floor_z,
            z_ceiling_mm=here[2],
        )
        legs = guarded.move_axiswise(here[0], here[1], target_z,
                                     speed=args.speed, label="lower", order="zyx")
        after = guarded.pose()
        print(f"[lower] done in {len(legs)} leg(s); now "
              f"x={after[0]:.4f} y={after[1]:.4f} z={after[2]:.4f}")
        print("[lower] NOT taught. Run teach_location.py explicitly if this is the "
              "new anchor.")
    except (ArmError, SafetyError) as exc:
        raise SystemExit(f"[lower] ABORT: {exc}") from exc
    finally:
        robot.close()
        print("[lower] disconnected.")
    return 0


def main(argv: list[str] | None = None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
