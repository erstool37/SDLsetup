#!/usr/bin/env python3
"""lift_plate.py -- raise the tool by N mm at the current XY, in verified steps.

    python3 scripts/tool_building/lift_plate.py 15            # dry-run
    python3 scripts/tool_building/lift_plate.py 15 --execute

WHY IT IS SEPARATE FROM lower_plate.py
--------------------------------------
``lower_plate.py`` guarantees it CANNOT raise -- its envelope ceiling is pinned
to the arm's current pose. That guarantee is load-bearing at the microscope,
where up means "toward the fixed objective", and adding an --up flag would
destroy it. So raising lives here, with its own, smaller budget and its own
reasons.

WHERE UP IS SAFE, AND WHERE IT IS NOT. At the UV-Vis carrier, up is away from a
shallow tray and is the only way to withdraw without dragging the jaws
sideways. At the microscope it is the dangerous direction. This tool therefore
has no default height and a hard cap well below any bench transit: the caller
must say how far, every time.

SAFETY
  G1  hard cap MAX_LIFT_MM on any single invocation, whatever is asked
  G2  the envelope ceiling is the START pose plus the requested lift and no
      more, so overshoot fails validation rather than continuing
  G3  XY and orientation frozen: this is a Z move, and a lateral drift inside
      an instrument is exactly what must not happen
  G4  stepped, with the arm's ACTUAL pose read back and checked every step, so
      a refusal or a stall stops the sequence instead of being averaged away
  G5  dry-run by default
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse  # noqa: E402
import math  # noqa: E402

from tools.arm import Arm, ArmSettings, SafetyError  # noqa: E402
from tools.arm.driver import ArmError  # noqa: E402

#: Largest rise this tool will ever perform in one call. Deliberately far below
#: the 191 mm bench-transit height: this is a withdrawal from an instrument, not
#: a traverse, and a big rise inside a reader housing is the thing to avoid.
MAX_LIFT_MM = 30.0

#: Step size. Small enough that a collision is met gently, large enough to be
#: above the arm's 0.05 mm arrival tolerance by a wide margin.
STEP_MM = 3.0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="raise the tool by N mm at the current XY, in verified steps")
    p.add_argument("lift_mm", type=float, help=f"how far to rise, mm (0..{MAX_LIFT_MM})")
    p.add_argument("--execute", action="store_true")
    p.add_argument("--host")
    p.add_argument("--step-mm", type=float, default=STEP_MM)
    p.add_argument("--speed", type=float, default=3.0, help="mm/s (default 3, slow)")
    return p


def run(args) -> int:
    lift = float(args.lift_mm)
    if not math.isfinite(lift) or lift <= 0 or lift > MAX_LIFT_MM:
        raise SystemExit(f"[lift] lift_mm must be >0 and <= {MAX_LIFT_MM}; got {lift}. "
                         f"This tool withdraws from an instrument; it is not a "
                         f"transit and will not perform one.")
    step = float(args.step_mm)
    if not (0 < step <= lift):
        raise SystemExit(f"[lift] --step-mm must be >0 and <= the lift; got {step}")

    overrides = {"live": args.execute}
    if args.host:
        overrides["host"] = args.host
    settings = ArmSettings.from_config(**overrides)

    if not args.execute:
        print(f"[lift] DRY-RUN: would rise {lift:.2f} mm at the current XY, "
              f"in {math.ceil(lift / step)} step(s) of <= {step} mm.")
        print("[lift] XY and orientation frozen; ceiling is start+lift exactly.")
        return 0

    robot = Arm(settings, log=lambda m: print(m, flush=True))
    print(f"[lift] connecting to {settings.host} ...", flush=True)
    try:
        robot.connection.connect(arm=True)
    except ArmError as exc:
        raise SystemExit(f"[lift] ABORT: {exc}") from exc

    try:
        start = list(robot.pose())
        ceiling = start[2] + lift
        print(f"[lift] at z={start[2]:.4f}; rising to {ceiling:.4f} "
              f"(x={start[0]:.3f} y={start[1]:.3f} held)")

        # G2/G3: ceiling is exactly start+lift; XY and orientation frozen.
        guarded = robot.free_envelope(
            reason=(f"withdraw {lift} mm vertically at fixed XY from an instrument; "
                    f"ceiling pinned to start+{lift} mm so an overshoot fails "
                    f"validation rather than continuing"),
            z_floor_mm=start[2] - 1.0,
            z_ceiling_mm=ceiling,
        )
        n = math.ceil(lift / step)
        for i in range(1, n + 1):
            target_z = min(start[2] + step * i, ceiling)
            res = guarded.move_pose((start[0], start[1], target_z,
                                     start[3], start[4], start[5]),
                                    speed=args.speed,
                                    label=f"lift [{i}/{n} z->{target_z:.3f}]",
                                    takeup=False)
            arrival = res.verify()
            if arrival["moved"] is False:
                raise SystemExit(
                    f"[lift] ABORT at step {i}/{n}: {arrival['reason']}. The tool "
                    f"did not reach the commanded height -- it has met something. "
                    f"Not commanding the next step.")
        after = list(guarded.pose())
        print(f"[lift] done: z={after[2]:.4f} ({after[2] - start[2]:+.3f} mm), "
              f"x={after[0]:.4f} y={after[1]:.4f}")
        return 0
    except (ArmError, SafetyError) as exc:
        raise SystemExit(f"[lift] ABORT: {exc}") from exc
    finally:
        robot.close()
        print("[lift] disconnected.")


def main(argv: list[str] | None = None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
