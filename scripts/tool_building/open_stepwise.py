#!/usr/bin/env python3
"""open_stepwise.py -- open the BIO gripper a MILLIMETRE AT A TIME, reading back
after every step, against a hard budget.

    python3 scripts/tool_building/open_stepwise.py                 # dry-run
    python3 scripts/tool_building/open_stepwise.py --execute
    python3 scripts/tool_building/open_stepwise.py --execute --budget-mm 5 --step-mm 1

WHY
---
At the UV-Vis station the plate carrier is narrow and shallow, and the operator
reports that opening the jaws wide there will strike the tray. ``release()``
cannot be used: in open/close mode the gripper has exactly two positions and
"open" means the full 150 mm. The only way to open a little is position mode
plus :meth:`Arm.set_opening`, and the only safe way to use that near a tray is
to creep.

THE BUDGET IS A HARD CEILING, NOT A TARGET. It is measured from the jaw
position read at the start, so it bounds TRAVEL, not an absolute span -- 5 mm of
budget from 125 mm can never command past 130 mm no matter what else is passed.
The absolute ``BIO_POS_MAX_MM`` still applies on top.

HOW A COLLISION SHOWS UP. ``set_opening`` reports ``actual_mm`` read back from
the gripper and a ``settled`` flag. Jaws that stop short of their commanded span
have hit something -- the tray, the plate, or a stall. This script treats that
as STOP, immediately, without trying the next step. The alternative (keep
commanding outward against an obstruction) is how a gripper is bent.

That check is the whole point of stepping. A single 5 mm command would report
the same failure only after all 5 mm of travel had already been attempted.

WHAT IT DOES NOT DO. It never closes, never moves the arm, and never decides
whether the plate has been released -- it reports jaw spans and stops. Whether
the plate is seated is for the operator's eye, or a later sensor.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse  # noqa: E402
import math  # noqa: E402
import time  # noqa: E402

from tools.arm import Arm, ArmSettings  # noqa: E402
from tools.arm.api import BIO_POS_MAX_MM, BIO_POS_MIN_MM  # noqa: E402
from tools.arm.driver import BIO_MODE_POSITION, ArmError  # noqa: E402

#: Largest travel this tool will ever perform, whatever is asked. A second
#: ceiling above the operator's budget, so a typo in --budget-mm cannot become a
#: wide open next to a tray.
ABSOLUTE_MAX_TRAVEL_MM = 10.0

#: A step that lands further than this from its command means the jaws stopped
#: on something. Well above the gripper's own resolution, well below one step.
STEP_TOL_MM = 0.5

#: Smallest step this tool will command.
#:
#: MEASURED 2026-08-13, not chosen: the gripper reports its position to 1 mm
#: resolution -- every reading all session has been a whole number (71, 125,
#: 126, 130). A 0.5 mm command therefore cannot be confirmed as reached, so the
#: SDK's wait=True sits until it expires and returns code=100
#: (WAIT_FINISH_TIMEOUT). The jaws DID move; only the confirmation was
#: impossible. A step below the readback resolution is unverifiable by
#: construction, and an unverifiable step next to a tray is the thing this whole
#: tool exists to avoid.
MIN_STEP_MM = 1.0

#: Settling pause between commanding and re-reading.
SETTLE_S = 0.6


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="open the gripper stepwise against a hard travel budget")
    p.add_argument("--execute", action="store_true")
    p.add_argument("--host")
    p.add_argument("--budget-mm", type=float, default=5.0,
                   help="HARD ceiling on total travel from the starting span "
                        "(default 5.0, operator-set for the UV-Vis tray)")
    p.add_argument("--step-mm", type=float, default=1.0)
    p.add_argument("--speed", type=int, default=None,
                   help="gripper speed; default from config")
    p.add_argument("--force", type=int, default=20,
                   help="drive force %%, low by default so a collision pushes "
                        "less hard (default 20)")
    return p


def run(args) -> int:
    budget = float(args.budget_mm)
    step = float(args.step_mm)
    if not (0 < budget <= ABSOLUTE_MAX_TRAVEL_MM):
        raise SystemExit(f"[open] --budget-mm must be >0 and <= "
                         f"{ABSOLUTE_MAX_TRAVEL_MM}; got {budget}")
    if not (0 < step <= budget):
        raise SystemExit(f"[open] --step-mm must be >0 and <= the budget; got {step}")
    if step < MIN_STEP_MM:
        raise SystemExit(
            f"[open] --step-mm {step} is below the gripper's {MIN_STEP_MM} mm "
            f"readback resolution, so the step could not be verified as reached "
            f"(the SDK returns code=100 WAIT_FINISH_TIMEOUT while the jaws move "
            f"anyway). Refusing an unverifiable step next to the tray.")

    overrides = {"live": args.execute, "control_mode": BIO_MODE_POSITION}
    if args.host:
        overrides["host"] = args.host
    settings = ArmSettings.from_config(**overrides)

    print(f"[open] control_mode  {settings.control_mode} (position mode -- required "
          f"for millimetre control)")
    print(f"[open] budget        {budget} mm of TRAVEL, step {step} mm, "
          f"force {args.force}%")
    print(f"[open] jaw limits    {BIO_POS_MIN_MM}-{BIO_POS_MAX_MM} mm")

    if not args.execute:
        print("\n[open] DRY-RUN (no --execute): the gripper was not touched.")
        return 0

    robot = Arm(settings, log=lambda m: print(m, flush=True))
    # arm=False: this is jaw-only. Motion is never enabled, so the arm body
    # cannot move as a side effect of opening the gripper.
    robot.connection.connect(arm=False)
    try:
        start = robot.opening().get("mm")
        if start is None or not math.isfinite(float(start)):
            raise SystemExit("[open] ABORT: jaw position unreadable, so travel "
                             "cannot be bounded. Refusing to open blind.")
        start = float(start)
        ceiling = min(start + budget, float(BIO_POS_MAX_MM))
        print(f"[open] start {start:.1f} mm  ->  hard ceiling {ceiling:.1f} mm")

        target = start
        n = 0
        while target < ceiling - 1e-9:
            n += 1
            target = min(target + step, ceiling)
            res = robot.set_opening(target, speed=args.speed, force=args.force)
            time.sleep(SETTLE_S)
            actual = res.get("actual_mm")
            settled = res.get("settled")
            if actual is None:
                raise SystemExit(f"[open] ABORT after step {n}: jaw position "
                                 f"unreadable. Stopping rather than continuing blind.")
            actual = float(actual)
            gap = abs(actual - target)
            print(f"[open] step {n}: commanded {target:.2f} mm -> actual "
                  f"{actual:.2f} mm  (gap {gap:.2f}, settled={settled})")
            if gap > STEP_TOL_MM:
                print(f"\n[open] STOPPED. The jaws are {gap:.2f} mm short of the "
                      f"{target:.2f} mm commanded, which means they stopped ON "
                      f"SOMETHING -- the tray, the plate, or a stall. Not "
                      f"commanding the next step; pushing outward against an "
                      f"obstruction is how the gripper gets bent.")
                print(f"[open] jaws left at {actual:.2f} mm "
                      f"({actual - start:+.2f} mm from the start).")
                return 2
        final = robot.opening().get("mm")
        print(f"\n[open] reached the {ceiling:.1f} mm ceiling without obstruction; "
              f"jaws now {final} mm ({float(final) - start:+.2f} mm travelled).")
        print("[open] whether the plate is released and seated is for your eye -- "
              "this tool reports jaw spans only.")
        return 0
    except ArmError as exc:
        raise SystemExit(f"[open] ABORT: {exc}") from exc
    finally:
        robot.close()
        print("[open] disconnected.")


def main(argv: list[str] | None = None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
