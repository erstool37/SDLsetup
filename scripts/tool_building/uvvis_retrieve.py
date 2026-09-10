#!/usr/bin/env python3
"""uvvis_retrieve.py -- take the plate back out of the SPECTROstar Nano carrier.

    python3 scripts/tool_building/uvvis_retrieve.py            # dry-run
    python3 scripts/tool_building/uvvis_retrieve.py --execute

THE RULE THIS STATION HAS
-------------------------
**grip() and release() are never used here.** In open/close mode the BIO gripper
has exactly two positions: 71 mm and 150 mm. "Close" means all the way shut and
"open" means all the way open, and the operator's standing constraint is that a
wide open at this carrier strikes the tray.

On 2026-08-13 grip() was called here anyway. The SDK sent pos=71, the jaws were
observed at 146 mm mid-motion, the gripper latched END_EFFECTOR_HAS_FAULT
(code=102, gripper error 12), and the close finished at 71 mm holding nothing.
Every safe jaw movement at this station has instead gone through
Arm.set_opening() in POSITION mode, one millimetre at a time, read back.

So this procedure only ever commands spans, and only in position mode.

THE SEQUENCE
  1. verify the arm is on the taught uvvis2 XY -- a descent at the wrong XY
     does not land on the carrier
  2. lift clear, stepped and verified
  3. open to the approach span -- ONLY once clear of the tray, never down in it
  4. descend onto uvvis2, stepped and verified
  5. close to the PLATE span by stepped set_opening, and check where the jaws
     stopped: stopping at the plate's thickness means the plate is held;
     running on to the 71 mm minimum means it is not, and nothing is lifted
  6. lift the plate clear

WHY STEP 5 IS THE INTERESTING ONE. A close that reaches the jaw minimum has
caught nothing. That is the same check that caught an empty grip at the tray on
2026-08-11, expressed in position mode: the jaws should STOP EARLY, on the
plate. Stopping early is success here; arriving exactly is failure.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse  # noqa: E402
import math  # noqa: E402
import time  # noqa: E402

from tools.arm import Arm, ArmSettings, SafetyError  # noqa: E402
from tools.arm.api import BIO_POS_MIN_MM  # noqa: E402
from tools.arm.driver import BIO_MODE_POSITION, ArmError  # noqa: E402
from tools.arm.workspace import WorkspaceStore  # noqa: E402

#: Span the jaws take before descending around the plate. 130 mm was measured
#: safe at this carrier on 2026-08-13 (5 mm of travel from the 125 mm grip,
#: every step landing with gap 0.00 and no obstruction).
APPROACH_SPAN_MM = 130.0

#: Span a held plate reads. Measured all session: a good grip is 125.0 mm.
PLATE_SPAN_MM = 125.0

#: How far above the plate span the jaws may stop and still count as holding it.
#: Generous, because the plate need not be centred in the jaws.
PLATE_SPAN_TOL_MM = 3.0

#: 1 mm is the gripper's position READBACK resolution -- measured, not chosen.
#: A smaller step cannot be confirmed as reached, so the SDK's wait expires with
#: code=100 (WAIT_FINISH_TIMEOUT) while the jaws move anyway.
JAW_STEP_MM = 1.0

LIFT_MM = 60.0
Z_STEP_MM = 3.0
SPEED_MM_S = 3.0
SETTLE_S = 0.6
XY_TOL_MM = 0.5


def _jaw_to(robot, target, *, force, log):
    """Walk the jaws to `target` in 1 mm steps. Returns the final span."""
    cur = float(robot.opening().get("mm"))
    n = max(1, math.ceil(abs(target - cur) / JAW_STEP_MM))
    step = (target - cur) / n
    for i in range(1, n + 1):
        want = cur + step * i
        res = robot.set_opening(round(want, 1), force=force)
        time.sleep(SETTLE_S)
        actual = res.get("actual_mm")
        if actual is None:
            raise SystemExit("[uv] ABORT: jaw position unreadable mid-move.")
        log(f"[uv]   jaw {i}/{n}: commanded {want:.1f} -> actual {float(actual):.1f}")
    return float(robot.opening().get("mm"))


def _z_to(robot, guarded, start, target_z, *, log, tag):
    n = max(1, math.ceil(abs(target_z - start[2]) / Z_STEP_MM))
    for i in range(1, n + 1):
        z = start[2] + (target_z - start[2]) * i / n
        res = guarded.move_pose((start[0], start[1], z, start[3], start[4], start[5]),
                                speed=SPEED_MM_S, label=f"{tag} [{i}/{n}]", takeup=False)
        if res.verify()["moved"] is False:
            raise SystemExit(f"[uv] ABORT during {tag} at step {i}/{n}: the tool did "
                             f"not reach {z:.3f}; it has met something.")
    log(f"[uv] {tag} done: z={guarded.pose()[2]:.4f}")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="retrieve the plate from the UV-Vis carrier")
    p.add_argument("--execute", action="store_true")
    p.add_argument("--host")
    p.add_argument("--force", type=int, default=20)
    p.add_argument("--lift-mm", type=float, default=LIFT_MM)
    return p


def run(args) -> int:
    settings = ArmSettings.from_config(
        **({"live": args.execute, "control_mode": BIO_MODE_POSITION}
           | ({"host": args.host} if args.host else {})))
    ws = WorkspaceStore(settings.workspace_path)
    t = ws.require_location("uvvis2").pose
    anchor = (float(t.x), float(t.y), float(t.z))

    print(f"[uv] uvvis2      x={anchor[0]:.4f} y={anchor[1]:.4f} z={anchor[2]:.4f}")
    print(f"[uv] plan        lift {args.lift_mm} mm -> open to {APPROACH_SPAN_MM} mm "
          f"-> descend -> close toward {PLATE_SPAN_MM} mm -> lift")
    print("[uv] jaw control set_opening in position mode only; "
          "grip()/release() are NOT used at this station")
    if not args.execute:
        print("\n[uv] DRY-RUN (no --execute): nothing moved, no jaw command sent.")
        return 0

    robot = Arm(settings, log=lambda m: print(m, flush=True))
    robot.connection.connect(arm=True)
    log = print
    try:
        start = list(robot.pose())
        off = math.hypot(start[0] - anchor[0], start[1] - anchor[1])
        if off > XY_TOL_MM:
            raise SystemExit(f"[uv] ABORT: {off:.3f} mm off the taught uvvis2 XY. "
                             f"This procedure only operates on that column.")
        top_z = anchor[2] + args.lift_mm
        guarded = robot.free_envelope(
            reason=("UV-Vis retrieval at fixed XY: floor pinned to the taught uvvis2 "
                    "height, ceiling to that plus the lift, so the tool can neither "
                    "drive below the operator-set position nor wander upward"),
            z_floor_mm=anchor[2], z_ceiling_mm=top_z)

        log(f"[uv] 1/5 lifting to z={top_z:.3f}")
        _z_to(robot, guarded, list(guarded.pose()), top_z, log=log, tag="lift")

        log(f"[uv] 2/5 opening to {APPROACH_SPAN_MM} mm (clear of the tray)")
        span = _jaw_to(robot, APPROACH_SPAN_MM, force=args.force, log=log)
        log(f"[uv]     jaws at {span:.1f} mm")

        log(f"[uv] 3/5 descending to z={anchor[2]:.3f}")
        _z_to(robot, guarded, list(guarded.pose()), anchor[2], log=log, tag="descend")

        log(f"[uv] 4/5 closing toward {PLATE_SPAN_MM} mm")
        span = _jaw_to(robot, PLATE_SPAN_MM, force=args.force, log=log)
        log(f"[uv]     jaws stopped at {span:.1f} mm")
        if span <= BIO_POS_MIN_MM + PLATE_SPAN_TOL_MM:
            raise SystemExit(
                f"[uv] ABORT: the jaws reached {span:.1f} mm, at/near their "
                f"{BIO_POS_MIN_MM} mm minimum -- NOTHING IS HELD. Not lifting. "
                f"The plate is still in the carrier.")
        if abs(span - PLATE_SPAN_MM) > PLATE_SPAN_TOL_MM:
            raise SystemExit(
                f"[uv] ABORT: jaws stopped at {span:.1f} mm, more than "
                f"{PLATE_SPAN_TOL_MM} mm from the {PLATE_SPAN_MM} mm a plate reads. "
                f"Whatever is between the jaws is not the plate as we know it.")
        log(f"[uv]     holding: {span:.1f} mm is within {PLATE_SPAN_TOL_MM} mm of "
            f"the {PLATE_SPAN_MM} mm plate span")

        log(f"[uv] 5/5 lifting the plate to z={top_z:.3f}")
        _z_to(robot, guarded, list(guarded.pose()), top_z, log=log, tag="lift-out")

        final = list(guarded.pose())
        log(f"\n[uv] DONE. Plate held at {robot.opening().get('mm')} mm, "
            f"clear of the carrier at z={final[2]:.4f}.")
        return 0
    except (ArmError, SafetyError) as exc:
        raise SystemExit(f"[uv] ABORT: {exc}") from exc
    finally:
        robot.close()
        print("[uv] disconnected.")


def main(argv: list[str] | None = None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
