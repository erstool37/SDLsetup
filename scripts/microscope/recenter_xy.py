#!/usr/bin/env python3
"""recenter_xy.py -- put the arm back on A1's XY column without changing Z.

    python3 scripts/microscope/recenter_xy.py             # plan only
    python3 scripts/microscope/recenter_xy.py --execute

WHY THIS EXISTS
---------------
A search leaves the arm wherever it stopped -- a few millimetres off A1 in XY,
at whatever height it was scanning at. Nothing else can pick up from there:

* ``goto_rise.py`` validates readback against an envelope with ``xy_max_mm=0``
  and 0.5 mm of slack, so it refuses at anything past half a millimetre off A1.
  That refusal is correct for what that script does -- it is a Z-only tool and
  must not be handed an arm that has wandered laterally -- but it means it
  cannot be the thing that recovers from a wander either.
* ``goto_a1.py`` would work, but only by running the whole standoff-and-push-in
  route: a metre of travel to fix a 5 mm offset.

So this does the one small guarded move that closes that gap.

SAFETY
------
The envelope is anchored on the pose the arm is **already at**, with the Z
budget pinned shut (``z_min_rise_mm = z_max_rise_mm = 0``), so the permitted Z
window is a single value and no pose this builds can change the height. That
matters more than usual here: the tray now sits at the taught A1 height, which
is the objective's focal plane, and Z is not this routine's to touch.

The XY box is sized to the move being asked for and nothing more, and a move
longer than ``MAX_RECOVER_MM`` is refused outright -- at that distance the arm
is not "slightly off A1", something else is wrong, and a confident lateral move
across an unknown gap is the wrong response to not knowing where you are.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


import argparse  # noqa: E402
import math  # noqa: E402

from tools.arm import Arm  # noqa: E402
from tools.arm.driver import ArmError, XArmConnection  # noqa: E402
from tools.arm.safety import Envelope, SafetyError  # noqa: E402

ANCHOR = "microscope"
ARM_HOST_DEFAULT = "192.168.1.201"

#: Longest lateral recovery this will make. One well pitch: a larger offset is
#: not a wander, and this routine has no way to know what it would cross.
MAX_RECOVER_MM = 9.0

#: Slow. This is a recovery move over ground the caller did not choose.
SPEED_MM_S = 5.0


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--execute", action="store_true",
                    help="connect and move (default: dry-run, commands nothing)")
    ap.add_argument("--host", default=ARM_HOST_DEFAULT)
    ap.add_argument("--speed", type=float, default=SPEED_MM_S)
    return ap


def run(args: argparse.Namespace) -> int:
    arm = Arm.from_config(live=args.execute, host=args.host, clear_errors=False,
                          anchor=ANCHOR, speed_mm_s=args.speed)
    a1 = arm.location_pose(ANCHOR)

    if args.execute:
        here = arm.pose()
    else:
        probe = XArmConnection(args.host, live=True, clear_errors=False,
                               log=lambda _m: None)
        try:
            here = probe.read_pose()
        except ArmError as exc:
            raise SystemExit("cannot read the arm's pose: %s" % exc) from None
        finally:
            probe.disconnect()

    dx, dy = a1[0] - here[0], a1[1] - here[1]
    dist = math.hypot(dx, dy)
    print("taught A1  x=%9.4f y=%9.4f z=%9.4f" % (a1[0], a1[1], a1[2]))
    print("arm is at  x=%9.4f y=%9.4f z=%9.4f   (A1 %+.3f mm in z)"
          % (here[0], here[1], here[2], here[2] - a1[2]))
    print("move       dx=%+.4f dy=%+.4f  -> %.4f mm, Z held at %.4f"
          % (dx, dy, dist, here[2]))

    if dist > MAX_RECOVER_MM:
        raise SystemExit(
            "REFUSED: %.3f mm is further than %.1f mm from A1's column. That is not a "
            "wander off a search, and this routine cannot know what a move that long "
            "would cross. Use goto_a1.py, which has a validated route."
            % (dist, MAX_RECOVER_MM))
    if dist < 1e-4:
        print("already on A1's column; nothing to do")
        return 0

    envelope = Envelope.anchored(
        here, name="recover origin", z_min_rise_mm=0.0, z_max_rise_mm=0.0,
        xy_max_mm=dist + 0.5, orient_tol_deg=0.5, speed_max_mm_s=max(args.speed, 10.0))
    moving = arm.with_envelope(envelope)
    print("envelope   %s" % envelope.describe())

    if not args.execute:
        envelope.check_target([a1[0], a1[1], here[2], here[3], here[4], here[5]],
                              what="recovery target")
        print()
        print("DRY-RUN: target validated, nothing commanded. Add --execute to move.")
        return 0

    try:
        with moving.occupied("recenter_xy"):
            moving.move(a1[0], a1[1], here[2], speed=args.speed,
                        label="back to A1 XY", takeup=False)
    except (SafetyError, ArmError) as exc:
        arm.connection.halt()
        print("\nREFUSED/FAILED: %s" % exc, file=sys.stderr)
        return 2

    final = arm.pose()
    print("arrived    x=%9.4f y=%9.4f z=%9.4f" % (final[0], final[1], final[2]))
    print("residual   dx=%+.4f dy=%+.4f dz=%+.4f"
          % (final[0] - a1[0], final[1] - a1[1], final[2] - here[2]))
    return 0


def main(argv=None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
