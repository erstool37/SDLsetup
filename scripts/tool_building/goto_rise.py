#!/usr/bin/env python3
"""goto_rise.py -- move to a given height above well A1, and nothing else.

    python3 scripts/goto_rise.py <rise_mm> [--execute]

Reuses the same guarded path every procedure on this rig moves through:
tools.arm.Arm, with a Z-only envelope built the same way
focus_sweep.py builds its sweep envelope -- Z confined to
[a1.z, a1.z + Z_MAX_RISE_MM], X/Y/orientation frozen to A1 (checked on both the
commanded target and the READ-BACK pose), dry-run by default, and any failure
lowers Z back toward the taught A1 height.

Before the 2026-08 consolidation this file imported Arm/check_readback/load_a1
straight out of focus_sweep.py via a sys.path hack keyed to one hardcoded
checkout path (``/home/lamp/SDLsetup/scripts``), which only worked by cwd
accident. It now imports nothing from focus_sweep.py; both files call the same
shared tools.arm.Arm instead.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


import argparse
import sys

from tools.arm import Arm, Envelope
from tools.arm.safety import READBACK_SLACK_DEG, READBACK_SLACK_MM, Z_MAX_RISE_MM

#: Hardcoded in the pre-consolidation script (never a CLI flag); pinned here so
#: this file's numeric behaviour does not drift from what it always was.
ARM_HOST_DEFAULT = "192.168.1.201"
SPEED_MM_S = 3.0

ANCHOR = "microscope"


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("rise_mm", type=float,
                    help="height above A1, mm (0..%.1f)" % Z_MAX_RISE_MM)
    ap.add_argument("--execute", action="store_true",
                    help="connect to the arm and move (default: dry-run)")
    return ap


def run(args: argparse.Namespace) -> int:
    rise = args.rise_mm
    if not (0.0 <= rise <= Z_MAX_RISE_MM):
        raise SystemExit("rise must be within [0, %.1f] mm above A1" % Z_MAX_RISE_MM)

    arm = Arm.from_config(live=args.execute, host=ARM_HOST_DEFAULT, speed_mm_s=SPEED_MM_S,
                          clear_errors=False, anchor=ANCHOR)
    a1 = arm.location_pose(ANCHOR)
    # Same guard formula as focus_sweep.build_sweep_envelope(): Z confined to
    # the full [a1.z, a1.z + Z_MAX_RISE_MM] budget, XY/orientation frozen to
    # A1 on the target, POSITION_TOL_MM/DEG slack allowed on readback only.
    envelope = Envelope.anchored(
        a1, name="A1", z_max_rise_mm=Z_MAX_RISE_MM, xy_max_mm=0.0, orient_tol_deg=0.0,
        readback_slack_mm=READBACK_SLACK_MM, readback_slack_deg=READBACK_SLACK_DEG)
    arm = arm.with_envelope(envelope)

    target = a1[2] + rise
    print("A1 z      : %.4f" % a1[2])
    print("target    : %.4f  (A1%+.3f mm)" % (target, rise))
    if not args.execute:
        print("DRY-RUN — pass --execute to move")
        arm.close()
        return 0

    status = "incomplete"
    try:
        actual = arm.pose()
        print("readback  : x=%.3f y=%.3f z=%.3f (A1%+.3f mm)"
              % (actual[0], actual[1], actual[2], actual[2] - a1[2]))
        arm.envelope.check_readback(actual, where="before goto")
        arm.move(a1[0], a1[1], target, label="goto A1%+.3f mm" % rise)
        info = arm.connection.status()
        final = info["pose"]
        print("arrived   : z=%.4f  (A1%+.4f mm)  error=%s"
              % (final[2], final[2] - a1[2], info.get("error_code")))
        status = "arrived"
    except BaseException as exc:
        print("ABORT: %r" % (exc,), file=sys.stderr)
        arm.retreat()
        status = "aborted: %s" % (exc if str(exc) else repr(exc))
    finally:
        arm.close()

    return 0 if status == "arrived" else 1


def main(argv=None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
