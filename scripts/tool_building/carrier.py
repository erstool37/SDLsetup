#!/usr/bin/env python3
"""carrier.py -- move the reader's plate carrier, gated on a MEASURED arm clearance.

    python3 scripts/tool_building/carrier.py out            # plan only
    python3 scripts/tool_building/carrier.py out --execute
    python3 scripts/tool_building/carrier.py in  --execute
    python3 scripts/tool_building/carrier.py status

WHY THIS IS ITS OWN STEP
------------------------
The carrier slides OUT into the volume the arm swings through. Whoever moves it
must first establish that the arm is not there -- and "establish" has to mean
measured, not declared.

Three gates, in increasing order of how much they actually prove:

  1  DECLARED   ``interlock.arm_clear`` -- a flag someone set. Kept because the
                reader layer wants it, but it proves nothing on its own.
  2  OBSERVED   ``occupancy.require_free("uv_vis")`` -- reads the live claim
                registry, so an arm moving in ANY process refuses carrier
                motion even if no flag was updated. Already enforced inside
                SpectroStarNano._require_clear.
  3  MEASURED   this module: read the arm's ACTUAL pose from the controller and
                require it to be above the carrier in Z and off the uvvis2
                column in XY.

Gate 3 is what the operator asked for: the arm signals clearance by BEING
somewhere provably clear, not by asserting it. A flag can be stale; a pose read
back from the controller a moment ago cannot.

WHAT THE CARRIER ITSELF CANNOT TELL US
--------------------------------------
Its position. ``CarrierState`` records what was commanded because the control
software's OUT status strings never marshal back, and PlateOut/PlateIn share
the run-log marker "(Plate command)", so the log proves a carrier move LANDED
but not which way it went. That is reported honestly here rather than dressed
up: ``direction_verified`` is always False.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402

from tools.arm import Arm, ArmSettings  # noqa: E402
from tools.arm.workspace import WorkspaceStore  # noqa: E402

#: The arm must be at least this high to count as clear of the carrier.
#: uvvis2 sits at z=59.78; the transit height proven on that column is 120.
CLEAR_Z_MM = 110.0

#: ...and at least this far off the uvvis2 column in XY. One plate half-width
#: (~64 mm) plus margin, so a plate held at the boundary is still outside.
CLEAR_XY_MM = 80.0


def measure_clearance(host: str | None = None) -> dict:
    """Read the arm's ACTUAL pose and judge whether it is clear of the carrier.

    Reports; decides nothing. Connects read-only -- ``arm=False`` never enables
    motion, so asking whether the arm is clear cannot itself move it.
    """
    settings = ArmSettings.from_config(**({"live": True}
                                          | ({"host": host} if host else {})))
    ws = WorkspaceStore(settings.workspace_path)
    uv = ws.require_location("uvvis2").pose
    arm = Arm(settings)
    arm.connection.connect(arm=False)
    try:
        st = arm.connection.status()
        pose = st.get("pose")
        if pose is None:
            return {"clear": False, "reason": "arm pose unreadable -- unknown is "
                                              "not clear", "pose": None}
        x, y, z = float(pose[0]), float(pose[1]), float(pose[2])
        if not all(math.isfinite(v) for v in (x, y, z)):
            return {"clear": False, "reason": f"non-finite pose {pose[:3]}",
                    "pose": pose[:3]}
        xy_off = math.hypot(x - float(uv.x), y - float(uv.y))
        high_enough = z >= CLEAR_Z_MM
        far_enough = xy_off >= CLEAR_XY_MM
        clear = high_enough or far_enough
        why = []
        if high_enough:
            why.append(f"z {z:.1f} >= {CLEAR_Z_MM}")
        if far_enough:
            why.append(f"xy {xy_off:.1f} mm off the uvvis2 column >= {CLEAR_XY_MM}")
        if not clear:
            why.append(f"z {z:.1f} < {CLEAR_Z_MM} AND only {xy_off:.1f} mm off the "
                       f"column -- the arm is over the carrier")
        return {"clear": clear, "reason": "; ".join(why),
                "pose": [round(x, 3), round(y, 3), round(z, 3)],
                "xy_off_uvvis2_mm": round(xy_off, 3),
                "jaws_mm": arm.opening().get("mm")}
    finally:
        arm.close()


def move(direction: str, *, execute: bool, host: str | None = None) -> dict:
    from tools import uvvis
    from tools.uvvis import dde

    if direction not in ("in", "out"):
        raise SystemExit(f"[c] direction must be 'in' or 'out', got {direction!r}")

    clearance = measure_clearance(host)
    print(f"[c] arm clearance: {'CLEAR' if clearance['clear'] else 'NOT CLEAR'} "
          f"-- {clearance['reason']}")
    print(f"[c]   pose {clearance.get('pose')}  jaws {clearance.get('jaws_mm')} mm")
    if not clearance["clear"]:
        raise SystemExit(
            "[c] REFUSED: the arm is not measurably clear of the carrier. Moving "
            "it now would drive the tray into the arm. Lift the arm above "
            f"z={CLEAR_Z_MM} or take it off the uvvis2 column first.")

    if not execute:
        print(f"[c] DRY-RUN: would move the carrier {direction.upper()}.")
        return {"executed": False, "clearance": clearance}

    if not uvvis.control_running():
        print("[c] starting the control software ...")
        uvvis.ensure_control_running()

    before = dde.log_count("(Plate command)")
    (dde.plate_out if direction == "out" else dde.plate_in)()
    after = dde.log_count("(Plate command)")
    print(f"[c] '(Plate command)' {before} -> {after}")
    if after <= before:
        raise SystemExit("[c] ABORT: the run log shows no new '(Plate command)' -- "
                         "the command did not reach the instrument.")
    print(f"[c] carrier commanded {direction.upper()}. Direction is NOT verified: "
          f"PlateOut and PlateIn share this marker and the carrier reports no "
          f"position back.")
    return {"executed": True, "direction": direction,
            "command_landed": True, "direction_verified": False,
            "clearance": clearance}


def build_parser():
    p = argparse.ArgumentParser(description="move the reader carrier, gated on a "
                                            "measured arm clearance")
    p.add_argument("action", choices=("in", "out", "status"))
    p.add_argument("--execute", action="store_true")
    p.add_argument("--host")
    return p


def run(args) -> int:
    if args.action == "status":
        print(json.dumps(measure_clearance(args.host), indent=1))
        return 0
    print(json.dumps(move(args.action, execute=args.execute, host=args.host),
                     indent=1))
    return 0


def main(argv=None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
