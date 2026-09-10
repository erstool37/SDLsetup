#!/usr/bin/env python3
"""teach_location.py -- write the arm's ACTUAL current pose into workspace.json.

    python scripts/tool_building/teach_location.py --show
    python scripts/tool_building/teach_location.py microscope --write
    python scripts/tool_building/teach_location.py floor --write

WHY THE ARM MUST BE JOGGED THERE FIRST
--------------------------------------
``pick_place.py`` moves in JOINT SPACE using taught angles, never from a pose --
there is no Cartesian envelope in joint space, so a joint move computed from an
XYZ triple would be an unbounded 7-DOF traverse with nothing checking it. Joint
angles cannot be typed in; they only exist as a reading off the controller at
the pose you actually want. So a location is taught by putting the arm there and
reading it, never by entering coordinates.

READ-ONLY WITH RESPECT TO THE HARDWARE
--------------------------------------
Connects with ``arm=False``: no motion_enable, no set_mode, no set_state, no
motion. It reads ``get_position`` and ``get_servo_angle`` and nothing else.
``--show`` writes nothing at all.

WHAT A NEW A1 INVALIDATES
-------------------------
The 96-well grid is derived as A1 + col*9.0 + row*9.0, so re-teaching
``microscope`` corrects every well at once -- but it also makes stale:
  * ``scope_standoff``   -- defined as A1 + (0, -150, 0); recomputed here
  * ``routes.json``      -- frozen Cartesian legs against the old anchor; the
                            store is already empty ({}), and this refuses to run
                            if it is not
  * every taught joint angle set for ``microscope`` -- replaced by the reading
Nothing is guessed. A field that cannot be read is written UNKNOWN.
"""
from __future__ import annotations

import argparse
import datetime as _dt
import json
import shutil
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from tools.arm import Arm, ArmSettings, SafetyError  # noqa: E402
from tools.arm.workspace import DEFAULT_STORE  # noqa: E402

Y_STANDOFF_MM = 150.0          # scope_standoff = A1 + (0, -Y_STANDOFF, 0)
#: "uvvis" is the SPECTROstar Nano's plate carrier, taught 2026-08-12 onward.
#: Its tray is NARROW and SHALLOW compared with the microscope station, so the
#: gripper must be opened only slightly to enter it -- see Arm.set_opening and
#: decisions/2026-08-11-bio-gripper-position-mode.md. Unlike "microscope",
#: teaching it recomputes NOTHING: there is no derived standoff for it yet, and
#: inventing one before the approach is surveyed would be a guess.
#: "floor2" is a SECOND grip point on the tray, on the BACK part of the plate
#: (operator, 2026-08-13). It exists because the plate must sit differently in
#: the jaws to fit the UV-Vis carrier: where you grip the plate decides how far
#: it protrudes, and the first grip point does not clear that tray. "uvvis2" is
#: its counterpart at the reader -- the place the arm delivers to when carrying
#: from floor2.
#:
#: floor and floor2 are NOT interchangeable. A route that picks at one and
#: places at the other's destination carries the plate at the wrong offset.
TEACHABLE = ("microscope", "floor", "floor2", "home", "uvvis", "uvvis2")


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description=__doc__.split("WHY")[0].strip(),
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("name", nargs="?", choices=TEACHABLE,
                    help="taught location to overwrite with the current pose")
    ap.add_argument("--show", action="store_true",
                    help="print the live pose and joints; write nothing")
    ap.add_argument("--write", action="store_true",
                    help="actually overwrite the location (a backup is kept)")
    ap.add_argument("--why", default=None,
                    help="one line recorded in the location's description")
    ap.add_argument("--host", default=None)
    return ap


def _read(args) -> tuple[list[float], list[float]]:
    settings = ArmSettings.from_config(str(REPO / "configs" / "config.yaml"),
                                       host=args.host, live=True)
    arm = Arm(settings)                 # arm=False: reading never arms
    pose = [float(v) for v in arm.status()["pose"]]
    joints = [float(a) for a in arm.joints()]
    return pose, joints


def run(args) -> int:
    pose, joints = _read(args)
    print("live pose    x=%.4f y=%.4f z=%.4f  roll=%.4f pitch=%.4f yaw=%.4f"
          % tuple(pose[:6]))
    print("live joints  %s" % ", ".join("%.4f" % a for a in joints))

    if args.show or not args.name:
        print("\n[show] nothing written. Re-run with a name and --write.")
        return 0
    if not args.write:
        print("\n[dry-run] would overwrite %r. Re-run with --write." % args.name)
        return 0

    store_path = Path(DEFAULT_STORE)
    data = json.loads(store_path.read_text())

    routes = data.get("routes") or {}
    if args.name == "microscope" and routes:
        print("\nREFUSING: routes.json-style frozen routes still present in the "
              "store (%d). They were taught against the old anchor and would "
              "drive the arm to the old place. Clear them first." % len(routes))
        return 3

    stamp = _dt.datetime.now().isoformat(timespec="seconds")
    backup = store_path.with_suffix(".json.bak.%s"
                                    % _dt.datetime.now().strftime("%Y%m%d-%H%M%S"))
    shutil.copy2(store_path, backup)

    loc = data["locations"].get(args.name, {})
    previous = loc.get("pose")
    keys = ("x", "y", "z", "roll", "pitch", "yaw")
    data["locations"][args.name] = {
        "name": args.name,
        "pose": dict(zip(keys, [round(v, 6) for v in pose[:6]], strict=True), units="mm_deg"),
        "description": (args.why or
                        "Taught live from the arm's actual pose on %s." % stamp),
        "created_at": stamp,
        "metadata": {
            "joint_angles_deg": [round(a, 6) for a in joints],
            "joint_count": len(joints),
            "captured_at": stamp,
            "source": "xArm live get_position+get_servo_angle (read-only)",
            "previous_pose": previous,
            "backup_of_store": str(backup),
        },
    }

    touched = [args.name]
    if args.name == "microscope":
        a1 = data["locations"]["microscope"]["pose"]
        so = dict(a1)
        so["y"] = round(a1["y"] - Y_STANDOFF_MM, 6)
        data["locations"]["scope_standoff"] = {
            "name": "scope_standoff",
            "pose": so,
            "description": ("Microscope staging pose: A1 displaced %.1f mm along "
                            "-Y at A1's exact Z. DERIVED from A1, not taught."
                            % Y_STANDOFF_MM),
            "created_at": stamp,
            "metadata": {
                "derived_from": "microscope (A1)",
                "derivation": "a1 + (0, -%.1f, 0)" % Y_STANDOFF_MM,
                "y_standoff_mm": Y_STANDOFF_MM,
                "y_approach_sign": -1,
                "joint_angles_deg": "UNKNOWN",
                "joint_angles_note": ("derived pose, never read off the "
                                      "controller; joint-space moves to this "
                                      "location are therefore not available"),
                "ik_verified": "UNKNOWN -- not checked by this script",
                "source": "computed by teach_location.py (no hardware read)",
                "captured_at": stamp,
            },
        }
        touched.append("scope_standoff")
        data["locations"]["microscope"]["metadata"]["invalidates"] = [
            "scope_standoff (recomputed here)",
            "any frozen route taught against the previous A1",
            "pixel_to_arm / well_calibration if the plate or optics moved",
        ]

    store_path.write_text(json.dumps(data, indent=2))
    print("\nwrote %s" % ", ".join(touched))
    print("backup %s" % backup)
    if args.name == "microscope":
        print("scope_standoff recomputed to y=%.4f (A1 - %.1f mm)"
              % (data["locations"]["scope_standoff"]["pose"]["y"], Y_STANDOFF_MM))
        print("NOT checked: IK reachability of the approach legs at this X. The "
              "first transit after this must be a dry run.")
    return 0


def main(argv=None) -> int:
    try:
        return run(build_parser().parse_args(argv))
    except SafetyError as exc:
        print("\nSAFETY: %s" % exc)
        return 6


if __name__ == "__main__":
    raise SystemExit(main())
