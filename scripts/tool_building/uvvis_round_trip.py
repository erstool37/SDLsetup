#!/usr/bin/env python3
"""uvvis_round_trip.py -- fetch the plate from the UV-Vis carrier and put it in the tray.

    python3 scripts/tool_building/uvvis_round_trip.py             # plan only
    python3 scripts/tool_building/uvvis_round_trip.py --execute

ROUTE (operator-specified, 2026-08-13)

    at home   rotate wrist 0 -> 90 deg          (operator: safe at this position)
    lift      z -> TRANSIT_Z                     (operator: "up is mostly safe")
    traverse  Y then X, at TRANSIT_Z, to the uvvis2 column
    open      jaws -> APPROACH_SPAN, ONLY up here, never down in the tray
    descend   z -> taught uvvis2
    close     jaws -> PLATE_SPAN, and CHECK the jaws stopped ON the plate
    lift      z -> TRANSIT_Z, plate held
    traverse  Y back over open bench
    rotate    wrist 90 -> 0 deg, clear of the reader
    traverse  X then Y to the floor2 column
    descend   z -> taught floor2
    release   jaws -> RELEASE_SPAN, stepped

THE STANDING RULE AT THE UV-VIS: grip() and release() are NEVER used. In
open/close mode the gripper has two positions, 71 and 150 mm, and a wide open at
this carrier strikes the tray. Called there on 2026-08-13 it latched
END_EFFECTOR_HAS_FAULT (code=102, gripper error 12) with the jaws seen at 146 mm
mid-motion. Every jaw movement here is a 1 mm set_opening in POSITION mode,
read back. 1 mm is the gripper's readback resolution, measured -- a smaller step
cannot be confirmed and the SDK's wait expires with code=100.

WHY THE ROTATIONS ARE WHERE THEY ARE. The wrist turn out of the reader happens
AFTER the Y traverse has carried the plate back over open bench, not at the
uvvis2 column: rotating a 128 x 86 mm plate sweeps a ~77 mm radius, and doing
that beside an instrument that has already caused two error-31 collisions is not
a risk worth taking for a few seconds. The turn INTO 90 deg is at home, which
the operator confirmed is safe.

WHY TRANSIT_Z IS 120. Both error-31 collisions were at z = 77.9 and 81.9. The
arm has been driven to z = 119.87 on the uvvis2 column this session with no
incident, so 120 mm is both measured-clear on the column that matters and ~38 mm
above the known obstruction band. It is NOT the 191 mm bench-transit height:
nothing has ever surveyed 191 mm above the reader.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse  # noqa: E402
import math  # noqa: E402
import time  # noqa: E402

from tools.arm import Arm, ArmSettings, SafetyError  # noqa: E402
from tools.arm.driver import BIO_MODE_POSITION, ArmError  # noqa: E402
from tools.arm.workspace import WorkspaceStore  # noqa: E402

TRANSIT_Z = 120.0          # see module docstring
APPROACH_SPAN_MM = 130.0   # measured safe at this carrier, 2026-08-13
PLATE_SPAN_MM = 125.0      # a held plate has read 125.0 all session
RELEASE_SPAN_MM = 135.0    # enough to free the plate in the tray

#: Span COMMANDED when grasping -- deliberately NARROWER than the plate.
#:
#: Commanding the plate's own 125 mm is not a grasp test: free jaws arrive at
#: 125 too, so "did it stop on the plate" and "did it close on air" are the same
#: reading. Commanding 120 makes them different -- a plate STALLS the jaws wide
#: at ~125, empty jaws reach 120 and keep going toward the 71 mm minimum.
GRASP_COMMAND_MM = 120.0

#: The jaws must end up at least this far WIDER than what was commanded for
#: something to be considered held. Anything less and they closed on air.
GRASP_STALL_MIN_MM = 3.0

PLATE_SPAN_TOL_MM = 3.0

#: Grasping is ONE fast, strong command, not a 1 mm creep.
#:
#: Operator, 2026-08-13: stepping inward at 20% force does not clamp the plate
#: hard enough to carry it. Stepping exists to protect the TRAY while opening
#: outward; closing inward moves AWAY from the tray walls, so it needs neither
#: the steps nor the low force.
GRASP_FORCE_PCT = 90
GRASP_SPEED = 3000
JAW_STEP_MM = 1.0          # the gripper's readback resolution. Measured.
Z_STEP_MM = 3.0
XY_STEP_MM = 25.0
SPEED_MM_S = 5.0
FINE_SPEED_MM_S = 3.0
SETTLE_S = 0.6
XY_TOL_MM = 0.5
#: Open-bench Y at which the wrist turn happens: clear of the reader, clear of
#: the tray, and the column the arm already parks on at home.
ROTATE_Y = 0.424


def _log(msg):
    print(msg, flush=True)


def _jaw_to(robot, target, *, force):
    cur = float(robot.opening().get("mm"))
    n = max(1, math.ceil(abs(target - cur) / JAW_STEP_MM))
    for i in range(1, n + 1):
        want = cur + (target - cur) * i / n
        res = robot.set_opening(round(want, 1), force=force)
        time.sleep(SETTLE_S)
        actual = res.get("actual_mm")
        # Unknown is NOT success. `nan <= x` and `abs(nan - x) > y` are both
        # False, so a non-finite reading would sail through every later check --
        # the exact bug grip() already carries a guard for.
        if actual is None or not math.isfinite(float(actual)):
            raise SystemExit(f"[rt] ABORT: jaw position read back as {actual!r} "
                             f"mid-move. Unknown is not holding.")
    got = float(robot.opening().get("mm"))
    _log(f"[rt]     jaws {cur:.0f} -> {got:.0f} mm (wanted {target:.0f})")
    return got


def _axis_to(arm, x=None, y=None, z=None, *, speed, tag):
    """One axis at a time; the others are held at the arm's ACTUAL values."""
    for axis, want in (("z", z), ("y", y), ("x", x)):
        if want is None:
            continue
        here = list(arm.pose())
        idx = {"x": 0, "y": 1, "z": 2}[axis]
        span = want - here[idx]
        if abs(span) < 0.05:
            continue
        n = max(1, math.ceil(abs(span) / (Z_STEP_MM if axis == "z" else XY_STEP_MM)))
        for i in range(1, n + 1):
            tgt = list(here)
            tgt[idx] = here[idx] + span * i / n
            res = arm.move_pose(tuple(tgt), speed=speed,
                                label=f"{tag} {axis} [{i}/{n}]", takeup=False)
            if res.verify()["moved"] is False:
                raise SystemExit(f"[rt] ABORT: {tag} {axis} step {i}/{n} did not "
                                 f"arrive -- the tool met something.")
        _log(f"[rt]     {tag}: {axis} -> {arm.pose()[idx]:.3f}")


def build_parser():
    p = argparse.ArgumentParser(description="UV-Vis fetch and return to floor2")
    p.add_argument("--execute", action="store_true")
    p.add_argument("--host")
    p.add_argument("--force", type=int, default=20)
    p.add_argument("--transit-z", type=float, default=TRANSIT_Z)
    return p


def run(args) -> int:
    settings = ArmSettings.from_config(
        **({"live": args.execute, "control_mode": BIO_MODE_POSITION}
           | ({"host": args.host} if args.host else {})))
    ws = WorkspaceStore(settings.workspace_path)
    uv = ws.require_location("uvvis2").pose
    fl = ws.require_location("floor2").pose
    uv_xyz = (float(uv.x), float(uv.y), float(uv.z))
    fl_xyz = (float(fl.x), float(fl.y), float(fl.z))
    uv_yaw, fl_yaw = float(uv.yaw), float(fl.yaw)
    tz = float(args.transit_z)
    # Deriving the ceiling from tz means no tz can ever be refused -- including
    # 80, which is inside the 77.9-81.9 collision band. Bound it independently.
    if not 100.0 <= tz <= 125.0:
        raise SystemExit(
            f"[rt] --transit-z {tz} is outside the surveyed 100-125 mm band. "
            f"Below 100 approaches the 77.9-81.9 mm collision band; above 125 "
            f"nothing has ever been surveyed over the reader.")

    print(f"[rt] uvvis2 {uv_xyz} yaw {uv_yaw:.3f}")
    print(f"[rt] floor2 {fl_xyz} yaw {fl_yaw:.3f}")
    print(f"[rt] transit z {tz}   approach span {APPROACH_SPAN_MM}   "
          f"plate span {PLATE_SPAN_MM}")
    print("[rt] jaw control: set_opening, position mode, 1 mm steps. "
          "grip()/release() NOT used.")
    if not args.execute:
        print("\n[rt] DRY-RUN: nothing moved, no jaw command sent.")
        return 0

    robot = Arm(settings, log=_log)
    robot.connection.connect(arm=True)
    try:
        start = list(robot.pose())
        # A free envelope constrains Z ONLY (safety.py gates the XY and
        # orientation branches on `anchor is not None`), so without this the
        # rotation would sweep a 77 mm radius from ANY pose in the corridor.
        home = ws.require_location("home").pose
        d_home = math.dist((start[0], start[1], start[2]),
                           (float(home.x), float(home.y), float(home.z)))
        if d_home > 80.0:
            raise SystemExit(
                f"[rt] ABORT: the arm is {d_home:.1f} mm from the taught home "
                f"(x={start[0]:.1f} y={start[1]:.1f} z={start[2]:.1f}). This route "
                f"starts from home; it will not begin from an unknown pose.")
        floor_z = min(fl_xyz[2], uv_xyz[2]) - 0.5
        arm = robot.free_envelope(
            reason=("UV-Vis round trip: fetch the plate from the carrier and return "
                    "it to the tray. Z corridor spans the two taught heights and the "
                    "operator-approved transit height, nothing more."),
            z_floor_mm=floor_z, z_ceiling_mm=tz + 0.5)

        # LIFT BEFORE ROTATING. Measured 2026-08-13: rotating at `home` failed on
        # the first 15 deg step -- a commanded pure YAW turn dragged ROLL 0.152 deg
        # off (tolerance 0.050). home is the all-joints-near-zero configuration,
        # where the wrist axes align and yaw couples into roll. The tray->scope
        # rotations that passed this same check earlier were done at z=191 with
        # the arm extended. Lifting first moves it off that ill-conditioned pose.
        #
        # The guard is NOT loosened. Roll drift during a rotation is exactly what
        # the Codex review of 2026-08-12 added the six-axis check for: an
        # unnoticed orientation drift makes the NEXT step a compound move.
        _log(f"[rt] 1/11 lift to z={tz} (BEFORE rotating -- home is near-singular)")
        _axis_to(arm, z=tz, speed=SPEED_MM_S, tag="lift")

        _log(f"[rt] 2/11 rotate wrist -> {uv_yaw:.3f} deg, off the singular pose")
        here0 = list(arm.pose())
        arm.rotate_in_place(here0[3], here0[4], uv_yaw, speed=FINE_SPEED_MM_S,
                            label="to uvvis yaw")

        _log("[rt] 3/11 traverse to the uvvis2 column (Y then X)")
        _axis_to(arm, y=uv_xyz[1], speed=SPEED_MM_S, tag="to-uv")
        _axis_to(arm, x=uv_xyz[0], speed=SPEED_MM_S, tag="to-uv")
        off = math.hypot(arm.pose()[0] - uv_xyz[0], arm.pose()[1] - uv_xyz[1])
        if off > XY_TOL_MM:
            raise SystemExit(f"[rt] ABORT: {off:.3f} mm off the uvvis2 column.")

        _log(f"[rt] 4/11 open to {APPROACH_SPAN_MM} mm (clear of the tray)")
        _jaw_to(robot, APPROACH_SPAN_MM, force=args.force)

        _log(f"[rt] 5/11 descend to z={uv_xyz[2]:.3f}")
        _axis_to(arm, z=uv_xyz[2], speed=FINE_SPEED_MM_S, tag="descend")

        _log(f"[rt] 6/11 grasp: ONE command to {GRASP_COMMAND_MM} mm at "
             f"{GRASP_FORCE_PCT}% force (narrower than the plate, so the plate "
             f"stalls the jaws)")
        robot.set_opening(GRASP_COMMAND_MM, speed=GRASP_SPEED, force=GRASP_FORCE_PCT)
        time.sleep(SETTLE_S * 2)
        raw = robot.opening().get("mm")
        if raw is None or not math.isfinite(float(raw)):
            raise SystemExit(f"[rt] ABORT: jaw span read back as {raw!r}. "
                             f"Unknown is not holding; not lifting.")
        span = float(raw)
        stall = span - GRASP_COMMAND_MM
        _log(f"[rt]     commanded {GRASP_COMMAND_MM:.0f}, jaws at {span:.0f} "
             f"(stalled {stall:+.0f} mm wide)")
        if stall < GRASP_STALL_MIN_MM:
            raise SystemExit(
                f"[rt] ABORT: the jaws reached {span:.0f} mm against a "
                f"{GRASP_COMMAND_MM:.0f} mm command -- they did NOT stall on "
                f"anything, so NOTHING IS HELD. Not lifting; the plate is still "
                f"in the carrier.")
        if abs(span - PLATE_SPAN_MM) > PLATE_SPAN_TOL_MM:
            raise SystemExit(
                f"[rt] ABORT: the jaws stalled at {span:.0f} mm, more than "
                f"{PLATE_SPAN_TOL_MM} mm from the {PLATE_SPAN_MM} mm a plate "
                f"reads. Something is between the jaws, but it is not the plate "
                f"as we know it.")
        _log(f"[rt]     HOLDING: stalled on the plate at {span:.0f} mm")

        _log(f"[rt] 7/11 lift the plate to z={tz}")
        _axis_to(arm, z=tz, speed=FINE_SPEED_MM_S, tag="lift-out")

        _log(f"[rt] 8/11 traverse Y to open bench (y={ROTATE_Y})")
        _axis_to(arm, y=ROTATE_Y, speed=SPEED_MM_S, tag="to-bench")

        _log(f"[rt] 9/11 rotate wrist -> {fl_yaw:.3f} deg, clear of the reader")
        here = list(arm.pose())
        arm.rotate_in_place(here[3], here[4], fl_yaw, speed=FINE_SPEED_MM_S,
                            label="to floor yaw")

        _log("[rt] 10/11 traverse to the floor2 column (X then Y), then descend")
        _axis_to(arm, x=fl_xyz[0], speed=SPEED_MM_S, tag="to-floor")
        _axis_to(arm, y=fl_xyz[1], speed=SPEED_MM_S, tag="to-floor")
        _axis_to(arm, z=fl_xyz[2], speed=FINE_SPEED_MM_S, tag="place")

        _log(f"[rt] 11/11 release to {RELEASE_SPAN_MM} mm")
        _jaw_to(robot, RELEASE_SPAN_MM, force=args.force)

        p = list(arm.pose())
        _log(f"\n[rt] DONE. Plate in the tray at floor2; arm at "
             f"x={p[0]:.3f} y={p[1]:.3f} z={p[2]:.3f}, jaws "
             f"{robot.opening().get('mm')} mm.")
        return 0
    except (ArmError, SafetyError, RuntimeError, TypeError) as exc:
        raise SystemExit(f"[rt] ABORT: {type(exc).__name__}: {exc}") from exc
    finally:
        # Always say WHERE it stopped. An abort inside the reader that prints
        # only "disconnected" tells the operator nothing about what to expect.
        try:
            p = list(robot.pose())
            _log(f"[rt] final pose x={p[0]:.3f} y={p[1]:.3f} z={p[2]:.3f} "
                 f"yaw={p[5]:.3f}; jaws {robot.opening().get('mm')} mm")
        except Exception:                                      # noqa: BLE001
            _log("[rt] final pose UNREADABLE -- inspect the rig before re-running")
        robot.close()
        _log("[rt] disconnected.")


def main(argv=None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
