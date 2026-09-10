#!/usr/bin/env python3
"""uvvis_fetch_back.py -- the moment the run finishes, take the plate back to the tray.

    python3 scripts/tool_building/uvvis_fetch_back.py                 # plan only
    python3 scripts/tool_building/uvvis_fetch_back.py --execute
    python3 scripts/tool_building/uvvis_fetch_back.py --execute --wait-for-run

THE TRIGGER
-----------
``--wait-for-run`` blocks on the instrument's own "End of test run" marker and
starts the arm the instant it appears, polling once a second rather than the
5 s default -- the operator's requirement is that the arm move as soon as the
reader is done, not some seconds later.

The wait is TIME-ANCHORED, and that is not a detail. The marker string is
already in the log from every previous run, so "does the log contain it" is
satisfied before the run even starts. This records the count first and requires
it to INCREASE. A check that would pass before the thing happened is not a
check -- it cost a wrong "MEASURED" report on 2026-08-13.

Without ``--wait-for-run`` the run is assumed already finished, which is the
normal case when this is invoked after uvvis_a1_protocol.

ORDER, AND WHY
--------------
  1  carrier OUT   -- with the arm parked at the bench and NOT moving
  2  arm           -- cross, descend, grasp (stall test), lift, retreat
  3  arm           -- Y first, then X, then down onto floor2, release

Step 1 happens before the arm approaches because the carrier slides out into the
volume the arm occupies. Step 3 puts Y before X because reaching back at low Y
runs the arm out of DOF -- error 23, observed 2026-08-13 at x=344.7, y=0.4.
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

TRANSIT_Z = 120.0
BENCH_Y = 0.424
APPROACH_SPAN_MM = 130.0
GRASP_COMMAND_MM = 120.0
GRASP_STALL_MIN_MM = 3.0
PLATE_SPAN_MM = 125.0
PLATE_SPAN_TOL_MM = 3.0
RELEASE_SPAN_MM = 135.0
GRASP_FORCE_PCT = 90
GRASP_SPEED = 3000
JAW_FORCE_PCT = 30
FAST = 80.0
FINE = 15.0
XY_STEP = 50.0
Z_STEP = 6.0
SETTLE_S = 0.5
RUN_FINISHED = "End of test run"


def _log(m):
    print(m, flush=True)


def _leg(arm, axis, want, speed, step, tag):
    here = list(arm.pose())
    i = {"x": 0, "y": 1, "z": 2}[axis]
    span = want - here[i]
    if abs(span) < 0.05:
        return
    n = max(1, math.ceil(abs(span) / step))
    for k in range(1, n + 1):
        t = list(here)
        t[i] = here[i] + span * k / n
        r = arm.move_pose(tuple(t), speed=speed, label=f"{tag}.{axis}[{k}/{n}]",
                          takeup=False)
        if r.verify()["moved"] is False:
            raise SystemExit(f"[f] ABORT: {tag} {axis} step {k}/{n} did not arrive.")
    _log(f"[f]   {tag}: {axis} -> {arm.pose()[i]:.3f}")


def _jaw_to(robot, target, force):
    cur = float(robot.opening().get("mm"))
    n = max(1, int(round(abs(target - cur))))
    for k in range(1, n + 1):
        res = robot.set_opening(round(cur + (target - cur) * k / n, 1), force=force)
        time.sleep(SETTLE_S)
        a = res.get("actual_mm")
        if a is None or not math.isfinite(float(a)):
            raise SystemExit(f"[f] ABORT: jaw span read back as {a!r}.")
    got = float(robot.opening().get("mm"))
    _log(f"[f]   jaws {cur:.0f} -> {got:.0f} mm")
    return got


def wait_for_run_end(dde, *, timeout_s: float, poll_s: float) -> bool:
    """Block until the run-finished marker count INCREASES. Time-anchored."""
    before = dde.log_count(RUN_FINISHED)
    _log(f"[f] waiting for {RUN_FINISHED!r} to go past {before} "
         f"(poll {poll_s}s, timeout {timeout_s}s)")
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if dde.log_count(RUN_FINISHED) > before:
            _log("[f] RUN FINISHED -- triggering the arm now")
            return True
        time.sleep(poll_s)
    return False


def build_parser():
    p = argparse.ArgumentParser(description="fetch the plate back from the reader")
    p.add_argument("--execute", action="store_true")
    p.add_argument("--host")
    p.add_argument("--wait-for-run", action="store_true",
                   help="block until the instrument reports the run finished, then "
                        "start the arm immediately")
    p.add_argument("--timeout-s", type=float, default=900.0)
    p.add_argument("--poll-s", type=float, default=1.0)
    return p


def run(args) -> int:
    from tools.uvvis import dde

    settings = ArmSettings.from_config(
        **({"live": args.execute, "control_mode": BIO_MODE_POSITION,
            "clear_errors": True} | ({"host": args.host} if args.host else {})))
    ws = WorkspaceStore(settings.workspace_path)
    uv = ws.require_location("uvvis2").pose
    fl = ws.require_location("floor2").pose
    uv_xyz = (float(uv.x), float(uv.y), float(uv.z))
    fl_xyz = (float(fl.x), float(fl.y), float(fl.z))

    print(f"[f] uvvis2 {uv_xyz}   floor2 {fl_xyz}")
    print(f"[f] trigger: {'wait for run end' if args.wait_for_run else 'run assumed finished'}")
    if not args.execute:
        print("\n[f] DRY-RUN: nothing moved, no reader command.")
        return 0

    if args.wait_for_run and not wait_for_run_end(
            dde, timeout_s=args.timeout_s, poll_s=args.poll_s):
        raise SystemExit(f"[f] ABORT: no new {RUN_FINISHED!r} within "
                         f"{args.timeout_s}s. Not moving the arm.")

    robot = Arm(settings, log=_log)
    robot.connection.connect(arm=True)
    try:
        arm = robot.free_envelope(
            reason="fetch the measured plate from the carrier back to the tray",
            z_floor_mm=min(fl_xyz[2], uv_xyz[2]) - 0.5,
            z_ceiling_mm=TRANSIT_Z + 0.5)

        # -- 1. carrier OUT, arm parked and still ----------------------------
        _log("[f] 1/3 carrier OUT (the arm is parked at the bench and not moving)")
        before = dde.log_count("(Plate command)")
        dde.plate_out()
        _log(f"[f]   '(Plate command)' {before} -> {dde.log_count('(Plate command)')} "
             f"(direction not reported by the instrument -- assumed OUT)")

        # -- 2. fetch --------------------------------------------------------
        _log("[f] 2/3 fetch the plate")
        _jaw_to(robot, APPROACH_SPAN_MM, JAW_FORCE_PCT)
        _leg(arm, "z", TRANSIT_Z, FAST, Z_STEP, "lift")
        _leg(arm, "y", uv_xyz[1], FAST, XY_STEP, "toUV")
        _leg(arm, "x", uv_xyz[0], FAST, XY_STEP, "toUV")
        _leg(arm, "z", uv_xyz[2], FINE, Z_STEP, "descend")
        robot.set_opening(GRASP_COMMAND_MM, speed=GRASP_SPEED, force=GRASP_FORCE_PCT)
        time.sleep(SETTLE_S * 2)
        span = float(robot.opening().get("mm"))
        if span - GRASP_COMMAND_MM < GRASP_STALL_MIN_MM:
            raise SystemExit(f"[f] ABORT: jaws reached {span:.0f} against a "
                             f"{GRASP_COMMAND_MM:.0f} command -- NOTHING HELD. "
                             f"The plate is still in the carrier.")
        if abs(span - PLATE_SPAN_MM) > PLATE_SPAN_TOL_MM:
            raise SystemExit(f"[f] ABORT: jaws stalled at {span:.0f}, not the "
                             f"{PLATE_SPAN_MM:.0f} a plate reads.")
        _log(f"[f]   HOLDING: stalled at {span:.0f} mm")
        _leg(arm, "z", TRANSIT_Z, FINE, Z_STEP, "lift-out")
        _leg(arm, "y", BENCH_Y, FAST, XY_STEP, "retreat")

        # -- 3. return to the tray. Y BEFORE X -------------------------------
        _log("[f] 3/3 return to floor2 (Y before X -- reaching back at low Y "
             "runs out of DOF)")
        h = list(arm.pose())
        arm.rotate_in_place(h[3], h[4], float(fl.yaw), speed=FINE, label="to floor yaw")
        _leg(arm, "y", fl_xyz[1], FAST, XY_STEP, "toTray")
        _leg(arm, "x", fl_xyz[0], FAST, XY_STEP, "toTray")
        _leg(arm, "z", fl_xyz[2], FINE, Z_STEP, "place")
        _jaw_to(robot, RELEASE_SPAN_MM, JAW_FORCE_PCT)
        p = list(arm.pose())
        _log(f"\n[f] DONE. Plate back in the tray at floor2. "
             f"x={p[0]:.3f} y={p[1]:.3f} z={p[2]:.3f}")
        return 0
    except (ArmError, SafetyError, RuntimeError, TypeError) as exc:
        raise SystemExit(f"[f] ABORT: {type(exc).__name__}: {exc}") from exc
    finally:
        try:
            p = list(robot.pose())
            _log(f"[f] final x={p[0]:.3f} y={p[1]:.3f} z={p[2]:.3f} "
                 f"jaws {robot.opening().get('mm')} mm")
        except Exception:                                      # noqa: BLE001
            _log("[f] final pose UNREADABLE")
        robot.close()


def main(argv=None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
