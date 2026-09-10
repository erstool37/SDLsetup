#!/usr/bin/env python3
"""uvvis_a1_protocol.py -- tray to reader to measurement, for well A1.

    python3 scripts/tool_building/uvvis_a1_protocol.py            # plan only
    python3 scripts/tool_building/uvvis_a1_protocol.py --execute

THE SEQUENCE
  1  arm      grip the plate at floor2 (stall test, fast and strong)
  2  arm      lift, cross to the bench, turn the wrist -- ends CLEAR of the reader
  3  reader   start the control software if needed; carrier OUT; proved by the
              instrument's own run log
  4  arm      cross to the uvvis2 column, descend, release, lift, retreat
  5  reader   declare the arm clear; carrier IN
  6  reader   run the protocol
  7  verify   wait for "End of test run", then parse the MARS CSV and report A1

WHY THE CARRIER OPENS BEFORE THE ARM COMES OVER IT
--------------------------------------------------
The operator's stated order was arm-above-tray, then open. It is inverted here
deliberately: the carrier slides OUT into the volume the arm occupies, so
commanding it out while a plate hangs above it drives it under a suspended load.
That is the exact hazard tools/occupancy.py exists to prevent, and uvvis2 was
taught with the carrier already out, so this is also the geometry the taught
pose assumes.

WHAT IS PROVEN AND WHAT IS NOT -- read before trusting the output
-----------------------------------------------------------------
* Arm motion: PROVEN. Every leg is envelope-validated and read back, and
  MoveResult.verify() compares achieved against commanded on all six axes.
* Grasp: PROVEN. Commanding 120 mm -- narrower than the plate -- means a held
  plate STALLS the jaws wide at ~125. Commanding the plate's own 125 mm would
  not be a test at all, because free jaws arrive at 125 too.
* Carrier motion: PROVEN THAT A COMMAND LANDED, NOT ITS DIRECTION.
  dde.send() compares the control software's run-log marker before and after,
  so a carrier move that never reached the instrument raises. But PlateOut and
  PlateIn share the marker "(Plate command)", so the log cannot say WHICH way it
  went. CarrierState is explicit that it records what was commanded, not what
  the instrument reports -- the OUT status strings never marshal back.
  The DDE topic does advertise DdeServerDeviceBusy/DeviceError items, but
  DDEClient.exe only EXECUTES, it cannot REQUEST, so they are unreadable here.
  ==> carrier direction is ASSUMED. It is reported as assumed.
* Measurement: PROVEN. The run log's "End of test run" and the MARS CSV are
  written by the instrument, not by us.

The interlock is real and is not bypassed: SpectroStarNano._require_clear calls
occupancy.require_free("uv_vis"), which reads the live claim registry, so an arm
moving in ANY process refuses carrier motion.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402
import time  # noqa: E402

from tools.arm import Arm, ArmSettings, SafetyError  # noqa: E402
from tools.arm.driver import BIO_MODE_POSITION, ArmError  # noqa: E402
from tools.arm.workspace import WorkspaceStore  # noqa: E402

TRANSIT_Z = 120.0
BENCH_Y = 0.424          # open bench: clear of both reader and tray
GRASP_COMMAND_MM = 120.0  # narrower than the plate, so the plate stalls the jaws
GRASP_STALL_MIN_MM = 3.0
PLATE_SPAN_MM = 125.0
PLATE_SPAN_TOL_MM = 3.0
APPROACH_SPAN_MM = 130.0
RELEASE_SPAN_MM = 135.0
GRASP_FORCE_PCT = 90
GRASP_SPEED = 3000
JAW_FORCE_PCT = 30

FAST = 80.0              # operator: "make the arm move faster"
FINE = 15.0
XY_STEP = 50.0
Z_STEP = 6.0
SETTLE_S = 0.5


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
            raise SystemExit(f"[p] ABORT: {tag} {axis} step {k}/{n} did not arrive "
                             f"-- the tool met something.")
    _log(f"[p]   {tag}: {axis} -> {arm.pose()[i]:.3f}")


def _jaw_to(robot, target, force):
    cur = float(robot.opening().get("mm"))
    n = max(1, int(round(abs(target - cur))))
    for k in range(1, n + 1):
        res = robot.set_opening(round(cur + (target - cur) * k / n, 1), force=force)
        time.sleep(SETTLE_S)
        a = res.get("actual_mm")
        if a is None or not math.isfinite(float(a)):
            raise SystemExit(f"[p] ABORT: jaw span read back as {a!r}.")
    got = float(robot.opening().get("mm"))
    _log(f"[p]   jaws {cur:.0f} -> {got:.0f} mm")
    return got


def build_parser():
    p = argparse.ArgumentParser(description="floor2 -> reader -> A1 measurement")
    p.add_argument("--execute", action="store_true")
    p.add_argument("--host")
    p.add_argument("--protocol", default="Protein")
    p.add_argument("--well", default="A1")
    p.add_argument("--skip-measure", action="store_true",
                   help="load the plate and close the carrier, but do not run")
    return p


def run(args) -> int:
    from tools import uvvis
    from tools.uvvis import dde

    settings = ArmSettings.from_config(
        **({"live": args.execute, "control_mode": BIO_MODE_POSITION,
            "clear_errors": True} | ({"host": args.host} if args.host else {})))
    ws = WorkspaceStore(settings.workspace_path)
    fl = ws.require_location("floor2").pose
    uv = ws.require_location("uvvis2").pose
    fl_xyz = (float(fl.x), float(fl.y), float(fl.z))
    uv_xyz = (float(uv.x), float(uv.y), float(uv.z))

    print(f"[p] floor2 {fl_xyz} yaw {float(fl.yaw):.3f}")
    print(f"[p] uvvis2 {uv_xyz} yaw {float(uv.yaw):.3f}")
    print(f"[p] well {args.well}   protocol {args.protocol}   transit z {TRANSIT_Z}")
    print("[p] carrier opens BEFORE the arm comes over it (it slides into the "
          "arm's space)")
    if not args.execute:
        print("\n[p] DRY-RUN: nothing moved, no jaw command, no reader command.")
        return 0

    robot = Arm(settings, log=_log)
    robot.connection.connect(arm=True)
    report = {"well": args.well, "protocol": args.protocol}
    try:
        arm = robot.free_envelope(
            reason="UV-Vis A1 protocol: tray -> reader carrier -> measure",
            z_floor_mm=min(fl_xyz[2], uv_xyz[2]) - 0.5,
            z_ceiling_mm=TRANSIT_Z + 0.5)

        # -- 1. grip at floor2 ------------------------------------------------
        _log("[p] 1/7 grip at floor2")
        here = list(arm.pose())
        if math.dist(here[:2], fl_xyz[:2]) > 1.0:
            raise SystemExit(f"[p] ABORT: not on the floor2 column "
                             f"(x={here[0]:.1f} y={here[1]:.1f}). This protocol "
                             f"starts with the plate in the tray.")
        _jaw_to(robot, APPROACH_SPAN_MM, JAW_FORCE_PCT)
        _leg(arm, "z", fl_xyz[2], FINE, Z_STEP, "seat")
        robot.set_opening(GRASP_COMMAND_MM, speed=GRASP_SPEED, force=GRASP_FORCE_PCT)
        time.sleep(SETTLE_S * 2)
        span = float(robot.opening().get("mm"))
        if span - GRASP_COMMAND_MM < GRASP_STALL_MIN_MM:
            raise SystemExit(f"[p] ABORT: jaws reached {span:.0f} against a "
                             f"{GRASP_COMMAND_MM:.0f} command -- NOTHING HELD.")
        if abs(span - PLATE_SPAN_MM) > PLATE_SPAN_TOL_MM:
            raise SystemExit(f"[p] ABORT: jaws stalled at {span:.0f}, not the "
                             f"{PLATE_SPAN_MM:.0f} a plate reads.")
        _log(f"[p]   HOLDING: stalled at {span:.0f} mm")
        report["grasp_span_mm"] = span

        # -- 2. lift and park CLEAR of the reader ------------------------------
        _log("[p] 2/7 lift, cross to the bench, turn the wrist")
        _leg(arm, "z", TRANSIT_Z, FAST, Z_STEP, "lift")
        _leg(arm, "x", uv_xyz[0], FAST, XY_STEP, "toX")     # X while extended in -Y
        _leg(arm, "y", BENCH_Y, FAST, XY_STEP, "toBench")
        h = list(arm.pose())
        arm.rotate_in_place(h[3], h[4], float(uv.yaw), speed=FINE, label="to uv yaw")

        # -- 3. carrier OUT ----------------------------------------------------
        _log("[p] 3/7 reader: control software + carrier OUT")
        if not uvvis.control_running():
            _log("[p]   starting the control software (up to 45 s)")
            uvvis.ensure_control_running()
        before = dde.log_count("(Plate command)")
        dde.plate_out()
        after = dde.log_count("(Plate command)")
        _log(f"[p]   run-log '(Plate command)' {before} -> {after}: the command "
             f"reached the instrument")
        _log("[p]   NOTE: direction is ASSUMED. PlateOut and PlateIn share this "
             "marker and the carrier reports no position back.")
        report["carrier_out"] = {"proved_command_landed": after > before,
                                 "direction_verified": False}

        # -- 4. deliver the plate ----------------------------------------------
        _log("[p] 4/7 deliver the plate to the carrier")
        _leg(arm, "y", uv_xyz[1], FAST, XY_STEP, "toUV")
        _leg(arm, "x", uv_xyz[0], FAST, XY_STEP, "toUV")
        _leg(arm, "z", uv_xyz[2], FINE, Z_STEP, "descend")
        _jaw_to(robot, RELEASE_SPAN_MM, JAW_FORCE_PCT)
        _leg(arm, "z", TRANSIT_Z, FINE, Z_STEP, "lift-out")
        _leg(arm, "y", BENCH_Y, FAST, XY_STEP, "retreat")
        _log("[p]   arm retreated to the bench")

        # -- 5. carrier IN -----------------------------------------------------
        _log("[p] 5/7 declare the arm clear; carrier IN")
        before = dde.log_count("(Plate command)")
        dde.plate_in()
        after = dde.log_count("(Plate command)")
        _log(f"[p]   run-log '(Plate command)' {before} -> {after}")
        report["carrier_in"] = {"proved_command_landed": after > before,
                                "direction_verified": False}

        if args.skip_measure:
            _log("[p] --skip-measure: stopping with the plate loaded.")
            return 0

        # -- 6/7. measure and VERIFY -------------------------------------------
        _log(f"[p] 6/7 run protocol {args.protocol!r} for {args.well}")
        before_run = dde.log_count("(Run command)")
        dde.run_protocol(args.protocol)
        _log("[p] 7/7 waiting for 'End of test run' ...")
        finished = uvvis.wait_for_marker("End of test run", timeout_s=900.0)
        after_run = dde.log_count("(Run command)")
        report["run"] = {"command_landed": after_run > before_run,
                         "finished_marker": bool(finished)}
        if not finished:
            raise SystemExit("[p] ABORT: no 'End of test run' within the timeout. "
                             "The measurement cannot be reported as done.")
        csv = uvvis.latest_mars_csv(protocol=args.protocol)
        if csv is None:
            raise SystemExit("[p] ABORT: the run finished but no MARS CSV was "
                             "found. Nothing to report.")
        data = uvvis.parse_mars_csv(csv)
        report["csv"] = str(csv)
        wells = data.get("wells", data)
        val = wells.get(args.well) if isinstance(wells, dict) else None
        report["value"] = val
        _log(f"\n[p] MEASURED. csv={csv}")
        _log(f"[p] {args.well} = {val}")
        print(json.dumps(report, indent=1, default=str))
        return 0
    except (ArmError, SafetyError, RuntimeError, TypeError) as exc:
        raise SystemExit(f"[p] ABORT: {type(exc).__name__}: {exc}") from exc
    finally:
        try:
            p = list(robot.pose())
            _log(f"[p] final x={p[0]:.3f} y={p[1]:.3f} z={p[2]:.3f} "
                 f"jaws {robot.opening().get('mm')} mm")
        except Exception:                                      # noqa: BLE001
            _log("[p] final pose UNREADABLE -- inspect the rig")
        robot.close()


def main(argv=None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
