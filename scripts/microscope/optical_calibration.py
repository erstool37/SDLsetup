#!/usr/bin/env python3
"""optical_calibration.py -- drive the arm into position, then calibrate.

    python scripts/microscope/optical_calibration.py                 # plan only
    python scripts/microscope/optical_calibration.py --execute
    python scripts/microscope/optical_calibration.py --execute --from calibrate

WHY THIS EXISTS
---------------
The optical calibration stage was a set of scripts whose preconditions and
postconditions do not meet, so a human had to know the bridging chain and run it
by hand. Measured cases:

* ``find_target`` requires A1 within +/-0.5 mm at the TAUGHT height. Its own
  success path leaves the arm at the WORKING height (A1+3.0) and up to
  ``--xy-radius`` off A1. It therefore cannot run twice in a row. Observed
  2026-08-05: ``find_target/20260805-154651-721887`` aborted with
  *"z 188.3171 outside [184.8177, 185.8177] ... x is -3.1275 mm from A1"* --
  the exact pose its previous run had left behind.
* ``goto_a1 --to a1`` ends at the working height, not the taught one.
* ``goto_rise`` refuses more than 0.5 mm off A1's column, so it cannot fix XY;
  ``recenter_xy`` holds Z, so it cannot fix the height. Recovery needs BOTH, in
  that order, and nothing said so.

This script is that missing layer. It reads where the arm ACTUALLY is, chooses
the cheapest legal way back to the anchor, and only then calibrates.

WHAT IT DOES NOT DO
-------------------
It does not reimplement any stage. Each step shells into the script that already
owns it, through that script's own ``main(argv)``, so every guard, envelope and
readback check applies unchanged. This file contributes sequencing and the
normalisation nobody had written -- nothing else.

SAFETY
------
* Dry-run by default. ``--execute`` is required for any motion.
* Normalisation NEVER invents a route. Far from A1 it defers to ``goto_a1``'s
  validated six-leg transit; near A1 it uses the two small guarded moves. The
  boundary is ``NEAR_A1_MM``, well inside the lens radius rule that
  ``approach.validate_route`` enforces for anything closer.
* It refuses to start on a latched controller fault and never clears one.
* Between stages it re-reads the pose rather than assuming the previous stage
  ended where it said it would -- which is the whole failure this addresses.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from tools.arm.api import ARRIVAL_TOL_MM, Arm, ArmSettings  # noqa: E402
from tools.arm.safety import SafetyError  # noqa: E402
from tools.arm.workspace import WorkspaceStore  # noqa: E402

#: Beyond this distance from A1's XY, normalisation uses goto_a1's full
#: validated transit instead of a local correction. Inside it, the arm is
#: already in the microscope's neighbourhood and the two small guarded moves
#: apply. Chosen to sit inside approach.LENS_XY_RADIUS_MM (20 mm) so the
#: near-lens rule governs everything closer.
NEAR_A1_MM = 15.0

#: The height find_target demands to start: the taught A1 exactly.
CALIBRATE_START_RISE_MM = 0.0

#: XY agreement required before a stage that wants "at A1". find_target uses
#: +/-0.5; normalisation aims tighter so it does not hand over a borderline pose.
XY_OK_MM = 0.20

#: Z agreement for the same. ARRIVAL_TOL_MM is the measured arrival tolerance
#: (0.05 mm); a normalisation that lands inside it has genuinely arrived.
Z_OK_MM = max(ARRIVAL_TOL_MM, 0.02)

STAGES = ("normalise", "calibrate")


class StageError(RuntimeError):
    pass


def _load(path: Path):
    """Import a sibling phase script by path, registered in sys.modules.

    Registration matters: a module loaded from a spec alone is absent from
    sys.modules, and @dataclasses.dataclass resolves its own module through it --
    which fails with a bare AttributeError deep inside dataclasses.
    """
    spec = importlib.util.spec_from_file_location(path.stem, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def read_pose(host: str | None) -> tuple[list[float], dict]:
    """The arm's ACTUAL pose and status. Read-only: this never arms the arm."""
    settings = ArmSettings.from_config(str(REPO / "configs" / "config.yaml"),
                                       host=host, live=True)
    arm = Arm(settings)
    status = arm.status()
    pose = list(status["pose"])
    return pose, status


def a1_pose() -> list[float]:
    store = WorkspaceStore()
    p = store.require_location("microscope").pose
    return [p.x, p.y, p.z, p.roll, p.pitch, p.yaw]


def describe(pose: list[float], a1: list[float]) -> dict:
    dx, dy = pose[0] - a1[0], pose[1] - a1[1]
    return {"dx_mm": dx, "dy_mm": dy,
            "xy_mm": (dx * dx + dy * dy) ** 0.5,
            "rise_mm": pose[2] - a1[2]}


def plan_normalisation(off: dict) -> list[tuple[str, list[str]]]:
    """The cheapest legal route back to the calibration start pose.

    Returns a list of (script, argv). Empty means already there.
    """
    steps: list[tuple[str, list[str]]] = []
    need_xy = off["xy_mm"] > XY_OK_MM
    need_z = abs(off["rise_mm"] - CALIBRATE_START_RISE_MM) > Z_OK_MM

    if off["xy_mm"] > NEAR_A1_MM:
        # Far away: do not improvise. goto_a1 owns the only validated route,
        # including the dogleg that keeps the tray clear of the UV-Vis reader.
        steps.append(("goto_a1", ["--to", "a1", "--all"]))
        # It lands at the WORKING height, which is not where calibration starts.
        steps.append(("goto_rise", [str(CALIBRATE_START_RISE_MM)]))
        return steps

    # Near A1. Order is fixed and not a preference: recenter_xy holds Z, and
    # goto_rise refuses more than 0.5 mm off A1's column -- so XY must be right
    # before Z is touched, or the Z step is refused.
    if need_xy:
        steps.append(("recenter_xy", []))
    if need_z or need_xy:
        steps.append(("goto_rise", [str(CALIBRATE_START_RISE_MM)]))
    return steps


SCRIPTS = {
    "goto_a1": REPO / "scripts/microscope/goto_a1.py",
    "recenter_xy": REPO / "scripts/microscope/recenter_xy.py",
    "goto_rise": REPO / "scripts/tool_building/goto_rise.py",
    "find_target": REPO / "scripts/microscope/find_target.py",
}


def run_step(name: str, argv: list[str], execute: bool) -> int:
    path = SCRIPTS[name]
    full = list(argv) + (["--execute"] if execute else [])
    print("\n>>> %s %s" % (path.name, " ".join(full)), flush=True)
    mod = _load(path)
    rc = mod.main(full)
    print("<<< %s rc=%d" % (path.name, rc), flush=True)
    return rc


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description=__doc__.split("WHY THIS EXISTS")[0].strip(),
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--execute", action="store_true",
                    help="actually move (default: plan only, commands nothing)")
    ap.add_argument("--host", default=None)
    ap.add_argument("--from", dest="from_stage", default="normalise",
                    choices=STAGES,
                    help="skip ahead; the pose is still re-read and checked "
                         "(default: %(default)s)")
    ap.add_argument("--xy-radius", type=float, default=4.0,
                    help="passed to find_target (default: %(default)s)")
    ap.add_argument("--top-n", type=int, default=3,
                    help="passed to find_target (default: %(default)s)")
    ap.add_argument("--dot-mm", type=float, default=1.0,
                    help="passed to find_target (default: %(default)s)")
    ap.add_argument("--out-dir", default=None)
    return ap


def run(args) -> int:
    a1 = a1_pose()
    print("=" * 72)
    print("optical calibration stage      mode: %s"
          % ("LIVE -- THE ARM WILL MOVE" if args.execute else "DRY-RUN"))
    print("taught A1   x=%.4f y=%.4f z=%.4f" % (a1[0], a1[1], a1[2]))

    try:
        pose, status = read_pose(args.host)
    except Exception as exc:  # noqa: BLE001
        print("\nCANNOT READ THE ARM: %s" % exc)
        print("Normalisation needs the ACTUAL pose; refusing to plan blind.")
        return 2

    off = describe(pose, a1)
    print("arm is at   x=%.4f y=%.4f z=%.4f" % (pose[0], pose[1], pose[2]))
    print("offset      dx=%+.3f dy=%+.3f  |xy|=%.3f mm  rise=%+.3f mm"
          % (off["dx_mm"], off["dy_mm"], off["xy_mm"], off["rise_mm"]))
    print("controller  state=%s error=%s warn=%s"
          % (status.get("state"), status.get("error_code"), status.get("warn_code")))

    if status.get("error_code"):
        print("\nREFUSING: the controller has a latched fault (error_code=%s). "
              "A latched fault may be the record of a previous collision; it is "
              "not cleared here. Inspect the arm, then clear it deliberately."
              % status["error_code"])
        return 3

    steps = plan_normalisation(off)
    print()
    if not steps:
        print("normalise   already at the calibration start pose, nothing to do")
    else:
        how = "full validated transit" if off["xy_mm"] > NEAR_A1_MM \
            else "local correction (|xy| <= %.0f mm)" % NEAR_A1_MM
        print("normalise   %s, %d step(s):" % (how, len(steps)))
        for name, argv in steps:
            print("              %s %s" % (SCRIPTS[name].name, " ".join(argv)))
    print("calibrate   find_target.py --xy-radius %.1f --top-n %d --dot-mm %.2f"
          % (args.xy_radius, args.top_n, args.dot_mm))
    print("=" * 72)

    if not args.execute:
        print("\n[dry-run] nothing was commanded. Re-run with --execute.")
        return 0

    started = time.time()
    record = {"a1": a1, "start_pose": pose, "start_offset": off, "steps": []}

    if args.from_stage == "normalise":
        for name, argv in steps:
            rc = run_step(name, argv, execute=True)
            record["steps"].append({"stage": "normalise", "script": name,
                                    "argv": argv, "rc": rc})
            if rc != 0:
                print("\nNORMALISATION FAILED at %s (rc=%d). Not calibrating from "
                      "an unknown pose." % (name, rc))
                return 4

        # Re-read rather than trust. This is the failure the whole file exists
        # for: a stage that reports success is not evidence of a pose.
        pose2, _ = read_pose(args.host)
        off2 = describe(pose2, a1)
        print("\nnormalised  x=%.4f y=%.4f z=%.4f  (|xy|=%.3f mm, rise=%+.3f mm)"
              % (pose2[0], pose2[1], pose2[2], off2["xy_mm"], off2["rise_mm"]))
        record["normalised_pose"] = pose2
        record["normalised_offset"] = off2
        if off2["xy_mm"] > 0.5 or abs(off2["rise_mm"] - CALIBRATE_START_RISE_MM) > 0.5:
            print("REFUSING: normalisation reported success but the arm is not at "
                  "the calibration start pose. find_target would abort anyway; "
                  "stopping here with the evidence instead.")
            return 5

    cal_argv = ["--xy-radius", str(args.xy_radius),
                "--top-n", str(args.top_n),
                "--dot-mm", str(args.dot_mm)]
    if args.out_dir:
        cal_argv += ["--out-dir", args.out_dir]
    rc = run_step("find_target", cal_argv, execute=True)
    record["steps"].append({"stage": "calibrate", "script": "find_target",
                            "argv": cal_argv, "rc": rc})
    record["elapsed_s"] = round(time.time() - started, 1)
    record["rc"] = rc

    out = REPO / "dataset" / "captures" / "optical_calibration"
    out.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    (out / ("%s.json" % stamp)).write_text(json.dumps(record, indent=1))
    print("\n[record] %s" % (out / ("%s.json" % stamp)))
    print("stage rc=%d in %.0f s" % (rc, record["elapsed_s"]))
    return rc


def main(argv=None) -> int:
    try:
        return run(build_parser().parse_args(argv))
    except SafetyError as exc:
        print("\nSAFETY: %s" % exc)
        return 6
    except KeyboardInterrupt:
        print("\ninterrupted")
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
