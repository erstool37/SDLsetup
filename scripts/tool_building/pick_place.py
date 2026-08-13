#!/usr/bin/env python3
"""xArm pick sequence: home -> floor -> grab -> home -> microscope.

Uses the UFACTORY BIO Gripper G2 (open/close mode; 71-150 mm range; <=20 N
force-limited). Joint-space moves using angles taught into the workspace store
(~/.sdl_lab/robot_arm/workspace.json). Unlike scripts/gripper.py, this script
DOES command arm motion (joint moves via the shared Arm) and is therefore
gated behind --execute. Without --execute it only prints the planned moves
(dry-run).

Sequence:
  1. open gripper (prepare to grab)   [skip with --no-open]
  2. joint  -> home        (robot zero pose; the sole joint step)
  3. axis   -> floor       (Y across at height, then Z down)
  4. close gripper         (GRAB)
  5. axis   -> tray lift   (pure Z rise at the tray column, to working height)
  6. rotate -> scope yaw   (wrist 0 -> 90 deg, above the tray, XYZ held)
  7. axis   -> standoff    (Y across the bench, then X align)
  8. axis   -> microscope  (pure Y slide into the wells)

=== Migrated Arm Plumbing (this reorg) ===
This used to open its own XArmAPI session (connect/motion_enable/set_mode/
set_state/set_servo_angle/BIO gripper calls, all hand-rolled). It now builds
ONE shared tools.arm.Arm and drives everything through it:
  - Every move here is joint-space, via Arm.move_joints() with angles read
    straight from the taught workspace store -- never computed from a pose
    (see joints_for()). No Cartesian envelope applies to a joint move.
  - Like plate_imaging.py, this crosses the whole workspace (home -> floor ->
    microscope), so it still builds an explicit Arm.free_envelope() with a Z
    corridor derived from the three taught locations' own Z's, for
    consistency/defensive completeness (status displays, and any future
    Cartesian use) even though move_joints() itself does not consult it.
  - The BIO gripper goes through Arm.grip() / Arm.release().
  - Host and gripper speeds come from ArmSettings (config.yaml > arm.json >
    code default) instead of a direct robot.json read.

DELIBERATE BEHAVIOUR CHANGE -- a safety tightening, not a relaxation: the old
enable_motion() unconditionally called clean_warn()/clean_error() BEFORE
checking arm.error_code, so a pre-existing latched fault was silently wiped on
every connect and the old error_code check could only catch a fault raised
during enable itself. The shared driver never clears a fault automatically
(clear_errors=False by default) -- a latched error_code now blocks connecting
outright unless the new --clear-errors flag is passed. This is the same
"never clear a controller fault automatically" contract the migration brief
calls out; pick_sequence.py never had a way to express it before.

Config (host, gripper params) is read from ArmSettings, which resolves
src/tools/devices/arm/arm.json (renamed/moved from
src/tools/robot_arm/robot.json) through config.yaml precedence; locations
from the taught workspace store.

Gripper config keys used (ArmSettings.open_speed / close_speed, from
arm.json's gripper section):
  gripper.open_speed   (int, 0-4000, default 2000)
  gripper.close_speed  (int, 0-4000, default 1000)

Usage:
  python scripts/pick_sequence.py            # dry-run: print plan, no motion
  python scripts/pick_sequence.py --execute  # LIVE: moves the arm + gripper
  python scripts/pick_sequence.py --execute --speed 15
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


import argparse
from typing import Any

from tools.arm import Arm, ArmSettings, SafetyError
from tools.arm.driver import ArmError
from tools.arm.workspace import WorkspaceStore

# The sequence, as typed steps rather than a flat list of location names.
#
# TWO KINDS OF MOVE, deliberately mixed:
#
#   axis   -- a Cartesian target reached ONE AXIS AT A TIME, never diagonally.
#             Operator rule 2026-08-07, in .claude/rules/axis-sequential-motion.md.
#             ENTERING a taught position and LEAVING it are BOTH of this kind.
#             The tray is entered by traversing Y at height then descending Z,
#             and left by a pure Z lift; A1 is entered by a pure Y slide. A joint
#             interpolation's Cartesian path is an emergent property of seven
#             angles, so an approach direction it happens to take is not one the
#             code ever stated.
#   joint  -- taught angles, straight off the controller. ONE step uses this,
#             and only because the controller refuses the alternative: see home,
#             below. A joint move has no Cartesian envelope, so its angles may
#             only ever come from a taught location, never from arithmetic.
#
# Why the tray legs had to stop being joint moves. After the tray was relocated
# 322 mm in -Y (floor y +137.563 -> -184.774), both of these happened in one
# session, with endpoints that were individually fine:
#
#   * floor -> microscope aborted with controller error 31 (collision) at
#     x=461.0 y=253.9 z=77.9, a third of the way across the bench;
#   * floor -> home was ACCEPTED, moved ~23 mm of Z, stopped, and reported
#     success -- the run printed "arrived home" over the tray.
#
# home appears ONCE, before the plate is picked up, and it is the sole joint
# step -- the documented exception to the Cartesian in/out law above. It is
# deliberately absent from the carrying route, so the exception never applies
# while anything is held or anything is being approached.
#
# Measured 2026-08-07, with the Y leg subdivided into 25 mm steps: steps 1-7
# (y -184.8 -> -22.7 at x=366.56, z=61.37) all landed; step 8, the one whose
# target IS the home pose (y=0.424), was refused with code=-9 and no latched
# fault. home is the all-joints-zero configuration -- a kinematic singularity --
# and asking for it as a Cartesian target is a different request from asking for
# it as seven angles. INFERENCE, not a controller message: the evidence is that
# the seven steps approaching it succeeded and only the one landing on it did
# not. It is not worth resolving, because the route has no reason to go there:
# the tray and the scope are both reachable from each other directly.
#   rotate -- a wrist turn at fixed XYZ. NEEDED because neither move() nor
#             move_axiswise() can change orientation: the first inherits the
#             anchor's, the second inherits whatever the arm is already holding.
#             The tray is taught at yaw 0.003 and the scope at yaw 89.985, so
#             without an explicit step the plate reaches the objective 90 deg
#             out. The joint route this replaced turned the wrist only as an
#             emergent property of interpolating seven angles -- nothing said
#             so, which is why going Cartesian dropped it silently.
SEQUENCE = [
    {"action": "open"},
    # REACH THE TRAY CARTESIAN-ONLY. The joint move to `home` that used to sit
    # here was removed 2026-08-11. A joint interpolation's Cartesian path is
    # whatever seven angles sweep out -- a diagonal by construction, constrained
    # by no envelope, because joint space has none. The operator watched it
    # disturb the tray. Rising first is what makes the Y traverse safe: the two
    # error-31 collisions were at z~78-82, and this crosses at z~191.
    #
    # This assumes the arm starts on the TRAY side (home region). Resuming from
    # the wells goes through --return-to-tray first, which uses the mirrored
    # corridor; do not reorder these axes to try to serve both.
    {"action": "axis", "name": "tray approach", "order": "zyx", "speed": "transit",
     "xyz_from": ("floor", "floor", "scope_standoff")},
    {"action": "rotate", "name": "to tray yaw", "rpy_from": "floor", "speed": "fine"},
    {"action": "axis", "name": "floor", "order": "zyx", "speed": "fine"},
    {"action": "grip"},
    # Leave the tray straight up: X and Y stay at the tray's own column and only
    # Z rises, to the scope's working height. Composed from two taught locations
    # because the top of the lift is not itself a taught place.
    {"action": "axis", "name": "tray lift", "order": "zyx", "speed": "fine",
     "xyz_from": ("floor", "floor", "scope_standoff")},
    # Turn HERE, above the tray, before crossing the bench -- operator decision
    # 2026-08-11. The plate then traverses already in its final orientation, so
    # nothing rotates near the objective.
    {"action": "rotate", "name": "to scope yaw", "rpy_from": "scope_standoff",
     "speed": "fine"},
    {"action": "axis", "name": "scope_standoff", "order": "zyx", "speed": "transit"},
    {"action": "axis", "name": "microscope", "order": "yxz", "speed": "fine"},
]

# THE RETURN, which is the outbound route read backwards. Each leg is still a
# single axis, and the wrist still turns above the tray -- reversing a route
# must not invent a path the forward route never took.
#
#   scope_standoff  pure Y out of the wells                        [fine]
#   tray lift       X back, then Y back, at working height         [transit]
#   to tray yaw     wrist 90 -> 0, above the tray                  [fine]
#   floor           pure Z down into the tray                      [fine]
#   open            release
#
# "speed" names which cap the leg answers to, never a number: legs near the
# objective or descending into the tray are fine-positioning moves and stay at
# --cart-speed; only the open bench crossing takes --transit-speed.
RETURN_SEQUENCE = [
    {"action": "axis", "name": "scope_standoff", "order": "yxz", "speed": "fine"},
    {"action": "axis", "name": "tray lift", "order": "xyz", "speed": "transit",
     "xyz_from": ("floor", "floor", "scope_standoff")},
    {"action": "rotate", "name": "to tray yaw", "rpy_from": "floor", "speed": "fine"},
    {"action": "axis", "name": "floor", "order": "zyx", "speed": "fine"},
    {"action": "open"},
]

#: Headroom added to BOTH ends of the transit Z corridor.
#:
#: The corridor is derived from the taught locations' own Z values, and the
#: route travels at exactly the taught working height -- so with no margin the
#: ceiling is the operating height and any settling error aborts the run. It
#: did, on 2026-08-11: actual z 191.209518 against a ceiling of 191.209473,
#: over by 45 NANOMETRES, mid-way through carrying the plate back.
#:
#: 0.5 mm is the envelope's own READBACK_SLACK_MM -- the tolerance the readback
#: check already allows -- and is 20x smaller than the 10 mm the anchored sweep
#: envelope permits above A1, so it cannot license a climb toward the objective.
#: It is settling headroom, not travel budget.
Z_CORRIDOR_MARGIN_MM = 0.5

#: Locations that must exist in the taught store. Step names are labels and are
#: NOT all taught places ("tray lift" is composed), so the Z corridor below is
#: derived from this list rather than from whatever the sequence happens to name.
TAUGHT = ("home", "floor", "scope_standoff", "microscope")

# Steps before the plate is in the gripper. --already-holding skips exactly
# these, so an aborted run can be resumed without the gripper opening at
# whatever height the arm happened to stop at and dropping the plate.
#
# The first step of a resume is the standoff leg, ordered zyx, so it buys
# the full working height BEFORE travelling -- the correct recovery from any
# low pose the abort happened to leave the arm in.
STEPS_BEFORE_HOLDING = 5

#: How far off the standoff the arm may be and still be allowed to slide in.
#: Deliberately far tighter than the 150 mm slide it authorises: the check is
#: asking "is the arm where the route left it", not "is it roughly nearby".
SLIDE_IN_TOL_MM = 1.0
SLIDE_IN_TOL_DEG = 0.5

# WHY THE APPROACH GOES THROUGH scope_standoff
# --------------------------------------------
# home -> microscope as a single joint move collided TWICE, reproducibly:
# controller error 31 at (461.0, 253.9, 77.9) and again at (397.3, 216.7, 81.9),
# both with the plate held. There is something on the bench at z ~ 80 between
# the base and the scope, and a joint interpolation drives straight through it.
#
# scope_standoff is A1 displaced 150 mm along -Y at A1's exact Z, so the route
# becomes: buy the full working height FIRST (z 61 -> 185, clearing the obstacle
# by ~105 mm), traverse in Y, align X -- all at a height where the bench is
# empty -- and only then slide in along Y alone. The final leg changes ONE axis,
# which is the whole reason the standoff exists.
#
# It also sidesteps a stale reading: the taught microscope POSE has z=185.3177
# (re-taught 2026-08-04), but its stored joint angles still forward-kinematic to
# z=177.247 -- the z re-teach never updated them. The Cartesian route uses the
# pose, which is the intended working height; the joint angles are 8.07 mm low
# and should not be used to reach A1 until they are re-taught.


def joints_for(store: WorkspaceStore, name: str) -> list[float]:
    loc = store.require_location(name)
    angles = loc.metadata.get("joint_angles_deg")
    if not angles or not isinstance(angles, list):
        raise SystemExit(
            f"location {name!r} has no joint_angles_deg in metadata; cannot do a "
            f"joint-space move. Teach it live with "
            f"scripts/tool_building/teach_location.py."
        )
    return [float(a) for a in angles]


def xyz_for(store: WorkspaceStore, name: str) -> tuple[float, float, float]:
    pose = store.require_location(name).pose
    return (float(pose.x), float(pose.y), float(pose.z))


def xyz_from(store: WorkspaceStore, sources: tuple[str, str, str]) -> tuple[float, float, float]:
    """One axis from each named location, for a target no single place defines.

    The top of the tray lift is (tray X, tray Y, scope Z): a real point on the
    route that nobody ever jogged the arm to, so it has no taught pose of its
    own. Composing it from taught values keeps it derived rather than typed in.
    """
    axes = ("x", "y", "z")
    return tuple(float(getattr(store.require_location(loc).pose, axis))
                 for loc, axis in zip(sources, axes, strict=True))


def rpy_for(store: WorkspaceStore, name: str) -> tuple[float, float, float]:
    pose = store.require_location(name).pose
    return (float(pose.roll), float(pose.pitch), float(pose.yaw))


def build_plan(store: WorkspaceStore, sequence: list[dict] | None = None) -> list[dict]:
    plan: list[dict] = []
    for step in (SEQUENCE if sequence is None else sequence):
        item = dict(step)
        if step["action"] == "joint":
            item["angles"] = joints_for(store, step["name"])
        elif step["action"] == "axis":
            item["xyz"] = (xyz_from(store, step["xyz_from"]) if "xyz_from" in step
                           else xyz_for(store, step["name"]))
        elif step["action"] == "rotate":
            item["rpy"] = rpy_for(store, step["rpy_from"])
        plan.append(item)
    return plan


def print_plan(plan: list[dict], settings: ArmSettings) -> None:
    print("[seq] planned sequence:")
    for n, item in enumerate(plan, 1):
        act = item["action"]
        if act == "open":
            print(f"  {n:>2}. gripper OPEN (bio)  open_speed={settings.open_speed}")
        elif act == "grip":
            print(f"  {n:>2}. gripper CLOSE (bio, GRAB)  "
                  f"close_speed={settings.close_speed}")
        elif act == "joint":
            a = ", ".join(f"{v:.3f}" for v in item["angles"])
            print(f"  {n:>2}. joint  -> {item['name']:<11} [{a}] deg")
        elif act == "axis":
            x, y, z = item["xyz"]
            legs = " then ".join(item["order"])
            print(f"  {n:>2}. axis   -> {item['name']:<13} "
                  f"x={x:.3f} y={y:.3f} z={z:.3f}   {legs}  "
                  f"[{item.get('speed', 'fine')}]")
        elif act == "rotate":
            roll, pitch, yaw = item["rpy"]
            print(f"  {n:>2}. rotate -> {item['name']:<13} "
                  f"roll={roll:.3f} pitch={pitch:.3f} yaw={yaw:.3f}   "
                  f"(from {item['rpy_from']}, XYZ held)")


def speed_for(item: dict, args: argparse.Namespace) -> float:
    """Which cap this leg answers to. Unmarked legs are fine-positioning.

    Defaulting to the SLOWER of the two matters: a leg added later without a
    speed class must not silently inherit workspace-transit speed.
    """
    return args.transit_speed if item.get("speed") == "transit" else args.cart_speed


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="xArm pick sequence "
                                                 "(home->floor->grab->home->microscope)")
    parser.add_argument("--execute", action="store_true", help="LIVE: actually move the "
                                                               "arm + gripper")
    parser.add_argument("--host")
    parser.add_argument("--speed", type=float, default=20.0, help="joint speed deg/s "
                                                                  "(default 20, conservative)")
    parser.add_argument("--return-to-tray", action="store_true",
                        help="REVERSE: carry the plate from the wells back to the "
                             "tray and release it. Retraces the outbound path, "
                             "wrist turn included, in reverse.")
    parser.add_argument("--transit-speed", type=float, default=None,
                        help="mm/s for the open bench crossing only (default: same "
                             "as --cart-speed). Legs near the objective or into the "
                             "tray always use --cart-speed.")
    parser.add_argument("--slide-in", action="store_true",
                        help="run ONLY the final Y slide into the wells, from a "
                             "plate already staged at scope_standoff. Verifies the "
                             "arm is actually there first and aborts if not.")
    parser.add_argument("--stop-at-standoff", action="store_true",
                        help="end the run staged at scope_standoff, before the "
                             "final Y slide into the wells")
    parser.add_argument("--stop-after-grip", action="store_true",
                        help="run only home -> floor -> GRAB and stop there, with "
                             "the plate held at the tray. Nothing transits.")
    parser.add_argument("--already-holding", action="store_true",
                        help="the plate is ALREADY in the gripper (e.g. resuming "
                             "after an aborted run): skip the open/home/floor/grab "
                             "steps so the gripper never opens mid-air")
    parser.add_argument("--cart-speed", type=float, default=20.0,
                        help="Cartesian speed mm/s for the axis-sequential tray "
                             "legs (default 20, conservative)")
    parser.add_argument("--close-speed", type=int, default=None,
                        help="BIO gripper closing speed (default: from config). "
                             "Lower is gentler; a fast closure can knock the plate "
                             "aside instead of seating on it.")
    parser.add_argument("--no-open", action="store_true", help="do not open the gripper "
                                                               "before grabbing")
    parser.add_argument(
        "--clear-errors",
        action="store_true",
        help=(
            "allow clearing a latched controller fault before connecting. Never "
            "automatic -- a latched error_code may record a previous collision, "
            "and now (via the shared Arm connection) blocks connecting outright "
            "unless this is passed"
        ),
    )
    return parser


def run(args: argparse.Namespace) -> int:
    overrides: dict[str, Any] = {"live": args.execute, "clear_errors": args.clear_errors}
    if args.host:
        overrides["host"] = args.host
    if args.close_speed is not None:
        overrides["close_speed"] = args.close_speed
    settings = ArmSettings.from_config(**overrides)

    store = WorkspaceStore(settings.workspace_path)
    if args.transit_speed is None:
        args.transit_speed = args.cart_speed
    if args.return_to_tray:
        plan = build_plan(store, RETURN_SEQUENCE)
        print("[seq] --return-to-tray: carrying the plate back and releasing it")
    else:
        plan = build_plan(store)
    if args.stop_after_grip:
        plan = plan[:STEPS_BEFORE_HOLDING]
        print("[seq] --stop-after-grip: the pick only; no transit will be attempted")
    if args.stop_at_standoff:
        plan = [i for i in plan if i.get("name") != "microscope"]
        print("[seq] --stop-at-standoff: stopping beside the scope; the plate will "
              "NOT go under the objective")
    if args.return_to_tray and (args.slide_in or args.stop_after_grip
                                or args.already_holding or args.stop_at_standoff):
        raise SystemExit("[seq] --return-to-tray is the whole reverse route; it does "
                         "not combine with the outbound resume flags.")
    if args.slide_in:
        plan = [i for i in plan if i.get("name") == "microscope"]
        print("[seq] --slide-in: the final Y slide only; the plate must already "
              "be staged at scope_standoff")
    if args.already_holding:
        skipped, plan = plan[:STEPS_BEFORE_HOLDING], plan[STEPS_BEFORE_HOLDING:]
        print("[seq] --already-holding: skipping %d step(s) -- %s"
              % (len(skipped), ", ".join(i["action"] for i in skipped)))
    print_plan(plan, settings)

    if not args.execute:
        print("\n[seq] DRY-RUN (no --execute): nothing was moved.")
        return 0

    # This crosses the whole workspace (home -> floor -> microscope), so --
    # like plate_imaging.py -- it uses Arm.free_envelope() rather than the
    # default anchored A1 box. Every move below is joint-space (move_joints()
    # does not consult the envelope at all), so this is built for
    # consistency/defensive completeness rather than an active guard here.
    # The Z corridor is derived from the three taught locations' own poses.
    # Crosses the whole workspace, so an unanchored envelope with a Z corridor
    # spanning the taught locations. Unlike before, this is now an ACTIVE guard:
    # the tray legs are Cartesian moves and every one of them is validated
    # against it. The joint legs still consult nothing -- joint space has no
    # Cartesian envelope, which is why they use taught angles only.
    known_zs = [store.require_location(n).pose.z for n in TAUGHT]
    z_floor = min(known_zs) - Z_CORRIDOR_MARGIN_MM
    z_ceiling = max(known_zs) + Z_CORRIDOR_MARGIN_MM
    robot = Arm(settings, log=lambda msg: print(msg, flush=True)).free_envelope(
        reason="pick sequence carries the plate home -> floor -> home -> "
               "microscope; the tray legs are axis-sequential Cartesian, the "
               "rest joint-space on taught angles",
        z_floor_mm=z_floor,
        z_ceiling_mm=z_ceiling,
    )
    print(f"[seq] transit Z corridor {z_floor:.4f} .. {z_ceiling:.4f} "
          f"(taught span +/- {Z_CORRIDOR_MARGIN_MM} mm settling margin)")

    print(f"[seq] connecting to {settings.host} ...", flush=True)
    try:
        robot.connection.connect(arm=True)
    except ArmError as exc:
        raise SystemExit(f"[seq] ABORT: {exc}") from exc
    status = robot.connection.status()
    print(
        f"[seq] motion enabled: mode={status.get('mode')} "
        f"state={status.get('state')} err={status.get('error_code')}",
        flush=True,
    )

    if args.slide_in:
        want = store.require_location("scope_standoff").pose
        actual = robot.pose()
        gaps = {"x": abs(actual[0] - float(want.x)), "z": abs(actual[2] - float(want.z))}
        yaw_gap = abs((actual[5] - float(want.yaw) + 180.0) % 360.0 - 180.0)
        bad = [f"{k}={v:.3f} mm" for k, v in gaps.items() if v > SLIDE_IN_TOL_MM]
        if yaw_gap > SLIDE_IN_TOL_DEG:
            bad.append(f"yaw={yaw_gap:.3f} deg")
        print(f"[seq] slide-in precondition: x/z/yaw off by "
              f"{gaps['x']:.3f} mm / {gaps['z']:.3f} mm / {yaw_gap:.3f} deg")
        if bad:
            robot.close()
            raise SystemExit(
                "[seq] ABORT: the arm is not staged at scope_standoff (" +
                ", ".join(bad) + "). A pure Y slide is only safe from there; from "
                "anywhere else it is an unbounded traverse ending under the "
                "objective. Re-run the transit rather than sliding from here.")

    try:
        for item in plan:
            act = item["action"]
            if act == "open":
                if args.no_open:
                    print("[seq] gripper OPEN skipped (--no-open)")
                    continue
                print(f"[seq] gripper OPEN (bio)  speed={settings.open_speed} ...",
                      flush=True)
                result = robot.release()
                print(f"[seq] gripper OPEN done: bio_status={result.get('bio_status')}")

            elif act == "grip":
                print(f"[seq] gripper CLOSE (bio, GRAB)  "
                      f"speed={settings.close_speed} ...", flush=True)
                result = robot.grip()
                print(f"[seq] gripper CLOSE done: bio_status={result.get('bio_status')} "
                      f"opening={result.get('opening_mm')} mm")
                # THE CHECK THAT WAS MISSING. On 2026-08-11 the jaws closed on
                # empty air (71.0 mm, their hard minimum) and the run lifted,
                # rotated and crossed the bench with nothing in the gripper.
                # Refuse to carry what we cannot prove we are holding.
                bottomed = result.get("bottomed_out")
                if bottomed is None:
                    raise SystemExit(
                        "[seq] ABORT: could not read the jaw position after closing, "
                        "so whether the plate is held is UNKNOWN. Refusing to lift -- "
                        "an unverified grip is exactly how a plate gets carried off "
                        "the bench and dropped.")
                if bottomed:
                    raise SystemExit(
                        f"[seq] ABORT: the jaws closed to {result.get('opening_mm')} mm, "
                        f"their hard minimum -- NOTHING IS HELD. A good grip on this "
                        f"plate reads about 125 mm. Nothing has been lifted. Check the "
                        f"tray seating and the taught `floor` before retrying.")

            elif act == "joint":
                name, angles = item["name"], item["angles"]
                print(f"[seq] joint -> {name} "
                      f"{['%.3f' % a for a in angles]} @ {args.speed} deg/s ...",
                      flush=True)
                res = robot.move_joints(angles, speed_deg_s=args.speed, label=name)
                arrival = res.get("arrival") or {}
                print(f"[seq] arrived {name}  "
                      f"(worst joint gap {arrival.get('worst_gap_deg')} deg)")

            elif act == "axis":
                name = item["name"]
                x, y, z = item["xyz"]
                mm_s = speed_for(item, args)
                print(f"[seq] axis  -> {name} x={x:.3f} y={y:.3f} z={z:.3f} "
                      f"@ {mm_s} mm/s ({item.get('speed', 'fine')}), "
                      f"order {item['order']} ...", flush=True)
                legs = robot.move_axiswise(x, y, z, speed=mm_s,
                                           label=f"to {name}", order=item["order"])
                print(f"[seq] arrived {name}  ({len(legs)} axis leg(s))")

            elif act == "rotate":
                name = item["name"]
                roll, pitch, yaw = item["rpy"]
                mm_s = speed_for(item, args)
                print(f"[seq] rotate -> {name} yaw={yaw:.3f} "
                      f"@ {mm_s} mm/s, XYZ held ...", flush=True)
                legs = robot.rotate_in_place(roll, pitch, yaw,
                                             speed=mm_s, label=name)
                print(f"[seq] rotated {name}  ({len(legs)} step(s))")

        if args.stop_at_standoff:
            print("[seq] STAGED AT STANDOFF -- the plate is held 150 mm short of the "
                  "wells in Y, at the working height. Confirm clearance before the "
                  "slide in.")
        elif args.stop_after_grip:
            print("[seq] PICK COMPLETE -- the plate is held at the tray and the arm "
                  "is stopped. Confirm by eye before the transit is run.")
        elif args.return_to_tray:
            print("[seq] RETURNED -- the plate is back in the tray and released.")
        elif args.slide_in:
            print("[seq] SLIDE-IN COMPLETE -- the plate is under the objective at the "
                  "taught A2.")
        else:
            print("[seq] sequence complete.")
    except (ArmError, SafetyError) as exc:
        raise SystemExit(f"[seq] ABORT: {exc}") from exc
    finally:
        robot.close()
        print("[seq] disconnected.")

    return 0


def main(argv: list[str] | None = None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
