#!/usr/bin/env python3
"""goto_a1.py -- carry the held plate between the park pose and well A1, leg by leg.

    python3 scripts/microscope/goto_a1.py                    # print the plan, move nothing
    python3 scripts/microscope/goto_a1.py --all --execute    # run the whole route
    python3 scripts/microscope/goto_a1.py --leg 3 --execute  # or one leg at a time
    python3 scripts/microscope/goto_a1.py --to home --all --execute      # and back

THE ROUTE
---------
Driven live end to end on 2026-08-04 and amended by the operator watching it
run. Six legs out, six mirrored back::

    1  Z-lift      Z only    rise to the WORKING height, in place
    2  wrist       RPY only  square to A1's orientation, in place
    3  Y-advance   Y only    cross to 150 mm short of the standoff plane
    4  X-align     X only    swing across to A1's column  <-- the dogleg
    5  Y-close     Y only    close the last 150 mm to the standoff
    6  Y-push-in   Y only    slide 150 mm in to A1   *** UNDER THE LENS ***

THE WORKING HEIGHT (operator, 2026-08-04)
-----------------------------------------
``WORKING_RISE_MM`` is **3 mm above the taught A1 pose, and the plate stays
there** -- including under the objective. It is not a transit clearance that
gets spent before the lens; an earlier version read it that way and added a
Z-settle leg to drop back to the taught height at the standoff. The operator
corrected that explicitly: *"the position right underneath the microscope
should be pushed upward three millimetres, you're not supposed to pull down
when you're in front of the microscope."* So there is no descent, and the
taught A1's Z is a reference the working height is measured from, not a
destination.

This raises the plate 3 mm **toward** a fixed objective, which is the direction
the whole safety layer exists to be careful about. Two things bound it:

* ``check_rise`` refuses anything past ``Z_MAX_RISE_MM`` (10 mm), the
  operator's standing hard limit on how far the plate may ever rise above
  taught A1. 3 mm spends roughly a third of that budget.
* The lens-clearance invariant is checked against the **working** A1, not the
  taught one: within ``LENS_XY_RADIUS_MM`` (20 mm) of A1's XY, Z must equal the
  working height exactly. So the plate may not arrive near the objective at any
  *other* height -- including the taught one.

The 10 mm plate-to-objective clearance this all sits inside is an operator
assertion; nothing on the rig measures it. Re-read that before raising the
working height further.

WHY THE DOGLEG (operator, 2026-08-04)
-------------------------------------
The first version ran Y all the way to the standoff plane and only then moved
X. That put the plate directly **over the UV-Vis spectrometer** on the way
past. Breaking the Y run short, swinging X across there, and only then closing
the gap keeps the plate clear of the reader. The backoff was 100 mm on the
first live run and the operator moved it 50 mm further out after watching it.

Note the sign trap in :func:`dogleg_y`: "short of the standoff" is from the
*park pose's* point of view, so the swing sits FURTHER from A1 than the
standoff, not nearer.

ONE LEG PER COMMAND, AND THE ORDER IS CHECKED
---------------------------------------------
The plan is rebuilt from the arm's *freshly read* pose on every invocation, and
a leg runs only when every axis it does **not** change already reads where the
route says it should. An arm jogged by hand, or a leg that never finished,
refuses the next leg rather than continuing from an assumed position.

Stating that per-axis rather than as "every earlier leg completed" is the
difference between recoverable and stuck. Abort the push-in halfway and X, Z
and orientation are all still correct with only Y partway, so the push-in stays
runnable and finishes the slide. The earlier-legs phrasing refused exactly that
case -- leaving the plate under the objective with no way forward, which is the
one state this route must never be able to reach.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


import argparse  # noqa: E402
import math  # noqa: E402

from tools.arm import Arm  # noqa: E402
from tools.arm.approach import (  # noqa: E402
    LENS_XY_RADIUS_MM,
    Y_APPROACH_SIGN,
    Y_STANDOFF_MM,
    RouteError,
    validate_route,
)
from tools.arm.driver import ArmError, XArmConnection  # noqa: E402
from tools.arm.safety import AXES, Z_MAX_RISE_MM, Envelope, SafetyError  # noqa: E402

ANCHOR = "microscope"
PARK = "home"
ARM_HOST_DEFAULT = "192.168.1.201"

#: Working height above the taught A1 pose. Operator, 2026-08-04. The plate
#: rises to this and STAYS there, under the objective included -- see the
#: module docstring. Hard-capped at Z_MAX_RISE_MM (10.0).
WORKING_RISE_MM = 3.0

#: How far short of the standoff plane the Y run stops so the X swing happens
#: clear of the UV-Vis. Operator: 100 mm on the first run, then further back
#: after watching it -- capped by Y_TRAVERSE_FLOOR_MM below, which the arm
#: itself decided.
DOGLEG_BACKOFF_MM = 110.0

#: **Measured, 2026-08-04.** The lowest Y the arm will actually traverse to at
#: A1's column, with the plate held and the wrist squared.
#:
#: Commanding y=290.625 at x=26.029, z=177.246 stopped the controller dead at
#: **y=319.855** -- ``wait_feedback, xarm is stop, state=4``, ``code=-9``, and
#: ``error_code=0``, so no latched fault, just a refusal. y=340.625 has run
#: clean in both directions several times.
#:
#: Inverse kinematics is NOT a usable predictor of this and must not be
#: substituted for it: probed on a 10 mm grid at the same X and Z, IK returned
#: a solution for y=290 and y=260 but *not* for y=280 or y=250. It answers
#: "does a joint solution exist for this single pose", while what stops the arm
#: is a straight-line Cartesian path holding one orientation the whole way.
#:
#: 330.0 is 319.855 plus roughly 10 mm of margin. Do not lower it without
#: re-measuring, and re-measure by walking DOWN in small steps from a known
#: good Y, not by commanding a long move at the unknown one.
Y_TRAVERSE_FLOOR_MM = 330.0

#: Speeds. The route has been driven end to end in both directions and the
#: operator declared it verified, so the transit legs no longer crawl. Still at
#: or under SPEED_MAX_MM_S (30) rather than the 100 mm/s transit ceiling.
TRANSIT_SPEED_MM_S = 30.0
#: The leg that ends under the objective. Deliberately far slower.
PUSH_IN_SPEED_MM_S = 5.0
#: Lifting to the working height and setting back down at the park pose.
SETTLE_SPEED_MM_S = 5.0

#: Largest orientation change in a single command. A pure rotation has no
#: Cartesian displacement, so ``set_position``'s ``speed`` -- a linear mm/s --
#: does not pace it. Bounding the excursion per command is what can be checked
#: before it is sent.
MAX_ROT_STEP_DEG = 10.0

#: Slack below the lowest pose on the route, so the corridor admits it.
Z_FLOOR_MARGIN_MM = 1.0

#: A leg counts as already done when the machine reads this close on the axes
#: it changes. Wider than READBACK_SLACK_MM (0.5) because it compares across
#: two separate settles rather than checking one.
DONE_TOL_MM = 1.0
DONE_TOL_DEG = 1.0

AXIS_INDEX = {name: i for i, name in enumerate(AXES)}
_ANGLE_AXES = ("roll", "pitch", "yaw")


def _fmt(pose) -> str:
    return ("x=%9.3f y=%9.3f z=%9.3f  roll=%8.3f pitch=%8.3f yaw=%8.3f"
            % tuple(float(v) for v in pose))


def _leg(index, label, changes, pose, speed, why):
    return {"index": index, "label": label, "changes": tuple(changes),
            "pose": [float(v) for v in pose], "speed_mm_s": float(speed), "why": why}


def check_rise(rise_mm: float) -> float:
    """Refuse a working rise past the operator's hard limit.

    Raising Z drives the plate toward the objective, so this is the one number
    in the file that is not the caller's to choose freely.
    """
    rise = float(rise_mm)
    if not math.isfinite(rise) or rise < 0.0:
        raise SafetyError("working rise must be a finite non-negative number, got %r"
                          % (rise_mm,))
    if rise > Z_MAX_RISE_MM:
        raise SafetyError(
            "working rise %.3f mm is above the operator's hard limit of %.1f mm over "
            "taught A1. Raising Z drives the plate toward the objective."
            % (rise, Z_MAX_RISE_MM))
    return rise


def working_pose(a1, rise_mm=WORKING_RISE_MM):
    """The taught A1 lifted by the working rise. This is where the plate sits."""
    pose = [float(v) for v in a1]
    pose[2] += check_rise(rise_mm)
    return pose


def standoff_y(a1) -> float:
    """A1's Y displaced by the standoff distance, on the scope's open side."""
    return float(a1[1]) + Y_APPROACH_SIGN * Y_STANDOFF_MM


def dogleg_y(a1, backoff_mm=None) -> float:
    """Where the X swing happens: ``backoff_mm`` FURTHER from A1 than the standoff.

    Derived from A1 rather than from the standoff, because the sign is the trap.
    ``Y_APPROACH_SIGN`` is -1 here (the standoff sits at lower Y), so "short of
    the standoff, coming from the park pose" means one more step in the *same*
    direction -- ``a1 + sign * (150 + backoff)``. Writing it as
    ``standoff - sign * backoff`` reads correctly in English and puts the swing
    on the far side, nearer the lens instead of further from it.
    """
    backoff = DOGLEG_BACKOFF_MM if backoff_mm is None else float(backoff_mm)
    if not math.isfinite(backoff) or backoff < 0.0:
        raise SafetyError("dogleg backoff must be finite and non-negative, got %r"
                          % (backoff_mm,))
    y = float(a1[1]) + Y_APPROACH_SIGN * (Y_STANDOFF_MM + backoff)
    if y < Y_TRAVERSE_FLOOR_MM:
        raise SafetyError(
            "dogleg backoff %.1f mm puts the X swing at y=%.3f, below the measured "
            "traverse floor of %.1f mm. The arm stopped itself at y=319.855 the one "
            "time it was asked to go past this (state=4, code=-9, no latched error). "
            "The largest backoff this A1 allows is %.1f mm."
            % (backoff, y, Y_TRAVERSE_FLOOR_MM,
               float(a1[1]) - Y_STANDOFF_MM - Y_TRAVERSE_FLOOR_MM))
    return y


def legs_to_a1(start, a1, *, rise_mm=WORKING_RISE_MM, backoff_mm=DOGLEG_BACKOFF_MM,
               transit_speed=TRANSIT_SPEED_MM_S, push_speed=PUSH_IN_SPEED_MM_S,
               settle_speed=SETTLE_SPEED_MM_S):
    """The six legs from wherever the arm is to the working A1. Commands nothing.

    Each target is a function of A1 plus whichever start coordinates the earlier
    legs do not touch, so feeding this a pose partway along the route returns
    the remaining legs unchanged.
    """
    work = working_pose(a1, rise_mm)
    sx, sy = float(start[0]), float(start[1])
    wx, wy, wz = work[0], work[1], work[2]
    rpy = [work[3], work[4], work[5]]
    stand = standoff_y(a1)
    dogleg = dogleg_y(a1, backoff_mm)
    rise = wz - float(a1[2])

    return [
        _leg(1, "1/6 Z-lift in place to the working height (A1+%.1f mm)" % rise, ("z",),
             [sx, sy, wz, start[3], start[4], start[5]], settle_speed,
             "gain the working height here, in the open, before travelling"),
        _leg(2, "2/6 square the wrist to A1 orientation", ("roll", "pitch", "yaw"),
             [sx, sy, wz] + rpy, transit_speed,
             "rotating the wrist is safe here and must never happen under the lens"),
        _leg(3, "3/6 Y-advance to %.0f mm short of the standoff" % backoff_mm, ("y",),
             [sx, dogleg, wz] + rpy, transit_speed,
             "stop short: running Y to the standoff here passes over the UV-Vis"),
        _leg(4, "4/6 X-align to A1 column  <-- dogleg, clear of the UV-Vis", ("x",),
             [wx, dogleg, wz] + rpy, transit_speed,
             "swing across at the near Y, where the reader is not underneath"),
        _leg(5, "5/6 Y-close to the standoff plane", ("y",),
             [wx, stand, wz] + rpy, transit_speed,
             "close up now that X is already right"),
        _leg(6, "6/6 Y-push-in to A1   *** UNDER THE LENS ***", ("y",),
             [wx, wy, wz] + rpy, push_speed,
             "the only leg near the objective: pure Y at the working height"),
    ]


def legs_to_home(start, a1, home, *, rise_mm=WORKING_RISE_MM,
                 backoff_mm=DOGLEG_BACKOFF_MM, transit_speed=TRANSIT_SPEED_MM_S,
                 push_speed=PUSH_IN_SPEED_MM_S, settle_speed=SETTLE_SPEED_MM_S):
    """The route back, mirrored. Leg 1 is the pure-Y pull-out and must stay first.

    Nothing else may move until the plate is clear of the objective -- that is
    the whole reason the standoff exists, and it is why the wrist is not
    un-squared and Z is not dropped until the last two legs.
    """
    check_rise(rise_mm)
    hx, hy, hz = float(home[0]), float(home[1]), float(home[2])
    hrpy = [float(home[3]), float(home[4]), float(home[5])]
    stand = standoff_y(a1)
    dogleg = dogleg_y(a1, backoff_mm)

    # The axes the pull-out does not touch are held at the arm's OWN reading,
    # not at a taught value. "Changes Y only" has to mean literally that: the
    # controller settles a few nanometres off the commanded pose, and pinning X
    # to the taught number made leg 1 a Y move that also nudged X by 9e-6 mm --
    # which validate_route correctly refused, at 1e-6 mm, every single time.
    sx, sz = float(start[0]), float(start[2])
    srpy = [float(start[3]), float(start[4]), float(start[5])]

    return [
        _leg(1, "1/6 Y-pull-out from A1 to the standoff", ("y",),
             [sx, stand, sz] + srpy, push_speed,
             "pure Y at the height it is already at -- nothing else moves until clear"),
        _leg(2, "2/6 Y-back to %.0f mm short of the standoff" % backoff_mm, ("y",),
             [sx, dogleg, sz] + srpy, transit_speed,
             "keep retreating along Y before anything else moves"),
        _leg(3, "3/6 X-back toward the park column", ("x",),
             [hx, dogleg, sz] + srpy, transit_speed,
             "swing across at the near Y, clear of the UV-Vis"),
        _leg(4, "4/6 Y-back to the park plane", ("y",),
             [hx, hy, sz] + srpy, transit_speed,
             "cross back at the working height"),
        _leg(5, "5/6 un-square the wrist to the park orientation",
             ("roll", "pitch", "yaw"), [hx, hy, sz] + hrpy, transit_speed,
             "rotate only once the plate is back over open bench"),
        _leg(6, "6/6 Z-down to the park height", ("z",),
             [hx, hy, hz] + hrpy, settle_speed,
             "set down last, straight down, in place"),
    ]


def leg_satisfied(leg, actual) -> bool:
    """Does the machine already read at this leg's target, on the axes it changes?"""
    for name in leg["changes"]:
        i = AXIS_INDEX[name]
        tol = DONE_TOL_DEG if name in _ANGLE_AXES else DONE_TOL_MM
        if abs(float(actual[i]) - float(leg["pose"][i])) > tol:
            return False
    return True


def precondition_failures(leg, actual):
    """Which axes are wrong for this leg to start. Empty list means go.

    A leg's precondition is exactly: **every axis this leg does not change is
    already where the route says it should be.** The axes it *does* change may
    read anywhere -- driving them to the target is the leg's whole job, and the
    plan was rebuilt from this very reading, so it starts from where they are.
    """
    changing = set(leg["changes"])
    out = []
    for i, name in enumerate(AXES):
        if name in changing:
            continue
        tol = DONE_TOL_DEG if name in _ANGLE_AXES else DONE_TOL_MM
        want, got = float(leg["pose"][i]), float(actual[i])
        if abs(got - want) > tol:
            out.append((name, want, got))
    return out


def check_declared_axes(leg, previous) -> None:
    """G1 by hand: a leg must move only the axes it declares."""
    declared = set(leg["changes"])
    for i, name in enumerate(AXES):
        delta = float(leg["pose"][i]) - float(previous[i])
        tol = DONE_TOL_DEG if name in _ANGLE_AXES else DONE_TOL_MM
        if name not in declared and abs(delta) > tol:
            raise SafetyError(
                "leg %d (%s) would change %s by %+.4f but declares only %s"
                % (leg["index"], leg["label"], name, delta, sorted(declared)))


def check_lens_clearance(legs, a1, start, rise_mm=WORKING_RISE_MM) -> None:
    """No waypoint may sit near A1's XY at anything but the WORKING height.

    Checked against the working pose rather than the taught one, because the
    working height is where the plate belongs now. A leg that arrived at the
    taught height would be just as wrong as one that arrived 3 mm high, and this
    refuses both.

    Delegated to :func:`tools.arm.approach.validate_route` on purpose -- that
    module owns the invariant, and a second copy here would be a second thing to
    keep in step. ``start`` is threaded in rather than substituting the first
    waypoint: doing that makes leg 1's own declared-axis check compare the pose
    to itself and pass no matter what it declares.
    """
    work = working_pose(a1, rise_mm)
    route = [{"pose": leg["pose"], "changes": list(leg["changes"]),
              "label": leg["label"], "speed_mm_s": leg["speed_mm_s"]} for leg in legs]
    ends_at_work = all(abs(route[-1]["pose"][i] - work[i]) <= 1e-6 for i in range(6))
    validate_route([float(v) for v in start], route, work,
                   expect_final_at_a1=ends_at_work, lens_radius_mm=LENS_XY_RADIUS_MM)


def substeps(leg, start):
    """Split a leg into the poses actually commanded.

    Only orientation legs split. A translation is paced by ``set_position``'s
    ``speed`` argument, which is a linear mm/s; a pure rotation has no linear
    displacement for that to act on, so it is bounded by size instead.

    **Only the declared axes interpolate.** Every other axis is pinned to the
    leg's target, so a rotation leg cannot translate no matter what pose it is
    handed. Interpolating all six looked equivalent -- the precondition already
    requires the untouched axes to be right -- but "right" there means within
    1 mm, so a leg that settled a hair off turned the wrist square into nine
    little Z creeps. Small, and still the axis this route refuses to be
    careless about.
    """
    target = [float(v) for v in leg["pose"]]
    changing = [a for a in leg["changes"] if a in _ANGLE_AXES]
    if not changing:
        return [target]

    start = [float(v) for v in start]
    biggest = max(abs(target[AXIS_INDEX[a]] - start[AXIS_INDEX[a]]) for a in changing)
    n = max(1, int(math.ceil(biggest / MAX_ROT_STEP_DEG)))
    out = []
    for k in range(1, n + 1):
        f = k / float(n)
        pose = list(target)
        for name in leg["changes"]:
            i = AXIS_INDEX[name]
            pose[i] = start[i] + (target[i] - start[i]) * f
        out.append(pose)
    return out


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--to", choices=("a1", "home"), default="a1",
                    help="destination (default: %(default)s)")
    ap.add_argument("--leg", type=int, default=None,
                    help="execute exactly this leg (1-6)")
    ap.add_argument("--all", action="store_true",
                    help="execute every remaining leg in order, stopping on the first "
                         "refusal. Each leg is still its own validated command.")
    ap.add_argument("--execute", action="store_true",
                    help="connect and move (default: dry-run, commands nothing)")
    ap.add_argument("--host", default=ARM_HOST_DEFAULT)
    ap.add_argument("--rise", type=float, default=WORKING_RISE_MM,
                    help="working height above taught A1, mm (default: %(default)s, "
                         "hard limit " + ("%.1f" % Z_MAX_RISE_MM) + ")")
    ap.add_argument("--backoff", type=float, default=DOGLEG_BACKOFF_MM,
                    help="how far short of the standoff the X swing happens, mm "
                         "(default: %(default)s)")
    ap.add_argument("--transit-speed", type=float, default=TRANSIT_SPEED_MM_S,
                    help="mm/s for the transit legs (default: %(default)s)")
    ap.add_argument("--push-in-speed", type=float, default=PUSH_IN_SPEED_MM_S,
                    help="mm/s for the leg under the lens (default: %(default)s)")
    return ap


def _read_pose(arm, host, live):
    if live:
        return arm.pose(), "read live from the controller"
    # A live=False session has nothing to read, and a dry-run still has to plan
    # against where the arm really is. read_pose() goes through _open(arm=False),
    # so it never calls motion_enable/set_mode/set_state.
    probe = XArmConnection(host, live=True, clear_errors=False, log=lambda _m: None)
    try:
        return probe.read_pose(), "read from the controller (read-only probe)"
    except ArmError as exc:
        raise SystemExit("could not read the arm's pose to build the plan: %s\n"
                         "Check `python -m tools.arm controller`." % exc) from None
    finally:
        probe.disconnect()


def run(args: argparse.Namespace) -> int:
    if args.leg is not None and args.all:
        raise SystemExit("--leg and --all are mutually exclusive")
    if args.execute and args.leg is None and not args.all:
        raise SystemExit("--execute needs --leg N or --all")

    arm = Arm.from_config(live=args.execute, host=args.host, clear_errors=False,
                          anchor=ANCHOR, speed_mm_s=args.transit_speed)
    a1 = arm.location_pose(ANCHOR)
    start, origin = _read_pose(arm, args.host, args.execute)

    kw = dict(rise_mm=args.rise, backoff_mm=args.backoff,
              transit_speed=args.transit_speed, push_speed=args.push_in_speed)
    if args.to == "home":
        home = arm.location_pose(PARK)
        legs = legs_to_home(start, a1, home, **kw)
        destination = "taught '%s'" % PARK
    else:
        home = None
        legs = legs_to_a1(start, a1, **kw)
        destination = "well A1 (taught '%s') at the working height" % ANCHOR

    work = working_pose(a1, args.rise)
    try:
        check_lens_clearance(legs, a1, start, args.rise)
    except RouteError as exc:
        raise SystemExit("route refused: %s" % exc) from None

    lowest = [float(start[2]), float(a1[2])] + ([float(home[2])] if home else [])
    z_floor = min(lowest) - Z_FLOOR_MARGIN_MM
    z_ceiling = work[2]
    envelope = Envelope.free(
        reason=("goto_a1 --to %s: carrying the held plate at the working height "
                "A1+%.1f mm" % (args.to, work[2] - a1[2])),
        z_floor_mm=z_floor, z_ceiling_mm=z_ceiling,
        speed_max_mm_s=max(args.transit_speed, args.push_in_speed, SETTLE_SPEED_MM_S),
    )
    moving = arm.with_envelope(envelope)

    print("goto_a1 --to %s   (%s)" % (args.to, destination))
    print("  start        %s   (%s)" % (_fmt(start), origin))
    print("  taught A1    %s" % _fmt(a1))
    print("  WORKING A1   %s   <- taught +%.1f mm, plate stays here"
          % (_fmt(work), work[2] - a1[2]))
    print("  corridor     z in [%.3f, %.3f] mm" % (z_floor, z_ceiling))
    print("  within %.0f mm of A1 XY, z must equal the working %.3f -- checked"
          % (LENS_XY_RADIUS_MM, work[2]))
    print()

    previous, total_mm = [float(v) for v in start], 0.0
    for leg in legs:
        check_declared_axes(leg, previous)
        d = [leg["pose"][i] - previous[i] for i in range(3)]
        dist = math.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2)
        total_mm += dist
        steps = substeps(leg, previous)
        detail = ("%6.1f mm  %4.0f s @ %.0f mm/s"
                  % (dist, dist / leg["speed_mm_s"], leg["speed_mm_s"])
                  if dist > 1e-6 else
                  "rotation only, %d steps of <=%.0f deg" % (len(steps), MAX_ROT_STEP_DEG))
        print("  [%s] %s" % ("done" if leg_satisfied(leg, start) else "    ", leg["label"]))
        print("         %s" % _fmt(leg["pose"]))
        print("         changes %-16s %s" % (", ".join(leg["changes"]), detail))
        print("         %s" % leg["why"])
        previous = [float(v) for v in leg["pose"]]
    print()
    print("  total path %.1f mm" % total_mm)

    if args.leg is None and not args.all:
        print()
        print("  plan only -- nothing was commanded.")
        print("      python3 scripts/microscope/goto_a1.py --to %s --all --execute"
              % args.to)
        return 0

    wanted = list(range(1, len(legs) + 1)) if args.all else [args.leg]
    if any(not (1 <= n <= len(legs)) for n in wanted):
        raise SystemExit("--leg must be 1..%d" % len(legs))

    for number in wanted:
        rc = _one_leg(arm, moving, legs, number, args, a1, work)
        if rc != 0:
            return rc
        # Re-read: the next leg's precondition must be checked against the
        # machine, not against what the previous leg was asked for.
        start, _ = _read_pose(arm, args.host, args.execute)
        legs = (legs_to_home(start, a1, home, **kw) if args.to == "home"
                else legs_to_a1(start, a1, **kw))
    return 0


def _one_leg(arm, moving, legs, number, args, a1, work) -> int:
    leg = legs[number - 1]
    here, _ = _read_pose(arm, args.host, args.execute)

    failures = precondition_failures(leg, here)
    if failures:
        print()
        print("REFUSED: leg %d cannot start from here." % leg["index"])
        print("  It moves %s, so every other axis must already be on the route:"
              % ", ".join(leg["changes"]))
        for name, want, got in failures:
            print("      %-5s wants %10.3f   arm reads %10.3f   (off by %+.3f)"
                  % (name, want, got, got - want))
        earlier = [ln for ln in legs[: number - 1] if not leg_satisfied(ln, here)]
        if earlier:
            print("  Run leg %d first." % earlier[0]["index"])
        else:
            print("  No earlier leg explains this -- the arm may have been moved by hand.")
        return 2

    if leg_satisfied(leg, here):
        print("  [skip] leg %d already satisfied" % leg["index"])
        return 0

    steps = substeps(leg, here)
    if not args.execute:
        for pose in steps:
            moving.envelope.check_target(pose, what="leg %d sub-step" % leg["index"])
        print("  [dry ] leg %d validated, %d command(s), nothing sent"
              % (leg["index"], len(steps)))
        return 0

    print()
    print(">>> EXECUTING leg %d: %s" % (leg["index"], leg["label"]))
    print("    %d command(s) at %.0f mm/s" % (len(steps), leg["speed_mm_s"]))
    try:
        with moving.occupied("goto_a1 leg %d" % leg["index"]):
            for k, pose in enumerate(steps, start=1):
                label = ("leg %d" % leg["index"] if len(steps) == 1
                         else "leg %d step %d/%d" % (leg["index"], k, len(steps)))
                moving.move_pose(pose, speed=leg["speed_mm_s"], label=label,
                                 takeup=False, verify=True)
    except KeyboardInterrupt:
        arm.connection.halt()
        print("\nINTERRUPTED -- stop commanded. Re-run the same leg to finish it.",
              file=sys.stderr)
        raise SystemExit(130) from None
    except ArmError as exc:
        # The controller refusing a move is a fact worth reporting precisely,
        # not a traceback. It has already stopped itself; say where it actually
        # ended up, because that reading is the only evidence of where the
        # boundary is.
        arm.connection.halt()
        print()
        print("CONTROLLER REFUSED THE MOVE: %s" % exc, file=sys.stderr)
        try:
            stopped = arm.pose()
            print("  commanded  %s" % _fmt(leg["pose"]), file=sys.stderr)
            print("  stopped at %s" % _fmt(stopped), file=sys.stderr)
            print("  short by   dx=%+.3f dy=%+.3f dz=%+.3f"
                  % tuple(stopped[i] - leg["pose"][i] for i in range(3)), file=sys.stderr)
        except Exception as read_exc:
            print("  could not read where it stopped: %r" % (read_exc,), file=sys.stderr)
        print("  The arm is halted and holding. Nothing further was commanded.",
              file=sys.stderr)
        return 3

    achieved = arm.pose()
    print("    achieved %s" % _fmt(achieved))
    print("    residual dx=%+.4f dy=%+.4f dz=%+.4f  droll=%+.4f dpitch=%+.4f dyaw=%+.4f"
          % tuple(achieved[i] - leg["pose"][i] for i in range(6)))

    if args.to == "a1" and leg["index"] == len(legs):
        anchored = Envelope.anchored(
            a1, name=ANCHOR, z_max_rise_mm=arm.settings.z_max_rise_mm,
            xy_max_mm=arm.settings.xy_max_mm, orient_tol_deg=arm.settings.orient_tol_deg)
        try:
            anchored.check_readback(achieved, where="after the push-in to A1")
            print("    at A1 +%.1f mm, inside the +/-%.2f mm box and the %.1f mm rise "
                  "budget. Ready for calibration."
                  % (achieved[2] - a1[2], arm.settings.xy_max_mm,
                     arm.settings.z_max_rise_mm))
        except SafetyError as exc:
            print("    at A1 but OUTSIDE the box: %s" % exc)
            print("    That is a measurement, not a failure -- it is how far the taught")
            print("    A1 is from where the arm settles. Do not re-teach from it until")
            print("    the marker calibration has said where A1 really is.")
    return 0


def main(argv=None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
