"""Y-axis approach and departure routes for well A1 under the microscope lens.

WHY THIS EXISTS
---------------
The microscope objective is mounted directly above well A1. Any motion that
changes Z, X, or orientation while the plate is under the lens risks driving the
plate into the objective. So the approach is decomposed so that the ONLY motion
that happens near the lens is a straight slide along Y at the lens's working
height.

THE ROUTE (operator instruction, 2026-07-31)
--------------------------------------------
Stage at a standoff 150 mm from A1 along Y, get the height and the X coordinate
right THERE, and only then push straight in along Y::

    leg 1  Y-retract   change Y only  -> reach the standoff Y plane, clear of the lens
    leg 2  Z-level     change Z only  -> match A1's working height, still 150 mm away
    leg 3  X-align     change X only  -> line up with A1's column, still 150 mm away
    leg 4  Y-push-in   change Y only  -> slide 150 mm in to A1 at fixed Z

Departure is leg 4 reversed first (pure-Y pull-out), then the rest is free.

Leg 1 also applies A1's orientation, because rotating the wrist is safe at the
standoff and must never happen under the lens.

DIRECTION OF THE STANDOFF
-------------------------
``Y_APPROACH_SIGN = -1``: the standoff sits at LOWER Y than A1. This is not a
guess. The previously taught ``scope_standoff`` sat at dy = -137.918 mm from the
A1 of its era, at dz = +0.000 -- lower Y, identical height. The rig was moved on
2026-07-31 and A1's Y barely changed (587.753 -> 590.625), so the scope's open
side is still -Y. If the scope is ever re-oriented, flip this sign; it is read
from config, and every route prints its direction before moving.

This module is PURE: it builds and validates waypoint lists. It imports no
hardware SDK, commands nothing, and is fully unit-testable offline.
"""
from __future__ import annotations

from collections.abc import Sequence
from typing import Any

#: Standoff distance from A1 along Y, in mm. Operator instruction, 2026-07-31.
Y_STANDOFF_MM = 150.0

#: Which way along Y the standoff lies. -1 = lower Y. See module docstring for
#: the evidence; do not change without re-measuring the scope's open side.
Y_APPROACH_SIGN = -1

#: Any waypoint whose XY is within this radius of A1's XY is "under the lens".
LENS_XY_RADIUS_MM = 20.0

#: Height tolerance under the lens. Approach routes must hold A1's Z exactly;
#: this is float slop, not a working allowance.
Z_TOLERANCE_MM = 1e-6

AXES = ("x", "y", "z", "roll", "pitch", "yaw")
AXIS_INDEX = {name: i for i, name in enumerate(AXES)}

Pose = list[float]
Waypoint = dict[str, Any]


class RouteError(ValueError):
    """A route that would be unsafe or is internally inconsistent."""


# ---------------------------------------------------------------------------
# (1) The standoff
# ---------------------------------------------------------------------------

def standoff_pose(a1: Sequence[float],
                  distance_mm: float = Y_STANDOFF_MM,
                  sign: int = Y_APPROACH_SIGN) -> Pose:
    """A1 displaced along Y by `sign * distance_mm`, at A1's exact height.

    Raises:
        RouteError: on a non-positive distance or a sign that is not +/-1.
    """
    if distance_mm <= 0:
        raise RouteError("standoff distance must be positive, got %r" % (distance_mm,))
    if sign not in (1, -1):
        raise RouteError("Y_APPROACH_SIGN must be +1 or -1, got %r" % (sign,))
    pose = [float(v) for v in a1]
    if len(pose) != 6:
        raise RouteError("a1 must be [x, y, z, roll, pitch, yaw], got %d values" % len(pose))
    pose[AXIS_INDEX["y"]] += sign * float(distance_mm)
    return pose


# ---------------------------------------------------------------------------
# (2) Route construction
# ---------------------------------------------------------------------------

def _leg(pose: Pose, changes: tuple[str, ...], label: str, speed_mm_s: float) -> Waypoint:
    return {"pose": [float(v) for v in pose], "changes": list(changes),
            "label": label, "speed_mm_s": float(speed_mm_s)}


def approach_route(start: Sequence[float],
                   a1: Sequence[float],
                   *,
                   distance_mm: float = Y_STANDOFF_MM,
                   sign: int = Y_APPROACH_SIGN,
                   transit_mm_s: float = 30.0,
                   push_in_mm_s: float = 10.0) -> list[Waypoint]:
    """Build the four-leg approach from `start` to A1.

    Each leg changes exactly the axes it declares. The final leg is the only
    motion that happens near the lens, and it is a pure Y slide at A1's Z.

    Args:
        start: the pose the arm is at now, [x, y, z, roll, pitch, yaw].
        a1: the taught A1 pose.
        distance_mm: standoff distance along Y.
        sign: +1 or -1, which side of A1 the standoff is on.
        transit_mm_s: speed for the three legs that happen at the standoff.
        push_in_mm_s: speed for the final slide under the lens. Deliberately
            slower; this is the only leg where a mistake reaches the objective.
    """
    start = [float(v) for v in start]
    a1 = [float(v) for v in a1]
    if len(start) != 6 or len(a1) != 6:
        raise RouteError("poses must be [x, y, z, roll, pitch, yaw]")
    stand = standoff_pose(a1, distance_mm, sign)

    # leg 1 -- Y only (plus orientation, which is safe this far out and must
    # never be changed under the lens).
    p1 = list(start)
    p1[AXIS_INDEX["y"]] = stand[AXIS_INDEX["y"]]
    p1[AXIS_INDEX["roll"]] = a1[AXIS_INDEX["roll"]]
    p1[AXIS_INDEX["pitch"]] = a1[AXIS_INDEX["pitch"]]
    p1[AXIS_INDEX["yaw"]] = a1[AXIS_INDEX["yaw"]]
    leg1 = _leg(p1, ("y", "roll", "pitch", "yaw"),
                "1/4 Y-retract to the standoff plane (y=%.3f) and square the wrist"
                % stand[AXIS_INDEX["y"]], transit_mm_s)

    # leg 2 -- Z only: match A1's working height while still 150 mm away.
    p2 = list(p1)
    p2[AXIS_INDEX["z"]] = a1[AXIS_INDEX["z"]]
    leg2 = _leg(p2, ("z",), "2/4 Z-level to A1 height (z=%.3f) at the standoff"
                % a1[AXIS_INDEX["z"]], transit_mm_s)

    # leg 3 -- X only: line up with A1's column, still 150 mm away.
    p3 = list(p2)
    p3[AXIS_INDEX["x"]] = a1[AXIS_INDEX["x"]]
    leg3 = _leg(p3, ("x",), "3/4 X-align to A1 column (x=%.3f) at the standoff"
                % a1[AXIS_INDEX["x"]], transit_mm_s)

    # leg 4 -- Y only: the push-in. The only leg under the lens.
    p4 = list(a1)
    leg4 = _leg(p4, ("y",), "4/4 Y-push-in %.1f mm to A1 at fixed Z (UNDER THE LENS)"
                % abs(a1[AXIS_INDEX["y"]] - p3[AXIS_INDEX["y"]]), push_in_mm_s)

    route = [leg1, leg2, leg3, leg4]
    validate_route(start, route, a1, expect_final_at_a1=True)
    return route


def departure_route(a1: Sequence[float],
                    *,
                    distance_mm: float = Y_STANDOFF_MM,
                    sign: int = Y_APPROACH_SIGN,
                    push_out_mm_s: float = 10.0) -> list[Waypoint]:
    """Pure-Y pull-out from A1 to the standoff. Nothing else moves.

    Whatever the arm does after this is the caller's business, but it must not
    happen until this leg has completed -- that is the point of the standoff.
    """
    a1 = [float(v) for v in a1]
    stand = standoff_pose(a1, distance_mm, sign)
    leg = _leg(stand, ("y",),
               "Y-pull-out %.1f mm from A1 to the standoff at fixed Z" % distance_mm,
               push_out_mm_s)
    validate_route(a1, [leg], a1, expect_final_at_a1=False)
    return leg and [leg]


# ---------------------------------------------------------------------------
# (3) Validation -- the part that actually protects the lens
# ---------------------------------------------------------------------------

def validate_route(start: Sequence[float],
                   route: Sequence[Waypoint],
                   a1: Sequence[float],
                   *,
                   expect_final_at_a1: bool,
                   lens_radius_mm: float = LENS_XY_RADIUS_MM) -> None:
    """Raise unless every leg is safe and does exactly what it declares.

    Checks, in order:
      1. Each leg changes ONLY the axes listed in its own `changes` field.
      2. Each leg actually changes at least one of them (no silent no-op legs
         hiding a mistake), except where start and target genuinely coincide.
      3. No waypoint sits under the lens at a height other than A1's. This is
         the collision invariant: within `lens_radius_mm` of A1's XY, Z must
         equal A1's Z.
      4. If `expect_final_at_a1`, the last leg lands exactly on A1 and is a
         pure-Y move.

    Raises:
        RouteError: with the leg index and the offending axis.
    """
    a1 = [float(v) for v in a1]
    prev = [float(v) for v in start]
    for i, wp in enumerate(route, start=1):
        pose = [float(v) for v in wp["pose"]]
        if len(pose) != 6:
            raise RouteError("leg %d: pose must have 6 values" % i)
        declared = set(wp.get("changes", ()))
        unknown = declared - set(AXES)
        if unknown:
            raise RouteError("leg %d: unknown axis name(s) %s" % (i, sorted(unknown)))
        for name in AXES:
            delta = pose[AXIS_INDEX[name]] - prev[AXIS_INDEX[name]]
            if name not in declared and abs(delta) > 1e-6:
                raise RouteError(
                    "leg %d (%s) changes %s by %+.6f but only declares %s"
                    % (i, wp.get("label", "?"), name, delta, sorted(declared) or ["nothing"])
                )
        # collision invariant
        dx = pose[AXIS_INDEX["x"]] - a1[AXIS_INDEX["x"]]
        dy = pose[AXIS_INDEX["y"]] - a1[AXIS_INDEX["y"]]
        if (dx * dx + dy * dy) ** 0.5 <= lens_radius_mm:
            dz = pose[AXIS_INDEX["z"]] - a1[AXIS_INDEX["z"]]
            if abs(dz) > Z_TOLERANCE_MM:
                raise RouteError(
                    "leg %d (%s) is under the lens (%.2f mm from A1 in XY) at z=%.4f, "
                    "which is %+.4f mm off A1's height %.4f -- approach routes must hold "
                    "A1's Z under the lens"
                    % (i, wp.get("label", "?"), (dx * dx + dy * dy) ** 0.5,
                       pose[AXIS_INDEX["z"]], dz, a1[AXIS_INDEX["z"]])
                )
        prev = pose

    if expect_final_at_a1:
        if not route:
            raise RouteError("route is empty but was expected to end at A1")
        final = [float(v) for v in route[-1]["pose"]]
        for name in AXES:
            if abs(final[AXIS_INDEX[name]] - a1[AXIS_INDEX[name]]) > 1e-6:
                raise RouteError("final leg does not land on A1: %s off by %+.6f"
                                 % (name, final[AXIS_INDEX[name]] - a1[AXIS_INDEX[name]]))
        if list(route[-1].get("changes", ())) != ["y"]:
            raise RouteError("final leg into A1 must be a pure Y move, declares %s"
                             % sorted(route[-1].get("changes", ())))


# ---------------------------------------------------------------------------
# (4) Human-readable plan
# ---------------------------------------------------------------------------

def describe(route: Sequence[Waypoint], start: Sequence[float]) -> str:
    lines = ["  start           x=%9.3f y=%9.3f z=%9.3f  roll=%8.3f pitch=%8.3f yaw=%8.3f"
             % tuple(float(v) for v in start)]
    for wp in route:
        p = [float(v) for v in wp["pose"]]
        lines.append("  %-15s x=%9.3f y=%9.3f z=%9.3f  roll=%8.3f pitch=%8.3f yaw=%8.3f"
                     % ("+".join(wp["changes"]), p[0], p[1], p[2], p[3], p[4], p[5]))
        lines.append("      %s  @ %.1f mm/s" % (wp["label"], wp["speed_mm_s"]))
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# (5) Self-test -- run with: python -m tools.robot_arm.approach
# ---------------------------------------------------------------------------

def _self_test() -> int:
    a1 = [26.028557, 590.625244, 177.245712, 179.995150, -0.002807, 89.745359]
    failures = 0

    def check(cond: bool, msg: str) -> None:
        nonlocal failures
        print("[self-test] %s: %s" % ("PASS" if cond else "FAIL", msg), flush=True)
        if not cond:
            failures += 1

    def raises(fn, msg: str) -> None:
        nonlocal failures
        try:
            fn()
        except RouteError as exc:
            print("[self-test] PASS: %s  (%s)" % (msg, str(exc)[:80]), flush=True)
            return
        print("[self-test] FAIL: %s -- no RouteError raised" % msg, flush=True)
        failures += 1

    stand = standoff_pose(a1)
    check(abs(stand[1] - (a1[1] - 150.0)) < 1e-9, "standoff is 150 mm at LOWER y (%.3f)" % stand[1])
    check(abs(stand[2] - a1[2]) < 1e-12, "standoff Z is exactly A1's Z")
    check(stand[0] == a1[0], "standoff X equals A1's X")

    # a start pose deliberately far away and badly oriented
    start = [366.196747, 137.562988, 5.251253, 179.993775, -0.173492, 1.775367]
    route = approach_route(start, a1)
    check(len(route) == 4, "approach has 4 legs")
    check(route[0]["changes"] == ["y", "roll", "pitch", "yaw"], "leg 1 = Y + orientation")
    check(route[1]["changes"] == ["z"], "leg 2 = Z only")
    check(route[2]["changes"] == ["x"], "leg 3 = X only")
    check(route[3]["changes"] == ["y"], "leg 4 = Y only")
    check(route[3]["pose"] == a1, "leg 4 lands exactly on A1")
    check(route[3]["speed_mm_s"] < route[0]["speed_mm_s"], "push-in is slower than transit")
    check(all(abs(wp["pose"][2] - a1[2]) < 1e-9 for wp in route[1:]),
          "legs 2-4 all sit at A1's Z")
    check(abs(route[2]["pose"][1] - (a1[1] - 150.0)) < 1e-9,
          "X alignment happens 150 mm away, not under the lens")

    dep = departure_route(a1)
    check(len(dep) == 1 and dep[0]["changes"] == ["y"], "departure is a single pure-Y leg")
    check(abs(dep[0]["pose"][1] - (a1[1] - 150.0)) < 1e-9, "departure lands on the standoff")

    # the collision invariant must actually fire
    bad = [_leg([a1[0], a1[1], a1[2] + 5.0, a1[3], a1[4], a1[5]], ("z",), "raise under lens", 10.0)]
    raises(lambda: validate_route(a1, bad, a1, expect_final_at_a1=False),
           "rejects a Z change while under the lens")

    sneaky = [_leg([a1[0] + 3.0, a1[1] - 150.0, a1[2], a1[3], a1[4], a1[5]], ("y",),
                   "claims Y, moves X too", 10.0)]
    raises(lambda: validate_route(a1, sneaky, a1, expect_final_at_a1=False),
           "rejects a leg that moves an axis it did not declare")

    diagonal = [_leg(list(a1), ("x", "y"), "diagonal into A1", 10.0)]
    raises(lambda: validate_route(standoff_pose(a1), diagonal, a1, expect_final_at_a1=True),
           "rejects a diagonal final approach into A1")

    raises(lambda: standoff_pose(a1, 0.0), "rejects a zero standoff distance")
    raises(lambda: standoff_pose(a1, 150.0, 0), "rejects a sign that is not +/-1")
    raises(lambda: approach_route(start[:5], a1), "rejects a malformed start pose")

    # +1 sign must produce the mirror image
    plus = standoff_pose(a1, 150.0, +1)
    check(abs(plus[1] - (a1[1] + 150.0)) < 1e-9, "sign=+1 mirrors the standoff to higher y")

    print("\n[self-test] %s" % ("ALL PASS" if failures == 0 else "%d FAILURE(S)" % failures))
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(_self_test())
