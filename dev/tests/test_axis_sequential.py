#!/usr/bin/env python3
"""No diagonals: a Cartesian move is decomposed into single-axis legs.

THE FAILURES THIS EXISTS FOR, both measured on this rig on 2026-08-07 after the
tray was relocated 322 mm in -Y:

  1. The direct floor -> microscope traverse aborted with controller error 31
     (collision / abnormal current) at x=461.0 y=253.9 z=77.9.
  2. Lifting out of the new tray, the controller ACCEPTED floor -> home, moved
     about 23 mm of Z, stopped, and reported success. The run printed
     "arrived home" with the arm still over the tray.

The straight line between two safe poses is not itself safe, and a joint move
that returns without raising is not evidence of arrival.

This checks the ordering contract and the arrival check with a fake arm. No
hardware, no motion, no network.

    python dev/tests/test_axis_sequential.py
"""
from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from scripts.tool_building.pick_place import Z_CORRIDOR_MARGIN_MM  # noqa: E402
from tools.arm.api import (  # noqa: E402
    ARRIVAL_TOL_DEG,
    ARRIVAL_TOL_MM,
    JOINT_ARRIVAL_TOL_DEG,
    MAX_ROT_STEP_DEG,
    Arm,
)
from tools.arm.safety import SafetyError  # noqa: E402

failures = 0


def ok(condition: bool, message: str, detail: str = "") -> None:
    global failures
    print("[test] %s: %s%s" % ("PASS" if condition else "FAIL", message,
                               "  (%s)" % detail if detail else ""))
    if not condition:
        failures += 1


HERE = [366.562, 0.423, 61.372, 180.0, 0.0, 0.0]
FLOOR = [366.562, -184.774, 7.389]


class FakeArm:
    """Records the legs move_axiswise asks for. Arrives unless told otherwise."""

    def __init__(self, here, *, refuse_leg=None):
        self._pose = list(here)
        self.legs = []
        self.refuse_leg = refuse_leg
        self.settings = type("S", (), {"live": True, "speed_mm_s": 10.0})()

    def pose(self):
        return list(self._pose)

    def _log(self, msg, **kw):
        pass

    def move(self, x, y, z, *, speed=None, label="move", takeup=True):
        self.legs.append({"target": (x, y, z), "label": label, "takeup": takeup})
        refuse = self.refuse_leg is not None and len(self.legs) == self.refuse_leg
        if not refuse:
            self._pose[0], self._pose[1], self._pose[2] = x, y, z
        achieved = list(self._pose[:3]) + list(HERE[3:])

        class R:
            target = (x, y, z)

            @staticmethod
            def verify(*a, **k):
                gap = max(abs(a_ - b_) for a_, b_ in zip((x, y, z), achieved[:3], strict=True))
                return {"moved": gap <= ARRIVAL_TOL_MM, "reason": "gap %.3f mm" % gap,
                        "worst_gap": gap}
        return R()

    axiswise = Arm.move_axiswise


def run_axiswise(fake, x, y, z, **kw):
    return Arm.move_axiswise(fake, x, y, z, **kw)


# -- 1. descending: horizontal first, height given up last -----------------
f = FakeArm(HERE)
run_axiswise(f, *FLOOR, speed=10.0, label="to floor")
# derive which axis each leg changed, in order
changed = []
prev = [HERE[0], HERE[1], HERE[2]]
for leg in f.legs:
    t = list(leg["target"])
    for name, k in (("x", 0), ("y", 1), ("z", 2)):
        if abs(t[k] - prev[k]) > ARRIVAL_TOL_MM:
            changed.append(name)
    prev = t

ok(len(changed) == len(f.legs),
   "each leg changes exactly one axis",
   "legs=%d axis-changes=%d %s" % (len(f.legs), len(changed), changed))
ok(changed[-1] == "z",
   "descending to the tray gives up height LAST",
   "order=%s" % changed)
ok("y" in changed and changed.index("y") < changed.index("z"),
   "Y travel precedes the Z descent (the operator's rule)",
   "order=%s" % changed)

# -- 2. ascending: height bought first --------------------------------------
f2 = FakeArm(list(FLOOR) + HERE[3:])
run_axiswise(f2, HERE[0], HERE[1], HERE[2], speed=10.0, label="off floor")
prev = list(FLOOR)
changed2 = []
for leg in f2.legs:
    t = list(leg["target"])
    for name, k in (("x", 0), ("y", 1), ("z", 2)):
        if abs(t[k] - prev[k]) > ARRIVAL_TOL_MM:
            changed2.append(name)
    prev = t
ok(changed2[0] == "z",
   "leaving the tray buys height FIRST",
   "order=%s" % changed2)

# -- 3. an axis already in place is skipped, not re-commanded ---------------
f3 = FakeArm(HERE)
run_axiswise(f3, HERE[0], -184.774, 7.389, speed=10.0, label="same x")
moved_x = [leg for leg in f3.legs if abs(leg["target"][0] - HERE[0]) > ARRIVAL_TOL_MM]
ok(not moved_x,
   "an axis already within tolerance is never commanded",
   "%d of %d step(s) touched x" % (len(moved_x), len(f3.legs)))

# -- 4. a refused leg stops the sequence ------------------------------------
f4 = FakeArm(HERE, refuse_leg=1)
try:
    run_axiswise(f4, *FLOOR, speed=10.0, label="refused")
    ok(False, "a refused leg raises instead of continuing")
except SafetyError as exc:
    ok(len(f4.legs) == 1,
       "a refused leg stops the sequence rather than commanding the rest",
       "%d leg(s) attempted, then: %s" % (len(f4.legs), str(exc)[:60]))

# -- 5. order must be a real permutation ------------------------------------
f5 = FakeArm(HERE)
try:
    run_axiswise(f5, *FLOOR, speed=10.0, order="zz")
    ok(False, "a malformed order is rejected")
except SafetyError:
    ok(True, "a malformed order is rejected rather than silently reinterpreted")

# -- 6. the joint tolerance is loose enough to be real, tight enough to bite -
ok(0.05 < JOINT_ARRIVAL_TOL_DEG < 2.0,
   "joint arrival tolerance sits between settling noise and a real leg",
   "%.2f deg" % JOINT_ARRIVAL_TOL_DEG)

# ===========================================================================
# ROTATION -- the same contract, on the orientation axes.
# ===========================================================================
ROT_HERE = [366.562, -184.222, 191.208, 179.995, -0.002, 0.003]
SCOPE_RPY = (179.995, -0.003, 89.985)


class FakeRotArm:
    """Records the orientation legs rotate_in_place asks for."""

    def __init__(self, here, *, refuse_leg=None):
        self._pose = list(here)
        self.legs = []
        self.refuse_leg = refuse_leg
        self.settings = type("S", (), {"live": True, "speed_mm_s": 10.0})()

    def pose(self):
        return list(self._pose)

    def _log(self, msg, **kw):
        pass

    def move_pose(self, pose, *, speed=None, label="move", takeup=False,
                  verify=True, announce=True):
        self.legs.append({"target": tuple(pose), "label": label})
        refuse = self.refuse_leg is not None and len(self.legs) == self.refuse_leg
        if not refuse:
            self._pose = [float(v) for v in pose]
        achieved = list(self._pose)

        class R:
            target = tuple(pose)
        R.achieved = achieved
        return R

    rotator = Arm.rotate_in_place


def run_rotate(fake, roll, pitch, yaw, **kw):
    return fake.rotator(roll, pitch, yaw, **kw)


# -- 7. a 90 deg rotation is chunked, never one command ----------------------
r1 = FakeRotArm(ROT_HERE)
legs1 = run_rotate(r1, *SCOPE_RPY, speed=20.0)
ok(len(r1.legs) >= 6,
   "a 90 deg rotation is subdivided into bounded steps, not one command",
   "%d step(s) of <= %.0f deg" % (len(r1.legs), MAX_ROT_STEP_DEG))

worst_step = max(
    abs(leg["target"][5] - (ROT_HERE[5] if i == 0 else r1.legs[i - 1]["target"][5]))
    for i, leg in enumerate(r1.legs))
ok(worst_step <= MAX_ROT_STEP_DEG + 1e-6,
   "no single rotation step exceeds the cap",
   "worst %.3f deg" % worst_step)

# -- 8. XYZ is pinned for the whole rotation --------------------------------
moved_xyz = [leg["target"][:3] for leg in r1.legs
             if max(abs(leg["target"][j] - ROT_HERE[j]) for j in range(3)) > 1e-9]
ok(not moved_xyz,
   "a rotation translates nothing: every step holds the starting XYZ",
   "%d step(s) moved XYZ" % len(moved_xyz))

# -- 9. it actually lands on the taught orientation -------------------------
ok(abs(r1.pose()[5] - SCOPE_RPY[2]) <= ARRIVAL_TOL_DEG,
   "the rotation arrives at the taught yaw",
   "yaw %.4f, wanted %.4f" % (r1.pose()[5], SCOPE_RPY[2]))
ok(legs1 and len(legs1) == len(r1.legs), "every commanded step is returned")

# -- 10. an orientation already in place commands nothing -------------------
r2 = FakeRotArm([366.562, -184.222, 191.208, *SCOPE_RPY])
legs2 = run_rotate(r2, *SCOPE_RPY, speed=20.0)
ok(not r2.legs and legs2 == [],
   "a rotation already within tolerance commands nothing at all",
   "%d step(s)" % len(r2.legs))

# -- 11. a refused rotation leg raises, as a refused translation leg does ----
r3 = FakeRotArm(ROT_HERE, refuse_leg=2)
try:
    run_rotate(r3, *SCOPE_RPY, speed=20.0)
    ok(False, "a refused rotation leg raises rather than reporting arrival")
except SafetyError as exc:
    ok(len(r3.legs) == 2,
       "a refused rotation leg stops the sequence rather than rotating on",
       "%d step(s) attempted, then: %s" % (len(r3.legs), str(exc)[:60]))

# -- 12. the rotation takes the short way round the wrap ---------------------
r4 = FakeRotArm([366.562, -184.222, 191.208, 179.995, -0.002, 179.0])
run_rotate(r4, 179.995, -0.003, -179.0, speed=20.0)
ok(len(r4.legs) == 1,
   "179 -> -179 deg is a 2 deg move, not a 358 deg unwind",
   "%d step(s)" % len(r4.legs))

# -- 13. the orientation tolerance is sane -----------------------------------
ok(0.0 < ARRIVAL_TOL_DEG <= 0.5,
   "orientation arrival is at least as tight as the envelope's own tolerance",
   "%.2f deg arrival vs 0.50 deg envelope" % ARRIVAL_TOL_DEG)

# ===========================================================================
# TRANSIT CORRIDOR -- it has to admit the height the route travels at.
# ===========================================================================
from tools.arm.safety import Envelope  # noqa: E402

# The measured case, to six decimals.
TAUGHT_CEIL = 191.209473      # max taught z (scope_standoff / microscope)
TAUGHT_FLOOR = 10.008110      # taught tray z
SETTLED_Z = 191.209518        # what the arm actually read, 45 nm higher

# -- 14. zero margin is what bit: reproduce the abort ------------------------
tight = Envelope.free(reason="regression: corridor with no headroom",
                      z_floor_mm=TAUGHT_FLOOR, z_ceiling_mm=TAUGHT_CEIL)
try:
    tight.check_target((0.0, 0.0, SETTLED_Z, 179.995, -0.003, 89.985),
                       what="settled at the ceiling")
    ok(False, "a zero-margin corridor rejects the height it operates at")
except SafetyError:
    ok(True, "a zero-margin corridor rejects the height it operates at",
       "45 nm over the ceiling aborts the run -- this is the 2026-08-11 bug")

# -- 15. the margin admits it, and still bounds the sweep --------------------
margined = Envelope.free(reason="regression: corridor with settling margin",
                         z_floor_mm=TAUGHT_FLOOR - Z_CORRIDOR_MARGIN_MM,
                         z_ceiling_mm=TAUGHT_CEIL + Z_CORRIDOR_MARGIN_MM)
try:
    margined.check_target((0.0, 0.0, SETTLED_Z, 179.995, -0.003, 89.985),
                          what="settled at the ceiling")
    ok(True, "the margined corridor admits the working height plus settling",
       "%.4f mm margin" % Z_CORRIDOR_MARGIN_MM)
except SafetyError as exc:
    ok(False, "the margined corridor admits the working height plus settling",
       str(exc)[:70])

# -- 16. the margin is settling-sized, not a licence to climb ----------------
ok(0.0 < Z_CORRIDOR_MARGIN_MM <= 1.0,
   "the corridor margin covers settling only, far below the 10 mm sweep rise",
   "%.2f mm margin vs 10.0 mm Z_MAX_RISE" % Z_CORRIDOR_MARGIN_MM)

try:
    margined.check_target((0.0, 0.0, TAUGHT_CEIL + 5.0, 179.995, -0.003, 89.985),
                          what="5 mm above the ceiling")
    ok(False, "the margined corridor still refuses a real climb toward the lens")
except SafetyError:
    ok(True, "the margined corridor still refuses a real climb toward the lens",
       "+5 mm rejected")

print()
print("%d failure(s)" % failures if failures else "all checks passed")
sys.exit(1 if failures else 0)
