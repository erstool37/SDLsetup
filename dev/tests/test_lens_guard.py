#!/usr/bin/env python3
"""ensure_safe_z must refuse to ascend in place under the objective.

THE HAZARD. plate_imaging.py's module docstring has always said "NEVER call
ensure_safe_z above A1 -- that would ascend through the lens." Step 1 of the
imaging loop calls it unconditionally at whatever XY the arm is at, and
find_target, calibration, sharpness_test, goto_a1 and pick_place ALL leave the
arm parked at A1. With z_standoff_mm=200 and home z=61.37, min_safe_z is 261.37
against A1's 185.3177 -- a ~76 mm ascent straight into a fixed objective.

Nothing caught it: the free envelope's ceiling is max(known_zs)+200 = 385.3 mm,
and plate_imaging never calls approach.validate_route the way goto_a1 does.

The docstring was a comment. This is the guard.

No hardware, no motion, no network. Nothing here connects to the controller.

    python dev/tests/test_lens_guard.py
"""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from tools.arm.approach import LENS_XY_RADIUS_MM  # noqa: E402
from tools.arm.safety import SafetyError  # noqa: E402

_spec = importlib.util.spec_from_file_location(
    "plate_imaging", REPO / "scripts" / "tool_building" / "plate_imaging.py")
pi = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = pi          # dataclasses resolve via sys.modules
_spec.loader.exec_module(pi)

failures = 0

#: The live taught A1 (workspace.json, 2026-08-04 re-teach).
A1_XY = (26.028557, 590.625244)
A1_Z = 185.3177
#: home z and the standoff the executor actually uses.
HOME_Z, STANDOFF = 61.37, 200.0


def ok(condition: bool, message: str, detail: str = "") -> None:
    global failures
    print("[test] %s: %s%s" % ("PASS" if condition else "FAIL", message,
                               "  (%s)" % detail if detail else ""))
    if not condition:
        failures += 1


class _FakeConn:
    def __init__(self):
        self.sent = []


class _FakeRobot:
    """Just enough Arm surface for ensure_safe_z. Commands nothing."""

    def __init__(self, pose):
        self._pose = list(pose)
        self.settings = type("S", (), {"workspace_path": None})()
        self.connection = _FakeConn()

    def pose(self):
        return list(self._pose)


def executor_at(x, y, z, lens_xy=A1_XY):
    ex = pi.Executor.__new__(pi.Executor)          # no __init__: no hardware
    ex.robot = _FakeRobot((x, y, z, 179.995, -0.003, 89.745))
    ex.cfg = type("C", (), {"z_standoff_mm": STANDOFF})()
    ex.execute = True
    ex.holding = False
    ex._speed_for = lambda **kw: {"cart": 10.0, "joint": 10.0}
    ex._lens_xy = lambda: lens_xy                   # bypass the store in tests
    return ex


# -- 1. THE CASE THAT WOULD HAVE HIT THE LENS ------------------------------
ex = executor_at(A1_XY[0], A1_XY[1], A1_Z)
rise = (HOME_Z + STANDOFF) - A1_Z
try:
    ex.ensure_safe_z(HOME_Z)
    ok(False, "parked at A1, ensure_safe_z(home_z) is REFUSED",
       "it returned instead of raising -- a %.1f mm ascent into the lens" % rise)
except SafetyError as exc:
    ok(True, "parked at A1, ensure_safe_z(home_z) is REFUSED", str(exc)[:90])
except Exception as exc:  # noqa: BLE001
    ok(False, "parked at A1, ensure_safe_z(home_z) is REFUSED",
       "raised %s, not SafetyError: %s" % (type(exc).__name__, exc))

ok(rise > 70, "the refused move really was a large ascent", "%.1f mm" % rise)

# -- 2. just inside / just outside the lens radius -------------------------
inside = executor_at(A1_XY[0] + LENS_XY_RADIUS_MM - 1.0, A1_XY[1], A1_Z)
try:
    inside.ensure_safe_z(HOME_Z)
    ok(False, "still refused 1 mm inside the lens radius")
except SafetyError:
    ok(True, "still refused 1 mm inside the lens radius",
       "%.0f mm radius" % LENS_XY_RADIUS_MM)

outside = executor_at(A1_XY[0] + LENS_XY_RADIUS_MM + 5.0, A1_XY[1], A1_Z)
try:
    outside.ensure_safe_z(HOME_Z)
    ok(True, "ALLOWED once clear of the lens radius -- the guard is not a blanket ban")
except SafetyError as exc:
    ok(False, "ALLOWED once clear of the lens radius", "refused: %s" % str(exc)[:70])
except Exception:  # noqa: BLE001
    ok(True, "ALLOWED once clear of the lens radius (failed later, past the guard)")

# -- 3. a DESCENT under the lens is not the hazard and must still pass ------
high = executor_at(A1_XY[0], A1_XY[1], HOME_Z + STANDOFF + 50.0)
try:
    high.ensure_safe_z(HOME_Z)
    ok(True, "no refusal when already above the safe height (nothing ascends)")
except SafetyError as exc:
    ok(False, "no refusal when already above the safe height",
       "refused a non-ascent: %s" % str(exc)[:70])
except Exception:  # noqa: BLE001
    ok(True, "no refusal when already above the safe height (failed past the guard)")

# -- 4. fail-safe: no taught A1 means the guard cannot check ---------------
unknown = executor_at(A1_XY[0], A1_XY[1], A1_Z, lens_xy=None)
try:
    unknown.ensure_safe_z(HOME_Z)
    ok(True, "with no taught A1 the guard does not invent one (behaviour unchanged)")
except SafetyError as exc:
    ok(False, "with no taught A1 the guard does not invent one",
       "raised anyway: %s" % str(exc)[:70])
except Exception:  # noqa: BLE001
    ok(True, "with no taught A1 the guard does not invent one (failed past the guard)")

print()
print("ALL PASS" if failures == 0 else "%d FAILURE(S)" % failures)
sys.exit(1 if failures else 0)
