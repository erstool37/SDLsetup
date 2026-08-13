#!/usr/bin/env python3
"""Safety-critical unit tests for scripts/microscope/find_target.py.

NO HARDWARE. tools.arm.safety.Envelope is exercised directly (real guard
code, no fake), and tools.arm.Arm is stubbed with a minimal fake that
raises tools.arm.safety.SafetyError on command, so the retreat/coverage
orchestration can be tested without a controller. Import find_target.py as
a module the way scripts/microscope/sharpness_test.py etc. do -- it lives
outside any package (no __init__.py in scripts/), so it is loaded by path.

    python dev/tests/test_find_target_safety.py
"""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from tools.arm.safety import SafetyError  # noqa: E402
from tools.microscope.focus import FocusSample  # noqa: E402

_spec = importlib.util.spec_from_file_location(
    "find_target", REPO / "scripts" / "microscope" / "find_target.py")
ft = importlib.util.module_from_spec(_spec)
# Register BEFORE exec: @dataclasses.dataclass resolves its own module through
# sys.modules, and a module loaded by spec alone is not there yet -- which
# fails with a bare AttributeError deep inside dataclasses.
sys.modules[_spec.name] = ft
_spec.loader.exec_module(ft)

failures = 0


def ok(condition: bool, message: str, detail: str = "") -> None:
    global failures
    print("[test] %s: %s%s" % ("PASS" if condition else "FAIL", message,
                               f"  ({detail})" if detail else ""))
    if not condition:
        failures += 1


A1 = [26.028557, 590.625244, 177.245712, 179.995150, -0.002807, 89.745359]

print("--- envelope SHAPES: XY and Z never change in the same command ---")

xy_env = ft.build_xy_envelope(A1, working_rise_mm=3.0, xy_radius_mm=4.0)
ok(xy_env.is_anchored and xy_env.z_min_rise_mm == xy_env.z_max_rise_mm == 3.0,
   "XY-travel envelope pins Z to a single value (the working rise)")
try:
    xy_env.check_target((A1[0] + 1.0, A1[1], A1[2] + 3.5, *A1[3:]), what="t")
    ok(False, "XY envelope rejects a Z change away from the pinned working height")
except SafetyError:
    ok(True, "XY envelope rejects a Z change away from the pinned working height")
xy_env.check_target((A1[0] + 1.0, A1[1] + 1.0, A1[2] + 3.0, *A1[3:]), what="t")
ok(True, "XY envelope allows an XY-only move at the pinned working height")

z_env = ft.build_z_envelope(A1, dx_mm=1.5, dy_mm=-0.5, z_max_rise_mm=10.0)
ok(z_env.xy_max_mm == 0.0, "Z-probe envelope pins XY (xy_max_mm=0.0)")
ok(abs(z_env.anchor[2] - A1[2]) < 1e-9,
   "Z-probe envelope's Z reference is the IMMUTABLE taught A1 Z, not a translated one",
   f"anchor z={z_env.anchor[2]} a1 z={A1[2]}")
try:
    z_env.check_target((A1[0] + 1.5 + 0.5, A1[1] - 0.5, A1[2] + 2.0, *A1[3:]), what="t")
    ok(False, "Z-probe envelope rejects any XY drift away from its pinned candidate XY")
except SafetyError:
    ok(True, "Z-probe envelope rejects any XY drift away from its pinned candidate XY")
z_env.check_target((A1[0] + 1.5, A1[1] - 0.5, A1[2] + 8.0, *A1[3:]), what="t")
ok(True, "Z-probe envelope allows a Z-only move at its own pinned XY")

print("\n--- Z budget is measured from the immutable taught A1, always ---")
z_env_2 = ft.build_z_envelope(A1, dx_mm=3.0, dy_mm=3.0, z_max_rise_mm=10.0)
try:
    z_env_2.check_target((A1[0] + 3.0, A1[1] + 3.0, A1[2] + 10.001, *A1[3:]), what="t")
    ok(False, "0.001mm above the 10mm ceiling above taught A1 is refused")
except SafetyError:
    ok(True, "0.001mm above the 10mm ceiling above taught A1 is refused")
z_env_2.check_target((A1[0] + 3.0, A1[1] + 3.0, A1[2] + 10.0, *A1[3:]), what="t")
ok(True, "exactly the 10mm ceiling above taught A1 is allowed")
try:
    z_env_2.check_target((A1[0] + 3.0, A1[1] + 3.0, A1[2] - 0.01, *A1[3:]), what="t")
    ok(False, "below the taught A1 height is refused")
except SafetyError:
    ok(True, "below the taught A1 height is refused")

print("\n--- attempt_move: a silent refusal becomes data, not an abort ---")


class _FakeMoveResult:
    def __init__(self, achieved):
        self.achieved = achieved


class _FakeArm:
    """Duck-typed stand-in for tools.arm.Arm's .move(); nothing else is used."""

    def __init__(self, *, refuse: bool = False, live: bool = True):
        self.refuse = refuse
        self.settings = type("S", (), {"live": live})()
        self.calls = []

    def move(self, x, y, z, *, speed, label, takeup=False):
        self.calls.append((x, y, z, label))
        if self.refuse:
            raise SafetyError("y is below the reachable limit at this x (measured refusal)")
        return _FakeMoveResult([x, y, z, 0.0, 0.0, 0.0])

    def with_envelope(self, envelope):
        return self


ok_arm = _FakeArm(refuse=False)
outcome = ft.attempt_move(ok_arm, 1.0, 2.0, 3.0, speed=5.0, label="t")
ok(outcome.ok and outcome.error is None, "attempt_move reports success when move() succeeds")

refusing_arm = _FakeArm(refuse=True)
outcome2 = ft.attempt_move(refusing_arm, 1.0, 2.0, 3.0, speed=5.0, label="t")
ok(not outcome2.ok and outcome2.error, "attempt_move turns SafetyError into ok=False + reason",
   f"{outcome2.error}")

print("\n--- goto_xy / goto_z build the right envelope shape and never crash ---")
xy_probe = ft.goto_xy(_FakeArm(refuse=False), A1, 1.0, -1.0, A1[2] + 3.0, xy_env,
                     speed=5.0, label="t")
ok(xy_probe.ok, "goto_xy succeeds against a non-refusing fake arm")
z_probe = ft.goto_z(_FakeArm(refuse=False), A1, 1.0, -1.0, A1[2] + 5.0, 10.0,
                   speed=5.0, label="t")
ok(z_probe.ok, "goto_z succeeds against a non-refusing fake arm")
z_probe_refused = ft.goto_z(_FakeArm(refuse=True), A1, 1.0, -1.0, A1[2] + 5.0, 10.0,
                           speed=5.0, label="t")
ok(not z_probe_refused.ok, "goto_z reports the refusal rather than raising out of the caller")

print("\n--- a saturated/dark structure sample still advances to the next XY ---")

sat_sample = FocusSample(path=Path("/x.jpg"), method="tenengrad", masked=True,
                         mean=251.0, std=3.0, mask_frac=0.5, usable=False,
                         reason="scored region saturated (mean 251.0)")
facts = ft.classify_structure_sample(sat_sample)
ok(facts["saturated"] and not facts["dark"] and facts["std"] == 3.0,
   "a saturated sample is flagged but its std is still reported (not dropped)",
   f"{facts}")

dark_sample = FocusSample(path=Path("/x.jpg"), method="tenengrad", masked=True,
                          mean=2.0, std=0.5, mask_frac=0.5, usable=False,
                          reason="scored region too dark (mean 2.0 < 30.0)")
facts_dark = ft.classify_structure_sample(dark_sample)
ok(facts_dark["dark"] and not facts_dark["saturated"], "a dark sample is flagged as dark")

no_mask_sample = FocusSample(path=Path("/x.jpg"), method="tenengrad", masked=True,
                             mean=None, std=None, mask_frac=0.0, usable=False,
                             reason="no lit sample anywhere in the frame")
facts_none = ft.classify_structure_sample(no_mask_sample)
ok(not facts_none["has_mask"] and facts_none["std"] == 0.0,
   "no lit mask at all reports std=0.0 and has_mask=False, distinct from dark/saturated")

# A structure-pass record with status != "captured" (unreachable / refused)
# must never rank, and a captured-but-no-mask record must not either --
# rank_by_structure must not fabricate a candidate out of nothing.
records = [
    {"status": "unreachable", "reason": "refused", "dx_mm": 0.0, "dy_mm": 0.0},
    {"status": "captured", "has_mask": False, "std": 0.0, "dx_mm": 1.0, "dy_mm": 0.0},
    {"status": "captured", "has_mask": True, "std": 8.5, "dx_mm": 2.0, "dy_mm": 0.0},
    {"status": "captured", "has_mask": True, "std": 3.0, "dx_mm": -1.0, "dy_mm": 0.0},
]
ranked = ft.rank_by_structure(records, top_n=2)
ok(len(ranked) == 2 and ranked[0]["std"] == 8.5 and ranked[1]["std"] == 3.0,
   "rank_by_structure only ranks captured, masked records, highest std first",
   f"{[r['std'] for r in ranked]}")
ranked_1 = ft.rank_by_structure(records, top_n=1)
ok(len(ranked_1) == 1, "rank_by_structure honours top_n")

print("\n--- the probe list is complete, finite, and capped up front ---")

plan = ft.build_probe_list(fov_mm=(2.843, 2.379), xy_radius_mm=4.0, overlap=0.45,
                           max_probes=200)
ok(len(plan) > 0, "a real probe list is non-empty for the default search geometry",
   f"{len(plan)} points")
ok(len(plan) <= 200, "the plan respects the cap")
ok(plan[0].dx_mm == 0.0 and plan[0].dy_mm == 0.0,
   "the plan always starts at the anchor (ring 0)")

try:
    ft.build_probe_list(fov_mm=(0.05, 0.05), xy_radius_mm=4.0, overlap=0.0, max_probes=5)
    ok(False, "an over-cap plan raises FindTargetError rather than silently truncating")
except ft.FindTargetError as exc:
    ok(True, "an over-cap plan raises FindTargetError rather than silently truncating",
       str(exc)[:80])

print("\n--- retreat_to_working_height: best-effort, runs on every branch ---")


class _FakeArmWithPose(_FakeArm):
    def __init__(self, *, here, fail_pose: bool = False, refuse: bool = False, live=True):
        super().__init__(refuse=refuse, live=live)
        self.here = list(here)
        self.fail_pose = fail_pose

    def pose(self):
        if self.fail_pose:
            raise RuntimeError("simulated controller read failure")
        return list(self.here)

    def move(self, x, y, z, *, speed, label, takeup=False):
        result = super().move(x, y, z, speed=speed, label=label, takeup=takeup)
        self.here = [x, y, z, self.here[3], self.here[4], self.here[5]]
        return result


dry_run_arm = _FakeArmWithPose(here=A1, live=False)
ok(ft.retreat_to_working_height(dry_run_arm, A1, A1[2] + 3.0, xy_env, speed=5.0)
   and not dry_run_arm.calls,
   "retreat is a safe no-op in dry-run (nothing commanded, nothing to fail)")

live_here = [A1[0] + 1.0, A1[1] - 0.5, A1[2] + 5.0, *A1[3:]]
live_arm = _FakeArmWithPose(here=live_here, live=True)
retreated = ft.retreat_to_working_height(live_arm, A1, A1[2] + 3.0, xy_env, speed=5.0)
ok(retreated, "retreat succeeds against a non-refusing fake arm")
ok(live_arm.calls and live_arm.calls[-1][2] == A1[2] + 3.0,
   "retreat's commanded Z is the working height, XY held at wherever the arm was",
   f"{live_arm.calls[-1]}")
ok(live_arm.calls[-1][0] == live_here[0] and live_arm.calls[-1][1] == live_here[1],
   "retreat does not change XY")

failing_read_arm = _FakeArmWithPose(here=live_here, live=True, fail_pose=True)
ok(ft.retreat_to_working_height(failing_read_arm, A1, A1[2] + 3.0, xy_env, speed=5.0) is False,
   "retreat reports failure (never raises) when the controller read itself fails")

refusing_retreat_arm = _FakeArmWithPose(here=live_here, live=True, refuse=True)
ok(ft.retreat_to_working_height(refusing_retreat_arm, A1, A1[2] + 3.0, xy_env,
                                speed=5.0) is False,
   "retreat reports failure (never raises) when the retreat move itself is refused")

print("\n%s" % ("ALL PASS" if failures == 0 else "%d FAILURE(S)" % failures))
sys.exit(1 if failures else 0)
