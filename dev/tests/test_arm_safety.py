#!/usr/bin/env python3
"""Guard tests for the arm's motion envelope. No hardware, no motion, no network.

These are the tests that were written against ``focus_sweep``'s inline guards
and now target the shared :mod:`tools.arm.safety`. Every original
assertion is kept -- including the regression case that matters most, the arm
parked at the *old* ``scope_standoff`` 340 mm away, which the first version of
the sweep would have driven away from with a full 6-DOF traverse it believed was
a "Z step".

Added since the consolidation: the structural guarantee that a pose cannot reach
the SDK without passing a guard.

    python test/test_arm_safety.py
"""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from tools.arm.api import Arm, ArmSettings  # noqa: E402
from tools.arm.driver import XArmConnection  # noqa: E402
from tools.arm.safety import (  # noqa: E402
    BACKLASH_TAKEUP_MM,
    SPEED_MAX_MM_S,
    TRANSIT_SPEED_MAX_MM_S,
    XY_MAX_MM,
    Z_MAX_RISE_MM,
    Envelope,
    SafetyError,
    ValidatedMove,
)

#: The taught A1 on this rig, 2026-07-31.
A1 = [26.028557, 590.625244, 177.245712, 179.995150, -0.002807, 89.745359]

#: Where the arm sat before the rig was relocated. 340 mm away in X and above
#: the Z ceiling -- the pose that made the readback guard necessary.
OLD_STANDOFF = [343.832336, 449.834473, 188.366989, 179.907774, -0.002521, 90.595456]

fails = 0


def ok(condition: bool, message: str) -> None:
    global fails
    print("[test] %s: %s" % ("PASS" if condition else "FAIL", message))
    if not condition:
        fails += 1


def blocked(fn, message: str) -> None:
    global fails
    try:
        fn()
    except SafetyError as exc:
        print("[test] PASS: %s  -> %s" % (message, str(exc)[:78]))
        return
    print("[test] FAIL: %s -- nothing raised" % message)
    fails += 1


# The Z-sweep envelope: XY frozen at A1, Z free to rise within the budget.
SWEEP = Envelope.anchored(A1, name="A1", z_max_rise_mm=Z_MAX_RISE_MM,
                          xy_max_mm=0.0, orient_tol_deg=0.0)
# The centring envelope: Z pinned at the focused height, XY free within the box.
CENTRE = Envelope.anchored(A1, name="A1", z_max_rise_mm=Z_MAX_RISE_MM,
                           xy_max_mm=XY_MAX_MM, orient_tol_deg=0.5,
                           readback_slack_mm=0.0, readback_slack_deg=0.0)

print("--- target guard: Z budget and declared axes ---")
p = SWEEP.pose_at(z=A1[2] + Z_MAX_RISE_MM)
SWEEP.check_target(p, what="t"); ok(True, "ceiling exactly is allowed")
blocked(lambda: SWEEP.check_target(SWEEP.pose_at(z=A1[2] + Z_MAX_RISE_MM + 0.001), what="t"),
        "0.001 mm above ceiling")
blocked(lambda: SWEEP.check_target(SWEEP.pose_at(z=A1[2] - 0.01), what="t"),
        "below A1 (downward, toward the deck)")
blocked(lambda: SWEEP.check_target(SWEEP.pose_at(y=A1[1] + 9.0), what="t"),
        "one well pitch in Y during a Z-only sweep")
blocked(lambda: SWEEP.check_target([*A1[:5], A1[5] + 2.0], what="t"),
        "2 deg of yaw during a Z-only sweep")

print("\n--- target guard: the XY box ---")
CENTRE.check_target(CENTRE.pose_at(x=A1[0] + XY_MAX_MM), what="t")
ok(True, "XY box edge exactly is allowed")
blocked(lambda: CENTRE.check_target(CENTRE.pose_at(x=A1[0] + XY_MAX_MM + 0.01), what="t"),
        "0.01 mm outside the XY box")
blocked(lambda: CENTRE.check_target(CENTRE.pose_at(y=A1[1] - XY_MAX_MM - 0.01), what="t"),
        "outside the XY box in -Y")

print("\n--- readback guard (the fix for 'first move is a 6-DOF traverse') ---")
SWEEP.check_readback(list(A1), where="t"); ok(True, "exactly at A1 accepted")
SWEEP.check_readback(SWEEP.pose_at(z=A1[2] + 5.0), where="t"); ok(True, "mid-window Z accepted")
blocked(lambda: SWEEP.check_readback(OLD_STANDOFF, where="t"),
        "arm parked at the OLD scope_standoff (340 mm away, above the ceiling)")
blocked(lambda: SWEEP.check_readback(SWEEP.pose_at(x=A1[0] + 1.0), where="t"),
        "1 mm off in X")
blocked(lambda: SWEEP.check_readback([*A1[:5], A1[5] + 2.0], where="t"),
        "2 deg off in yaw")
blocked(lambda: SWEEP.check_readback(SWEEP.pose_at(z=A1[2] + Z_MAX_RISE_MM + 2.0), where="t"),
        "readback Z above the window")
SWEEP.check_readback(SWEEP.pose_at(x=A1[0] + 0.4), where="t")
ok(True, "0.4 mm of settling slack accepted (readback tolerance 0.5 mm)")

print("\n--- malformed poses ---")
blocked(lambda: SWEEP.check_target([1, 2, 3], what="t"), "3-element pose")
blocked(lambda: SWEEP.check_target(SWEEP.pose_at(z=float("nan")), what="t"), "NaN Z")
blocked(lambda: SWEEP.check_target(SWEEP.pose_at(z=float("inf")), what="t"), "infinite Z")
blocked(lambda: Envelope.anchored([1, 2, 3]), "anchor with the wrong shape")

print("\n--- speed cap ---")
SWEEP.check_speed(SPEED_MAX_MM_S, what="t"); ok(True, "cap exactly is allowed")
blocked(lambda: SWEEP.check_speed(SPEED_MAX_MM_S + 0.1, what="t"), "0.1 mm/s over the cap")
blocked(lambda: SWEEP.check_speed(500.0, what="t"), "500 mm/s")
blocked(lambda: SWEEP.check_speed(float("inf"), what="t"), "infinite speed")
blocked(lambda: SWEEP.check_speed(0.0, what="t"), "zero speed")
blocked(lambda: SWEEP.check_speed(-5.0, what="t"), "negative speed")

print("\n--- the two speed caps are not interchangeable ---")
ok(TRANSIT_SPEED_MAX_MM_S > SPEED_MAX_MM_S,
   "transit cap (%.0f) is looser than the fine-positioning cap (%.0f)"
   % (TRANSIT_SPEED_MAX_MM_S, SPEED_MAX_MM_S))
blocked(lambda: SWEEP.check_speed(TRANSIT_SPEED_MAX_MM_S, what="t"),
        "transit speed refused by an anchored (fine-positioning) envelope")
Envelope.free(reason="transit", z_floor_mm=0.0, z_ceiling_mm=500.0).check_speed(
    TRANSIT_SPEED_MAX_MM_S, what="t")
ok(True, "transit speed allowed by a free envelope")
blocked(lambda: Envelope.free(reason="transit", z_floor_mm=0.0, z_ceiling_mm=500.0)
        .check_speed(TRANSIT_SPEED_MAX_MM_S + 1, what="t"),
        "1 mm/s over even the transit cap")

print("\n--- an unanchored envelope must be asked for explicitly ---")
blocked(lambda: Envelope.free(reason="", z_floor_mm=0.0, z_ceiling_mm=1.0),
        "Envelope.free() with no reason")
blocked(lambda: Envelope.free(reason="transit", z_floor_mm=10.0, z_ceiling_mm=1.0),
        "Envelope.free() with an inverted Z corridor")
free = Envelope.free(reason="plate transport", z_floor_mm=100.0, z_ceiling_mm=300.0)
free.check_target([0.0, 0.0, 200.0, 180.0, 0.0, 90.0], what="t")
ok(True, "a pose inside the free corridor is allowed")
blocked(lambda: free.check_target([0.0, 0.0, 350.0, 180.0, 0.0, 90.0], what="t"),
        "above the free corridor's Z ceiling")

print("\n--- guards cannot be bypassed (structural) ---")
blocked(lambda: ValidatedMove(object(), A1, 10.0, "forged", SWEEP),
        "ValidatedMove constructed directly, without a guard")
try:
    XArmConnection("127.0.0.1", live=False).send(A1)  # a bare list, not a ValidatedMove
    print("[test] FAIL: driver.send() accepted an unvalidated pose")
    fails += 1
except SafetyError as exc:
    print("[test] PASS: driver.send() rejects an unvalidated pose  -> %s" % str(exc)[:60])

move = SWEEP.validate(SWEEP.pose_at(z=A1[2] + 1.0), 10.0, "ok")
ok(isinstance(move, ValidatedMove), "Envelope.validate() produces a ValidatedMove")
ok(XArmConnection("127.0.0.1", live=False).send(move) is None,
   "a dry-run connection accepts a validated move and commands nothing")

print("\n--- one session, one engagement flag (fake SDK; nothing real is touched) ---")


class _FakeApi:
    """Stands in for XArmAPI. No network, no SDK, no arm."""

    connected = True
    error_code = 0
    warn_code = 0
    state = 0
    mode = 0
    last_pose = list(A1)

    def __init__(self):
        self.sent = []
        self.armed = 0          # motion_enable / set_mode / set_state calls

    def set_position(self, **kw):
        self.sent.append(kw)
        return 0

    def get_position(self, is_radian=False):
        return 0, list(self.last_pose)

    def motion_enable(self, *a, **k):
        self.armed += 1

    def set_mode(self, *a, **k):
        self.armed += 1

    def set_state(self, *a, **k):
        self.armed += 1

    def get_bio_gripper_status(self):
        return 0

    def get_bio_gripper_error(self):
        return 0


_arm = Arm(ArmSettings(host="127.0.0.1", live=True), envelope=CENTRE)
_fake = _FakeApi()
_arm.connection._api = _fake                     # pre-opened with the stub
_view = _arm.with_envelope(SWEEP)                # a second view of the SAME session

ok(not _arm.engaged and not _view.engaged, "a fresh session owes no retreat")

print("\n--- connecting is not arming ---")
_arm.connection.read_pose()
_arm.connection.status()
ok(_fake.armed == 0,
   "reading pose and status never enables motion (the retired gripper.py "
   "connected without arming, and 'controller'/'gripper status' are documented "
   "read-only)")

print("\n--- the arm's real pose is checked BEFORE the first commanded move ---")
_stray = Arm(ArmSettings(host="127.0.0.1", live=True), envelope=SWEEP)
_stray_fake = _FakeApi()
_stray.connection._api = _stray_fake
_FakeApi.last_pose = list(OLD_STANDOFF)          # rig relocated; arm is 340 mm away
blocked(lambda: _stray.move(A1[0], A1[1], A1[2] + 1.0, label="first step", takeup=False),
        "first move refused while the arm sits at the OLD standoff")
ok(len(_stray_fake.sent) == 0,
   "and nothing at all reached the SDK -- the traverse never started")

_FakeApi.last_pose = [A1[0], A1[1], A1[2] + 1.0, A1[3], A1[4], A1[5]]
_view.move(A1[0], A1[1], A1[2] + 1.0, label="view move", takeup=False)
ok(_arm.engaged and _view.engaged,
   "moving through one view engages the other -- otherwise the original's "
   "retreat would be skipped and the plate could be left raised")
ok(len(_fake.sent) == 1, "exactly one command reached the SDK")
ok(_fake.armed > 0, "the motion path DID arm the controller before commanding")

_dry = Arm(ArmSettings(host="127.0.0.1", live=False), envelope=CENTRE)
_dry.move(A1[0], A1[1], A1[2] + 1.0, label="dry move")
ok(not _dry.engaged, "a dry-run move never engages, so it owes no retreat")

print("\n--- a config gate never guesses ---")
from tools import config as _cfg  # noqa: E402

for _word, _want in (("no", False), ("off", False), ("false", False), ("n", False),
                     ("yes", True), ("on", True), ("true", True)):
    _got = _cfg.resolve("live", config={"live": _word}, default=None, cast=bool).value
    ok(_got is _want,
       "allow_motion: %-5s -> %s (bool(%r) would have been True)" % (_word, _got, _word))
try:
    _cfg.resolve("live", config={"live": "flase"}, default=None, cast=bool)
    ok(False, "a typo'd boolean must raise, not enable motion")
except _cfg.ConfigError:
    ok(True, "a typo'd boolean raises rather than enabling motion")

blocked_cfg = 0
try:
    ArmSettings.from_config({"arm": {"z_max_rise_mm": 50.0}})
except _cfg.ConfigError:
    blocked_cfg += 1
try:
    ArmSettings.from_config({"arm": {"speed_max_mm_s": 300.0}})
except _cfg.ConfigError:
    blocked_cfg += 1
ok(blocked_cfg == 2, "config.yaml cannot loosen a hardware-clearance limit")
ok(ArmSettings.from_config({"arm": {"z_max_rise_mm": 3.0}}).z_max_rise_mm == 3.0,
   "config.yaml CAN tighten one")

print("\n--- retreat can actually fire, under every envelope shape ---")

for _name, _env in (("Z-only sweep (xy_max_mm=0)", SWEEP), ("XY box (xy_max_mm=2.5)", CENTRE)):
    _r = Arm(ArmSettings(host="127.0.0.1", live=True), envelope=_env)
    _rf = _FakeApi()
    _r.connection._api = _rf
    # the arm is raised, and its settled XY is NOT bit-exact with the anchor
    _FakeApi.last_pose = [A1[0] + 0.0004, A1[1] - 0.0003, A1[2] + 6.0, A1[3], A1[4], A1[5]]
    _r.connection.engaged = True                 # pretend a sweep step already ran
    _r.retreat()
    _lowered = [c for c in _rf.sent if abs(c["z"] - A1[2]) < 1e-6]
    ok(len(_lowered) == 1,
       "retreat lowered the plate under the %s envelope" % _name)

_FakeApi.last_pose = list(A1)

print("\n--- backlash take-up always approaches from below ---")
target = SWEEP.pose_at(z=A1[2] + 5.0)
dip = SWEEP.backlash_dip(target, takeup_mm=BACKLASH_TAKEUP_MM)
ok(dip is not None and dip[2] < target[2], "the dip sits below the target")
ok(dip is not None and abs(dip[2] - (target[2] - BACKLASH_TAKEUP_MM)) < 1e-9,
   "the dip is exactly one take-up below")
floor_target = SWEEP.pose_at(z=A1[2])
ok(SWEEP.backlash_dip(floor_target) is None,
   "no dip below the floor: A1 itself is never undercut")
SWEEP.check_target(dip, what="the dip is itself inside the envelope"); ok(True, "dip is guarded too")

print("\n--- CLI validation (unbounded numbers reach no motion) ---")
SCRIPT = REPO / "scripts" / "microscope" / "sharpness_test.py"
for argv, why in [
    (["--max-rise", "nan"], "--max-rise nan"),
    (["--max-rise", "20"], "--max-rise above the hard ceiling"),
    (["--max-rise", "-1"], "--max-rise negative"),
    (["--speed", "500"], "--speed 500 mm/s"),
    (["--speed", "inf"], "--speed inf"),
    (["--min-mean", "0"], "--min-mean 0 (would disable the brightness gate)"),
    (["--fine-step", "0.0005"], "step config implying a huge number of moves"),
    (["--frame-ordinal", "0"], "--frame-ordinal 0"),
]:
    result = subprocess.run([sys.executable, str(SCRIPT), *argv],
                            capture_output=True, text=True, timeout=90)
    rejected = result.returncode != 0
    output = (result.stderr or result.stdout).strip()
    tail = output.splitlines()[-1][:76] if output else "(no message)"
    print("[test] %s: %s  -> %s" % ("PASS" if rejected else "FAIL", why, tail))
    if not rejected:
        fails += 1

print("\n%s" % ("ALL PASS" if fails == 0 else "%d FAILURE(S)" % fails))
sys.exit(1 if fails else 0)
