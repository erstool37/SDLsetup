"""Synthetic/fake-interface test suite for the calibration routines.

The package these covered was split by owner in the reorganisation --
array primitives to the microscope, the well grid and pose teaching to the
arm, the pixel<->arm transform to tools.calib, and the port-based
routines to tools.calib.centering. The aliases below keep every
assertion in this file unchanged, which is the point: the split must not
have altered behaviour.


NO HARDWARE. Every test uses synthetic numpy images and fake ArmPort/
CameraPort doubles. Run with:

    cd /home/lamp/SDLsetup && ~/.pyenv/bin/pyenv exec python \
        test/test_calibration.py

Tests are numbered 2-12 matching the build spec (test 1, the bare import
check, is run separately as a one-liner).
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


import dataclasses
import json
import math
import sys
import tempfile
from pathlib import Path

import numpy as np

from tools.arm import geometry as store
from tools.arm import teach
from tools.calib import centering as routine
from tools.calib import pixel_to_arm as frames
from tools.microscope import imaging as detect

FAILURES = []


def check(name, condition, detail=""):
    status = "PASS" if condition else "FAIL"
    print(f"[{status}] {name} {detail}")
    if not condition:
        FAILURES.append(name)


# ---------------------------------------------------------------------------
# Synthetic image helpers
# ---------------------------------------------------------------------------
def render_dot(width, height, cx, cy, radius, base_val=200.0, dot_val=50.0, noise_sigma=0.0, seed=0):
    yy, xx = np.mgrid[0:height, 0:width].astype(np.float64)
    mask = (xx - cx) ** 2 + (yy - cy) ** 2 <= radius ** 2
    img = np.full((height, width), base_val, dtype=np.float64)
    img[mask] = dot_val
    if noise_sigma > 0:
        rng = np.random.default_rng(seed)
        img = img + rng.normal(0.0, noise_sigma, size=img.shape)
    return np.clip(img, 0.0, 255.0).astype(np.float32)


def render_texture(width, height, amplitude, seed=1):
    """A fixed checkerboard-ish high-frequency pattern scaled by amplitude,
    for sharpness (Laplacian-variance) tests. amplitude=0 -> flat/blurry."""
    yy, xx = np.mgrid[0:height, 0:width]
    pattern = ((xx // 4 + yy // 4) % 2).astype(np.float64) * 60.0 - 30.0
    return np.clip(128.0 + amplitude * pattern, 0.0, 255.0).astype(np.float32)


# ---------------------------------------------------------------------------
# Test 2/3/4 — find_dark_dot
# ---------------------------------------------------------------------------
def test_2_recover_centre_clean():
    w, h = 1536, 1024
    positions = [(768, 512), (100, 100), (1450, 950), (50, 512), (768, 50)]
    max_err = 0.0
    for cx, cy in positions:
        img = render_dot(w, h, cx, cy, radius=20)
        det = detect.find_dark_dot(img)
        assert det.found, f"not found at {(cx, cy)}: {det.note}"
        err = math.hypot(det.cx_px - cx, det.cy_px - cy)
        max_err = max(max_err, err)
    check("2. find_dark_dot recovers centre (clean, 5 positions)", max_err <= 1.0, f"max_err={max_err:.4f}px")


def test_3_recover_centre_noisy():
    w, h = 1536, 1024
    positions = [(768, 512), (100, 100), (1450, 950), (50, 512), (768, 50)]
    max_err = 0.0
    for i, (cx, cy) in enumerate(positions):
        img = render_dot(w, h, cx, cy, radius=20, noise_sigma=15.0, seed=100 + i)
        det = detect.find_dark_dot(img)
        assert det.found, f"not found at {(cx, cy)}: {det.note}"
        err = math.hypot(det.cx_px - cx, det.cy_px - cy)
        max_err = max(max_err, err)
    check("3. find_dark_dot recovers centre (noise sigma=15)", max_err <= 2.0, f"max_err={max_err:.4f}px")


def test_4_not_found_cases():
    w, h = 640, 480
    blank = np.full((h, w), 200.0, dtype=np.float32)
    det_blank = detect.find_dark_dot(blank)

    tiny = render_dot(w, h, w // 2, h // 2, radius=2)  # area ~ pi*4 ~12.6 < min_area_px=30
    det_tiny = detect.find_dark_dot(tiny)

    mostly_dark = np.full((h, w), 40.0, dtype=np.float32)
    mostly_dark[: h // 2, :] = 200.0  # ~50% dark half of frame is > max_area_frac test target
    mostly_dark2 = np.full((h, w), 40.0, dtype=np.float32)  # fully dark: >50%
    det_dark = detect.find_dark_dot(mostly_dark2)

    ok = (not det_blank.found and det_blank.cx_px is None and det_blank.cy_px is None
          and not det_tiny.found and det_tiny.cx_px is None
          and not det_dark.found and det_dark.cx_px is None)
    check(
        "4. find_dark_dot found=False on blank/tiny/mostly-dark",
        ok,
        f"blank={det_blank.note!r} tiny={det_tiny.note!r} dark={det_dark.note!r}",
    )


# ---------------------------------------------------------------------------
# Test 5 — solve_from_probes
# ---------------------------------------------------------------------------
def test_5_solve_from_probes():
    angle = math.radians(12.0)
    scale = 55.0  # px per mm
    j_true = scale * np.array([[math.cos(angle), -math.sin(angle)],
                                [math.sin(angle), math.cos(angle)]])
    base_px = (500.0, 400.0)
    step_mm = 0.2
    after_x = np.array(base_px) + j_true[:, 0] * step_mm
    after_y = np.array(base_px) + j_true[:, 1] * step_mm

    t = frames.solve_from_probes(base_px, tuple(after_x), tuple(after_y), step_mm)
    j_est = np.array([[t.a11, t.a12], [t.a21, t.a22]])
    max_diff = float(np.max(np.abs(j_est - j_true)))
    check("5a. solve_from_probes recovers known transform", max_diff < 1e-6, f"max_diff={max_diff:.3e}")

    raised = False
    try:
        frames.solve_from_probes(base_px, tuple(np.array(base_px) + [1.0, 0.0]),
                                  tuple(np.array(base_px) + [2.0, 0.0]), step_mm)
    except ValueError:
        raised = True
    check("5b. near-singular probe set raises ValueError", raised)


# ---------------------------------------------------------------------------
# Fake hardware for tests 6-10
# ---------------------------------------------------------------------------
class FakeArm:
    def __init__(self, pose):
        self.pose = list(pose)
        self.history = []

    def get_pose(self):
        return list(self.pose)

    def move_cart(self, x, y, z, roll, pitch, yaw, speed):
        self.history.append((x, y, z, roll, pitch, yaw, speed))
        self.pose = [x, y, z, roll, pitch, yaw]


class PhysicalFakeCamera:
    """Renders a frame as a genuine function of the fake arm's pose.

    - XY: pixel offset of the dot = G @ (arm_xy - true_target_xy), where
      true_target_xy is looked up as whichever *nominal* target the arm is
      currently nearest to (models "one dot visible under the objective at
      a time"). G is an independent geometry model from any PixelToArm the
      routine under test uses -- an inverted/estimated transform is exposed
      to the divergence guard, not baked into the fake.
    - Z: image "sharpness" is amplitude-modulated by a Gaussian in Z peaked
      at a configured z_peak, rendered through the real detect.sharpness().
    """

    def __init__(self, arm, g_matrix, nominal_targets, true_targets,
                 z_peak=None, z_sigma=0.6, width=640, height=480, radius=20,
                 render_dot_overlay=True):
        self.arm = arm
        self.g = np.asarray(g_matrix, dtype=np.float64)
        self.nominal_targets = nominal_targets  # {name: (x, y)}
        self.true_targets = true_targets        # {name: (x, y)}
        self.z_peak = z_peak
        self.z_sigma = z_sigma
        self.width = width
        self.height = height
        self.radius = radius
        self.render_dot_overlay = render_dot_overlay

    def _nearest_target_name(self):
        ax, ay = self.arm.pose[0], self.arm.pose[1]
        return min(
            self.nominal_targets,
            key=lambda k: math.hypot(self.nominal_targets[k][0] - ax, self.nominal_targets[k][1] - ay),
        )

    def frame(self):
        name = self._nearest_target_name()
        tx, ty = self.true_targets[name]
        ax, ay = self.arm.pose[0], self.arm.pose[1]
        disp = self.g @ np.array([ax - tx, ay - ty])
        cx = self.width / 2.0 + disp[0]
        cy = self.height / 2.0 + disp[1]

        if self.z_peak is not None:
            z = self.arm.pose[2]
            amplitude = math.exp(-((z - self.z_peak) ** 2) / (2.0 * self.z_sigma ** 2))
        else:
            amplitude = 1.0

        # amplitude in [0,1]; keep the modulated range well inside 0-255 so
        # np.clip saturation never masks the Gaussian-in-Z shape being tested.
        base = render_texture(self.width, self.height, amplitude=amplitude)
        if self.render_dot_overlay:
            yy, xx = np.mgrid[0:self.height, 0:self.width].astype(np.float64)
            mask = (xx - cx) ** 2 + (yy - cy) ** 2 <= self.radius ** 2
            base[mask] = 50.0
        return base.astype(np.float32)


def _identity_g(scale=40.0):
    return scale * np.eye(2)


# ---------------------------------------------------------------------------
# Test 6/7/8 — center_on_dot
# ---------------------------------------------------------------------------
def test_6_center_on_dot_converges():
    g = _identity_g(scale=40.0)
    true_xy = (100.0, 200.0)
    start_offset_mm = (1.5 / math.sqrt(2), 1.5 / math.sqrt(2))  # magnitude 1.5mm
    arm = FakeArm([true_xy[0] + start_offset_mm[0], true_xy[1] + start_offset_mm[1], 50.0, 180.0, 0.0, 90.0])
    cam = PhysicalFakeCamera(
        arm, g_matrix=g,
        nominal_targets={"A1": true_xy}, true_targets={"A1": true_xy},
    )
    transform = frames.PixelToArm(
        a11=g[0, 0], a12=g[0, 1], a21=g[1, 0], a22=g[1, 1],
        um_per_px_x=1000.0 / g[0, 0], um_per_px_y=1000.0 / g[1, 1],
        derived_at="test", probe_step_mm=0.1, residual_px=0.0,
    )
    result = routine.center_on_dot(arm, cam, transform, tolerance_mm=0.02, max_iterations=4, max_correction_mm=3.0)
    residuals = [it["residual_mm"] for it in result.iterations]
    print(f"    iteration residuals (mm): {[f'{r:.5f}' for r in residuals]}")
    check(
        "6. center_on_dot converges from 1.5mm in <=3 iterations",
        result.converged and len(result.iterations) <= 3,
        f"converged={result.converged} n_iter={len(result.iterations)} final_residual={result.residual_mm:.5f}",
    )
    return residuals


def test_7_sign_inverted_aborts():
    g = _identity_g(scale=40.0)
    true_xy = (100.0, 200.0)
    start_offset_mm = (1.5 / math.sqrt(2), 1.5 / math.sqrt(2))
    arm = FakeArm([true_xy[0] + start_offset_mm[0], true_xy[1] + start_offset_mm[1], 50.0, 180.0, 0.0, 90.0])
    cam = PhysicalFakeCamera(
        arm, g_matrix=g,
        nominal_targets={"A1": true_xy}, true_targets={"A1": true_xy},
    )
    good = frames.PixelToArm(
        a11=g[0, 0], a12=g[0, 1], a21=g[1, 0], a22=g[1, 1],
        um_per_px_x=1000.0 / g[0, 0], um_per_px_y=1000.0 / g[1, 1],
        derived_at="test", probe_step_mm=0.1, residual_px=0.0,
    )
    inverted = dataclasses.replace(good, a11=-good.a11, a12=-good.a12, a21=-good.a21, a22=-good.a22)
    result = routine.center_on_dot(arm, cam, inverted, tolerance_mm=0.02, max_iterations=6, max_correction_mm=3.0)
    iter_residuals = [round(it["residual_mm"], 4) for it in result.iterations]
    print(f"    aborted_reason: {result.aborted_reason!r}")
    print(f"    iterations: {iter_residuals}")
    check(
        "7. sign-inverted transform aborts via divergence guard",
        (not result.converged) and result.aborted_reason is not None,
        f"aborted_reason={result.aborted_reason!r}",
    )
    return result.aborted_reason


def test_8_max_correction_guard():
    g = _identity_g(scale=40.0)
    true_xy = (100.0, 200.0)
    start_offset_mm = (5.0, 0.0)
    arm = FakeArm([true_xy[0] + start_offset_mm[0], true_xy[1] + start_offset_mm[1], 50.0, 180.0, 0.0, 90.0])
    cam = PhysicalFakeCamera(
        arm, g_matrix=g,
        nominal_targets={"A1": true_xy}, true_targets={"A1": true_xy},
    )
    transform = frames.PixelToArm(
        a11=g[0, 0], a12=g[0, 1], a21=g[1, 0], a22=g[1, 1],
        um_per_px_x=1000.0 / g[0, 0], um_per_px_y=1000.0 / g[1, 1],
        derived_at="test", probe_step_mm=0.1, residual_px=0.0,
    )
    result = routine.center_on_dot(arm, cam, transform, tolerance_mm=0.02, max_iterations=4, max_correction_mm=3.0)
    check(
        "8. max_correction_mm guard aborts without moving (5mm start)",
        (not result.converged) and len(arm.history) == 0 and result.aborted_reason is not None,
        f"n_moves={len(arm.history)} aborted_reason={result.aborted_reason!r}",
    )


# ---------------------------------------------------------------------------
# Test 9 — autofocus_z
# ---------------------------------------------------------------------------
def test_9_autofocus_z():
    taught_z = 100.0
    z_peak = 98.7
    g = _identity_g(scale=40.0)
    true_xy = (100.0, 200.0)
    arm = FakeArm([true_xy[0], true_xy[1], taught_z, 180.0, 0.0, 90.0])
    cam = PhysicalFakeCamera(
        arm, g_matrix=g, nominal_targets={"A1": true_xy}, true_targets={"A1": true_xy},
        z_peak=z_peak, z_sigma=0.6, render_dot_overlay=False,
    )
    focus = routine.autofocus_z(arm, cam, taught_z=taught_z, span_mm=3.0, coarse_step_mm=0.25, fine_step_mm=0.05)
    commanded_zs = [h[2] for h in arm.history]
    never_above = all(z <= taught_z + 1e-9 for z in commanded_zs)
    within_step = abs(focus.best_z - z_peak) <= 0.05 + 1e-9
    check(
        "9a. autofocus_z finds peak within one fine_step_mm; never commands Z above taught_z",
        within_step and never_above and not focus.at_ceiling,
        f"best_z={focus.best_z:.4f} target={z_peak} never_above={never_above} at_ceiling={focus.at_ceiling}",
    )

    # at_ceiling path: peak sits at (or above, unreachable) taught_z
    arm2 = FakeArm([true_xy[0], true_xy[1], taught_z, 180.0, 0.0, 90.0])
    cam2 = PhysicalFakeCamera(
        arm2, g_matrix=g, nominal_targets={"A1": true_xy}, true_targets={"A1": true_xy},
        z_peak=taught_z + 5.0, z_sigma=0.6, render_dot_overlay=False,
        # peak unreachable above ceiling -> monotonic decreasing in sweep range
    )
    focus2 = routine.autofocus_z(arm2, cam2, taught_z=taught_z, span_mm=3.0, coarse_step_mm=0.25, fine_step_mm=0.05)
    commanded_zs2 = [h[2] for h in arm2.history]
    never_above2 = all(z <= taught_z + 1e-9 for z in commanded_zs2)
    check(
        "9b. autofocus_z sets at_ceiling=True when peak is at taught_z",
        focus2.at_ceiling and never_above2,
        f"best_z={focus2.best_z:.4f} at_ceiling={focus2.at_ceiling} never_above={never_above2}",
    )


# ---------------------------------------------------------------------------
# Test 10 — calibrate_plate end-to-end
# ---------------------------------------------------------------------------
def test_10_calibrate_plate():
    taught_a1 = (366.560883, 587.752625, 100.0, 179.995035, -0.002521, 90.151357)
    true_translation = (0.30, -0.20)
    true_rotation_deg = 1.0  # kept small: at a 99mm baseline, rotation induces a lateral
    # offset of ~99*sin(theta) at A12 that must stay under center_on_dot's
    # max_correction_mm guard for the fake loop to converge without a spiral search
    true_scale = 1.005
    z_peak = taught_a1[2] - 0.3

    a1_true = (taught_a1[0] + true_translation[0], taught_a1[1] + true_translation[1])
    theta = math.radians(true_rotation_deg)
    nominal_offset = np.array([-store.NOMINAL_A1_TO_A12_MM, 0.0])
    rot = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
    a12_offset = true_scale * (rot @ nominal_offset)
    a12_true = (a1_true[0] + a12_offset[0], a1_true[1] + a12_offset[1])

    nominal_targets = {
        "A1": (taught_a1[0], taught_a1[1]),
        "A12": (taught_a1[0] - store.NOMINAL_A1_TO_A12_MM, taught_a1[1]),
    }
    true_targets = {"A1": a1_true, "A12": a12_true}

    g = _identity_g(scale=40.0)
    arm = FakeArm(list(taught_a1))
    cam = PhysicalFakeCamera(
        arm, g_matrix=g, nominal_targets=nominal_targets, true_targets=true_targets,
        z_peak=z_peak, z_sigma=0.6,
    )
    transform = frames.PixelToArm(
        a11=g[0, 0], a12=g[0, 1], a21=g[1, 0], a22=g[1, 1],
        um_per_px_x=1000.0 / g[0, 0], um_per_px_y=1000.0 / g[1, 1],
        derived_at="test", probe_step_mm=0.1, residual_px=0.0,
    )

    calibration = routine.calibrate_plate(arm, cam, transform, taught_a1=taught_a1)
    trans_err = math.hypot(
        calibration.translation_mm[0] - true_translation[0],
        calibration.translation_mm[1] - true_translation[1],
    )
    rot_err = abs(calibration.rotation_deg - true_rotation_deg)
    check(
        "10. calibrate_plate recovers translation/rotation within 0.05mm/0.05deg",
        trans_err <= 0.05 and rot_err <= 0.05,
        f"trans_err={trans_err:.4f}mm rot_err={rot_err:.4f}deg scale={calibration.scale_along_row:.4f}",
    )


# ---------------------------------------------------------------------------
# Test 11 — store.well_pose
# ---------------------------------------------------------------------------
def test_11_well_pose_roundtrip():
    a1_pose = (366.560883, 587.752625, 188.366989, 179.995035, -0.002521, 90.151357)
    cal_zero = store.WellCalibration(
        a1_pose=a1_pose, a12_pose=a1_pose, taught_a1_pose=a1_pose,
        translation_mm=(0.0, 0.0), rotation_deg=0.0, scale_along_row=1.0,
        residual_mm=0.0, calibrated_at="test", n_iterations=0,
    )
    x_a1, y_a1 = store.well_pose(cal_zero, row=0, col=1)
    x_a12, y_a12 = store.well_pose(cal_zero, row=0, col=12)
    ok_a1 = math.isclose(x_a1, a1_pose[0], abs_tol=1e-9) and math.isclose(y_a1, a1_pose[1], abs_tol=1e-9)
    ok_a12 = (math.isclose(x_a12, a1_pose[0] - 99.0, abs_tol=1e-9)
              and math.isclose(y_a12, a1_pose[1], abs_tol=1e-9))
    check("11a. well_pose zero-rotation reproduces nominal grid (A1, A12)", ok_a1 and ok_a12,
          f"A1={(x_a1, y_a1)} A12={(x_a12, y_a12)}")

    cal_rot = dataclasses.replace(cal_zero, rotation_deg=10.0, scale_along_row=1.0)
    x_c9, y_c9 = store.well_pose(cal_rot, row=2, col=9)  # row C (index 2), col 9
    dx_nom, dy_nom = -9.0 * 8, 9.0 * 2
    theta = math.radians(10.0)
    dx_hand = math.cos(theta) * dx_nom - math.sin(theta) * dy_nom
    dy_hand = math.sin(theta) * dx_nom + math.cos(theta) * dy_nom
    expected = (a1_pose[0] + dx_hand, a1_pose[1] + dy_hand)
    ok_rot = math.isclose(x_c9, expected[0], abs_tol=1e-9) and math.isclose(y_c9, expected[1], abs_tol=1e-9)
    check("11b. well_pose known rotation matches hand-computed", ok_rot, f"got={(x_c9, y_c9)} expected={expected}")


# ---------------------------------------------------------------------------
# Test 12 — teach.capture_pose
# ---------------------------------------------------------------------------
class FakeTeachArm:
    def __init__(self, pose, angles):
        self._pose = pose
        self._angles = angles

    def get_position(self):
        return list(self._pose)

    def get_servo_angle(self):
        return list(self._angles)


def test_12_capture_pose():
    with tempfile.TemporaryDirectory() as td:
        target = Path(td) / "workspace.json"
        seed_data = {
            "locations": {
                "home": {
                    "name": "home", "pose": {"x": 0, "y": 0, "z": 0, "roll": 0, "pitch": 0, "yaw": 0, "units": "mm_deg"},
                    "description": "seed", "created_at": "2020-01-01T00:00:00",
                    "metadata": {"joint_angles_deg": [0] * 7, "joint_count": 7,
                                 "joints_captured_at": "2020-01-01T00:00:00", "joints_source": "seed"},
                }
            },
            "routes": {}, "protocols": {},
        }
        target.write_text(json.dumps(seed_data, indent=2))

        arm = FakeTeachArm(
            pose=[366.560883, 587.752625, 188.366989, 179.995035, -0.002521, 90.151357],
            angles=[30.174364, 27.848327, 21.846079, 76.121969, -12.882498, 50.708083, -32.397899],
        )
        teach.capture_pose(arm, "microscope_test", "test capture", path=target,
                                    joints_source="fake arm (unit test)")

        data = json.loads(target.read_text())
        loc = data["locations"]["microscope_test"]
        schema_ok = (
            set(loc.keys()) >= {"name", "pose", "description", "created_at", "metadata"}
            and set(loc["pose"].keys()) == {"x", "y", "z", "roll", "pitch", "yaw", "units"}
            and set(loc["metadata"].keys()) >= {"joint_angles_deg", "joint_count", "joints_captured_at", "joints_source"}
            and loc["metadata"]["joint_count"] == 7
            and "home" in data["locations"]  # original entry preserved
        )
        backups = list(Path(td).glob("workspace.json.bak.*"))
        check(
            "12. teach.capture_pose writes matching schema, preserves original, backs up",
            schema_ok and len(backups) == 1,
            f"schema_ok={schema_ok} n_backups={len(backups)} entry_keys={sorted(loc.keys())}",
        )


def main():
    residuals = test_6_center_on_dot_converges()
    sign_reason = test_7_sign_inverted_aborts()
    test_2_recover_centre_clean()
    test_3_recover_centre_noisy()
    test_4_not_found_cases()
    test_5_solve_from_probes()
    test_8_max_correction_guard()
    test_9_autofocus_z()
    test_10_calibrate_plate()
    test_11_well_pose_roundtrip()
    test_12_capture_pose()

    print()
    print(f"CONVERGENCE_RESIDUALS={residuals}")
    print(f"SIGN_GUARD_REASON={sign_reason!r}")
    print()
    if FAILURES:
        print(f"RESULT: FAIL ({len(FAILURES)} failed: {FAILURES})")
        raise SystemExit(1)
    print("RESULT: ALL PASS")


if __name__ == "__main__":
    main()
