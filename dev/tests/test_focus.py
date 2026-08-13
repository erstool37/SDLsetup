#!/usr/bin/env python3
"""Focus measurement and scan planning. Synthetic images; no hardware, no files.

Carried over from the retired ``autofocus._self_test``, which lived inside the
module it tested and therefore never ran on its own. It is a real test file now
because the split of measurement (``devices.microscope.focus``) from policy
(``procedures.autofocus``) is exactly the kind of change that can leave one half
importing a name the other half owned -- and did, once: the planning half lost
its numpy import in the move and every call raised NameError.

    python test/test_focus.py
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from tools.microscope.autofocus import (  # noqa: E402
    best_sample,
    fine_scan_values,
    parabolic_refine,
    plan_scan,
    scan_report,
)
from tools.microscope.focus import (  # noqa: E402
    BRIGHTNESS_MIN_MEAN,
    FOCUS_METHODS,
    brightness_guard,
    focus_mask,
    focus_score,
    score_frame,
)

failures = 0


def check(name: str, ok: bool, detail: str = "") -> None:
    global failures
    status = "PASS" if ok else "FAIL"
    if not ok:
        failures += 1
    print(f"[test] {status}: {name}" + (f"  ({detail})" if detail else ""), flush=True)


failures = 0

# --- Build a random checkerboard and blur it at increasing sigmas ---
rng = np.random.default_rng(42)
tile = 16
board = (np.indices((256, 256)).sum(axis=0) // tile % 2 * 200).astype(np.uint8)
noise = rng.integers(0, 56, size=board.shape, dtype=np.uint8)
sharp = cv2.add(board, noise)  # sharp, textured target

sigmas = [0.8, 2.0, 4.0, 8.0]
images = [sharp] + [
    cv2.GaussianBlur(sharp, (0, 0), sigmaX=s, sigmaY=s) for s in sigmas
]  # index 0 = sharpest, monotonically blurrier after

for method in FOCUS_METHODS:
    scores = [focus_score(img, method=method) for img in images]
    monotone = all(scores[i] > scores[i + 1] for i in range(len(scores) - 1))
    check(
        f"focus_score[{method}] ranks sharpest highest (monotone)",
        scores[0] == max(scores) and monotone,
        "scores=" + ", ".join(f"{s:.1f}" for s in scores),
    )

# --- BGR input path: color version of the same images must agree ---
bgr_sharp = cv2.cvtColor(sharp, cv2.COLOR_GRAY2BGR)
bgr_blur = cv2.cvtColor(images[-1], cv2.COLOR_GRAY2BGR)
check(
    "focus_score accepts BGR and still ranks sharp > blurred",
    focus_score(bgr_sharp) > focus_score(bgr_blur),
)

# --- Unreadable input must raise ValueError ---
try:
    focus_score("/nonexistent/definitely_missing_image_xyz.jpg")
    check("focus_score raises ValueError on unreadable path", False)
except ValueError:
    check("focus_score raises ValueError on unreadable path", True)
try:
    focus_score(sharp, method="bogus")
    check("focus_score raises ValueError on unknown method", False)
except ValueError:
    check("focus_score raises ValueError on unknown method", True)

# --- Brightness guard ---
dark = np.full((64, 64), 1, dtype=np.uint8)
lit = np.full((64, 64), 128, dtype=np.uint8)
ok_dark, mean_dark = brightness_guard(dark)
ok_lit, mean_lit = brightness_guard(lit)
check(
    "brightness_guard rejects dark, accepts lit",
    (not ok_dark) and ok_lit,
    f"dark_mean={mean_dark:.2f} lit_mean={mean_lit:.2f} "
    f"threshold={BRIGHTNESS_MIN_MEAN}",
)

# --- plan_scan: the production A1 scan geometry ---
plan = plan_scan(188.37, 218.37, 5.0, 1.0)
check(
    "plan_scan(188.37, 218.37, 5, 1) -> 7 coarse points ending at z_max",
    plan["n_coarse"] == 7
    and abs(plan["coarse_z"][0] - 188.37) < 1e-6
    and abs(plan["coarse_z"][-1] - 218.37) < 1e-6,
    f"coarse_z={plan['coarse_z']}",
)

# --- fine_scan_values: clamped to hard range ---
fine = fine_scan_values(218.37, 188.37, 218.37, 1.0, 2.5)
check(
    "fine_scan_values clamps to z_max",
    max(fine) <= 218.37 + 1e-9 and min(fine) >= 188.37 - 1e-9,
    f"fine={fine}",
)

# --- parabolic_refine: recover a known synthetic peak ---
true_peak = 205.3
zs = [200.0, 202.5, 205.0, 207.5, 210.0]
ss = [-(z - true_peak) ** 2 for z in zs]
est = parabolic_refine(zs, ss)
check(
    "parabolic_refine recovers synthetic peak",
    abs(est - true_peak) < 1e-6,
    f"est={est:.4f} true={true_peak}",
)
# boundary argmax -> no extrapolation beyond range
est_edge = parabolic_refine([1.0, 2.0, 3.0], [3.0, 2.0, 1.0])
check("parabolic_refine returns boundary z when peak at edge",
      est_edge == 1.0, f"est={est_edge}")

# ---------------------------------------------------------------------------
# the masked scorer, on a synthetic version of this rig's field
# ---------------------------------------------------------------------------

print("\n--- masked scoring vs the unlit surround ---")
import tempfile  # noqa: E402

rng2 = np.random.default_rng(7)
h, w = 400, 400
yy, xx = np.mgrid[0:h, 0:w]
lit = ((yy - h / 2) ** 2 + (xx - w / 2) ** 2) < (120 ** 2)   # illuminated disc
texture = rng2.integers(60, 200, size=(h, w)).astype(np.uint8)
field = np.where(lit, texture, 4).astype(np.uint8)           # median 4/255 outside

found = focus_mask(field)
check("mask found the lit region", found.ok and 0.0 < found.n_frac < 0.5,
      f"frac={found.n_frac}")
check("mask is smaller than the whole frame", found.n_frac < 1.0)

with tempfile.TemporaryDirectory() as td:
    sharp_p = Path(td) / "sharp.jpg"
    blur_p = Path(td) / "blur.jpg"
    cv2.imwrite(str(sharp_p), field)
    cv2.imwrite(str(blur_p), cv2.GaussianBlur(field, (0, 0), 4.0, 4.0))
    s_sharp = score_frame(sharp_p, method="tenengrad")
    s_blur = score_frame(blur_p, method="tenengrad")
    check("masked score ranks sharp above blurred",
          s_sharp.usable and s_blur.usable and s_sharp.score > s_blur.score,
          f"{s_sharp.score:.1f} vs {s_blur.score:.1f}")
    check("score_normalised is reported alongside the raw score",
          s_sharp.score_normalised is not None and s_sharp.score_normalised > 0)

    dark_p = Path(td) / "dark.jpg"
    cv2.imwrite(str(dark_p), np.full((h, w), 3, dtype=np.uint8))
    s_dark = score_frame(dark_p, method="tenengrad")
    check("an unlit frame is reported unusable, not scored",
          (not s_dark.usable) and s_dark.score is None and s_dark.reason)

    sat_p = Path(td) / "sat.jpg"
    cv2.imwrite(str(sat_p), np.full((h, w), 255, dtype=np.uint8))
    s_sat = score_frame(sat_p, method="tenengrad")
    check("a saturated frame is reported unusable",
          (not s_sat.usable) and "saturat" in s_sat.reason)

print("\n--- peak selection is the procedure's job, and reports its margin ---")


class _S:
    def __init__(self, score, usable=True, reason=""):
        self.score = score
        self.score_normalised = None if score is None else score / 100.0
        self.usable = usable
        self.reason = reason
        self.method = "tenengrad"
        self.masked = True


heights = [0.0, 1.0, 2.0, 3.0, 4.0]
samples = [_S(40.1), _S(43.9), _S(50.8), _S(352.4), _S(98.0)]
best = best_sample(samples)
check("best_sample picks the real peak", best is samples[3])
rep = scan_report(samples, heights)
check("scan_report agrees, and at the right height", rep["ok"] and rep["best_z_mm"] == 3.0)
check("scan_report reports the margin over the runner-up",
      abs(rep["margin"] - 352.4 / 98.0) < 1e-3, f"margin={rep['margin']}")
check("scan_report says what it ranked on", rep["ranked_on"] == "score")

unusable = [_S(None, usable=False, reason="too dark")] * 3
check("an all-unusable scan reports not-ok rather than picking one",
      best_sample(unusable) is None and not scan_report(unusable, [0.0, 1.0, 2.0])["ok"])
mixed = [_S(None, usable=False, reason="too dark"), _S(10.0), _S(5.0)]
rep2 = scan_report(mixed, [0.0, 1.0, 2.0])
check("an unusable sample is skipped, not scored as zero",
      rep2["ok"] and rep2["best_z_mm"] == 1.0 and len(rep2["skipped"]) == 1)

print("\n%s" % ("ALL PASS" if failures == 0 else "%d FAILURE(S)" % failures))
sys.exit(1 if failures else 0)
