#!/usr/bin/env python3
"""The focus metric still picks the plane the operator picked by eye.

This is the only ground truth in the repo that did not come from the code being
tested: on 2026-07-31 the operator looked at eleven frames and said +3.0 mm.
Every masked metric agrees; full-frame variance-of-Laplacian does not, which is
how the metric was found to be wrong.

No hardware. Reads `fixtures/a1_sweep_20260731/`, and SKIPS LOUDLY if it is
absent rather than passing quietly.

    python dev/tests/test_focus_known_answer.py
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

FIXTURE = Path(__file__).resolve().parent / "fixtures" / "a1_sweep_20260731"

#: What the operator picked, looking at the frames. Not a computed value.
OPERATOR_ANSWER_MM = 3.0

failures = 0


def ok(condition: bool, message: str, detail: str = "") -> None:
    global failures
    print("[test] %s: %s%s" % ("PASS" if condition else "FAIL", message,
                               f"  ({detail})" if detail else ""))
    if not condition:
        failures += 1


frames = sorted(FIXTURE.glob("coarse_*_z+*.jpg"))
if not frames:
    print(f"[test] SKIP: no fixture at {FIXTURE}")
    print("[test] SKIPPED -- the focus metric is UNVERIFIED in this checkout.")
    print("       See fixtures/README.md; the frames are not committed.")
    sys.exit(0)

from tools.microscope import autofocus as af  # noqa: E402
from tools.microscope import focus  # noqa: E402

heights = [float(re.search(r"z\+(\d+\.\d+)", f.name).group(1)) for f in frames]
ok(len(frames) == 11, "the fixture has all eleven heights", f"{len(frames)} frames")

print("\n--- masked: every metric must agree with the operator ---")
for method in focus.FOCUS_METHODS:
    samples = [focus.score_frame(f, method=method, masked=True) for f in frames]
    report = af.scan_report(samples, heights)
    ok(report["ok"] and report["best_z_mm"] == OPERATOR_ANSWER_MM,
       f"masked {method} picks {OPERATOR_ANSWER_MM:+.1f} mm",
       f"picked {report.get('best_z_mm')}, margin {report.get('margin')}")

print("\n--- the masked tenengrad margin is what makes the pick robust ---")
samples = [focus.score_frame(f, method="tenengrad", masked=True) for f in frames]
report = af.scan_report(samples, heights)
ok(report["margin"] > 3.0,
   "masked tenengrad beats the runner-up by more than 3x",
   f"margin {report['margin']}")
ok(all(s.usable for s in samples), "every frame in the sweep is scorable")

print("\n--- full-frame laplacian is the metric that was wrong ---")
full = [focus.score_frame(f, method="laplacian", masked=False) for f in frames]
full_report = af.scan_report(full, heights)
ok(full_report["best_z_mm"] != OPERATOR_ANSWER_MM,
   "full-frame laplacian still disagrees with the operator, as recorded",
   f"picks {full_report['best_z_mm']:+.1f} mm")
print("       (if this ever starts agreeing, the recorded reason for masking "
      "no longer holds -- re-check it rather than deleting the test)")

print("\n%s" % ("ALL PASS" if failures == 0 else "%d FAILURE(S)" % failures))
sys.exit(1 if failures else 0)
