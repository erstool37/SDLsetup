#!/usr/bin/env python3
"""The size/shape prior rejects the 2026-08-04 false positives.

GROUND TRUTH THAT DID NOT COME FROM THE CODE. On 2026-08-04 the first live dot
scan ran `detect(mode="dark")` over 28 frames of the tray under the objective.
The operator confirmed there was no dot in any of them -- the tray's metal top
surface was in focus, and the dots sit at the bottom of the wells. The
discriminant nevertheless fired on 21 of the 28, latching onto a machined bore
and the dark well wall. No amount of tuning inside the discriminant fixes that:
a large unlit region inside the illuminated hull is exactly what it looks for.

What separates a dot from a bore is that the caller knows how big the dot should
be. `MarkPrior` carries that, and this suite pins two things:

  1. Synthetic, so it runs anywhere: a disc of the expected size matches; one at
     half that size does not; a ring of the right radius but hollow does not.
     The ring is the one that matters -- radius alone is NOT sufficient, and the
     real data proves it: `scan_020` holds a blob at radius 462.7 px against an
     expected 444 px (ratio 1.04, squarely in band) that is only rejected
     because its fill ratio is 0.34.
  2. Regression against the real frames, which SKIP LOUDLY when absent --
     `dataset/` is gitignored, so they are not guaranteed to be on any machine.

No hardware, no motion, no network.

    python dev/tests/test_marker_prior.py
"""
from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

import numpy as np  # noqa: E402

from tools.microscope import marker  # noqa: E402

#: The run with no dot in any frame. Gitignored; absence is a skip, not a pass.
REAL = REPO / "dataset" / "captures" / "dot_scan" / "20260804-172853"

#: Measured 2026-08-04 by phase correlation at the focus plane: 883.1 px/mm
#: along arm X, 892.3 along arm Y. The mean is what a round mark sees.
PX_PER_MM = (883.1 + 892.3) / 2.0
DOT_MM = 1.0

failures = 0


def ok(condition: bool, message: str, detail: str = "") -> None:
    global failures
    print("[test] %s: %s%s" % ("PASS" if condition else "FAIL", message,
                               "  (%s)" % detail if detail else ""))
    if not condition:
        failures += 1


prior = marker.MarkPrior(diameter_mm=DOT_MM, px_per_mm=PX_PER_MM)

# -- the prior's own arithmetic --------------------------------------------
ok(abs(prior.radius_px - 443.9) < 0.1, "1.0 mm at 887.7 px/mm is a 444 px radius",
   "%.1f" % prior.radius_px)
lo, hi = prior.band_px
ok(abs(lo - 310.7) < 0.1 and abs(hi - 577.0) < 0.1,
   "the +/-30% band is 311-577 px", "%.0f-%.0f" % (lo, hi))

for bad in ({"diameter_mm": 0.0, "px_per_mm": 800.0},
            {"diameter_mm": 1.0, "px_per_mm": -1.0},
            {"diameter_mm": 1.0, "px_per_mm": 800.0, "radius_tol_frac": 1.5}):
    try:
        marker.MarkPrior(**bad)
        ok(False, "MarkPrior rejects %r" % bad)
    except marker.MarkerError:
        ok(True, "MarkPrior rejects %r" % bad)


# -- synthetic candidates ---------------------------------------------------
def candidate(radius: float, fill: float, disagreement=None, fit_radius=None):
    """The subset of a `_describe` record the prior actually reads."""
    rec = {"equiv_radius_px": radius, "fill_ratio": fill,
           "disagreement_px": disagreement, "circle_fit": None}
    if fit_radius is not None:
        rec["circle_fit"] = {"radius_px": fit_radius}
    return rec


v = marker._judge_against_prior(candidate(444.0, 0.78, 5.0, 444.0), prior)
ok(v["matches_prior"], "a filled disc of the expected size matches")

v = marker._judge_against_prior(candidate(222.0, 0.78, 5.0, 222.0), prior)
ok(not v["matches_prior"] and "outside" in v["prior_reason"],
   "a disc at half the expected radius does not match", v["prior_reason"][:60])

# The load-bearing case: right size, wrong shape.
v = marker._judge_against_prior(candidate(444.0, 0.34, 5.0, 444.0), prior)
ok(not v["matches_prior"] and "fill ratio" in v["prior_reason"],
   "a hollow outline at the expected radius does NOT match on radius alone",
   v["prior_reason"][:60])

v = marker._judge_against_prior(candidate(444.0, 0.78, 300.0, 444.0), prior)
ok(not v["matches_prior"] and "disagree" in v["prior_reason"],
   "centroid and circle fit far apart does not match", v["prior_reason"][:60])

# A clipped mark loses area but keeps its fitted radius -- the reason the prior
# is allowed to read either. Equivalent radius 250 px is out of band; the fit is
# in band, so this must still match.
v = marker._judge_against_prior(candidate(250.0, 0.78, 5.0, 440.0), prior)
ok(v["matches_prior"] and v["prior_radius_source"] == "circle_fit_radius",
   "a clipped mark matches on its fitted radius", v["prior_radius_source"])


# -- a drawn disc through the whole detect() path ---------------------------
def frame_with_disc(radius: int) -> Path:
    """A lit field with one black disc in it, written to a temp JPEG."""
    import tempfile

    import cv2
    h, w = 2048, 3072
    img = np.zeros((h, w, 3), np.uint8)
    cv2.circle(img, (w // 2, h // 2), 900, (200, 200, 200), -1)   # lit field
    cv2.circle(img, (w // 2, h // 2), radius, (10, 10, 10), -1)   # the mark
    p = Path(tempfile.mkdtemp()) / ("disc_%d.jpg" % radius)
    cv2.imwrite(str(p), img, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
    return p


det = marker.detect(frame_with_disc(444), mode="dark", expect=prior)
ok(det["n_prior_matches"] == 1,
   "a drawn 444 px disc yields exactly one prior match",
   "n_prior_matches=%d of %d candidates" % (det["n_prior_matches"],
                                            det["n_candidates"]))
ok(det["candidates"] and det["candidates"][0]["matches_prior"],
   "the matching candidate sorts first")
ok("prior" in det and det["prior"]["expected_radius_px"] == 443.9,
   "the report carries the prior it was judged against")

det = marker.detect(frame_with_disc(150), mode="dark", expect=prior)
ok(det["n_prior_matches"] == 0, "a drawn 150 px disc yields no prior match",
   "n_candidates=%d" % det["n_candidates"])

# Additive: without a prior the report must not grow prior fields.
det = marker.detect(frame_with_disc(444), mode="dark")
ok("n_prior_matches" not in det and "prior" not in det,
   "without `expect` the report is unchanged")


# -- regression against the real false-positive run -------------------------
frames = sorted(REAL.glob("scan_*.jpg"))
if not frames:
    print()
    print("[skip] LOUD SKIP: the 2026-08-04 dot_scan frames are not on this "
          "machine (%s)." % REAL)
    print("[skip] The synthetic checks above ran; the regression that proves "
          "the prior kills REAL false positives did NOT.")
else:
    matched = examined = 0
    unchanged = 0
    for f in frames:
        plain = marker.detect(f, mode="dark")
        withp = marker.detect(f, mode="dark", expect=prior)
        unchanged += int(plain["n_candidates"] == withp["n_candidates"])
        matched += withp["n_prior_matches"]
        examined += len(withp["candidates"])
    ok(unchanged == len(frames),
       "the prior changes no candidate count", "%d/%d frames" % (unchanged, len(frames)))
    ok(matched == 0,
       "no candidate in a run with no dot matches the prior",
       "%d matches over %d candidates in %d frames" % (matched, examined, len(frames)))

print()
print("ALL PASS" if failures == 0 else "%d FAILURE(S)" % failures)
sys.exit(1 if failures else 0)
