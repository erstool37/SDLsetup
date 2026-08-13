"""
marker.py -- find the operator's alignment mark in a microscope frame.

Reports measurements and quality flags. It never decides where to move; that
belongs to the orchestrator (surface law: sensors report, they do not decide).

TWO WAYS TO LOCATE THE SAME MARK, AND WHY BOTH ARE REPORTED
-----------------------------------------------------------
`centroid`    -- the centre of mass of the detected mark pixels. Exact when the
                 whole mark is visible. Biased toward the visible side when part
                 of the mark falls outside the illuminated field.
`circle_fit`  -- a least-squares circle through the mark's true boundary, with
                 boundary points lying on the illumination edge excluded. Works
                 when the mark is partly cut off, but extrapolates, so it is
                 noisier and can be wrong if the mark is not round.

MEASURED ON THIS RIG, 2026-07-31 -- the reason both exist. With the black mark
partly sunk into the unlit rim, the two disagreed by 542 px (0.63 mm):

    centroid    offset (-31.8, -0.4) px   "centred"
    circle fit  offset (+510.6, -7.8) px  0.59 mm to the right

The centroid was wrong, and it was wrong in the confident direction: it said the
job was done. After correcting on the circle fit and bringing the mark fully
into view, the two closed to 108 px. **Their disagreement is therefore the
quality signal that matters** -- it is reported as `disagreement_px`, and a large
value means neither number should be trusted and the mark needs to be brought
further into the lit field first.

MODES
-----
"dark" -- a black mark. A global threshold is useless: measured, the frame's
    median is 4/255 because most of the field is unlit vignette, so a plain
    threshold selects the vignette rather than the mark. Instead find the
    illuminated disc by Otsu, take its convex hull (which bridges over the bite
    the mark takes out of the rim), and look for dark regions inside that hull.

"red" -- a red mark. Preferred when available: red ink is bright in R and dark
    in G/B, so it separates from both the vignette and the greyscale scratches
    in the plastic. But the plate plastic is itself warm-toned, so ABSOLUTE
    redness is high everywhere -- measured, a red-free frame yielded a false
    blob covering 24.6 % of the image. What identifies a real mark is a LOCAL
    excess of redness over its own surroundings, so a broadly blurred copy is
    subtracted first, and a minimum separation is required before any candidate
    is believed at all.

"blue" -- a blue mark, the same construction with B measured against
    max(R, G). Red and blue are mutually exclusive by construction: each is the
    excess of one channel over the MAXIMUM of the other two, so a pixel cannot
    carry an excess of both.

WHY `classify_dots` EXISTS -- THE MODE IS AN INPUT, NOT AN ANSWER
-----------------------------------------------------------------
`detect(mode=...)` is told which colour to look for and never reports which one
it saw, and "dark" fires on ANY dark region inside the lit hull. Measured with
cv2's BGR2GRAY: a saturated red dot greys to 76/255 and a saturated blue dot to
29/255, against a lit field near 200/255. So mode="dark" reports a red dot and a
blue dot as "the black mark" -- silently, with a confident centroid and an
"ok" quality flag.

That is a one-well error, and the geometry makes it unrecoverable by inspection.
The plate carries BLACK at A1, RED at B1, BLUE at A2; the well pitch is 9.0 mm
while the field of view is only about 2.84 x 2.38 mm, so the frame can never
show two dots at once and there is no second landmark to contradict the wrong
one. Land on B1, see red, be told it is black, and the centring loop converges
neatly onto a well 9 mm from the one it names.

`classify_dots` runs all three discriminants on one frame and reports which
fired, how hard (`margin`), and which claims cover the same blob (`conflicts`).
It attributes an overlapping dark-and-colour claim to the colour, because the
colour discriminants are specific and the dark one is not -- see `_disambiguate`
for the rule. It never picks a well, a move, or a "the" dot.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np

#: Minimum separation between the 99.9th percentile and the median of the local
#: red-excess channel before a red mark is believed present. Measured: without
#: it a red-free frame produced a false blob covering 24.6 % of the image.
RED_MIN_SEPARATION = 40.0

#: The same floor for blue. NOT independently measured -- the blue-excess
#: channel is built by the identical construction on the identical 8-bit sensor,
#: so red's measured noise floor is the best available estimate. Re-measure it
#: on a blue-free frame before trusting a blue detection that sits near it.
BLUE_MIN_SEPARATION = 40.0

#: Fraction of the frame a candidate must occupy to be considered at all.
MIN_AREA_FRAC = 0.002
MAX_AREA_FRAC = 0.35

# -- the size/shape prior --------------------------------------------------
#
# WHY IT EXISTS. On 2026-08-04 the first live dot scan fired `mode="dark"` on
# 21 of 28 frames and there was no dot in any of them: the tray's machined bore
# and the dark well wall are large unlit regions inside the illuminated hull,
# which is exactly what the "dark" discriminant looks for. The colour
# discriminants correctly reported nothing. Re-measured 2026-08-05 over the
# saved frames of that run:
#
#     candidates examined                        61
#     equivalent radii                    57 .. 692 px
#     candidates near the expected size            0
#
# The area gate alone admits radii 63..837 px, so it cannot separate a 1 mm dot
# from a 4 mm bore. But the caller ALREADY KNOWS how big the dot should look:
# the mark's diameter is a property of the tray and the pixel scale is measured
# every run. 1.0 mm at the measured 887.7 px/mm mean is a 444 px radius, and
# nothing in that failed run came near it under the bands below.
#
# The prior ANNOTATES; it does not filter. Every candidate is still reported,
# with `matches_prior` and a `prior_reason` saying which test it failed. Which
# candidate to steer on -- or to declare the frame empty -- stays with the
# phase script, per the surface law that sensors report and do not decide.

#: Half-width of the accepted radius band, as a fraction of the expected
#: radius. 0.30 admits 311-577 px for a 1 mm dot at 887.7 px/mm. Chosen to
#: straddle the plausible focus-blur and threshold spread while still excluding
#: every candidate in the 2026-08-04 run; it is a tolerance, not a measurement.
PRIOR_RADIUS_TOL_FRAC = 0.30

#: A filled disc occupies pi/4 = 0.785 of its bounding box. 0.65 leaves room for
#: threshold ragged edges. Measured: the 2026-08-04 candidates ran 0.18-0.77,
#: and the only ones above 0.65 were 64-70 px specks far outside the radius band.
PRIOR_MIN_FILL_RATIO = 0.65

#: Centroid-vs-circle-fit disagreement, as a fraction of the fitted radius,
#: beyond which the candidate is not a clean round mark. Consistent with the
#: 0.35 `partially_unlit` flag but stricter, because that flag describes a real
#: mark that is partly out of the light, whereas this asks whether it is a mark
#: at all. The 2026-08-04 large blobs disagreed by 228-1261 px.
PRIOR_MAX_DISAGREEMENT_FRAC = 0.25


@dataclass(frozen=True)
class MarkPrior:
    """How big the mark should look, in the frame it is being hunted in.

    `px_per_mm` is the scale MEASURED at the height the frame was taken at, not
    a stored constant: it moves with Z, and `pixel_scale` re-measures it each
    run. Passing a stale scale silently shifts the whole band.
    """

    diameter_mm: float
    px_per_mm: float
    radius_tol_frac: float = PRIOR_RADIUS_TOL_FRAC
    min_fill_ratio: float = PRIOR_MIN_FILL_RATIO
    max_disagreement_frac: float = PRIOR_MAX_DISAGREEMENT_FRAC

    def __post_init__(self) -> None:
        if self.diameter_mm <= 0 or self.px_per_mm <= 0:
            raise MarkerError(
                "MarkPrior needs a positive diameter_mm and px_per_mm, got "
                "%r and %r" % (self.diameter_mm, self.px_per_mm))
        if not 0 < self.radius_tol_frac < 1:
            raise MarkerError("radius_tol_frac must be in (0, 1), got %r"
                              % (self.radius_tol_frac,))

    @property
    def radius_px(self) -> float:
        return 0.5 * self.diameter_mm * self.px_per_mm

    @property
    def band_px(self) -> tuple[float, float]:
        r = self.radius_px
        return r * (1 - self.radius_tol_frac), r * (1 + self.radius_tol_frac)

    def as_dict(self) -> dict[str, Any]:
        lo, hi = self.band_px
        return {"diameter_mm": self.diameter_mm,
                "px_per_mm": round(self.px_per_mm, 1),
                "expected_radius_px": round(self.radius_px, 1),
                "radius_band_px": [round(lo, 1), round(hi, 1)],
                "min_fill_ratio": self.min_fill_ratio,
                "max_disagreement_frac": self.max_disagreement_frac}


def _judge_against_prior(cand: dict[str, Any],
                         prior: MarkPrior) -> dict[str, Any]:
    """Annotate one candidate. Returns the fields to merge into it.

    Size is checked against the equivalent radius AND, if the blob is partly
    cut off, the fitted circle's radius -- a clipped mark loses area but keeps
    its fitted radius, so demanding the equivalent radius alone would reject
    exactly the case `circle_fit` was built for. Verified this does not
    resurrect a false positive: of the 61 candidates in the 2026-08-04 run only
    2 had a fitted radius in band, and both failed on fill or disagreement.
    """
    lo, hi = prior.band_px
    equiv = float(cand["equiv_radius_px"])
    fit = (cand.get("circle_fit") or {}).get("radius_px")
    source, radius = "equiv_radius", equiv
    if not lo <= equiv <= hi and fit is not None and lo <= float(fit) <= hi:
        source, radius = "circle_fit_radius", float(fit)

    out: dict[str, Any] = {
        "prior_radius_source": source,
        "prior_radius_ratio": round(radius / prior.radius_px, 3),
    }

    if not lo <= radius <= hi:
        out["matches_prior"] = False
        out["prior_reason"] = (
            "%s %.0f px is outside the %.0f-%.0f px expected for a %.2f mm mark "
            "at %.1f px/mm" % (source, radius, lo, hi, prior.diameter_mm,
                               prior.px_per_mm))
        return out

    if float(cand["fill_ratio"]) < prior.min_fill_ratio:
        out["matches_prior"] = False
        out["prior_reason"] = (
            "fill ratio %.2f is below %.2f; a filled disc fills pi/4 = 0.79 of "
            "its bounding box, so this outline is not disc-shaped"
            % (cand["fill_ratio"], prior.min_fill_ratio))
        return out

    dis, fit_r = cand.get("disagreement_px"), fit
    if dis is not None and fit_r and float(dis) > prior.max_disagreement_frac * float(fit_r):
        out["matches_prior"] = False
        out["prior_reason"] = (
            "centroid and circle fit disagree by %.0f px, %.0f%% of the fitted "
            "radius (limit %.0f%%); the two estimates do not describe one round "
            "mark" % (dis, 100 * float(dis) / float(fit_r),
                      100 * prior.max_disagreement_frac))
        return out

    out["matches_prior"] = True
    out["prior_reason"] = ""
    return out

#: Circle-fit boundary points within this many pixels of the illumination edge
#: are dropped: they trace the edge of the light, not the edge of the mark.
ILLUM_EDGE_EXCLUSION_PX = 7

#: Modes driven by a local colour-excess channel, as opposed to "dark", which is
#: driven by absence of light inside the illuminated hull.
COLOUR_MODES = ("red", "blue")

#: Every mode `detect` accepts.
DETECT_MODES = ("dark",) + COLOUR_MODES

#: The dot colours the plate actually carries, and the discriminant that finds
#: each. "black" is the physical ink; "dark" is only the discriminant, and the
#: gap between those two words is the whole reason `classify_dots` exists.
MODE_FOR_COLOUR = {"black": "dark", "red": "red", "blue": "blue"}

#: Fixed report order, so two runs on the same frame compare line by line.
COLOUR_ORDER = ("black", "red", "blue")

#: What a below-floor colour reading is most likely to be instead. Red's is
#: measured (the warm plastic above); blue's states the principle only.
_ABSENT_NOTE_TAIL = {
    "red": "Warm-toned plastic is not a red mark.",
    "blue": "A cool white-balance cast is not a blue mark.",
}

#: How `margin` is defined per discriminant, carried in the report so a caller
#: reading the number does not have to guess its units or its meaning.
MARGIN_RULE = {
    "colour": ("(p99.9 - median) of the local colour-excess channel, minus that "
               "colour's separation floor. Positive means the colour fired above the "
               "floor; a value near zero is barely-above-noise, not a clear dot."),
    "dark": ("Otsu lit/unlit threshold minus the blob's mean grey level. Positive "
             "means the blob is darker than the split that defined the lit field. It "
             "does NOT mean the ink is black -- a saturated colour dot is dark too, "
             "which is what the disambiguation step is for."),
}

MARGIN_UNITS = {"colour": "8-bit colour-excess counts above the separation floor",
                "dark": "grey levels below the Otsu lit/unlit split"}


class MarkerError(ValueError):
    pass


def _min_separation(colour: str) -> float:
    """Read the floor at call time so raising one in a session actually bites."""
    return RED_MIN_SEPARATION if colour == "red" else BLUE_MIN_SEPARATION


def _gray(path: Path) -> np.ndarray:
    g = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if g is None:
        raise MarkerError("unreadable image: %s" % path)
    return g


def fit_circle(points: np.ndarray) -> dict[str, float] | None:
    """Algebraic least-squares circle through Nx2 points.

    Solves x^2 + y^2 + Dx + Ey + F = 0, which is LINEAR in (D, E, F), so there is
    a closed form and no starting guess. Centre = (-D/2, -E/2),
    radius = sqrt(cx^2 + cy^2 - F).

    Returns None when fewer than 12 points survive, or when the algebraic system
    is degenerate (collinear points), rather than returning a fabricated centre.
    """
    if points is None or len(points) < 12:
        return None
    x = points[:, 0].astype(float)
    y = points[:, 1].astype(float)

    # Collinearity guard. Least squares does not fail on collinear points -- it
    # happily returns an enormous circle through them, which reads as a
    # confident answer. Compare the two principal spreads of the point cloud:
    # a straight line has essentially no width.
    pts = np.column_stack([x, y])
    ev = np.linalg.eigvalsh(np.cov(pts - pts.mean(axis=0), rowvar=False))
    ev = np.sort(np.abs(ev))
    if ev[1] <= 0 or ev[0] / ev[1] < 1e-6:
        return None

    A = np.column_stack([x, y, np.ones(len(x))])
    b = -(x * x + y * y)
    try:
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    except np.linalg.LinAlgError:
        return None
    D, E, F = (float(v) for v in sol)
    cx, cy = -D / 2.0, -E / 2.0
    disc = cx * cx + cy * cy - F
    if disc <= 0:
        return None
    r = float(np.sqrt(disc))
    # A radius far larger than the data's own extent means the arc was too flat
    # to constrain a centre; the fit is an extrapolation, not a measurement.
    if r > 50.0 * float(np.sqrt(ev[1])):
        return None
    resid = float(np.std(np.hypot(x - cx, y - cy) - r))
    return {"cx_px": cx, "cy_px": cy, "radius_px": r,
            "residual_px": resid, "n_points": int(len(x))}


@dataclass(eq=False)  # it holds an image; array __eq__ does not return a bool
class _ColourExcess:
    """The local colour-excess channel for one colour, plus its own statistics.

    One object per colour so "red" and "blue" run the identical arithmetic
    instead of two hand-maintained near-copies drifting apart.
    """

    colour: str
    excess: np.ndarray
    p50: float
    p999: float
    peak: int
    floor: float

    @property
    def separation(self) -> float:
        """Peak above the field's own median -- the only evidence of a mark."""
        return self.p999 - self.p50

    @property
    def fires(self) -> bool:
        return self.separation >= self.floor

    @property
    def threshold(self) -> float:
        """Halfway from the median to the peak; meaningless unless `fires`."""
        return self.p50 + 0.5 * self.separation

    def mask(self) -> np.ndarray:
        """Candidate pixels. CLOSE first to seal the specular pinholes printed
        ink leaves, then OPEN to drop the speckle the closing joined up."""
        m = (self.excess >= self.threshold).astype(np.uint8)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
        return cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))


def _colour_excess(bgr: np.ndarray, colour: str, w: int, h: int) -> _ColourExcess:
    """Local excess of one channel over the maximum of the other two.

    The MAXIMUM, not the mean: it demands the colour beat both rivals, which is
    what makes red and blue mutually exclusive rather than merely different.

    The broad-blur subtraction is the part that matters. Absolute colour is high
    across the whole frame (warm plastic, lamp cast, white balance), so an
    absolute threshold selects the background; subtracting a heavily blurred copy
    of the same channel leaves only what is coloured RELATIVE TO ITS OWN
    SURROUNDINGS, which is what a printed dot is and a tint is not.
    """
    b, gr, r = (bgr[:, :, i].astype(np.int16) for i in range(3))
    if colour == "red":
        own, rivals = r, np.maximum(gr, b)
    elif colour == "blue":
        own, rivals = b, np.maximum(r, gr)
    else:
        raise MarkerError("no colour-excess channel for mode %r" % (colour,))
    colourness = cv2.GaussianBlur(
        np.clip(own - rivals, 0, 255).astype(np.uint8), (15, 15), 0)
    # sigma scaled to the frame, so the same code works at any sensor size: the
    # background it removes is anything smoother than ~1/24 of the long edge.
    broad = cv2.GaussianBlur(colourness, (0, 0), sigmaX=max(w, h) / 24.0)
    excess = cv2.subtract(colourness, broad)
    return _ColourExcess(colour=colour, excess=excess,
                         p50=float(np.percentile(excess, 50)),
                         p999=float(np.percentile(excess, 99.9)),
                         peak=int(excess.max()),
                         floor=_min_separation(colour))


def _lit_region(g: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
    """Illuminated disc, its convex hull, and the Otsu threshold used."""
    blur = cv2.GaussianBlur(g, (31, 31), 0)
    thr, _ = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    lit = (blur > thr).astype(np.uint8)
    n, lab, st, _ = cv2.connectedComponentsWithStats(lit, 8)
    if n < 2:
        raise MarkerError("no illuminated region in the frame; is the lamp on?")
    big = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    comp = (lab == big).astype(np.uint8)
    cnts, _ = cv2.findContours(comp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    hull = cv2.convexHull(max(cnts, key=cv2.contourArea))
    hullmask = np.zeros_like(comp)
    cv2.drawContours(hullmask, [hull], -1, 1, -1)
    return comp, hullmask, float(thr)


def _describe(mask: np.ndarray, g: np.ndarray, edge: np.ndarray | None,
              w: int, h: int) -> list[dict[str, Any]]:
    area = float(h * w)
    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
    out: list[dict[str, Any]] = []
    for i in range(1, n):
        a_i = int(stats[i, cv2.CC_STAT_AREA])
        if not (MIN_AREA_FRAC * area <= a_i <= MAX_AREA_FRAC * area):
            continue
        cx, cy = float(cents[i][0]), float(cents[i][1])
        bx, by = int(stats[i, cv2.CC_STAT_LEFT]), int(stats[i, cv2.CC_STAT_TOP])
        bw, bh = int(stats[i, cv2.CC_STAT_WIDTH]), int(stats[i, cv2.CC_STAT_HEIGHT])
        blob = (labels == i).astype(np.uint8)

        # boundary points that are NOT on the illumination edge
        cnts, _ = cv2.findContours(blob, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        pts = max(cnts, key=cv2.contourArea).reshape(-1, 2).astype(float)
        n_total = len(pts)
        if edge is not None and n_total:
            keep = np.array([edge[int(round(y)), int(round(x))] == 0 for x, y in pts])
            pts = pts[keep]
        circ = fit_circle(pts)

        rec: dict[str, Any] = {
            "centroid_px": [round(cx, 2), round(cy, 2)],
            "centroid_offset_px": [round(cx - w / 2.0, 1), round(cy - h / 2.0, 1)],
            "area_px": a_i, "area_frac": round(a_i / area, 5),
            "bbox": [bx, by, bw, bh],
            "equiv_radius_px": round(float((a_i / np.pi) ** 0.5), 1),
            "fill_ratio": round(float(a_i / (bw * bh)) if bw and bh else 0.0, 3),
            "touches_frame_border": bool(bx <= 1 or by <= 1 or bx + bw >= w - 1
                                         or by + bh >= h - 1),
            "boundary_points_total": int(n_total),
            "boundary_points_used": int(len(pts)),
            "mean_inside": round(float(g[labels == i].mean()), 1),
        }
        if circ is None:
            rec["circle_fit"] = None
            rec["circle_offset_px"] = None
            rec["disagreement_px"] = None
            rec["quality"] = "centroid_only"
            rec["reason"] = ("not enough clean boundary points for a circle fit (%d); the "
                             "centroid is the only estimate and may be biased if the mark "
                             "is partly unlit" % len(pts))
        else:
            rec["circle_fit"] = {k: round(v, 2) for k, v in circ.items()}
            rec["circle_offset_px"] = [round(circ["cx_px"] - w / 2.0, 1),
                                       round(circ["cy_px"] - h / 2.0, 1)]
            dis = float(np.hypot(circ["cx_px"] - cx, circ["cy_px"] - cy))
            rec["disagreement_px"] = round(dis, 1)
            # A large gap means part of the mark is outside the lit field, so the
            # centroid is pulled toward the visible side. Measured 542 px in that
            # state and 108 px once the mark was fully in view.
            if dis > 0.35 * circ["radius_px"]:
                rec["quality"] = "partially_unlit"
                rec["reason"] = ("centroid and circle fit disagree by %.0f px (%.0f%% of the "
                                 "fitted radius): part of the mark is outside the lit field. "
                                 "Trust the circle fit, move, and re-measure."
                                 % (dis, 100 * dis / circ["radius_px"]))
            elif circ["residual_px"] > 0.15 * circ["radius_px"]:
                rec["quality"] = "poor_circle"
                rec["reason"] = ("circle-fit residual %.0f px is %.0f%% of the radius; the "
                                 "mark may not be round. Prefer the centroid."
                                 % (circ["residual_px"],
                                    100 * circ["residual_px"] / circ["radius_px"]))
            else:
                rec["quality"] = "ok"
                rec["reason"] = ""
        out.append(rec)
    out.sort(key=lambda c: -c["area_px"])
    return out


def detect(path: Path, *, mode: str = "dark",
           expect: MarkPrior | None = None) -> dict[str, Any]:
    """Locate the alignment mark. Reports candidates; chooses none.

    `mode` names the DISCRIMINANT, never the answer: the caller asserts which
    colour to hunt for and gets no contradiction if the frame holds a different
    one. Use `classify_dots` when the colour is what is in question.

    `expect` supplies how big the mark should look (see :class:`MarkPrior`).
    With it, every candidate additionally carries `matches_prior` and a
    `prior_reason`, candidates that match sort first, and `n_prior_matches`
    says how many survived. Without it the behaviour is unchanged, which is why
    it is optional: `mode="dark"` on a machined tray is a false-positive
    generator, and `n_candidates` on its own has already been trusted once and
    should not be again.

    It still does not filter. A caller that wants "is the dot here" reads
    `n_prior_matches`; one that wants to know what else was in the frame still
    sees it.
    """
    if mode not in DETECT_MODES:
        raise MarkerError("mode must be one of %s, got %r"
                          % (", ".join(repr(m) for m in DETECT_MODES), mode))
    g = _gray(path)
    h, w = g.shape
    info: dict[str, Any] = {"mode": mode, "width": w, "height": h,
                            "centre_px": [w / 2.0, h / 2.0], "source": str(path)}

    if mode in COLOUR_MODES:
        bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if bgr is None:
            raise MarkerError("unreadable image: %s" % path)
        exc = _colour_excess(bgr, mode, w, h)
        info["%s_excess_p50" % mode] = round(exc.p50, 1)
        info["%s_excess_p999" % mode] = round(exc.p999, 1)
        info["%s_excess_max" % mode] = exc.peak
        if not exc.fires:
            info.update(threshold_used=None, n_candidates=0, candidates=[],
                        note=("no %s mark present: local %s excess peaks only %.1f above "
                              "its median (need %.0f). %s"
                              % (mode, mode, exc.separation, exc.floor,
                                 _ABSENT_NOTE_TAIL[mode])))
            return info
        info["threshold_used"] = round(float(exc.threshold), 1)
        cands = _describe(exc.mask(), g, None, w, h)
    else:
        comp, hullmask, thr = _lit_region(g)
        dark = ((hullmask == 1) & (comp == 0)).astype(np.uint8)
        dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((25, 25), np.uint8))
        cnts, _ = cv2.findContours(comp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        hull = cv2.convexHull(max(cnts, key=cv2.contourArea))
        edge = np.zeros_like(comp)
        cv2.drawContours(edge, [hull], -1, 1, ILLUM_EDGE_EXCLUSION_PX)
        info.update(threshold_used=round(thr, 1),
                    lit_area_frac=round(float(comp.sum()) / (h * w), 4),
                    hull_area_frac=round(float(hullmask.sum()) / (h * w), 4))
        cands = _describe(dark, g, edge, w, h)

    if expect is not None:
        for c in cands:
            c.update(_judge_against_prior(c, expect))
        # Matching candidates first, then closest to the expected size. Within
        # the non-matching tail the original largest-first order is preserved,
        # so a frame read without a prior and one read with it list the same
        # blobs in a comparable order.
        cands.sort(key=lambda c: (not c["matches_prior"],
                                  abs(c["prior_radius_ratio"] - 1.0)
                                  if c["matches_prior"] else 0.0))
        info["prior"] = expect.as_dict()
        info["n_prior_matches"] = sum(1 for c in cands if c["matches_prior"])

    info["n_candidates"] = len(cands)
    info["candidates"] = cands[:6]
    return info


def best_offset_px(candidate: dict[str, Any]) -> tuple[list[float], str]:
    """Which of the two offsets to steer on, and why.

    Not a policy buried in the detector -- the caller can ignore it -- but the
    rule is worth having in one place: when the mark is partly unlit, the
    centroid is biased and the circle fit is the honest estimate; otherwise the
    centroid is both exact and less noisy.
    """
    q = candidate.get("quality")
    if q == "partially_unlit" and candidate.get("circle_offset_px"):
        return candidate["circle_offset_px"], "circle_fit (mark partly outside the lit field)"
    if q == "poor_circle" or not candidate.get("circle_offset_px"):
        return candidate["centroid_offset_px"], "centroid (circle fit unreliable)"
    return candidate["centroid_offset_px"], "centroid"


#: The disambiguation rule, quoted verbatim into every conflict record so the
#: caller reads which way an ambiguity went and on what grounds -- rather than
#: finding out from a well 9 mm away.
DARK_YIELDS_TO_COLOUR = (
    "a dark claim and a colour claim covering the same blob are attributed to the "
    "colour: mode='dark' fires on ANY dark region inside the lit hull, and a saturated "
    "red dot (grey 76) or blue dot (grey 29) is dark against a lit field near 200, "
    "while no black ink produces a local excess of one channel over the other two")

#: Red against blue is NOT resolved here. By construction each is an excess over
#: the maximum of the other two channels, so overlapping red and blue claims mean
#: the frame is not what this rig is set up to see -- a caller decision, not a
#: detector one.
COLOUR_VS_COLOUR_UNRESOLVED = (
    "two colour discriminants claim the same blob; neither is more specific than the "
    "other, so both stay reported and the caller decides")


def blob_overlap(a: dict[str, Any], b: dict[str, Any]) -> dict[str, Any]:
    """Do two candidate records describe the same physical blob?

    Both tests must pass: their bounding boxes must intersect, AND their
    centroids must lie within the sum of their equivalent radii. Two claims on
    one dot satisfy both trivially; a bbox that merely brackets a distant blob
    fails the second. Reports the metrics as well as the verdict, because a
    marginal overlap is exactly the case a caller should look at itself.

    Distinct dots cannot collide here in any case: the field of view is ~2.84 mm
    and the well pitch is 9.0 mm, so one frame never holds two of them.
    """
    ax, ay, aw, ah = a["bbox"]
    bx, by, bw, bh = b["bbox"]
    ix = max(0, min(ax + aw, bx + bw) - max(ax, bx))
    iy = max(0, min(ay + ah, by + bh) - max(ay, by))
    inter = float(ix * iy)
    union = float(aw * ah + bw * bh) - inter
    dist = float(np.hypot(a["centroid_px"][0] - b["centroid_px"][0],
                          a["centroid_px"][1] - b["centroid_px"][1]))
    reach = float(a["equiv_radius_px"] + b["equiv_radius_px"])
    return {"bbox_iou": round(inter / union, 3) if union > 0 else 0.0,
            "centroid_distance_px": round(dist, 1),
            "reach_px": round(reach, 1),
            "overlapping": bool(inter > 0 and dist <= reach)}


def _candidates(det: dict[str, Any] | None) -> list[dict[str, Any]]:
    return list(det["candidates"]) if det else []


def _margin(colour: str, det: dict[str, Any] | None,
            cand: dict[str, Any] | None) -> float | None:
    """How hard this discriminant fired, measured against its own threshold.

    Signed on purpose: a negative colour margin is the useful statement "this
    colour was looked for and was below its noise floor by this much", which a
    bare found=False cannot distinguish from "barely missed".
    """
    if det is None:
        return None
    if MODE_FOR_COLOUR[colour] in COLOUR_MODES:
        sep = det["%s_excess_p999" % colour] - det["%s_excess_p50" % colour]
        return round(sep - _min_separation(colour), 1)
    if cand is None or det.get("threshold_used") is None:
        return None
    return round(float(det["threshold_used"]) - float(cand["mean_inside"]), 1)


def _record(colour: str, det: dict[str, Any] | None, err: str | None,
            candidates: list[dict[str, Any]], suppressed_by: str | None,
            n_attributed: int) -> dict[str, Any]:
    """One colour's line in the report.

    `found` is the answer AFTER attribution; `discriminant_fired` is the raw
    signal before it. They differ exactly when this colour's blob was taken by a
    more specific discriminant, and `suppressed_by` names the taker.
    """
    kind = "colour" if MODE_FOR_COLOUR[colour] in COLOUR_MODES else "dark"
    cand = candidates[0] if candidates else None
    rec: dict[str, Any] = {
        "colour": colour,
        "mode": MODE_FOR_COLOUR[colour],
        "found": cand is not None,
        "discriminant_fired": bool(det and det["n_candidates"]),
        "suppressed_by": suppressed_by,
        "n_attributed_to_colour": n_attributed,
        "centroid_px": cand["centroid_px"] if cand else None,
        "offset_px": None,
        "offset_source": None,
        "area_px": cand["area_px"] if cand else None,
        "area_frac": cand["area_frac"] if cand else None,
        "quality": cand["quality"] if cand else None,
        "margin": _margin(colour, det, cand),
        "margin_units": MARGIN_UNITS[kind],
        "margin_rule": MARGIN_RULE[kind],
        "n_candidates": det["n_candidates"] if det else 0,
        "candidate": cand,
        "candidates": candidates,
        "error": err,
    }
    if cand is not None:
        rec["offset_px"], rec["offset_source"] = best_offset_px(cand)

    if err is not None:
        rec["reason"] = "this discriminant could not run on this frame: %s" % err
    elif suppressed_by is not None and cand is None:
        rec["reason"] = ("the dark discriminant fired, but its blob is attributed to %s. %s"
                         % (suppressed_by, DARK_YIELDS_TO_COLOUR))
    elif cand is None:
        rec["reason"] = det.get("note", "") if det else ""
        if not rec["reason"]:
            rec["reason"] = ("no dark region inside the illuminated hull survived the "
                             "area gate (%.1f%%-%.0f%% of the frame)"
                             % (100 * MIN_AREA_FRAC, 100 * MAX_AREA_FRAC))
    else:
        rec["reason"] = cand.get("reason", "")
    return rec


def _attribute_dark(dark_cands: list[dict[str, Any]],
                    colour_records: dict[str, Any]) -> tuple:
    """Hand every dark blob that a colour also claims over to that colour.

    Nothing is discarded: an attributed blob leaves the dark record but travels
    whole inside its conflict entry, so the caller can see the reading that was
    reinterpreted rather than being told it never existed.
    """
    conflicts: list[dict[str, Any]] = []
    survivors: list[dict[str, Any]] = []
    claimed_largest = None
    for rank, cand in enumerate(dark_cands):
        taken_by = None
        for cname in COLOUR_MODES:
            other = colour_records[cname]["candidate"]
            if other is None:
                continue
            ov = blob_overlap(cand, other)
            if not ov["overlapping"]:
                continue
            taken_by = cname
            entry = {"colours": ["black", cname], "resolved_to": cname,
                     "rule": DARK_YIELDS_TO_COLOUR, "black_candidate": cand,
                     "%s_centroid_px" % cname: other["centroid_px"]}
            entry.update(ov)
            conflicts.append(entry)
            break
        if taken_by is None:
            survivors.append(cand)
        elif rank == 0:
            claimed_largest = taken_by
    return survivors, claimed_largest, conflicts


def _colour_conflicts(records: dict[str, Any]) -> list[dict[str, Any]]:
    red, blue = records["red"]["candidate"], records["blue"]["candidate"]
    if red is None or blue is None:
        return []
    ov = blob_overlap(red, blue)
    if not ov["overlapping"]:
        return []
    entry = {"colours": ["red", "blue"], "resolved_to": None,
             "rule": COLOUR_VS_COLOUR_UNRESOLVED,
             "red_centroid_px": red["centroid_px"],
             "blue_centroid_px": blue["centroid_px"]}
    entry.update(ov)
    return [entry]


def classify_dots(path: Path, *, expect: MarkPrior | None = None) -> dict[str, Any]:
    """Run all three discriminants on one frame and report which colour is there.

    The counterpart to `detect`, which is TOLD the colour. Returns, per colour:
    whether a dot was found, its centroid, its signed pixel offset from the frame
    centre (via `best_offset_px`, same rule the centring loop steers on), its
    area, its quality flag, and a signed `margin` saying how far the discriminant
    cleared its own threshold -- so "clearly red" and "barely above noise" are
    distinguishable numbers rather than the same boolean.

    Overlapping claims are attributed by `DARK_YIELDS_TO_COLOUR` and every
    overlap is listed in `conflicts`, resolved or not. Deciding what to do about
    a conflict, and which well the frame is over, stays with the caller.

    NOT FINDING ANYTHING IS THE NORMAL CASE, NOT AN ERROR. With a ~2.84 mm field
    and a 9.0 mm pitch there is roughly 6 mm of blank plate between neighbouring
    dots, so most positions during a search show nothing: that comes back as
    `present == []` with every record found=False. A discriminant that cannot run
    on this particular frame -- "dark" needs an illuminated hull, and an unlit
    frame has none -- records its own `error` and leaves the other two alone.

    `expect` is passed through to every discriminant (see :class:`MarkPrior`).
    It is worth supplying whenever the dot's size is known: the "dark"
    discriminant fired on 21 of 28 dot-free frames on 2026-08-04 because a
    machined bore is also a dark region, and the size prior is what tells the
    two apart. Each record then carries `n_prior_matches`, and the top-level
    report carries `prior`.

    Raises MarkerError only when the path itself cannot be read, which is a
    caller bug and must not be reported as an empty field of view.
    """
    path = Path(path)
    g = _gray(path)
    h, w = g.shape

    # Three separate detect() calls re-read the file, which costs a few hundred
    # ms on a 2448x2048 frame. Deliberate: the alternative is a parallel code
    # path whose candidates could differ from what detect() reports for the same
    # frame, and a classifier that disagrees with the detector is worse than a
    # slow one.
    dets: dict[str, Any] = {}
    errs: dict[str, Any] = {}
    for colour in COLOUR_ORDER:
        try:
            dets[colour] = detect(path, mode=MODE_FOR_COLOUR[colour],
                                  expect=expect)
            errs[colour] = None
        except MarkerError as exc:
            dets[colour], errs[colour] = None, str(exc)

    records: dict[str, Any] = {}
    for colour in COLOUR_MODES:
        records[colour] = _record(colour, dets[colour], errs[colour],
                                  _candidates(dets[colour]), None, 0)

    dark_cands = _candidates(dets["black"])
    survivors, claimed, conflicts = _attribute_dark(dark_cands, records)
    records["black"] = _record("black", dets["black"], errs["black"], survivors,
                               claimed, len(dark_cands) - len(survivors))
    conflicts = conflicts + _colour_conflicts(records)

    present = [c for c in COLOUR_ORDER if records[c]["found"]]
    if present:
        shown = ", ".join(
            "%s (margin %s)" % (c, "n/a" if records[c]["margin"] is None
                                else "%.1f" % records[c]["margin"])
            for c in present)
        note = "found: %s" % shown
        if conflicts:
            note += "; %d overlapping claim(s), see conflicts" % len(conflicts)
    else:
        note = ("no dot in this frame. This is the normal reading over blank plate: "
                "the field of view is about 2.84 x 2.38 mm and the well pitch is "
                "9.0 mm, so most positions between two dots show nothing at all.")

    out = {"source": str(path), "width": w, "height": h,
           "centre_px": [w / 2.0, h / 2.0],
           "colours": {c: records[c] for c in COLOUR_ORDER},
           "present": present,
           "n_present": len(present),
           "conflicts": conflicts,
           "unresolved_conflicts": sum(1 for c in conflicts if c["resolved_to"] is None),
           "note": note}

    if expect is not None:
        # `present` is deliberately left alone: it means "the discriminant
        # fired", which is what every existing caller already reads it as.
        # `prior_matched` is the stricter question -- "and it is the right size
        # and shape" -- reported beside it rather than silently redefining it.
        # A caller hunting a known dot should steer on `prior_matched`; the gap
        # between the two lists is the false-positive rate, and keeping both
        # visible is what makes that measurable instead of invisible.
        for colour in COLOUR_ORDER:
            cands = records[colour].get("candidates") or []
            records[colour]["n_prior_matches"] = sum(
                1 for c in cands if c.get("matches_prior"))
        matched = [c for c in COLOUR_ORDER
                   if records[c]["found"] and records[c]["n_prior_matches"]]
        out["prior"] = expect.as_dict()
        out["prior_matched"] = matched
        out["n_prior_matched"] = len(matched)
        if present and not matched:
            out["note"] = (
                "%s -- but NONE is the expected size: no candidate matched the "
                "%.2f mm prior at %.1f px/mm. On this rig that reading is a "
                "machined feature or a well wall, not a dot."
                % (note, expect.diameter_mm, expect.px_per_mm))

    return out


def _self_test() -> int:
    import tempfile
    fails = 0

    def ok(c, m):
        nonlocal fails
        print("[self-test] %s: %s" % ("PASS" if c else "FAIL", m))
        if not c:
            fails += 1

    # exact circle recovery from a full ring
    th = np.linspace(0, 2 * np.pi, 400, endpoint=False)
    pts = np.column_stack([1234.5 + 300 * np.cos(th), 987.6 + 300 * np.sin(th)])
    f = fit_circle(pts)
    ok(f and abs(f["cx_px"] - 1234.5) < 1e-6 and abs(f["cy_px"] - 987.6) < 1e-6
       and abs(f["radius_px"] - 300) < 1e-6, "circle fit exact on a full ring")

    # a 40 % arc still recovers the centre -- this is the partly-unlit case
    th2 = np.linspace(0.6 * np.pi, 1.4 * np.pi, 160)
    arc = np.column_stack([1234.5 + 300 * np.cos(th2), 987.6 + 300 * np.sin(th2)])
    f2 = fit_circle(arc)
    ok(f2 and abs(f2["cx_px"] - 1234.5) < 1e-3 and abs(f2["radius_px"] - 300) < 1e-3,
       "circle fit recovers the centre from a 40%% arc (cx err %.2e px)"
       % (abs(f2["cx_px"] - 1234.5) if f2 else float("nan")))

    ok(fit_circle(np.zeros((5, 2))) is None, "too few points -> None, not a fabricated centre")
    ok(fit_circle(np.column_stack([np.arange(50.0), np.arange(50.0)])) is None,
       "collinear points -> None")

    # synthetic frame: lit disc with a dark mark fully inside it
    img = np.zeros((600, 900, 3), np.uint8)
    cv2.circle(img, (450, 300), 260, (170, 170, 170), -1)
    cv2.circle(img, (500, 320), 70, (12, 12, 12), -1)
    with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as fh:
        p = Path(fh.name)
    cv2.imwrite(str(p), img)
    d = detect(p, mode="dark")
    ok(d["n_candidates"] == 1, "one dark candidate found (%d)" % d["n_candidates"])
    if d["candidates"]:
        c = d["candidates"][0]
        ok(abs(c["centroid_px"][0] - 500) < 3 and abs(c["centroid_px"][1] - 320) < 3,
           "centroid at the mark (%.1f, %.1f), expected (500, 320)" % tuple(c["centroid_px"]))
        ok(c["quality"] in ("ok", "poor_circle"), "fully lit mark is not flagged partly-unlit "
           "(quality=%s)" % c["quality"])
        off, why = best_offset_px(c)
        ok(why.startswith("centroid"), "steers on the centroid when the mark is fully lit")

    # red mode on a red-free frame must report nothing
    d2 = detect(p, mode="red")
    ok(d2["n_candidates"] == 0, "red mode finds nothing when there is no red mark")

    # red mode on a frame WITH a red mark
    cv2.circle(img, (300, 380), 55, (30, 30, 220), -1)
    cv2.imwrite(str(p), img)
    d3 = detect(p, mode="red")
    ok(d3["n_candidates"] == 1, "red mode finds the red mark (%d)" % d3["n_candidates"])
    if d3["candidates"]:
        c3 = d3["candidates"][0]
        ok(abs(c3["centroid_px"][0] - 300) < 4 and abs(c3["centroid_px"][1] - 380) < 4,
           "red centroid at (%.1f, %.1f), expected (300, 380)" % tuple(c3["centroid_px"]))

    # blue mode, on a frame carrying only a blue mark
    imgb = np.zeros((600, 900, 3), np.uint8)
    cv2.circle(imgb, (450, 300), 260, (170, 170, 170), -1)
    cv2.circle(imgb, (430, 290), 45, (220, 30, 30), -1)
    cv2.imwrite(str(p), imgb)
    ok(detect(p, mode="blue")["n_candidates"] == 1, "blue mode finds the blue mark")
    ok(detect(p, mode="red")["n_candidates"] == 0, "red mode does not fire on a blue mark")

    # The defect classify_dots exists for: mode="dark" DOES fire on a blue mark,
    # and reports it as the black one with no hint that anything is wrong.
    ok(detect(p, mode="dark")["n_candidates"] == 1,
       "mode='dark' fires on the blue mark -- the mis-attribution this guards against")
    cls = classify_dots(p)
    ok(cls["present"] == ["blue"], "classify_dots reports blue and only blue (%s)" % cls["present"])
    ok(cls["colours"]["black"]["found"] is False
       and cls["colours"]["black"]["suppressed_by"] == "blue",
       "the dark claim is attributed to blue rather than silently kept as black")
    ok(len(cls["conflicts"]) == 1 and cls["conflicts"][0]["resolved_to"] == "blue",
       "the overlap is reported as a conflict, not resolved out of sight")

    # blank lit plate -- the normal reading during a search
    blank = np.zeros((600, 900, 3), np.uint8)
    cv2.circle(blank, (450, 300), 260, (170, 170, 170), -1)
    cv2.imwrite(str(p), blank)
    cls0 = classify_dots(p)
    ok(cls0["present"] == [] and cls0["n_present"] == 0,
       "a blank lit frame classifies as nothing, without raising")
    p.unlink(missing_ok=True)

    print("\n[self-test] %s" % ("ALL PASS" if fails == 0 else "%d FAILURE(S)" % fails))
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(_self_test())
