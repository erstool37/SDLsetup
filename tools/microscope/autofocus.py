"""Autofocus policy: plan a Z scan, and pick the plane from what was measured.

The *measurement* lives in :mod:`tools.microscope.focus` -- it reports
sharpness and quality flags and nothing else. This module holds what that layer
deliberately refuses to do: decide which heights to sample, where the peak is,
and whether a scan is trustworthy.

That split is the lab's standing rule -- a sensing node reports, the
orchestration layer decides -- applied to focus.
"""
from __future__ import annotations

from collections.abc import Sequence
from typing import Any

import numpy as np

from .focus import FocusSample

# ---------------------------------------------------------------------------
# (1) Scan planning
# ---------------------------------------------------------------------------

def plan_scan(
    z_min: float,
    z_max: float,
    coarse_step: float,
    fine_step: float,
) -> dict[str, Any]:
    """
    Describe a two-pass autofocus Z scan over the HARD range [z_min, z_max].

    Pass 1 (coarse): z_min .. z_max inclusive at coarse_step (z_max appended if
                     the grid does not land on it, so the top of the range is
                     always sampled).
    Pass 2 (fine):   fine_step steps within +/- (coarse_step / 2) around the
                     coarse peak, clamped to [z_min, z_max]. The actual fine Z
                     values depend on the measured coarse peak -- see
                     fine_scan_values().
    Then parabolic_refine() on all samples gives the final sub-step estimate.

    Raises:
        ValueError: on inverted range or non-positive steps.
    """
    if z_max <= z_min:
        raise ValueError(f"z_max ({z_max}) must be > z_min ({z_min})")
    if coarse_step <= 0 or fine_step <= 0:
        raise ValueError("coarse_step and fine_step must be > 0")

    coarse_z: list[float] = [float(z) for z in
                             np.arange(z_min, z_max + 1e-9, coarse_step)]
    if coarse_z[-1] < z_max - 1e-6:
        coarse_z.append(float(z_max))

    fine_half = coarse_step / 2.0
    n_fine_max = int(np.floor(2.0 * fine_half / fine_step)) + 1

    return {
        "z_min": z_min,
        "z_max": z_max,
        "coarse_step": coarse_step,
        "fine_step": fine_step,
        "coarse_z": [round(z, 6) for z in coarse_z],
        "n_coarse": len(coarse_z),
        "fine_half_window": fine_half,
        "n_fine_max": n_fine_max,
        "description": (
            f"coarse: {len(coarse_z)} pts @ {coarse_step}mm over "
            f"[{z_min:.2f}, {z_max:.2f}]; fine: {fine_step}mm steps within "
            f"+/-{fine_half}mm of coarse peak (clamped); then parabolic refine"
        ),
    }


def fine_scan_values(
    peak_z: float,
    z_min: float,
    z_max: float,
    fine_step: float,
    half_window: float,
) -> list[float]:
    """
    Fine-pass Z values: peak_z +/- half_window at fine_step spacing, clamped to
    the hard range [z_min, z_max]. Values are deduplicated and sorted ascending.
    """
    lo = max(z_min, peak_z - half_window)
    hi = min(z_max, peak_z + half_window)
    values = np.arange(peak_z - half_window, peak_z + half_window + 1e-9,
                       fine_step)
    clamped = sorted({float(round(min(max(float(v), lo), hi), 6))
                      for v in values})
    return clamped


# ---------------------------------------------------------------------------
# (2) Sub-step peak refinement
# ---------------------------------------------------------------------------

def parabolic_refine(
    z_values: Sequence[float],
    scores: Sequence[float],
) -> float:
    """
    Sub-step focus-peak estimate: fit a parabola through the 3 points around the
    argmax score and return the vertex Z, clamped to [min(z), max(z)].

    Falls back to the argmax Z when the peak is at a boundary, fewer than 3
    samples exist, or the 3 points are (near-)collinear.

    Raises:
        ValueError: empty input or mismatched lengths.
    """
    if len(z_values) == 0 or len(z_values) != len(scores):
        raise ValueError(
            f"Need equal-length non-empty z_values/scores "
            f"(got {len(z_values)}/{len(scores)})"
        )

    pairs = sorted(zip(z_values, scores, strict=False), key=lambda p: p[0])
    zs = [p[0] for p in pairs]
    ss = [p[1] for p in pairs]
    z_lo, z_hi = zs[0], zs[-1]

    idx = int(np.argmax(ss))
    if len(zs) < 3 or idx == 0 or idx == len(zs) - 1:
        return float(zs[idx])

    z0, z1, z2 = zs[idx - 1], zs[idx], zs[idx + 1]
    s0, s1, s2 = ss[idx - 1], ss[idx], ss[idx + 1]

    # General 3-point parabola vertex (handles non-uniform spacing).
    denom = (z1 - z0) * (s1 - s2) - (z1 - z2) * (s1 - s0)
    if abs(denom) < 1e-12:
        return float(z1)
    z_vertex = z1 - 0.5 * (
        (z1 - z0) ** 2 * (s1 - s2) - (z1 - z2) ** 2 * (s1 - s0)
    ) / denom
    if not np.isfinite(z_vertex):
        return float(z1)
    return float(min(max(z_vertex, z_lo), z_hi))



# ---------------------------------------------------------------------------
# (3) Peak selection
# ---------------------------------------------------------------------------

def best_sample(samples: Sequence[FocusSample],
                key: str = "score") -> FocusSample | None:
    """The sharpest usable sample, or None if nothing was measurable.

    Unusable samples are skipped, never treated as zero: a frame that was too
    dark to score is not evidence that its height is out of focus.

    ``key`` selects ``score`` (raw) or ``score_normalised`` (divided by mean
    squared, comparable across changing illumination). Which one to rank on is a
    judgement about the run, which is why it is decided here and not in the
    measurement layer.
    """
    if key not in ("score", "score_normalised"):
        raise ValueError(f"key must be 'score' or 'score_normalised', got {key!r}")
    usable = [s for s in samples if s.usable and getattr(s, key) is not None]
    if not usable:
        return None
    return max(usable, key=lambda s: getattr(s, key))


#: Coarse buckets for `skipped` reasons in `scan_report`, so a caller can see
#: "3 dark, 1 saturated" rather than 4 mutually-unique sentences. Matched
#: against the FRONT of `FocusSample.reason`, which is where
#: `tools.microscope.focus.score_frame` puts the fixed phrase before any
#: measured numbers -- see that module's `too dark` / `saturated` / `no lit
#: sample` strings.
_REASON_PREFIXES = (
    ("no lit sample", "no_lit_sample"),
    ("too dark", "dark"),
    ("saturat", "saturated"),
)


def _reason_category(reason: str) -> str:
    low = reason.lower()
    for prefix, label in _REASON_PREFIXES:
        if prefix in low:
            return label
    return "other" if reason else "unknown"


def scan_report(samples: Sequence[FocusSample], heights_mm: Sequence[float],
                key: str = "score") -> dict[str, Any]:
    """Summarise a scan: the pick, its margin, and what was skipped.

    ``margin`` is the ratio of the best score to the RUNNER-UP -- sensitive to
    one strong second-place sample. ``peak_over_median`` is the ratio of the
    best score to the MEDIAN of every scored sample, which is what tells a
    real focus peak (peak/median ~50 on this rig) apart from a dot-free sweep
    of noise (peak/median ~1.16, every sample about as "sharp" as every
    other) -- see :func:`judge_scan`, which is the caller-side test built on
    this number.

    Facts added for that purpose, all reported unconditionally when the scan
    is ``ok`` (never a verdict -- :func:`judge_scan` applies the thresholds):

    ``baseline_median``   median of every SCORED sample's ``key`` value.
    ``peak_over_median``  ``best_score / baseline_median`` (``None`` if the
                          median is zero -- a zero baseline is degenerate,
                          not a legitimate infinite prominence).
    ``peak_at_boundary``  the pick sits at the first or last value of
                          ``heights_mm`` -- the scan may not have covered the
                          true peak.
    ``dynamic_range``     ``max(score) / min(score)`` over scored samples.
    ``n_unusable`` / ``unusable_frac``   how much of the scan produced no
                          usable measurement at all.
    ``unusable_by_reason``  counts per :func:`_reason_category`.
    """
    if len(samples) != len(heights_mm):
        raise ValueError(f"{len(samples)} samples but {len(heights_mm)} heights")
    if key not in ("score", "score_normalised"):
        raise ValueError(f"key must be 'score' or 'score_normalised', got {key!r}")
    scored = [(z, s) for z, s in zip(heights_mm, samples, strict=False)
              if s.usable and getattr(s, key) is not None]
    skipped = [{"z_mm": z, "reason": s.reason}
               for z, s in zip(heights_mm, samples, strict=False) if not s.usable]
    if not scored:
        return {"ok": False, "reason": "no usable sample in the scan",
                "skipped": skipped, "n_scored": 0}
    scored.sort(key=lambda pair: getattr(pair[1], key), reverse=True)
    best_z, best = scored[0]
    best_score = getattr(best, key)
    runner_up = getattr(scored[1][1], key) if len(scored) > 1 else None
    margin = (best_score / runner_up) if runner_up else None

    all_scores = [getattr(s, key) for _z, s in scored]
    baseline_median = float(np.median(all_scores))
    peak_over_median = (best_score / baseline_median) if baseline_median > 0 else None
    lo_h, hi_h = min(heights_mm), max(heights_mm)
    peak_at_boundary = best_z == lo_h or best_z == hi_h
    lowest_score = min(all_scores)
    dynamic_range = (best_score / lowest_score) if lowest_score > 0 else None

    unusable_by_reason: dict[str, int] = {}
    for entry in skipped:
        cat = _reason_category(entry["reason"])
        unusable_by_reason[cat] = unusable_by_reason.get(cat, 0) + 1

    return {
        "ok": True,
        "best_z_mm": best_z,
        "best_score": best_score,
        "ranked_on": key,
        "runner_up_score": runner_up,
        "margin": round(margin, 3) if margin is not None else None,
        "method": best.method,
        "masked": best.masked,
        "n_scored": len(scored),
        "skipped": skipped,
        "baseline_median": round(baseline_median, 6),
        "peak_over_median": round(peak_over_median, 3) if peak_over_median is not None else None,
        "peak_at_boundary": peak_at_boundary,
        "dynamic_range": round(dynamic_range, 3) if dynamic_range is not None else None,
        "n_unusable": len(skipped),
        "unusable_frac": round(len(skipped) / len(samples), 4),
        "unusable_by_reason": unusable_by_reason,
    }


def judge_scan(report: dict[str, Any], *, min_prominence: float,
               min_absolute_score: float, max_unusable_frac: float,
               require_interior: bool) -> dict[str, Any]:
    """Apply CALLER-SUPPLIED thresholds to a :func:`scan_report`. Decides nothing else.

    There are no module-level defaults here for a caller to silently inherit --
    every threshold is a required keyword, so the numbers a procedure judged a
    scan by are always visible at its own call site, not buried in this file.

    Prominence alone is not sufficient: a single dust speck can spike
    ``peak_over_median`` on an otherwise flat, dark scan, so
    ``min_absolute_score`` is a required, SEPARATE floor -- both must pass.

    Returns ``{"usable": bool, "reasons": [...]}`` -- one entry per failed
    test, empty when every test the caller asked for passed. Never raises for
    an ordinary "not usable" scan; a malformed ``report`` (missing the fields
    :func:`scan_report` always produces when ``ok`` is true) is a caller bug
    and surfaces as a ``KeyError``, not a silently-empty verdict.
    """
    if not report.get("ok"):
        return {"usable": False,
                "reasons": [f"scan not ok: {report.get('reason', 'unknown')}"]}

    reasons: list[str] = []
    prominence = report["peak_over_median"]
    if prominence is None or prominence < min_prominence:
        reasons.append(
            f"prominence (peak/median) {prominence!r} below floor {min_prominence:g}")
    best_score = report["best_score"]
    if best_score is None or best_score < min_absolute_score:
        reasons.append(
            f"best score {best_score!r} below the absolute floor {min_absolute_score:g}")
    unusable_frac = report["unusable_frac"]
    if unusable_frac > max_unusable_frac:
        reasons.append(
            f"unusable fraction {unusable_frac:.3f} exceeds the cap {max_unusable_frac:g} "
            f"({report['n_unusable']} of {report['n_scored'] + report['n_unusable']} samples)")
    if require_interior and report["peak_at_boundary"]:
        reasons.append(
            f"peak z={report['best_z_mm']!r} sits at the edge of the scanned range; "
            f"the true peak may lie outside it")

    return {"usable": len(reasons) == 0, "reasons": reasons}


__all__ = [
    "best_sample",
    "fine_scan_values",
    "judge_scan",
    "parabolic_refine",
    "plan_scan",
    "scan_report",
]
