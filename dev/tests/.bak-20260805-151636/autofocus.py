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


def scan_report(samples: Sequence[FocusSample], heights_mm: Sequence[float],
                key: str = "score") -> dict[str, Any]:
    """Summarise a scan: the pick, its margin, and what was skipped.

    ``margin`` is the ratio of the best score to the runner-up. A margin near
    1.0 means the scan did not actually distinguish the planes, which is a
    result the caller must be able to see rather than a number to act on.
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
    }


__all__ = [
    "best_sample",
    "fine_scan_values",
    "parabolic_refine",
    "plan_scan",
    "scan_report",
]
