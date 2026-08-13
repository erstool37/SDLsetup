#!/usr/bin/env python3
"""find_target.py -- coupled XY/Z search for the plate's alignment mark.

    python3 scripts/microscope/find_target.py                 # dry-run, prints the plan
    python3 scripts/microscope/find_target.py --execute        # search, then report

WHY THIS EXISTS
----------------
Every other calibration/alignment routine on this rig (sharpness_test.py,
calibration.py, dot_scan.py) assumes the plate is where it was taught: they
sweep Z at a fixed XY, or sweep XY at a fixed Z. When the plate is not where it
was taught -- and it does not seat identically every time -- the Z sweep finds
no focal plane and the run dies with nothing to show for it. This script
searches BOTH axes, coupled, because they cannot be searched independently
without one search corrupting the other (see below).

THE KEY DESIGN CONSTRAINT -- READ BEFORE "IMPROVING" THIS
-----------------------------------------------------------
Finding a candidate XY must NOT use a focus metric. The depth of field here is
under 0.5 mm (measured); a Z probe coarse enough to afford at every XY in a
search grid aliases straight past a real peak, and the DOF-resolved 0.3 mm step
that would not alias is far too slow to run at every candidate XY.

A 1 mm printed dot is a large, high-contrast feature against a ~2.84 x 2.38 mm
field, and it still casts a visible dark blob when defocused by millimetres --
unlike sharpness, which collapses within a fraction of a millimetre (measured:
variance-of-Laplacian fell from 364 to 7.6 within 0.47 mm of the focus plane).
So this script has two decoupled phases:

  1. STRUCTURE PASS -- rank every candidate XY by a DEFOCUS-TOLERANT structure
     signal (masked std, from tools.microscope.focus.score_frame, which is
     already computed for every frame regardless of whether the frame judges
     "usable"), plus saturated/dark quality flags. One frame per XY, at the
     working height. No focus judgement happens here.
  2. FOCUS PASS -- only at the top-N XY by structure -- spends a real,
     DOF-resolved Z sweep (0.3 mm default, never coarser) and judges it with
     tools.microscope.autofocus.judge_scan.

Peak quality (phase 2) and target IDENTITY (phase 3, below) are separate
acceptance tests. A sharp fixture edge or a well wall can produce an excellent
focus peak and is not a dot -- see tools.microscope.marker.classify_dots's own
`prior_matched` vs `present` split, which this script re-uses rather than
re-deriving.

  3. IDENTITY PASS -- at an accepted focus plane, measure px/mm there (height
     changes magnification) and run tools.microscope.marker.classify_dots with
     a tools.microscope.marker.MarkPrior built from that measurement. Only a
     `prior_matched` colour counts as a confirmed target.

SAFETY MODEL
------------
G1  XY and Z NEVER change in the same command. Two envelope SHAPES enforce
    this structurally, not by convention:
      * the XY-travel envelope pins Z to a single value (z_min_rise_mm ==
        z_max_rise_mm == the working rise) and allows XY within --xy-radius;
      * a Z-probe envelope pins xy_max_mm=0.0 at one candidate XY and allows Z
        within the operator's rise budget.
    See build_xy_envelope() / build_z_envelope().
G2  Every Z envelope's Z window is measured from the IMMUTABLE taught A1 pose
    (`a1`, read once at the top of run() and never reassigned), never from a
    previous probe's readback and never from a raised current pose -- so
    successive probes cannot ratchet the ceiling toward the objective.
G3  Every move is followed by a readback comparison, via the same mechanism
    every other procedure on this rig uses: tools.arm.Arm.move() re-reads the
    controller after every LIVE move and checks it against the active
    envelope. The arm silently refuses out-of-reach moves (measured: y<~320 at
    x=26 stops short, state=4 code=-9, no latched fault) -- a refusal leaves
    the arm outside the commanded envelope, so Arm.move() itself raises
    SafetyError, and attempt_move() (below) turns that into an "unreachable"
    record instead of aborting the whole search.
G4  Dry-run by default; --execute is required to move anything.
G5  Never below the taught A1, never above A1 + --z-max-rise (hard-capped at
    tools.arm.safety.Z_MAX_RISE_MM) -- enforced by the envelopes themselves,
    not by this script's arithmetic.
G6  Any controller fault stops the whole search and is never auto-cleared
    (--clear-errors is the explicit operator override, same as every sibling
    script); a hardware ArmError is deliberately NOT caught here and aborts
    the run.
G7  Retreat to the WORKING HEIGHT on every non-success exit path: failed
    sweep, dark, saturated, capture error, metric exception,
    KeyboardInterrupt, SIGTERM/SIGHUP. See retreat_to_working_height() for why
    this is a purpose-built function and not tools.arm.Arm.retreat().
G8  The complete, finite XY probe list is enumerated ONCE, up front, printed,
    and checked against a hard cap (--max-probes) before anything moves. No
    ring is generated mid-run and no probe retries on refusal.
G9  Never silently settle on the least-bad spot. A focus peak is only
    accepted via tools.microscope.autofocus.judge_scan against thresholds
    this script owns (no baked-in defaults, per that function's contract);
    a target is only accepted once tools.microscope.marker.classify_dots
    reports a `prior_matched` colour. If nothing is accepted, the exit status
    and printed banner say which of "found no structure at all" (barren),
    "found structure but no acceptable focal plane" (with everywhere-
    saturated called out separately, because the remedy differs -- exposure
    vs position), or "found a focal plane but it is not a dot" happened, with
    the full coverage map -- never a fallback move to an unconfirmed spot.

WHY dataset/captures/find_target/, NOT tools.runs.Run
-------------------------------------------------------
Deliberate. tools.runs.Run exists and is tested, but as of this script no
phase script in this repo has migrated to it yet (see .claude/rules/run-data.md,
"Status, 2026-08-03"); every sibling this script was built from
(sharpness_test.py, calibration.py, dot_scan.py) writes its own timestamped
directory under dataset/captures/<phase>/<stamp>/ with its own manifest.json.
This script follows that convention rather than migrating alone.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


import argparse  # noqa: E402
import dataclasses  # noqa: E402
import datetime as dt  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402
import signal  # noqa: E402
import sys  # noqa: E402
import time  # noqa: E402
from pathlib import Path  # noqa: E402
from typing import Any  # noqa: E402

from tools import config as _config  # noqa: E402
from tools.arm import Arm, Envelope  # noqa: E402
from tools.arm.driver import ArmError  # noqa: E402
from tools.arm.safety import (  # noqa: E402
    READBACK_SLACK_DEG,
    READBACK_SLACK_MM,
    SPEED_MAX_MM_S,
    Z_MAX_RISE_MM,
    SafetyError,
)
from tools.calib import pixel_scale as ps  # noqa: E402
from tools.calib import search as _search  # noqa: E402
from tools.microscope import Microscope  # noqa: E402
from tools.microscope import autofocus as af  # noqa: E402
from tools.microscope import focus as _focus  # noqa: E402
from tools.microscope import marker as _marker  # noqa: E402

# ---------------------------------------------------------------------------
# Constants -- operator/rig limits. Do not change without re-measuring.
# ---------------------------------------------------------------------------

ANCHOR = "microscope"
ARM_HOST_DEFAULT = "192.168.1.201"

#: The height every other px/mm and focus reference number on this rig was
#: measured at (tools.calib.search.FOV_MM_AT_A1_PLUS_3MM,
#: tools/microscope/focus.py's masked-scoring table). Also this script's
#: default structure-pass height.
WORKING_RISE_MM = 3.0

#: Depth of field is under 0.5 mm (measured). This is a hard ceiling on
#: --coarse-step, not a default a caller may loosen -- a coarser step aliases
#: straight past the real peak, which is the whole reason phase 1 does not use
#: a focus metric at all.
MAX_COARSE_STEP_MM = 0.3

#: Extra structure-pass heights tried, as fractions of the remaining rise
#: budget above the working height, ONLY if the working-height pass finds no
#: structure anywhere. Two extra planes, per the acceptance spec.
STRUCTURE_EXTRA_Z_FRACTIONS = (0.5, 1.0)

#: Hard cap on the enumerated XY probe list. Mirrors focus_sweep.py's
#: MAX_STEPS -- an unbounded plan is a safety hole (it hides how many moves a
#: CLI combination actually implies) as much as it is a runtime one.
MAX_PROBES_DEFAULT = 200

#: Calibration step for the identity-pass px/mm measurement -- same order of
#: magnitude and same reasoning as dot_scan.py's CAL_STEP_MM: small enough to
#: stay inside a sub-0.5mm depth of field, large enough for phase correlation
#: to resolve.
CAL_STEP_MM = 0.30

#: Pixel scale used to size the LIVE trigger band before anything is measured.
#: Mean of the 2026-08-04 per-axis measurement (883.1 / 892.3 px/mm at A1+3).
#: It only has to be right to within the trigger's +/-30% band; the identity
#: pass re-measures the scale at the focal plane and rebuilds the prior there.
TRIGGER_PX_PER_MM = 887.7

#: When the trigger matches on the FITTED circle rather than the blob's own
#: equivalent radius, at least this fraction of the disc must actually be
#: visible. Without it a small arc fragment extrapolates to any radius you like:
#: measured 2026-08-06, a 102 px sliver of a plate edge fitted a circle inside
#: the 311-577 px band and stopped the hunt on a frame with no mark in it.
FIT_MIN_VISIBLE_FRAC = 0.5
MIN_RESPONSE = 0.05
MIN_SHIFT_PX = 8.0

FRAME_TIMEOUT_S = 30.0
SETTLE_S_DEFAULT = 0.8
SCAN_SPEED_MM_S = 5.0

DEFAULT_OUT_DIR = _config.PROJECT_ROOT / "dataset" / "captures" / "find_target"


class FindTargetError(RuntimeError):
    """Anything that must stop the search."""


class _Terminated(FindTargetError):
    """SIGTERM / SIGHUP arrived -- treated as an abort so retreat still runs."""


# ---------------------------------------------------------------------------
# (1) Envelopes. Two SHAPES, never mixed: XY travel pins Z, a Z probe pins XY.
# ---------------------------------------------------------------------------

def build_xy_envelope(a1: list[float], *, working_rise_mm: float,
                      xy_radius_mm: float,
                      orient_tol_deg: float = READBACK_SLACK_DEG) -> Envelope:
    """XY travel at a single, pinned Z. The anchor is A1 itself, unmodified.

    z_min_rise_mm == z_max_rise_mm == working_rise_mm collapses the permitted
    Z window to exactly one value, so no move validated by this envelope can
    change height -- structurally, not by convention (same trick
    dot_scan.py's scan envelope uses, anchored on A1 instead of the arm's
    current pose so repeated runs cannot drift).
    """
    return Envelope.anchored(
        a1, name="A1", z_min_rise_mm=working_rise_mm, z_max_rise_mm=working_rise_mm,
        xy_max_mm=xy_radius_mm, orient_tol_deg=orient_tol_deg)


def build_z_envelope(a1: list[float], *, dx_mm: float, dy_mm: float,
                     z_max_rise_mm: float,
                     orient_tol_deg: float = READBACK_SLACK_DEG) -> Envelope:
    """A Z-only probe at one XY offset from the IMMUTABLE taught A1.

    Only the anchor's X/Y are translated, to the XY this probe is AT; its Z
    stays A1's own recorded Z, literally -- never a previous probe's readback,
    never a raised current pose. xy_max_mm=0.0 then means no move this
    envelope validates may change XY at all, which is what makes "XY and Z
    never change in the same command" a structural property rather than an
    invariant this script has to remember to preserve.
    """
    anchor_at_xy = (a1[0] + dx_mm, a1[1] + dy_mm, a1[2], a1[3], a1[4], a1[5])
    return Envelope.anchored(
        anchor_at_xy, name="A1@(%+.3f,%+.3f)" % (dx_mm, dy_mm),
        z_min_rise_mm=0.0, z_max_rise_mm=z_max_rise_mm,
        xy_max_mm=0.0, orient_tol_deg=orient_tol_deg)


# ---------------------------------------------------------------------------
# (2) The complete, finite probe list -- built once, capped, never regenerated.
# ---------------------------------------------------------------------------

def build_probe_list(*, fov_mm: Any, xy_radius_mm: float, overlap: float,
                     max_probes: int, pitch_mm: float = _search.DEFAULT_PITCH_MM):
    """Ring-spiral XY probes covering +/-xy_radius_mm, via tools.calib.search.

    A thin, capped wrapper -- tools.calib.search.scan_plan() already builds
    the whole plan as one list (no mid-scan regeneration is possible even by
    accident), and the coverage guarantee documented there is not re-derived
    here.

    Raises:
        FindTargetError: the plan exceeds max_probes. Narrowing --xy-radius or
            widening --overlap are the ways to bring it back down; raising
            --max-probes is a deliberate override, not a default.
    """
    plan = _search.scan_plan(fov_mm=fov_mm, pitch_mm=pitch_mm,
                             max_offset_mm=xy_radius_mm, overlap=overlap)

    # scan_plan steps outward in FOV-sized increments until it COVERS
    # max_offset_mm, so its outermost ring overshoots the radius -- measured
    # 2026-08-05: a +/-4.00 mm request produced probes at 4.69 and 5.23 mm, and
    # the XY envelope (built at exactly xy_radius_mm) refused 28 of 63. The
    # guard caught every one, which is the system working; emitting them at all
    # is the planner being wrong. Clip per axis, because the envelope is a box.
    kept = [pt for pt in plan
            if abs(pt.dx_mm) <= xy_radius_mm + 1e-9
            and abs(pt.dy_mm) <= xy_radius_mm + 1e-9]
    dropped = len(plan) - len(kept)
    if dropped:
        print("  probe plan: dropped %d of %d point(s) outside the +/-%.2f mm "
              "envelope (scan_plan's outermost ring overshoots by design)"
              % (dropped, len(plan), xy_radius_mm))
    plan = kept

    if len(plan) > max_probes:
        raise FindTargetError(
            "the scan plan has %d points, over the cap of %d (fov=%r mm, xy_radius=%.2f mm, "
            "overlap=%.2f). Narrow --xy-radius, widen --overlap, or raise --max-probes "
            "deliberately -- this is a cost cap, not a coverage bug."
            % (len(plan), max_probes, fov_mm, xy_radius_mm, overlap))
    return plan


# ---------------------------------------------------------------------------
# (3) Moves that report refusal instead of raising it into the search.
# ---------------------------------------------------------------------------

@dataclasses.dataclass
class MoveOutcome:
    ok: bool
    result: Any = None
    error: str | None = None


def attempt_move(arm: Arm, x: float, y: float, z: float, *, speed: float,
                 label: str, takeup: bool = False) -> MoveOutcome:
    """Command one pose; turn an out-of-envelope refusal into data.

    tools.arm.Arm.move() already re-reads the controller after every LIVE move
    and checks the result against the active envelope (G3 in
    .claude/rules/motion-safety.md). The arm can silently refuse a move
    (measured on this rig: y<~320 at x=26 stops short, state=4 code=-9, no
    latched controller fault) -- when that happens the achieved pose sits
    outside the envelope and Arm.move() raises SafetyError from inside the
    readback check. That is the ONLY exception this function catches: a
    hardware ArmError (an actual controller fault) is deliberately left to
    propagate and abort the whole search, per G6 in the module docstring.
    """
    try:
        result = arm.move(x, y, z, speed=speed, label=label, takeup=takeup)
    except SafetyError as exc:
        return MoveOutcome(False, error=str(exc))
    return MoveOutcome(True, result=result)


def goto_xy(arm: Arm, a1: list[float], dx_mm: float, dy_mm: float, working_z: float,
           xy_envelope: Envelope, *, speed: float, label: str) -> MoveOutcome:
    """Reposition to one candidate XY, Z pinned at the working height. XY-only.

    Every caller that is about to start a Z sweep or an identity pass at some
    (dx, dy) MUST call this first, even if the arm is believed to already be
    there: the arm's actual XY after a previous candidate's retreat is
    wherever THAT candidate was, and building a Z-only envelope pinned to a
    DIFFERENT XY while the arm sits somewhere else would make the very first
    move of that envelope a combined XY+Z traverse -- exactly what G1 forbids.
    Explicitly re-establishing XY first, under the XY-travel envelope (Z
    pinned), keeps every individual command single-axis.
    """
    return attempt_move(arm.with_envelope(xy_envelope), a1[0] + dx_mm, a1[1] + dy_mm,
                        working_z, speed=speed, label=label, takeup=False)


def goto_z(arm: Arm, a1: list[float], dx_mm: float, dy_mm: float, target_z: float,
          z_max_rise_mm: float, *, speed: float, label: str,
          takeup: bool = False) -> MoveOutcome:
    """Move Z only, at a candidate XY the arm is already AT. Z-only.

    Callers must have called :func:`goto_xy` for this (dx, dy) first in this
    same session -- this function does not itself verify that, it only builds
    an envelope whose xy_max_mm=0.0 makes any XY drift raise.
    """
    z_env = build_z_envelope(a1, dx_mm=dx_mm, dy_mm=dy_mm, z_max_rise_mm=z_max_rise_mm)
    return attempt_move(arm.with_envelope(z_env), a1[0] + dx_mm, a1[1] + dy_mm, target_z,
                        speed=speed, label=label, takeup=takeup)


def goto_plane(arm: Arm, a1: list[float], *, from_xy_envelope: Envelope, from_z: float,
               to_z: float, z_max_rise_mm: float, speed: float, label: str) -> MoveOutcome:
    """Move from wherever the last pass left the arm to (dx=0, dy=0) at a new Z plane.

    Every structure-pass Z-plane change in this script goes through here, so
    a plane change is always two single-axis moves -- XY home under the plane
    the arm is CURRENTLY pinned to, then Z-only to the new plane -- never a
    diagonal implied by wherever the previous pass's last probe happened to
    leave the arm. `from_z` must be the Z the arm is structurally guaranteed
    to be at (every envelope this script builds for a structure pass pins Z
    to a single value for its whole duration, so this is always true on
    return from run_structure_pass or retreat_to_working_height).
    """
    home = goto_xy(arm, a1, 0.0, 0.0, from_z, from_xy_envelope, speed=speed,
                   label="%s home XY" % label)
    if not home.ok:
        return home
    return goto_z(arm, a1, 0.0, 0.0, to_z, z_max_rise_mm, speed=speed,
                  label="%s to Z=%.4f" % (label, to_z), takeup=True)


# ---------------------------------------------------------------------------
# (4) Structure pass: rank XY by a defocus-tolerant signal. No focus metric.
# ---------------------------------------------------------------------------

def classify_structure_sample(sample: _focus.FocusSample) -> dict[str, Any]:
    """Structure-ranking facts from one FocusSample. Not a focus judgement.

    tools.microscope.focus.score_frame computes mean/std over the masked
    region whenever it finds a lit mask at all -- even for a frame it then
    reports as "too dark" or "saturated" -- so those frames are still ranked
    here on `std`, not silently dropped. Only a frame with NO lit mask found
    anywhere (`sample.mean is None`) reports std=0.0 and no structure.
    """
    has_mask = sample.mean is not None
    dark = has_mask and sample.mean < _focus.BRIGHTNESS_MIN_MEAN
    saturated = has_mask and sample.mean >= _focus.BRIGHTNESS_MAX_MEAN
    std = float(sample.std) if (has_mask and sample.std is not None) else 0.0
    return {"std": std, "mean": sample.mean, "mask_frac": sample.mask_frac,
           "has_mask": has_mask, "dark": dark, "saturated": saturated,
           "reason": sample.reason}


def rank_by_structure(records: list[dict[str, Any]], *, top_n: int) -> list[dict[str, Any]]:
    """The top-N captured records by structure std, descending.

    Records that were unreachable, failed to capture, or found no mask at all
    (std == 0.0 and not has_mask) never rank -- a candidate with literally no
    lit content is not "the least bad XY", it is not a candidate.
    """
    candidates = [r for r in records
                 if r.get("status") == "captured" and r.get("has_mask")]
    candidates.sort(key=lambda r: r["std"], reverse=True)
    return candidates[:top_n]


def dot_trigger(frame_path: Path, prior) -> dict | None:
    """Is this frame worth STOPPING the grid for? A trigger, NOT a verdict.

    WHY IT IS DELIBERATELY LOOSE. The structure pass runs at the working height,
    which is NOT a focal plane -- so every blob it sees is defocused. Three
    shape discriminators were measured against operator-labelled frames from the
    2026-08-06 run and ALL THREE failed to separate the real dot from a machined
    slot:

        field           real dot                 machined slot
        fill_ratio      0.58 / 0.41 / 0.39       0.77   <- inverted
        circle resid/r  0.082 / 0.022 / 0.129    0.260 / 0.024
        aspect          1.51 / 1.26 / 3.20       1.64 / 1.28
        surround lit    0.497 / 0.370 / 0.324    0.633  <- inverted

    A defocused disc simply does not keep the shape statistics of a sharp one,
    and the blur halo defeats the context measure too. So this does not try to
    be certain. It asks only "is something here the right SIZE and fully inside
    the frame", stops the grid, and lets the Z sweep be the arbiter -- a dot in
    focus is unmistakable, and that is a measurement rather than a guess.

    Cost asymmetry drives the looseness: a false trigger costs one Z sweep
    (~100 s) and the grid resumes; a missed dot costs the entire grid, which is
    what happened on 2026-08-06 when the dot was photographed at
    (-1.56, +1.31) and walked past.

    Border-touching blobs are excluded: the one candidate that passed the strict
    prior that day was a slot whose rounded end was clipped by the frame edge.
    """
    try:
        det = _marker.detect(frame_path, mode="dark", expect=prior)
    except Exception:  # noqa: BLE001 - a detector failure is not a sighting
        return None
    lo, hi = prior.band_px
    for c in det.get("candidates") or []:
        if c.get("touches_frame_border"):
            continue
        r_equiv = float(c.get("equiv_radius_px") or 0.0)
        r_fit = float((c.get("circle_fit") or {}).get("radius_px") or 0.0)

        if lo <= r_equiv <= hi:
            matched_on, matched_r = "equiv_radius", r_equiv
        elif lo <= r_fit <= hi and r_equiv >= FIT_MIN_VISIBLE_FRAC * r_fit:
            # A clipped disc keeps its fitted radius while losing area, so the
            # fit is allowed to carry the match -- but ONLY if a real fraction
            # of the disc is actually present. Measured false positive
            # 2026-08-06 at x=-8.599: a 102 px fragment of a plate edge fitted
            # a circle inside the band and was reported as a mark. Half a disc
            # has equiv radius 0.71r and a quarter 0.5r, so 0.5 admits a badly
            # clipped mark and excludes a sliver.
            matched_on, matched_r = "circle_fit_radius", r_fit
        else:
            continue

        return {"radius_px": matched_r,
                "matched_on": matched_on,
                "equiv_radius_px": r_equiv,
                "circle_radius_px": r_fit or None,
                "visible_frac": round(r_equiv / r_fit, 3) if r_fit else None,
                "fill_ratio": c.get("fill_ratio"),
                "centroid_px": c.get("centroid_px"),
                "quality": c.get("quality"),
                "band_px": [round(lo, 1), round(hi, 1)]}
    return None


def run_structure_pass(arm: Arm, scope: Microscope, a1: list[float], plan,
                       xy_envelope: Envelope, working_z: float, out_dir: Path,
                       *, tag: str, args: argparse.Namespace,
                       prior=None) -> list[dict[str, Any]]:
    """One structure pass over the probe list at a single pinned Z.

    Returns EARLY, with the hit as the last record, as soon as a frame holds a
    size-plausible candidate -- see dot_trigger. Callers detect this by testing
    the last record for "dot_trigger", and may resume from the next probe.
    """
    xy_arm = arm.with_envelope(xy_envelope)
    records: list[dict[str, Any]] = []
    for point in plan:
        label = "%s ring%d idx%d (%+.3f,%+.3f)" % (tag, point.ring, point.index,
                                                    point.dx_mm, point.dy_mm)
        base: dict[str, Any] = {"stage": tag, "index": point.index, "ring": point.ring,
                                "dx_mm": point.dx_mm, "dy_mm": point.dy_mm, "z_mm": working_z}
        outcome = attempt_move(xy_arm, a1[0] + point.dx_mm, a1[1] + point.dy_mm, working_z,
                               speed=args.speed, label=label, takeup=False)
        if not outcome.ok:
            records.append({**base, "status": "unreachable", "reason": outcome.error})
            continue
        if not arm.settings.live:
            records.append({**base, "status": "skipped", "reason": "dry-run: no frame taken"})
            continue
        moved_at = time.time()
        time.sleep(args.settle)
        dest = out_dir / ("%s_%04d_dx%+06.2f_dy%+06.2f.jpg"
                          % (tag, point.index, point.dx_mm, point.dy_mm))
        frame = scope.grab_frame(dest, after=moved_at, ordinal=args.frame_ordinal,
                                 timeout_s=FRAME_TIMEOUT_S)
        if not frame.ok:
            records.append({**base, "status": "capture_failed", "reason": frame.reason})
            continue
        sample = scope.measure_focus(frame.path, method=args.method,
                                     min_mean=args.min_mean, masked=True)
        facts = classify_structure_sample(sample)
        rec = {**base, "status": "captured", "frame": str(dest), **facts}

        # Look for the mark on EVERY frame, not once at the end. The frame is
        # already captured, so this costs no motion at all -- and running it
        # only at the end is exactly how the 2026-08-06 grid photographed the
        # dot and kept going.
        hit = dot_trigger(frame.path, prior) if prior is not None else None
        if hit:
            rec["dot_trigger"] = hit
            records.append(rec)
            print("  [HIT] candidate the right size at (%+.3f,%+.3f): r=%.0f px "
                  "(band %.0f-%.0f). STOPPING the grid to verify by focus."
                  % (point.dx_mm, point.dy_mm, hit["radius_px"],
                     hit["band_px"][0], hit["band_px"][1]), flush=True)
            return records
        records.append(rec)
    return records


# ---------------------------------------------------------------------------
# (5) Focus pass: a real, DOF-resolved Z sweep at the top-N XY only.
# ---------------------------------------------------------------------------

def run_focus_sweep_at(arm: Arm, scope: Microscope, a1: list[float], *, dx_mm: float,
                       dy_mm: float, z_max_rise_mm: float, xy_envelope: Envelope,
                       out_dir: Path, tag: str, args: argparse.Namespace) -> dict[str, Any]:
    """Full coarse Z sweep at one XY, judged by af.judge_scan.

    Always retreats to the working height before returning, success or not --
    the caller does not have to remember to do it for every candidate.
    """
    working_z = a1[2] + args.working_rise

    # Re-establish XY under the XY-travel envelope FIRST (see goto_xy's
    # docstring): the arm may currently be parked at a different candidate's
    # XY, and starting the Z sweep without this would make its first move a
    # combined XY+Z traverse.
    reposition = goto_xy(arm, a1, dx_mm, dy_mm, working_z, xy_envelope,
                        speed=args.speed, label="%s goto XY" % tag)
    if not reposition.ok:
        empty_report = {"ok": False, "reason": "candidate XY unreachable", "skipped": [],
                        "n_scored": 0}
        return {"dx_mm": dx_mm, "dy_mm": dy_mm, "report": empty_report,
               "judged": af.judge_scan(empty_report, min_prominence=args.min_prominence,
                                       min_absolute_score=args.min_absolute_score,
                                       max_unusable_frac=args.max_unusable_frac,
                                       require_interior=args.require_interior),
               "probes": [{"stage": tag, "dx_mm": dx_mm, "dy_mm": dy_mm,
                          "status": "unreachable", "reason": reposition.error}]}

    z_values = af.plan_scan(a1[2], a1[2] + z_max_rise_mm, args.coarse_step,
                            args.coarse_step)["coarse_z"]
    samples: list[_focus.FocusSample] = []
    probe_records: list[dict[str, Any]] = []
    try:
        for i, z in enumerate(z_values, 1):
            label = "%s z %d/%d (%+.3f,%+.3f)" % (tag, i, len(z_values), dx_mm, dy_mm)
            base = {"stage": tag, "dx_mm": dx_mm, "dy_mm": dy_mm, "z_mm": z}
            outcome = goto_z(arm, a1, dx_mm, dy_mm, z, z_max_rise_mm,
                             speed=args.speed, label=label, takeup=True)
            if not outcome.ok:
                probe_records.append({**base, "status": "unreachable", "reason": outcome.error})
                # An unusable sample must not read as "at focus height zero" --
                # best_sample()/scan_report() already skip unusable samples
                # rather than scoring them, so a placeholder unusable sample
                # here is the correct, not a lossy, way to keep z_values and
                # samples the same length.
                samples.append(_focus.FocusSample(path=None, method=args.method, masked=True,
                                                  usable=False, reason=outcome.error or ""))
                continue
            if not arm.settings.live:
                probe_records.append({**base, "status": "skipped", "reason": "dry-run"})
                samples.append(_focus.FocusSample(path=None, method=args.method, masked=True,
                                                  usable=False, reason="dry-run"))
                continue
            time.sleep(args.settle)
            dest = out_dir / ("%s_%03d_z%+07.3f.jpg" % (tag, i, z - a1[2]))
            frame = scope.grab_frame(dest, ordinal=args.frame_ordinal, timeout_s=FRAME_TIMEOUT_S)
            if not frame.ok:
                probe_records.append({**base, "status": "capture_failed", "reason": frame.reason})
                samples.append(_focus.FocusSample(path=None, method=args.method, masked=True,
                                                  usable=False, reason=frame.reason))
                continue
            sample = scope.measure_focus(frame.path, method=args.method,
                                         min_mean=args.min_mean, masked=True)
            probe_records.append({**base, "status": "captured", "frame": str(dest),
                                  **sample.as_dict()})
            samples.append(sample)
    finally:
        retreat_to_working_height(arm, a1, working_z, xy_envelope,
                                  speed=args.speed, label="%s retreat" % tag)

    report = af.scan_report(samples, z_values)
    judged = af.judge_scan(report, min_prominence=args.min_prominence,
                           min_absolute_score=args.min_absolute_score,
                           max_unusable_frac=args.max_unusable_frac,
                           require_interior=args.require_interior)
    return {"dx_mm": dx_mm, "dy_mm": dy_mm, "report": report, "judged": judged,
           "probes": probe_records}


# ---------------------------------------------------------------------------
# (6) Identity pass: px/mm at the accepted plane, then classify_dots.
# ---------------------------------------------------------------------------

class MeasurementError(RuntimeError):
    """The pixel scale could not be measured at this plane."""


def measure_scale_here(arm: Arm, scope: Microscope, a1: list[float], x: float, y: float,
                       z: float, out_dir: Path, tag: str, *, cal_step: float, speed: float,
                       settle: float, orient_tol_deg: float) -> dict[str, Any]:
    """px/mm at (x, y, z), by two small moves and phase correlation.

    Reuses tools.calib.pixel_scale (image_shift_px / jacobian_from_moves /
    decompose) -- the same measurement calibration.py's calibrate_here() and
    dot_scan.py's measure_jacobian() are built from -- rather than a third
    inline implementation of phase-correlation calibration. Orientation is
    taken from the taught A1 pose (`a1[3:6]`), never re-derived from whatever
    envelope `arm` happens to be carrying at the call site.
    """
    here_env = Envelope.anchored(
        (x, y, z, a1[3], a1[4], a1[5]),
        name="%s scale-cal" % tag, z_min_rise_mm=0.0, z_max_rise_mm=0.0,
        xy_max_mm=cal_step + 0.5, orient_tol_deg=orient_tol_deg)
    cal_arm = arm.with_envelope(here_env)

    ref_dest = out_dir / ("%s_scale_ref.jpg" % tag)
    ref_frame = scope.grab_frame(ref_dest, ordinal=2, timeout_s=FRAME_TIMEOUT_S)
    if not ref_frame.ok:
        raise MeasurementError("reference frame: %s" % ref_frame.reason)

    shifts: dict[str, tuple[float, float]] = {}
    try:
        for axis, (dx, dy) in (("x", (cal_step, 0.0)), ("y", (0.0, cal_step))):
            outcome = attempt_move(cal_arm, x + dx, y + dy, z, speed=speed,
                                   label="%s scale-cal +%s" % (tag, axis.upper()))
            if not outcome.ok:
                raise MeasurementError("scale-cal +%s move refused: %s" % (axis, outcome.error))
            time.sleep(settle)
            dest = out_dir / ("%s_scale_%s.jpg" % (tag, axis))
            frame = scope.grab_frame(dest, ordinal=2, timeout_s=FRAME_TIMEOUT_S)
            if not frame.ok:
                raise MeasurementError("scale-cal %s frame: %s" % (axis, frame.reason))
            shift = ps.image_shift_px(ref_dest, dest)
            if shift["response"] < MIN_RESPONSE:
                raise MeasurementError(
                    "scale-cal %s: correlation response %.4f too weak to trust"
                    % (axis, shift["response"]))
            mag = math.hypot(shift["dx_px"], shift["dy_px"])
            if mag < MIN_SHIFT_PX:
                raise MeasurementError(
                    "scale-cal %s: image moved only %.2f px for a %.2f mm move -- below "
                    "the %.1f px trust floor" % (axis, mag, cal_step, MIN_SHIFT_PX))
            shifts[axis] = (shift["dx_px"], shift["dy_px"])
            back = attempt_move(cal_arm, x, y, z, speed=speed, label="%s scale-cal back" % tag)
            if not back.ok:
                raise MeasurementError("could not return from scale-cal %s: %s"
                                       % (axis, back.error))
    finally:
        pass  # cal_arm never left the pinned-Z envelope's Z; nothing to retreat here.

    J = ps.jacobian_from_moves(shifts["x"], shifts["y"], cal_step)
    return ps.decompose(J)


def run_identity_pass(arm: Arm, scope: Microscope, a1: list[float], *, dx_mm: float,
                      dy_mm: float, z: float, working_z: float, xy_envelope: Envelope,
                      z_max_rise_mm: float, out_dir: Path, tag: str,
                      args: argparse.Namespace) -> dict[str, Any]:
    x, y = a1[0] + dx_mm, a1[1] + dy_mm

    # Re-establish XY (working height), THEN move Z-only to the accepted focus
    # plane -- same two-step discipline as run_focus_sweep_at, and for the
    # same reason: the arm may be parked at a different candidate's XY.
    reposition = goto_xy(arm, a1, dx_mm, dy_mm, working_z, xy_envelope,
                        speed=args.speed, label="%s goto XY" % tag)
    if not reposition.ok:
        return {"dx_mm": dx_mm, "dy_mm": dy_mm, "z_mm": z,
               "scale_error": "candidate XY unreachable: %s" % reposition.error,
               "classified": None, "confirmed": []}
    if z != working_z:
        ascend = goto_z(arm, a1, dx_mm, dy_mm, z, z_max_rise_mm, speed=args.speed,
                        label="%s goto focus Z" % tag, takeup=True)
        if not ascend.ok:
            return {"dx_mm": dx_mm, "dy_mm": dy_mm, "z_mm": z,
                   "scale_error": "focus Z unreachable: %s" % ascend.error,
                   "classified": None, "confirmed": []}

    try:
        scale = measure_scale_here(arm, scope, a1, x, y, z, out_dir, tag,
                                   cal_step=args.cal_step, speed=args.speed,
                                   settle=args.settle, orient_tol_deg=READBACK_SLACK_DEG)
    except (MeasurementError, SafetyError) as exc:
        return {"dx_mm": dx_mm, "dy_mm": dy_mm, "z_mm": z, "scale_error": str(exc),
               "classified": None, "confirmed": []}

    prior = _marker.MarkPrior(diameter_mm=args.dot_mm, px_per_mm=scale["px_per_mm_mean"])
    dest = out_dir / ("%s_identity.jpg" % tag)
    frame = scope.grab_frame(dest, ordinal=args.frame_ordinal, timeout_s=FRAME_TIMEOUT_S)
    if not frame.ok:
        return {"dx_mm": dx_mm, "dy_mm": dy_mm, "z_mm": z, "scale": scale,
               "capture_error": frame.reason, "classified": None, "confirmed": []}

    classified = _marker.classify_dots(frame.require(), expect=prior)
    confirmed = classified.get("prior_matched") or []
    return {"dx_mm": dx_mm, "dy_mm": dy_mm, "z_mm": z, "scale": scale,
           "frame": str(dest), "classified": classified, "confirmed": confirmed}


# ---------------------------------------------------------------------------
# (7) Retreat. Deliberately NOT tools.arm.Arm.retreat() -- see docstring.
# ---------------------------------------------------------------------------

def retreat_to_working_height(arm: Arm, a1: list[float], working_z: float,
                              xy_envelope: Envelope, *, speed: float,
                              label: str = "retreat to working height") -> bool:
    """Lower/raise Z to the working height; XY held wherever the arm is.

    tools.arm.Arm.retreat() always targets its envelope's ANCHOR Z, which for
    this script's envelopes is the immutable taught A1 height -- outside every
    Z window this script's own envelopes enforce (they pin Z to the working
    height or to one Z-probe's own rise budget, never to bare A1). Calling the
    generic retreat() here would validate against the wrong window and simply
    fail. "Retreat" in this script means "return to the working height", so
    this function builds that target explicitly, using the same XY-travel
    envelope shape run_structure_pass() uses (Z pinned, XY boxed to
    --xy-radius around A1) -- which is valid from anywhere this script's own
    moves can have left the arm, because every one of them stayed inside that
    box.

    Best-effort and swallows everything: a retreat failure during an abort
    must not itself crash the abort path. Reads back and re-checks before
    returning, per the acceptance spec ("verify by readback before the next
    XY command").
    """
    if not arm.settings.live:
        return True
    try:
        here = arm.pose()
        target_arm = arm.with_envelope(xy_envelope)
        target_arm.move(here[0], here[1], working_z, speed=speed, label=label, takeup=False)
        actual = target_arm.pose()
        xy_envelope.check_readback(actual, where="after %s" % label)
        return True
    except Exception as exc:  # noqa: BLE001 -- best-effort safety path, log and continue
        print("  [safety] RETREAT TO WORKING HEIGHT FAILED: %r -- LOWER THE ARM MANUALLY"
             % (exc,), file=sys.stderr, flush=True)
        return False


# ---------------------------------------------------------------------------
# (8) CLI
# ---------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--execute", action="store_true",
                    help="connect to the arm and move (default: dry-run)")
    ap.add_argument("--host", default=ARM_HOST_DEFAULT)
    ap.add_argument("--xy-radius", type=float, default=4.0,
                    help="XY search half-width around the taught A1, mm (default: "
                         "%(default)s; must stay under one well pitch, 9.0 mm)")
    ap.add_argument("--fov-mm", type=float, nargs=2,
                    default=list(_search.FOV_MM_AT_A1_PLUS_3MM),
                    metavar=("X", "Y"),
                    help="field of view at the working height, mm (default: the measured "
                         "value at A1+3.0mm, %(default)s -- re-measure and pass this "
                         "explicitly if the working height changes)")
    ap.add_argument("--overlap", type=float, default=0.45,
                    help="scan_plan overlap fraction (default: %(default)s -- sized for a "
                         "1.0 mm dot in the default field of view; see --dot-mm)")
    ap.add_argument("--max-probes", type=int, default=MAX_PROBES_DEFAULT)
    ap.add_argument("--top-n", type=int, default=5,
                    help="focus-sweep only the top N XY candidates by structure (default: "
                         "%(default)s)")
    ap.add_argument("--working-rise", type=float, default=WORKING_RISE_MM,
                    help="structure-pass height above taught A1, mm (default: %(default)s)")
    ap.add_argument("--z-max-rise", type=float, default=Z_MAX_RISE_MM,
                    help="Z budget for the focus-pass sweep, mm above taught A1 "
                         "(default: the hardware ceiling, %(default)s)")
    ap.add_argument("--coarse-step", type=float, default=MAX_COARSE_STEP_MM,
                    help="focus-pass Z step, mm (default: %(default)s -- the DOF ceiling; "
                         "may not be raised above it, see the module docstring)")
    ap.add_argument("--dot-mm", type=float, default=1.0,
                    help="printed dot diameter, mm -- sizes both the search coverage "
                         "(via --overlap's derivation) and the identity-pass MarkPrior")
    ap.add_argument("--cal-step", type=float, default=CAL_STEP_MM,
                    help="identity-pass px/mm calibration move, mm (default: %(default)s)")
    ap.add_argument("--speed", type=float, default=SCAN_SPEED_MM_S)
    ap.add_argument("--settle", type=float, default=SETTLE_S_DEFAULT)
    ap.add_argument("--frame-ordinal", type=int, default=2)
    ap.add_argument("--method", default="laplacian",
                    choices=("laplacian", "tenengrad", "brenner"),
                    help="focus-pass scoring method (default: %(default)s). MASKED "
                         "LAPLACIAN IS THE MEASURED DEFAULT AND THE THRESHOLDS BELOW "
                         "ARE ITS NUMBERS. Measured 2026-08-05 over the two saved "
                         "sweeps -- barren vs real-peak peak/median separation: "
                         "laplacian 41.3x, brenner 6.9x, tenengrad 10.1x. Change this "
                         "and you MUST re-derive --min-prominence and "
                         "--min-absolute-score; under tenengrad the barren sweep "
                         "scores prominence 27.3 and would be ACCEPTED.")
    ap.add_argument("--min-mean", type=float, default=_focus.BRIGHTNESS_MIN_MEAN)
    ap.add_argument("--min-prominence", type=float, default=4.0,
                    help="af.judge_scan floor on peak/median, MASKED LAPLACIAN "
                         "(default: %(default)s). Measured on the saved sweeps: "
                         "barren 1.17, real peak 48.16. 4.0 sits in that gap with "
                         "margin on both sides.")
    ap.add_argument("--min-absolute-score", type=float, default=40.0,
                    help="af.judge_scan floor on the raw best score, MASKED LAPLACIAN "
                         "(default: %(default)s). Measured: barren best 10.4, real "
                         "peak best 362.7. Separate from prominence because a single "
                         "dust speck can spike a ratio without ever being sharp.")
    ap.add_argument("--max-unusable-frac", type=float, default=0.5,
                    help="af.judge_scan cap on the unusable fraction of a Z sweep "
                         "(default: %(default)s)")
    ap.add_argument("--require-interior", action="store_true", default=True,
                    help="af.judge_scan: reject a peak pinned at either end of the Z "
                         "sweep (default: on)")
    ap.add_argument("--no-require-interior", dest="require_interior", action="store_false")
    ap.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    ap.add_argument("--clear-errors", action="store_true",
                    help="clear a pre-existing controller fault (inspect the arm first)")
    return ap


# ---------------------------------------------------------------------------
# (9) The procedure
# ---------------------------------------------------------------------------

def run(args: argparse.Namespace) -> int:
    def finite(name: str, v: float) -> float:
        if not math.isfinite(v):
            raise SystemExit("%s must be a finite number, got %r" % (name, v))
        return v

    for name, v in (("--xy-radius", args.xy_radius), ("--overlap", args.overlap),
                    ("--working-rise", args.working_rise), ("--z-max-rise", args.z_max_rise),
                    ("--coarse-step", args.coarse_step), ("--dot-mm", args.dot_mm),
                    ("--cal-step", args.cal_step), ("--speed", args.speed),
                    ("--settle", args.settle), ("--min-mean", args.min_mean),
                    ("--min-prominence", args.min_prominence),
                    ("--min-absolute-score", args.min_absolute_score),
                    ("--max-unusable-frac", args.max_unusable_frac)):
        finite(name, v)
    if not (0 < args.coarse_step <= MAX_COARSE_STEP_MM + 1e-9):
        raise SystemExit("--coarse-step must be in (0, %.2f]; the depth of field here is "
                         "under 0.5 mm and a coarser step aliases past the real peak"
                         % MAX_COARSE_STEP_MM)
    if not (0 < args.z_max_rise <= Z_MAX_RISE_MM + 1e-9):
        raise SystemExit("--z-max-rise must be in (0, %.1f]" % Z_MAX_RISE_MM)
    if not (0 <= args.working_rise <= args.z_max_rise + 1e-9):
        raise SystemExit("--working-rise must be in [0, --z-max-rise]")
    if not (0 < args.speed <= SPEED_MAX_MM_S):
        raise SystemExit("--speed must be in (0, %.1f]" % SPEED_MAX_MM_S)
    if not (0 < args.xy_radius <= _search.DEFAULT_PITCH_MM):
        raise SystemExit("--xy-radius must be in (0, %.1f] -- a seating error past one well "
                         "pitch means the plate is not in the nest" % _search.DEFAULT_PITCH_MM)
    if not (0.0 <= args.overlap < 1.0):
        raise SystemExit("--overlap must be in [0, 1)")
    if not (0.0 <= args.max_unusable_frac <= 1.0):
        raise SystemExit("--max-unusable-frac must be in [0, 1]")
    if args.top_n < 1:
        raise SystemExit("--top-n must be >= 1")
    if args.max_probes < 1:
        raise SystemExit("--max-probes must be >= 1")
    if args.settle < 0:
        raise SystemExit("--settle must be >= 0")

    arm = Arm.from_config(live=args.execute, host=args.host, speed_mm_s=args.speed,
                          clear_errors=args.clear_errors, anchor=ANCHOR)
    scope = Microscope.from_config()

    a1 = arm.location_pose(ANCHOR)
    working_z = a1[2] + args.working_rise
    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    out_dir = args.out_dir / stamp

    xy_envelope = build_xy_envelope(a1, working_rise_mm=args.working_rise,
                                    xy_radius_mm=args.xy_radius)

    plan = build_probe_list(fov_mm=tuple(args.fov_mm), xy_radius_mm=args.xy_radius,
                            overlap=args.overlap, max_probes=args.max_probes)

    print("A1 (taught)   x=%.4f y=%.4f z=%.4f" % (a1[0], a1[1], a1[2]))
    print("working Z     %.4f  (A1+%.2f)" % (working_z, args.working_rise))
    print("Z budget      up to A1+%.2f mm (hard cap %.1f)" % (args.z_max_rise, Z_MAX_RISE_MM))
    print("XY search     +/-%.2f mm, fov %.3fx%.3f mm, overlap %.2f -> %d probe(s), "
         "cap %d" % (args.xy_radius, args.fov_mm[0], args.fov_mm[1], args.overlap,
                     len(plan), args.max_probes))
    print("focus pass    top %d candidate(s), coarse step %.2f mm over A1+[0, %.2f] mm"
         % (args.top_n, args.coarse_step, args.z_max_rise))
    print("output        %s" % out_dir)
    print("mode          %s" % ("LIVE -- THE ARM WILL MOVE" if args.execute else "DRY-RUN"))
    print()

    if not args.execute:
        print("[dry-run] nothing was commanded. %d probe(s) planned; "
             "re-run with --execute to search." % len(plan))
        return 0

    out_dir.mkdir(parents=True, exist_ok=True)

    status = "incomplete"
    all_structure_records: list[dict[str, Any]] = []
    focus_results: list[dict[str, Any]] = []
    identity_results: list[dict[str, Any]] = []
    accepted: dict[str, Any] | None = None

    def _term(signum, _frame):
        raise _Terminated("received signal %d" % signum)

    for sig in (signal.SIGTERM, signal.SIGHUP):
        try:
            signal.signal(sig, _term)
        except (ValueError, OSError):
            pass

    try:
        with arm.occupied("find_target: coupled XY/Z search"):
            try:
                # -- Step 1: confirm we are actually at the taught anchor -----
                check_env = Envelope.anchored(
                    a1, name="A1", z_min_rise_mm=0.0, z_max_rise_mm=0.0,
                    xy_max_mm=0.0, orient_tol_deg=READBACK_SLACK_DEG,
                    readback_slack_mm=READBACK_SLACK_MM,
                    readback_slack_deg=READBACK_SLACK_DEG)
                arm = arm.with_envelope(check_env)
                actual = arm.verify_at_anchor(where="find_target: before the search")
                print("[readback] at A1: x=%.3f y=%.3f z=%.3f" % tuple(actual[:3]))

                # -- Step 2: ascend to the working height. Pure Z: XY is (0,0)
                # both before (just verified at A1) and after this move.
                ascend = goto_z(arm, a1, 0.0, 0.0, working_z, args.z_max_rise,
                               speed=args.speed, label="ascend to working height", takeup=True)
                if not ascend.ok:
                    raise FindTargetError("cannot ascend to the working height: %s"
                                          % ascend.error)

                # -- Step 3: structure pass at the working height -------------
                print("\n[structure] %d probe(s) @ Z=A1+%.2f" % (len(plan), args.working_rise))
                # The prior for the LIVE trigger. px/mm is not measured yet at
                # this point, so the reference scale is used and the band is the
                # generous +/-30%. That is acceptable for a trigger whose only
                # job is to earn a Z sweep; the identity pass later rebuilds the
                # prior from the scale measured at the focal plane.
                trigger_prior = (None if not args.dot_mm else
                                 _marker.MarkPrior(diameter_mm=args.dot_mm,
                                                  px_per_mm=TRIGGER_PX_PER_MM))
                if trigger_prior is not None:
                    lo_b, hi_b = trigger_prior.band_px
                    print("  live dot trigger: %.2f mm mark at %.1f px/mm -> "
                          "accept radius %.0f-%.0f px, grid stops on first hit"
                          % (args.dot_mm, TRIGGER_PX_PER_MM, lo_b, hi_b))

                records = run_structure_pass(arm, scope, a1, plan, xy_envelope, working_z,
                                             out_dir, tag="structure", args=args,
                                             prior=trigger_prior)
                all_structure_records += records
                found_any = any(r.get("has_mask") for r in records)


                # -- Step 4: barren-at-working-height fallback: 2 more planes -
                if not found_any:
                    print("[structure] nothing at the working height; trying "
                         "%d more Z plane(s)" % len(STRUCTURE_EXTRA_Z_FRACTIONS))
                    remaining = args.z_max_rise - args.working_rise
                    current_z = working_z
                    for k, frac in enumerate(STRUCTURE_EXTRA_Z_FRACTIONS, 1):
                        extra_rise = args.working_rise + frac * remaining
                        extra_z = a1[2] + extra_rise
                        extra_env = build_xy_envelope(a1, working_rise_mm=extra_rise,
                                                      xy_radius_mm=args.xy_radius)
                        tag = "structure_extra%d" % k
                        print("  [structure] plane %d/%d @ A1+%.2f"
                             % (k, len(STRUCTURE_EXTRA_Z_FRACTIONS), extra_rise))
                        # XY home at the CURRENT plane, then Z-only to the new
                        # plane -- never a diagonal from wherever the previous
                        # pass's last probe left the arm.
                        transition = goto_plane(arm, a1, from_xy_envelope=xy_envelope,
                                                from_z=current_z, to_z=extra_z,
                                                z_max_rise_mm=args.z_max_rise,
                                                speed=args.speed, label=tag)
                        if not transition.ok:
                            all_structure_records.append(
                                {"stage": tag, "dx_mm": 0.0, "dy_mm": 0.0, "z_mm": extra_z,
                                "status": "unreachable", "reason": transition.error})
                            continue
                        extra_records = run_structure_pass(arm, scope, a1, plan, extra_env,
                                                           extra_z, out_dir, tag=tag, args=args)
                        all_structure_records += extra_records
                        retreat_to_working_height(arm, a1, working_z, xy_envelope,
                                                  speed=args.speed,
                                                  label="%s retreat" % tag)
                        current_z = working_z
                        if any(r.get("has_mask") for r in extra_records):
                            found_any = True
                            break

                if not found_any:
                    n_saturated = sum(1 for r in all_structure_records if r.get("saturated"))
                    n_dark = sum(1 for r in all_structure_records if r.get("dark"))
                    n_captured = sum(1 for r in all_structure_records
                                     if r.get("status") == "captured")
                    if n_captured and n_saturated == n_captured:
                        status = "barren: everywhere saturated"
                        print("\n[BARREN] every captured frame was saturated -- this is an "
                             "EXPOSURE problem, not a position problem. Check illumination.")
                    else:
                        status = "barren: no structure found"
                        print("\n[BARREN] no structure found anywhere searched "
                             "(%d dark, %d saturated of %d captured)."
                             % (n_dark, n_saturated, n_captured))
                    raise FindTargetError(status)

                # -- Step 5: focus pass at the top-N candidates ----------------
                top = rank_by_structure(all_structure_records, top_n=args.top_n)
                # A live trigger outranks any std score: std says "something is
                # here", the trigger says "something the right SIZE is here".
                hits = [r for r in all_structure_records if r.get("dot_trigger")]
                if hits:
                    keyed = {(round(r["dx_mm"], 3), round(r["dy_mm"], 3)) for r in hits}
                    rest = [r for r in top
                            if (round(r["dx_mm"], 3), round(r["dy_mm"], 3)) not in keyed]
                    top = hits + rest
                    print("\n[trigger] %d probe(s) held a size-plausible mark; "
                          "focusing there FIRST" % len(hits))
                print("\n[focus] top %d of %d structured XY, by masked std:"
                     % (len(top), sum(1 for r in all_structure_records if r.get("has_mask"))))
                for r in top:
                    print("  (%+.3f,%+.3f)  std=%.2f mean=%.1f%s%s"
                         % (r["dx_mm"], r["dy_mm"], r["std"], r["mean"],
                            "  DARK" if r["dark"] else "", "  SATURATED" if r["saturated"] else ""))

                accepted_focus = []
                for i, cand in enumerate(top, 1):
                    print("\n  [focus %d/%d] sweeping (%+.3f,%+.3f)"
                         % (i, len(top), cand["dx_mm"], cand["dy_mm"]))
                    fr = run_focus_sweep_at(arm, scope, a1, dx_mm=cand["dx_mm"],
                                            dy_mm=cand["dy_mm"], z_max_rise_mm=args.z_max_rise,
                                            xy_envelope=xy_envelope, out_dir=out_dir,
                                            tag="focus_%d" % i, args=args)
                    focus_results.append(fr)
                    j = fr["judged"]
                    print("    judge_scan: %s%s" % ("usable" if j["usable"] else "NOT usable",
                                                     "" if j["usable"]
                                                     else "  (%s)" % "; ".join(j["reasons"])))
                    if j["usable"]:
                        accepted_focus.append(fr)

                if not accepted_focus:
                    status = "no acceptable focal plane"
                    print("\n[NO FOCAL PLANE] none of the top %d candidates passed judge_scan."
                         % len(top))
                    raise FindTargetError(status)

                # -- Step 6: identity pass, best focus first -------------------
                accepted_focus.sort(key=lambda fr: fr["report"]["best_score"], reverse=True)
                for i, fr in enumerate(accepted_focus, 1):
                    z = fr["report"]["best_z_mm"]
                    print("\n  [identity %d/%d] (%+.3f,%+.3f) @ z=%.4f (A1+%.3f)"
                         % (i, len(accepted_focus), fr["dx_mm"], fr["dy_mm"], z, z - a1[2]))
                    ir = run_identity_pass(arm, scope, a1, dx_mm=fr["dx_mm"], dy_mm=fr["dy_mm"],
                                           z=z, working_z=working_z, xy_envelope=xy_envelope,
                                           z_max_rise_mm=args.z_max_rise, out_dir=out_dir,
                                           tag="identity_%d" % i, args=args)
                    identity_results.append(ir)
                    if ir["confirmed"]:
                        print("    CONFIRMED: %s" % ", ".join(ir["confirmed"]))
                        accepted = {"focus": fr, "identity": ir}
                        break
                    reason = (ir.get("scale_error") or ir.get("capture_error")
                             or (ir["classified"] or {}).get("note") or "no prior-matched dot")
                    print("    not confirmed: %s" % reason)
                    retreat_to_working_height(arm, a1, working_z, xy_envelope,
                                              speed=args.speed,
                                              label="identity_%d retreat" % i)

                if accepted is None:
                    status = "focal plane(s) found, no confirmed dot"
                    print("\n[NO DOT CONFIRMED] %d focal plane(s) passed judge_scan, but "
                         "none classified as a dot of the expected size." % len(accepted_focus))
                    raise FindTargetError(status)

                status = "confirmed"

            except (FindTargetError, SafetyError, ArmError, KeyboardInterrupt, _Terminated) as exc:
                if status == "incomplete":
                    status = "aborted: %s" % (exc if str(exc) else repr(exc))
                print("\n[ABORT] %s" % status, file=sys.stderr, flush=True)
            finally:
                retreat_to_working_height(arm, a1, working_z, xy_envelope,
                                          speed=args.speed, label="final retreat")
    finally:
        manifest = {
            "artifact": "coupled XY/Z target search",
            "produced_at": dt.datetime.now().isoformat(timespec="seconds"),
            "status": status,
            "code": str(Path(__file__).resolve()),
            "entrypoint": " ".join([sys.executable, *sys.argv]),
            "arm_host": args.host,
            "a1_pose": a1,
            "working_z": working_z,
            "z_max_rise_mm": args.z_max_rise,
            "hard_ceiling_mm": Z_MAX_RISE_MM,
            "xy_radius_mm": args.xy_radius,
            "fov_mm": list(args.fov_mm),
            "overlap": args.overlap,
            "n_probes_planned": len(plan),
            "coarse_step_mm": args.coarse_step,
            "top_n": args.top_n,
            "thresholds": {"min_prominence": args.min_prominence,
                          "min_absolute_score": args.min_absolute_score,
                          "max_unusable_frac": args.max_unusable_frac,
                          "require_interior": args.require_interior},
            "dot_mm": args.dot_mm,
            "structure_records": all_structure_records,
            "focus_results": [
                {"dx_mm": fr["dx_mm"], "dy_mm": fr["dy_mm"], "report": fr["report"],
                "judged": fr["judged"]} for fr in focus_results],
            "identity_results": identity_results,
            "accepted": ({"dx_mm": accepted["focus"]["dx_mm"],
                        "dy_mm": accepted["focus"]["dy_mm"],
                        "z_mm": accepted["focus"]["report"]["best_z_mm"],
                        "confirmed": accepted["identity"]["confirmed"]}
                       if accepted else None),
        }
        out_dir.mkdir(parents=True, exist_ok=True)
        (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=2, default=str))
        print("\n[manifest] %s" % (out_dir / "manifest.json"))
        arm.close()

    if accepted is not None:
        print("\n%s" % ("=" * 72))
        print("TARGET CONFIRMED at (%+.3f, %+.3f) mm from A1, z=%.4f (A1+%.3f): %s"
             % (accepted["focus"]["dx_mm"], accepted["focus"]["dy_mm"],
                accepted["focus"]["report"]["best_z_mm"],
                accepted["focus"]["report"]["best_z_mm"] - a1[2],
                ", ".join(accepted["identity"]["confirmed"])))
        return 0

    print("\n%s" % ("=" * 72))
    print("NO TARGET CONFIRMED: %s" % status)
    print("Coverage (%d structure probe(s), %d focus sweep(s), %d identity attempt(s)):"
         % (len(all_structure_records), len(focus_results), len(identity_results)))
    for r in all_structure_records:
        print("  [structure/%s] (%+.3f,%+.3f) %s%s" % (
            r["stage"], r["dx_mm"], r["dy_mm"], r["status"],
            "" if r["status"] != "captured" else
            "  std=%.2f%s%s" % (r.get("std", 0.0), "  dark" if r.get("dark") else "",
                                "  saturated" if r.get("saturated") else "")))
    for fr in focus_results:
        print("  [focus] (%+.3f,%+.3f) %s -- %s" % (
            fr["dx_mm"], fr["dy_mm"],
            "usable" if fr["judged"]["usable"] else "not usable",
            "; ".join(fr["judged"]["reasons"]) or "ok"))
    for ir in identity_results:
        print("  [identity] (%+.3f,%+.3f) confirmed=%r" % (ir["dx_mm"], ir["dy_mm"],
                                                           ir["confirmed"]))
    return 4


def main(argv=None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
