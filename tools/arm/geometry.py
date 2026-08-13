"""The 96-well plate grid, anchored on A1, and its calibration state.

The grid has exactly one anchor. Every well is A1 plus an integer number of
9.0 mm steps, so **correcting A1 corrects all 96 wells** -- and conversely, a
route computed from a stale A1 stays stale until it is rebuilt. Frozen route
caches are the trap: they outlive the A1 they were built from.

**The signs here are the NOMINAL grid, not the authority.** ``col_sign=-1`` /
``row_sign=+1`` describe how the plate is normally mounted; the operator-
calibrated ``axis_map`` in ``~/.sdl_lab/robot_arm/plate_config.json`` is what
actually moves the plate, and ``procedures/plate_imaging.py`` reads it. If the
two disagree, the calibrated file wins and this constant is stale -- do not
"fix" plate_config to match this module.

Belongs to the arm because it is a statement about where the plate sits in the
arm's frame. The camera-side half of the geometry -- how many pixels a
millimetre is -- lives in :mod:`tools.calib`.

Well-grid calibration state with provenance.

KNOWN LIMITATION (documented, not silently "fixed"): two points (A1, A12)
cannot fully disambiguate row-axis handedness, nor can they detect a
mislabeled/reversed probe pair — a third, non-collinear well would be needed
to resolve that ambiguity. This module does not attempt to over-engineer past
the 2-point spec it was given. As a cheap guard it flags `operator_note` when
the recovered geometry looks implausible for a nominally axis-aligned mount
(see `_SANITY_ROTATION_DEG_MAX` / `_SANITY_TRANSLATION_MM_MAX`), but a flag of
"looks fine" is not proof the handedness is correct.
"""
from __future__ import annotations

import dataclasses
import datetime as dt
import json
import math
from pathlib import Path

DEFAULT_PATH = Path.home() / ".sdl_lab" / "robot_arm" / "well_calibration.json"

WELL_PITCH_MM = 9.0
NOMINAL_A1_TO_A12_MM = 99.0  # 9.0 * 11, along -X per axis_map (col_sign=-1)
_SANITY_ROTATION_DEG_MAX = 15.0
_SANITY_SCALE_TOLERANCE = 0.01  # 1%
_SANITY_TRANSLATION_MM_MAX = 5.0 * WELL_PITCH_MM
_HISTORY_KEEP = 20


@dataclasses.dataclass
class WellCalibration:
    a1_pose: tuple
    a12_pose: tuple
    taught_a1_pose: tuple
    translation_mm: tuple
    rotation_deg: float
    scale_along_row: float
    residual_mm: float
    calibrated_at: str
    n_iterations: int
    operator_note: str = ""


def _normalize_deg(angle_deg: float) -> float:
    """Wrap to (-180, 180]."""
    a = math.fmod(angle_deg + 180.0, 360.0)
    if a <= 0:
        a += 360.0
    return a - 180.0


def derive_calibration(
    a1_pose: tuple,
    a12_pose: tuple,
    taught_a1_pose: tuple,
    *,
    residual_mm: float = 0.0,
    n_iterations: int = 0,
) -> WellCalibration:
    """Derive translation/rotation/scale from measured A1, A12 vs the nominal grid."""
    dx_meas = a12_pose[0] - a1_pose[0]
    dy_meas = a12_pose[1] - a1_pose[1]
    measured_len = math.hypot(dx_meas, dy_meas)

    angle_measured = math.degrees(math.atan2(dy_meas, dx_meas))
    angle_nominal = math.degrees(math.atan2(0.0, -NOMINAL_A1_TO_A12_MM))  # 180.0
    rotation_deg = _normalize_deg(angle_measured - angle_nominal)

    scale_along_row = measured_len / NOMINAL_A1_TO_A12_MM if NOMINAL_A1_TO_A12_MM else float("nan")

    translation_mm = (a1_pose[0] - taught_a1_pose[0], a1_pose[1] - taught_a1_pose[1])

    notes = []
    if abs(rotation_deg) > _SANITY_ROTATION_DEG_MAX:
        notes.append(
            f"rotation_deg={rotation_deg:.3f} exceeds sanity bound "
            f"{_SANITY_ROTATION_DEG_MAX} deg — flag for human review, possible "
            f"mislabeled well or handedness error"
        )
    if abs(scale_along_row - 1.0) > _SANITY_SCALE_TOLERANCE:
        notes.append(
            f"scale_along_row={scale_along_row:.4f} deviates "
            f">{_SANITY_SCALE_TOLERANCE*100:.0f}% from nominal"
        )
    trans_mag = math.hypot(*translation_mm)
    if trans_mag > _SANITY_TRANSLATION_MM_MAX:
        notes.append(
            f"translation magnitude {trans_mag:.2f}mm implausibly large relative "
            f"to well_pitch_mm={WELL_PITCH_MM}"
        )
    operator_note = "; ".join(notes) if notes else "ok"

    return WellCalibration(
        a1_pose=tuple(a1_pose), a12_pose=tuple(a12_pose), taught_a1_pose=tuple(taught_a1_pose),
        translation_mm=translation_mm, rotation_deg=rotation_deg,
        scale_along_row=scale_along_row, residual_mm=residual_mm,
        calibrated_at=dt.datetime.now().isoformat(timespec="seconds"),
        n_iterations=n_iterations, operator_note=operator_note,
    )


def well_pose(calibration: WellCalibration, row: int, col: int) -> tuple:
    """Return the (x, y) pose for a well given 0-based row (A=0..) and 1-based col (1..12).

    Applies the calibration's rotation + scale to the nominal grid offset from
    A1, with the measured a1_pose as origin. Zero rotation and unit scale
    reproduces the nominal grid exactly.
    """
    if not (0 <= row <= 7):
        raise ValueError(f"row out of range 0-7: {row}")
    if not (1 <= col <= 12):
        raise ValueError(f"col out of range 1-12: {col}")

    dx_nom = -WELL_PITCH_MM * (col - 1)  # col_sign = -1
    dy_nom = WELL_PITCH_MM * row          # row_sign = +1

    theta = math.radians(calibration.rotation_deg)
    s = calibration.scale_along_row
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    dx = s * (cos_t * dx_nom - sin_t * dy_nom)
    dy = s * (sin_t * dx_nom + cos_t * dy_nom)

    x = calibration.a1_pose[0] + dx
    y = calibration.a1_pose[1] + dy
    return (x, y)


def save(calibration: WellCalibration, path: Path | None = None) -> None:
    """Save calibration, appending to a bounded history list."""
    target = Path(path) if path is not None else DEFAULT_PATH
    target.parent.mkdir(parents=True, exist_ok=True)
    history = []
    if target.exists():
        try:
            existing = json.loads(target.read_text())
            history = existing.get("history", [])
            if "current" in existing:
                history.append(existing["current"])
        except (json.JSONDecodeError, OSError):
            history = []
    history = history[-(_HISTORY_KEEP - 1):]
    payload = {"current": dataclasses.asdict(calibration), "history": history}
    target.write_text(json.dumps(payload, indent=2))


def load(path: Path | None = None) -> WellCalibration:
    target = Path(path) if path is not None else DEFAULT_PATH
    data = json.loads(target.read_text())
    current = data["current"]
    return WellCalibration(
        a1_pose=tuple(current["a1_pose"]),
        a12_pose=tuple(current["a12_pose"]),
        taught_a1_pose=tuple(current["taught_a1_pose"]),
        translation_mm=tuple(current["translation_mm"]),
        rotation_deg=current["rotation_deg"],
        scale_along_row=current["scale_along_row"],
        residual_mm=current["residual_mm"],
        calibrated_at=current["calibrated_at"],
        n_iterations=current["n_iterations"],
        operator_note=current.get("operator_note", ""),
    )
