"""Where a well is, in arm coordinates. One implementation, two models.

WHY THIS EXISTS
---------------
The arithmetic "well n is A1 plus an integer number of 9 mm steps" had been
written out five times -- in ``calibrate``, ``column_calibration``,
``plate_frame_scan``, ``well_survey`` and ``plate_imaging`` -- each with its own
copy of the axis-map sign handling. That is five chances for a sign to drift,
and a sign error here mis-addresses all 96 wells while looking perfectly
plausible. It lives here once.

TWO MODELS, deliberately distinguished by name at the call site:

* :func:`nominal_xy` -- the grid as CONFIGURED: integer steps of the nominal
  pitch along the arm's own axes, signs from ``plate_config.json``. Correct only
  if the plate is square to the arm, which a hand-seated plate is not. Measured
  2026-08-11: the nominal grid was **1.3686 mm** out at A12, most of a field of
  view.

* :func:`measured_xy` -- the grid as MEASURED, from a stored calibration's
  origin and step vectors. This is the one to use once a calibration exists.

Neither decides anything. A caller that wants "the best available" asks for it
explicitly via :func:`best_xy`, which reports which model it used.
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any

#: Rows A-H, columns 1-12.
MAX_ROW = 7
MAX_COL = 11


class WellNameError(ValueError):
    """A well name that is not on a 96-well plate."""


def parse_well(name: str) -> tuple[int, int]:
    """``'A1' -> (0, 0)``, ``'B1' -> (1, 0)``, ``'H12' -> (7, 11)``.

    Raises rather than clamping: a typo'd well name must not silently become a
    neighbouring one.
    """
    text = str(name).strip().upper()
    if len(text) < 2 or not text[0].isalpha() or not text[1:].isdigit():
        raise WellNameError(f"{name!r} is not a well name like 'A1' or 'H12'")
    row = ord(text[0]) - ord("A")
    col = int(text[1:]) - 1
    if not (0 <= row <= MAX_ROW) or not (0 <= col <= MAX_COL):
        raise WellNameError(
            f"{name!r} is outside a 96-well plate (rows A-H, columns 1-12)")
    return row, col


def well_name(row: int, col: int) -> str:
    return f"{chr(ord('A') + row)}{col + 1}"


def nominal_xy(a1_xy, row: int, col: int, axis_map: dict, pitch_mm: float = 9.0):
    """The grid as CONFIGURED. See the module docstring for when this is wrong.

    ``axis_map`` is ``plate_config.json``'s: which arm axis each plate axis runs
    along, and with which sign. The sign inversion (the arm travels opposite to
    the plate-side vector, because the plate moves and the camera does not) is
    already absorbed into those signs -- do not apply it again.
    """
    x, y = float(a1_xy[0]), float(a1_xy[1])
    dc = float(pitch_mm) * col * float(axis_map["col_sign"])
    dr = float(pitch_mm) * row * float(axis_map["row_sign"])
    if axis_map["col_axis"] == "x":
        x += dc
    else:
        y += dc
    if axis_map["row_axis"] == "x":
        x += dr
    else:
        y += dr
    return x, y


def measured_xy(calibration: dict, row: int, col: int) -> tuple[float, float]:
    """The grid as MEASURED: origin plus measured step vectors.

    Note the row step in a column-only calibration is the NOMINAL one and the
    calibration says so in ``row_axis``. Callers that care should check it
    rather than assume both axes were measured.
    """
    o = calibration["a1_centred"]
    c = calibration["col_step_mm"]
    r = calibration["row_step_mm"]
    return (float(o[0]) + float(c[0]) * col + float(r[0]) * row,
            float(o[1]) + float(c[1]) * col + float(r[1]) * row)


def best_xy(row: int, col: int, *, a1_xy, axis_map: dict, pitch_mm: float = 9.0,
            calibration: dict | Path | str | None = None,
            ) -> tuple[tuple[float, float], str]:
    """Measured grid if one is available, else nominal. Returns (xy, model).

    ``model`` is ``"measured"`` or ``"nominal"``. It is returned rather than
    logged so a caller can record WHICH grid a result came from -- a position
    from the nominal grid and one from the measured grid are not interchangeable
    evidence.
    """
    cal: dict[str, Any] | None = None
    if isinstance(calibration, (str, Path)):
        try:
            cal = json.loads(Path(calibration).read_text())
        except (OSError, ValueError):
            cal = None
    elif isinstance(calibration, dict):
        cal = calibration
    if cal and "a1_centred" in cal and "col_step_mm" in cal:
        return measured_xy(cal, row, col), "measured"
    return nominal_xy(a1_xy, row, col, axis_map, pitch_mm), "nominal"
