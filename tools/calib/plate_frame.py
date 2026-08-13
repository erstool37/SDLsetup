"""The plate's own row/column frame, solved from the three printed dots.

===========================================================================
1. WHY THIS EXISTS
===========================================================================
The plate is seated by hand, so it lands slightly turned. Every well is A1
plus an integer number of 9.0 mm steps (``tools/arm/geometry.py``), and that
arithmetic assumes the grid is square to the arm's X/Y axes. It is not. Over
the full plate the error compounds: a 1 deg seating error puts H12, 117 mm
from A1, off by 2.0 mm -- most of a well.

Three dots are printed on the plate for this: BLACK at A1, RED at B1 (one
ROW step), BLUE at A2 (one COLUMN step). The arm can drive each in turn under
the fixed objective and record the arm XY at which it is centred. Two vectors
of known true length, along the plate's own axes, are enough to recover the
whole 2x2 grid-to-arm map.

The operator's ask -- "match how parallel the rows are", make the black-red
and black-blue lines straight -- is exactly ``rotation_deg`` driven to zero.

WHY THREE DOTS AND NOT TWO. The objective's field of view is about
2.84 x 2.38 mm at the working height, far smaller than the 9 mm pitch, so no
two dots are ever in frame together and a single image can never measure this.
Each dot must be visited. Two dots (A1, A12) is what ``tools/arm/geometry.py``
already does, and its docstring records the known limitation: two points on
one axis cannot separate rotation from a row-axis error. A third,
non-collinear dot is what closes that gap, which is the whole point here.

===========================================================================
2. THE CONVENTION -- READ BEFORE TOUCHING ANYTHING
===========================================================================
The three inputs are **arm poses, not plate-side vectors**: ``p_red`` is the
arm XY at which the B1 dot sits at the camera centre. That is the same
quantity the rest of the repo already means by a "well pose"
(``tools/arm/geometry.well_pose``, ``scripts/tool_building/plate_imaging.well_offset``):
the pose you drive to in order to image that well.

Physically the plate moves and the camera does not, so the arm must travel
*opposite* to the plate-side vector between two wells. That inversion is
already absorbed into the ``axis_map`` signs in
``~/.sdl_lab/robot_arm/plate_config.json`` -- ``col_sign=-1`` says A2 is
imaged at arm X *minus* 9 mm. Do not apply it a second time here. Everything
below is in pose space, where

    arm_offset_from_A1 = M @ [col_steps, row_steps]

and M's first column is the measured A1->A2 step, its second the A1->B1 step.
M is in mm per grid step, so its columns are ~9 mm long, not unit vectors.

===========================================================================
3. WHAT THE DECOMPOSITION MEANS
===========================================================================
M is compared against the nominal grid M_nom built from the axis map, NOT
against the bare arm axes -- M_nom's determinant is negative (col -> -X,
row -> +Y is a reflection, exactly as ``pixel_scale`` notes for the camera
Jacobian), so "angle from the arm +X axis" would be a meaningless mixture of
that reflection and the seating error.

  rotation_deg           mean of the two per-axis turns from nominal, positive
                         from arm +X toward arm +Y. This is the seating error,
                         and the number the operator drives to zero.
  scale_col / scale_row  measured step length / pitch_mm. The true length is
                         9.0 mm by construction of the plate, so a scale that
                         is not 1.0 is a *measurement or seating fault* -- a
                         dot centred on the wrong well, a plate not seated
                         flat, or a wrong pitch assumption -- never a
                         correction to apply.
  non_orthogonality_deg  departure of the measured row-to-column angle from
                         the nominal 90 deg. A real plate's axes are square:
                         its wells are drilled, not hand-placed. So this is a
                         QUALITY FLAG, not a shear to correct out. A large
                         value means one of the three centrings is wrong.
  residual_*_mm          how far each measured step vector lands from where
                         the nominal axis map said it would. Reported in mm
                         because that is the unit the operator can act on.

===========================================================================
4. WHAT THIS MODULE DOES NOT DO
===========================================================================
It measures and flags; it never decides. It will not re-order a bad reading,
retry a centring, pick which dot to visit next, or declare a calibration
converged -- those belong to the caller under ``scripts/``. Degenerate input
comes back as ``quality="degenerate"`` with the offending number in
``reason``, and with ``inverse`` and ``rotation_deg`` set to None rather than
to a fabricated matrix, so a caller that ignores the flag fails loudly
instead of steering on invented geometry.

Pure arithmetic on six floats: no numpy, no hardware, no I/O. Kept
dependency-free deliberately -- a 2x2 inverse written out is easier to audit
for a sign error than a library call, and this module must stay importable
wherever a pose is available.
"""
from __future__ import annotations

import dataclasses
import math

#: Well pitch of an SBS 96-well plate, mm. A property of the plate, measured
#: by its manufacturer, not by us. Matches tools/arm/geometry.WELL_PITCH_MM.
PITCH_MM = 9.0

#: Nominal grid-to-arm axis map, as read from
#: ~/.sdl_lab/robot_arm/plate_config.json (col_sign=-1, row_sign=+1) and as
#: documented in tools/arm/geometry.py. That file is the authority: pass the
#: loaded axis_map in rather than trusting this copy if the plate is remounted.
DEFAULT_AXIS_MAP = {"col_axis": "x", "col_sign": -1, "row_axis": "y", "row_sign": 1}

#: Last row / column index of a 96-well plate, 0-based from A1 (H = 7, 12 = 11).
MAX_ROW_INDEX = 7
MAX_COL_INDEX = 11

#: Shortest step vector accepted, mm. The true step is 9.0 mm and the field of
#: view is only ~2.8 mm wide, so two different dots can never be centred at
#: nearly the same pose: anything under 1 mm means the same dot was centred
#: twice, or a pose was recorded before the arm finished moving.
MIN_VECTOR_MM = 1.0

#: |sin| of the angle between the two measured step vectors, below which the
#: map cannot be inverted meaningfully. 0.1 is ~5.7 deg from parallel; at that
#: point rotation and non-orthogonality are no longer separable and the
#: inverse amplifies centring noise by >10x.
MIN_SIN_BETWEEN = 0.1

#: Departure from square, deg, above which the measurement is called suspect.
#: Over the 9 mm baseline 2 deg is 0.31 mm -- an order of magnitude worse than
#: the 0.02 mm centring tolerance in tools/calib/centering.py, so it cannot be
#: centring noise.
MAX_NON_ORTHOGONALITY_DEG = 2.0

#: Fractional departure of a measured step from 9.0 mm before it is suspect.
#: 1%, matching _SANITY_SCALE_TOLERANCE in tools/arm/geometry.py.
MAX_SCALE_DEVIATION = 0.01

#: Seating rotation, deg, beyond which the plate is more likely mislabelled or
#: mis-seated than merely turned. 15 deg matches _SANITY_ROTATION_DEG_MAX in
#: tools/arm/geometry.py.
MAX_ROTATION_DEG = 15.0

QUALITY_OK = "ok"
QUALITY_SUSPECT = "suspect"
QUALITY_DEGENERATE = "degenerate"


class PlateFrameError(ValueError):
    """Raised for malformed INPUT: a bad axis map, pitch, point, or well index.

    A bad *measurement* is not an error and does not raise. Zero-length or
    collinear step vectors come back as a PlateFrame with
    ``quality=QUALITY_DEGENERATE``, because a reading the operator can look at
    is more useful than a traceback -- and because judging what to do about it
    is the caller's job, not this module's.
    """


@dataclasses.dataclass(frozen=True)
class PlateFrame:
    """The solved grid-to-arm map, its decomposition, and how much to trust it.

    ``matrix`` is row-major: ``matrix[i][j]`` is arm axis i, grid axis j, so
    ``matrix[0]`` is a matrix row and *not* the plate's row axis. The plate's
    axes are column 0 (grid columns, A1->A2) and column 1 (grid rows, A1->B1);
    they are also given plainly as ``col_vec_mm`` and ``row_vec_mm``.

    ``inverse``, ``rotation_deg`` and ``non_orthogonality_deg`` are None when
    they could not be computed. Everything else is always a real measurement:
    the vectors, scales, and residuals are just differences of the recorded
    poses and stay meaningful even when the solve fails.
    """

    p_black: tuple
    p_red: tuple
    p_blue: tuple
    pitch_mm: float
    col_axis: str
    col_sign: int
    row_axis: str
    row_sign: int
    col_vec_mm: tuple
    row_vec_mm: tuple
    matrix: tuple
    inverse: tuple | None
    rotation_deg: float | None
    scale_col: float
    scale_row: float
    non_orthogonality_deg: float | None
    residual_col_mm: float
    residual_row_mm: float
    residual_mm: float
    quality: str
    reason: str


# ---------------------------------------------------------------------------
# Small geometry helpers. Written out rather than pulled from numpy so the sign
# of every cross product is visible at the point it is used.
# ---------------------------------------------------------------------------

def _wrap_deg(angle_deg: float) -> float:
    """Wrap to (-180, 180], matching _normalize_deg in tools/arm/geometry.py."""
    a = math.fmod(angle_deg + 180.0, 360.0)
    if a <= 0:
        a += 360.0
    return a - 180.0


def _signed_angle_deg(a: tuple, b: tuple) -> float:
    """Signed angle from vector a to vector b, positive from arm +X toward +Y.

    atan2 of the cross product over the dot product, so it is correct through
    the full turn and does not lose the sign the way acos would.
    """
    cross = a[0] * b[1] - a[1] * b[0]
    dot = a[0] * b[0] + a[1] * b[1]
    return math.degrees(math.atan2(cross, dot))


def _unit_axis(axis: str, sign: int) -> tuple:
    return (float(sign), 0.0) if axis == "x" else (0.0, float(sign))


def _xy(point, name: str) -> tuple:
    """First two coordinates of a point, so a full 6-DOF arm pose can be passed."""
    try:
        return (float(point[0]), float(point[1]))
    except (TypeError, IndexError, KeyError, ValueError) as exc:
        raise PlateFrameError(
            "%s must be an (x, y) pair or a longer pose whose first two entries "
            "are numbers; got %r" % (name, point)
        ) from exc


def _normalize_axis_map(axis_map) -> tuple:
    """Validate and flatten an axis map to (col_axis, col_sign, row_axis, row_sign).

    Extra keys such as ``calibrated`` are ignored, so the dict loaded straight
    out of plate_config.json can be handed over unedited.
    """
    src = DEFAULT_AXIS_MAP if axis_map is None else axis_map
    try:
        col_axis = str(src["col_axis"]).lower()
        row_axis = str(src["row_axis"]).lower()
        col_sign = int(src["col_sign"])
        row_sign = int(src["row_sign"])
    except (KeyError, TypeError, ValueError) as exc:
        raise PlateFrameError(
            "axis_map needs col_axis/col_sign/row_axis/row_sign; got %r" % (src,)
        ) from exc
    if col_axis not in ("x", "y") or row_axis not in ("x", "y"):
        raise PlateFrameError(
            "axis_map axes must be 'x' or 'y'; got col_axis=%r row_axis=%r" % (col_axis, row_axis)
        )
    if col_axis == row_axis:
        raise PlateFrameError(
            "axis_map maps both grid axes onto arm %r; the nominal grid would be "
            "singular and there would be nothing to compare a measurement against" % col_axis
        )
    if col_sign not in (1, -1) or row_sign not in (1, -1):
        raise PlateFrameError(
            "axis_map signs must be +1 or -1; got col_sign=%r row_sign=%r" % (col_sign, row_sign)
        )
    return (col_axis, col_sign, row_axis, row_sign)


# ---------------------------------------------------------------------------
# The solve
# ---------------------------------------------------------------------------

def solve_plate_frame(p_black, p_red, p_blue, *, pitch_mm: float = PITCH_MM,
                      axis_map=None) -> PlateFrame:
    """Recover the grid-to-arm map from the three dot-centring poses.

    Args:
        p_black: arm XY (or full pose) at which the A1 dot is at frame centre.
        p_red:   arm XY at which the B1 dot is centred -- one ROW step from A1.
        p_blue:  arm XY at which the A2 dot is centred -- one COLUMN step from A1.
        pitch_mm: true well pitch. Both step vectors are 9.0 mm long on a real
            plate, which is what makes the per-axis scale a fault indicator
            rather than a free parameter.
        axis_map: the nominal grid orientation to measure against; defaults to
            DEFAULT_AXIS_MAP. Pass the one loaded from plate_config.json.

    Returns:
        PlateFrame. Never raises on a bad *reading* -- a zero-length or
        collinear pair comes back with quality=QUALITY_DEGENERATE, inverse and
        rotation_deg None, and the offending number quoted in reason. Raises
        PlateFrameError only when the input itself is malformed.
    """
    col_axis, col_sign, row_axis, row_sign = _normalize_axis_map(axis_map)
    if not isinstance(pitch_mm, (int, float)) or not math.isfinite(pitch_mm) or pitch_mm <= 0:
        raise PlateFrameError("pitch_mm must be a positive finite number; got %r" % (pitch_mm,))
    pitch_mm = float(pitch_mm)

    black = _xy(p_black, "p_black")
    red = _xy(p_red, "p_red")
    blue = _xy(p_blue, "p_blue")

    # Column axis is A1 -> A2 (one column step), row axis is A1 -> B1 (one row
    # step). Both are pose differences; see convention note in the module
    # docstring before "fixing" an apparent sign inversion here.
    col_vec = (blue[0] - black[0], blue[1] - black[1])
    row_vec = (red[0] - black[0], red[1] - black[1])

    nominal_col = tuple(c * pitch_mm for c in _unit_axis(col_axis, col_sign))
    nominal_row = tuple(c * pitch_mm for c in _unit_axis(row_axis, row_sign))

    len_col = math.hypot(col_vec[0], col_vec[1])
    len_row = math.hypot(row_vec[0], row_vec[1])
    residual_col = math.hypot(col_vec[0] - nominal_col[0], col_vec[1] - nominal_col[1])
    residual_row = math.hypot(row_vec[0] - nominal_row[0], row_vec[1] - nominal_row[1])

    matrix = ((col_vec[0], row_vec[0]),
              (col_vec[1], row_vec[1]))

    base = dict(
        p_black=black, p_red=red, p_blue=blue, pitch_mm=pitch_mm,
        col_axis=col_axis, col_sign=col_sign, row_axis=row_axis, row_sign=row_sign,
        col_vec_mm=col_vec, row_vec_mm=row_vec, matrix=matrix,
        scale_col=len_col / pitch_mm, scale_row=len_row / pitch_mm,
        residual_col_mm=residual_col, residual_row_mm=residual_row,
        # The larger of the two, not the mean: one good axis must not average
        # away one bad one.
        residual_mm=max(residual_col, residual_row),
    )

    def _degenerate(reason: str, non_orthogonality_deg=None) -> PlateFrame:
        return PlateFrame(inverse=None, rotation_deg=None,
                          non_orthogonality_deg=non_orthogonality_deg,
                          quality=QUALITY_DEGENERATE, reason=reason, **base)

    # NaN fails every comparison below silently, so it is caught first rather
    # than allowed to walk out the far end as a plausible-looking matrix.
    if not all(math.isfinite(v) for v in black + red + blue):
        return _degenerate(
            "a recorded pose is not finite: p_black=%r p_red=%r p_blue=%r" % (black, red, blue)
        )

    if len_col < MIN_VECTOR_MM or len_row < MIN_VECTOR_MM:
        return _degenerate(
            "step vector too short to be a real 9 mm move (|A1->A2|=%.4f mm, "
            "|A1->B1|=%.4f mm, minimum %.1f mm): the same dot was probably "
            "centred twice" % (len_col, len_row, MIN_VECTOR_MM)
        )

    theta_col = _signed_angle_deg(nominal_col, col_vec)
    theta_row = _signed_angle_deg(nominal_row, row_vec)
    # Identically the departure of the measured row-to-column angle from the
    # nominal one: the two nominal terms cancel in the difference.
    non_orthogonality = _wrap_deg(theta_row - theta_col)

    sin_between = abs(col_vec[0] * row_vec[1] - col_vec[1] * row_vec[0]) / (len_col * len_row)
    if sin_between < MIN_SIN_BETWEEN:
        return _degenerate(
            "the two measured axes are within %.2f deg of collinear "
            "(|sin|=%.4f < %.2f); rotation and non-orthogonality cannot be "
            "separated and the map cannot be inverted"
            % (math.degrees(math.asin(min(sin_between, 1.0))), sin_between, MIN_SIN_BETWEEN),
            non_orthogonality_deg=non_orthogonality,
        )

    # Mean of the two per-axis turns, computed through the wrapped difference so
    # it stays correct if one angle sits just across the +-180 seam.
    rotation = _wrap_deg(theta_col + non_orthogonality / 2.0)

    # |det| = len_col * len_row * sin_between >= MIN_VECTOR_MM^2 * MIN_SIN_BETWEEN
    # by the two guards above, so this division is safe by construction.
    det = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]
    inverse = ((matrix[1][1] / det, -matrix[0][1] / det),
               (-matrix[1][0] / det, matrix[0][0] / det))

    faults = []
    if abs(non_orthogonality) > MAX_NON_ORTHOGONALITY_DEG:
        faults.append(
            "row and column axes are %.3f deg from square (limit %.1f); a real "
            "plate's axes ARE square, so this is a bad measurement to repeat, "
            "not a shear to correct" % (non_orthogonality, MAX_NON_ORTHOGONALITY_DEG)
        )
    for name, scale in (("column", base["scale_col"]), ("row", base["scale_row"])):
        if abs(scale - 1.0) > MAX_SCALE_DEVIATION:
            faults.append(
                "%s step is %.4f x the true %.1f mm pitch (%.3f mm), beyond %.0f%%: "
                "check the dot was centred on the well it is named for"
                % (name, scale, pitch_mm, scale * pitch_mm, MAX_SCALE_DEVIATION * 100.0)
            )
    if abs(rotation) > MAX_ROTATION_DEG:
        faults.append(
            "seating rotation %.3f deg exceeds %.1f deg; more likely a swapped "
            "red/blue dot than a plate that far out" % (rotation, MAX_ROTATION_DEG)
        )

    return PlateFrame(
        inverse=inverse, rotation_deg=rotation, non_orthogonality_deg=non_orthogonality,
        quality=QUALITY_SUSPECT if faults else QUALITY_OK,
        reason="; ".join(faults) if faults else "ok",
        **base,
    )


# ---------------------------------------------------------------------------
# Using the solved frame
# ---------------------------------------------------------------------------

def _grid_index(name: str, value, limit: int) -> int:
    try:
        as_int = int(value)
    except (TypeError, ValueError) as exc:
        raise PlateFrameError("%s must be a whole number; got %r" % (name, value)) from exc
    if as_int != value or not 0 <= as_int <= limit:
        raise PlateFrameError(
            "%s must be a whole number in 0..%d, counted from A1; got %r" % (name, limit, value)
        )
    return as_int


def well_offset(frame: PlateFrame, row: int, col: int) -> tuple:
    """Arm XY offset in mm from the A1 pose to any well, through the solved frame.

    ``row`` and ``col`` are 0-BASED STEP COUNTS FROM A1 -- A1 is (0, 0), B1 is
    (1, 0), A2 is (0, 1), H12 is (7, 11) -- the same counts the three dots
    define. They are NOT the mixed 0-based-row / 1-based-column pair that
    ``tools/arm/geometry.well_pose`` takes.

    Going through the frame is the entire point: it applies the measured
    rotation to the grid, so a turned plate's far wells come out where they
    actually are instead of where an axis-aligned grid would put them. For a
    5 deg seating error that is 10.2 mm at H12.

    Raises PlateFrameError on a degenerate frame -- there is no honest offset
    to return from a map that could not be solved. A ``suspect`` frame is
    computed normally: flagging it is this module's job, deciding whether to
    use it is the caller's.
    """
    row = _grid_index("row", row, MAX_ROW_INDEX)
    col = _grid_index("col", col, MAX_COL_INDEX)
    if frame.quality == QUALITY_DEGENERATE:
        raise PlateFrameError(
            "cannot compute a well offset from a degenerate plate frame: %s" % frame.reason
        )
    m = frame.matrix
    return (m[0][0] * col + m[0][1] * row,
            m[1][0] * col + m[1][1] * row)


def yaw_correction_deg(frame: PlateFrame) -> float:
    """Wrist yaw DELTA that squares the plate to the arm axes, in degrees.

    SIGN CONVENTION -- the part that must not be guessed:

    ``rotation_deg`` is how far the plate's grid is turned from nominal,
    positive from arm +X toward arm +Y. Removing it means turning the plate the
    other way, so this returns ``-rotation_deg``. It is a DELTA to ADD to the
    pose's yaw field::

        new_yaw = current_pose[5] + yaw_correction_deg(frame)

    and it assumes that increasing that field turns the held plate in the same
    positive +X-toward-+Y sense. If the rig's yaw runs the other way, adding
    this delta DOUBLES the seating error instead of removing it -- 5 deg
    becomes 10 -- which is silent, because the move succeeds and only the far
    wells miss. Confirm the sense once on hardware with a deliberate small
    turn before trusting a correction; the doubling is exercised as a test but
    only against this module's own convention, which cannot prove the arm's.

    Raises PlateFrameError on a degenerate frame rather than returning 0.0: a
    zero correction and an unsolvable frame must not look alike to the caller.
    """
    if frame.quality == QUALITY_DEGENERATE or frame.rotation_deg is None:
        raise PlateFrameError(
            "no rotation was solved, so there is no yaw correction to make: %s" % frame.reason
        )
    return _wrap_deg(-frame.rotation_deg)
