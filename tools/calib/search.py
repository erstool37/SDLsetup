"""
search.py -- find a printed plate dot when the plate did not seat where it was taught.

Pure geometry: plan generation, a coverage metric, and the sighting-to-move
arithmetic. It opens no camera, commands no motion, and reads no files. It
reports numbers and the assumptions behind them; the driver in ``scripts/``
decides whether to move, when to give up, and whether a reading is good enough
(surface law: tools report, scripts decide).

===========================================================================
1. WHY A STEP PICKED BY FEEL FAILS HERE
===========================================================================
The objective camera is a Leica K3C, 2448 x 2048 px, and the pixel scale was
measured at 861 px/mm along arm X at A1 + 3.0 mm. Magnification depends on
height, so that number is the scale AT THAT HEIGHT and nowhere else --
:mod:`tools.calib.pixel_scale` fits s(z) when another height is needed.

    field of view = 2448 / 861 x 2048 / 861 = 2.84 x 2.38 mm
    well pitch    = 9.0 mm

The field is about a third of a pitch across. Between two adjacent dots lies
roughly 6 mm of blank plate, so a search step chosen because it "seemed about
right" does one of two bad things: a step well under the field re-images ground
it has already covered and burns moves, and a step over the field steps clean
across the dot and then reports, confidently, that there is nothing there.

Hence the two rules this module is built around. The step is never a literal:
it is ``fov * (1 - overlap)``, derived from the field the caller measured. And
the coverage claim is not asserted in a comment, it is computed --
``coverage_gap_mm`` returns the largest gap the plan leaves, so a caller or a
test can require it to be <= 0 instead of trusting this docstring.

===========================================================================
2. THREE DOTS, NOT ONE -- WHAT MAKES THE SEARCH TRACTABLE
===========================================================================
The plate carries three printed dots, one pitch apart:

    black at A1     red at B1 (one ROW from A1)     blue at A2 (one COLUMN)

Letter = row, number = column, so B1 differs from A1 by a row and A2 by a
column. Because the colour identifies the well, seeing ANY ONE of the three
fixes the plate's position completely: the search does not have to find the
black dot, it has to find *a* dot, and ``resolve_from_sighting`` then computes
the move straight to black.

That matters because it turns a rare, expensive event (missing the dot
entirely and having to widen the search) into a common, cheap one (landing on
the wrong dot and stepping 9 mm). It also means an overshooting search still
succeeds instead of silently failing.

===========================================================================
3. WHY A RING SPIRAL RATHER THAN A BOUSTROPHEDON
===========================================================================
The coverage guarantee below is a property of the LATTICE, not of the order the
lattice is walked in, so both patterns cover the same ground with the same
number of points. The order is therefore chosen purely on expected cost:

* The prior is concentrated at the centre. The taught pose is unbiased and the
  seating error is usually small, so the dot is usually within a point or two
  of the nominal start. A boustrophedon raster begins at a corner of the search
  box and reaches the centre only halfway through, meaning the typical run pays
  the worst case's cost before it tries the likely case. An outward ring spiral
  tries the likely case first and, on a typical run, stops after a handful of
  points.
* Consecutive spiral points are lattice neighbours, so every move is one step
  (~2.4 mm here) and the arm never crosses the whole search box at speed.
* ``ring`` is a progress log a human can read: "ring 2 of 4".

The cost of the spiral, stated rather than hidden: the two steps differ
(2.42 mm along X, 2.02 mm along Y at the default overlap), so ``ring`` is a
Chebyshev distance in GRID UNITS, not a radius in millimetres. Ring order is
therefore only approximately outward in mm -- a ring-3 point on the short axis
sits 6.07 mm out while a ring-2 corner sits 6.30 mm out. ``ScanPoint`` carries
``radius_mm`` so a caller that genuinely wants strict nearest-first ordering can
re-sort; it will trade arm travel for it.

===========================================================================
4. THE COVERAGE GUARANTEE, AND HOW TO CHECK IT
===========================================================================
At a scan point the camera covers an arm-XY rectangle ``fov_w`` x ``fov_h``
centred on that point. (Axis-aligned with the arm: see ``arm_aligned_fov_mm``
for the rotated case, which this rig has, because the plate seating is not
perfectly repeatable in angle.)

A dot of diameter ``d`` is imaged ENTIRELY inside the frame only when its centre
lies within that rectangle shrunk by d/2 on every side -- half-widths
(fov_w - d)/2 and (fov_h - d)/2. Two lattice-adjacent scan points therefore
leave no gap in the whole-dot sense exactly when

    step_axis <= fov_axis - d                                            (1)

and the plan sets ``step_axis = fov_axis * (1 - overlap)``, so (1) becomes

    d <= fov_axis * overlap                                              (2)

GUARANTEE. With ``step = fov * (1 - overlap)``, a dot of diameter

    d <= overlap * min(fov_w, fov_h)

whose centre lies anywhere within ``max_offset_mm`` of the nominal start is
imaged entirely inside the frame at at least one point of the plan.

At the default overlap of 0.15 and the measured field that is d <= 0.357 mm.
The relation is linear, so a 1.0 mm dot needs overlap >= 1.0 / 2.38 = 0.42, and
a caller that knows its dot size should set ``overlap`` from it rather than
accept the default. Nothing here decides that for the caller; it only makes the
number available.

The weaker claim -- the dot's CENTRE is covered, so at least part of the dot is
somewhere in frame -- needs only ``step <= fov``, which any ``overlap >= 0``
satisfies. ``coverage_gap_mm(plan, fov_mm)`` measures that weaker version, and
passing it ``fov - d`` measures the whole-dot version, since the gap it returns
is ``step - fov`` per axis.

What ``coverage_gap_mm`` does NOT measure: whether the plan's footprint is big
enough for the uncertainty. That is a separate question, answerable from the
``dx_mm``/``dy_mm`` the points carry, and it belongs to the caller who owns
``max_offset_mm``.

===========================================================================
5. SIGNS, ONE AT A TIME
===========================================================================
``resolve_from_sighting`` composes two moves. Both are easy to get backwards,
and a sign error in the second one is a 9 mm move in the wrong direction.

(a) CENTRING. ``offset_px`` is (dot - frame centre) in pixels, u to the right
    and v downward -- the convention of
    ``tools.microscope.imaging.offset_from_center`` and
    ``tools.microscope.marker.best_offset_px``. To bring a dot that sits to the
    RIGHT of centre INTO the centre, the imaged content must move LEFT, so the
    commanded move is the NEGATIVE of the offset converted to millimetres:

        centring_x = -offset_u * IMAGE_AXES_TO_ARM[0] / px_per_mm_x
        centring_y = -offset_v * IMAGE_AXES_TO_ARM[1] / px_per_mm_y

    That negation is the same one carried by ``pixel_scale.correction_mm`` and
    ``PixelToArm.apply``; dropping it inverts the loop and drives the dot out of
    frame instead of into the middle.

(b) THE WELL STEP. ``axis_map`` says where the grid runs in arm coordinates.
    With the operator-calibrated values (col_axis="x", col_sign=-1,
    row_axis="y", row_sign=+1):

        column index +1  ->  arm x - 9.0        so A2 = A1 + (-9, 0)
        row    index +1  ->  arm y + 9.0        so B1 = A1 + ( 0, +9)

    A sighting names the well under the camera -- black A1, red B1, blue A2 --
    and getting back to A1 means UNDOING that well's row and column indices, so
    the step is the negative of the offset that put it there:

        step_on(row_axis) = -row_sign * pitch_mm * row_index
        step_on(col_axis) = -col_sign * pitch_mm * col_index

        red  (row 1, col 0):  y = -(+1) * 9.0 * 1 = -9.0,  x =  0.0
        blue (row 0, col 1):  x = -(-1) * 9.0 * 1 = +9.0,  y =  0.0

    In words, which is the check that catches an algebra slip: B1 lies one row
    at +y from A1, so B1 -> A1 travels -y. A2 lies one column at -x from A1, so
    A2 -> A1 travels +x. Seeing black needs no step at all.

(c) WHAT THE STEP DOES NOT CORRECT. It assumes the plate's rows and columns are
    parallel to the arm axes. A plate rotated by theta leaves a residual of
    about pitch * sin(theta) -- 0.157 mm per degree at a 9 mm pitch -- and a
    pitch error of 1 % leaves another 0.09 mm. Both are far inside the 2.38 mm
    field, so after the move the black dot IS in frame; neither is small enough
    to skip re-centring on it. The returned dict carries the per-degree figure
    so the caller can size its own tolerance instead of inheriting one.

===========================================================================
6. WHAT THIS MODULE DELIBERATELY DOES NOT DO
===========================================================================
It does not move, retry, widen a search that came up empty, reject a sighting it
finds implausible, or declare a search converged. Every one of those is a policy
choice belonging to ``scripts/``. What comes back is a plan, a number, and a
list of the assumptions that number rests on.
"""
from __future__ import annotations

import dataclasses
import math
from typing import Any

#: Objective camera frame, px. Leica K3C. Reference value only -- callers pass
#: their own measured ``fov_mm``, because it changes with height.
FRAME_PX = (2448, 2048)

#: Measured 2026-07-31, along arm X, at A1 + 3.0 mm. Magnification depends on
#: height, so this is the scale at that height and nowhere else.
PX_PER_MM_AT_A1_PLUS_3MM = 861.0

#: The field that follows from the two constants above: 2.843 x 2.379 mm.
#: Recorded so the worked numbers in this docstring can be re-derived, not so
#: that callers skip measuring their own.
FOV_MM_AT_A1_PLUS_3MM = (FRAME_PX[0] / PX_PER_MM_AT_A1_PLUS_3MM,
                         FRAME_PX[1] / PX_PER_MM_AT_A1_PLUS_3MM)

#: Well pitch, mm. The same number as ``tools.arm.geometry.WELL_PITCH_MM``, kept
#: as a default argument rather than an import so this module stays independent
#: of the motion-safety package and importable with no hardware stack present.
DEFAULT_PITCH_MM = 9.0

#: Which well each printed dot sits on, as (row_index, col_index) from A1, both
#: 0-based. Letter = row and number = column, so B1 is one ROW from A1 while A2
#: is one COLUMN from it -- the pair that is easiest to transpose.
DOT_WELLS = {"black": (0, 0), "red": (1, 0), "blue": (0, 1)}

#: Grid orientation as calibrated by the operator in
#: ``~/.sdl_lab/robot_arm/plate_config.json``. Note that the DEFAULTS literal in
#: ``scripts/tool_building/plate_imaging.py`` carries col_sign=+1: the
#: calibrated file is the authority (``tools/arm/geometry.py`` says so), and the
#: two are allowed to disagree. Pass the loaded axis_map explicitly rather than
#: leaning on this copy, which is only a fallback for offline use.
DEFAULT_AXIS_MAP = {"col_axis": "x", "col_sign": -1, "row_axis": "y", "row_sign": 1}

#: Which way each arm axis pushes the image. Moving the arm +1 mm along arm X
#: moves the imaged content by +px_per_mm_x * IMAGE_AXES_TO_ARM[0] pixels in u
#: (column, rightward); +1 mm along arm Y moves it by
#: +px_per_mm_y * IMAGE_AXES_TO_ARM[1] in v (row, downward).
#:
#: UNVERIFIED on this rig -- it depends on how the camera is bolted on and on
#: whether the objective inverts, neither of which the 861 px/mm measurement
#: constrains. It is echoed in every ``resolve_from_sighting`` result so it can
#: never be applied silently. A camera rotated by more than a degree or two is
#: not describable by two signs at all: use ``PixelToArm.apply`` for the
#: centring half and take only ``well_step_mm`` from this module.
IMAGE_AXES_TO_ARM = (1, 1)

#: Slack allowed when comparing a lattice spacing against its own step. Two
#: exact multiples of the same float can differ from that float by an ulp, and a
#: coverage claim must not turn on 1e-16.
_LATTICE_EPS_MM = 1e-9


class SearchError(ValueError):
    """A search input is malformed, or a plan is not the shape it claims to be."""


@dataclasses.dataclass(frozen=True)
class ScanPoint:
    """One pose to visit, as an offset from the nominal (taught A1) pose.

    ``ix``/``iy`` are the grid indices the offset came from. They are kept
    rather than recomputed so a caller can reason about the lattice without
    re-deriving the step, and so ``coverage_gap_mm`` can confirm the plan really
    is a complete rectangle before trusting its per-axis projection.

    ``ring`` is ``max(|ix|, |iy|)`` -- Chebyshev distance in grid units, which is
    what "outward" means here. ``radius_mm`` is the Euclidean distance in
    millimetres, which is not the same ordering when the two steps differ; see
    section 3 of the module docstring.
    """

    index: int
    ring: int
    ix: int
    iy: int
    dx_mm: float
    dy_mm: float
    radius_mm: float


# ---------------------------------------------------------------------------
# Input coercion. Every public entry point takes "a scalar or a pair" for the
# quantities that are usually isotropic but need not be.
# ---------------------------------------------------------------------------

def _as_pair(value: Any, name: str) -> tuple[float, float]:
    """Accept a scalar (meaning both components) or a 2-sequence."""
    try:
        scalar = float(value)
    except (TypeError, ValueError):
        pass
    else:
        return (scalar, scalar)
    try:
        first, second = value
    except (TypeError, ValueError) as exc:
        raise SearchError("%s must be a number or a 2-sequence, got %r" % (name, value)) from exc
    try:
        return (float(first), float(second))
    except (TypeError, ValueError) as exc:
        raise SearchError("%s components must be numbers, got %r" % (name, value)) from exc


def _positive_pair(value: Any, name: str) -> tuple[float, float]:
    pair = _as_pair(value, name)
    for component in pair:
        if not math.isfinite(component) or component <= 0.0:
            raise SearchError("%s must be finite and positive, got %r" % (name, pair))
    return pair


def _finite_pair(value: Any, name: str) -> tuple[float, float]:
    pair = _as_pair(value, name)
    for component in pair:
        if not math.isfinite(component):
            raise SearchError("%s must be finite, got %r" % (name, pair))
    return pair


# ---------------------------------------------------------------------------
# The plan
# ---------------------------------------------------------------------------

def _rings_needed(max_offset_mm: float, step_mm: float) -> int:
    """Grid half-extent that reaches at least ``max_offset_mm`` along one axis.

    The epsilon stops an exactly-representable ratio such as 2.0000000000000004
    from buying a whole extra ring, which on the long axis is eight more poses.
    """
    if max_offset_mm <= 0.0:
        return 0
    return int(math.ceil(max_offset_mm / step_mm - _LATTICE_EPS_MM))


def _ring_indices(ring: int) -> list[tuple[int, int]]:
    """Grid indices on the perimeter of ``ring``, walked once around it.

    Consecutive entries are lattice neighbours, which is the property that keeps
    each arm move one step long. The walk is right edge upward, then top edge
    leftward, then left edge downward, then bottom edge rightward -- 8*ring
    points for ring >= 1, and the single origin for ring 0.
    """
    if ring == 0:
        return [(0, 0)]
    out = [(ring, j) for j in range(-ring, ring + 1)]
    out += [(i, ring) for i in range(ring - 1, -ring - 1, -1)]
    out += [(-ring, j) for j in range(ring - 1, -ring - 1, -1)]
    out += [(i, -ring) for i in range(-ring + 1, ring)]
    return out


def scan_plan(*, fov_mm: Any, pitch_mm: float = DEFAULT_PITCH_MM, max_offset_mm: float,
              overlap: float = 0.15) -> list[ScanPoint]:
    """Outward ring-spiral scan over the region the dot could be in.

    Args:
        fov_mm: field of view in ARM axes, mm. A scalar means a square field; a
            pair is (along arm X, along arm Y). If the camera or plate is
            rotated relative to the arm axes, pass ``arm_aligned_fov_mm(...)``
            instead of the raw sensor footprint -- the coverage guarantee is
            stated for an arm-aligned rectangle and is not true of a rotated one.
        pitch_mm: well pitch, mm. Used only as the sanity ceiling on
            ``max_offset_mm`` (below); the search itself never needs it.
        max_offset_mm: how far the dot could be from the nominal start, mm. The
            plan's footprint reaches at least this far along each axis
            independently, so the covered region contains the square
            [-max_offset_mm, +max_offset_mm]^2, not merely the inscribed disc.
        overlap: fraction of the field that consecutive frames share, in [0, 1).
            This is the whole coverage budget: a dot of diameter up to
            ``overlap * min(fov_mm)`` cannot be missed, and nothing larger is
            guaranteed. Set it from the dot size, do not inherit the default
            blindly.

    Returns:
        Points ordered by ring, outward from ``(0.0, 0.0)``, which is always the
        first point. Offsets are relative to the nominal start pose, in arm mm.

    Raises:
        SearchError: on a malformed field, an overlap outside [0, 1), a
            non-finite or negative ``max_offset_mm``, or a ``max_offset_mm``
            larger than ``pitch_mm``. That last one is a ceiling, not a law: a
            seating error past a full pitch means the plate is not in the nest
            it is thought to be in, and a wider search only spends minutes
            confirming a mechanical fault. ``pitch_mm`` is a named parameter
            precisely so raising the ceiling stays the caller's decision.
    """
    fov_x, fov_y = _positive_pair(fov_mm, "fov_mm")
    if not math.isfinite(overlap) or not (0.0 <= overlap < 1.0):
        raise SearchError("overlap must be in [0, 1), got %r" % (overlap,))
    if not math.isfinite(max_offset_mm) or max_offset_mm < 0.0:
        raise SearchError("max_offset_mm must be finite and >= 0, got %r" % (max_offset_mm,))
    if not math.isfinite(pitch_mm) or pitch_mm <= 0.0:
        raise SearchError("pitch_mm must be finite and positive, got %r" % (pitch_mm,))
    if max_offset_mm > pitch_mm:
        raise SearchError(
            "max_offset_mm=%.3f exceeds one well pitch (%.3f mm). A seating error that large "
            "means the plate is not in the nest, which a search cannot fix. Raise pitch_mm "
            "explicitly if a wider sweep is really wanted." % (max_offset_mm, pitch_mm))

    step_x = fov_x * (1.0 - overlap)
    step_y = fov_y * (1.0 - overlap)
    n_x = _rings_needed(max_offset_mm, step_x)
    n_y = _rings_needed(max_offset_mm, step_y)

    plan: list[ScanPoint] = []
    for ring in range(max(n_x, n_y) + 1):
        for ix, iy in _ring_indices(ring):
            # Rings past the shorter half-extent are partial: the sides that
            # left the search box are dropped, the rest of the ring is kept.
            if abs(ix) > n_x or abs(iy) > n_y:
                continue
            dx_mm = ix * step_x
            dy_mm = iy * step_y
            plan.append(ScanPoint(index=len(plan), ring=ring, ix=ix, iy=iy,
                                  dx_mm=dx_mm, dy_mm=dy_mm,
                                  radius_mm=math.hypot(dx_mm, dy_mm)))
    return plan


def coverage_gap_mm(plan: list[ScanPoint], fov_mm: Any) -> float:
    """Largest uncovered gap the plan leaves between adjacent frames, mm.

    Negative means the frames overlap by that much and the union of them is a
    solid rectangle; zero means they abut exactly; positive is the width of a
    blind stripe the plan would step over, which is the failure mode this module
    exists to prevent.

    Pass the raw field to check that dot CENTRES are covered. Pass
    ``fov - dot_diameter`` to check the stronger claim that a dot of that
    diameter is imaged whole at some point -- the returned number is
    ``spacing - fov`` per axis, so the substitution is exact (module docstring,
    equation (1)).

    This measures gaps INSIDE the plan's own footprint. Whether that footprint
    is large enough for the caller's uncertainty is a different question,
    answerable from the offsets the points carry, and it belongs to whoever owns
    ``max_offset_mm``.

    Raises:
        SearchError: if the plan is empty, or if it is not a complete
            rectangular lattice. The per-axis projection used here is only valid
            for a product grid; on a plan with holes it would report a gap of
            zero while a hole sat in the middle of it, which is exactly the
            reassuring-but-wrong answer this function exists to avoid.
    """
    if not plan:
        raise SearchError("cannot measure coverage of an empty plan")
    fov_x, fov_y = _positive_pair(fov_mm, "fov_mm")

    # Compared exactly, not to a tolerance: every offset in a generated plan is
    # ``index * step``, so two points in the same column carry the identical
    # float and a rounding pass would only throw away the last few digits of the
    # spacing the gap is measured from.
    xs = sorted({point.dx_mm for point in plan})
    ys = sorted({point.dy_mm for point in plan})
    if len(xs) * len(ys) != len(plan):
        raise SearchError(
            "plan is not a complete rectangular lattice: %d points span a %d x %d grid, so "
            "%d nodes are missing and the per-axis coverage projection does not apply"
            % (len(plan), len(xs), len(ys), len(xs) * len(ys) - len(plan)))

    def _axis_gap(values: list[float], fov: float) -> float:
        # A single column or row is one unbroken strip: no interior gap at all,
        # and the most useful thing to report is how wide that strip is.
        if len(values) < 2:
            return -fov
        return max(values[i + 1] - values[i] for i in range(len(values) - 1)) - fov

    return max(_axis_gap(xs, fov_x), _axis_gap(ys, fov_y))


def arm_aligned_fov_mm(fov_mm: Any, rotation_deg: float) -> tuple[float, float]:
    """The arm-aligned field guaranteed to sit inside a field rotated by ``rotation_deg``.

    The coverage guarantee is stated for a rectangle aligned with the arm axes,
    but the camera's rectangle is aligned with the CAMERA axes, and the plate
    does not seat at a repeatable angle. Planning on the raw sensor footprint
    when the two frames differ by theta quietly overstates coverage near the
    corners, so a caller that knows theta should shrink the field first.

    Derivation. An arm-aligned rectangle a x b, concentric with a camera
    rectangle W x H rotated by theta, sits inside it exactly when both

        a*cos + b*sin <= W        and        a*sin + b*cos <= H

    (the four corners, rotated back into the camera frame, with cos and sin
    taken of |theta|). Holding the aspect ratio fixed at a = kW, b = kH turns
    that into a closed form:

        k = min( W / (W*cos + H*sin),  H / (W*sin + H*cos) )

    which is exact at theta = 0 (k = 1) and conservative elsewhere, because a
    rectangle free to change aspect could do slightly better. Conservative is
    the correct direction for a bound the search will not re-check.

    At the measured field, 1 deg of rotation costs 2.0 % of each side and 5 deg
    costs 9.1 % -- small, but the corners are where a dot goes missing.
    """
    fov_x, fov_y = _positive_pair(fov_mm, "fov_mm")
    if not math.isfinite(rotation_deg):
        raise SearchError("rotation_deg must be finite, got %r" % (rotation_deg,))
    cos_t = abs(math.cos(math.radians(rotation_deg)))
    sin_t = abs(math.sin(math.radians(rotation_deg)))
    k = min(fov_x / (fov_x * cos_t + fov_y * sin_t),
            fov_y / (fov_x * sin_t + fov_y * cos_t))
    return (fov_x * k, fov_y * k)


# ---------------------------------------------------------------------------
# From a sighting to a move
# ---------------------------------------------------------------------------

def _checked_axis_map(axis_map: dict[str, Any] | None) -> dict[str, Any]:
    """Validate the grid orientation, because both errors here are 9 mm errors.

    Two axes that resolve to the same letter would stack a row step and a column
    step onto one arm axis and leave the other untouched; a sign that is not
    exactly +/-1 would scale the step by an arbitrary factor. Neither is
    recoverable downstream, and neither is visible in the returned numbers.
    """
    checked = dict(DEFAULT_AXIS_MAP if axis_map is None else axis_map)
    for key in ("col_axis", "col_sign", "row_axis", "row_sign"):
        if key not in checked:
            raise SearchError("axis_map is missing %r: %r" % (key, checked))
    for key in ("col_axis", "row_axis"):
        if checked[key] not in ("x", "y"):
            raise SearchError("axis_map[%r] must be 'x' or 'y', got %r" % (key, checked[key]))
    if checked["col_axis"] == checked["row_axis"]:
        raise SearchError(
            "axis_map puts rows and columns on the same arm axis (%r); one axis of the plate "
            "grid would be unreachable" % (checked["col_axis"],))
    for key in ("col_sign", "row_sign"):
        if checked[key] not in (1, -1, 1.0, -1.0):
            raise SearchError("axis_map[%r] must be +1 or -1, got %r" % (key, checked[key]))
    return checked


def resolve_from_sighting(colour: str, offset_px: Any, px_per_mm: Any, *,
                          pitch_mm: float = DEFAULT_PITCH_MM,
                          axis_map: dict[str, Any] | None = None) -> dict[str, Any]:
    """Arm move that brings the BLACK dot to frame centre, given any dot in view.

    Args:
        colour: which dot was seen -- "black", "red" or "blue", case-insensitive.
            The colour is what identifies the well, so this argument, not the
            pixel position, is what fixes the plate.
        offset_px: (u, v) of the dot minus the frame centre, u rightward and v
            downward, as returned by ``imaging.offset_from_center`` or
            ``marker.best_offset_px``.
        px_per_mm: pixel scale at the height the frame was taken. A scalar or
            (along arm X, along arm Y). It must be the scale AT THAT HEIGHT --
            magnification changes with Z, so a value carried over from another
            plane scales the centring move by the ratio of the two.
        pitch_mm: well pitch, mm.
        axis_map: grid orientation, defaulting to ``DEFAULT_AXIS_MAP``. Pass the
            one loaded from ``plate_config.json``; the calibrated file is the
            authority and this module's copy is only an offline fallback.

    Returns:
        A dict of measurements, decisions left to the caller:

        ``move_mm``          (dx, dy) arm move, mm: centring plus well step.
        ``centering_mm``     the centring half alone.
        ``well_step_mm``     the whole-well half alone. Separated so a rig with
                             a rotated camera can discard ``centering_mm``,
                             compute it with ``PixelToArm.apply`` instead, and
                             still use this step.
        ``well``             the well inferred to be under the camera.
        ``row_index``        0-based row of that well, from A1.
        ``col_index``        0-based column of that well, from A1.
        ``step_residual_mm_per_deg``  how much of the step survives per degree
                             of plate rotation, ``pitch * sin(1 deg)``. Zero for
                             black, which takes no step.
        ``px_per_mm``, ``pitch_mm``, ``axis_map``, ``image_axes_to_arm``  the
                             inputs actually used, echoed so a logged result is
                             re-derivable.
        ``assumptions``      the list of things that must hold for ``move_mm``
                             to land where it says.

    Raises:
        SearchError: on an unknown colour, a malformed axis_map, a non-finite
            offset, or a non-positive scale or pitch.
    """
    key = str(colour).strip().lower()
    if key not in DOT_WELLS:
        raise SearchError("unknown dot colour %r; the plate carries %s"
                          % (colour, ", ".join(sorted(DOT_WELLS))))
    if not math.isfinite(pitch_mm) or pitch_mm <= 0.0:
        raise SearchError("pitch_mm must be finite and positive, got %r" % (pitch_mm,))
    offset_u, offset_v = _finite_pair(offset_px, "offset_px")
    scale_x, scale_y = _positive_pair(px_per_mm, "px_per_mm")
    grid = _checked_axis_map(axis_map)

    # (a) Centring: move the arm so the imaged content shifts by -offset. The
    # sign flips of IMAGE_AXES_TO_ARM are their own inverse, so multiplying by
    # them is the same as dividing.
    sign_u, sign_v = IMAGE_AXES_TO_ARM
    centering = {"x": -offset_u * sign_u / scale_x, "y": -offset_v * sign_v / scale_y}

    # (b) Well step: undo the row and column indices of the well we are over.
    row_index, col_index = DOT_WELLS[key]
    step = {"x": 0.0, "y": 0.0}
    step[grid["row_axis"]] += -float(grid["row_sign"]) * pitch_mm * row_index
    step[grid["col_axis"]] += -float(grid["col_sign"]) * pitch_mm * col_index

    steps_taken = row_index + col_index
    residual_per_deg = pitch_mm * math.sin(math.radians(1.0)) * steps_taken

    assumptions = [
        "offset_px is (dot - frame centre), u rightward and v downward",
        "arm +X drives image u by %+d and arm +Y drives image v by %+d "
        "(IMAGE_AXES_TO_ARM, unverified on this rig)" % (sign_u, sign_v),
        "px_per_mm=(%.1f, %.1f) is the scale at the height this frame was taken; "
        "magnification changes with Z" % (scale_x, scale_y),
        "the %s dot sits on well %s, and no other dot on the plate is that colour"
        % (key, _well_name(row_index, col_index)),
    ]
    if steps_taken:
        assumptions.append(
            "the plate grid is parallel to the arm axes and the pitch is exactly %.3f mm; "
            "a rotation of theta leaves about %.3f mm of residual per degree, so the black "
            "dot lands in frame but not centred" % (pitch_mm, residual_per_deg))
    assumptions.append(
        "the move is open loop: nothing here checks that the black dot arrived")

    return {
        "colour": key,
        "well": _well_name(row_index, col_index),
        "row_index": row_index,
        "col_index": col_index,
        "move_mm": (centering["x"] + step["x"], centering["y"] + step["y"]),
        "centering_mm": (centering["x"], centering["y"]),
        "well_step_mm": (step["x"], step["y"]),
        "step_residual_mm_per_deg": residual_per_deg,
        "px_per_mm": (scale_x, scale_y),
        "pitch_mm": float(pitch_mm),
        "axis_map": grid,
        "image_axes_to_arm": tuple(IMAGE_AXES_TO_ARM),
        "assumptions": assumptions,
    }


def _well_name(row_index: int, col_index: int) -> str:
    """Well label from 0-based indices: (0, 0) -> 'A1', (1, 0) -> 'B1'."""
    return "%s%d" % (chr(ord("A") + row_index), col_index + 1)
