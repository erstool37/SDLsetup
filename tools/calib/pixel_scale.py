"""
pixel_scale.py -- how many pixels the image moves per millimetre the arm moves,
as a function of height, and in which direction.

Pure module: model, fitting, and inversion. It touches no hardware and reads no
images. The measuring driver lives in scripts/calibrate_pixel_scale.py.

===========================================================================
1. WHAT WE NEED
===========================================================================
To centre a feature we must convert "the dot is 300 px right and 270 px down"
into "move the arm -0.42 mm in X and +0.31 mm in Y". That conversion is a 2x2
matrix, and it is NOT a single number, because:

  * the camera's pixel axes are rotated with respect to the arm's X/Y axes,
  * the two axes can have slightly different scales (anisotropic pixels,
    non-square sensor binning, or a tilted optical axis),
  * and the scale CHANGES WITH HEIGHT, because raising the plate toward the
    objective increases magnification.

So the object we calibrate is a height-dependent Jacobian J(z):

      [ du ]        [ dx ]                      [ a(z)  b(z) ]
      [ dv ]  =  J(z) [ dy ]     with    J(z) = [ c(z)  d(z) ]

  du, dv : image displacement in PIXELS (u = column, v = row, v grows downward)
  dx, dy : arm displacement in MILLIMETRES, in the arm's base frame
  J(z)   : pixels per millimetre, at plate height z (mm, arm base frame)

Inverting it gives the move that centres a feature:

      [ dx ]           [ -offset_u ]
      [ dy ]  = J(z)^-1 [ -offset_v ]

where (offset_u, offset_v) is the feature's position minus the image centre.
The minus sign is the part that is easy to get backwards: to bring a feature
that sits to the RIGHT of centre INTO the centre, the image must move LEFT, so
we ask for a displacement of -offset.

===========================================================================
2. WHY THE SCALE DEPENDS ON HEIGHT, AND THE EXACT FORM IT TAKES
===========================================================================
For a thin lens of focal length f imaging an object at distance d, the
transverse magnification is

      m = f / (d - f)

The sensor samples the image at a fixed pitch p (mm per pixel), so

      s(d) = m / p = f / (p (d - f))          [pixels per millimetre]

Raising the plate by dz REDUCES the object distance one-for-one, d = d0 - (z - z0).
Substituting and collecting the constants gives the working form

      s(z) = A / (B - z)                                              (1)

  A : lumps focal length and pixel pitch, [px·mm/mm] = [px]
  B : the height at which the plate would reach the lens' front principal
      plane, where s would diverge. It is NOT a physical travel limit and must
      never be used as one -- the mechanical clearance is a separate, measured
      number. B only says where this *model* blows up.
  z : plate height in the arm's base frame, mm.

Equation (1) is a hyperbola in z, which is awkward to fit -- but its RECIPROCAL
is exactly linear:

      1/s(z) = (B - z)/A = B/A - z/A  =  p0 + p1 z                    (2)

      with   p0 = B/A,   p1 = -1/A

So we fit a straight line to (z, 1/s) by ordinary least squares, then recover

      A = -1/p1,      B = -p0/p1                                      (3)

This is the whole reason the module fits reciprocals: a linear fit has a closed
form, no starting guess, and no convergence failure. Two heights suffice; three
or more let us report a residual, which is what tells us whether the model is
right rather than merely fitted.

REGIME AND ASSUMPTIONS. Equation (1) assumes a single thin lens, a sensor
parallel to the object plane, and travel small compared with the working
distance. Over the ~10 mm this stage moves, that is a good approximation. It
fails if the optics are telecentric -- a telecentric objective is designed so
magnification does NOT change with height, which shows up here as p1 ~ 0 and a
divergent A. `fit_scale_model` detects that and returns a CONSTANT model instead
of dividing by ~0. If the dominant term is dropped -- that is, if you assume
s is constant when it is not -- a correction computed at the wrong height is
wrong by the ratio s(z_cal)/s(z_now), which over 10 mm can be tens of percent.

===========================================================================
3. DIRECTION
===========================================================================
Decompose the measured Jacobian into a scale-and-rotation picture:

      theta = atan2(J[1,0], J[0,0])        rotation of image u-axis vs arm +X
      s_x   = hypot(J[0,0], J[1,0])        px per mm of arm X travel
      s_y   = hypot(J[0,1], J[1,1])        px per mm of arm Y travel
      shear = angle between the two image-space column vectors, minus 90 deg

theta is a property of how the camera is bolted on and should be essentially
CONSTANT with height; s_x and s_y follow equation (1). A theta that drifts with
height means the optical axis is not parallel to the travel axis, and is worth
knowing rather than averaging away. `fit_scale_model` therefore fits theta
separately as a constant plus an optional linear term and reports the spread.

A NOTE ON HANDEDNESS. Image v grows DOWNWARD while arm Y grows in whatever
direction the robot's base frame says. That means det(J) is commonly NEGATIVE:
the mapping is orientation-reversing. This is normal and must not be "fixed" --
inverting J handles it. What matters is that |det(J)| is not near zero, which
would mean the two measured shifts were collinear and the mapping is not
invertible.
"""
from __future__ import annotations

import json
import math
from collections.abc import Sequence
from pathlib import Path
from typing import Any

import cv2
import numpy as np

#: Below this |det(J)| (px^2/mm^2) the two measured axes are effectively
#: collinear and the mapping must not be inverted.
MIN_ABS_DET = 1e-3

#: If the reciprocal-scale slope |p1| is below this, the optics are effectively
#: telecentric over this travel and a constant model is used instead.
TELECENTRIC_SLOPE = 1e-6


class ScaleError(ValueError):
    pass


# ---------------------------------------------------------------------------
# Frame-to-frame pixel shift, by phase correlation -- the raw measurement
# jacobian_from_moves() below is built from. Whole-frame correlation is used
# rather than any single detected feature so the measurement is unbiased by a
# feature that is soft-edged or clipped by the frame border.
# ---------------------------------------------------------------------------

def _gray_for_shift(path: Path) -> np.ndarray:
    img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise ScaleError("unreadable image: %s" % path)
    return img


def image_shift_px(ref: Path, mov: Path) -> dict[str, Any]:
    """Pixel shift (dx, dy) that maps `ref` onto `mov`, by phase correlation.

    Returns the shift and the correlation response; a low response means the
    two frames do not share enough structure and the number must not be
    trusted.
    """
    a = _gray_for_shift(ref).astype(np.float32)
    b = _gray_for_shift(mov).astype(np.float32)
    if a.shape != b.shape:
        raise ScaleError("frame size changed between captures: %s vs %s" % (a.shape, b.shape))
    win = cv2.createHanningWindow((a.shape[1], a.shape[0]), cv2.CV_32F)
    (dx, dy), response = cv2.phaseCorrelate(a, b, win)
    return {"dx_px": float(dx), "dy_px": float(dy), "response": float(response)}


# ---------------------------------------------------------------------------
# Jacobian from two measured displacements
# ---------------------------------------------------------------------------

def jacobian_from_moves(shift_for_x: Sequence[float], shift_for_y: Sequence[float],
                        step_mm: float) -> np.ndarray:
    """Build J (px per mm) from the image shifts caused by two known moves.

    Args:
        shift_for_x: (du, dv) in px observed for a +step_mm move along arm X.
        shift_for_y: (du, dv) in px observed for a +step_mm move along arm Y.
        step_mm: the size of each calibration move, mm.

    Returns:
        2x2 array J with J @ [dx_mm, dy_mm] = [du_px, dv_px].
    """
    if step_mm <= 0:
        raise ScaleError("step_mm must be positive, got %r" % (step_mm,))
    J = np.array([[shift_for_x[0], shift_for_y[0]],
                  [shift_for_x[1], shift_for_y[1]]], dtype=float) / float(step_mm)
    if abs(float(np.linalg.det(J))) < MIN_ABS_DET:
        raise ScaleError("the two measured shifts are collinear (det=%.3e px^2/mm^2); "
                         "the mapping cannot be inverted. Re-measure with a larger step, "
                         "or check that the arm actually moved on both axes."
                         % float(np.linalg.det(J)))
    return J


def decompose(J: np.ndarray) -> dict[str, float]:
    """Scale, rotation, shear and handedness of a Jacobian. Reports; decides nothing."""
    J = np.asarray(J, dtype=float)
    s_x = float(math.hypot(J[0, 0], J[1, 0]))
    s_y = float(math.hypot(J[0, 1], J[1, 1]))
    theta = float(math.degrees(math.atan2(J[1, 0], J[0, 0])))
    theta_y = float(math.degrees(math.atan2(J[1, 1], J[0, 1])))
    det = float(np.linalg.det(J))
    reversing = det < 0
    # Angle from the arm-X image direction to the arm-Y image direction, wrapped
    # to (-180, 180]. For an orthogonal map it is -90 when the mapping reverses
    # orientation (image v grows downward while arm Y grows "up") and +90 when it
    # does not. Shear is the departure from that expectation -- so it must be
    # measured against the correct sign, or a perfectly square map reads as 180
    # degrees of shear and the reconstruction comes out mirrored.
    delta = float(((theta_y - theta) + 180.0) % 360.0 - 180.0)
    shear = delta + 90.0 if reversing else delta - 90.0
    return {"px_per_mm_x": s_x, "px_per_mm_y": s_y, "px_per_mm_mean": (s_x + s_y) / 2.0,
            "rotation_deg": theta, "shear_deg": shear, "det": det,
            "orientation_reversing": reversing}


def invert(J: np.ndarray) -> np.ndarray:
    J = np.asarray(J, dtype=float)
    if abs(float(np.linalg.det(J))) < MIN_ABS_DET:
        raise ScaleError("J is singular; cannot invert")
    return np.linalg.inv(J)


def correction_mm(J: np.ndarray, offset_px: Sequence[float]) -> tuple[float, float]:
    """Arm move (dx, dy) in mm that brings a feature at `offset_px` to the centre.

    offset_px is (feature - centre) in pixels, v growing downward. The negation
    is the direction-of-travel step that is easy to get backwards: a feature to
    the right of centre needs the image to move LEFT.
    """
    d = invert(J) @ (-np.asarray(offset_px, dtype=float))
    return float(d[0]), float(d[1])


# ---------------------------------------------------------------------------
# Height model
# ---------------------------------------------------------------------------

def fit_scale_model(heights_mm: Sequence[float],
                    jacobians: Sequence[np.ndarray]) -> dict[str, Any]:
    """Fit s(z) = A/(B - z) per axis and a rotation model, from >=2 heights.

    Fitting is done on 1/s, which is linear in z -- see the module docstring,
    equation (2). With three or more heights the RMS residual is reported; that
    residual, not the fit itself, is what says whether the model holds.
    """
    z = np.asarray(heights_mm, dtype=float)
    if z.size < 2:
        raise ScaleError("need at least 2 heights to fit a height model, got %d" % z.size)
    if len(jacobians) != z.size:
        raise ScaleError("heights and jacobians differ in length")
    if np.ptp(z) < 1e-6:
        raise ScaleError("all calibration heights are the same; the model is unconstrained")

    parts = [decompose(np.asarray(J, dtype=float)) for J in jacobians]
    out: dict[str, Any] = {"heights_mm": z.tolist(), "n_points": int(z.size), "axes": {}}

    for key in ("px_per_mm_x", "px_per_mm_y"):
        s = np.array([p[key] for p in parts], dtype=float)
        if np.any(s <= 0):
            raise ScaleError("non-positive scale in %s: %s" % (key, s.tolist()))
        inv = 1.0 / s
        p1, p0 = np.polyfit(z, inv, 1)          # inv = p1*z + p0
        pred = p1 * z + p0
        resid = float(np.sqrt(np.mean((inv - pred) ** 2))) if z.size > 2 else None
        if abs(p1) < TELECENTRIC_SLOPE:
            model = {"form": "constant", "s0": float(np.mean(s)),
                     "note": ("scale does not vary measurably with height over this range "
                              "(|slope| < %.1e); treating the optics as telecentric here"
                              % TELECENTRIC_SLOPE)}
        else:
            A = -1.0 / p1
            B = -p0 / p1
            model = {"form": "A/(B - z)", "A": float(A), "B": float(B),
                     "note": ("B is where this MODEL diverges, not a mechanical limit. "
                              "Never use it as a travel ceiling.")}
        model.update({"recip_slope": float(p1), "recip_intercept": float(p0),
                      "recip_rms_residual": resid,
                      "measured": s.tolist()})
        out["axes"][key] = model

    th = np.array([p["rotation_deg"] for p in parts], dtype=float)
    th_un = np.degrees(np.unwrap(np.radians(th)))
    if z.size >= 2:
        m, c = np.polyfit(z, th_un, 1)
    else:
        m, c = 0.0, float(th_un[0])
    out["rotation"] = {"mean_deg": float(np.mean(th_un)),
                       "spread_deg": float(np.ptp(th_un)),
                       "slope_deg_per_mm": float(m), "intercept_deg": float(c),
                       "measured_deg": th.tolist(),
                       "note": ("rotation should be essentially constant with height; a "
                                "spread of more than ~1 deg suggests the optical axis is "
                                "not parallel to the travel axis")}
    out["shear_deg"] = [p["shear_deg"] for p in parts]
    out["orientation_reversing"] = [p["orientation_reversing"] for p in parts]
    return out


def _s_at(model: dict[str, Any], z: float) -> float:
    if model["form"] == "constant":
        return float(model["s0"])
    denom = model["B"] - z
    if abs(denom) < 1e-9:
        raise ScaleError("height %.4f is at the model's singularity B=%.4f; the fit does "
                         "not apply here" % (z, model["B"]))
    s = model["A"] / denom
    if s <= 0:
        raise ScaleError("model gives a non-positive scale %.4f at z=%.4f; you are on the "
                         "far side of the singularity and the fit does not apply" % (s, z))
    return float(s)


def jacobian_at(model: dict[str, Any], z: float) -> np.ndarray:
    """Reconstruct J at an arbitrary height from a fitted model.

    Rebuilds from scale + rotation + shear rather than interpolating the matrix
    entries, so the result stays a well-formed scale-rotation-shear map instead
    of drifting into something with no geometric meaning.
    """
    s_x = _s_at(model["axes"]["px_per_mm_x"], z)
    s_y = _s_at(model["axes"]["px_per_mm_y"], z)
    rot = model["rotation"]
    theta = math.radians(rot["intercept_deg"] + rot["slope_deg_per_mm"] * z)
    shear = math.radians(float(np.mean(model.get("shear_deg", [0.0]))))
    rev = model.get("orientation_reversing", [False])
    reversing = bool(rev[0]) if isinstance(rev, (list, tuple)) else bool(rev)
    # column 1: the arm +X direction in image space.
    c1 = np.array([math.cos(theta), math.sin(theta)]) * s_x
    # column 2: the arm +Y direction, a quarter turn away -- clockwise when the
    # mapping reverses orientation, anticlockwise when it does not -- plus the
    # measured shear, which is defined against that same expectation.
    quarter = -math.pi / 2.0 if reversing else math.pi / 2.0
    ang2 = theta + quarter + shear
    c2 = np.array([math.cos(ang2), math.sin(ang2)]) * s_y
    return np.column_stack([c1, c2])


def save(model: dict[str, Any], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(model, indent=2))


def load(path: Path) -> dict[str, Any]:
    return json.loads(Path(path).read_text())


# ---------------------------------------------------------------------------
# Self-test: python -m tools.robot_arm.pixel_scale
# ---------------------------------------------------------------------------

def _self_test() -> int:
    fails = 0

    def ok(c, m):
        nonlocal fails
        print("[self-test] %s: %s" % ("PASS" if c else "FAIL", m))
        if not c:
            fails += 1

    def raises(fn, m):
        nonlocal fails
        try:
            fn()
        except ScaleError as e:
            print("[self-test] PASS: %s  (%s)" % (m, str(e)[:70]))
            return
        print("[self-test] FAIL: %s -- nothing raised" % m)
        fails += 1

    # --- synthetic ground truth: A/(B-z), 12 deg rotation, orientation-reversing
    A_true, B_true, th = 900.0, 60.0, 12.0
    def J_true(z, sx_scale=1.0):
        s = A_true / (B_true - z)
        t = math.radians(th)
        c1 = np.array([math.cos(t), math.sin(t)]) * s * sx_scale
        c2 = np.array([math.cos(t - math.pi / 2), math.sin(t - math.pi / 2)]) * s
        return np.column_stack([c1, c2])

    zs = [10.0, 15.0, 20.0, 25.0]
    Js = [J_true(z) for z in zs]
    model = fit_scale_model(zs, Js)
    ax = model["axes"]["px_per_mm_x"]
    ok(ax["form"] == "A/(B - z)", "hyperbolic form selected")
    ok(abs(ax["A"] - A_true) < 1e-6 * A_true, "A recovered (%.4f vs %.1f)" % (ax["A"], A_true))
    ok(abs(ax["B"] - B_true) < 1e-6 * B_true, "B recovered (%.4f vs %.1f)" % (ax["B"], B_true))
    ok(abs(model["rotation"]["mean_deg"] - th) < 1e-6, "rotation recovered (%.4f deg)"
       % model["rotation"]["mean_deg"])
    ok(model["rotation"]["spread_deg"] < 1e-9, "rotation is constant with height")

    z_new = 17.5
    Jr = jacobian_at(model, z_new)
    err = float(np.max(np.abs(Jr - J_true(z_new))))
    ok(err < 1e-6, "J reconstructed at an unmeasured height (max err %.2e)" % err)

    # extrapolating past the singularity must fail loudly, not return nonsense
    raises(lambda: jacobian_at(model, B_true + 1.0), "refuses to evaluate past the singularity")
    raises(lambda: jacobian_at(model, B_true), "refuses to evaluate at the singularity")

    # --- inversion and sign convention
    J = np.array([[100.0, 0.0], [0.0, -100.0]])     # 100 px/mm, v flipped
    dx, dy = correction_mm(J, (300.0, 200.0))
    ok(abs(dx + 3.0) < 1e-9, "a feature 300 px RIGHT needs dx = -3.00 mm (got %+.3f)" % dx)
    ok(abs(dy - 2.0) < 1e-9, "a feature 200 px DOWN with flipped v needs dy = +2.00 mm "
                             "(got %+.3f)" % dy)

    # --- degenerate inputs
    raises(lambda: jacobian_from_moves((10, 0), (20, 0), 0.5), "collinear moves rejected")
    raises(lambda: jacobian_from_moves((10, 0), (0, 10), 0.0), "zero step rejected")
    raises(lambda: fit_scale_model([5.0], [J_true(5.0)]), "single height rejected")
    raises(lambda: fit_scale_model([5.0, 5.0], [J_true(5.0)] * 2), "identical heights rejected")

    # --- telecentric case: scale independent of height
    Jc = [J_true(10.0) for _ in range(4)]
    mc = fit_scale_model([10.0, 12.0, 14.0, 16.0], Jc)
    ok(mc["axes"]["px_per_mm_x"]["form"] == "constant", "telecentric optics -> constant model")
    ok(abs(float(np.max(np.abs(jacobian_at(mc, 99.0) - J_true(10.0))))) < 1e-6,
       "constant model extrapolates without blowing up")

    # --- residual is reported and is ~0 for exact data, large for perturbed data
    ok(ax["recip_rms_residual"] is not None and ax["recip_rms_residual"] < 1e-12,
       "residual ~0 on exact data (%.2e)" % ax["recip_rms_residual"])
    Jp = [J_true(z) for z in zs]
    Jp[2] = Jp[2] * 1.25                                  # 25 % outlier
    mp = fit_scale_model(zs, Jp)
    ok(mp["axes"]["px_per_mm_x"]["recip_rms_residual"] > 1e-5,
       "residual grows on perturbed data (%.2e)"
       % mp["axes"]["px_per_mm_x"]["recip_rms_residual"])

    # --- anisotropic axes are kept separate, not averaged
    Ja = [J_true(z, sx_scale=1.30) for z in zs]
    ma = fit_scale_model(zs, Ja)
    rx = ma["axes"]["px_per_mm_x"]["measured"][0]
    ry = ma["axes"]["px_per_mm_y"]["measured"][0]
    ok(abs(rx / ry - 1.30) < 1e-9, "anisotropy preserved (x/y = %.4f)" % (rx / ry))

    print("\n[self-test] %s" % ("ALL PASS" if fails == 0 else "%d FAILURE(S)" % fails))
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(_self_test())
