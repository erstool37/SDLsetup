"""Shared ports between the guarded Arm/Microscope and tools.calib.centering.

WHY THIS MODULE EXISTS
----------------------
``centering.center_on_dot`` talks to two Protocols -- ``ArmPort`` (get_pose /
move_cart) and ``CameraPort`` (frame) -- plus a transform with ``.apply``.
Implementations of all three were written twice, in ``plate_frame_scan.py`` and
``column_calibration.py``, and a third was about to be written for the wrapped
calibration entry point. Two implementations of one measurement is the failure
this repo has already had (two Jacobian solvers, one with a 1000x looser
singularity threshold). They live here once.

REPORTS ONLY. Nothing here decides anything: the camera returns pixels, the arm
executes a validated move, the transform converts px to mm. Retry, accept and
reject stay with the phase script.
"""
from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image, ImageFilter

from ..arm import Arm
from . import pixel_scale as ps

#: Blur radius, px, used to estimate the illumination field.
#:
#: Measured 2026-08-11: only ~32% of the raw frame is above the dark threshold.
#: The illumination is a bright central disc with dark corners, so the dot and
#: the unlit surround form ONE connected component and no threshold separates
#: them -- ``find_dark_dot`` returned either a 6284 px speck or the whole
#: 4.34e6 px surround, and three ROI sizes disagreed by over 1600 px.
#:
#: Dividing by a heavily blurred copy removes the profile and leaves the dot
#: dark relative to its LOCAL surround. After it, full-frame and 60%-ROI
#: detection agree: a 1.40 mm disc at ~(-100, +50) px.
#:
#: Must stay well ABOVE the dot radius (~600 px) or the dot is blurred into its
#: own background and erased.
FLAT_FIELD_BLUR_PX = 180

FRAME_TIMEOUT_S = 30.0


def flat_field(img: Image.Image, blur_px: int = FLAT_FIELD_BLUR_PX) -> np.ndarray:
    """Illumination-corrected greyscale array. See FLAT_FIELD_BLUR_PX."""
    raw = np.asarray(img.convert("L"), dtype=float)
    bg = np.asarray(img.convert("L").filter(ImageFilter.GaussianBlur(radius=blur_px)),
                    dtype=float)
    return np.clip(raw / np.maximum(bg, 1.0) * 128.0, 0, 255).astype(np.uint8)


class FlatFieldCamera:
    """CameraPort: newest published frame, illumination-corrected, saved to disk."""

    def __init__(self, scope, out_dir: Path, ordinal: int = 2, tag: str = "f",
                 blur_px: int = FLAT_FIELD_BLUR_PX):
        self.scope = scope
        self.out_dir = Path(out_dir)
        self.ordinal = ordinal
        self.tag = tag
        self.blur_px = blur_px
        self.n = 0
        self.last_path: Path | None = None

    def frame(self) -> np.ndarray:
        self.n += 1
        dest = self.out_dir / f"{self.tag}_{self.n:03d}.jpg"
        got = self.scope.grab_frame(dest, ordinal=self.ordinal, timeout_s=FRAME_TIMEOUT_S)
        if not got.ok:
            raise RuntimeError(f"frame grab failed: {got.reason}")
        self.last_path = dest
        return flat_field(Image.open(dest), self.blur_px)


class PinnedArm:
    """ArmPort with Z and orientation PINNED to a taught anchor.

    Centring is an XY operation. Echoing back a MEASURED Z re-commands settling
    noise: the arm reads ~0.7 um above the taught height, and against a
    zero-budget Z window that aborts the run outright (observed 2026-08-11,
    'z 191.2095 outside [191.2088, 191.2088]').

    Pinning is the stronger fix than widening the window -- Z genuinely must not
    move here, because raising it drives the plate toward a fixed objective.
    """

    def __init__(self, arm: Arm, anchor):
        self.arm = arm
        self.anchor = tuple(float(v) for v in anchor)

    def get_pose(self) -> list:
        return list(self.arm.pose())

    def move_cart(self, x, y, z, roll, pitch, yaw, speed) -> None:
        a = self.anchor
        self.arm.move_pose((x, y, a[2], a[3], a[4], a[5]), speed=speed,
                           label="centre", takeup=False)


class PinnedXYArm:
    """ArmPort for a Z-only operation: X, Y and orientation pinned to the anchor.

    The mirror of :class:`PinnedArm`, for autofocus. A focus sweep must move Z
    and nothing else; pinning XY here means a lateral drift cannot accumulate
    across the sweep and cannot be commanded back in from a reading.
    """

    def __init__(self, arm: Arm, anchor):
        self.arm = arm
        self.anchor = tuple(float(v) for v in anchor)

    def get_pose(self) -> list:
        return list(self.arm.pose())

    def move_cart(self, x, y, z, roll, pitch, yaw, speed) -> None:
        a = self.anchor
        self.arm.move_pose((a[0], a[1], z, a[3], a[4], a[5]), speed=speed,
                           label="focus", takeup=True)


class JacobianTransform:
    """transform.apply(offset_px) -> (dx_mm, dy_mm) from a measured Jacobian."""

    def __init__(self, J):
        self.J = J

    def apply(self, offset_px):
        return ps.correction_mm(self.J, offset_px)

    def describe(self) -> dict[str, Any]:
        return ps.decompose(self.J)


def axis_sequential_to(arm: Arm, anchor, x: float, y: float, *, speed: float,
                       label: str, tol_mm: float = 0.05) -> list:
    """Move to (x, y) one axis at a time with Z/orientation pinned to `anchor`.

    ``Arm.move_axiswise`` builds each leg from the arm's MEASURED pose, which is
    right for a transit but wrong under a zero-budget Z envelope: it carries a
    Z of 191.2089 into a window of exactly [191.2088, 191.2088]. Here the
    commanded Z is always the taught one.

    Returns the MoveResult of each leg actually commanded.
    """
    a = tuple(float(v) for v in anchor)
    here = list(arm.pose())
    out = []
    for leg_x, leg_y, name in ((x, here[1], "x"), (x, y, "y")):
        if abs(leg_x - here[0]) < tol_mm and abs(leg_y - here[1]) < tol_mm:
            continue
        res = arm.move_pose((leg_x, leg_y, a[2], a[3], a[4], a[5]),
                            speed=speed, label=f"{label} [{name}]", takeup=False)
        arrival = res.verify()
        if arrival["moved"] is False:
            raise RuntimeError(f"{label} leg {name!r} did not arrive: {arrival['reason']}")
        out.append(res)
        here = list(arm.pose())
    return out


def lit_centroid_offset_px(flat: np.ndarray, threshold: float = 96.0):
    """Offset of the ILLUMINATED region's centroid from the frame centre, px.

    A coarse centring check for wells that carry NO dot. It measures where the
    lit aperture sits, which is not the same thing as the well wall -- treat it
    as a sanity flag, never as a position measurement. Returns None when the lit
    region is too small or fills the frame, both of which make the centroid
    meaningless.
    """
    h, w = flat.shape
    lit = flat >= threshold
    frac = float(lit.mean())
    if frac < 0.02 or frac > 0.98:
        return None, frac
    ys, xs = np.nonzero(lit)
    cx, cy = float(xs.mean()), float(ys.mean())
    return (cx - (w - 1) / 2.0, cy - (h - 1) / 2.0), frac


def offset_mm(transform: JacobianTransform, offset_px) -> tuple[float, float]:
    dx, dy = transform.apply(offset_px)
    return float(dx), float(dy)


def hypot_mm(offset) -> float:
    return math.hypot(float(offset[0]), float(offset[1]))
