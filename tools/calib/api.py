"""tools.calib public surface -- the only names a script outside this package
should import.

    from tools.calib import api as calib

    cam   = calib.FlatFieldCamera(scope, out_dir, tag="A1")
    arm   = calib.PinnedArm(guarded, anchor)
    J     = calib.jacobian_at(calib.load_pixel_scale(path), z)
    res   = calib.center_on_dot(arm, cam, calib.JacobianTransform(J))
    x, y  = calib.measured_xy(cal, row, col)

THE CONVENTION THIS FILE EXISTS FOR
-----------------------------------
Every device and support package exposes ONE module naming what may be called
from outside it (``tools/arm/api.py``, ``tools/microscope/api.py``, and this).
Anything not re-exported here is package-internal, and package-internal helpers
carry a leading underscore.

Why the underscore is not enough on its own: a name without one is only
*conventionally* public, and this tree has ~111 module-level names with no
in-tree caller -- a mix of genuine API, argparse-dispatched command handlers,
Protocols, and returned dataclasses. Reading that list, there is no way to tell
which are meant for outside use. An explicit re-export list is the difference
between "nobody happens to call this yet" and "this is the supported surface".

WHAT IS DELIBERATELY NOT HERE. The measurement internals -- the Otsu split, the
connected-component labeller, the circle fit, the phase-correlation kernel.
Reaching past this module for those means either the surface is missing
something (add it here, deliberately) or the caller is reimplementing a
measurement that already exists, which is the failure this package has already
had twice: two Jacobian solvers, one with a 1000x looser singularity threshold.

NAME CLASHES ARE RESOLVED HERE, NOT AT THE CALL SITE. Both ``pixel_scale`` and
``arm.geometry`` define ``load``/``save``. They are re-exported under explicit
names so a reader never has to work out which store a bare ``load`` meant.
"""
from __future__ import annotations

# -- ports and adapters: how a guarded Arm/Microscope reaches this package ----
from .adapters import (
    FLAT_FIELD_BLUR_PX,
    FlatFieldCamera,
    JacobianTransform,
    PinnedArm,
    PinnedXYArm,
    axis_sequential_to,
    flat_field,
    hypot_mm,
    lit_centroid_offset_px,
    offset_mm,
)

# -- the centring / focus routines -------------------------------------------
from .centering import (
    ArmPort,
    CameraPort,
    CenteringResult,
    FocusResult,
    SearchResult,
    autofocus_z,
    calibrate_plate,
    center_on_dot,
    spiral_search,
)

# -- where a well is ----------------------------------------------------------
from .grid import (
    WellNameError,
    best_xy,
    measured_xy,
    nominal_xy,
    parse_well,
    well_name,
)

# -- the pixel <-> arm transform ----------------------------------------------
from .pixel_scale import (
    ScaleError,
    correction_mm,
    decompose,
    fit_scale_model,
    invert,
    jacobian_at,
    jacobian_from_moves,
)
from .pixel_scale import load as load_pixel_scale
from .pixel_scale import save as save_pixel_scale

# -- the plate's own row/column frame ------------------------------------------
from .plate_frame import (
    PlateFrame,
    PlateFrameError,
    solve_plate_frame,
    well_offset,
    yaw_correction_deg,
)

__all__ = [
    "ARM_PORTS",
    "FLAT_FIELD_BLUR_PX",
    "ArmPort",
    "CameraPort",
    "CenteringResult",
    "FlatFieldCamera",
    "FocusResult",
    "JacobianTransform",
    "PinnedArm",
    "PinnedXYArm",
    "PlateFrame",
    "PlateFrameError",
    "ScaleError",
    "SearchResult",
    "WellNameError",
    "autofocus_z",
    "axis_sequential_to",
    "best_xy",
    "calibrate_plate",
    "center_on_dot",
    "correction_mm",
    "decompose",
    "fit_scale_model",
    "flat_field",
    "hypot_mm",
    "invert",
    "jacobian_at",
    "jacobian_from_moves",
    "lit_centroid_offset_px",
    "load_pixel_scale",
    "measured_xy",
    "nominal_xy",
    "offset_mm",
    "parse_well",
    "save_pixel_scale",
    "solve_plate_frame",
    "spiral_search",
    "well_name",
    "well_offset",
    "yaw_correction_deg",
]

#: The two Protocols a caller must satisfy to drive anything in this package.
#: Named together because implementing one without the other is never useful.
ARM_PORTS = (ArmPort, CameraPort)
