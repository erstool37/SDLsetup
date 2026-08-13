"""Leica K3C + TIS DFK33UX264 -- everything needed to operate the microscope.

    from tools import microscope

    scope = microscope.Microscope.from_config("config.yaml")
    frame = scope.grab_frame(Path("shot.jpg"))
    sample = scope.measure_focus(frame.require())

Two cameras, one package: they are captured together, published together, and
calibrated together, so they are one lifecycle-managed aggregate. The Leica is
the one looking through the objective and is what every focus and centring
measurement uses.

Layers, lowest to highest:

``backends``   per-vendor capture programs (GenTL, TWAIN, tisgrabber) + PS1 bridges
``capture``    camera classes, the capture batch, and the capture CLI
``viewer``     the live monitor web UI (stream + record + save)
``imaging``    array-level primitives: grayscale, Otsu, components, sharpness
``marker``     locate the operator's alignment mark (dark or red), with quality flags
``focus``      sharpness measurement and the lit-region mask
``api``        Microscope -- what orchestration calls
``node``       Lab node wrapper for the dashboard

This package measures and reports. Deciding a plane is in focus, retrying a
frame, or choosing the next well belongs to :mod:`tools`.
"""
from . import batch, capture, focus, imaging, marker
from .api import (
    DEFAULT_STREAM_DIR,
    LINKED_CAMERAS,
    OBJECTIVE_CAMERA,
    STREAM_FILES,
    Frame,
    Microscope,
    MicroscopeError,
    MicroscopeSettings,
    default,
    grab_frame,
    jpeg_complete,
    latest_frame,
    measure_focus,
    status,
)
from .batch import (
    DualCameraModule,
    LeicaK3CModule,
    TisDFK33Module,
    capture_both,
    capture_leica,
    capture_tis,
)
from .focus import FOCUS_METHODS, FocusSample, focus_mask, focus_score, score_frame
from .node import CameraNode

__all__ = [
    "DEFAULT_STREAM_DIR", "FOCUS_METHODS", "Frame", "FocusSample", "LINKED_CAMERAS",
    "Microscope", "MicroscopeError", "MicroscopeSettings", "OBJECTIVE_CAMERA",
    "STREAM_FILES", "CameraNode",
    "DualCameraModule", "LeicaK3CModule", "TisDFK33Module",
    "batch", "capture", "capture_both", "capture_leica", "capture_tis",
    "default", "focus", "focus_mask", "focus_score", "grab_frame",
    "imaging", "jpeg_complete", "latest_frame", "marker", "measure_focus",
    "score_frame", "status",
]
