"""Focus measurement. Pure image in, numbers out -- no arm code, no decisions.

This module replaces four separate scorers that had drifted apart (the camera
layer's ``autofocus.focus_score``, ``align_a1.score_focus``,
``focus_sweep.score_frame``, and ``calibration.detect.sharpness``). It reports
sharpness and the quality flags needed to judge whether that number means
anything; **choosing** a plane, planning a scan, or deciding to re-measure is
:mod:`tools.microscope.autofocus`'s job.

Why the mask exists -- and why full-frame scoring is wrong here
==============================================================

This rig looks through a microscope objective at a plate carried underneath it.
Most of the frame is unlit vignette (frame median ~4/255), and the illumination
boundary is a hard, permanently sharp edge that does not change with focus. A
whole-frame sharpness metric is therefore dominated by two things that carry no
focus information at all.

Measured, on the saved A1 sweep of 2026-07-31 (11 frames, +0.0 to +10.0 mm;
re-checked against this implementation on 2026-08-03). The operator, looking at
the frames, independently picked +3.0 mm:

====================  ========  =============================
metric                picks     top three (rise: score)
====================  ========  =============================
laplacian, masked     **+3.0**  +3: 8.5   +6: 7.3   +5: 7.2
laplacian, full-frame   +1.0    +1: 4.1   +2: 3.9   +3: 3.6
tenengrad, masked     **+3.0**  +3: 352.4 +4: 98.0  +6: 83.5
tenengrad, full-frame   +3.0    +3: 89.0  +1: 35.9  +4: 30.0
brenner, masked       **+3.0**  +3: 11.0  +4: 3.8   +6: 3.1
brenner, full-frame     +3.0    +3: 2.9   +1: 1.3   +4: 1.2
====================  ========  =============================

Read it precisely: full-frame scoring is *outright wrong* only for
variance-of-Laplacian, which is where it was first caught. Full-frame tenengrad
and brenner happen to agree on this sweep -- but with a much weaker margin
(2.5x against 3.6x for masked tenengrad) and with +1.0 mm promoted to second
place, which is the same error showing through in a form that a single run
would not expose. Masking is what makes the ranking robust rather than lucky.

A fixed central ROI fails differently: once the marker is centred -- the goal of
the centring stage -- the central ROI *is* the marker, a featureless dark disc,
and scoring it ranks sensor noise. That failure was observed live (ROI mean
7.8/255) and is why the window grows instead of staying fixed.

So the mask is built from what actually carries focus information:

* the illuminated sample surface, found by Otsu on a heavily blurred copy;
* a dilation of it, which pulls in the dark side of the marker's boundary so the
  whole gradient across that edge is inside the mask -- the marker edge is the
  highest-contrast feature in the field and it lies exactly on the plate
  surface, which makes it the best available focus target;
* not the marker's deep interior and not the outer vignette, because neither
  sharpens with focus.

Metrics are computed on the **whole frame** and then averaged **over the mask**,
never on a cropped rectangle -- a crop boundary is itself an edge and would
contribute spurious gradient.

``masked=False`` keeps the old whole-frame behaviour available for comparison
and for frames that are uniformly lit. It is not the default, and a run that
uses it says so in its record.
"""
from __future__ import annotations

import dataclasses
from pathlib import Path
from typing import Union

import cv2
import numpy as np

FOCUS_METHODS = ("laplacian", "tenengrad", "brenner")

#: Frames whose scored region is dimmer than this carry no focus signal --
#: scoring them chases sensor noise. A dark corridor frame with the lamp off
#: still reads a few counts, so the floor sits well above the noise level.
BRIGHTNESS_MIN_MEAN = 30.0

#: A region this bright is clipped; gradients inside it are truncated.
BRIGHTNESS_MAX_MEAN = 250.0

#: Starting half-size of the growing focus window, as a fraction of the frame.
DEFAULT_WINDOW_FRAC = 0.45

#: Window fractions tried in order until enough lit pixels are inside.
WINDOW_LADDER = (0.60, 0.75, 0.90, 1.0)

#: A window must be at least this fraction lit to be scored.
MIN_LIT_FRACTION = 0.02

#: Blur kernel for the Otsu split (large: we want the illumination field, not texture).
_OTSU_BLUR = 31

#: Dilation that reaches across the marker's edge band.
_EDGE_DILATION = 25

ImageLike = Union[str, Path, np.ndarray]


class FocusError(ValueError):
    """An image cannot be scored (unreadable, malformed, or unknown method)."""


# ---------------------------------------------------------------------------
# (1) grayscale
# ---------------------------------------------------------------------------

def to_gray(img_or_path: ImageLike) -> np.ndarray:
    """A 2-D grayscale array from a path or an ndarray (BGR/BGRA/gray)."""
    if isinstance(img_or_path, (str, Path)):
        img = cv2.imread(str(img_or_path), cv2.IMREAD_UNCHANGED)
        if img is None:
            raise FocusError(f"unreadable image file: {img_or_path}")
    elif isinstance(img_or_path, np.ndarray):
        img = img_or_path
    else:
        raise FocusError(f"expected an image path or numpy ndarray, got {type(img_or_path)!r}")

    if img.size == 0:
        raise FocusError("empty image (zero-size array)")
    if img.ndim == 2:
        return img
    if img.ndim == 3:
        channels = img.shape[2]
        if channels == 1:
            return img[:, :, 0]
        if channels == 3:
            return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if channels == 4:
            return cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)
        raise FocusError(f"unsupported channel count: {channels}")
    raise FocusError(f"unsupported image ndim: {img.ndim}")


#: Kept as the historical private name used by older callers.
_to_gray = to_gray


# ---------------------------------------------------------------------------
# (2) the mask
# ---------------------------------------------------------------------------

@dataclasses.dataclass
class FocusMask:
    """Where in the frame focus is measurable, and how that was determined."""

    mask: np.ndarray | None
    window_frac: float | None
    otsu: float
    n_px: int
    n_frac: float

    @property
    def ok(self) -> bool:
        return self.mask is not None

    def as_dict(self) -> dict:
        return {"window_frac": self.window_frac, "otsu": round(self.otsu, 1),
                "mask_px": self.n_px, "mask_frac": self.n_frac}


def focus_mask(gray: np.ndarray, min_frac: float = DEFAULT_WINDOW_FRAC) -> FocusMask:
    """Lit sample surface plus the marker's edge band, inside a growing window.

    The window starts central and grows until it holds enough lit pixels, so a
    centred marker widens the search instead of starving it. The window actually
    used is reported and never silently substituted.
    """
    if gray.ndim != 2:
        raise FocusError(f"focus_mask needs a 2-D grayscale image, got ndim={gray.ndim}")
    height, width = gray.shape
    blur = cv2.GaussianBlur(gray, (_OTSU_BLUR, _OTSU_BLUR), 0)
    threshold, _ = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    lit = (blur > threshold).astype(np.uint8)
    lit_dilated = cv2.dilate(lit, np.ones((_EDGE_DILATION, _EDGE_DILATION), np.uint8))

    for frac in (min_frac, *WINDOW_LADDER):
        window = np.zeros_like(lit_dilated)
        half_h, half_w = int(height * frac / 2), int(width * frac / 2)
        window[height // 2 - half_h:height // 2 + half_h,
               width // 2 - half_w:width // 2 + half_w] = 1
        mask = (lit_dilated & window).astype(bool)
        count = int(mask.sum())
        if count >= MIN_LIT_FRACTION * (4 * half_h * half_w):
            return FocusMask(mask, frac, float(threshold), count,
                             round(count / float(height * width), 4))
    return FocusMask(None, None, float(threshold), 0, 0.0)


# ---------------------------------------------------------------------------
# (3) metrics
# ---------------------------------------------------------------------------

def focus_score(img_or_path: ImageLike, method: str = "laplacian",
                mask: np.ndarray | None = None) -> float:
    """Sharpness; HIGHER = SHARPER.

    With ``mask``, the metric is computed over the whole frame and then reduced
    over the masked pixels only -- so an irregular region can be scored without
    its own boundary contributing gradient.
    """
    if method not in FOCUS_METHODS:
        raise FocusError(f"unknown focus method {method!r} (valid: {FOCUS_METHODS})")
    gray = to_gray(img_or_path)
    if mask is not None and mask.shape != gray.shape:
        raise FocusError(f"mask shape {mask.shape} does not match image {gray.shape}")

    if method == "laplacian":
        field = cv2.Laplacian(gray, cv2.CV_64F)
        return float(field[mask].var() if mask is not None else field.var())

    values = gray.astype(np.float64)
    if method == "tenengrad":
        gx = cv2.Sobel(values, cv2.CV_64F, 1, 0, ksize=3)
        gy = cv2.Sobel(values, cv2.CV_64F, 0, 1, ksize=3)
        field = gx * gx + gy * gy
    else:  # brenner: squared difference between pixels two rows apart
        field = np.zeros_like(values)
        field[:-2, :] = values[2:, :] - values[:-2, :]
        field = field * field
    return float(field[mask].mean() if mask is not None else field.mean())


def brightness_guard(gray: ImageLike,
                     min_mean: float = BRIGHTNESS_MIN_MEAN) -> tuple[bool, float]:
    """``(bright_enough, mean)`` for a frame or region. Reports; decides nothing."""
    array = to_gray(gray) if not isinstance(gray, np.ndarray) or gray.ndim != 2 else gray
    mean = float(np.mean(array))
    return (mean >= min_mean, mean)


# ---------------------------------------------------------------------------
# (4) the scored sample
# ---------------------------------------------------------------------------

@dataclasses.dataclass
class FocusSample:
    """One frame's focus measurement, with the flags needed to trust it.

    ``usable`` is a *measurement-quality* fact, not an instruction. A procedure
    decides what to do about an unusable sample; this layer only says that the
    number would not mean anything.

    Two numbers, because they answer different questions:

    ``score``             raw sharpness over the scored region.
    ``score_normalised``  the same divided by mean squared. Sharpness metrics
                          scale with contrast and contrast scales with
                          illumination, so this is the one to compare across
                          samples whose lighting differed. Within one sweep at
                          fixed illumination the two rank identically -- checked
                          against the saved A1 sweep, where both pick +3.0 mm.

    Ranking on one or the other is a choice, so it belongs to the procedure:
    :func:`tools.microscope.autofocus.best_sample` takes a ``key``.

    (Field naming note: the retired ``focus_sweep.score_frame`` used ``score``
    for the *normalised* value and ``score_raw`` for the raw one. The names here
    say which is which, so old manifests and new ones must not be compared by
    key alone.)
    """

    path: Path | None
    method: str
    masked: bool
    score: float | None = None
    score_normalised: float | None = None
    mean: float | None = None
    std: float | None = None
    usable: bool = False
    reason: str = ""
    window_frac: float | None = None
    mask_frac: float = 0.0
    otsu: float | None = None

    def as_dict(self) -> dict:
        record = dataclasses.asdict(self)
        record["path"] = str(self.path) if self.path is not None else None
        return record


def score_frame(path: Path, *, method: str = "tenengrad",
                min_mean: float = BRIGHTNESS_MIN_MEAN,
                masked: bool = True,
                window_frac: float = DEFAULT_WINDOW_FRAC) -> FocusSample:
    """Score one saved frame. The measurement entry point procedures call."""
    if method not in FOCUS_METHODS:
        raise FocusError(f"unknown focus method {method!r} (valid: {FOCUS_METHODS})")
    gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if gray is None:
        raise FocusError(f"unreadable image: {path}")

    sample = FocusSample(path=Path(path), method=method, masked=masked)

    if masked:
        found = focus_mask(gray, window_frac)
        sample.window_frac = found.window_frac
        sample.mask_frac = found.n_frac
        sample.otsu = round(found.otsu, 1)
        if not found.ok:
            sample.reason = (f"no lit sample anywhere in the frame (Otsu {found.otsu:.0f}); "
                             f"is the lamp on and the plate under the objective?")
            return sample
        region = gray[found.mask]
        mask_array: np.ndarray | None = found.mask
    else:
        region = gray.reshape(-1)
        mask_array = None
        sample.mask_frac = 1.0

    values = region.astype(np.float64)
    mean = float(values.mean())
    sample.mean = round(mean, 2)
    sample.std = round(float(values.std()), 2)

    if mean < min_mean:
        sample.reason = (f"scored region too dark (mean {mean:.1f} < {min_mean:.1f}); "
                         f"scoring it would rank sensor noise")
        return sample
    if mean >= BRIGHTNESS_MAX_MEAN:
        sample.reason = f"scored region saturated (mean {mean:.1f})"
        return sample

    raw = focus_score(gray, method, mask_array)
    sample.score = round(raw, 4)
    # Contrast, and therefore every sharpness metric, scales with illumination.
    sample.score_normalised = round(raw / (mean * mean), 8) if mean > 0 else None
    sample.usable = True
    return sample


__all__ = [
    "BRIGHTNESS_MAX_MEAN",
    "BRIGHTNESS_MIN_MEAN",
    "DEFAULT_WINDOW_FRAC",
    "FOCUS_METHODS",
    "FocusError",
    "FocusMask",
    "FocusSample",
    "ImageLike",
    "brightness_guard",
    "focus_mask",
    "focus_score",
    "score_frame",
    "to_gray",
]
