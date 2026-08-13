"""Array-level image primitives: grayscale, Otsu, components, sharpness.

The dependency-light layer (numpy + PIL, no OpenCV) underneath the microscope's
measurements. :mod:`.marker` is the path-based detector to reach for in a
procedure -- it handles the unlit vignette and reports a circle fit alongside the
centroid. These are the primitives, and the fake-testable ones: the calibration
routine suite drives them with synthetic arrays and no hardware.

Pure image analysis for dot detection and sharpness.

SENSOR LAW: this module only REPORTS measurements and quality flags. It never
decides what to do next, never retries, never discards a reading on its own
judgement. Callers (routine.py) interpret and act.

No hardware, no OpenCV/scipy — Otsu threshold, connected components (iterative
union-find over a boolean mask, no Python recursion), and a 3x3 Laplacian are
all implemented directly on numpy arrays.
"""
from __future__ import annotations

import dataclasses

import numpy as np
from PIL import Image


def load_gray(path: str) -> np.ndarray:
    """Load an image file and return float32 luminance in [0, 255]."""
    with Image.open(path) as im:
        return np.asarray(im.convert("L"), dtype=np.float32)


@dataclasses.dataclass
class DotDetection:
    found: bool
    cx_px: float | None
    cy_px: float | None
    radius_px: float | None
    area_px: int
    contrast: float
    fill_ratio: float
    n_candidates: int
    image_size: tuple
    note: str


def _otsu_threshold(img: np.ndarray) -> float:
    """Otsu's method via a 256-bin histogram. Returns the threshold value."""
    hist, edges = np.histogram(img, bins=256, range=(0.0, 255.0))
    hist = hist.astype(np.float64)
    total = hist.sum()
    if total <= 0:
        return 127.5
    bin_centers = (edges[:-1] + edges[1:]) / 2.0

    w0 = np.cumsum(hist)
    w1 = total - w0
    sum_all = np.sum(hist * bin_centers)
    sum0 = np.cumsum(hist * bin_centers)
    with np.errstate(divide="ignore", invalid="ignore"):
        mean0 = np.where(w0 > 0, sum0 / w0, 0.0)
        mean1 = np.where(w1 > 0, (sum_all - sum0) / w1, 0.0)
    between = w0 * w1 * (mean0 - mean1) ** 2
    idx = int(np.argmax(between))
    return float(bin_centers[idx])


def _label_components(mask: np.ndarray) -> tuple:
    """Iterative (stack-based, no recursion) 4-connected labeling.

    Returns (labels array int32, number of components).
    """
    h, w = mask.shape
    labels = np.zeros((h, w), dtype=np.int32)
    current_label = 0
    stack = []
    for y in range(h):
        for x in range(w):
            if mask[y, x] and labels[y, x] == 0:
                current_label += 1
                labels[y, x] = current_label
                stack.append((y, x))
                while stack:
                    cy, cx = stack.pop()
                    for ny, nx in ((cy - 1, cx), (cy + 1, cx), (cy, cx - 1), (cy, cx + 1)):
                        if 0 <= ny < h and 0 <= nx < w and mask[ny, nx] and labels[ny, nx] == 0:
                            labels[ny, nx] = current_label
                            stack.append((ny, nx))
    return labels, current_label


def find_dark_dot(
    img: np.ndarray,
    *,
    min_area_px: int = 30,
    max_area_frac: float = 0.25,
    roi: tuple | None = None,
    min_contrast: float = 5.0,
) -> DotDetection:
    """Locate the single largest dark blob (a dot against a bright field).

    Reports a measurement + quality note; never guesses a centroid. found is
    False and cx_px/cy_px/radius_px are all None whenever no qualifying blob
    exists, the best candidate is implausibly large (vignetting/shadow rather
    than a dot), or contrast is below the floor.
    """
    full_h, full_w = img.shape
    if roi is not None:
        x0, y0, x1, y1 = roi
        sub = img[y0:y1, x0:x1]
        offset = (x0, y0)
    else:
        sub = img
        offset = (0, 0)

    h, w = sub.shape
    frame_area = h * w
    thresh = _otsu_threshold(sub)
    mask = sub < thresh  # dark pixels

    labels, n_labels = _label_components(mask)
    if n_labels == 0:
        return DotDetection(
            found=False, cx_px=None, cy_px=None, radius_px=None,
            area_px=0, contrast=0.0, fill_ratio=0.0, n_candidates=0,
            image_size=(full_w, full_h), note="no dark region found",
        )

    areas = np.bincount(labels.ravel())  # index 0 = background
    candidate_labels = [
        lbl for lbl in range(1, n_labels + 1)
        if min_area_px <= areas[lbl] <= max_area_frac * frame_area
    ]
    n_candidates = len(candidate_labels)

    if not candidate_labels:
        best_area = int(areas[1:].max()) if n_labels > 0 else 0
        if best_area > max_area_frac * frame_area:
            note = (f"largest dark region ({best_area}px) exceeds max_area_frac; "
                    f"likely vignetting/shadow, not a dot")
        else:
            note = f"no dark region reaches min_area_px={min_area_px}"
        return DotDetection(
            found=False, cx_px=None, cy_px=None, radius_px=None,
            area_px=best_area, contrast=0.0, fill_ratio=0.0,
            n_candidates=0, image_size=(full_w, full_h), note=note,
        )

    best_label = max(candidate_labels, key=lambda lbl: areas[lbl])
    blob_mask = labels == best_label
    ys, xs = np.nonzero(blob_mask)
    area = int(areas[best_label])

    weights = (255.0 - sub[ys, xs])  # darker => higher weight
    weights = np.clip(weights, 1e-6, None)
    cx = float(np.sum(xs * weights) / np.sum(weights)) + offset[0]
    cy = float(np.sum(ys * weights) / np.sum(weights)) + offset[1]

    x_min, x_max = xs.min(), xs.max()
    y_min, y_max = ys.min(), ys.max()
    bbox_area = (x_max - x_min + 1) * (y_max - y_min + 1)
    fill_ratio = float(area / bbox_area) if bbox_area > 0 else 0.0
    radius_px = float(np.sqrt(area / np.pi))

    background_mask = ~mask
    if np.any(background_mask):
        background_median = float(np.median(sub[background_mask]))
    else:
        background_median = float(np.median(sub))
    blob_median = float(np.median(sub[ys, xs]))
    contrast = background_median - blob_median

    if contrast < min_contrast:
        return DotDetection(
            found=False, cx_px=None, cy_px=None, radius_px=None,
            area_px=area, contrast=contrast, fill_ratio=fill_ratio,
            n_candidates=n_candidates, image_size=(full_w, full_h),
            note=f"contrast {contrast:.2f} below floor {min_contrast}",
        )

    return DotDetection(
        found=True, cx_px=cx, cy_px=cy, radius_px=radius_px,
        area_px=area, contrast=contrast, fill_ratio=fill_ratio,
        n_candidates=n_candidates, image_size=(full_w, full_h),
        note="ok",
    )


def offset_from_center(det: DotDetection, image_size: tuple) -> tuple:
    """Signed pixel offset of the dot from frame centre.

    +x is right, +y is down (image convention). Caller must check det.found.
    """
    w, h = image_size
    if not det.found or det.cx_px is None or det.cy_px is None:
        raise ValueError("offset_from_center called on a non-found detection")
    cx0 = w / 2.0
    cy0 = h / 2.0
    return (det.cx_px - cx0, det.cy_px - cy0)


def sharpness(img: np.ndarray, *, roi: tuple | None = None) -> float:
    """Variance of a 3x3 discrete Laplacian, a focus-quality proxy.

    Only comparable within one sweep at one fixed exposure/illumination —
    never compare this value across sessions or lighting conditions.
    """
    if roi is not None:
        x0, y0, x1, y1 = roi
        sub = img[y0:y1, x0:x1]
    else:
        sub = img
    if sub.shape[0] < 3 or sub.shape[1] < 3:
        return 0.0
    center = sub[1:-1, 1:-1]
    up = sub[:-2, 1:-1]
    down = sub[2:, 1:-1]
    left = sub[1:-1, :-2]
    right = sub[1:-1, 2:]
    lap = up + down + left + right - 4.0 * center
    return float(np.var(lap))
