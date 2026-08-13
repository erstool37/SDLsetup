"""Port-based closed-loop centring, focus, and plate calibration routines.

**Nothing live calls this module today.** Its only consumer is
``test/test_calibration.py``, which drives it through fake ``ArmPort`` /
``CameraPort`` objects. It predates the shared ``tools.arm`` guard
layer and does not use it: motion here goes through whatever the caller passes
as ``ArmPort``, with no ``Envelope`` and no ``ValidatedMove``.

Two consequences worth knowing before wiring it to real hardware:

* ``_guarded_z_target`` **clamps**, where the motion-safety rule says guards
  raise. That is a deliberate, tested choice for sweep arithmetic that overshoots
  by construction -- but it is the opposite convention from
  ``devices/arm/safety.py``, and it also assumes Z *decreases* toward the sample,
  which is not this rig's geometry (here, raising Z drives the plate toward a
  fixed objective).
* Anything wired to a real arm should go through ``devices.arm.Arm`` instead, or
  pass an ``ArmPort`` that is backed by it.

Closed-loop centering, focus, and plate calibration routines.

HARD SAFETY NOTE: this module is written to be run ONLY against fakes
(ArmPort/CameraPort test doubles) during development. It must never be
exercised against the real xArm controller or a real camera by an agent
building or reviewing this code. All Z-computing paths funnel through the
single `_guarded_z_target` guard, which refuses to command a Z above the
taught reference Z and rejects non-finite targets.

Every routine here returns facts plus an advisory field; none of them retry a
measurement or discard a reading on their own judgement (surface law: sensors
report, they do not decide — this extends to the control loop as a whole).
"""
from __future__ import annotations

import dataclasses
import math
from typing import Protocol

import numpy as np

from ..arm import geometry as store
from ..microscope import imaging as detect


class ArmPort(Protocol):
    def get_pose(self) -> list: ...
    def move_cart(self, x, y, z, roll, pitch, yaw, speed) -> None: ...


class CameraPort(Protocol):
    def frame(self) -> np.ndarray: ...


# ---------------------------------------------------------------------------
# Centralized Z guard (guidance item 2). Every Z value this module computes
# and intends to command MUST pass through this function first.
# ---------------------------------------------------------------------------
def _guarded_z_target(target_z: float, taught_z: float) -> float:
    """Clamp target_z to never exceed taught_z; reject non-finite input.

    Raises ValueError on NaN/inf. Silently clamps (never raises) when
    target_z > taught_z, since coarse/fine sweep arithmetic can overshoot the
    ceiling by construction — the safe behavior is to cap at the taught
    reference, not to abort the whole sweep.
    """
    if not math.isfinite(target_z):
        raise ValueError(f"Z target is not finite: {target_z}")
    if not math.isfinite(taught_z):
        raise ValueError(f"taught_z is not finite: {taught_z}")
    return min(target_z, taught_z)


def _reject_nonfinite(*values: float, context: str = "") -> None:
    for v in values:
        if not math.isfinite(v):
            raise ValueError(f"non-finite value encountered ({context}): {v}")


# ---------------------------------------------------------------------------
# center_on_dot
# ---------------------------------------------------------------------------
@dataclasses.dataclass
class CenteringResult:
    converged: bool
    residual_mm: float
    iterations: list
    aborted_reason: str | None
    final_pose: tuple | None


def center_on_dot(
    arm: ArmPort,
    cam: CameraPort,
    transform,
    *,
    tolerance_mm: float = 0.02,
    max_iterations: int = 4,
    max_correction_mm: float = 3.0,
    speed: float = 10.0,
) -> CenteringResult:
    """Iteratively move the arm so the detected dot sits at frame centre.

    Mandatory guards, in order each iteration: dot-not-found abort;
    non-finite/singular correction abort; single-step correction magnitude
    vs max_correction_mm; cumulative displacement from the starting pose vs
    max_correction_mm; residual-growth-for-two-iterations sign-error guard.
    Never silently accepts non-convergence — returns converged=False with an
    explicit aborted_reason in every non-convergent path.
    """
    start_pose = list(arm.get_pose())
    start_x, start_y = start_pose[0], start_pose[1]
    iterations = []
    prev_residual = None
    growth_streak = 0

    for _ in range(max_iterations):
        frame = cam.frame()
        det = detect.find_dark_dot(frame)
        if not det.found:
            return CenteringResult(
                converged=False, residual_mm=float("nan"), iterations=iterations,
                aborted_reason=f"dot not found: {det.note}",
                final_pose=tuple(arm.get_pose()),
            )

        offset_px = detect.offset_from_center(det, det.image_size)
        try:
            dx_mm, dy_mm = transform.apply(offset_px)
        except ValueError as exc:
            return CenteringResult(
                converged=False, residual_mm=float("nan"), iterations=iterations,
                aborted_reason=f"correction computation failed: {exc}",
                final_pose=tuple(arm.get_pose()),
            )

        residual_mm = math.hypot(dx_mm, dy_mm)
        iterations.append({
            "offset_px": offset_px, "correction_mm": (dx_mm, dy_mm),
            "residual_mm": residual_mm,
        })

        if residual_mm <= tolerance_mm:
            return CenteringResult(
                converged=True, residual_mm=residual_mm, iterations=iterations,
                aborted_reason=None, final_pose=tuple(arm.get_pose()),
            )

        if residual_mm > max_correction_mm:
            return CenteringResult(
                converged=False, residual_mm=residual_mm, iterations=iterations,
                aborted_reason=(
                    f"single-step correction {residual_mm:.3f}mm exceeds "
                    f"max_correction_mm={max_correction_mm}"
                ),
                final_pose=tuple(arm.get_pose()),
            )

        cur = list(arm.get_pose())
        cum_dx = (cur[0] + dx_mm) - start_x
        cum_dy = (cur[1] + dy_mm) - start_y
        cum_mag = math.hypot(cum_dx, cum_dy)
        if cum_mag > max_correction_mm:
            return CenteringResult(
                converged=False, residual_mm=residual_mm, iterations=iterations,
                aborted_reason=(
                    f"cumulative displacement {cum_mag:.3f}mm from start exceeds "
                    f"max_correction_mm={max_correction_mm}"
                ),
                final_pose=tuple(arm.get_pose()),
            )

        if prev_residual is not None and residual_mm > prev_residual:
            growth_streak += 1
            if growth_streak >= 2:
                return CenteringResult(
                    converged=False, residual_mm=residual_mm, iterations=iterations,
                    aborted_reason=(
                        "residual grew for two consecutive iterations "
                        f"({prev_residual:.4f} -> {residual_mm:.4f}); "
                        "possible sign-inverted transform, aborting"
                    ),
                    final_pose=tuple(arm.get_pose()),
                )
        else:
            growth_streak = 0
        prev_residual = residual_mm

        new_x = cur[0] + dx_mm
        new_y = cur[1] + dy_mm
        _reject_nonfinite(new_x, new_y, context="center_on_dot target pose")
        arm.move_cart(new_x, new_y, cur[2], cur[3], cur[4], cur[5], speed)

    return CenteringResult(
        converged=False,
        residual_mm=prev_residual if prev_residual is not None else float("nan"),
        iterations=iterations,
        aborted_reason=f"max_iterations={max_iterations} exhausted without convergence",
        final_pose=tuple(arm.get_pose()),
    )


# ---------------------------------------------------------------------------
# spiral_search
# ---------------------------------------------------------------------------
@dataclasses.dataclass
class SearchResult:
    found: bool
    pose: tuple | None
    detection: detect.DotDetection | None
    points_tried: int
    aborted_reason: str | None


def _square_spiral_offsets(step_mm: float, max_radius_mm: float):
    """Yield (dx, dy) offsets in an outward square spiral, XY only."""
    yield (0.0, 0.0)
    ring = 1
    while ring * step_mm <= max_radius_mm + 1e-9:
        r = ring * step_mm
        # right edge (top to bottom), bottom edge (right to left),
        # left edge (bottom to top), top edge (left to right)
        n = 2 * ring
        for i in range(n):
            yield (r, -r + (i + 1) * step_mm)
        for i in range(n):
            yield (r - (i + 1) * step_mm, r)
        for i in range(n):
            yield (-r, r - (i + 1) * step_mm)
        for i in range(n):
            yield (-r + (i + 1) * step_mm, -r)
        ring += 1


def spiral_search(
    arm: ArmPort,
    cam: CameraPort,
    *,
    step_mm: float = 0.5,
    max_radius_mm: float = 3.0,
    speed: float = 10.0,
) -> SearchResult:
    """Outward square spiral in XY only (never touches Z) until a confident detection."""
    start = list(arm.get_pose())
    start_x, start_y = start[0], start[1]
    points_tried = 0
    for dx, dy in _square_spiral_offsets(step_mm, max_radius_mm):
        x = start_x + dx
        y = start_y + dy
        _reject_nonfinite(x, y, context="spiral_search target")
        arm.move_cart(x, y, start[2], start[3], start[4], start[5], speed)
        points_tried += 1
        det = detect.find_dark_dot(cam.frame())
        if det.found:
            return SearchResult(
                found=True, pose=tuple(arm.get_pose()), detection=det,
                points_tried=points_tried, aborted_reason=None,
            )
    return SearchResult(
        found=False, pose=tuple(arm.get_pose()), detection=None,
        points_tried=points_tried,
        aborted_reason=f"no dot found within max_radius_mm={max_radius_mm}",
    )


# ---------------------------------------------------------------------------
# autofocus_z
# ---------------------------------------------------------------------------
@dataclasses.dataclass
class FocusResult:
    best_z: float
    best_sharpness: float
    curve: list
    at_ceiling: bool
    note: str


def autofocus_z(
    arm: ArmPort,
    cam: CameraPort,
    *,
    taught_z: float,
    span_mm: float = 3.0,
    coarse_step_mm: float = 0.25,
    fine_step_mm: float = 0.05,
    roi: tuple | None = None,
    speed: float = 10.0,
) -> FocusResult:
    """Coarse-then-fine Z sweep for peak sharpness, never above taught_z.

    Sweeps strictly downward from taught_z through taught_z - span_mm. Every
    commanded Z passes through `_guarded_z_target`. If the coarse peak sits
    at the ceiling (taught_z itself), sets at_ceiling=True and does not
    auto-extend the range upward — that requires explicit operator approval.
    """
    if not math.isfinite(taught_z) or not math.isfinite(span_mm):
        raise ValueError(f"non-finite taught_z/span_mm: {taught_z}, {span_mm}")
    cur = list(arm.get_pose())
    curve = []

    def _measure_at(z_raw: float) -> tuple:
        z = _guarded_z_target(z_raw, taught_z)
        arm.move_cart(cur[0], cur[1], z, cur[3], cur[4], cur[5], speed)
        s = detect.sharpness(cam.frame(), roi=roi)
        curve.append((z, s))
        return z, s

    n_coarse = int(round(span_mm / coarse_step_mm)) + 1
    for i in range(n_coarse):
        _measure_at(taught_z - i * coarse_step_mm)

    best_z, best_s = max(curve, key=lambda t: t[1])
    if math.isclose(best_z, taught_z, abs_tol=1e-9):
        return FocusResult(
            best_z=best_z, best_sharpness=best_s, curve=list(curve), at_ceiling=True,
            note=(
                "sharpness peak found at the taught_z ceiling; a higher Z may "
                "sharpen further but this routine will not auto-extend range — "
                "requires explicit operator approval"
            ),
        )

    fine_lo = best_z - coarse_step_mm
    fine_hi = min(best_z + coarse_step_mm, taught_z)
    n_fine = max(int(round((fine_hi - fine_lo) / fine_step_mm)), 0) + 1
    for i in range(n_fine):
        _measure_at(fine_hi - i * fine_step_mm)

    best_z, best_s = max(curve, key=lambda t: t[1])
    at_ceiling = math.isclose(best_z, taught_z, abs_tol=1e-9)
    note = "at_ceiling: see FLAGS" if at_ceiling else "ok"
    return FocusResult(best_z=best_z, best_sharpness=best_s, curve=list(curve),
                       at_ceiling=at_ceiling, note=note)


# ---------------------------------------------------------------------------
# calibrate_plate
# ---------------------------------------------------------------------------
def calibrate_plate(
    arm: ArmPort,
    cam: CameraPort,
    transform,
    *,
    taught_a1: tuple,
    scale_tolerance: float = 0.01,
    speed: float = 10.0,
    **routine_kwargs,
) -> store.WellCalibration:
    """End-to-end plate calibration: focus + centre at A1, then at nominal A12.

    Refuses (raises RuntimeError) to commit a calibration if either centring
    fails to converge, or if the recovered scale_along_row is out of
    tolerance of nominal.
    """
    x1, y1, z1, roll1, pitch1, yaw1 = taught_a1
    arm.move_cart(x1, y1, z1, roll1, pitch1, yaw1, speed)

    focus = autofocus_z(arm, cam, taught_z=z1, roi=routine_kwargs.get("roi"))
    cur = list(arm.get_pose())
    z_focused = _guarded_z_target(focus.best_z, z1)
    arm.move_cart(cur[0], cur[1], z_focused, cur[3], cur[4], cur[5], speed)

    center_a1 = center_on_dot(arm, cam, transform, speed=speed)
    if not center_a1.converged:
        raise RuntimeError(f"A1 centering failed to converge: {center_a1.aborted_reason}")
    a1_pose = tuple(arm.get_pose())

    nominal_a12_x = x1 - store.NOMINAL_A1_TO_A12_MM
    arm.move_cart(nominal_a12_x, y1, z_focused, roll1, pitch1, yaw1, speed)

    det = detect.find_dark_dot(cam.frame())
    if not det.found:
        search = spiral_search(arm, cam)
        if not search.found:
            raise RuntimeError(f"A12 not found even after spiral search: {search.aborted_reason}")

    center_a12 = center_on_dot(arm, cam, transform, speed=speed)
    if not center_a12.converged:
        raise RuntimeError(f"A12 centering failed to converge: {center_a12.aborted_reason}")
    a12_pose = tuple(arm.get_pose())

    calibration = store.derive_calibration(
        a1_pose=a1_pose, a12_pose=a12_pose, taught_a1_pose=tuple(taught_a1),
        residual_mm=max(center_a1.residual_mm, center_a12.residual_mm),
        n_iterations=len(center_a1.iterations) + len(center_a12.iterations),
    )

    if abs(calibration.scale_along_row - 1.0) > scale_tolerance:
        raise RuntimeError(
            f"scale_along_row={calibration.scale_along_row:.4f} outside tolerance "
            f"{scale_tolerance}; refusing to commit calibration"
        )

    return calibration
