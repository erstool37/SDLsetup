"""Pixel-to-arm-mm affine transform, and its sign-convention contract.

SIGN CONVENTION (read before touching this file):

`solve_from_probes` measures how a KNOWN arm move changes the dot's pixel
position: it moves the arm by +step_mm in X (holding Y), then by +step_mm in
Y (holding X), and records where the dot lands in the image each time. That
gives the forward Jacobian J such that, to first order,

    [pixel_dx]       [arm_dx]
    [pixel_dy] = J @ [arm_dy]

i.e. J maps a KNOWN arm displacement to the OBSERVED pixel displacement it
causes.

`apply(offset_px)` runs this backwards for centering: given the CURRENT
pixel offset of the dot from the frame centre (dx_px, dy_px, from
`detect.offset_from_center` — the offset the dot currently has, not a
commanded move), it must return the arm XY correction that CANCELS that
offset, i.e. moving the arm by the returned amount is expected to bring the
dot back toward the frame centre.

If the offset is modeled as itself being an image of some (unknown) arm
displacement through the same J, then offset_px ~= J @ hypothetical_arm_disp.
Solving for hypothetical_arm_disp gives J^{-1} @ offset_px, which is the arm
move that WOULD HAVE PRODUCED this offset. Moving the arm by the NEGATIVE of
that quantity is what removes the offset:

    correction = -(J^{-1}) @ offset_px

This negative sign is not a bug and not a simplification target: dropping it
inverts the control loop and drives the dot to diverge instead of converge.
`routine.center_on_dot`'s residual-growth guard exists specifically to catch
this class of error at runtime; test 7 in the test suite exercises it with a
deliberately-inverted transform.
"""
from __future__ import annotations

import dataclasses
import json
import math
from pathlib import Path

import numpy as np

DEFAULT_PATH = Path.home() / ".sdl_lab" / "robot_arm" / "pixel_to_arm.json"


@dataclasses.dataclass
class PixelToArm:
    a11: float
    a12: float
    a21: float
    a22: float
    um_per_px_x: float
    um_per_px_y: float
    derived_at: str
    probe_step_mm: float
    residual_px: float
    source_note: str = ""

    def _matrix(self) -> np.ndarray:
        return np.array([[self.a11, self.a12], [self.a21, self.a22]], dtype=np.float64)

    def apply(self, det_offset_px: tuple) -> tuple:
        """Return the arm (dx_mm, dy_mm) correction that cancels det_offset_px.

        See module docstring for the sign derivation. Rejects non-finite
        inputs/outputs rather than passing them through.
        """
        dx_px, dy_px = det_offset_px
        if not (math.isfinite(dx_px) and math.isfinite(dy_px)):
            raise ValueError(f"non-finite pixel offset: {det_offset_px}")
        j = self._matrix()
        det = np.linalg.det(j)
        if abs(det) < 1e-9:
            raise ValueError(f"transform matrix is near-singular (det={det:.3e})")
        j_inv = np.linalg.inv(j)
        hypothetical_arm_disp = j_inv @ np.array([dx_px, dy_px], dtype=np.float64)
        correction = -hypothetical_arm_disp
        dx_mm, dy_mm = float(correction[0]), float(correction[1])
        if not (math.isfinite(dx_mm) and math.isfinite(dy_mm)):
            raise ValueError(f"computed correction is non-finite: ({dx_mm}, {dy_mm})")
        return (dx_mm, dy_mm)

    def invert(self) -> PixelToArm:
        """Return the transform mapping arm-mm displacement -> pixel displacement inverse.

        i.e. a PixelToArm-shaped object whose 2x2 matrix is J^{-1} instead of J.
        """
        j = self._matrix()
        det = np.linalg.det(j)
        if abs(det) < 1e-9:
            raise ValueError(f"transform matrix is near-singular (det={det:.3e})")
        j_inv = np.linalg.inv(j)
        return dataclasses.replace(
            self,
            a11=float(j_inv[0, 0]), a12=float(j_inv[0, 1]),
            a21=float(j_inv[1, 0]), a22=float(j_inv[1, 1]),
            source_note=(self.source_note + " [inverted]").strip(),
        )

    def save(self, path: Path | None = None) -> None:
        target = Path(path) if path is not None else DEFAULT_PATH
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(dataclasses.asdict(self), indent=2))

    @classmethod
    def load(cls, path: Path | None = None) -> PixelToArm:
        target = Path(path) if path is not None else DEFAULT_PATH
        data = json.loads(target.read_text())
        return cls(**data)


def solve_from_probes(
    base_px: tuple,
    after_x_px: tuple,
    after_y_px: tuple,
    step_mm: float,
    *,
    det_threshold: float = 1e-6,
) -> PixelToArm:
    """Solve the forward Jacobian J from three probe observations.

    base_px: dot pixel position at the starting pose.
    after_x_px: dot pixel position after moving the arm +step_mm in X only.
    after_y_px: dot pixel position after moving the arm +step_mm in Y only.
    step_mm: the KNOWN arm displacement used for both probes (must be > 0).

    J's columns are the observed pixel displacement per unit arm-mm
    displacement in X and Y respectively:
        J[:, 0] = (after_x_px - base_px) / step_mm
        J[:, 1] = (after_y_px - base_px) / step_mm
    """
    if step_mm <= 0 or not math.isfinite(step_mm):
        raise ValueError(f"step_mm must be a positive finite number, got {step_mm}")
    base = np.array(base_px, dtype=np.float64)
    ax = np.array(after_x_px, dtype=np.float64)
    ay = np.array(after_y_px, dtype=np.float64)

    col_x = (ax - base) / step_mm
    col_y = (ay - base) / step_mm
    j = np.column_stack([col_x, col_y])

    det = np.linalg.det(j)
    if abs(det) < det_threshold:
        raise ValueError(
            f"probe set is near-singular (det={det:.3e} < {det_threshold}); "
            "the two probe moves did not produce independent pixel motion"
        )

    norm_x = np.hypot(j[0, 0], j[1, 0])
    norm_y = np.hypot(j[0, 1], j[1, 1])
    um_per_px_x = 1000.0 / norm_x if norm_x > 0 else float("inf")
    um_per_px_y = 1000.0 / norm_y if norm_y > 0 else float("inf")

    import datetime as dt
    return PixelToArm(
        a11=float(j[0, 0]), a12=float(j[0, 1]),
        a21=float(j[1, 0]), a22=float(j[1, 1]),
        um_per_px_x=float(um_per_px_x), um_per_px_y=float(um_per_px_y),
        derived_at=dt.datetime.now().isoformat(timespec="seconds"),
        probe_step_mm=float(step_mm), residual_px=0.0,
        source_note="solved from 3-point probe (base, +x, +y)",
    )
