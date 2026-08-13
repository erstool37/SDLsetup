#!/usr/bin/env python3
"""calibrate.py -- one call that calibrates the microscope position, the focal
point, and the grid-movement model.

    from scripts.microscope.calibrate import calibrate_microscope_position_and_focal_point
    report = calibrate_microscope_position_and_focal_point(live=True)
    print(report.summary())

    python3 scripts/microscope/calibrate.py --execute        # same thing, CLI

WHAT IT ASSUMES, AND WHY THAT SHAPES THE ALGORITHM
--------------------------------------------------
The operator's starting conditions, and the algorithm is built around them:

* **A1 is approximately right, not exactly.** The taught pose puts at least half
  the well in frame, but the black dot may be well off centre. So stage 1 does
  not assume the dot is under the crosshair -- it searches outward if the dot is
  not in frame at all, then centres it.
* **Focus is approximately right.** It has been set before, so stage 2 VERIFIES
  rather than searches: a narrow sweep about the current height. A verification
  that cannot fail is worthless, so it reports prominence and says plainly
  whether the plane is real.
* **The plate may be re-seated and tilted.** Nominal 9.0 mm steps along the arm
  axes are then wrong, and the error compounds: 1 deg of seating rotation moves
  a well 99 mm away by ~1.7 mm, more than half a field of view. Stage 3
  measures the real grid from whichever reference dots exist.

STAGES, each of which reports and none of which silently continues on failure:

  1. preflight   -- arm reachable and near the taught A1, illumination in band,
                    a measured Jacobian available.
  2. centre_a1   -- put the black dot on the crosshair. Searches outward first
                    if it is not in frame.
  3. focus_check -- narrow Z sweep about the current height; prominence
                    reported. Advisory by default (require_focus=True to gate).
  4. grid        -- centre whatever reference dots exist and solve the grid.
                    Full 2x2 if a ROW-neighbour dot is found, column-only
                    otherwise, and it says which. Two collinear points cannot
                    separate a rotation from a row-axis error.
  5. store       -- write the model with provenance, including what was NOT
                    measured.

THE FEEDBACK LOOP -- :func:`image_wells`
----------------------------------------
Driving to a well is not evidence of arriving over it. Every capture is checked
two ways and the result travels with the image:

* **arm side** -- ``MoveResult.verify()`` compares achieved against commanded on
  all six axes. This catches the controller's silent refusal, which reports
  success with error_code 0.
* **image side** -- if a dot is detected, its offset from the frame centre is
  converted to mm through the measured Jacobian and reported. If no dot is
  present (most wells have none) the illuminated aperture's centroid is used as
  a COARSE flag, and if even that is unusable the record says ``no_reference``
  rather than pretending the frame is centred.

Never asserts a well is centred on no evidence. `no_reference` is a real
outcome and is reported as one.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse  # noqa: E402
import dataclasses  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402
from collections.abc import Sequence  # noqa: E402
from typing import Any

from PIL import Image  # noqa: E402

from tools.arm import Arm, ArmSettings  # noqa: E402
from tools.arm.safety import Envelope  # noqa: E402
from tools.arm.workspace import WorkspaceStore  # noqa: E402
from tools.calib import adapters, centering  # noqa: E402
from tools.calib import pixel_scale as ps  # noqa: E402
from tools.microscope import Microscope, MicroscopeSettings  # noqa: E402
from tools.microscope import imaging as detect  # noqa: E402

WORKSPACE_ANCHOR = "microscope"
PLATE_CONFIG = Path("/home/lamp/.sdl_lab/robot_arm/plate_config.json")
SCALE_JSON = Path("/home/lamp/.sdl_lab/robot_arm/pixel_scale.json")
CALIBRATION_JSON = Path("/home/lamp/.sdl_lab/robot_arm/well_calibration.json")

#: XY box for the whole procedure. Must reach column 12 (99 mm) and no further.
XY_BOX_MM = 105.0

#: Frame mean outside this band means the scored image is noise or clipped, and
#: nothing downstream is trustworthy. Measured on this rig: a usable frame reads
#: ~159; 10x underexposed read 2.3 and preflight correctly refused it.
MIN_FRAME_MEAN = 30.0
MAX_FRAME_MEAN = 245.0

#: Centring tolerance. The dot is HAND-DRAWN and ~1.4 mm across; its centroid
#: jitters ~25 px (~0.03 mm) frame to frame, so a tighter tolerance produces a
#: limit cycle rather than convergence. Measured 2026-08-11: at 0.02 mm the loop
#: oscillated over a 33 um band and never converged; at 0.05 mm it converged in
#: one iteration.
CENTRE_TOL_MM = 0.05

#: How far centring may walk at one well before it is presumed to be chasing the
#: wrong feature. The field of view is 2.84 x 2.38 mm, so a correction larger
#: than its half-height means the target is off-frame -- which is exactly how a
#: 2026-08-11 run walked 2.02 mm onto a well-wall arc.
MAX_CORRECTION_MM = 1.2

#: A fitted column pitch may deviate this much from nominal before the
#: reference is rejected as "not a dot". 96-well plates are drilled, not
#: hand-placed, so a real column step is within a percent or two of 9.0 mm.
MAX_PITCH_DEVIATION = 0.05

#: Largest seating rotation the correction budget should tolerate, deg.
#:
#: The budget cannot be one number. A dot 99 mm from A1 is displaced by
#: distance*tan(rotation): 0.8 deg puts A12 1.4 mm off nominal, which a flat
#: 1.2 mm cap rejects as "chasing the wrong feature" even though it is exactly
#: the displacement this calibration exists to measure. The same 1.4 mm at A2,
#: 9 mm away, would imply 9 deg and IS implausible.
#:
#: So the budget grows with distance from A1, which separates the two cases by
#: the only thing that distinguishes them: the angle they imply. 1.5 deg is
#: generous against the -0.39 deg measured on this rig.
MAX_SEATING_ROT_DEG = 1.5

#: Budget floor: centring precision plus how far a HAND-DRAWN dot may sit from
#: its own well centre. Applies even at zero distance.
CORRECTION_BASE_MM = 0.35

#: Dot plausibility, as equivalent radius in mm. The A1 dot measures ~0.70 mm
#: radius. Anything far outside this is a speck or the unlit surround, not a dot.
#:
#: The FLOOR is what separates a drawn dot from debris, and it was measured, not
#: guessed: the A1 and A12 dots are ~0.70 mm equivalent radius, while the specks
#: and hairs sitting in A4 and A8 are far smaller. At a 0.15 mm floor those
#: specks were classified as dots and the per-frame check reported A4 and A8
#: "off_centre" by 0.45 and 1.41 mm -- false alarms about wells that carry no
#: mark at all. A false "off_centre" is not harmless: it invites a correction
#: toward a speck.
#: A filled disc scores pi/4 = 0.785 against its bounding box; a crescent
#: hugging the frame edge scores far lower. Measured on this rig's real dots.
MIN_DOT_FILL_RATIO = 0.55

DOT_R_MIN_MM = 0.40
DOT_R_MAX_MM = 1.60


@dataclasses.dataclass
class StageResult:
    name: str
    ok: bool
    detail: dict[str, Any] = dataclasses.field(default_factory=dict)
    reason: str | None = None

    def line(self) -> str:
        mark = "ok  " if self.ok else "FAIL"
        bits = " ".join(f"{k}={v}" for k, v in self.detail.items())
        return f"  [{mark}] {self.name:<12} {bits}" + (
            f"\n           reason: {self.reason}" if self.reason else "")


@dataclasses.dataclass
class CalibrationReport:
    stages: list[StageResult] = dataclasses.field(default_factory=list)
    a1_centred: tuple[float, float] | None = None
    focus_z: float | None = None
    grid: dict[str, Any] | None = None
    stored_to: str | None = None

    @property
    def ok(self) -> bool:
        return bool(self.stages) and all(s.ok for s in self.stages)

    def summary(self) -> str:
        head = "CALIBRATION %s" % ("OK" if self.ok else "INCOMPLETE")
        body = "\n".join(s.line() for s in self.stages)
        tail = []
        if self.a1_centred:
            tail.append("  A1 centred at x=%.4f y=%.4f" % self.a1_centred)
        if self.focus_z is not None:
            tail.append("  focus z=%.4f" % self.focus_z)
        if self.grid:
            tail.append("  grid: col step (%+.4f, %+.4f) mm, rotation %+.4f deg, row %s"
                        % (self.grid["col_step_mm"][0], self.grid["col_step_mm"][1],
                           self.grid["col_rotation_deg"], self.grid["row_axis"]))
        if self.stored_to:
            tail.append("  stored: %s" % self.stored_to)
        return "\n".join([head, body] + tail)


def parse_well(name: str) -> tuple[int, int]:
    name = name.strip().upper()
    row = ord(name[0]) - ord("A")
    col = int(name[1:]) - 1
    if not (0 <= row <= 7 and 0 <= col <= 11):
        raise ValueError(f"{name!r} is not a well on a 96-well plate")
    return row, col


def _dot_in_frame(flat, px_per_mm: float, tmp: Path | None = None) -> tuple[bool, dict]:
    """Is there a plausible DOT here? Reports; the caller decides.

    SIZE ALONE DOES NOT SEPARATE A DOT FROM A WELL-WALL SHADOW. Measured
    2026-08-11/12: the real dots at A1 and A12 have equivalent radii of 0.698
    and 0.544 mm, while the wall shadows in A4 and A8 -- wells that carry no
    mark at all -- measure 0.739 and 0.578 mm. The radius bands overlap
    completely, which is why raising the floor from 0.15 to 0.40 mm changed
    nothing.

    SHAPE and BORDER CONTACT do separate them. A drawn dot is a compact blob
    sitting away from the frame edge; a wall shadow is a crescent hugging it
    (A8's sat 1266 px right of centre, at the edge of a 3072 px frame). Both
    are already computed by ``marker.detect``, so that is used rather than a
    third blob analyser -- two implementations of one measurement is the failure
    this repo keeps having.
    """
    det = detect.find_dark_dot(flat, max_area_frac=0.5)
    if not det.found:
        return False, {"found": False, "note": getattr(det, "note", "")}
    area = float(getattr(det, "area_px", 0.0))
    r_mm = math.sqrt(max(area, 0.0) / math.pi) / max(px_per_mm, 1e-9)
    info: dict[str, Any] = {"found": True, "r_mm": round(r_mm, 3)}
    if not (DOT_R_MIN_MM <= r_mm <= DOT_R_MAX_MM):
        info["reject"] = "radius out of band"
        return False, info

    # Shape test, via the existing detector. Needs a file, so the flat frame is
    # written out; that copy is also the evidence for the record.
    try:
        from tools.microscope import marker
        tmp = tmp or Path("/tmp/_dotshape.png")
        Image.fromarray(flat).save(tmp)
        m = marker.detect(tmp, mode="dark")
        cands = [c for c in m.get("candidates", []) if c.get("area_px")]
        if not cands:
            # FAIL CLOSED. An empty candidate list means the shape detector
            # found nothing dot-like -- that is a rejection, not a pass. Written
            # as `if cands:` this fell through to "confirmed dot" and reported
            # A4 and A8 off_centre by 0.45 and 1.41 mm on the strength of a well
            # shadow, in wells that carry no mark at all.
            info["reject"] = "no dot-shaped candidate in the frame"
            return False, info
        if cands:
            c = max(cands, key=lambda c: c["area_px"])
            fill = float(c.get("fill_ratio", 0.0))
            border = bool(c.get("touches_frame_border", False))
            info.update(fill_ratio=round(fill, 3), touches_border=border)
            if border:
                info["reject"] = "touches the frame border (wall shadow, not a dot)"
                return False, info
            if fill < MIN_DOT_FILL_RATIO:
                info["reject"] = (f"fill ratio {fill:.2f} below "
                                  f"{MIN_DOT_FILL_RATIO} (crescent, not a disc)")
                return False, info
    except Exception as exc:                                   # noqa: BLE001
        # Unknown shape must not read as "confirmed dot".
        info["reject"] = f"shape test unavailable: {exc}"
        return False, info
    return True, info


def calibrate_microscope_position_and_focal_point(
    *,
    live: bool = False,
    host: str | None = None,
    reference_wells: Sequence[str] = ("A2", "B1", "A12"),
    speed: float = 5.0,
    centre_tol_mm: float = CENTRE_TOL_MM,
    max_correction_mm: float = MAX_CORRECTION_MM,
    max_iterations: int = 8,
    search_radius_mm: float = 2.0,
    search_step_mm: float = 0.5,
    focus_span_mm: float = 1.2,
    focus_coarse_mm: float = 0.15,
    focus_fine_mm: float = 0.05,
    require_focus: bool = False,
    store: bool = True,
    out_dir: str | Path = "/home/lamp/SDLsetup/dataset/captures/calibrate",
    log=print,
) -> CalibrationReport:
    """Calibrate microscope XY position, focal point and grid movement.

    Returns a :class:`CalibrationReport`. Raises only on genuinely unrecoverable
    conditions; expected failures come back as a stage with ``ok=False`` and a
    reason, so a caller can inspect what happened rather than catching.
    """
    report = CalibrationReport()
    settings = ArmSettings.from_config(**({"live": live} | ({"host": host} if host else {})))
    store_ws = WorkspaceStore(settings.workspace_path)
    a1p = store_ws.require_location(WORKSPACE_ANCHOR).pose
    a1 = (float(a1p.x), float(a1p.y), float(a1p.z),
          float(a1p.roll), float(a1p.pitch), float(a1p.yaw))

    cfg = json.loads(PLATE_CONFIG.read_text())
    amap = cfg["axis_map"]
    pitch_nom = float(cfg.get("well_pitch_mm", 9.0))

    def nominal(row: int, col: int) -> tuple[float, float]:
        x, y = a1[0], a1[1]
        dc = pitch_nom * col * float(amap["col_sign"])
        dr = pitch_nom * row * float(amap["row_sign"])
        if amap["col_axis"] == "x":
            x += dc
        else:
            y += dc
        if amap["row_axis"] == "x":
            x += dr
        else:
            y += dr
        return x, y

    # ---------------------------------------------------------- 1. preflight
    try:
        model = ps.load(SCALE_JSON)
        J = ps.jacobian_at(model, a1[2])
        jd = ps.decompose(J)
    except Exception as exc:                                   # noqa: BLE001
        report.stages.append(StageResult(
            "preflight", False, {"jacobian": "missing"},
            f"no usable pixel->arm transform at {SCALE_JSON}: {exc}. "
            f"Run scripts/microscope/pixel_scale.py first -- without it a pixel "
            f"offset cannot be turned into an arm move."))
        return report
    transform = adapters.JacobianTransform(J)
    px_per_mm = float(jd["px_per_mm_mean"])

    if not live:
        report.stages.append(StageResult(
            "preflight", True,
            {"mode": "dry-run", "px_per_mm": round(px_per_mm, 1),
             "rotation_deg": round(jd["rotation_deg"], 2)},
            "dry run: nothing moved, nothing captured"))
        for name in ("centre_a1", "focus_check", "grid", "store"):
            report.stages.append(StageResult(name, True, {"mode": "dry-run"}))
        return report

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    robot = Arm(settings, log=log)
    robot.connection.connect(arm=True)
    # Z budget sized to the FOCUS SWEEP and nothing more. A zero-budget window
    # made stage 3 structurally impossible -- it aborted with
    # "z 191.0588 outside [191.2088, 191.2088]" because the sweep cannot move at
    # all. The upward half is the dangerous direction (toward the objective) so
    # it is exactly the declared span, never a round number chosen for comfort;
    # the downward half is away from the lens.
    env = Envelope.anchored(a1, name="A1", xy_max_mm=XY_BOX_MM,
                            z_max_rise_mm=float(focus_span_mm),
                            z_min_rise_mm=-float(focus_span_mm),
                            orient_tol_deg=0.5)
    guarded = robot.with_envelope(env)
    scope = Microscope(MicroscopeSettings.from_config())

    try:
        here = list(guarded.pose())
        off_a1 = math.hypot(here[0] - a1[0], here[1] - a1[1])
        # The entry point must work when called cold, from wherever the previous
        # run left the arm. Measured: a run ended at A12 and the next call began
        # centring 99 mm away, which simply refused. Anything inside the XY box
        # is recoverable by a planar move at the working height; anything
        # outside it is not this function's job to fix.
        if off_a1 > XY_BOX_MM:
            report.stages.append(StageResult(
                "preflight", False, {"off_taught_A1_mm": round(off_a1, 3)},
                f"the arm is {off_a1:.1f} mm from the taught A1, outside the "
                f"{XY_BOX_MM} mm working box. Bring it to the microscope station "
                f"first (pick_place.py); this function will not traverse the bench."))
            return report
        if off_a1 > 0.5:
            log(f"[cal] arm is {off_a1:.3f} mm from the taught A1; driving there first")
            adapters.axis_sequential_to(guarded, a1, a1[0], a1[1],
                                        speed=speed, label="to A1")
            here = list(guarded.pose())
        cam = adapters.FlatFieldCamera(scope, out_dir, tag="pre")
        flat = cam.frame()
        raw_mean = float(
            __import__("numpy").asarray(Image.open(cam.last_path).convert("L"),
                                        dtype=float).mean())
        lit_ok = MIN_FRAME_MEAN <= raw_mean <= MAX_FRAME_MEAN
        report.stages.append(StageResult(
            "preflight", lit_ok,
            {"off_taught_A1_mm": round(off_a1, 3),
             "now_off_mm": round(math.hypot(here[0] - a1[0], here[1] - a1[1]), 3),
             "frame_mean": round(raw_mean, 1),
             "px_per_mm": round(px_per_mm, 1),
             "rotation_deg": round(jd["rotation_deg"], 2)},
            None if lit_ok else
            f"frame mean {raw_mean:.1f} outside [{MIN_FRAME_MEAN}, {MAX_FRAME_MEAN}] -- "
            f"scoring this ranks sensor noise or clipping. Fix the illumination."))
        if not lit_ok:
            return report

        # ------------------------------------------------------ 2. centre A1
        found, dinfo = _dot_in_frame(flat, px_per_mm)
        searched = None
        if not found:
            log(f"[cal] no plausible dot at A1 ({dinfo}); searching outward "
                f"<= {search_radius_mm} mm")
            sres = centering.spiral_search(
                adapters.PinnedArm(guarded, a1), cam,
                step_mm=search_step_mm, max_radius_mm=search_radius_mm, speed=speed)
            searched = getattr(sres, "found", None)
            if not searched:
                report.stages.append(StageResult(
                    "centre_a1", False, {"searched_mm": search_radius_mm},
                    "no dot found at A1 or within the search radius. The taught A1 "
                    "may be wrong, or this well carries no mark."))
                return report

        cres = centering.center_on_dot(
            adapters.PinnedArm(guarded, a1), cam, transform,
            tolerance_mm=centre_tol_mm, max_iterations=max_iterations,
            max_correction_mm=max_correction_mm, speed=speed)
        pose = list(guarded.pose())
        report.stages.append(StageResult(
            "centre_a1", bool(cres.converged),
            {"residual_mm": round(float(cres.residual_mm), 4),
             "x": round(pose[0], 4), "y": round(pose[1], 4),
             "searched": bool(searched)},
            None if cres.converged else str(cres.aborted_reason)))
        if not cres.converged:
            return report
        report.a1_centred = (pose[0], pose[1])
        centred_anchor = (pose[0], pose[1], a1[2], a1[3], a1[4], a1[5])

        # ---------------------------------------------------- 3. focus check
        try:
            fres = centering.autofocus_z(
                adapters.PinnedXYArm(guarded, centred_anchor), cam,
                taught_z=a1[2], span_mm=focus_span_mm,
                coarse_step_mm=focus_coarse_mm, fine_step_mm=focus_fine_mm,
                speed=speed)
            fz = getattr(fres, "best_z", None) or getattr(fres, "z", None)
            prom = getattr(fres, "prominence", None)
            fok = bool(getattr(fres, "ok", True))
            report.focus_z = float(fz) if fz is not None else None
            report.stages.append(StageResult(
                "focus_check", fok if require_focus else True,
                {"best_z": None if fz is None else round(float(fz), 4),
                 "prominence": prom, "gating": require_focus},
                None if fok else
                f"no convincing focal plane (prominence {prom}). Reported, not "
                f"gated: require_focus=True to make this fatal."))
        except Exception as exc:                               # noqa: BLE001
            # ok=False regardless of require_focus. `require_focus` decides
            # whether a WEAK focal plane is fatal; it must never make a stage
            # that failed to RUN look like one that passed.
            report.stages.append(StageResult(
                "focus_check", False, {"error": type(exc).__name__},
                f"focus verification did not complete: {exc}"))

        # ------------------------------------------------------- 4. the grid
        refs: dict[str, tuple[float, float]] = {}
        for well in reference_wells:
            r, c = parse_well(well)
            nx, ny = nominal(r, c)
            # Offset the nominal by A1's own correction so the reference starts
            # from the same place the grid does.
            nx += report.a1_centred[0] - a1[0]
            ny += report.a1_centred[1] - a1[1]
            dist = math.hypot(nx - report.a1_centred[0], ny - report.a1_centred[1])
            budget = CORRECTION_BASE_MM + dist * math.tan(
                math.radians(MAX_SEATING_ROT_DEG))
            log(f"[cal] reference {well}: driving to x={nx:.4f} y={ny:.4f} "
                f"({dist:.1f} mm from A1, correction budget {budget:.2f} mm "
                f"= {MAX_SEATING_ROT_DEG} deg of seating rotation)")
            adapters.axis_sequential_to(guarded, a1, nx, ny, speed=speed,
                                        label=f"to {well}")
            cam = adapters.FlatFieldCamera(scope, out_dir, tag=well)
            ok_dot, info = _dot_in_frame(cam.frame(), px_per_mm)
            if not ok_dot:
                log(f"[cal] {well}: no plausible dot ({info}) -- skipped")
                continue
            cr = centering.center_on_dot(
                adapters.PinnedArm(guarded, a1), cam, transform,
                tolerance_mm=centre_tol_mm, max_iterations=max_iterations,
                max_correction_mm=budget, speed=speed)
            if not cr.converged:
                log(f"[cal] {well}: centring did not converge "
                    f"({cr.aborted_reason}) -- skipped")
                continue
            p = list(guarded.pose())
            refs[well] = (p[0], p[1])
            log(f"[cal] {well}: centred x={p[0]:.4f} y={p[1]:.4f}")

        grid = _solve_grid(report.a1_centred, refs, amap, pitch_nom)
        report.grid = grid
        report.stages.append(StageResult(
            "grid", grid is not None,
            {"references": ",".join(refs) or "none",
             "row_axis": (grid or {}).get("row_axis", "n/a")},
            None if grid else
            "no reference dot could be centred, so the grid is still the nominal "
            "one. Movement to distant wells is unverified."))
        if grid is None:
            return report

        # ---------------------------------------------------------- 5. store
        # Only a run whose every stage passed may overwrite the stored model. A
        # bad grid mis-addresses all 96 wells silently, which is strictly worse
        # than keeping the previous one and saying so.
        if store and not report.ok:
            report.stages.append(StageResult(
                "store", False, {"path": "(not written)"},
                "an earlier stage failed, so the previous calibration was left "
                "in place rather than overwritten with this run's result."))
            return report
        if store:
            payload = {
                "taught_a1": list(a1), "a1_centred": list(report.a1_centred),
                "focus_z": report.focus_z, "references": {k: list(v) for k, v in refs.items()},
                "jacobian_decomposition": jd,
                **grid,
            }
            CALIBRATION_JSON.write_text(json.dumps(payload, indent=1))
            report.stored_to = str(CALIBRATION_JSON)
        report.stages.append(StageResult("store", True,
                                         {"path": report.stored_to or "(not stored)"}))
        return report
    finally:
        robot.close()


def _solve_grid(origin, refs: dict, amap: dict, pitch_nom: float) -> dict | None:
    """Grid model from whatever reference dots were centred.

    Prefers a ROW-neighbour + COLUMN-neighbour pair (full 2x2). Falls back to
    column-only, and SAYS SO: two points on one axis cannot separate a rotation
    from a row-axis error, so a column-only fit leaves the row direction nominal
    and unmeasured, which is not the same thing as measured-and-square.
    """
    if not refs:
        return None
    col_vecs, row_vecs = [], []
    for well, p in refs.items():
        r, c = parse_well(well)
        dx, dy = p[0] - origin[0], p[1] - origin[1]
        if c > 0 and r == 0:
            col_vecs.append(((dx / c, dy / c), well))
        elif r > 0 and c == 0:
            row_vecs.append(((dx / r, dy / r), well))
    # Plausibility gate. A reference whose implied step is far from the nominal
    # pitch is not a grid measurement -- it is a centring that locked onto
    # something that is not the dot. Measured 2026-08-11: A2 carries no mark, so
    # centring settled on the well ring and implied a 9.60 mm step (+6.7%),
    # which was then fitted and stored as if it were the grid.
    kept, rejected = [], []
    for vec, well in col_vecs:
        dev = math.hypot(*vec) / pitch_nom - 1.0
        (kept if abs(dev) <= MAX_PITCH_DEVIATION else rejected).append(
            (vec, well, dev))
    if rejected:
        for _, well, dev in rejected:
            print(f"[cal] REJECTED reference {well}: implied pitch deviates "
                  f"{dev * 100:+.1f}% from {pitch_nom} mm -- not a dot")
    if not kept:
        return None
    col_vecs = [v for v, _, _ in kept]
    col = (sum(v[0] for v in col_vecs) / len(col_vecs),
           sum(v[1] for v in col_vecs) / len(col_vecs))

    nx, ny = (pitch_nom * float(amap["col_sign"]), 0.0) \
        if amap["col_axis"] == "x" else (0.0, pitch_nom * float(amap["col_sign"]))
    rot = math.degrees(math.atan2(col[1], col[0]) - math.atan2(ny, nx))
    rot = (rot + 180.0) % 360.0 - 180.0
    pitch = math.hypot(*col)

    row_vecs = [v for v, _ in row_vecs]
    if row_vecs:
        row = (sum(v[0] for v in row_vecs) / len(row_vecs),
               sum(v[1] for v in row_vecs) / len(row_vecs))
        row_axis = "MEASURED"
        non_orth = abs(90.0 - abs(math.degrees(
            math.atan2(row[1], row[0]) - math.atan2(col[1], col[0]))))
    else:
        row = (0.0, pitch_nom * float(amap["row_sign"])) if amap["row_axis"] == "y" \
            else (pitch_nom * float(amap["row_sign"]), 0.0)
        row_axis = "NOMINAL -- NOT MEASURED"
        non_orth = None
    return {
        "col_step_mm": list(col), "row_step_mm": list(row),
        "col_pitch_mm": pitch, "col_rotation_deg": rot,
        "col_pitch_deviation": pitch / pitch_nom - 1.0,
        "row_axis": row_axis, "non_orthogonality_deg": non_orth,
        "n_col_refs": len(col_vecs), "n_row_refs": len(row_vecs),
    }


def well_pose(cal: dict, row: int, col: int) -> tuple[float, float]:
    """Arm XY for a well, from a stored calibration."""
    o, c, r = cal["a1_centred"], cal["col_step_mm"], cal["row_step_mm"]
    return (o[0] + c[0] * col + r[0] * row, o[1] + c[1] * col + r[1] * row)


def image_wells(wells: Sequence[str], *, live: bool = False, host: str | None = None,
                speed: float = 5.0, settle_s: float = 2.0,
                calibration: str | Path = CALIBRATION_JSON,
                out_dir: str | Path = "/home/lamp/SDLsetup/dataset/captures/wells",
                centred_tol_mm: float = 0.35, log=print) -> list[dict]:
    """Photograph wells through the calibrated grid, checking each frame.

    Every record carries BOTH checks described in the module docstring, and an
    explicit ``verdict``: ``centred`` / ``off_centre`` / ``no_reference``.
    ``no_reference`` means the frame carried nothing measurable -- it is not a
    pass.
    """
    cal = json.loads(Path(calibration).read_text())
    settings = ArmSettings.from_config(**({"live": live} | ({"host": host} if host else {})))
    ws = WorkspaceStore(settings.workspace_path)
    a1p = ws.require_location(WORKSPACE_ANCHOR).pose
    a1 = (float(a1p.x), float(a1p.y), float(a1p.z),
          float(a1p.roll), float(a1p.pitch), float(a1p.yaw))
    J = ps.jacobian_at(ps.load(SCALE_JSON), a1[2])
    transform = adapters.JacobianTransform(J)
    px_per_mm = float(ps.decompose(J)["px_per_mm_mean"])

    plan = [(w, *parse_well(w)) for w in wells]
    for w, r, c in plan:
        x, y = well_pose(cal, r, c)
        log(f"  {w:<4} -> x={x:9.4f} y={y:9.4f}")
    if not live:
        log("[wells] DRY-RUN: nothing moved, nothing captured.")
        return []

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    robot = Arm(settings, log=log)
    robot.connection.connect(arm=True)
    env = Envelope.anchored(a1, name="A1", xy_max_mm=XY_BOX_MM,
                            z_max_rise_mm=0.0, z_min_rise_mm=0.0, orient_tol_deg=0.5)
    guarded = robot.with_envelope(env)
    scope = Microscope(MicroscopeSettings.from_config())
    import time

    out = []
    try:
        for w, r, c in plan:
            x, y = well_pose(cal, r, c)
            legs = adapters.axis_sequential_to(guarded, a1, x, y, speed=speed, label=f"to {w}")
            time.sleep(settle_s)
            cam = adapters.FlatFieldCamera(scope, out_dir, tag=w)
            flat = cam.frame()
            achieved = list(guarded.pose())

            rec: dict[str, Any] = {
                "well": w, "commanded": [x, y], "achieved": achieved[:2],
                "arm_gap_mm": round(math.hypot(achieved[0] - x, achieved[1] - y), 4),
                "image": str(cam.last_path),
                "legs_verified": all(leg.verify()["moved"] is not False for leg in legs),
            }
            ok_dot, info = _dot_in_frame(flat, px_per_mm)
            if ok_dot:
                det = detect.find_dark_dot(flat, max_area_frac=0.5)
                off_px = detect.offset_from_center(det, flat.shape[::-1])
                dx, dy = adapters.offset_mm(transform, off_px)
                d = math.hypot(dx, dy)
                rec.update(reference="dot", offset_mm=[round(dx, 4), round(dy, 4)],
                           offset_norm_mm=round(d, 4),
                           verdict="centred" if d <= centred_tol_mm else "off_centre")
            else:
                lit, frac = adapters.lit_centroid_offset_px(flat)
                if lit is None:
                    rec.update(reference="none", lit_frac=round(frac, 3),
                               verdict="no_reference")
                else:
                    dx, dy = adapters.offset_mm(transform, lit)
                    d = math.hypot(dx, dy)
                    rec.update(reference="lit_aperture (COARSE)",
                               offset_mm=[round(dx, 4), round(dy, 4)],
                               offset_norm_mm=round(d, 4), lit_frac=round(frac, 3),
                               verdict="centred" if d <= centred_tol_mm else "off_centre")
            log(f"[wells] {w}: {rec['verdict']}  ref={rec.get('reference')}  "
                f"offset={rec.get('offset_norm_mm')} mm  arm_gap={rec['arm_gap_mm']} mm")
            out.append(rec)
        (out_dir / "wells.json").write_text(json.dumps(out, indent=1))
        log(f"[wells] {len(out)} well(s) -> {out_dir / 'wells.json'}")
        return out
    finally:
        robot.close()


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--execute", action="store_true")
    p.add_argument("--host")
    p.add_argument("--references", default="A2,B1,A12")
    p.add_argument("--require-focus", action="store_true")
    p.add_argument("--wells", default=None,
                   help="after calibrating, photograph these wells with the "
                        "per-frame centring check (e.g. A1,A6,D5,H12)")
    p.add_argument("--no-store", action="store_true")
    return p


def run(args) -> int:
    report = calibrate_microscope_position_and_focal_point(
        live=args.execute, host=args.host,
        reference_wells=[w.strip() for w in args.references.split(",") if w.strip()],
        require_focus=args.require_focus, store=not args.no_store)
    print()
    print(report.summary())
    if not report.ok:
        return 1
    if args.wells:
        print()
        image_wells([w.strip() for w in args.wells.split(",") if w.strip()],
                    live=args.execute, host=args.host)
    return 0


def main(argv: list[str] | None = None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
