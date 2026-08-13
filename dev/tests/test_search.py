#!/usr/bin/env python3
"""The dot search: coverage, ordering, and the two signs in a sighting.

Pure geometry -- no camera, no arm, no files. What these protect, worst
consequence first:

1. **A step that steps over the dot.** The field is 2.84 x 2.38 mm and the well
   pitch is 9.0 mm, so a step chosen by feel rather than derived from the field
   walks past the dot and reports nothing there. The simulator test is the one
   that matters: it drops a dot at 441 positions across the whole search box and
   requires the plan to image it whole at some point. Its negative control --
   the same sweep with a dot larger than the stated guarantee -- must MISS, or
   the test is passing for the wrong reason.
2. **A coverage claim that cannot fail.** ``coverage_gap_mm`` is asserted <= 0
   on the default plan, and asserted > 0 on a hand-built plan whose spacing
   exceeds the field. A metric that only ever returns "fine" verifies nothing.
3. **The 9 mm sign.** Seeing red means the camera is over B1, one row at +y from
   A1, so the step back is -y; seeing blue means A2, one column at -x, so the
   step back is +x. Getting either backwards drives the arm 9 mm the wrong way.
   The expected step is derived here from the FORWARD grid arithmetic that
   ``scripts/tool_building/plate_imaging.py`` applies -- position the well, then
   subtract -- rather than by replaying the module's own negation, and it is
   also pinned to literals for the calibrated axis_map so a matched pair of sign
   flips cannot hide.
4. **An axis_map that is ignored.** The same three colours are resolved against a
   flipped and a transposed axis_map and must produce different steps; a
   function returning hardcoded (0, -9) would pass item 3 and fail here.

    python dev/tests/test_search.py
"""
from __future__ import annotations

import itertools
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tools.calib import search  # noqa: E402

#: The measured field at A1 + 3.0 mm: 2448 x 2048 px at 861 px/mm.
FOV = search.FOV_MM_AT_A1_PLUS_3MM
PX_PER_MM = search.PX_PER_MM_AT_A1_PLUS_3MM

#: The seating error the plan is sized for in most of these tests.
MAX_OFFSET = 3.0

#: The operator-calibrated grid, restated here rather than imported so a change
#: to the module default shows up as a test failure instead of silently
#: redefining what the tests check.
CALIBRATED = {"col_axis": "x", "col_sign": -1, "row_axis": "y", "row_sign": 1}


def default_plan(overlap: float = 0.15, max_offset_mm: float = MAX_OFFSET) -> list:
    return search.scan_plan(fov_mm=FOV, max_offset_mm=max_offset_mm, overlap=overlap)


def unique_axis_values(plan: list) -> tuple[list, list]:
    xs = sorted({p.dx_mm for p in plan})
    ys = sorted({p.dy_mm for p in plan})
    return xs, ys


def first_hit(plan: list, dot_mm: tuple, fov_mm: tuple, dot_diameter_mm: float):
    """Index of the first scan point that images the WHOLE dot, or None.

    The tiny simulator the coverage claim is checked against. A dot of diameter
    d centred at ``dot_mm`` is entirely inside the frame at a scan point exactly
    when its centre lies within the frame shrunk by d/2 on every side.
    """
    half_w = (fov_mm[0] - dot_diameter_mm) / 2.0
    half_h = (fov_mm[1] - dot_diameter_mm) / 2.0
    for point in plan:
        if abs(dot_mm[0] - point.dx_mm) <= half_w and abs(dot_mm[1] - point.dy_mm) <= half_h:
            return point.index
    return None


def candidate_dots(max_offset_mm: float = MAX_OFFSET, n: int = 21) -> list:
    """An n x n grid of dot positions spanning the whole search box."""
    span = [-max_offset_mm + 2.0 * max_offset_mm * i / (n - 1) for i in range(n)]
    return list(itertools.product(span, span))


def well_xy(axis_map: dict, row_index: int, col_index: int, pitch_mm: float) -> tuple:
    """Arm (x, y) of a well with A1 at the origin -- the FORWARD grid arithmetic.

    This mirrors what ``plate_imaging.pose_for_well`` does to reach a well; the
    move back to A1 is then just its negation. Deriving the expectation this way
    keeps the test from being a restatement of the implementation.
    """
    pos = {"x": 0.0, "y": 0.0}
    pos[axis_map["col_axis"]] += axis_map["col_sign"] * pitch_mm * col_index
    pos[axis_map["row_axis"]] += axis_map["row_sign"] * pitch_mm * row_index
    return (pos["x"], pos["y"])


class TestStepDerivation(unittest.TestCase):
    def test_step_never_exceeds_fov_times_one_minus_overlap(self):
        for overlap in (0.0, 0.15, 0.3, 0.5, 0.75):
            for fov in (FOV, 1.0, (4.0, 1.5)):
                fov_x, fov_y = (fov, fov) if isinstance(fov, float) else fov
                plan = search.scan_plan(fov_mm=fov, max_offset_mm=MAX_OFFSET, overlap=overlap)
                xs, ys = unique_axis_values(plan)
                for values, fov_axis in ((xs, fov_x), (ys, fov_y)):
                    # Indexed rather than zipped: zip(strict=) is 3.10+ and the
                    # repo runs this file on older interpreters too.
                    for i in range(len(values) - 1):
                        self.assertLessEqual(values[i + 1] - values[i],
                                             fov_axis * (1.0 - overlap) + 1e-9,
                                             "overlap=%r fov=%r" % (overlap, fov))

    def test_step_is_derived_from_the_field_not_hardcoded(self):
        # Halving the field must halve the step; a literal step would not move.
        wide = default_plan()
        narrow = search.scan_plan(fov_mm=(FOV[0] / 2.0, FOV[1] / 2.0),
                                  max_offset_mm=MAX_OFFSET, overlap=0.15)
        self.assertAlmostEqual(unique_axis_values(wide)[0][1] - unique_axis_values(wide)[0][0],
                               2.0 * (unique_axis_values(narrow)[0][1]
                                      - unique_axis_values(narrow)[0][0]), places=9)

    def test_malformed_inputs_refused(self):
        for kwargs in ({"overlap": 1.0}, {"overlap": -0.1}, {"overlap": float("nan")},
                       {"fov_mm": 0.0}, {"fov_mm": -1.0}, {"fov_mm": (2.0, float("inf"))},
                       {"max_offset_mm": -1.0}, {"max_offset_mm": float("nan")},
                       {"pitch_mm": 0.0}):
            call = {"fov_mm": FOV, "max_offset_mm": MAX_OFFSET}
            call.update(kwargs)
            with self.assertRaises(search.SearchError, msg=repr(kwargs)):
                search.scan_plan(**call)

    def test_a_search_wider_than_one_pitch_is_refused_but_the_ceiling_is_the_callers(self):
        with self.assertRaises(search.SearchError):
            search.scan_plan(fov_mm=FOV, max_offset_mm=9.5)
        # Raising pitch_mm explicitly is how a caller overrides it.
        plan = search.scan_plan(fov_mm=FOV, max_offset_mm=9.5, pitch_mm=18.0)
        self.assertGreater(len(plan), 0)


class TestCoverage(unittest.TestCase):
    def test_default_plan_leaves_no_gap(self):
        self.assertLessEqual(search.coverage_gap_mm(default_plan(), FOV), 0.0)

    def test_gap_equals_the_overlap_on_the_short_axis(self):
        # gap = max over axes of (step - fov) = -overlap * min(fov). Pinning the
        # value, not just its sign, catches a plan that over-covers by accident.
        for overlap in (0.05, 0.15, 0.4):
            gap = search.coverage_gap_mm(default_plan(overlap=overlap), FOV)
            self.assertAlmostEqual(gap, -overlap * min(FOV), places=9)

    def test_the_whole_dot_guarantee_holds_at_the_stated_size(self):
        # Substituting fov - d checks the stronger claim (module eq. 1).
        overlap = 0.15
        d_max = overlap * min(FOV)
        plan = default_plan(overlap=overlap)
        shrunk = (FOV[0] - d_max, FOV[1] - d_max)
        self.assertLessEqual(search.coverage_gap_mm(plan, shrunk), 1e-9)

    def test_a_dot_larger_than_the_guarantee_is_reported_as_a_gap(self):
        overlap = 0.15
        too_big = 2.0 * overlap * min(FOV)
        plan = default_plan(overlap=overlap)
        shrunk = (FOV[0] - too_big, FOV[1] - too_big)
        self.assertGreater(search.coverage_gap_mm(plan, shrunk), 0.0)

    def test_a_plan_whose_spacing_exceeds_the_field_reports_a_positive_gap(self):
        # Hand-built lattice at 5 mm spacing under a 2.84 mm field: two blind
        # stripes per row. If coverage_gap_mm cannot report this it reports
        # nothing.
        pts = []
        for iy in (-1, 0, 1):
            for ix in (-1, 0, 1):
                pts.append(search.ScanPoint(index=len(pts), ring=max(abs(ix), abs(iy)),
                                            ix=ix, iy=iy, dx_mm=5.0 * ix, dy_mm=5.0 * iy,
                                            radius_mm=math.hypot(5.0 * ix, 5.0 * iy)))
        self.assertAlmostEqual(search.coverage_gap_mm(pts, FOV), 5.0 - FOV[1], places=9)

    def test_a_plan_with_a_hole_is_refused_rather_than_scored(self):
        plan = default_plan()
        holed = [p for p in plan if not (p.ix == 1 and p.iy == 1)]
        with self.assertRaises(search.SearchError):
            search.coverage_gap_mm(holed, FOV)

    def test_an_empty_plan_is_refused(self):
        with self.assertRaises(search.SearchError):
            search.coverage_gap_mm([], FOV)

    def test_a_single_point_plan_is_one_unbroken_strip(self):
        plan = search.scan_plan(fov_mm=FOV, max_offset_mm=0.0)
        self.assertEqual(len(plan), 1)
        self.assertAlmostEqual(search.coverage_gap_mm(plan, FOV), -min(FOV), places=9)


class TestOrdering(unittest.TestCase):
    def test_the_plan_is_finite_and_starts_at_the_nominal_pose(self):
        plan = default_plan()
        self.assertTrue(0 < len(plan) < 10000)
        self.assertEqual(plan[0].ring, 0)
        self.assertEqual((plan[0].dx_mm, plan[0].dy_mm), (0.0, 0.0))
        self.assertEqual([p.index for p in plan], list(range(len(plan))))

    def test_ring_never_decreases(self):
        for overlap in (0.0, 0.15, 0.5):
            for max_offset in (0.0, 1.0, 3.0, 8.0):
                plan = default_plan(overlap=overlap, max_offset_mm=max_offset)
                rings = [p.ring for p in plan]
                self.assertEqual(rings, sorted(rings))

    def test_ring_is_the_chebyshev_grid_distance(self):
        for p in default_plan():
            self.assertEqual(p.ring, max(abs(p.ix), abs(p.iy)))

    def test_offsets_are_the_grid_indices_times_the_derived_step(self):
        plan = default_plan()
        step_x = FOV[0] * 0.85
        step_y = FOV[1] * 0.85
        for p in plan:
            self.assertAlmostEqual(p.dx_mm, p.ix * step_x, places=9)
            self.assertAlmostEqual(p.dy_mm, p.iy * step_y, places=9)
            self.assertAlmostEqual(p.radius_mm, math.hypot(p.dx_mm, p.dy_mm), places=9)

    def test_the_footprint_reaches_max_offset_on_both_axes(self):
        for max_offset in (0.5, 3.0, 8.9):
            plan = default_plan(max_offset_mm=max_offset)
            self.assertGreaterEqual(max(p.dx_mm for p in plan), max_offset - 1e-9)
            self.assertGreaterEqual(max(p.dy_mm for p in plan), max_offset - 1e-9)

    def test_points_are_unique(self):
        plan = default_plan(max_offset_mm=8.0)
        self.assertEqual(len({(p.ix, p.iy) for p in plan}), len(plan))


class TestSimulatedSearch(unittest.TestCase):
    """Walk the plan against a simulated dot. 441 positions per sweep."""

    def test_a_dot_anywhere_within_max_offset_is_found(self):
        overlap = 0.15
        plan = default_plan(overlap=overlap)
        # Just inside the stated guarantee, to stay off the float knife-edge
        # where the shrunk windows abut exactly.
        diameter = 0.9 * overlap * min(FOV)
        dots = candidate_dots()
        self.assertGreaterEqual(len(dots), 200)
        missed = [d for d in dots if first_hit(plan, d, FOV, diameter) is None]
        self.assertEqual(missed, [], "%d of %d dot positions were missed" % (len(missed),
                                                                            len(dots)))

    def test_the_sweep_holds_across_overlaps_and_search_sizes(self):
        for overlap in (0.1, 0.15, 0.3, 0.5):
            for max_offset in (1.0, 3.0, 8.0):
                plan = default_plan(overlap=overlap, max_offset_mm=max_offset)
                diameter = 0.9 * overlap * min(FOV)
                missed = [d for d in candidate_dots(max_offset)
                          if first_hit(plan, d, FOV, diameter) is None]
                self.assertEqual(missed, [], "overlap=%r max_offset=%r missed %d"
                                             % (overlap, max_offset, len(missed)))

    def test_the_simulator_can_report_a_miss(self):
        # Negative control. A dot twice the guaranteed size must fall through
        # the plan somewhere, or the sweep above proves nothing.
        overlap = 0.15
        plan = default_plan(overlap=overlap)
        diameter = 2.0 * overlap * min(FOV)
        missed = [d for d in candidate_dots() if first_hit(plan, d, FOV, diameter) is None]
        self.assertGreater(len(missed), 0)

    def test_a_dot_at_the_nominal_pose_is_found_first(self):
        plan = default_plan()
        self.assertEqual(first_hit(plan, (0.0, 0.0), FOV, 0.2), 0)


class TestArmAlignedFov(unittest.TestCase):
    def test_zero_rotation_costs_nothing(self):
        got = search.arm_aligned_fov_mm(FOV, 0.0)
        self.assertAlmostEqual(got[0], FOV[0], places=9)
        self.assertAlmostEqual(got[1], FOV[1], places=9)

    def test_rotation_shrinks_the_field(self):
        got = search.arm_aligned_fov_mm(FOV, 5.0)
        self.assertLess(got[0], FOV[0])
        self.assertLess(got[1], FOV[1])

    def test_the_result_really_fits_inside_the_rotated_field(self):
        for theta in (0.0, 1.0, 3.0, 7.5, -12.0):
            a, b = search.arm_aligned_fov_mm(FOV, theta)
            c, s = math.cos(math.radians(-theta)), math.sin(math.radians(-theta))
            for sx, sy in itertools.product((-1, 1), (-1, 1)):
                px, py = sx * a / 2.0, sy * b / 2.0
                u = c * px - s * py
                v = s * px + c * py
                self.assertLessEqual(abs(u), FOV[0] / 2.0 + 1e-9, "theta=%r" % theta)
                self.assertLessEqual(abs(v), FOV[1] / 2.0 + 1e-9, "theta=%r" % theta)


class TestResolveFromSighting(unittest.TestCase):
    def test_black_takes_no_well_step(self):
        got = search.resolve_from_sighting("black", (0.0, 0.0), PX_PER_MM, axis_map=CALIBRATED)
        self.assertEqual(got["well"], "A1")
        self.assertEqual(got["well_step_mm"], (0.0, 0.0))
        self.assertEqual(got["step_residual_mm_per_deg"], 0.0)

    def test_red_means_B1_and_steps_one_row_back(self):
        got = search.resolve_from_sighting("red", (0.0, 0.0), PX_PER_MM, axis_map=CALIBRATED)
        self.assertEqual(got["well"], "B1")
        self.assertEqual((got["row_index"], got["col_index"]), (1, 0))
        # row_axis="y", row_sign=+1 -> B1 is at +9 in y -> the way back is -9.
        self.assertAlmostEqual(got["well_step_mm"][0], 0.0, places=9)
        self.assertAlmostEqual(got["well_step_mm"][1], -9.0, places=9)

    def test_blue_means_A2_and_steps_one_column_back(self):
        got = search.resolve_from_sighting("blue", (0.0, 0.0), PX_PER_MM, axis_map=CALIBRATED)
        self.assertEqual(got["well"], "A2")
        self.assertEqual((got["row_index"], got["col_index"]), (0, 1))
        # col_axis="x", col_sign=-1 -> A2 is at -9 in x -> the way back is +9.
        self.assertAlmostEqual(got["well_step_mm"][0], 9.0, places=9)
        self.assertAlmostEqual(got["well_step_mm"][1], 0.0, places=9)

    def test_the_step_is_the_negation_of_the_forward_grid_position(self):
        for axis_map in (CALIBRATED,
                         {"col_axis": "x", "col_sign": 1, "row_axis": "y", "row_sign": 1},
                         {"col_axis": "x", "col_sign": -1, "row_axis": "y", "row_sign": -1},
                         {"col_axis": "y", "col_sign": -1, "row_axis": "x", "row_sign": 1},
                         {"col_axis": "y", "col_sign": 1, "row_axis": "x", "row_sign": -1}):
            for colour in ("black", "red", "blue"):
                got = search.resolve_from_sighting(colour, (0.0, 0.0), PX_PER_MM,
                                                   axis_map=axis_map)
                wx, wy = well_xy(axis_map, got["row_index"], got["col_index"], 9.0)
                self.assertAlmostEqual(got["well_step_mm"][0], -wx, places=9,
                                       msg="%s %r" % (colour, axis_map))
                self.assertAlmostEqual(got["well_step_mm"][1], -wy, places=9,
                                       msg="%s %r" % (colour, axis_map))

    def test_a_flipped_column_sign_flips_the_blue_step(self):
        flipped = dict(CALIBRATED, col_sign=1)
        got = search.resolve_from_sighting("blue", (0.0, 0.0), PX_PER_MM, axis_map=flipped)
        self.assertAlmostEqual(got["well_step_mm"][0], -9.0, places=9)

    def test_a_transposed_axis_map_moves_the_step_to_the_other_axis(self):
        transposed = {"col_axis": "y", "col_sign": -1, "row_axis": "x", "row_sign": 1}
        red = search.resolve_from_sighting("red", (0.0, 0.0), PX_PER_MM, axis_map=transposed)
        blue = search.resolve_from_sighting("blue", (0.0, 0.0), PX_PER_MM, axis_map=transposed)
        self.assertAlmostEqual(red["well_step_mm"][0], -9.0, places=9)
        self.assertAlmostEqual(red["well_step_mm"][1], 0.0, places=9)
        self.assertAlmostEqual(blue["well_step_mm"][0], 0.0, places=9)
        self.assertAlmostEqual(blue["well_step_mm"][1], 9.0, places=9)

    def test_the_module_default_axis_map_is_the_calibrated_one(self):
        implicit = search.resolve_from_sighting("red", (0.0, 0.0), PX_PER_MM)
        explicit = search.resolve_from_sighting("red", (0.0, 0.0), PX_PER_MM,
                                                axis_map=CALIBRATED)
        self.assertEqual(implicit["well_step_mm"], explicit["well_step_mm"])

    def test_a_non_nominal_pitch_scales_the_step(self):
        got = search.resolve_from_sighting("red", (0.0, 0.0), PX_PER_MM, pitch_mm=4.5,
                                           axis_map=CALIBRATED)
        self.assertAlmostEqual(got["well_step_mm"][1], -4.5, places=9)

    def test_centring_cancels_the_pixel_offset(self):
        # 861 px right of centre is 1.00 mm right; with arm +X driving image u
        # positive, the arm must go -1.00 mm to bring it back.
        got = search.resolve_from_sighting("black", (PX_PER_MM, -2.0 * PX_PER_MM), PX_PER_MM,
                                           axis_map=CALIBRATED)
        sign_u, sign_v = search.IMAGE_AXES_TO_ARM
        self.assertAlmostEqual(got["centering_mm"][0], -1.0 * sign_u, places=9)
        self.assertAlmostEqual(got["centering_mm"][1], 2.0 * sign_v, places=9)

    def test_an_anisotropic_scale_is_applied_per_axis(self):
        got = search.resolve_from_sighting("black", (400.0, 400.0), (800.0, 200.0),
                                           axis_map=CALIBRATED)
        sign_u, sign_v = search.IMAGE_AXES_TO_ARM
        self.assertAlmostEqual(got["centering_mm"][0], -0.5 * sign_u, places=9)
        self.assertAlmostEqual(got["centering_mm"][1], -2.0 * sign_v, places=9)

    def test_the_move_is_the_sum_of_its_two_halves(self):
        for colour in ("black", "red", "blue"):
            got = search.resolve_from_sighting(colour, (300.0, -120.0), PX_PER_MM,
                                               axis_map=CALIBRATED)
            self.assertAlmostEqual(got["move_mm"][0],
                                   got["centering_mm"][0] + got["well_step_mm"][0], places=9)
            self.assertAlmostEqual(got["move_mm"][1],
                                   got["centering_mm"][1] + got["well_step_mm"][1], places=9)

    def test_the_rotation_residual_is_reported_for_a_stepped_sighting(self):
        got = search.resolve_from_sighting("red", (0.0, 0.0), PX_PER_MM, axis_map=CALIBRATED)
        self.assertAlmostEqual(got["step_residual_mm_per_deg"],
                               9.0 * math.sin(math.radians(1.0)), places=9)
        # 0.157 mm/deg: in frame (the field is 2.38 mm) but nowhere near centred.
        self.assertLess(got["step_residual_mm_per_deg"], min(FOV) / 2.0)

    def test_inputs_used_are_echoed_so_a_logged_result_is_re_derivable(self):
        got = search.resolve_from_sighting("BLUE ", (1.0, 2.0), (800.0, 900.0), pitch_mm=9.0,
                                           axis_map=CALIBRATED)
        self.assertEqual(got["colour"], "blue")
        self.assertEqual(got["px_per_mm"], (800.0, 900.0))
        self.assertEqual(got["pitch_mm"], 9.0)
        self.assertEqual(got["axis_map"], CALIBRATED)
        self.assertEqual(got["image_axes_to_arm"], tuple(search.IMAGE_AXES_TO_ARM))
        self.assertTrue(got["assumptions"])

    def test_bad_inputs_refused(self):
        base = ("red", (0.0, 0.0), PX_PER_MM)
        with self.assertRaises(search.SearchError):
            search.resolve_from_sighting("green", *base[1:])
        with self.assertRaises(search.SearchError):
            search.resolve_from_sighting("red", (float("nan"), 0.0), PX_PER_MM)
        with self.assertRaises(search.SearchError):
            search.resolve_from_sighting("red", (0.0, 0.0), 0.0)
        with self.assertRaises(search.SearchError):
            search.resolve_from_sighting("red", (0.0, 0.0), PX_PER_MM, pitch_mm=-1.0)

    def test_an_axis_map_that_would_lose_an_axis_is_refused(self):
        for bad in ({"col_axis": "x", "col_sign": -1, "row_axis": "x", "row_sign": 1},
                    {"col_axis": "z", "col_sign": -1, "row_axis": "y", "row_sign": 1},
                    {"col_axis": "x", "col_sign": 0, "row_axis": "y", "row_sign": 1},
                    {"col_axis": "x", "col_sign": -2, "row_axis": "y", "row_sign": 1},
                    {"col_axis": "x", "col_sign": -1, "row_axis": "y"}):
            with self.assertRaises(search.SearchError, msg=repr(bad)):
                search.resolve_from_sighting("red", (0.0, 0.0), PX_PER_MM, axis_map=bad)


if __name__ == "__main__":
    unittest.main(verbosity=2)
