#!/usr/bin/env python3
"""Guards on the three-dot plate frame solve. Pure arithmetic -- no hardware.

Every case is built the same way: TAKE a known rotation/scale/shear, generate
the three dot-centring poses it would produce, solve, and check the solver
hands back what was put in. A solver that agreed with its own generator and
nothing else would pass that, so two of the checks are written independently
of the generator -- ``test_five_degrees_by_hand`` writes the rotated
coordinates out longhand, and the well-offset checks compare against a chord
length computed from trigonometry rather than from the module.

What these protect, worst consequence first:

1. **The yaw sign.** ``yaw_correction_deg`` is added to the wrist angle. With
   the sign backwards the move succeeds, nothing looks wrong, and the seating
   error DOUBLES -- 5 deg becomes 10, which is 4 mm at H12. So the correction
   is applied in the test, both ways, and the doubling is asserted explicitly
   rather than the sign merely being compared to a literal.
2. **Degenerate input never yields a matrix.** Two dots centred on the same
   spot, or three collinear poses, must come back flagged with no inverse and
   no rotation. A fabricated map here steers every later well.
3. **The rotation is real, not naive.** ``well_offset`` must differ from the
   axis-aligned answer by exactly the chord a 5 deg turn subtends. If it
   matches the naive answer, the frame is being solved and then ignored.
4. **Shear is flagged, not corrected.** A real plate's axes are square, so a
   non-orthogonal solve is a bad measurement. It must be reported as such and
   still return its numbers, so the caller -- not this module -- decides.
5. **The axis map is the reference.** Nominal is read from the map, not
   hardcoded, or a remounted plate silently reads as 90 deg out.
"""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tools.calib.plate_frame import (  # noqa: E402
    DEFAULT_AXIS_MAP,
    QUALITY_DEGENERATE,
    QUALITY_OK,
    QUALITY_SUSPECT,
    PlateFrameError,
    solve_plate_frame,
    well_offset,
    yaw_correction_deg,
)

#: A real taught A1 as it reads on the rig (dev/tests/test_goto_a1.py), so the
#: solve is exercised far from the origin where a translation bug would hide.
A1_XY = (26.028557, 590.625244)

PITCH = 9.0
SWAPPED_AXIS_MAP = {"col_axis": "y", "col_sign": 1, "row_axis": "x", "row_sign": -1}


def rotate(v, deg):
    """Turn a vector by deg, positive from +X toward +Y. Written out on purpose."""
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return (v[0] * c - v[1] * s, v[0] * s + v[1] * c)


def unit(axis, sign):
    return (float(sign), 0.0) if axis == "x" else (0.0, float(sign))


def synth(rotation_deg=0.0, scale_col=1.0, scale_row=1.0, shear_deg=0.0,
          origin=A1_XY, pitch=PITCH, axis_map=None):
    """The three centring poses a plate with these faults would produce.

    The shear is put on the ROW axis only, so the solver's mean-of-both-axes
    rotation should come back as rotation_deg + shear_deg / 2 -- which is what
    makes the shear test a check on the averaging convention and not just on
    the flag.
    """
    amap = DEFAULT_AXIS_MAP if axis_map is None else axis_map
    n_col = unit(amap["col_axis"], amap["col_sign"])
    n_row = unit(amap["row_axis"], amap["row_sign"])
    col_vec = rotate((n_col[0] * pitch * scale_col, n_col[1] * pitch * scale_col), rotation_deg)
    row_vec = rotate((n_row[0] * pitch * scale_row, n_row[1] * pitch * scale_row),
                     rotation_deg + shear_deg)
    return {
        "p_black": origin,
        "p_blue": (origin[0] + col_vec[0], origin[1] + col_vec[1]),
        "p_red": (origin[0] + row_vec[0], origin[1] + row_vec[1]),
    }


def naive_offset(row, col, axis_map=None, pitch=PITCH):
    """The axis-aligned answer the repo computes today, ignoring plate rotation."""
    amap = DEFAULT_AXIS_MAP if axis_map is None else axis_map
    n_col = unit(amap["col_axis"], amap["col_sign"])
    n_row = unit(amap["row_axis"], amap["row_sign"])
    return (n_col[0] * pitch * col + n_row[0] * pitch * row,
            n_col[1] * pitch * col + n_row[1] * pitch * row)


class TestRecovery(unittest.TestCase):
    def test_a_square_plate_recovers_the_nominal_grid(self):
        f = solve_plate_frame(**synth(0.0))
        self.assertEqual(f.quality, QUALITY_OK)
        self.assertAlmostEqual(f.rotation_deg, 0.0, places=12)
        self.assertAlmostEqual(f.non_orthogonality_deg, 0.0, places=12)
        self.assertAlmostEqual(f.scale_col, 1.0, places=12)
        self.assertAlmostEqual(f.scale_row, 1.0, places=12)
        self.assertAlmostEqual(f.residual_mm, 0.0, places=12)
        # The nominal map: column step along -X, row step along +Y.
        self.assertAlmostEqual(f.col_vec_mm[0], -9.0, places=12)
        self.assertAlmostEqual(f.col_vec_mm[1], 0.0, places=12)
        self.assertAlmostEqual(f.row_vec_mm[0], 0.0, places=12)
        self.assertAlmostEqual(f.row_vec_mm[1], 9.0, places=12)

    def test_plus_five_degrees(self):
        f = solve_plate_frame(**synth(5.0))
        self.assertAlmostEqual(f.rotation_deg, 5.0, places=10)
        self.assertAlmostEqual(f.non_orthogonality_deg, 0.0, places=10)
        self.assertEqual(f.quality, QUALITY_OK)

    def test_minus_five_degrees(self):
        f = solve_plate_frame(**synth(-5.0))
        self.assertAlmostEqual(f.rotation_deg, -5.0, places=10)
        self.assertAlmostEqual(f.non_orthogonality_deg, 0.0, places=10)
        self.assertEqual(f.quality, QUALITY_OK)

    def test_five_degrees_by_hand(self):
        """Same case, coordinates written out longhand instead of generated.

        Turning the nominal column step (-9, 0) by +5 deg dips it toward -Y;
        turning the nominal row step (0, +9) by +5 deg leans it toward -X. If
        the solver and the generator ever agree on a wrong convention, this is
        the test that disagrees.
        """
        c, s = math.cos(math.radians(5.0)), math.sin(math.radians(5.0))
        p_black = (0.0, 0.0)
        p_blue = (-9.0 * c, -9.0 * s)
        p_red = (-9.0 * s, 9.0 * c)
        self.assertAlmostEqual(p_blue[0], -8.96575, places=4)
        self.assertAlmostEqual(p_blue[1], -0.78440, places=4)
        self.assertAlmostEqual(p_red[0], -0.78440, places=4)
        self.assertAlmostEqual(p_red[1], 8.96575, places=4)

        f = solve_plate_frame(p_black, p_red, p_blue)
        self.assertAlmostEqual(f.rotation_deg, 5.0, places=10)
        self.assertAlmostEqual(yaw_correction_deg(f), -5.0, places=10)

    def test_rotation_with_a_scale_error(self):
        f = solve_plate_frame(**synth(3.0, scale_col=1.02))
        # Scale does not disturb the angles: the rotation must still be exact.
        self.assertAlmostEqual(f.rotation_deg, 3.0, places=10)
        self.assertAlmostEqual(f.scale_col, 1.02, places=10)
        self.assertAlmostEqual(f.scale_row, 1.0, places=10)
        self.assertEqual(f.quality, QUALITY_SUSPECT)
        self.assertIn("column step", f.reason)
        # Still fully usable -- flagged, not withheld.
        self.assertIsNotNone(f.inverse)

    def test_residual_is_the_chord_the_rotation_subtends(self):
        f = solve_plate_frame(**synth(5.0))
        chord = 2.0 * PITCH * math.sin(math.radians(2.5))
        self.assertAlmostEqual(chord, 0.785149, places=6)
        self.assertAlmostEqual(f.residual_col_mm, chord, places=10)
        self.assertAlmostEqual(f.residual_row_mm, chord, places=10)
        self.assertAlmostEqual(f.residual_mm, chord, places=10)

    def test_shear_is_flagged_and_split_between_the_axes(self):
        f = solve_plate_frame(**synth(0.0, shear_deg=4.0))
        self.assertAlmostEqual(f.non_orthogonality_deg, 4.0, places=10)
        self.assertAlmostEqual(f.rotation_deg, 2.0, places=10)
        self.assertEqual(f.quality, QUALITY_SUSPECT)
        self.assertIn("square", f.reason)
        self.assertIsNotNone(f.inverse)

    def test_small_shear_stays_inside_the_flag(self):
        f = solve_plate_frame(**synth(0.0, shear_deg=1.0))
        self.assertEqual(f.quality, QUALITY_OK)

    def test_inverse_round_trips(self):
        f = solve_plate_frame(**synth(7.0, scale_col=1.01, scale_row=0.99))
        m, inv = f.matrix, f.inverse
        for i in range(2):
            for j in range(2):
                got = m[i][0] * inv[0][j] + m[i][1] * inv[1][j]
                self.assertAlmostEqual(got, 1.0 if i == j else 0.0, places=10)

    def test_a_large_rotation_is_flagged(self):
        f = solve_plate_frame(**synth(20.0))
        self.assertEqual(f.quality, QUALITY_SUSPECT)
        self.assertIn("seating rotation", f.reason)


class TestDegenerate(unittest.TestCase):
    def test_collinear_points_refuse_to_produce_a_map(self):
        # Row step nearly parallel to the column step: |sin| = 1.8/81.0 = 0.022.
        black = A1_XY
        f = solve_plate_frame(black,
                              (black[0] - 9.0, black[1] + 0.2),
                              (black[0] - 9.0, black[1]))
        self.assertEqual(f.quality, QUALITY_DEGENERATE)
        self.assertIsNone(f.inverse)
        self.assertIsNone(f.rotation_deg)
        self.assertIn("collinear", f.reason)
        # The evidence for the refusal is kept, not blanked out with it.
        self.assertIsNotNone(f.non_orthogonality_deg)
        self.assertAlmostEqual(f.scale_col, 1.0, places=10)

    def test_the_same_dot_centred_twice_is_degenerate(self):
        black = A1_XY
        f = solve_plate_frame(black, (black[0], black[1] + 9.0), black)
        self.assertEqual(f.quality, QUALITY_DEGENERATE)
        self.assertIsNone(f.inverse)
        self.assertIsNone(f.rotation_deg)
        self.assertIsNone(f.non_orthogonality_deg)
        self.assertIn("centred twice", f.reason)

    def test_a_nonfinite_pose_is_flagged_not_raised(self):
        f = solve_plate_frame(A1_XY, (float("nan"), 5.0), (1.0, 2.0))
        self.assertEqual(f.quality, QUALITY_DEGENERATE)
        self.assertIn("not finite", f.reason)
        self.assertIsNone(f.inverse)

    def test_a_degenerate_frame_will_not_hand_out_offsets_or_corrections(self):
        black = A1_XY
        f = solve_plate_frame(black, black, black)
        with self.assertRaises(PlateFrameError):
            well_offset(f, 3, 4)
        with self.assertRaises(PlateFrameError):
            yaw_correction_deg(f)


class TestWellOffset(unittest.TestCase):
    def test_the_dots_come_back_out(self):
        f = solve_plate_frame(**synth(5.0))
        self.assertAlmostEqual(well_offset(f, 0, 0)[0], 0.0, places=12)
        self.assertAlmostEqual(well_offset(f, 0, 0)[1], 0.0, places=12)
        self.assertAlmostEqual(well_offset(f, 1, 0)[0], f.row_vec_mm[0], places=12)  # B1
        self.assertAlmostEqual(well_offset(f, 1, 0)[1], f.row_vec_mm[1], places=12)
        self.assertAlmostEqual(well_offset(f, 0, 1)[0], f.col_vec_mm[0], places=12)  # A2
        self.assertAlmostEqual(well_offset(f, 0, 1)[1], f.col_vec_mm[1], places=12)

    def test_a_square_plate_matches_the_naive_grid(self):
        f = solve_plate_frame(**synth(0.0))
        for row, col in ((0, 0), (1, 0), (0, 1), (7, 11), (3, 5)):
            got, want = well_offset(f, row, col), naive_offset(row, col)
            self.assertAlmostEqual(got[0], want[0], places=10)
            self.assertAlmostEqual(got[1], want[1], places=10)

    def test_a_rotated_plate_departs_from_naive_by_the_chord(self):
        """|R(t)v - v| = 2|v| sin(t/2). At H12 a 5 deg seating error is 10.2 mm."""
        f = solve_plate_frame(**synth(5.0))
        row, col = 7, 11
        naive = naive_offset(row, col)
        got = well_offset(f, row, col)

        # It must be the naive answer TURNED, not the naive answer.
        turned = rotate(naive, 5.0)
        self.assertAlmostEqual(got[0], turned[0], places=9)
        self.assertAlmostEqual(got[1], turned[1], places=9)

        miss = math.hypot(got[0] - naive[0], got[1] - naive[1])
        chord = 2.0 * math.hypot(naive[0], naive[1]) * math.sin(math.radians(2.5))
        self.assertAlmostEqual(miss, chord, places=9)
        self.assertAlmostEqual(chord, 10.237, places=3)
        # Sanity on the premise: that is more than a well pitch out.
        self.assertGreater(miss, PITCH)

    def test_index_convention_is_enforced(self):
        f = solve_plate_frame(**synth(0.0))
        for row, col in ((-1, 0), (8, 0), (0, -1), (0, 12), (1.5, 0), (0, "3")):
            with self.assertRaises(PlateFrameError):
                well_offset(f, row, col)

    def test_a_suspect_frame_still_computes(self):
        f = solve_plate_frame(**synth(0.0, shear_deg=4.0))
        self.assertEqual(f.quality, QUALITY_SUSPECT)
        self.assertEqual(len(well_offset(f, 7, 11)), 2)


class TestYawCorrection(unittest.TestCase):
    def test_the_correction_opposes_the_rotation_in_both_directions(self):
        self.assertAlmostEqual(yaw_correction_deg(solve_plate_frame(**synth(5.0))),
                               -5.0, places=10)
        self.assertAlmostEqual(yaw_correction_deg(solve_plate_frame(**synth(-5.0))),
                               5.0, places=10)

    def test_applying_it_removes_the_error_and_reversing_it_doubles_it(self):
        """The failure this exists for: a backwards sign is silent and doubles.

        Turning the plate turns both measured step vectors by the same angle,
        so applying a correction is modelled by re-solving from poses whose
        step vectors have been turned. Correct sign -> 0 deg left. Reversed
        sign -> 10 deg, twice the original error.
        """
        for start in (5.0, -5.0, 1.7):
            f = solve_plate_frame(**synth(start))
            correction = yaw_correction_deg(f)

            after = solve_plate_frame(**self._turned(f, correction))
            self.assertAlmostEqual(after.rotation_deg, 0.0, places=9,
                                   msg="correction did not square the plate")

            reversed_ = solve_plate_frame(**self._turned(f, -correction))
            self.assertAlmostEqual(reversed_.rotation_deg, 2.0 * start, places=9,
                                   msg="reversed sign must double the error, not remove it")
            self.assertGreater(abs(reversed_.rotation_deg), abs(f.rotation_deg))

    @staticmethod
    def _turned(frame, deg):
        """The three poses that would be recorded after turning the plate by deg."""
        black = frame.p_black
        col_vec = rotate(frame.col_vec_mm, deg)
        row_vec = rotate(frame.row_vec_mm, deg)
        return {
            "p_black": black,
            "p_blue": (black[0] + col_vec[0], black[1] + col_vec[1]),
            "p_red": (black[0] + row_vec[0], black[1] + row_vec[1]),
        }


class TestAxisMap(unittest.TestCase):
    def test_the_nominal_reference_comes_from_the_map(self):
        points = synth(3.0, axis_map=SWAPPED_AXIS_MAP)
        f = solve_plate_frame(axis_map=SWAPPED_AXIS_MAP, **points)
        self.assertAlmostEqual(f.rotation_deg, 3.0, places=10)
        self.assertEqual(f.quality, QUALITY_OK)

    def test_the_wrong_map_is_caught_as_a_non_square_solve(self):
        """A mismatched map mirrors the grid; the flag must not read as ok."""
        points = synth(3.0, axis_map=SWAPPED_AXIS_MAP)
        f = solve_plate_frame(**points)
        self.assertEqual(f.quality, QUALITY_SUSPECT)
        self.assertGreater(abs(f.non_orthogonality_deg), 90.0)

    def test_extra_keys_are_ignored(self):
        amap = dict(DEFAULT_AXIS_MAP)
        amap["calibrated"] = True
        f = solve_plate_frame(axis_map=amap, **synth(0.0))
        self.assertEqual(f.quality, QUALITY_OK)

    def test_malformed_input_raises_instead_of_flagging(self):
        pts = synth(0.0)
        bad_maps = [
            {"col_axis": "x", "col_sign": -1, "row_axis": "x", "row_sign": 1},
            {"col_axis": "z", "col_sign": -1, "row_axis": "y", "row_sign": 1},
            {"col_axis": "x", "col_sign": 0, "row_axis": "y", "row_sign": 1},
            {"col_axis": "x", "col_sign": -1},
        ]
        for amap in bad_maps:
            with self.assertRaises(PlateFrameError):
                solve_plate_frame(axis_map=amap, **pts)
        for pitch in (0.0, -9.0, float("nan")):
            with self.assertRaises(PlateFrameError):
                solve_plate_frame(pitch_mm=pitch, **pts)
        with self.assertRaises(PlateFrameError):
            solve_plate_frame(A1_XY, (1.0,), (2.0, 3.0))

    def test_a_full_six_dof_pose_is_accepted(self):
        """The arm hands back [x, y, z, roll, pitch, yaw]; taking .pose[:2] at every
        call site is one more place to drop a coordinate."""
        pts = synth(5.0)
        posed = {k: list(v) + [177.245712, 179.995, -0.002, 89.745] for k, v in pts.items()}
        f = solve_plate_frame(**posed)
        self.assertAlmostEqual(f.rotation_deg, 5.0, places=10)


if __name__ == "__main__":
    unittest.main(verbosity=2)
