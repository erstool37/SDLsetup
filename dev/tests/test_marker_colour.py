"""
test_marker_colour.py -- prove the marker detector says WHICH colour it found.

NO HARDWARE, NO NETWORK. Every frame here is synthesised with numpy and written
with PIL, so the whole suite runs on a laptop with the rig switched off.

    ~/.pyenv/versions/agent/bin/python dev/tests/test_marker_colour.py
    ~/.pyenv/versions/agent/bin/python -m pytest dev/tests/test_marker_colour.py -q

THE BUG THIS SUITE EXISTS FOR
-----------------------------
`detect(mode="dark")` fires on any dark region inside the illuminated hull, and
a saturated red or blue dot IS dark in greyscale (76/255 and 29/255 against a
lit field near 200). So before `classify_dots`, landing on well B1 and seeing
the RED dot returned a confident, well-formed, quality="ok" report of "the black
mark" -- and the centring loop then converged neatly onto a well 9.0 mm from the
one it named. `test_a_red_dot_is_red_and_is_not_black` is that regression; the
rest of the suite exists to stop it being fixed by breaking something else.

WHY THE FRAMES ARE BUILT IN RGB AND SAVED WITH PIL
--------------------------------------------------
PIL writes RGB; OpenCV reads BGR. Building the fixtures through the opposite
channel order from the one the detector uses means a red/blue channel swap
inside marker.py cannot pass these tests -- the red and blue cases would trade
places. A fixture built with cv2 in BGR would hide exactly that class of error.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


import shutil
import tempfile
import unittest

import numpy as np
from PIL import Image

from tools.microscope import marker

#: Frame size. Not the rig's 2448x2048 -- a third of the linear size keeps the
#: suite quick while staying far above every area gate in the detector.
FRAME_W, FRAME_H = 1200, 900

#: The lamp lights a disc smaller than the sensor, which is the whole reason the
#: dark discriminant works on the hull rather than on a global threshold.
LIT_RADIUS = 380

#: Dot radius. Area 9503 px = 0.9 % of the frame, comfortably inside the
#: detector's [0.2 %, 35 %] gate even after the colour mask erodes it.
DOT_RADIUS = 55

#: Fixture inks, RGB. Saturated on purpose: a real printed dot is, and the
#: greyscale values they collapse to (76 for red, 29 for blue, against a lit
#: field of 200) are what makes the dark discriminant fire on all three.
LIT_GREY = (200, 200, 200)
BLACK_INK = (12, 12, 12)
RED_INK = (230, 0, 0)
BLUE_INK = (0, 0, 230)

#: A faint warm smudge, not a dot: 40 counts of raw red excess, of which the
#: broad-blur subtraction leaves 27 -- measured here, against a floor of 40, so
#: margin = -13. Chosen off a measured ladder on this fixture (raw excess 30 ->
#: margin -20, 40 -> -13, 50 -> -7, 60 -> 0.0, saturated -> +113): 60 sits
#: exactly ON the floor, which would make the fixture a coin toss.
PINK_SMUDGE = (230, 190, 190)

#: Offset used for the sign tests. Kept inside LIT_RADIUS - DOT_RADIUS so the
#: dot stays fully lit and the centroid is unbiased.
OFF_X, OFF_Y = 200, 130


def render_frame(path, dot=None, *, noise_sigma=0.0, seed=0):
    """An illuminated disc on an unlit field, optionally carrying one dot.

    `dot` is (dx, dy, rgb): the dot's centre as an offset from the FRAME centre,
    which is what the detector reports offsets against, so a test states its
    expectation once instead of converting between two origins.
    """
    yy, xx = np.mgrid[0:FRAME_H, 0:FRAME_W].astype(np.float64)
    cx, cy = FRAME_W / 2.0, FRAME_H / 2.0
    img = np.zeros((FRAME_H, FRAME_W, 3), np.float64)
    img[(xx - cx) ** 2 + (yy - cy) ** 2 <= LIT_RADIUS ** 2] = LIT_GREY
    if dot is not None:
        dx, dy, rgb = dot
        img[(xx - (cx + dx)) ** 2 + (yy - (cy + dy)) ** 2 <= DOT_RADIUS ** 2] = rgb
    if noise_sigma:
        img += np.random.default_rng(seed).normal(0.0, noise_sigma, img.shape)
    # No mode argument: Pillow deprecated it and drops it in Pillow 13
    # (2026-10-15). An (H, W, 3) uint8 array already infers "RGB".
    Image.fromarray(np.clip(img, 0, 255).astype(np.uint8)).save(str(path))
    return Path(path)


class MarkerColourTest(unittest.TestCase):
    """One tmp dir of fixtures for the whole class; nothing here mutates them."""

    @classmethod
    def setUpClass(cls):
        cls.tmp = Path(tempfile.mkdtemp(prefix="marker-colour-"))
        cls.black = render_frame(cls.tmp / "black.png", (0, 0, BLACK_INK))
        cls.red = render_frame(cls.tmp / "red.png", (0, 0, RED_INK))
        cls.blue = render_frame(cls.tmp / "blue.png", (0, 0, BLUE_INK))
        cls.blank = render_frame(cls.tmp / "blank.png", None, noise_sigma=4.0)
        cls.smudge = render_frame(cls.tmp / "smudge.png", (0, 0, PINK_SMUDGE))
        cls.down_right = render_frame(cls.tmp / "dr.png", (OFF_X, OFF_Y, BLACK_INK))
        cls.up_left = render_frame(cls.tmp / "ul.png", (-OFF_X, -OFF_Y, BLACK_INK))
        cls.red_off = render_frame(cls.tmp / "red_off.png", (OFF_X, OFF_Y, RED_INK))

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tmp, ignore_errors=True)

    # -- what colour is it -------------------------------------------------

    def test_a_black_dot_is_black_and_is_neither_red_nor_blue(self):
        res = marker.classify_dots(self.black)
        self.assertEqual(res["present"], ["black"], res["note"])
        self.assertTrue(res["colours"]["black"]["found"])
        self.assertFalse(res["colours"]["red"]["found"])
        self.assertFalse(res["colours"]["blue"]["found"])
        self.assertIsNone(res["colours"]["black"]["suppressed_by"])

    def test_a_red_dot_is_red_and_is_not_black(self):
        """THE regression. A red dot read as black is a 9.0 mm one-well error
        that converges and therefore looks exactly like success."""
        res = marker.classify_dots(self.red)
        self.assertEqual(res["present"], ["red"], res["note"])
        self.assertTrue(res["colours"]["red"]["found"])
        self.assertFalse(res["colours"]["black"]["found"])
        self.assertFalse(res["colours"]["blue"]["found"])

    def test_the_dark_discriminant_really_does_fire_on_the_red_dot(self):
        """Guards the test above from passing for the wrong reason. If a future
        change stopped mode="dark" seeing the red dot, the regression test would
        stay green while proving nothing about disambiguation."""
        self.assertEqual(marker.detect(self.red, mode="dark")["n_candidates"], 1)
        res = marker.classify_dots(self.red)
        self.assertTrue(res["colours"]["black"]["discriminant_fired"])
        self.assertEqual(res["colours"]["black"]["suppressed_by"], "red")

    def test_a_blue_dot_is_blue_and_is_not_black(self):
        res = marker.classify_dots(self.blue)
        self.assertEqual(res["present"], ["blue"], res["note"])
        self.assertTrue(res["colours"]["blue"]["found"])
        self.assertFalse(res["colours"]["black"]["found"])
        self.assertFalse(res["colours"]["red"]["found"])
        self.assertEqual(res["colours"]["black"]["suppressed_by"], "blue")

    def test_red_and_blue_do_not_fire_on_each_other(self):
        """Each channel is an excess over the MAXIMUM of the other two, so the
        two discriminants are mutually exclusive by construction."""
        self.assertEqual(marker.detect(self.red, mode="blue")["n_candidates"], 0)
        self.assertEqual(marker.detect(self.blue, mode="red")["n_candidates"], 0)

    # -- nothing there is a normal reading, not an error -------------------

    def test_a_blank_lit_frame_classifies_as_nothing_without_raising(self):
        """The usual reading during a search: ~6 mm of blank plate lies between
        two dots and the field of view is only ~2.84 mm."""
        res = marker.classify_dots(self.blank)
        self.assertEqual(res["present"], [])
        self.assertEqual(res["n_present"], 0)
        self.assertEqual(res["conflicts"], [])
        for colour in ("black", "red", "blue"):
            rec = res["colours"][colour]
            self.assertFalse(rec["found"])
            self.assertIsNone(rec["centroid_px"])
            self.assertIsNone(rec["offset_px"])
            self.assertTrue(rec["reason"])

    def test_an_unreadable_path_raises_rather_than_reporting_an_empty_field(self):
        """A missing file is a caller bug. Reporting it as "no dot here" would
        make a broken capture indistinguishable from blank plate."""
        with self.assertRaises(marker.MarkerError):
            marker.classify_dots(self.tmp / "does-not-exist.png")

    # -- margin ------------------------------------------------------------

    def test_margin_is_positive_for_a_clear_dot_and_negative_for_blank_plate(self):
        clear = marker.classify_dots(self.red)["colours"]["red"]["margin"]
        empty = marker.classify_dots(self.blank)["colours"]["red"]["margin"]
        self.assertGreater(clear, 0.0)
        self.assertLess(empty, 0.0)

    def test_margin_orders_blank_plate_below_a_smudge_below_a_real_dot(self):
        """The point of a margin: "barely missed", "nothing there", and "clearly
        red" are three different frames, and a bare found=False collapses them
        into one. The floor falls between the smudge and the dot."""
        blank = marker.classify_dots(self.blank)["colours"]["red"]["margin"]
        smudge = marker.classify_dots(self.smudge)
        clear = marker.classify_dots(self.red)["colours"]["red"]["margin"]
        self.assertLess(blank, smudge["colours"]["red"]["margin"])
        self.assertLess(smudge["colours"]["red"]["margin"], 0.0)
        self.assertLess(0.0, clear)
        # and a smudge that misses the floor is reported as no dot at all,
        # not as a weak red one
        self.assertEqual(smudge["present"], [], smudge["note"])

    def test_the_dark_margin_is_reported_in_grey_levels_below_the_otsu_split(self):
        rec = marker.classify_dots(self.black)["colours"]["black"]
        self.assertGreater(rec["margin"], 0.0)
        self.assertIn("grey levels", rec["margin_units"])

    # -- offsets -----------------------------------------------------------

    def test_a_dot_below_and_right_of_centre_reports_two_positive_offsets(self):
        """Image convention: +x right, +y down, offset = centroid - centre."""
        rec = marker.classify_dots(self.down_right)["colours"]["black"]
        self.assertTrue(rec["found"])
        dx, dy = rec["offset_px"]
        self.assertGreater(dx, 0.0)
        self.assertGreater(dy, 0.0)
        self.assertAlmostEqual(dx, OFF_X, delta=6.0)
        self.assertAlmostEqual(dy, OFF_Y, delta=6.0)

    def test_a_dot_above_and_left_of_centre_reports_two_negative_offsets(self):
        rec = marker.classify_dots(self.up_left)["colours"]["black"]
        self.assertTrue(rec["found"])
        dx, dy = rec["offset_px"]
        self.assertLess(dx, 0.0)
        self.assertLess(dy, 0.0)
        self.assertAlmostEqual(dx, -OFF_X, delta=6.0)
        self.assertAlmostEqual(dy, -OFF_Y, delta=6.0)

    def test_the_offset_sign_convention_holds_for_a_colour_dot_too(self):
        rec = marker.classify_dots(self.red_off)["colours"]["red"]
        self.assertTrue(rec["found"])
        dx, dy = rec["offset_px"]
        self.assertAlmostEqual(dx, OFF_X, delta=8.0)
        self.assertAlmostEqual(dy, OFF_Y, delta=8.0)

    def test_the_reported_offset_is_the_one_best_offset_px_would_choose(self):
        """classify_dots must not invent a second steering rule."""
        rec = marker.classify_dots(self.down_right)["colours"]["black"]
        expected, why = marker.best_offset_px(rec["candidate"])
        self.assertEqual(rec["offset_px"], expected)
        self.assertEqual(rec["offset_source"], why)

    # -- conflicts are reported, not hidden --------------------------------

    def test_the_overlapping_claim_is_reported_as_a_conflict(self):
        res = marker.classify_dots(self.red)
        self.assertEqual(len(res["conflicts"]), 1)
        conflict = res["conflicts"][0]
        self.assertEqual(conflict["colours"], ["black", "red"])
        self.assertEqual(conflict["resolved_to"], "red")
        self.assertEqual(res["unresolved_conflicts"], 0)
        self.assertTrue(conflict["rule"])

    def test_the_demoted_dark_reading_travels_inside_its_conflict(self):
        """Attribution reinterprets a measurement; it must not delete one."""
        conflict = marker.classify_dots(self.blue)["conflicts"][0]
        self.assertIn("black_candidate", conflict)
        self.assertIsNotNone(conflict["black_candidate"]["centroid_px"])

    def test_a_black_dot_alone_produces_no_conflict(self):
        self.assertEqual(marker.classify_dots(self.black)["conflicts"], [])

    # -- the overlap rule itself, without any image -------------------------

    def test_overlap_needs_both_an_intersecting_bbox_and_close_centroids(self):
        same = {"bbox": [100, 100, 80, 80], "centroid_px": [140.0, 140.0],
                "equiv_radius_px": 40.0}
        twin = {"bbox": [102, 98, 78, 82], "centroid_px": [141.0, 139.0],
                "equiv_radius_px": 39.0}
        self.assertTrue(marker.blob_overlap(same, twin)["overlapping"])

        far = {"bbox": [900, 700, 60, 60], "centroid_px": [930.0, 730.0],
               "equiv_radius_px": 30.0}
        self.assertFalse(marker.blob_overlap(same, far)["overlapping"])

        # bboxes touch, centroids nowhere near each other: a big sprawling blob
        # bracketing a small distant one is not the same blob.
        sprawl = {"bbox": [0, 0, 400, 400], "centroid_px": [200.0, 200.0],
                  "equiv_radius_px": 30.0}
        corner = {"bbox": [380, 380, 40, 40], "centroid_px": [400.0, 400.0],
                  "equiv_radius_px": 20.0}
        metrics = marker.blob_overlap(sprawl, corner)
        self.assertGreater(metrics["bbox_iou"], 0.0)
        self.assertFalse(metrics["overlapping"])

    # -- detect() itself: blue added, red untouched -------------------------

    def test_detect_accepts_blue_and_mirrors_the_red_report_shape(self):
        blue = marker.detect(self.blue, mode="blue")
        red = marker.detect(self.red, mode="red")
        self.assertEqual(blue["n_candidates"], 1)
        self.assertEqual(red["n_candidates"], 1)
        for info, colour in ((blue, "blue"), (red, "red")):
            for key in ("%s_excess_p50", "%s_excess_p999", "%s_excess_max"):
                self.assertIn(key % colour, info)
            self.assertIsNotNone(info["threshold_used"])
        # same keys, same order, colour name substituted -- the blue branch is
        # the red one with a channel swapped, not a second dialect.
        self.assertEqual([k.replace("red", "blue") for k in red], list(blue))

    def test_a_colour_free_frame_reports_no_mark_and_an_explanatory_note(self):
        for mode in ("red", "blue"):
            info = marker.detect(self.black, mode=mode)
            self.assertEqual(info["n_candidates"], 0)
            self.assertIsNone(info["threshold_used"])
            self.assertIn("no %s mark present" % mode, info["note"])

    def test_an_unknown_mode_is_refused(self):
        with self.assertRaises(marker.MarkerError):
            marker.detect(self.black, mode="green")

    def test_the_existing_dark_and_red_entry_points_still_work(self):
        """The public surface callers already depend on (xy_center.py,
        calibration.py) must be unchanged by the colour work."""
        dark = marker.detect(self.black, mode="dark")
        self.assertEqual(dark["mode"], "dark")
        self.assertEqual(dark["n_candidates"], 1)
        self.assertIn("lit_area_frac", dark)
        offset, why = marker.best_offset_px(dark["candidates"][0])
        self.assertEqual(len(offset), 2)
        self.assertTrue(why.startswith("centroid"))


if __name__ == "__main__":
    unittest.main(verbosity=2)
