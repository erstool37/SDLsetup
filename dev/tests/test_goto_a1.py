#!/usr/bin/env python3
"""Guards on the gated A1 transit. Pure planning -- no SDK, no hardware, no sockets.

What these protect, worst consequence first:

1. **The working height reaches the lens.** The plate rides 3 mm above taught
   A1 and *stays there*, under the objective included. So the lens-clearance
   invariant is checked against the working pose, and a leg arriving at the
   taught height near A1 is as wrong as one arriving 3 mm high. Both refuse.
2. **The rise cap.** 10 mm above taught A1 is the operator's hard limit and the
   only number here a caller may not choose freely. 3 mm spends a third of it.
3. **The dogleg.** The X swing must happen well short of the standoff plane,
   because running Y to the standoff first passes over the UV-Vis. "Short of"
   means FURTHER from A1, and getting that sign backwards is a 300 mm error
   that puts the sweep beside the lens.
4. **Declared axes.** ``Envelope.free`` has no anchor and cannot check this;
   without it a typo is an unrefused diagonal traverse.
5. **Recoverability.** A half-finished leg must stay finishable. An early
   version of the order check got this backwards and would have stranded a
   half-completed push-in under the objective.
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.microscope import goto_a1 as g  # noqa: E402
from tools.arm.approach import LENS_XY_RADIUS_MM, RouteError  # noqa: E402
from tools.arm.safety import Z_MAX_RISE_MM, Envelope, SafetyError  # noqa: E402

#: Real values as they read on 2026-08-04, so a behaviour change shows up
#: against the actual rig rather than against tidy numbers.
A1 = [26.028557, 590.625244, 177.245712, 179.99515, -0.002807, 89.745359]
PARK = [366.561432, 0.423674, 61.371540, 179.995, -0.002, 0.003]
HOME = list(PARK)
WORK_Z = 177.245712 + 3.0
STAND_Y = 440.625244
DOGLEG_Y = 590.625244 - 150.0 - 110.0        # standoff 150 + backoff 110


def corridor(start=PARK, a1=A1, rise=g.WORKING_RISE_MM, home=None) -> Envelope:
    lowest = [start[2], a1[2]] + ([home[2]] if home else [])
    return Envelope.free(reason="test", z_floor_mm=min(lowest) - g.Z_FLOOR_MARGIN_MM,
                         z_ceiling_mm=a1[2] + rise, speed_max_mm_s=g.TRANSIT_SPEED_MM_S)


class TestWorkingHeight(unittest.TestCase):
    """The correction the operator made twice: no pulling down at the lens."""

    def test_working_pose_is_taught_a1_lifted(self):
        work = g.working_pose(A1)
        self.assertAlmostEqual(work[2], WORK_Z, places=9)
        for i in (0, 1, 3, 4, 5):
            self.assertAlmostEqual(work[i], A1[i], places=9)

    def test_the_push_in_happens_at_the_working_height(self):
        legs = g.legs_to_a1(PARK, A1)
        self.assertAlmostEqual(legs[-1]["pose"][2], WORK_Z, places=9)

    def test_no_leg_ever_descends_to_the_taught_height(self):
        """There is no Z-settle. The plate does not pull down in front of the lens."""
        for leg in g.legs_to_a1(PARK, A1)[1:]:
            self.assertAlmostEqual(leg["pose"][2], WORK_Z, places=9, msg=leg["label"])

    def test_only_the_first_leg_changes_z(self):
        legs = g.legs_to_a1(PARK, A1)
        self.assertEqual([leg["index"] for leg in legs if "z" in leg["changes"]], [1])

    def test_the_whole_route_stays_within_the_rise_budget(self):
        for leg in g.legs_to_a1(PARK, A1):
            self.assertLessEqual(leg["pose"][2] - A1[2], Z_MAX_RISE_MM + 1e-9)


class TestPlanToA1(unittest.TestCase):
    def test_six_legs_in_the_amended_order(self):
        legs = g.legs_to_a1(PARK, A1)
        self.assertEqual([leg["index"] for leg in legs], list(range(1, 7)))
        self.assertEqual([tuple(leg["changes"]) for leg in legs], [
            ("z",), ("roll", "pitch", "yaw"), ("y",), ("x",), ("y",), ("y",)])

    def test_the_dogleg_swings_x_short_of_the_standoff(self):
        """Short of the standoff = FURTHER from A1, not nearer."""
        legs = g.legs_to_a1(PARK, A1)
        self.assertEqual(tuple(legs[3]["changes"]), ("x",))
        self.assertAlmostEqual(legs[3]["pose"][1], DOGLEG_Y, places=3)
        self.assertAlmostEqual(abs(legs[3]["pose"][1] - A1[1]),
                               150.0 + g.DOGLEG_BACKOFF_MM, places=3)
        self.assertGreater(abs(legs[3]["pose"][1] - A1[1]), abs(STAND_Y - A1[1]))

    def test_the_dogleg_is_on_the_same_side_of_a1_as_the_standoff(self):
        legs = g.legs_to_a1(PARK, A1)
        self.assertLess(legs[3]["pose"][1], A1[1])
        self.assertLess(legs[3]["pose"][1], STAND_Y)

    def test_x_moves_exactly_once_and_only_in_the_dogleg(self):
        legs = g.legs_to_a1(PARK, A1)
        self.assertEqual([leg["index"] for leg in legs if "x" in leg["changes"]], [4])

    def test_final_leg_is_a_pure_y_push_in(self):
        legs = g.legs_to_a1(PARK, A1)
        before, after = legs[4]["pose"], legs[5]["pose"]
        self.assertEqual(tuple(legs[5]["changes"]), ("y",))
        for i in (0, 2, 3, 4, 5):
            self.assertAlmostEqual(before[i], after[i], places=9)
        self.assertAlmostEqual(abs(after[1] - before[1]), 150.0, places=3)
        self.assertEqual([round(v, 6) for v in after],
                         [round(v, 6) for v in g.working_pose(A1)])

    def test_push_in_is_slower_than_the_transit(self):
        legs = g.legs_to_a1(PARK, A1)
        self.assertLess(legs[5]["speed_mm_s"], legs[2]["speed_mm_s"])


class TestPlanToHome(unittest.TestCase):
    def test_pull_out_is_first_and_pure_y_at_the_height_it_is_at(self):
        at = g.working_pose(A1)
        legs = g.legs_to_home(at, A1, HOME)
        self.assertEqual(tuple(legs[0]["changes"]), ("y",))
        self.assertAlmostEqual(legs[0]["pose"][2], WORK_Z, places=9)
        self.assertAlmostEqual(legs[0]["pose"][1], STAND_Y, places=3)
        self.assertEqual(legs[0]["speed_mm_s"], g.PUSH_IN_SPEED_MM_S)

    def test_nothing_but_y_moves_until_the_plate_is_clear(self):
        legs = g.legs_to_home(g.working_pose(A1), A1, HOME)
        self.assertEqual(tuple(legs[0]["changes"]), ("y",))
        self.assertEqual(tuple(legs[1]["changes"]), ("y",))

    def test_wrist_is_not_unsquared_until_back_at_the_park_plane(self):
        legs = g.legs_to_home(g.working_pose(A1), A1, HOME)
        self.assertEqual([leg["index"] for leg in legs if "yaw" in leg["changes"]], [5])
        self.assertAlmostEqual(legs[4]["pose"][1], HOME[1], places=3)

    def test_z_comes_down_only_at_the_very_end(self):
        legs = g.legs_to_home(g.working_pose(A1), A1, HOME)
        self.assertEqual([leg["index"] for leg in legs if "z" in leg["changes"]], [6])

    def test_it_retraces_the_dogleg(self):
        legs = g.legs_to_home(g.working_pose(A1), A1, HOME)
        self.assertEqual(tuple(legs[2]["changes"]), ("x",))
        self.assertAlmostEqual(legs[2]["pose"][1], DOGLEG_Y, places=3)

    def test_it_lands_on_the_taught_park_pose(self):
        legs = g.legs_to_home(g.working_pose(A1), A1, HOME)
        self.assertEqual([round(v, 6) for v in legs[-1]["pose"]],
                         [round(v, 6) for v in HOME])

    def test_a_pure_y_pullout_holds_the_arms_own_x_not_the_taught_one(self):
        """A 9e-6 mm settle difference must not make leg 1 an undeclared X move."""
        settled = list(g.working_pose(A1))
        settled[0] += 9e-6
        legs = g.legs_to_home(settled, A1, HOME)
        self.assertAlmostEqual(legs[0]["pose"][0], settled[0], places=12)
        g.check_lens_clearance(legs, A1, settled)


class TestLensClearance(unittest.TestCase):
    def test_forward_route_passes(self):
        g.check_lens_clearance(g.legs_to_a1(PARK, A1), A1, PARK)

    def test_reverse_route_passes(self):
        at = g.working_pose(A1)
        g.check_lens_clearance(g.legs_to_home(at, A1, HOME), A1, at)

    def test_no_waypoint_near_a1_sits_at_any_other_height(self):
        at = g.working_pose(A1)
        for legs, start in ((g.legs_to_a1(PARK, A1), PARK),
                            (g.legs_to_home(at, A1, HOME), at)):
            del start
            for leg in legs:
                dx = leg["pose"][0] - A1[0]
                dy = leg["pose"][1] - A1[1]
                if (dx * dx + dy * dy) ** 0.5 <= LENS_XY_RADIUS_MM:
                    self.assertAlmostEqual(leg["pose"][2], WORK_Z, places=9,
                                           msg="near the lens: " + leg["label"])

    def test_arriving_at_the_TAUGHT_height_is_refused(self):
        """The old behaviour. Now wrong: the plate must arrive 3 mm up."""
        legs = g.legs_to_a1(PARK, A1)
        legs[-1] = dict(legs[-1])
        legs[-1]["pose"] = list(legs[-1]["pose"])
        legs[-1]["pose"][2] = A1[2]
        with self.assertRaises(RouteError):
            g.check_lens_clearance(legs, A1, PARK)

    def test_arriving_too_high_is_refused(self):
        legs = g.legs_to_a1(PARK, A1)
        legs[-1] = dict(legs[-1])
        legs[-1]["pose"] = list(legs[-1]["pose"])
        legs[-1]["pose"][2] = WORK_Z + 2.0
        with self.assertRaises(RouteError):
            g.check_lens_clearance(legs, A1, PARK)

    def test_a_diagonal_leg_is_refused(self):
        legs = g.legs_to_a1(PARK, A1)
        legs[4] = dict(legs[4])
        legs[4]["pose"] = list(legs[4]["pose"])
        legs[4]["pose"][0] += 5.0
        with self.assertRaises(RouteError):
            g.check_lens_clearance(legs, A1, PARK)


class TestRiseCap(unittest.TestCase):
    def test_default_rise_is_inside_the_operator_limit(self):
        self.assertLessEqual(g.WORKING_RISE_MM, Z_MAX_RISE_MM)

    def test_rise_at_the_limit_is_allowed(self):
        self.assertEqual(g.check_rise(Z_MAX_RISE_MM), Z_MAX_RISE_MM)

    def test_rise_past_the_limit_is_refused(self):
        for bad in (Z_MAX_RISE_MM + 0.001, 15.0, 120.0):
            with self.assertRaises(SafetyError):
                g.check_rise(bad)

    def test_negative_and_nonfinite_rise_refused(self):
        for bad in (-0.1, float("nan"), float("inf")):
            with self.assertRaises(SafetyError):
                g.check_rise(bad)

    def test_planning_with_an_illegal_rise_refuses(self):
        for builder in (lambda: g.legs_to_a1(PARK, A1, rise_mm=25.0),
                        lambda: g.legs_to_home(PARK, A1, HOME, rise_mm=25.0),
                        lambda: g.working_pose(A1, 25.0)):
            with self.assertRaises(SafetyError):
                builder()


class TestCorridor(unittest.TestCase):
    def test_ceiling_admits_the_working_height_and_refuses_above_it(self):
        env = corridor()
        env.check_target([A1[0], DOGLEG_Y, WORK_Z, 180.0, 0.0, 89.7], what="transit")
        for over in (0.001, 1.0, 10.0):
            with self.assertRaises(SafetyError):
                env.check_target([A1[0], A1[1], WORK_Z + over, 180.0, 0.0, 89.7],
                                 what="above the ceiling")

    def test_every_commanded_pose_of_every_leg_passes(self):
        at = g.working_pose(A1)
        for legs, start, home in ((g.legs_to_a1(PARK, A1), PARK, None),
                                  (g.legs_to_home(at, A1, HOME), at, HOME)):
            env = corridor(start=start, home=home)
            previous = start
            for leg in legs:
                for pose in g.substeps(leg, previous):
                    env.check_target(pose, what=leg["label"])
                previous = leg["pose"]


class TestDeclaredAxes(unittest.TestCase):
    def test_each_leg_changes_only_what_it_declares(self):
        at = g.working_pose(A1)
        for legs, start in ((g.legs_to_a1(PARK, A1), PARK),
                            (g.legs_to_home(at, A1, HOME), at)):
            previous = start
            for leg in legs:
                g.check_declared_axes(leg, previous)
                previous = leg["pose"]

    def test_a_diagonal_is_refused(self):
        legs = g.legs_to_a1(PARK, A1)
        bad = dict(legs[2])
        bad["pose"] = list(bad["pose"])
        bad["pose"][0] += 5.0
        with self.assertRaises(SafetyError) as caught:
            g.check_declared_axes(bad, legs[1]["pose"])
        self.assertIn("x", str(caught.exception))


class TestRotationSubSteps(unittest.TestCase):
    def test_wrist_square_is_split_into_bounded_steps(self):
        legs = g.legs_to_a1(PARK, A1)
        steps = g.substeps(legs[1], legs[0]["pose"])
        self.assertGreater(len(steps), 1)
        previous = legs[0]["pose"]
        for step in steps:
            for i in (3, 4, 5):
                self.assertLessEqual(abs(step[i] - previous[i]), g.MAX_ROT_STEP_DEG + 1e-9)
            previous = step
        self.assertEqual([round(v, 6) for v in steps[-1]],
                         [round(v, 6) for v in legs[1]["pose"]])

    def test_rotation_sub_steps_never_translate_whatever_start_they_are_given(self):
        legs = g.legs_to_a1(PARK, A1)
        for handed in (PARK, legs[0]["pose"], A1, g.working_pose(A1)):
            for step in g.substeps(legs[1], handed):
                for i in (0, 1, 2):
                    self.assertAlmostEqual(step[i], legs[1]["pose"][i], places=9)

    def test_translation_legs_are_one_command(self):
        for leg in g.legs_to_a1(PARK, A1):
            if "yaw" not in leg["changes"]:
                self.assertEqual(len(g.substeps(leg, PARK)), 1)


class TestPreconditions(unittest.TestCase):
    def test_leg_1_runs_from_the_park_pose(self):
        self.assertEqual(g.precondition_failures(g.legs_to_a1(PARK, A1)[0], PARK), [])

    def test_later_legs_refuse_from_the_park_pose(self):
        legs = g.legs_to_a1(PARK, A1)
        for i in range(1, 6):
            self.assertTrue(g.precondition_failures(legs[i], PARK),
                            "leg %d should refuse from park" % (i + 1))

    def test_push_in_from_the_park_pose_names_z_and_the_wrist(self):
        names = [n for n, _w, _g in
                 g.precondition_failures(g.legs_to_a1(PARK, A1)[5], PARK)]
        self.assertIn("z", names)
        self.assertIn("yaw", names)

    def test_each_leg_becomes_runnable_exactly_when_its_turn_comes(self):
        at = list(PARK)
        for i in range(6):
            legs = g.legs_to_a1(at, A1)
            self.assertEqual(g.precondition_failures(legs[i], at), [],
                             "leg %d should run from the pose after leg %d" % (i + 1, i))
            at = list(legs[i]["pose"])
        self.assertEqual([round(v, 6) for v in at],
                         [round(v, 6) for v in g.working_pose(A1)])

    def test_the_reverse_route_runs_leg_by_leg_from_the_working_a1(self):
        at = list(g.working_pose(A1))
        for i in range(6):
            legs = g.legs_to_home(at, A1, HOME)
            self.assertEqual(g.precondition_failures(legs[i], at), [],
                             "reverse leg %d" % (i + 1))
            at = list(legs[i]["pose"])
        self.assertEqual([round(v, 6) for v in at], [round(v, 6) for v in HOME])

    def test_an_aborted_push_in_can_still_be_finished(self):
        """Ctrl-C halfway leaves the plate under the objective. It must stay live."""
        stranded = [A1[0], 500.0, WORK_Z, A1[3], A1[4], A1[5]]
        legs = g.legs_to_a1(stranded, A1)
        self.assertEqual(g.precondition_failures(legs[5], stranded), [])
        self.assertFalse(g.leg_satisfied(legs[5], stranded))
        self.assertAlmostEqual(legs[5]["pose"][1], A1[1], places=6)

    def test_an_aborted_traverse_can_still_be_finished(self):
        partway = [PARK[0], 200.0, WORK_Z, A1[3], A1[4], A1[5]]
        legs = g.legs_to_a1(partway, A1)
        self.assertEqual(g.precondition_failures(legs[2], partway), [])
        self.assertAlmostEqual(legs[2]["pose"][1], DOGLEG_Y, places=3)

    def test_a_hand_jogged_arm_refuses(self):
        jogged = [A1[0] + 40.0, STAND_Y, WORK_Z, A1[3], A1[4], A1[5]]
        failures = g.precondition_failures(g.legs_to_a1(jogged, A1)[5], jogged)
        self.assertEqual([n for n, _w, _g in failures], ["x"])

    def test_a_plate_left_at_the_taught_height_refuses_the_push_in(self):
        """It must be lifted to the working height before it goes under the lens."""
        low = [A1[0], STAND_Y, A1[2], A1[3], A1[4], A1[5]]
        failures = g.precondition_failures(g.legs_to_a1(low, A1)[5], low)
        self.assertEqual([n for n, _w, _g in failures], ["z"])

    def test_arrival_reports_the_route_finished(self):
        at = g.working_pose(A1)
        legs = g.legs_to_a1(at, A1)
        self.assertEqual(g.precondition_failures(legs[5], at), [])
        self.assertTrue(g.leg_satisfied(legs[5], at))


class TestTraverseFloor(unittest.TestCase):
    """The limit the arm itself set, 2026-08-04.

    Commanding y=290.625 at A1's column stopped the controller at y=319.855 --
    state=4, code=-9, error_code=0. Nothing in the software predicted it, and
    inverse kinematics actively disagreed (it reported y=290 reachable and
    y=280 not), so it is pinned here as a measured number.
    """

    def test_the_default_backoff_clears_the_floor(self):
        self.assertGreaterEqual(g.dogleg_y(A1), g.Y_TRAVERSE_FLOOR_MM)

    def test_the_floor_sits_above_where_the_arm_actually_stopped(self):
        self.assertGreater(g.Y_TRAVERSE_FLOOR_MM, 319.855)

    def test_a_backoff_past_the_floor_is_refused(self):
        for bad in (130.0, 150.0, 200.0):
            with self.assertRaises(SafetyError) as caught:
                g.dogleg_y(A1, bad)
            self.assertIn("319.855", str(caught.exception))

    def test_the_refusal_names_the_largest_allowed_backoff(self):
        with self.assertRaises(SafetyError) as caught:
            g.dogleg_y(A1, 150.0)
        self.assertIn("110.6", str(caught.exception))

    def test_planning_with_a_backoff_past_the_floor_is_refused(self):
        for builder in (lambda: g.legs_to_a1(PARK, A1, backoff_mm=150.0),
                        lambda: g.legs_to_home(PARK, A1, HOME, backoff_mm=150.0)):
            with self.assertRaises(SafetyError):
                builder()

    def test_negative_backoff_refused(self):
        with self.assertRaises(SafetyError):
            g.dogleg_y(A1, -5.0)

    def test_the_proven_hundred_mm_backoff_still_passes(self):
        self.assertAlmostEqual(g.dogleg_y(A1, 100.0), 340.625244, places=3)


if __name__ == "__main__":
    unittest.main(verbosity=2)
