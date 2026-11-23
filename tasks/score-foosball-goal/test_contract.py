"""CPU-only contract tests for the foosball scorer and geometry."""
import math
import unittest
import spec
from scorer import Evidence, Scorer


def evidence(**kwargs):
    base = dict(
        ball_progress_m=0.,
        ball_past_keeper=False,
        ball_in_goal=False,
        rod_turn_deg=0.,
        robot_touched_ball=False,
        robot_over_table=False,
        ball_z_m=spec.BALL_START[2],
        inside_workspace=True,
        intact=True,
    )
    base.update(kwargs)
    return Evidence(**base)


class Contract(unittest.TestCase):
    def test_aperture_cap(self):
        self.assertAlmostEqual(spec.finger_joint_position(.075), -.03745)
        with self.assertRaises(ValueError):
            spec.finger_joint_position(.076)

    def test_table_dimensions(self):
        self.assertAlmostEqual(spec.PF_L, 1.20)
        self.assertAlmostEqual(spec.PF_W, .75)
        self.assertAlmostEqual(spec.GOAL_W, .203)
        self.assertAlmostEqual(spec.BALL_D, .035)
        self.assertAlmostEqual(spec.rod_y(1) - spec.rod_y(8), 7 * spec.ROD_SPACING)

    def test_feet_clear_playfield_and_strike_ball(self):
        toe = spec.ROD_Z - spec.MAN_FOOT_BELOW
        self.assertAlmostEqual(toe - spec.PF_Z, .003)
        self.assertLess(toe, spec.BALL_START[2])
        self.assertLess(toe + spec.FOOT_HEIGHT, spec.BALL_START[2] + spec.BALL_D / 2)

    def test_ball_starts_between_men_in_an_open_lane(self):
        x = spec.BALL_START[0] - spec.X_C
        gap = spec.FOOT_WIDTH / 2 + spec.BALL_D / 2
        self.assertTrue(all(abs(x - m) > gap for m in spec.RODS[6][1]))
        self.assertLessEqual(abs(spec.BALL_OFFSET_X), spec.ROD_TRAVEL)
        for k in (7, 8):
            self.assertTrue(all(abs(x - m) > gap for m in spec.RODS[k][1]))
        self.assertLess(abs(x), spec.GOAL_W / 2 - spec.BALL_D / 2)
        self.assertFalse(spec.in_goal(spec.BALL_START))

    def test_goal_geometry(self):
        self.assertTrue(spec.in_goal((spec.X_C, spec.Y_GOAL - .03, spec.PF_Z - .02)))
        self.assertFalse(spec.in_goal((spec.X_C + spec.GOAL_W / 2 + .001, spec.Y_GOAL - .03, spec.PF_Z)))
        self.assertFalse(spec.in_goal((spec.X_C, spec.Y_GOAL + .01, spec.PF_Z + .02)))

    def test_start_scores_zero(self):
        self.assertEqual(Scorer().update(evidence()), 0.)

    def test_partial_and_full_ladder(self):
        s = Scorer()
        self.assertEqual(s.update(evidence(ball_progress_m=spec.PROGRESS - .001)), 0.)
        self.assertEqual(s.update(evidence(ball_progress_m=spec.PROGRESS)), .25)
        self.assertEqual(s.update(evidence(ball_progress_m=0.)), .25)
        self.assertEqual(s.update(evidence(ball_progress_m=.3, ball_past_keeper=True)), .5)
        self.assertEqual(s.update(evidence(ball_progress_m=.1)), .5)
        self.assertEqual(s.update(evidence(ball_progress_m=.38, ball_past_keeper=True, ball_in_goal=True,
                                           ball_z_m=spec.PF_Z - .02)), 1.)
        self.assertEqual(s.update(evidence(ball_progress_m=.4, ball_past_keeper=True, ball_in_goal=True,
                                           ball_z_m=spec.PF_Z - .03)), 1.)

    def test_ball_dropping_into_goal_is_not_off_table(self):
        s = Scorer()
        self.assertEqual(s.update(evidence(ball_in_goal=True, ball_z_m=spec.PF_Z - .2)), 1.)
        self.assertFalse(s.invalid)

    def test_turns_up_to_one_revolution_allowed(self):
        s = Scorer()
        self.assertEqual(s.update(evidence(rod_turn_deg=spec.SPIN_LIMIT_DEG)), 0.)
        self.assertFalse(s.invalid)

    def test_forbidden_events_zero_trial_for_good(self):
        for bad in [dict(rod_turn_deg=spec.SPIN_LIMIT_DEG + .1), dict(robot_touched_ball=True),
                    dict(robot_over_table=True), dict(ball_z_m=spec.PF_Z - .11), dict(intact=False),
                    dict(inside_workspace=False), dict(ball_progress_m=math.nan)]:
            s = Scorer()
            s.update(evidence(ball_progress_m=.3, ball_past_keeper=True))
            self.assertEqual(s.update(evidence(**bad)), 0.)
            self.assertTrue(s.invalid)
            self.assertEqual(s.update(evidence(ball_in_goal=True, ball_past_keeper=True)), 0.)


if __name__ == '__main__':
    unittest.main()
