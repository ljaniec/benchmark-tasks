"""CPU-only contract tests for the Newton's cradle scorer and geometry."""
import math
import unittest
import spec
from scorer import Evidence, Scorer


def evidence(**kwargs):
    base = dict(
        ball1_angle_deg=0.,
        ball1_taut=True,
        others_max_deg=0.,
        ball1_speed_mps=0.,
        far_out_deg=0.,
        middle_max_deg=0.,
        robot_touched_ball1=False,
        robot_touched_other=False,
        inside_workspace=True,
        intact=True,
    )
    base.update(kwargs)
    return Evidence(**base)


HELD = dict(ball1_angle_deg=30., robot_touched_ball1=True)


def swing(s, peak, middle=0.):
    """Ball 5 rises to `peak` degrees and turns back; returns the last score."""
    score = s.update(evidence())
    for a in [peak / 2, peak, peak - 3, peak / 3]:
        score = s.update(evidence(far_out_deg=a, others_max_deg=a, middle_max_deg=middle))
    return score


class Contract(unittest.TestCase):
    def test_aperture_cap(self):
        self.assertAlmostEqual(spec.finger_joint_position(.075), -.03745)
        with self.assertRaises(ValueError):
            spec.finger_joint_position(.076)

    def test_balls_hang_apart_on_their_lines(self):
        for k in range(spec.BALLS - 1):
            self.assertAlmostEqual(spec.ball_rest_x(k + 1) - spec.ball_rest_x(k) - spec.BALL_D, spec.BALL_GAP)
        self.assertAlmostEqual(spec.ball_rest_x(2), spec.BASE_CENTRE[0])
        for k in range(spec.BALLS):
            for a in (0., 20., 35.):
                p, _ = spec.ball_pose(k, a)
                self.assertAlmostEqual(spec.swing_angle_deg(k, p), a)
                self.assertAlmostEqual(spec.radius_error(k, p), 0.)
        # The line runs from the rail to the ball's top, which sits SWING_R - BALL_R below the rail.
        self.assertAlmostEqual(spec.LINE_LEN, math.hypot(spec.RAIL_Y, spec.SWING_R - spec.BALL_R))
        self.assertGreater(spec.BALL_Z - spec.BALL_R, spec.BASE_TOP_Z)

    def test_start_scores_zero(self):
        self.assertEqual(Scorer().update(evidence()), 0.)

    def test_lift_alone_is_quarter(self):
        s = Scorer()
        self.assertEqual(s.update(evidence(ball1_angle_deg=19.9, robot_touched_ball1=True)), 0.)
        self.assertEqual(s.update(evidence(**HELD, ball1_taut=False)), 0.)
        self.assertEqual(s.update(evidence(**HELD, others_max_deg=3.1)), 0.)
        self.assertEqual(s.update(evidence(**HELD)), .25)
        self.assertEqual(s.update(evidence()), .25)

    def test_clean_release_full(self):
        s = Scorer()
        s.update(evidence(**HELD))
        self.assertEqual(swing(s, 25.), 1.)
        self.assertEqual(s.update(evidence()), 1.)

    def test_half_for_short_or_dirty_swing(self):
        s = Scorer()
        s.update(evidence(**HELD))
        self.assertEqual(swing(s, 14.), .5)
        s = Scorer()
        s.update(evidence(**HELD))
        self.assertEqual(swing(s, 25., middle=6.1), .5)
        s = Scorer()
        s.update(evidence(**HELD))
        self.assertEqual(swing(s, 16.), 1.)
        s = Scorer()
        s.update(evidence(**HELD))
        self.assertEqual(swing(s, 9.), .25)

    def test_release_must_be_from_rest_high_and_taut(self):
        for held in [dict(ball1_angle_deg=19.), dict(ball1_taut=False), dict(ball1_speed_mps=.11),
                     dict(others_max_deg=3.5)]:
            s = Scorer()
            s.update(evidence(**{**HELD, **held}))
            self.assertLess(swing(s, 30.), .5)

    def test_only_first_swing_after_release_counts(self):
        s = Scorer()
        s.update(evidence(**HELD))
        self.assertEqual(swing(s, 8.), .25)
        self.assertEqual(swing(s, 30.), .25)

    def test_regrab_disarms(self):
        s = Scorer()
        s.update(evidence(**HELD))
        s.update(evidence())
        self.assertTrue(s.armed)
        s.update(evidence(robot_touched_ball1=True, ball1_angle_deg=5.))
        self.assertFalse(s.armed)
        s.update(evidence())
        self.assertFalse(s.armed)

    def test_slam_while_holding_scores_no_release(self):
        s = Scorer()
        s.update(evidence(**HELD))
        # Ball 1 driven into the row while still held: ball 5 flies out, but nothing was let go.
        for a in [10., 25., 30., 27.]:
            self.assertEqual(s.update(evidence(robot_touched_ball1=True, ball1_angle_deg=0., ball1_speed_mps=.6,
                                               far_out_deg=a, others_max_deg=a)), .25)
        self.assertEqual(swing(s, 30.), .25)

    def test_push_from_hanging_scores_zero(self):
        s = Scorer()
        s.update(evidence(robot_touched_ball1=True, ball1_angle_deg=2., ball1_speed_mps=.5))
        self.assertEqual(swing(s, 30.), 0.)

    def test_retained_grasp_caps_at_quarter(self):
        s = Scorer()
        for _ in range(400):
            score = s.update(evidence(**HELD))
        self.assertEqual(score, .25)
        self.assertFalse(s.armed)

    def test_swing_without_touching_ball1_scores_zero(self):
        self.assertEqual(swing(Scorer(), 30.), 0.)

    def test_forbidden_events_zero_trial_for_good(self):
        for bad in [dict(robot_touched_other=True), dict(intact=False), dict(inside_workspace=False),
                    dict(far_out_deg=math.nan)]:
            s = Scorer()
            s.update(evidence(**HELD))
            self.assertEqual(s.update(evidence(**bad)), 0.)
            self.assertTrue(s.invalid)
            s.update(evidence(**HELD))
            self.assertEqual(swing(s, 30.), 0.)


if __name__ == '__main__':
    unittest.main()
