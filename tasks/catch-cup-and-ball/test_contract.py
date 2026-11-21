"""CPU-only contract tests for the cup-and-ball scorer and geometry."""
import math
import unittest
import spec
from scorer import Evidence, Scorer


def evidence(**kwargs):
    base = dict(
        handle_clear_m=.05,
        tilt_deg=5.,
        ball_above_rim_m=-.03,
        ball_in_cup=True,
        ball_touched_robot=False,
        ball_touched_fixture=False,
        toy_z_m=1.,
        inside_workspace=True,
        intact=True,
    )
    base.update(kwargs)
    return Evidence(**base)


START = dict(handle_clear_m=-spec.SLEEVE_HEIGHT, tilt_deg=0., ball_above_rim_m=-.60, ball_in_cup=False)


class Contract(unittest.TestCase):
    def test_aperture_cap(self):
        self.assertAlmostEqual(spec.finger_joint_position(.075), -.03745)
        with self.assertRaises(ValueError):
            spec.finger_joint_position(.076)

    def test_published_toy_dimensions(self):
        self.assertAlmostEqual(spec.HANDLE_LEN + spec.CUP_HEIGHT, .1905)
        self.assertAlmostEqual(2 * spec.CUP_OUTER_R, .0889)
        self.assertAlmostEqual(2 * spec.BALL_R, .03175)
        self.assertAlmostEqual(spec.TOY_MASS + spec.BALL_MASS, .120)

    def test_ball_starts_hanging_on_full_string_clear_of_everything(self):
        tie = spec.tie_world(spec.TOY_START)
        ball = spec.ball_start()
        self.assertAlmostEqual(math.dist(tie, ball), spec.STRING_LEN + spec.BALL_R)
        self.assertGreater(ball[2] - spec.BALL_R, .30)
        # the string leaves the sleeve through its slot, outside the handle
        self.assertGreater(-spec.TIE_LOCAL[1], spec.HANDLE_R)
        self.assertLess(-spec.TIE_LOCAL[1], spec.SLEEVE_INNER_R + spec.SLEEVE_WALL)

    def test_handle_stands_in_sleeve(self):
        self.assertAlmostEqual(spec.TOY_START[2] - spec.HANDLE_LEN, spec.SLEEVE_BOTTOM_Z)
        self.assertLess(spec.HANDLE_R, spec.SLEEVE_INNER_R)

    def test_ball_on_cup_floor_is_in_cup(self):
        self.assertTrue(spec.in_cup((0., 0., spec.CUP_FLOOR_LOCAL_Z + spec.BALL_R)))
        self.assertFalse(spec.in_cup((0., 0., spec.RIM_LOCAL_Z + .01)))
        self.assertFalse(spec.in_cup((spec.CUP_INNER_R, 0., spec.CUP_FLOOR_LOCAL_Z + spec.BALL_R)))

    def test_start_scores_zero(self):
        self.assertEqual(Scorer().update(evidence(**START)), 0.)

    def test_partial_and_full_ladder(self):
        s = Scorer()
        self.assertEqual(s.update(evidence(handle_clear_m=spec.LIFT_CLEAR, ball_above_rim_m=-.6, ball_in_cup=False)), .25)
        self.assertEqual(s.update(evidence(ball_above_rim_m=.05, ball_in_cup=False)), .5)
        self.assertEqual(s.update(evidence(ball_above_rim_m=-.6, ball_in_cup=False)), .5)
        for _ in range(spec.DWELL_STEPS - 1):
            self.assertEqual(s.update(evidence()), .5)
        self.assertEqual(s.update(evidence()), 1.)

    def test_ball_above_rim_needs_upright_toy_and_lift(self):
        s = Scorer()
        self.assertEqual(s.update(evidence(handle_clear_m=0., ball_above_rim_m=.05, ball_in_cup=False)), 0.)
        self.assertEqual(s.update(evidence(tilt_deg=spec.UPRIGHT_DEG + 1, ball_above_rim_m=.05, ball_in_cup=False)), .25)

    def test_dwell_resets_when_ball_leaves_cup_or_toy_tips(self):
        s = Scorer()
        for _ in range(spec.DWELL_STEPS - 1):
            s.update(evidence())
        self.assertEqual(s.update(evidence(ball_in_cup=False)), .25)
        self.assertEqual(s.dwell, 0)
        s.update(evidence())
        self.assertEqual(s.update(evidence(tilt_deg=spec.UPRIGHT_DEG + 1)), .25)
        self.assertEqual(s.dwell, 0)

    def test_forbidden_contacts_zero_trial_for_good(self):
        for bad in [dict(ball_touched_robot=True), dict(ball_touched_fixture=True), dict(toy_z_m=spec.DROP_Z - .01),
                    dict(intact=False), dict(inside_workspace=False), dict(tilt_deg=math.nan)]:
            s = Scorer()
            s.update(evidence())
            self.assertEqual(s.update(evidence(**bad)), 0.)
            self.assertTrue(s.invalid)
            for _ in range(spec.DWELL_STEPS + 1):
                self.assertEqual(s.update(evidence()), 0.)

    def test_fixture_collision_is_distinct_from_cup_contact_or_nearby_paths(self):
        ball = '/World/ball'
        fixtures = ('/World/stand_sleeve', '/World/floor', '/World/pedestal_right')
        self.assertTrue(spec.ball_hit_fixture((ball + '/shape', fixtures[0] + '/wall_03'), ball, fixtures))
        self.assertTrue(spec.ball_hit_fixture((fixtures[1] + '/shape', ball + '/shape'), ball, fixtures))
        self.assertFalse(spec.ball_hit_fixture((ball + '/shape', '/World/toy/cup_wall_03'), ball, fixtures))
        self.assertFalse(spec.ball_hit_fixture((ball + '/shape', '/World/floor_marker/shape'), ball, fixtures))
        self.assertFalse(spec.ball_hit_fixture(('/World/toy/handle', fixtures[0] + '/wall'), ball, fixtures))


if __name__ == '__main__':
    unittest.main()
