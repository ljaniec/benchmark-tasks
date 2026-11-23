import unittest
from dataclasses import replace

import spec
from scorer import Evidence, Scorer

START = Evidence(False, spec.WEIGHT_START[0], spec.LANE_Y, 0., 0., 0., True, True)
IN_3 = spec.LINE3 + spec.LINE_W / 2 + spec.PUCK_R + .010
IN_2 = spec.LINE2 + spec.LINE_W / 2 + spec.PUCK_R + .010
IN_1 = spec.FOUL + spec.LINE_W / 2 + spec.PUCK_R + .010


def run(*rows):
    s = Scorer()
    return [s.update(r) for r in rows], s


def held(n=10, x=spec.WEIGHT_START[0], speed=0.):
    return [replace(START, robot_on_weight=True, weight_x=x, weight_speed=speed)] * n


def shot(x_end, rest=spec.REST_STEPS, y=spec.LANE_Y, lift=0.):
    """Held, pushed, let go at 1.3 m/s, slides to x_end and rests there."""
    rows = held() + held(2, .40, 1.2)
    rows += [replace(START, weight_x=x, weight_speed=v) for x, v in [(.5, 1.29), (1.2, 1.1), (2., .6), (x_end, .05)]]
    rows += [replace(START, weight_x=x_end, weight_y=y, weight_lift=lift, weight_speed=0.)] * rest
    return rows


class Geometry(unittest.TestCase):
    def test_aperture_fits_the_weight(self):
        self.assertLess(2 * spec.PUCK_R, .072)
        self.assertEqual(spec.finger_joint_position(.075), -.0749 / 2)
        with self.assertRaises(ValueError):
            spec.finger_joint_position(.076)

    def test_board_and_lines_from_sources(self):
        self.assertAlmostEqual(spec.BOARD_L, 92 * .0254, places=4)       # 7 ft 8 in
        self.assertAlmostEqual(spec.BOARD_W, 20 * .0254, places=4)
        self.assertAlmostEqual(spec.BOARD_Z, 30 * .0254, places=3)
        self.assertAlmostEqual(spec.X1 - spec.LINE3, 6 * .0254, places=4)
        self.assertAlmostEqual(spec.X1 - spec.LINE2, 12 * .0254, places=4)
        self.assertAlmostEqual(spec.FOUL - spec.X0, spec.X1 - spec.FOUL)
        (cx0, cy0), (cx1, cy1) = spec.CRADLE
        self.assertAlmostEqual(cy1 - cy0, 31 * .0254, delta=.005)
        self.assertAlmostEqual(cx1 - cx0, 108 * .0254, delta=.03)        # 9 ft overall

    def test_cradle_clear_of_pedestals_and_weight_on_board(self):
        (cx0, _), _ = spec.CRADLE
        self.assertGreater(cx0, .075)
        x, y, z = spec.WEIGHT_START
        self.assertGreater(x - spec.PUCK_R, spec.X0)
        self.assertEqual(z, spec.BOARD_Z)
        self.assertTrue(spec.on_board(x, y, 0.))
        self.assertEqual(spec.zone_score(x), 0.)

    def test_zones(self):
        self.assertEqual(spec.zone_score(IN_3), 1.)
        self.assertEqual(spec.zone_score(spec.X1), 1.)                   # a hanger counts
        self.assertEqual(spec.zone_score(IN_2), .5)
        self.assertEqual(spec.zone_score(IN_1), .25)
        # touching a line counts the lower zone
        self.assertEqual(spec.zone_score(spec.LINE3 + spec.PUCK_R), .5)
        self.assertEqual(spec.zone_score(spec.LINE2 + spec.PUCK_R), .25)
        self.assertEqual(spec.zone_score(spec.FOUL + spec.PUCK_R), 0.)


class Ladder(unittest.TestCase):
    def test_inactivity(self):
        scores, s = run(*[START] * 600)
        self.assertEqual(max(scores), 0.)
        self.assertFalse(s.shot or s.invalid)

    def test_holding_never_lets_go(self):
        scores, s = run(*held(300), *held(50, .45, 0.))
        self.assertEqual(max(scores), 0.)
        self.assertFalse(s.shot)

    def test_zones_after_one_shot(self):
        for x, expected in [(IN_3, 1.), (spec.X1 - .002, 1.), (IN_2, .5), (IN_1, .25), (spec.FOUL - .2, 0.)]:
            scores, s = run(*shot(x))
            self.assertEqual(scores[-1], expected, x)
            self.assertFalse(s.invalid)

    def test_scored_only_once_at_rest(self):
        scores, _ = run(*shot(IN_3, rest=spec.REST_STEPS - 1))
        self.assertEqual(max(scores), 0.)
        scores, _ = run(*shot(IN_3, rest=spec.REST_STEPS))
        self.assertEqual(scores[-1], 1.)

    def test_off_the_end_or_side_is_dead(self):
        for rows in (shot(spec.X1 + .05, lift=-.08), shot(IN_3, y=spec.Y0 - .05, lift=-.06)):
            scores, s = run(*rows)
            self.assertEqual(max(scores), 0.)
            self.assertTrue(s.dead)

    def test_dead_stays_dead(self):
        rows = shot(spec.X1 + .05, lift=-.08, rest=3) + [replace(START, weight_x=IN_3)] * 40
        scores, _ = run(*rows)
        self.assertEqual(max(scores), 0.)

    def test_weight_on_its_edge_does_not_score(self):
        rows = shot(IN_3)[:-spec.REST_STEPS] + [replace(START, weight_x=IN_3, weight_tilt_deg=80.)] * 40
        scores, _ = run(*rows)
        self.assertEqual(max(scores), 0.)

    def test_slow_nudges_are_not_a_shot(self):
        rows = held(5) + [replace(START, weight_x=spec.WEIGHT_START[0] + .01, weight_speed=.05)] * 3 + held(5, .32)
        _, s = run(*rows)
        self.assertFalse(s.shot or s.invalid)


class Fouls(unittest.TestCase):
    def test_throw(self):
        rows = held(5) + [replace(START, weight_x=.5, weight_lift=.03, weight_speed=1.5)] + shot(IN_3)[12:]
        scores, s = run(*rows)
        self.assertTrue(s.invalid)
        self.assertEqual(max(scores), 0.)

    def test_lifting_in_hand_is_allowed(self):
        rows = [replace(START, robot_on_weight=True, weight_lift=.05)] * 20 + shot(IN_3)
        scores, s = run(*rows)
        self.assertFalse(s.invalid)
        self.assertEqual(scores[-1], 1.)

    def test_touch_after_release(self):
        rows = shot(IN_2)
        rows.insert(14, replace(START, robot_on_weight=True, weight_x=.55, weight_speed=1.))
        scores, s = run(*rows)
        self.assertTrue(s.invalid)
        self.assertEqual(max(scores), 0.)

    def test_table_moved_or_weight_out_of_workspace(self):
        for bad in (replace(START, intact=False), replace(START, inside=False),
                    replace(START, weight_x=float('nan'))):
            scores, s = run(*held(3), bad, *shot(IN_3))
            self.assertTrue(s.invalid)
            self.assertEqual(max(scores), 0.)

    def test_foul_latches_after_full_score(self):
        rows = shot(IN_3) + [replace(START, weight_x=IN_3, robot_on_weight=True)]
        scores, s = run(*rows)
        self.assertEqual(scores[-2], 1.)
        self.assertEqual(scores[-1], 0.)
        self.assertTrue(s.invalid)


if __name__ == '__main__':
    unittest.main()
