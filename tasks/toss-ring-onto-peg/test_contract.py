"""CPU-only contract tests for the ring toss scorer and geometry."""
import math
import unittest
import numpy as np
import spec
from scorer import Evidence, Scorer


def evidence(**kwargs):
    base = dict(
        robot_past_line=False,
        ring_touched_set=False,
        ring_touched_target=False,
        ring_around_target=False,
        ring_landed=False,
        ring_touched_robot=False,
        ring_z_m=.85,
        inside_workspace=True,
        intact=True,
    )
    base.update(kwargs)
    return Evidence(**base)


RINGER = dict(ring_around_target=True, ring_landed=True, ring_z_m=.03)
PEG_BASE = (spec.SET_X, spec.SET_Y, spec.BASE_BAR[1] / 2)
PEG_TOP = (spec.SET_X, spec.SET_Y, spec.PEG_TOP_Z)


class Contract(unittest.TestCase):
    def test_aperture_cap(self):
        self.assertAlmostEqual(spec.finger_joint_position(.075), -.03745)
        with self.assertRaises(ValueError):
            spec.finger_joint_position(.076)

    def test_published_toy_dimensions(self):
        self.assertAlmostEqual(spec.RING_OUTER_D, .127)
        self.assertAlmostEqual(spec.BASE_SPAN, .406)
        self.assertAlmostEqual(spec.PEG_TOP_Z, .178)
        self.assertAlmostEqual(spec.SET_MASS + 5 * spec.RING_MASS, .272)

    def test_target_is_out_of_reach_and_beyond_the_line(self):
        for base in [(0., .305), (0., -.305)]:
            self.assertGreater(math.dist(base, (spec.SET_X, spec.SET_Y)), .80)
        self.assertGreater(spec.SET_X - spec.BASE_SPAN / 2, spec.LINE_X)

    def test_ring_overhangs_table_edge_and_rests_on_it(self):
        back_tube = spec.RING_START[0] - spec.RING_MID_R
        self.assertLess(back_tube + spec.RING_TUBE_D / 2, spec.TABLE_BACK_X)
        self.assertGreater(spec.RING_START[0], spec.TABLE_BACK_X)

    def test_around_peg_geometry(self):
        up = (0., 0., 1.)
        self.assertTrue(spec.around_peg((spec.SET_X, spec.SET_Y, .03), up, PEG_BASE, PEG_TOP))
        free = spec.RING_INNER_R - spec.PEG_D / 2
        self.assertTrue(spec.around_peg((spec.SET_X + free - .001, spec.SET_Y, .03), up, PEG_BASE, PEG_TOP))
        self.assertFalse(spec.around_peg((spec.SET_X + free + .002, spec.SET_Y, .03), up, PEG_BASE, PEG_TOP))
        # above the tip, or the hole facing sideways, is not a ringer
        self.assertFalse(spec.around_peg((spec.SET_X, spec.SET_Y, spec.PEG_TOP_Z + .01), up, PEG_BASE, PEG_TOP))
        self.assertFalse(spec.around_peg((spec.SET_X, spec.SET_Y, .1), (1., 0., 0.), PEG_BASE, PEG_TOP))
        tilted = np.array([math.sin(.3), 0., math.cos(.3)])
        self.assertTrue(spec.around_peg((spec.SET_X, spec.SET_Y, .06), tilted, PEG_BASE, PEG_TOP))

    def test_start_scores_zero(self):
        self.assertEqual(Scorer().update(evidence()), 0.)

    def test_partial_and_full_ladder(self):
        s = Scorer()
        self.assertEqual(s.update(evidence(ring_touched_set=True, ring_landed=True)), .25)
        self.assertEqual(s.update(evidence(ring_landed=True)), .25)
        self.assertEqual(s.update(evidence(ring_touched_set=True, ring_touched_target=True, ring_landed=True)), .5)
        self.assertEqual(s.update(evidence(ring_landed=True)), .5)
        for _ in range(spec.DWELL_STEPS - 1):
            self.assertEqual(s.update(evidence(**RINGER)), .5)
        self.assertEqual(s.update(evidence(**RINGER)), 1.)
        self.assertEqual(s.update(evidence(ring_landed=True)), 1.)

    def test_dwell_resets_when_ring_leaves_peg(self):
        s = Scorer()
        for _ in range(spec.DWELL_STEPS - 1):
            s.update(evidence(**RINGER))
        s.update(evidence(ring_landed=True))
        self.assertEqual(s.dwell, 0)
        for _ in range(spec.DWELL_STEPS - 1):
            self.assertLess(s.update(evidence(**RINGER)), 1.)
        self.assertEqual(s.update(evidence(**RINGER)), 1.)

    def test_robot_may_hold_ring_in_flight_but_not_after_landing(self):
        s = Scorer()
        self.assertEqual(s.update(evidence(ring_touched_robot=True)), 0.)
        self.assertFalse(s.invalid)
        self.assertEqual(s.update(evidence(ring_touched_robot=True, ring_landed=True)), 0.)
        self.assertTrue(s.invalid)

    def test_forbidden_events_zero_trial_for_good(self):
        for bad in [dict(robot_past_line=True), dict(ring_touched_robot=True, ring_landed=True),
                    dict(intact=False), dict(inside_workspace=False), dict(ring_z_m=math.nan)]:
            s = Scorer()
            s.update(evidence(ring_touched_set=True, ring_touched_target=True, ring_landed=True))
            self.assertEqual(s.update(evidence(**bad)), 0.)
            self.assertTrue(s.invalid)
            for _ in range(spec.DWELL_STEPS + 1):
                self.assertEqual(s.update(evidence(**RINGER)), 0.)


if __name__ == '__main__':
    unittest.main()
