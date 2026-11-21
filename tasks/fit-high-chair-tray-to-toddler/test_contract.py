"""CPU-only contract tests for the high-chair tray scorer and geometry."""
import math
import unittest
import spec
from scorer import Evidence, Scorer


def evidence(**kwargs):
    base = dict(
        tray_x_m=spec.POSITION_X[spec.TARGET_POSITION - 1],
        tray_z_m=spec.TRAY_Z,
        tray_yaw_deg=0.,
        bail_deg=0.,
        left_position=spec.TARGET_POSITION,
        right_position=spec.TARGET_POSITION,
        touching_child=False,
        hands_clear=True,
        stable=True,
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

    def test_target_is_closest_position_that_clears_the_toddler(self):
        gaps = [spec.rim_gap(i) for i in range(1, len(spec.POSITION_X) + 1)]
        self.assertEqual(len(gaps), 4)
        self.assertAlmostEqual(gaps[spec.TARGET_POSITION - 1], .020)
        self.assertTrue(all(g > 0 for g in gaps[:spec.TARGET_POSITION]))
        self.assertTrue(all(g < 0 for g in gaps[spec.TARGET_POSITION:]))

    def test_start_is_locked_at_position_one(self):
        self.assertEqual(spec.TRAY_START_X, spec.POSITION_X[0])
        self.assertGreater(spec.finger_top_rest_world(), spec.rail_bottom() + spec.ENGAGE_DEPTH)

    def test_squeezed_bail_clears_the_notches(self):
        drop = spec.finger_drop(math.radians(spec.BAIL_LIMIT_DEG))
        self.assertLess(spec.finger_top_rest_world() - drop, spec.rail_bottom())

    def test_toddler_belly_matches_belly_line(self):
        front = min(min(p0[0], p1[0]) - r for name, (p0, p1, r) in spec.TODDLER.items() if name.startswith('torso'))
        self.assertAlmostEqual(front, spec.BELLY_X)

    def test_start_scores_zero(self):
        s = Scorer()
        self.assertEqual(s.update(evidence(tray_x_m=spec.TRAY_START_X, left_position=1, right_position=1)), 0.)

    def test_partial_and_full_ladder(self):
        s = Scorer()
        moved = spec.POSITION_X[0] + spec.RELEASE_TRAVEL
        self.assertEqual(s.update(evidence(tray_x_m=moved, left_position=0, right_position=0)), .25)
        self.assertEqual(s.update(evidence(tray_x_m=spec.POSITION_X[1], left_position=2, right_position=2)), .5)
        self.assertEqual(s.update(evidence(hands_clear=False)), .5)
        for _ in range(spec.DWELL_STEPS):
            score = s.update(evidence())
        self.assertEqual(score, 1.)

    def test_one_finger_is_not_locked(self):
        s = Scorer()
        self.assertEqual(s.update(evidence(right_position=0)), .25)

    def test_touching_toddler_zeros_trial_for_good(self):
        s = Scorer()
        s.update(evidence())
        self.assertEqual(s.update(evidence(touching_child=True)), 0.)
        self.assertTrue(s.invalid)
        for _ in range(spec.DWELL_STEPS + 1):
            self.assertEqual(s.update(evidence()), 0.)

    def test_dropped_or_disturbed_scores_zero(self):
        for bad in [dict(tray_z_m=spec.DROP_Z - .01), dict(intact=False), dict(inside_workspace=False)]:
            s = Scorer()
            self.assertEqual(s.update(evidence(**bad)), 0.)
            self.assertTrue(s.invalid)


if __name__ == '__main__':
    unittest.main()
