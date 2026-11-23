"""CPU-only contract tests for the magnetic fishing game scorer and geometry."""
import math
import unittest
import spec
from scorer import Evidence, Scorer


def evidence(**kwargs):
    base = dict(
        magnet_touched_target=False,
        target_attached=False,
        target_lift_m=0.,
        target_above_rim_m=-spec.POND_WALL_H,
        other_fish_attached=False,
        robot_touched_game=False,
        inside_workspace=True,
        intact=True,
    )
    base.update(kwargs)
    return Evidence(**base)


CAUGHT = dict(target_attached=True, target_lift_m=.10, target_above_rim_m=.06)


class Contract(unittest.TestCase):
    def test_aperture_cap(self):
        self.assertAlmostEqual(spec.finger_joint_position(.075), -.03745)
        with self.assertRaises(ValueError):
            spec.finger_joint_position(.076)

    def test_fish_lie_inside_the_pond_apart(self):
        names = list(spec.FISH)
        back = -spec.FISH_LEN / 2 - spec.TAIL[0] + .004
        for name in names:
            x, y, heading, _ = spec.FISH[name]
            h = math.radians(heading)
            for u in (back, spec.FISH_LEN / 2):
                for v in (-spec.FISH_W / 2, spec.FISH_W / 2):
                    corner = (x + u * math.cos(h) - v * math.sin(h), y + u * math.sin(h) + v * math.cos(h))
                    self.assertLess(math.dist(corner, spec.POND_CENTRE), spec.POND_INNER_R - .010)
        for a in names:
            for b in names:
                if a < b:
                    # Washers further apart than the magnet's reach plus both radii.
                    gap = math.dist(spec.washer_start(a)[:2], spec.washer_start(b)[:2])
                    self.assertGreater(gap, spec.PULL_RANGE + spec.MAGNET_D + spec.WASHER_D)

    def test_handle_overhangs_box_and_magnet_rests_off_the_pond(self):
        handle_end = spec.ROD_START[0] - spec.ROD_LEN / 2
        self.assertLess(handle_end + .030, spec.BOX_CENTRE[0] - spec.BOX_SIZE[0] / 2)
        self.assertGreater(math.dist(spec.MAGNET_START[:2], spec.POND_CENTRE), spec.POND_INNER_R + .05)
        tip = (spec.ROD_START[0] + spec.ROD_LEN / 2, spec.ROD_START[1], spec.ROD_START[2])
        self.assertLess(math.dist(tip, spec.MAGNET_START), spec.CORD_LEN)

    def test_pull_profile(self):
        self.assertAlmostEqual(spec.magnet_pull(0.), spec.PULL)
        self.assertAlmostEqual(spec.magnet_pull(spec.PULL_SCALE), spec.PULL / 8)
        self.assertEqual(spec.magnet_pull(spec.PULL_RANGE), 0.)
        # At contact the magnet holds many times a fish's weight.
        self.assertGreater(spec.PULL, 5 * (spec.FISH_MASS + spec.WASHER_MASS) * 9.81)

    def test_start_scores_zero(self):
        self.assertEqual(Scorer().update(evidence()), 0.)

    def test_partial_and_full_ladder(self):
        s = Scorer()
        self.assertEqual(s.update(evidence(magnet_touched_target=True, target_attached=True)), .25)
        self.assertEqual(s.update(evidence()), .25)
        self.assertEqual(s.update(evidence(target_attached=True, target_lift_m=.02, target_above_rim_m=-.02)), .5)
        self.assertEqual(s.update(evidence()), .5)
        for _ in range(spec.DWELL_STEPS - 1):
            self.assertEqual(s.update(evidence(**CAUGHT)), .5)
        self.assertEqual(s.update(evidence(**CAUGHT)), 1.)
        self.assertEqual(s.update(evidence()), 1.)

    def test_lift_counts_only_on_the_magnet(self):
        s = Scorer()
        self.assertEqual(s.update(evidence(target_lift_m=.05)), 0.)
        self.assertEqual(s.update(evidence(target_lift_m=.05, target_above_rim_m=.06)), 0.)

    def test_dwell_needs_the_red_fish_alone_and_high(self):
        s = Scorer()
        for _ in range(spec.DWELL_STEPS - 1):
            s.update(evidence(**CAUGHT))
        s.update(evidence(**CAUGHT, other_fish_attached=True))
        self.assertEqual(s.dwell, 0)
        for _ in range(spec.DWELL_STEPS - 1):
            s.update(evidence(**CAUGHT))
        s.update(evidence(target_attached=True, target_lift_m=.08, target_above_rim_m=.049))
        self.assertEqual(s.dwell, 0)
        for _ in range(spec.DWELL_STEPS - 1):
            self.assertLess(s.update(evidence(**CAUGHT)), 1.)
        self.assertEqual(s.update(evidence(**CAUGHT)), 1.)

    def test_forbidden_events_zero_trial_for_good(self):
        for bad in [dict(robot_touched_game=True), dict(intact=False), dict(inside_workspace=False),
                    dict(target_lift_m=math.nan)]:
            s = Scorer()
            s.update(evidence(magnet_touched_target=True))
            self.assertEqual(s.update(evidence(**bad)), 0.)
            self.assertTrue(s.invalid)
            for _ in range(spec.DWELL_STEPS + 1):
                self.assertEqual(s.update(evidence(**CAUGHT)), 0.)


if __name__ == '__main__':
    unittest.main()
