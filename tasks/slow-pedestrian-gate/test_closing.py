"""Test the timing, latch dwell and bypass gates on CPU."""

import unittest

from closing_model import passive_release
from scorer import ClosingScore


class ClosingTests(unittest.TestCase):
    def setUp(self):
        self.scorer = ClosingScore()

    def tick(self, angle, *, held=False, clear=True, latched=False, intact=True):
        return self.scorer.tick(angle_deg=angle, holder_engaged=held,
                                sweep_clear=clear, latch_engaged=latched,
                                fixture_intact=intact)

    def sweep(self, controls, *, assisted=None):
        self.tick(90, held=True)
        self.tick(90)
        for i in range(1, controls):
            self.tick(90 - 78 * i / controls, clear=i != assisted)
        return self.tick(12)

    def test_hydraulic_adjustment_slows_the_full_size_door(self):
        fast = passive_release(0)
        tuned = passive_release(0.5)
        assert fast.sweep_seconds is not None and tuned.sweep_seconds is not None
        self.assertLess(fast.sweep_seconds, 4.0)
        self.assertGreaterEqual(tuned.sweep_seconds, 5.0)
        self.assertLessEqual(tuned.sweep_seconds, 8.0)

    def test_latch_requires_ten_steps_and_must_stay_engaged(self):
        self.assertEqual(self.sweep(118), 0.5)
        for _ in range(9):
            self.assertEqual(self.tick(0, latched=True), 0.5)
        self.assertEqual(self.tick(0, latched=True), 1.0)
        self.assertEqual(self.tick(3, latched=False), 0.5)

    def test_unadjusted_closer_and_hand_braking_get_zero(self):
        self.assertEqual(self.sweep(45), 0.0)
        self.assertEqual(self.tick(0, latched=True), 0.0)
        self.assertEqual(self.sweep(118, assisted=80), 0.0)
        self.assertEqual(self.tick(0, latched=True), 0.0)

    def test_short_release_and_already_closed_reset_get_zero(self):
        self.assertEqual(self.tick(0, latched=True), 0.0)
        self.tick(90, held=True)
        self.tick(76)
        self.assertEqual(self.tick(0, latched=True), 0.0)

    def test_release_can_cross_hold_tolerance_between_samples(self):
        self.tick(89.534, held=True)
        self.tick(89.192)
        for i in range(1, 121):
            angle = 89.192 + (12.0 - 89.192) * i / 120
            score = self.tick(angle)
        self.assertEqual(score, 0.5)
        for _ in range(10):
            score = self.tick(0, latched=True)
        self.assertEqual(score, 1.0)

    def test_partial_and_late_latch(self):
        self.assertEqual(self.sweep(95), 0.0)
        self.assertEqual(self.tick(0, latched=True), 0.25)
        self.assertEqual(self.sweep(95), 0.0)
        for _ in range(41):
            self.assertEqual(self.tick(12), 0.0)
        self.assertEqual(self.tick(0, latched=True), 0.0)
        self.assertEqual(self.sweep(118), 0.5)
        for _ in range(48):
            self.assertEqual(self.tick(12), 0.5)
        for _ in range(12):
            self.assertEqual(self.tick(0, latched=True), 0.5)

    def test_movement_of_frame_or_holder_invalidates_the_cycle(self):
        self.assertEqual(self.sweep(118), 0.5)
        self.assertEqual(self.tick(10, intact=False), 0.0)
        self.assertEqual(self.tick(0, latched=True), 0.0)
        self.assertEqual(self.sweep(118), 0.0)


if __name__ == "__main__":
    unittest.main()
