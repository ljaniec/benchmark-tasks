from unittest import TestCase

from scorer import END_DWELL_STEPS, HOLD_STEPS, MARKS_DEG, RELEASE_STEPS, Reading, WheelScore


def release(score, mark, drift, steps=RELEASE_STEPS):
    for _ in range(HOLD_STEPS):
        score.update(Reading(mark, True, True, False))
    score.update(Reading(mark, True, True, True))
    for tick in range(1, steps + 1):
        progress = min(1., tick / RELEASE_STEPS)
        value = score.update(Reading(mark + progress * drift, True, True, True))
    return value


class WheelScoreTest(TestCase):
    def test_four_distinct_released_checks_and_final_dwell(self):
        score = WheelScore()
        for mark in MARKS_DEG[:-1]:
            self.assertLess(release(score, mark, 1.), .5)
        self.assertEqual(release(score, MARKS_DEG[-1], 1.), .5)
        for index in range(END_DWELL_STEPS - 1):
            self.assertEqual(score.update(Reading(136., True, True, True)), .5, index)
        self.assertEqual(score.update(Reading(136., True, True, True)), 1.)

    def test_repeated_equilibrium_cannot_replace_other_marks(self):
        score = WheelScore()
        for _ in MARKS_DEG:
            self.assertLess(release(score, 90., 0., RELEASE_STEPS + END_DWELL_STEPS), .5)

    def test_balance_credit_requires_a_current_completed_release(self):
        score = WheelScore()
        for mark in MARKS_DEG:
            result = release(score, mark, 1.)
        self.assertEqual(result, .5)
        self.assertEqual(score.update(Reading(136., True, True, False)), 0.)
        self.assertEqual(score.update(Reading(136., True, True, True)), .25)
        self.assertLess(release(score, 135., 1., steps=RELEASE_STEPS - 1), .5)
        self.assertEqual(score.update(Reading(136., True, True, True)), .5)

    def test_large_drift_and_reinstalled_strip(self):
        score = WheelScore()
        for mark in MARKS_DEG[:-1]:
            release(score, mark, 1.)
        self.assertEqual(release(score, 135., 10., RELEASE_STEPS + END_DWELL_STEPS), .5)
        self.assertEqual(score.update(Reading(145., True, False, True)), 0.)
        self.assertEqual(score.update(Reading(145., True, True, True)), 0.)

    def test_previous_check_is_measured_only_in_its_four_and_a_half_second_window(self):
        score = WheelScore()
        release(score, MARKS_DEG[0], 1.)
        for _ in range(15):
            score.update(Reading(7., True, True, True))
        self.assertEqual(score.checks[0], 1.)
        for mark in MARKS_DEG[1:]:
            result = release(score, mark, 1., RELEASE_STEPS + END_DWELL_STEPS)
        self.assertEqual(result, 1.)

    def test_last_check_must_stay_within_bounds_for_its_final_dwell(self):
        score = WheelScore()
        for mark in MARKS_DEG:
            result = release(score, mark, 1.)
        self.assertEqual(result, .5)
        for _ in range(END_DWELL_STEPS):
            result = score.update(Reading(MARKS_DEG[-1] + 6., True, True, True))
        self.assertEqual(result, .5)

    def test_occlusion_interrupts_current_measurement_and_allows_retest(self):
        score = WheelScore()
        for mark in MARKS_DEG:
            result = release(score, mark, 1.)
        self.assertEqual(result, .5)
        self.assertEqual(score.update(Reading(None, True, True, True)), 0.)
        for _ in range(RELEASE_STEPS):
            self.assertLess(score.update(Reading(136., True, True, True)), .5)
        self.assertEqual(release(score, 135., 1., RELEASE_STEPS + END_DWELL_STEPS), 1.)

    def test_invalidation_persists_and_reset_clears_witnesses(self):
        score = WheelScore()
        for mark in MARKS_DEG:
            result = release(score, mark, 0., RELEASE_STEPS + END_DWELL_STEPS)
        self.assertEqual(result, 1.)
        self.assertEqual(score.update(Reading(135., True, True, True, invalid=True)), 0.)
        self.assertEqual(score.update(Reading(135., True, True, True)), 0.)
        self.assertEqual(WheelScore().update(Reading(135., True, True, True)), 0.)
