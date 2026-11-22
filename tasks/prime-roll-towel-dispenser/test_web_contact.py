import math
import unittest

import numpy as np

from web_contact import FeedScore, NIP_X, OUTLET_Z, Ratchet, WIDTH, reset_path


def path(tip, x=NIP_X):
    centre = np.array([[x, 0., 1.055], [x, 0., .980], [x, 0., tip]])
    return centre[:, None, :] + np.array([[0., WIDTH / 2, 0.], [0., -WIDTH / 2, 0.]])


class FeedTests(unittest.TestCase):
    def test_rolled_leader_does_not_start_engaged(self):
        score = FeedScore()
        reset = reset_path()[:, None, :] + np.array([[0., WIDTH / 2, 0.], [0., -WIDTH / 2, 0.]])
        score.observe(reset, 0., 0., 0., True, False)
        self.assertFalse(score.invalid)
        self.assertEqual(score.score, 0.)
        self.assertGreater(np.linalg.norm(np.diff(reset_path(), axis=0), axis=1).sum(), .28)

    def test_ratchet_returns_to_rest_without_reversing(self):
        clutch = Ratchet()
        self.assertEqual(clutch.roller_speed(math.radians(-1.)), 0.)
        first = clutch.roller_speed(math.radians(-20.))
        self.assertLess(first, 0.)
        self.assertEqual(clutch.roller_speed(math.radians(-15.)), 0.)
        self.assertEqual(clutch.roller_speed(math.radians(-20.)), 0.)
        self.assertEqual(clutch.roller_speed(0.), 0.)
        self.assertAlmostEqual(clutch.roller_speed(math.radians(-20.)), first)

    def test_crossing_nip_is_partial_credit(self):
        score = FeedScore()
        score.observe(path(.913), 0., 0., 0., True, False)
        self.assertEqual(score.score, .25)

    def test_open_loading_arm_clears_feed_credit(self):
        score = FeedScore()
        score.observe(path(.913), 0., 0., 0., True, False)
        score.observe(path(OUTLET_Z - .061), -.4, -4., 4., False, True)
        self.assertEqual(score.score, .5)
        score.observe(path(OUTLET_Z - .061), 0., -4., 4., True, False, nip_closed=False)
        self.assertEqual(score.score, 0.)
        self.assertEqual(score.driven_m, 0.)
        score.observe(path(OUTLET_Z - .061), 0., -4., 4., True, False)
        self.assertEqual(score.score, .25)

    def test_pull_around_rollers_or_turn_empty_rollers(self):
        score = FeedScore()
        score.observe(path(.96), 0., 0., 0., True, False)
        score.observe(path(.76, .37), -.3, -20., 0., True, True)
        self.assertEqual(score.score, 0.)
        score.observe(path(.90), 0., -20., 0., True, False)
        score.observe(path(.90), -.3, -30., 0., True, True)
        self.assertEqual(score.driven_m, 0.)
        score.observe(path(.76), 0., -31., 1., True, False)
        self.assertEqual(score.score, .25)

    def test_spinning_driver_alone_does_not_prove_paper_feed(self):
        score = FeedScore()
        score.observe(path(.91), 0., 0., 0., False, False)
        score.observe(path(.76), -.3, -10., 0., False, True)
        self.assertEqual(score.driven_m, 0.)
        self.assertEqual(score.score, .25)

    def test_physical_feed_and_released_dwell(self):
        score = FeedScore()
        score.observe(path(.91), 0., 0., 0., True, False)
        score.observe(path(OUTLET_Z - .020), -.4, -3., 3., False, True)
        self.assertEqual(score.score, .5)
        score.observe(path(OUTLET_Z - .061), -.45, -6., 6., False, True)
        for _ in range(9):
            score.observe(path(OUTLET_Z - .061), 0., -6., 6., True, False)
            self.assertEqual(score.score, .5)
        score.observe(path(OUTLET_Z - .061), 0., -6., 6., True, False)
        self.assertEqual(score.score, 1.)
        score.observe(path(OUTLET_Z - .061), 0., -6., 6., False, False)
        self.assertEqual(score.score, .5)

    def test_invalid_web_latches_zero(self):
        score = FeedScore()
        score.observe(path(.91), 0., 0., 0., True, False)
        self.assertEqual(score.score, .25)
        bad = path(.91)
        bad[-1, 0, 1] = .150
        score.observe(bad, -.3, -2., 2., True, True)
        score.observe(path(.91), 0., 0., 0., True, False)
        self.assertEqual(score.score, 0.)
        self.assertTrue(score.invalid)


if __name__ == "__main__":
    unittest.main()
