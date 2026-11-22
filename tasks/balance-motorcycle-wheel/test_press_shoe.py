from unittest import TestCase

import numpy as np

from press_shoe import MAGNET_HOLD_N, MAGNET_REACH_M, magnetic_reaction, shoe_boxes
from probe_strip_press import RADIAL_M, TANGENT_M, THICK_M


class ApplicatorTests(TestCase):
    def test_face_loads_the_back_without_extending_past_the_tape(self):
        _, center, extent = shoe_boxes()[0]
        self.assertAlmostEqual(center[1] - extent[1] / 2., THICK_M / 2.)
        self.assertGreaterEqual(extent[0], RADIAL_M)
        self.assertGreaterEqual(extent[2], TANGENT_M)

    def test_magnet_can_release_a_bonded_strip(self):
        zero, rotation = np.zeros(3), np.eye(3)
        np.testing.assert_allclose(magnetic_reaction(zero, rotation, zero, rotation), [0., MAGNET_HOLD_N, 0.])
        intermediate = magnetic_reaction(np.array([0., -.5 * MAGNET_REACH_M, 0.]), rotation, zero, rotation)
        self.assertGreater(intermediate[1], 0.)
        self.assertLess(intermediate[1], MAGNET_HOLD_N)
        distant = magnetic_reaction(np.array([0., -MAGNET_REACH_M, 0.]), rotation, zero, rotation)
        np.testing.assert_array_equal(distant, zero)
        reversed_strip = magnetic_reaction(zero, np.diag([1., -1., -1.]), zero, rotation)
        np.testing.assert_array_equal(reversed_strip, zero)
