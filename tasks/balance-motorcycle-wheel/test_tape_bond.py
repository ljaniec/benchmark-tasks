from unittest import TestCase

import numpy as np

from tape_bond import TapeBond


class BondTests(TestCase):
    def test_compression_does_not_break_a_good_bond(self):
        amid = np.zeros(3)
        seated = np.array([.23, .0505, 0.])
        orientation = np.eye(3)
        still = np.zeros(6)
        bond = TapeBond.at_contact(amid, orientation, seated, orientation, 1.)
        force, torque = bond.reaction(amid, orientation, still, seated - [0., .006, 0.], orientation, still)
        np.testing.assert_array_equal(force, amid)
        np.testing.assert_array_equal(torque, amid)

    def test_small_separation_resists_magnet_but_excess_pull_breaks(self):
        amid = np.zeros(3)
        seated = np.array([.23, -.0505, 0.])
        orientation = np.eye(3)
        still = np.zeros(6)
        bond = TapeBond.at_contact(amid, orientation, seated, orientation, -1.)
        force, _ = bond.reaction(amid, orientation, still, seated - [0., .001, 0.], orientation, still)
        np.testing.assert_allclose(force, [0., 1.2, 0.])
        self.assertIsNone(bond.reaction(amid, orientation, still, seated - [0., .006, 0.], orientation, still))
