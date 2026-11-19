"""Task-contract regression tests; these do not establish robot feasibility."""
import unittest
from dataclasses import replace
from scorer import Evidence, Scorer
from spec import finger_joint_position


class ContractTests(unittest.TestCase):
    def test_yam_aperture_direction_regression(self):
        self.assertEqual(finger_joint_position(0.),0.)
        self.assertAlmostEqual(finger_joint_position(.05),-.025)
        self.assertAlmostEqual(finger_joint_position(.075),-.03745)
        for aperture in [-.01,.076,float('nan')]:
            with self.assertRaises(ValueError):finger_joint_position(aperture)

    def setUp(self):
        self.s=Scorer()
        self.full=Evidence(True,True,True,True,0.)

    def test_no_action_and_milestones(self):
        self.assertEqual(self.s.update(Evidence()),0.)
        self.assertEqual(self.s.update(Evidence(restraint_open=True)),.25)
        self.assertEqual(self.s.update(Evidence(restraint_open=True,battery_clear=True)),.5)

    def test_exact_ten_step_dwell(self):
        for _ in range(9):self.assertEqual(self.s.update(self.full),.5)
        self.assertEqual(self.s.update(self.full),1.)

    def test_interrupted_dwell_and_terminal_loss(self):
        for _ in range(9):self.s.update(self.full)
        self.s.update(replace(self.full,hands_clear=False))
        for _ in range(9):self.assertEqual(self.s.update(self.full),.5)
        self.assertEqual(self.s.update(self.full),1.)
        self.assertEqual(self.s.update(replace(self.full,battery_on_mat=False)),.5)

    def test_tip_boundary_and_permanent_zero(self):
        self.assertEqual(self.s.update(replace(self.full,module_tilt_deg=15)),.5)
        self.assertEqual(self.s.update(replace(self.full,module_tilt_deg=15.001)),0.)
        for _ in range(11):self.assertEqual(self.s.update(self.full),0.)

    def test_bounds_damage_shortcuts_and_nonfinite(self):
        for bad in [replace(self.full,within_workspace=False),replace(self.full,intact=False),
                    replace(self.full,shortcut=True),replace(self.full,module_tilt_deg=float('nan'))]:
            self.s.reset();self.assertEqual(self.s.update(bad),0.)
            self.assertEqual(self.s.update(self.full),0.)

    def test_release_stability_and_prerequisites(self):
        for bad in [replace(self.full,hands_clear=False),replace(self.full,battery_motion_mm=1.001),
                    replace(self.full,battery_motion_mm=float('nan')),replace(self.full,restraint_open=False)]:
            self.s.reset()
            for _ in range(12):self.assertLess(self.s.update(bad),1.)

    def test_reset_clears_all_scoring_state(self):
        self.s.update(replace(self.full,shortcut=True));self.s.reset()
        self.assertFalse(self.s.invalid);self.assertEqual(self.s.dwell,0)
        self.assertEqual(self.s.score,0.)
        for _ in range(10):self.s.update(self.full)
        self.assertEqual(self.s.score,1.)
        self.s.reset();self.assertEqual(self.s.update(Evidence()),0.)


if __name__=='__main__':unittest.main()
