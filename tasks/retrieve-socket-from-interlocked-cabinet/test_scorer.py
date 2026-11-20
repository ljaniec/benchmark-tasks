"""Rubric regressions: outcomes, shortcuts, dwell and independent reset."""
import unittest
from dataclasses import replace
from scorer import Evidence,Scorer

class Rubric(unittest.TestCase):
    def setUp(self):
        self.e=Evidence(.2,0.,False,True,True,True,True,0.)
    def test_no_action(self):
        self.assertEqual(Scorer().update(self.e),0.)
    def test_partial_current_state(self):
        s=Scorer();self.assertEqual(s.update(replace(self.e,lower_open_m=.004)),.25)
        self.assertEqual(s.update(replace(self.e,lower_open_m=0.,upper_open_m=.201)),.5)
        self.assertEqual(s.update(self.e),0.)
    def test_ten_consecutive_released_steps(self):
        s=Scorer();e=replace(self.e,lower_open_m=0.,upper_open_m=.25,socket_parked=True)
        for _ in range(9):self.assertEqual(s.update(e),.5)
        self.assertEqual(s.update(e),1.)
    def test_contact_and_motion_interrupt(self):
        for field in ['hands_clear','stable','socket_parked']:
            s=Scorer();e=replace(self.e,upper_open_m=.25,socket_parked=True)
            for _ in range(9):s.update(e)
            self.assertEqual(s.update(replace(e,**{field:False})),.5)
            self.assertEqual(s.dwell,0)
    def test_invalid_is_permanent(self):
        for change in [{'inside_workspace':False},{'intact':False},{'chassis_tilt_deg':15.01},{'upper_open_m':float('nan')}]:
            s=Scorer();self.assertEqual(s.update(replace(self.e,**change)),0.)
            for _ in range(20):self.assertEqual(s.update(replace(self.e,socket_parked=True)),0.)
            self.assertTrue(s.invalid)
    def test_new_episode_clean(self):
        s=Scorer();s.update(replace(self.e,intact=False));new=Scorer()
        self.assertFalse(new.invalid);self.assertEqual(new.dwell,0);self.assertEqual(new.update(self.e),0.)
    def test_hovering_and_wrong_target(self):
        s=Scorer()
        for _ in range(20):self.assertEqual(s.update(replace(self.e,upper_open_m=.25)),.5)

if __name__=='__main__':unittest.main()
