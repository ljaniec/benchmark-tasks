"""Contract regressions: incomplete seating, release, dwell and invalid shortcuts."""
from dataclasses import replace
import unittest
from scorer import Evidence,Scorer
START=Evidence(False,False,False,True,0.,True,True,False)
ENTERED=replace(START,entered=True,hands_clear=False)
SEATED=replace(ENTERED,seated=True)
FULL=replace(SEATED,pressing=True,hands_clear=True)
class ContractTests(unittest.TestCase):
    def test_start_and_hover_get_zero(self):
        s=Scorer()
        for _ in range(30):self.assertEqual(s.update(START),0.)
        self.assertEqual(s.update(replace(START,pressing=True)),0.)
    def test_stages(self):
        s=Scorer();self.assertEqual(s.update(ENTERED),.25);self.assertEqual(s.update(SEATED),.5)
    def test_finger_must_contact_top(self):
        s=Scorer()
        for _ in range(20):self.assertEqual(s.update(replace(FULL,pressing=False)),.5)
    def test_release_required(self):
        s=Scorer()
        for _ in range(20):self.assertEqual(s.update(replace(FULL,hands_clear=False)),.5)
    def test_ten_consecutive_steps(self):
        s=Scorer()
        for _ in range(9):self.assertEqual(s.update(FULL),.5)
        self.assertEqual(s.update(FULL),1.)
        self.assertEqual(s.update(SEATED),.5)
        for _ in range(9):self.assertEqual(s.update(FULL),.5)
        self.assertEqual(s.update(FULL),1.)
    def test_no_historical_seating_credit(self):
        s=Scorer()
        for _ in range(10):s.update(FULL)
        self.assertEqual(s.update(START),0.)
    def test_invalid_latches_zero(self):
        for change in [dict(chassis_tilt_deg=15.01),dict(chassis_tilt_deg=float('nan')),dict(inside_workspace=False),dict(intact=False),dict(shortcut=True)]:
            with self.subTest(change=change):
                s=Scorer()
                for _ in range(10):s.update(FULL)
                self.assertEqual(s.update(replace(FULL,**change)),0.)
                for _ in range(20):self.assertEqual(s.update(FULL),0.)
    def test_reset_empty(self):
        s=Scorer();s.update(replace(FULL,shortcut=True));s=Scorer()
        self.assertEqual((s.score,s.dwell,s.invalid),(0.,0,False))
if __name__=='__main__':unittest.main()
