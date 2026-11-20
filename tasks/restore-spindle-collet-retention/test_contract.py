"""Regression checks for proof, release, dwell and invalid-event scoring."""
from dataclasses import replace
import unittest
from scorer import Evidence, Scorer

START=Evidence(False,False,True,False,90.,False,True,0.,True,True,False)
FIT=replace(START,entered=True,seated=True,hands_clear=False)
PROOF=replace(FIT,assembly_raised=True,nut_tilt_deg=120.)
PARK=replace(FIT,parked=True,hands_clear=True)

class ContractTests(unittest.TestCase):
    def test_no_credit_for_start_or_hover(self):
        s=Scorer()
        for _ in range(50):self.assertEqual(s.update(START),0.)

    def test_graduated_score_requires_collet_release(self):
        s=Scorer()
        self.assertEqual(s.update(replace(FIT,collet_clear=False)),.25)
        self.assertEqual(s.update(FIT),.5)

    def test_seated_placement_without_proof_is_partial(self):
        s=Scorer()
        for _ in range(20):self.assertEqual(s.update(PARK),.5)

    def test_two_independent_consecutive_dwells(self):
        s=Scorer()
        for _ in range(9):s.update(PROOF)
        self.assertFalse(s.verified)
        s.update(PROOF);self.assertTrue(s.verified)
        for _ in range(9):self.assertEqual(s.update(PARK),.5)
        self.assertEqual(s.update(PARK),1.)

    def test_supporting_collet_cannot_prove_retention(self):
        s=Scorer()
        for _ in range(30):s.update(replace(PROOF,collet_clear=False))
        self.assertFalse(s.verified)
        for _ in range(30):self.assertEqual(s.update(PARK),.5)

    def test_interrupted_proof_and_placement_restart(self):
        s=Scorer()
        for _ in range(9):s.update(PROOF)
        s.update(FIT)
        self.assertEqual(s.proof_steps,0)
        for _ in range(10):s.update(PROOF)
        for _ in range(9):s.update(PARK)
        s.update(replace(PARK,hands_clear=False))
        self.assertEqual(s.dwell,0)
        for _ in range(9):self.assertEqual(s.update(PARK),.5)
        self.assertEqual(s.update(PARK),1.)

    def test_unseating_or_regrasp_erases_proof(self):
        for change in [dict(seated=False),dict(collet_clear=False)]:
            with self.subTest(change=change):
                s=Scorer()
                for _ in range(10):s.update(PROOF)
                s.update(replace(FIT,**change))
                self.assertFalse(s.verified)
                for _ in range(20):self.assertEqual(s.update(PARK),.5)

    def test_invalid_events_latch_zero(self):
        cases=[dict(chassis_tilt_deg=15.01),dict(inside_workspace=False),dict(intact=False),
               dict(shortcut=True),dict(nut_tilt_deg=float('nan')),dict(chassis_tilt_deg=float('inf'))]
        for change in cases:
            with self.subTest(change=change):
                s=Scorer();s.update(FIT)
                self.assertEqual(s.update(replace(PROOF,**change)),0.)
                for _ in range(10):self.assertEqual(s.update(PROOF),0.)
                self.assertTrue(s.invalid)

    def test_new_scorer_has_no_history(self):
        s=Scorer()
        for _ in range(10):s.update(PROOF)
        for _ in range(10):s.update(PARK)
        s=Scorer()
        self.assertEqual((s.score,s.dwell,s.proof_steps,s.verified,s.invalid),(0.,0,0,False,False))

if __name__=='__main__':unittest.main()
