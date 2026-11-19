"""Adversarial optical/scoring checks; no simulator or paid API required."""
import unittest
from dataclasses import replace
import numpy as np
from scorer import Evidence,Scorer
import optics,spec


GOOD=Evidence(True,False,True,True,True,True,0.,0.,True,True,False,True)


class ScoreTests(unittest.TestCase):
    def sequence(self,s,e,n=10):
        return [s.update(e) for _ in range(n)]

    def test_full_score_requires_three_consecutive_dwells(self):
        s=Scorer()
        self.assertEqual(s.update(replace(GOOD,clamp_closed=False)),.25)
        self.assertEqual(self.sequence(s,GOOD),[.5]*10)
        blocked=replace(GOOD,beam_on=False,card_blocks=True,card_parked=False)
        self.assertEqual(self.sequence(s,blocked),[0.]*10)
        self.assertEqual(self.sequence(s,GOOD,9),[.5]*9)
        self.assertEqual(s.update(GOOD),1.)
        self.assertEqual(s.update(replace(GOOD,card_parked=False)),.5)
        self.assertEqual(s.update(replace(GOOD,card_released=False)),.5)

    def test_interrupted_alignment_cannot_authorize_block_test(self):
        s=Scorer();self.sequence(s,GOOD,9)
        s.update(replace(GOOD,beam_on=False))
        s.update(GOOD)
        self.sequence(s,replace(GOOD,beam_on=False,card_blocks=True),10)
        self.assertEqual(self.sequence(s,GOOD,10),[.5]*10)

    def test_blocking_with_hand_or_tool_is_not_proof(self):
        for change in [{'hands_clear':False},{'tool_clear':False},{'card_blocks':False}]:
            with self.subTest(change=change):
                s=Scorer();self.sequence(s,GOOD)
                state=replace(GOOD,beam_on=False,card_blocks=True)
                self.sequence(s,replace(state,**change))
                self.assertEqual(self.sequence(s,GOOD),[.5]*10)

    def test_setting_change_clears_completed_proof(self):
        s=Scorer();self.sequence(s,GOOD)
        self.sequence(s,replace(GOOD,beam_on=False,card_blocks=True))
        self.assertEqual(self.sequence(s,replace(GOOD,sensor_yaw_deg=1.1)),[.5]*10)

    def test_failures_zero_partial_credit_and_latch(self):
        for change in [{'inside_workspace':False},{'intact':False},{'shortcut':True},
                       {'chassis_tilt_deg':15.01},{'sensor_yaw_deg':float('nan')}]:
            with self.subTest(change=change):
                s=Scorer();s.update(GOOD)
                self.assertEqual(s.update(replace(GOOD,**change)),0.)
                self.assertEqual(s.update(GOOD),0.)
                self.assertEqual(Scorer().update(GOOD),.5)

    def test_tilt_boundary_is_inclusive(self):
        self.assertEqual(Scorer().update(replace(GOOD,chassis_tilt_deg=15.)),.5)


class OpticsTests(unittest.TestCase):
    def test_initial_misalignment_and_clear_alignment(self):
        from demo import rz
        for a,expected in [(spec.INITIAL_YAW,False),(0.,True)]:
            r=rz(a);origin=np.array(spec.PIVOT)+r@np.array(spec.LENS_LOCAL)
            self.assertEqual(optics.response(origin,r,spec.REFLECTOR,[])['on'],expected)

    def test_opaque_block_suppresses_return(self):
        origin=np.array(spec.PIVOT)+np.array(spec.LENS_LOCAL)
        result=optics.response(origin,np.eye(3),spec.REFLECTOR,[('card',(.39,-.02,.81),(.43,.02,.87))])
        self.assertFalse(result['on']);self.assertEqual(result['geometric_hits'],9)
        self.assertEqual(result['blocked_by'],['card'])

    def test_objects_behind_sensor_or_reflector_do_not_block(self):
        origin=np.array(spec.PIVOT)+np.array(spec.LENS_LOCAL)
        obstacles=[('behind',(.1,-1,0),(.2,1,2)),('beyond',(1.,-1,0),(1.1,1,2))]
        self.assertTrue(optics.response(origin,np.eye(3),spec.REFLECTOR,obstacles)['on'])

    def test_parallel_ray_outside_box(self):
        self.assertFalse(optics.ray_box((0,0,0),(1,0,0),(.2,.1,-1),(.4,.3,1),1.))
        self.assertTrue(optics.ray_box((0,0,0),(1,0,0),(.2,-.1,-1),(.4,.1,1),1.))

if __name__=='__main__':unittest.main()
