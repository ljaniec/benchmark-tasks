import math
import unittest
from wheel_support import clearance,support_gap
import spec


def pose(x,z,y=0.):return [x,y,z,math.sqrt(.5),0.,0.,math.sqrt(.5)]

class WheelSupport(unittest.TestCase):
    def test_floor_sill_and_corner_allow_normal_rolling(self):
        for x,z in [(.455,spec.WHEEL_RADIUS),(.380,spec.WHEEL_RADIUS+spec.SILL_HEIGHT),
                    (.420,spec.SILL_HEIGHT+math.sqrt(spec.WHEEL_RADIUS**2-.030**2))]:
            self.assertLess(clearance(pose(x,z)),.0005)
    def test_carried_wheel_has_visible_gap(self):
        self.assertAlmostEqual(clearance(pose(.550,spec.WHEEL_RADIUS+.020)),.020,places=5)
    def test_one_wheel_support_allows_normal_rocking(self):
        self.assertLess(support_gap({'wheel_left':pose(.55,spec.WHEEL_RADIUS,.1955),
                                    'wheel_right':pose(.55,spec.WHEEL_RADIUS+.010,-.1955)}),.0005)
    def test_both_wheels_airborne_fail(self):
        self.assertGreater(support_gap({'wheel_left':pose(.55,spec.WHEEL_RADIUS+.01,.1955),
                                       'wheel_right':pose(.55,spec.WHEEL_RADIUS+.02,-.1955)}),spec.WHEEL_SUPPORT_GAP)

if __name__=='__main__':unittest.main()
