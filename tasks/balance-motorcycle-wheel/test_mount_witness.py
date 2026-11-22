from unittest import TestCase

import numpy as np

from mount_witness import seated_on_rim
from wheel_readout import occupied_faces
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix


class MountWitnessTests(TestCase):
    def test_footprint_is_flush_on_the_correct_face(self):
        axle = np.array([.38, 0., .83])
        wheel_rotation = np.eye(3)
        angle = np.radians(4.9)
        tilt = np.array([[1., 0., 0.], [0., np.cos(angle), -np.sin(angle)],
                         [0., np.sin(angle), np.cos(angle)]])
        for side in (1., -1.):
            strip = axle + np.array([.230, side * .0505, 0.])
            rotation = np.diag([1., side, side])
            self.assertTrue(seated_on_rim(axle, wheel_rotation, strip, rotation, side))
            self.assertFalse(seated_on_rim(axle, wheel_rotation, strip, rotation, -side))
            for shift in ((0., -.0028 * side, 0.), (0., .002 * side, 0.), (.016, 0., 0.)):
                self.assertFalse(seated_on_rim(axle, wheel_rotation, strip + shift, rotation, side))
            lifted = strip + [0., .001 * side, 0.]
            self.assertFalse(seated_on_rim(axle, wheel_rotation, lifted, rotation @ tilt, side))

    def test_compact_strip_fits_at_the_measured_landing_radius(self):
        axle = np.array([.38, 0., .83])
        rotation = np.eye(3)
        for side in (1., -1.):
            strip = axle + np.array([.232, side * .0505, 0.])
            face = np.diag([1., side, side])
            for angle in np.linspace(0., 2. * np.pi, 72, endpoint=False):
                turned = axis_angle_matrix(np.array([0., 1., 0.]), angle) @ face
                self.assertTrue(seated_on_rim(axle, rotation, strip, turned, side))

    def test_identical_strips_may_trade_sides_but_nearby_tools_do_not_pass(self):
        axle = np.array([.38, 0., .83])
        strips = np.array([[.613, -.0505, .83, 1., 0., 0., 0.],
                           [.613, .0505, .83, 0., 0., 0., 1.]])
        shoes = np.array([[.25, .25, .83], [.25, -.25, .83]])
        self.assertEqual(occupied_faces(axle, np.eye(3), strips, shoes, quat_xyzw_to_matrix), (True, True))
        shoes[0] = strips[1, :3] + [.003, 0., 0.]
        self.assertEqual(occupied_faces(axle, np.eye(3), strips, shoes, quat_xyzw_to_matrix), (False, True))
