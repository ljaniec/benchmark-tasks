import json
from pathlib import Path
from unittest import TestCase

import numpy as np

from scorer import offset_deg
from wheel_index_rgb import GREEN_OFFSET_DEG, INDEX_RADIUS_M, INDEX_SIZE_M, INDEX_Y_M, WheelIndexReader, calibrated_lens
from yam_arena.station import top_camera


class IndexReaderTest(TestCase):
    def test_station_stand_stays_away_from_the_new_optical_view(self):
        from inspect_layout import overhead_camera
        from yam_arena.station import camera_stand

        optical = overhead_camera(top_camera())
        mount = overhead_camera(optical, 'station_stand')
        self.assertNotEqual(optical.pos, mount.pos)
        np.testing.assert_allclose(mount.pos, (.94, .42, 1.48))
        mast = camera_stand(mount)['stand_mast']
        np.testing.assert_allclose(mast[2][:2], (.87, .42))
        self.assertEqual(overhead_camera(optical).lens, optical.lens)

    def test_rotating_index_and_stationary_scale_at_policy_resolution(self):
        reader = WheelIndexReader()
        calibration = json.loads(Path(__file__).with_name('camera-projection.json').read_text())['delivery_candidate']
        camera = np.array(calibration['camera_m'])
        aim = camera - calibration['look_at_m']
        aim /= np.linalg.norm(aim)
        right = np.cross((0., 0., 1.), aim)
        right /= np.linalg.norm(right)
        rotation = np.column_stack((right, np.cross(aim, right), aim))
        lens = calibrated_lens(top_camera().lens, calibration).resized(320, 176)
        rows, columns = np.indices(reader.shape, dtype=float)
        pixels = np.stack((columns + .5, rows + .5), axis=-1)

        def projected_patch(angle, radius, y, half_length, half_width):
            theta = np.radians(angle)
            radial = np.array((np.cos(theta), 0., np.sin(theta)))
            tangent = np.array((-np.sin(theta), 0., np.cos(theta)))
            center = np.array((.38, y, .83)) + radius * radial
            vertices = np.array([center + half_length*d*radial + half_width*w*tangent
                                 for d,w in ((-1,-1),(1,-1),(1,1),(-1,1))])
            local = (vertices - camera) @ rotation
            poly = np.column_stack((lens.cx-lens.fx*local[:,0]/local[:,2], lens.cy+lens.fy*local[:,1]/local[:,2]))
            crosses = []
            for i, start in enumerate(poly):
                dx, dy = poly[(i + 1) % 4] - start
                delta = pixels - start
                crosses.append(dx * delta[..., 1] - dy * delta[..., 0])
            winding = np.asarray(crosses)
            return (winding >= 0.).all(axis=0) | (winding <= 0.).all(axis=0)

        scale = np.tile(np.array((55, 65, 62), dtype=np.uint8), (*reader.shape, 1))
        for mark in (0, 45, 90, 135):
            scale[projected_patch(mark, .352, .0886, .0115, .004)] = (244, 220, 98)
        self.assertIsNone(reader.angle_deg(scale))
        for angle in range(0, 360, 15):
            patch = projected_patch(angle, INDEX_RADIUS_M, INDEX_Y_M + INDEX_SIZE_M[1] / 2.,
                                    INDEX_SIZE_M[0] / 2., INDEX_SIZE_M[2] / 2.)
            for color in ((233, 67, 55), (69, 22, 18), (22, 3, 5)):
                frame = scale.copy()
                frame[patch] = color
                seen = reader.angle_deg(frame)
                self.assertIsNotNone(seen, angle)
                self.assertLess(abs(offset_deg(seen, angle)), .5)
            green = projected_patch(angle + GREEN_OFFSET_DEG, INDEX_RADIUS_M,
                                    INDEX_Y_M + INDEX_SIZE_M[1] / 2., INDEX_SIZE_M[0] / 2., INDEX_SIZE_M[2] / 2.)
            for green_color in ((55, 233, 67), (18, 69, 22), (5, 22, 3)):
                frame = scale.copy()
                frame[green] = green_color
                self.assertIsNone(reader.angle_deg(frame))
                selected = reader.route_angle_deg(frame)
                self.assertIsNotNone(selected, angle)
                self.assertLess(abs(offset_deg(selected, angle)), .5)
