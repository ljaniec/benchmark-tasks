"""Read the visible tire-sidewall index in the fixed overhead camera's image."""

import json
from dataclasses import replace
from pathlib import Path

import numpy as np

INDEX_RADIUS_M = .286
INDEX_Y_M = .060
INDEX_SIZE_M = (.043, .001, .022)
GREEN_OFFSET_DEG = 90.


def calibrated_lens(lens, calibration):
    return replace(lens, **calibration.get('native_lens', {}))


class WheelIndexReader:
    def __init__(self, size=(320, 176), calibration=None):
        from yam_arena.station import top_camera

        width, height = size
        if calibration is None:
            source = Path(__file__).with_name('camera-projection.json')
            calibration = json.loads(source.read_text())['delivery_candidate']
        lens = calibrated_lens(top_camera().lens, calibration).resized(width, height)
        position = np.asarray(calibration['camera_m'])
        orientation = np.asarray(calibration['camera_axes_columns'])
        v, u = np.indices((height, width), dtype=float)
        rays = np.stack(((u + .5 - lens.cx) / lens.fx, -(v + .5 - lens.cy) / lens.fy, -np.ones_like(u)), axis=-1)
        rays = rays @ orientation.T
        depth = np.divide(INDEX_Y_M + INDEX_SIZE_M[1] / 2. - position[1], rays[..., 1], out=np.full_like(u, np.nan),
                          where=np.abs(rays[..., 1]) > 1e-9)
        world = position + depth[..., None] * rays
        x, z = world[..., 0] - .38, world[..., 2] - .83
        radius = np.hypot(x, z)
        self.shape = (height, width)
        self.index_band = (depth > 0.) & (np.abs(radius - INDEX_RADIUS_M) <= .034)
        self.angle = np.arctan2(z, x)

    def angle_deg(self, rgb):
        return self._projected_angle(rgb, 0)

    def route_angle_deg(self, rgb):
        """Use the second painted index when the hands conceal the red one."""
        red = self.angle_deg(rgb)
        if red is not None:
            return red
        green = self._projected_angle(rgb, 1)
        return None if green is None else float((green - GREEN_OFFSET_DEG) % 360.)

    def _projected_angle(self, rgb, channel):
        pixels = np.asarray(rgb)
        if pixels.ndim != 3 or pixels.shape[:2] != self.shape or pixels.shape[2] < 3:
            raise ValueError('Expected the configured overhead RGB image')
        colors = np.moveaxis(pixels[..., :3].astype(np.float32), -1, 0)
        index = colors[channel]
        other = np.maximum(colors[(channel + 1) % 3], colors[(channel + 2) % 3])
        lit = (index >= 44.) & (index - other >= 30.) & (index >= 1.65 * other)
        shaded = (index >= 18.) & (index - other >= 11.) & (index >= 3. * other)
        angles = self.angle[self.index_band & (lit | shaded)]
        if len(angles) < 4:
            return None
        x, z = np.cos(angles).mean(), np.sin(angles).mean()
        if np.hypot(x, z) < .98:
            return None
        return float(np.degrees(np.arctan2(z, x)) % 360.)
