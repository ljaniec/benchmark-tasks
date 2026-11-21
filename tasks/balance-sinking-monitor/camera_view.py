"""Mount an overhead view with room for both screen heights."""
from dataclasses import replace
from itertools import product
import math

import numpy as np
from yam_arena import ik, station
from yam_arena.urdf import quat_xyzw_to_matrix

from geometry import BASE, KEY_START, MOUNT_ROTATION, PANEL, PIVOT_Z, REACH


POSITION_M = (-.40, 0., 2.12)
AIM_M = (.40, 0., .95)


def mount(camera):
    backwards = np.array(POSITION_M) - AIM_M
    backwards /= np.linalg.norm(backwards)
    right = np.cross([0., 0., 1.], backwards)
    right /= np.linalg.norm(right)
    up = np.cross(backwards, right)
    rotation = np.column_stack((right, up, backwards))
    return replace(camera, pos=POSITION_M, rot_xyzw=tuple(ik.matrix_to_quat_xyzw(rotation)))


def configure():
    if getattr(station.top_camera, "monitor_mount", False):
        return
    original = station.top_camera

    def top_camera(variant="calibrated"):
        return mount(original(variant))

    top_camera.monitor_mount = True
    station.top_camera = top_camera


def check_framing():
    camera = mount(station.top_camera())
    lens = camera.lens.resized(320, 176)
    points = [KEY_START]
    for tilt in (-21., 0., 12.):
        angle = math.radians(tilt)
        center = np.array([0., REACH * math.cos(angle), PIVOT_Z + REACH * math.sin(angle)])
        for signs in product((-1., 1.), repeat=3):
            coordinates = center + np.asarray(signs) * np.asarray(PANEL) / 2.
            points.append(np.asarray(BASE) + MOUNT_ROTATION @ coordinates)
    local = (np.asarray(points) - camera.pos) @ quat_xyzw_to_matrix(np.array(camera.rot_xyzw))
    depth = -local[:, 2]
    image = np.column_stack((lens.cx + lens.fx * local[:, 0] / depth,
                             lens.cy - lens.fy * local[:, 1] / depth))
    assert np.all(depth > 0.) and np.all(image >= 8.) and np.all(image <= [312., 168.]), image
    return [image.min(axis=0).tolist(), image.max(axis=0).tolist()]


if __name__ == "__main__":
    print(check_framing())
