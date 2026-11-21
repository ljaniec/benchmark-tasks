"""Optional wide overhead calibration for a 1.86 m bicycle."""
from dataclasses import replace
from itertools import product

import numpy as np
from yam_arena import ik, station
from yam_arena.urdf import quat_xyzw_to_matrix

POSITION_M = (-.46, 0., 2.45)
AIM_M = (.39, 0., .76)


def wide_camera(base):
    position = np.array(POSITION_M)
    backwards = position - AIM_M
    backwards /= np.linalg.norm(backwards)
    right = np.cross([0., 0., 1.], backwards)
    right /= np.linalg.norm(right)
    up = np.cross(backwards, right)
    orientation = np.column_stack((right,up,backwards))
    return replace(base, pos=POSITION_M, rot_xyzw=tuple(ik.matrix_to_quat_xyzw(orientation)))


def configure():
    """Keep both native wrist camera poses unchanged."""
    original = station.top_camera

    def top_camera(variant='calibrated'):
        return wide_camera(original(variant))

    station.top_camera = top_camera


def check_framing():
    camera = wide_camera(station.top_camera())
    lens = camera.lens
    rotation = quat_xyzw_to_matrix(np.asarray(camera.rot_xyzw))
    bounds = np.array(list(product((.10,.72),(-.94,.94),(.44,1.31))))
    local = (bounds - camera.pos) @ rotation
    distance = -local[:,2]
    image = np.column_stack((lens.cx + lens.fx*local[:,0]/distance,
                             lens.cy - lens.fy*local[:,1]/distance))
    assert np.all(image>=15.) and np.all(image<=[lens.width-15.,lens.height-15.]),image
    return np.round([image.min(axis=0), image.max(axis=0)],1)


if __name__=='__main__':
    print(check_framing())
