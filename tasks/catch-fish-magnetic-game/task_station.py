"""Unmodified YAM arms on 750 mm pedestals with a right-side view of the arms, the children's table and the fishing game."""
import numpy as np
from yam_arena import station
from yam_arena.rig import Camera, Rig
from yam_arena.ik import matrix_to_quat_xyzw
import spec

BASES = {'left': (0., .305, .75), 'right': (0., -.305, .75)}


def rig():
    p = np.array([.62, -1.60, 1.22])
    target = np.array([.45, -.32, .74])
    z = (p - target) / np.linalg.norm(p - target)
    x = np.cross([0., 0., 1.], z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    camera = Camera(tuple(p), tuple(matrix_to_quat_xyzw(np.column_stack([x, y, z]))),
                    'opengl', station.lens(station.TOP_CAM_K))
    return Rig('fishing', BASES, camera, {s: station.wrist_camera(s) for s in BASES}, True)
