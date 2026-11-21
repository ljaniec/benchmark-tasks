"""Mount and tool dimensions shared by the physical scene and fixed demonstration."""
import math

import numpy as np
from yam_arena.urdf import axis_angle_matrix

from preflight import REACH


TABLE = .7475
PIVOT_Z = 1.035
YAW_DEG = -35.
BASE = (.410, -.064, 0.)
PANEL = (.535, .035, .305)
SOCKET_DEPTH = .014
SOCKET_CENTER_Y = -.024
KEY_TIP_Y = .086
KEY_START = np.array([.260, -.223, TABLE + .130])
KEY_HANDLE_Y = -.006
KEY_REST_ROTATION = axis_angle_matrix(np.array([1., 0., 0.]), -math.pi / 2)
MOUNT_ROTATION = axis_angle_matrix(np.array([0., 0., 1.]), math.radians(YAW_DEG))
KEY_FRAME = np.array([[1., 0., 0.], [0., 0., 1.], [0., -1., 0.]])
REST_LIFT_DEG = -21.
