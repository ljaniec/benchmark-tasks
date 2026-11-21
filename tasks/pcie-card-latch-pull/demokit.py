"""Minimal demo helper: allow Waypoint.yaw to carry a full 3x3 TCP rotation matrix.

Copied from the pattern used by accepted bodykit tasks (allow_matrix_grips). Importing this
module patches yam_arena.demo.grip_rotation so ArmPlan and yam_scripted can follow pitched grips.
"""

from __future__ import annotations

import numpy as np


def allow_matrix_grips() -> None:
    from yam_arena import demo

    if getattr(demo, "_task_matrix_grips", False):
        return
    plain = demo.grip_rotation

    def grip_rotation(yaw=0.0):
        return np.asarray(yaw, float) if np.ndim(yaw) == 2 else plain(yaw)

    demo.grip_rotation = grip_rotation
    demo._task_matrix_grips = True


allow_matrix_grips()
