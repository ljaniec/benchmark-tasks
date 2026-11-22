"""Measure the filmed wheel, installed weights and hands-off clearance."""

from itertools import product
from math import cos, radians

import numpy as np

from mount_witness import seated_on_rim
from press_shoe import shoe_boxes
from scorer import Reading
from wheel_index_rgb import WheelIndexReader


AXLE_M = np.array((.38, 0., .83))
TIP_LIMIT = cos(radians(15.))
WORKSPACE_MIN = np.array((-.025, -.43, .415))
WORKSPACE_MAX = np.array((.91, .43, 1.34))
WHEEL_BOUNDS = ((.326, .060), (.153, .073))
HANDS_OFF_M = .020
BOX_CORNERS = np.array(list(product((-1., 1.), repeat=3)))


def wheel_clearance(points, center, rotation):
    """Conservative separation from the tire, rotor and hub envelopes."""
    local = (points - center) @ rotation
    minimum, maximum = local.min(axis=0), local.max(axis=0)
    axis_gap = np.maximum(0., np.maximum(minimum, -maximum))
    radius = np.hypot(axis_gap[0], axis_gap[2])
    return min(max(radius - edge, axis_gap[1] - half_width) for edge, half_width in WHEEL_BOUNDS)


def occupied_faces(center, wheel_rotation, strips, applicator_positions, to_rotation):
    """Judge the two rim faces independently of which identical strip was used."""
    uncovered = [(strip[:3], to_rotation(strip[3:])) for strip in strips
                 if all(np.linalg.norm(strip[:3] - shoe) >= .035 for shoe in applicator_positions)]
    return tuple(any(seated_on_rim(center, wheel_rotation, pos, rotation, sign)
                     for pos, rotation in uncovered) for sign in (1., -1.))


class WheelReadout:
    def __init__(self):
        from yam_arena.analysis import link_hulls

        self.reader = WheelIndexReader()
        self.arm_hulls = {name: [hull.vertices for hull in parts] for name, parts in link_hulls().items()}
        self.shoe_corners = [np.asarray(center) + BOX_CORNERS * (np.asarray(size) / 2.)
                             for _, center, size in shoe_boxes()]

    def observe(self, base):
        import cv2
        from yam_arena.runtime import link_poses

        physics = base.wheel_physics
        poses = physics.poses()
        if not np.isfinite(poses).all():
            return Reading(None, False, False, False, invalid=True)
        center = poses[0, :3]
        rotation = physics.rotation(poses[0, 3:])
        moved = np.linalg.norm(center - AXLE_M) > .005 or rotation[1, 1] < TIP_LIMIT
        outside = np.any(poses[1:, :3] < WORKSPACE_MIN) or np.any(poses[1:, :3] > WORKSPACE_MAX)
        if moved or outside:
            return Reading(None, False, False, False, invalid=True)

        hands_and_tools = []
        for side in ('left', 'right'):
            for link, pose in link_poses(base.scene[f'{side}_robot']).items():
                hands_and_tools.extend(vertices @ pose[:3, :3].T + pose[:3, 3]
                                       for vertices in self.arm_hulls.get(link, []))
        for shoe in poses[3:]:
            orient = physics.rotation(shoe[3:])
            hands_and_tools.extend(vertices @ orient.T + shoe[:3] for vertices in self.shoe_corners)
        hands_clear = all(wheel_clearance(points, center, rotation) >= HANDS_OFF_M for points in hands_and_tools)
        attached = occupied_faces(center, rotation, poses[1:3], poses[3:, :3], physics.rotation)

        frame = base.scene['top_cam'].data.output['rgb']
        frame = getattr(frame, 'torch', frame)[0, ..., :3].cpu().numpy()
        angle = self.reader.angle_deg(cv2.resize(frame, (320, 176)))
        return Reading(angle, *attached, hands_clear)
