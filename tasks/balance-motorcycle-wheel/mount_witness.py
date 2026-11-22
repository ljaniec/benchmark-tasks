"""Geometric strip-seat checks corresponding to the visible mounting face."""

from dataclasses import dataclass
from math import cos, radians

import numpy as np

from probe_strip_press import FACE_OFFSET_M, RADIAL_M, TANGENT_M, THICK_M


TAPE_CORNERS_M = np.array([
    (r * RADIAL_M / 2., -THICK_M / 2., t * TANGENT_M / 2.)
    for r, t in ((-1., -1.), (-1., 1.), (1., -1.), (1., 1.))
])


@dataclass(frozen=True)
class RimFit:
    face_alignment: float
    max_plane_error_m: float
    inner_corner_radius_m: float
    outer_corner_radius_m: float

    @property
    def seated(self):
        return (self.face_alignment >= cos(radians(5.)) and self.max_plane_error_m <= .0015
                and self.inner_corner_radius_m >= .220 and self.outer_corner_radius_m <= .248)


def rim_fit(wheel_center, wheel_rotation, strip_center, strip_rotation, side):
    """Measure the angle, edge positions and flatness of the tape footprint."""
    alignment = side * (strip_rotation[:, 1] @ wheel_rotation[:, 1])
    world_corners = strip_center + TAPE_CORNERS_M @ strip_rotation.T
    rim_corners = (world_corners - wheel_center) @ wheel_rotation
    depth_error = np.abs(side * rim_corners[:, 1] - FACE_OFFSET_M)
    radius = np.linalg.norm(rim_corners[:, (0, 2)], axis=1)
    return RimFit(float(alignment), float(depth_error.max()), float(radius.min()), float(radius.max()))


def seated_on_rim(wheel_center, wheel_rotation, strip_center, strip_rotation, side):
    return rim_fit(wheel_center, wheel_rotation, strip_center, strip_rotation, side).seated
