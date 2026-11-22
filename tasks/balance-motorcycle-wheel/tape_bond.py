"""Compliant adhesive that resists separation without supporting compression."""

from dataclasses import dataclass

import numpy as np


STIFFNESS_N_M = 1200.
DAMPING_N_S_M = 3.
TWIST_STIFFNESS_NM_RAD = .026
TWIST_DAMPING_NM_S_RAD = .000045
BREAK_FORCE_N = 5.
BREAK_TORQUE_NM = .05


@dataclass(frozen=True)
class TapeBond:
    local_center: np.ndarray
    local_rotation: np.ndarray
    side: float

    @classmethod
    def at_contact(cls, wheel_center, wheel_rotation, strip_center, strip_rotation, side):
        return cls(wheel_rotation.T @ (strip_center - wheel_center), wheel_rotation.T @ strip_rotation, side)

    def reaction(self, wheel_center, wheel_rotation, wheel_velocity, strip_center, strip_rotation, strip_velocity):
        """Return the strip wrench, or None after tensile, shear or peel overload."""
        normal = self.side * wheel_rotation[:, 1]
        anchor = wheel_rotation @ self.local_center
        distance = strip_center - wheel_center - anchor
        velocity = strip_velocity[:3] - wheel_velocity[:3] - np.cross(wheel_velocity[3:], anchor)
        gap, separation_speed = distance @ normal, velocity @ normal
        tangential = -STIFFNESS_N_M * (distance - gap * normal)
        tangential -= DAMPING_N_S_M * (velocity - separation_speed * normal)
        tension = max(0., STIFFNESS_N_M * gap + DAMPING_N_S_M * separation_speed) if gap > 0. else 0.
        force = tangential - tension * normal

        target_rotation = wheel_rotation @ self.local_rotation
        twist = .5 * np.cross(strip_rotation.T, target_rotation.T).sum(axis=0)
        torque = TWIST_STIFFNESS_NM_RAD * twist - TWIST_DAMPING_NM_S_RAD * (strip_velocity[3:] - wheel_velocity[3:])
        if np.linalg.norm(force) > BREAK_FORCE_N or np.linalg.norm(torque) > BREAK_TORQUE_NM:
            return None
        return force, torque
