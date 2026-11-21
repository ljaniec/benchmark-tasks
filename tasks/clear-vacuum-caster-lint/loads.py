"""Sensitivity of a service caster's detent and spin check to physical loads."""
from dataclasses import dataclass
from math import isfinite, radians, tan


@dataclass(frozen=True)
class Detent:
    """Symmetric cantilever fingers with a ramped steel-pin groove."""

    modulus_gpa: float
    width_mm: float
    thickness_mm: float
    length_mm: float
    deflection_mm: float
    friction: float

    def pull_force_n(self, ramp_angle_deg):
        beam_mm4 = self.width_mm * self.thickness_mm**3 / 12.
        stiffness_n_mm = 3. * self.modulus_gpa * 1000. * beam_mm4 / self.length_mm**3
        radial_n = stiffness_n_mm * self.deflection_mm
        slope = tan(radians(ramp_angle_deg))
        if self.friction * slope >= 1.:
            raise ValueError('The selected dry ramp is self-locking in this approximation')
        return 2. * radial_n * (slope + self.friction) / (1. - self.friction * slope)


def free_advance_rad(initial_speed, drag_nm, duration_s, inertia_kg_m2):
    """Constant opposing drag approximation while the wheel coasts."""
    values = (initial_speed, drag_nm, duration_s, inertia_kg_m2)
    if not all(isfinite(x) and x >= 0. for x in values) or inertia_kg_m2 == 0.:
        raise ValueError('Use finite nonnegative values and a positive inertia')
    slow_down = drag_nm / inertia_kg_m2
    interval = min(duration_s, initial_speed / slow_down) if slow_down else duration_s
    return initial_speed * interval - .5 * slow_down * interval**2


def report():
    # Engineering sensitivity only. Obtain the actual pin and plastic clip
    # dimensions and force traces before choosing one of these loads.
    cases = {
        'short_soft': Detent(1.8, 5., .70, 9., .30, .15),
        'mid': Detent(2.3, 6., .95, 8.5, .50, .20),
        'stiff': Detent(2.8, 6., 1.15, 8., .65, .25),
    }
    return {
        'model_only': True,
        'detent_forces_n': {label: {'seat_25_deg': detent.pull_force_n(25.),
                                    'release_52_deg': detent.pull_force_n(52.)}
                            for label, detent in cases.items()},
        'wheel_advance_rad_at_8_rad_s': {str(int(drag * 1e6)): free_advance_rad(8., drag, .5, 4e-6)
                                        for drag in (.00001, .00004, .00010, .00030)},
    }


if __name__ == '__main__':
    import json

    print(json.dumps(report(), indent=2))
