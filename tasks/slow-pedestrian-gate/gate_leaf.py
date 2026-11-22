"""Mass and inertia for the installed welded steel gate proposed for the preflight."""

from dataclasses import asdict, dataclass
import json

from closing_model import passive_release


STEEL_KG_M3 = 7850.0
GATE_WIDTH_M = 0.910
GATE_HEIGHT_M = 0.980
HOLDER_RADIUS_M = 0.615
POST_WIDTH_M = 0.075
FRAME_OFFSET_M = 0.044
HINGE = (0.220, -0.330, 0.0)
FRAME_Y = HINGE[1] - FRAME_OFFSET_M


@dataclass(frozen=True)
class Member:
    length_m: float
    outer_m: float
    wall_m: float
    radius_m: float
    radial_span_m: float = 0.0

    @property
    def mass_kg(self):
        inner_m = self.outer_m - 2.0 * self.wall_m
        return self.length_m * (self.outer_m**2 - inner_m**2) * STEEL_KG_M3

    @property
    def hinge_inertia_kg_m2(self):
        return self.mass_kg * (self.radius_m**2 + self.radial_span_m**2 / 12.0)


def leaf_members():
    for _ in range(2):
        yield Member(GATE_WIDTH_M, 0.040, 0.002, GATE_WIDTH_M / 2, GATE_WIDTH_M)
    vertical_length = GATE_HEIGHT_M - 0.080
    for radius in (0.020, GATE_WIDTH_M - 0.020):
        yield Member(vertical_length, 0.040, 0.002, radius)
    for i in range(1, 10):
        yield Member(vertical_length, 0.020, 0.0015, GATE_WIDTH_M * i / 10.0)


def closed_model():
    members = tuple(leaf_members())
    # Nominal moving portion of the latch and upper/lower hinge plates.
    latch_mass, latch_m = 0.65, 0.88
    hinge_plate_mass, hinge_plate_m = 0.35, 0.05
    mass = sum(m.mass_kg for m in members) + latch_mass + hinge_plate_mass
    inertia = (sum(m.hinge_inertia_kg_m2 for m in members)
               + latch_mass * latch_m**2 + hinge_plate_mass * hinge_plate_m**2
               + mass * FRAME_OFFSET_M**2)
    return {
        "gate_mass_kg": mass,
        "hinge_inertia_kg_m2": inertia,
        "release_time_s": {str(turns): asdict(passive_release(turns, inertia=inertia))
                           for turns in (0.0, 0.25, 0.375, 0.5, 0.625)},
    }


if __name__ == "__main__":
    print(json.dumps(closed_model(), indent=2))
