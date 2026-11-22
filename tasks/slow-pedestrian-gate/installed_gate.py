"""Full-size welded leaf and fixed posts for the gate development fixture."""

from math import pi

import numpy as np

import mechkit as m

from gate_leaf import (FRAME_OFFSET_M, FRAME_Y, GATE_HEIGHT_M, GATE_WIDTH_M, HINGE,
                       HOLDER_RADIUS_M, POST_WIDTH_M, leaf_members)
from gate_latches import gate_strikers


BASE_TUBE = 0.040
PICKET_TUBE = 0.020
GATE_WITNESS_M = 0.230

LOOKS = {
    "steel": m.Look(color=m.rgb(.23, .32, .34), roughness=.6),
    "posts": m.Look(color=m.rgb(.38, .43, .44), roughness=.7),
    "mark": m.Look(color=m.rgb(.97, .82, .22), roughness=.45),
}


def post_specs():
    for name, x, y, height in (
        ("hinge_post", HINGE[0] - .055, FRAME_Y, 1.015),
        ("strike_post", HINGE[0] + GATE_WIDTH_M + .047, FRAME_Y, 1.015),
        ("holdback_post", HINGE[0] - (.145 - FRAME_OFFSET_M), HINGE[1] + HOLDER_RADIUS_M, .823),
    ):
        yield name, (POST_WIDTH_M, POST_WIDTH_M, height), (x, y, height / 2)


def leaf_specs():
    for label, z in (("bottom", BASE_TUBE / 2), ("top", GATE_HEIGHT_M - BASE_TUBE / 2)):
        yield label + "_rail", (GATE_WIDTH_M, BASE_TUBE, BASE_TUBE), (
            HINGE[0] + GATE_WIDTH_M / 2, FRAME_Y, z
        )

    picket_height = GATE_HEIGHT_M - 2 * BASE_TUBE
    for label, radius in (("hinge", BASE_TUBE / 2), ("strike", GATE_WIDTH_M - BASE_TUBE / 2)):
        yield label + "_stile", (BASE_TUBE, BASE_TUBE, picket_height), (
            HINGE[0] + radius, FRAME_Y, GATE_HEIGHT_M / 2
        )

    for index in range(1, 10):
        yield f"picket_{index}", (PICKET_TUBE, PICKET_TUBE, picket_height), (
            HINGE[0] + GATE_WIDTH_M * index / 10, FRAME_Y, GATE_HEIGHT_M / 2
        )


def rigid_parts(specs, material):
    visuals = [(name, m.box(size, centre), material) for name, size, centre in specs]
    collision = [m.col_box(name + "_shape", size, centre).replace("contactOffset = 0.004", "contactOffset = 0.00015")
                 for name, size, centre in specs]
    return visuals, collision


def leaf_mass_properties(geometry):
    positions = np.array([pos for _, _, pos in geometry], dtype=float)
    sizes = np.array([size for _, size, _ in geometry], dtype=float)
    hardware = np.array([[1.10, FRAME_Y, .900], [.825, FRAME_Y, .824]])
    sizes = np.vstack([sizes, [.105, .053, .030], [.060, .044, .024]])
    positions = np.vstack([positions, hardware])
    masses = np.array([*(member.mass_kg for member in leaf_members()), .65, .35])
    first_moment = np.sum(positions * masses[:, None], axis=0)
    mass = float(masses.sum())
    centre = first_moment / mass
    shape_moments = (np.sum(sizes**2, axis=1)[:, None] - sizes**2) / 12
    displacement = positions - centre
    mass_moments = np.sum(displacement**2, axis=1)[:, None] - displacement**2
    tensor = np.sum(masses[:, None] * (shape_moments + mass_moments), axis=0)
    return mass, tuple(centre), tuple(tensor)


def installed_gate():
    geometry = tuple(leaf_specs())
    leaf_visuals, leaf_colliders = rigid_parts(geometry, "steel")
    strikers = gate_strikers()
    leaf_visuals.extend(strikers)
    leaf_colliders.extend(m.col_hull(name + "_shape", mesh).replace("contactOffset = 0.004", "contactOffset = 0.00015")
                          for name, mesh, _ in strikers)
    frame_visuals, frame_colliders = rigid_parts(tuple(post_specs()), "posts")
    witness = m.box((.025, .035, .0015), (HINGE[0] + GATE_WITNESS_M, FRAME_Y, GATE_HEIGHT_M + .001))
    leaf_visuals.append(("angle_witness", witness, "mark"))
    mass, centre, inertia = leaf_mass_properties(geometry)
    leaf = m.hinge("gate", "Z", HINGE, 0.0, 92.0, leaf_visuals, leaf_colliders,
                  mass=mass, com=centre, inertia=inertia, damping=0.0, friction=.04,
                  effort=16.0, armature=.0001)
    return m.mechanism("pedestrian_gate", (0.0, 0.0, 0.0), [leaf], LOOKS,
                       frame_parts=frame_visuals, frame_colliders=frame_colliders,
                       start=(("gate", pi / 2),), solver_iters=48)


if __name__ == "__main__":
    from pathlib import Path
    from pxr import Usd

    model = installed_gate()
    output = Path(__file__).with_name("pedestrian_gate.usda")
    output.write_text(model.build())
    stage = Usd.Stage.Open(str(output))
    assert stage.GetPrimAtPath("/P_pedestrian_gate/gate_joint").IsValid()
