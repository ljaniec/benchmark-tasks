"""Passive hold-open hook and self-latching strike for the native gate fixture."""

from math import radians

import numpy as np
import mechkit as m

from gate_leaf import FRAME_Y, HINGE, HOLDER_RADIUS_M


HOLDER_PIVOT = (.146, HINGE[1] + HOLDER_RADIUS_M, .843)
HOLDER_PIN_M = (.229, HOLDER_PIVOT[1], .824)
HOLDER_PIN_RADIUS_M = .004
HOLDER_PIN_LENGTH_M = .024
HOLDER_NOSE_X = .2375
HOLDER_NOSE_BOTTOM = .817
HOLDER_RELEASE_DEG = -16.0
HOLDER_GRIP_CENTRE = (.197, HOLDER_PIVOT[1], .861)
HOLDER_GRIP_SIZE = (.022, .027, .026)

STRIKE_RADIUS_M = .00475
STRIKE_CLOSED_Y = FRAME_Y + .066
STRIKE_HEIGHT = .900
LATCH_PIVOT = (1.154, -.329, .879)
PAWL_LIMIT_DEG = 42.
PAWL_SECTION = ((-.294, .919), (-.270, .919), (-.274, .911),
                (-.292, .892), (-.294, .891))
LOOKS = {
    "steel": m.Look(color=m.rgb(.54, .58, .60), roughness=.43, metallic=.30),
    "mark": m.Look(color=m.rgb(.92, .71, .25), roughness=.42),
}


def colliders(parts):
    return [m.col_hull(name + "_shape", mesh).replace("contactOffset = 0.004", "contactOffset = 0.00015")
            for name, mesh, _ in parts]


def gate_strikers():
    """Moving hardware authored at the gate's zero-degree position."""
    rx = HOLDER_PIN_M[1] - HINGE[1]
    ry = -(HOLDER_PIN_M[0] - HINGE[0])
    holder_pin = (HINGE[0] + rx, HINGE[1] + ry, HOLDER_PIN_M[2])
    bracket = (.842, -.358, .819)
    strike_offset = STRIKE_CLOSED_Y - (FRAME_Y + .024)
    return [
        ("holdback_plate", m.box((.045, .041, .006), bracket), "steel"),
        ("holdback_pin", m.cylinder(HOLDER_PIN_RADIUS_M, HOLDER_PIN_LENGTH_M, holder_pin), "steel"),
        ("strike_plate", m.box((.053, .006, .025), (1.091, FRAME_Y + .023, STRIKE_HEIGHT)), "steel"),
        ("strike_offset", m.box((.016, strike_offset, .013),
                                (1.110, STRIKE_CLOSED_Y - strike_offset / 2, STRIKE_HEIGHT)), "steel"),
        ("strike_pin", m.cylinder(STRIKE_RADIUS_M, .054, (1.141, STRIKE_CLOSED_Y, STRIKE_HEIGHT), axis="x"), "steel"),
    ]


def holder_hook():
    """The gate loads the nose until the hinged hook is physically lifted."""
    x, y, z = HOLDER_PIVOT
    base_shape = m.box((.036, .044, .018), (x - .015, y, .832))
    mount = [("hook_mount", base_shape, "steel")]

    top = z + .006
    length = HOLDER_NOSE_X - x
    span = m.box((length, .013, .012), (x + length / 2, y, z))
    nose = m.box((.009, .013, top - HOLDER_NOSE_BOTTOM),
                 (HOLDER_NOSE_X, y, (top + HOLDER_NOSE_BOTTOM) / 2))
    grip = m.box(HOLDER_GRIP_SIZE, HOLDER_GRIP_CENTRE)
    hook = [("hook_arm", span, "steel"), ("hook_nose", nose, "steel"), ("hook_grip", grip, "mark")]
    link = m.hinge("hook", "Y", HOLDER_PIVOT, -30.0, 0.0, hook, colliders(hook),
                   mass=.122, damping=.024, friction=.006, gravity=True,
                   effort=.12, com=(.189, y, .844), inertia=(.000011, .00011, .00011))
    return m.mechanism("gate_holder", (0., 0., 0.), [link], LOOKS,
                       frame_parts=mount, frame_colliders=colliders(mount),
                       friction=(.32, .25), solver_iters=48)


def yz_prism(outline, x, width):
    """Extrude a curved-plane latch section along the striker axis."""
    outline = np.asarray(outline, dtype=float)
    end = np.roll(outline, -1, axis=0)
    signed_area = np.sum(outline[:, 0] * end[:, 1] - outline[:, 1] * end[:, 0])
    if signed_area < 0:
        outline = outline[::-1]
    mesh = m.prism(outline, x - width / 2, x + width / 2)
    return m.Mesh(mesh.v[:, [2, 0, 1]], mesh.f)


def closing_latch():
    """The strike raises a beveled pawl, which falls behind the closed strike."""
    x, y, z = LATCH_PIVOT
    fixed = [
        ("catch_mount", m.box((.035, .004, .055), (x, -.3348, .899)), "steel"),
        ("catch_floor", m.box((.025, .024, .005), (x, -.321, .881)), "steel"),
        ("catch_pivot", m.cylinder(.0035, .030, (x + .012, y, z), axis="x"), "steel"),
    ]
    ramp = ("pawl_ramp", yz_prism(PAWL_SECTION, x, .013), "steel")
    # The pivot sits below the strike. Pulling outward loads the back
    # of the hook toward its closed stop. The entry face lifts the hook.
    heel = ("pawl_heel", yz_prism([
        (y - .003, z - .003), (y + .003, z - .003), (-.286, .901),
        (-.292, .903), (y - .003, z + .003),
    ], x + .021, .008), "steel")
    crossbar = ("pawl_crossbar", m.box((.023, .008, .004), (x + .012, -.289, .903)), "steel")
    pawl = (ramp, heel, crossbar)
    visual = [*pawl, ("paint", m.box((.011, .019, .001), (x, -.282, .9197)), "mark")]
    link = m.hinge("pawl", "X", LATCH_PIVOT, 0.0, PAWL_LIMIT_DEG, visual, colliders(pawl),
                   mass=.045, damping=.004, friction=.0012, gravity=True, effort=.12,
                   com=(x, -.294, .901), inertia=(.000033, .000016, .000022))
    return m.mechanism("gate_latch", (0., 0., 0.), [link], LOOKS,
                       frame_parts=fixed, frame_colliders=colliders(fixed),
                       friction=(.27, .18), solver_iters=48)


def geometry_witness():
    """Check the nose clears the actual keeper after the screened lift."""
    length = HOLDER_NOSE_X - .0045 - HOLDER_PIVOT[0]
    offset_z = HOLDER_NOSE_BOTTOM - HOLDER_PIVOT[2]
    angle = radians(HOLDER_RELEASE_DEG)
    raised_bottom = HOLDER_PIVOT[2] - length * np.sin(angle) + offset_z * np.cos(angle)
    pin_top = HOLDER_PIN_M[2] + HOLDER_PIN_LENGTH_M / 2
    post_front_y = FRAME_Y + .075 / 2
    pivot_y, pivot_z = LATCH_PIVOT[1:]
    section = np.array(PAWL_SECTION)
    offsets = section - (pivot_y, pivot_z)
    rotations = np.radians(np.linspace(0., PAWL_LIMIT_DEG, 100))
    moving_y = (pivot_y + offsets[:, 0] * np.cos(rotations[:, None])
                - offsets[:, 1] * np.sin(rotations[:, None]))
    moving_z = (pivot_z + offsets[:, 0] * np.sin(rotations[:, None])
                + offsets[:, 1] * np.cos(rotations[:, None]))
    swept_section = np.stack((moving_y, moving_z), axis=-1)
    segments = np.roll(swept_section, -1, axis=1) - swept_section
    closed_strike = np.array((STRIKE_CLOSED_Y, STRIKE_HEIGHT))
    to_strike = closed_strike - swept_section
    along = np.clip((to_strike * segments).sum(axis=-1) / (segments**2).sum(axis=-1), 0., 1.)
    swept_gap = np.linalg.norm(to_strike - along[:, :, None] * segments, axis=-1).min() - STRIKE_RADIUS_M
    return {
        "keeper_top_mm": 1000 * pin_top,
        "engaged_nose_overlap_mm": 1000 * (pin_top - HOLDER_NOSE_BOTTOM),
        "released_nominal_clearance_mm": 1000 * (raised_bottom - pin_top),
        "strike_pin_diameter_mm": 2000 * STRIKE_RADIUS_M,
        "minimum_cam_clearance_from_strike_post_mm": 1000 * (moving_y.min() - post_front_y),
        "closed_strike_to_backface_mm": 1000 * (section[:, 0].min() - STRIKE_CLOSED_Y - STRIKE_RADIUS_M),
        "fully_raised_cam_above_strike_mm": 1000 * (moving_z[-1].min() - STRIKE_HEIGHT - STRIKE_RADIUS_M),
        "full_cam_return_clearance_mm": 1000 * swept_gap,
        "backface_closing_moment_arm_mm": 1000 * (STRIKE_HEIGHT - pivot_z),
    }


if __name__ == "__main__":
    import json
    from pathlib import Path

    from pxr import Usd

    report = geometry_witness()
    assert report["engaged_nose_overlap_mm"] > 10
    assert report["released_nominal_clearance_mm"] > 4
    assert report["minimum_cam_clearance_from_strike_post_mm"] > 2.5
    assert report["closed_strike_to_backface_mm"] > 1.
    assert report["fully_raised_cam_above_strike_mm"] > 2.
    assert report["backface_closing_moment_arm_mm"] > 18.
    assert report["full_cam_return_clearance_mm"] > 2.0
    for asset in (holder_hook(), closing_latch()):
        output = Path(__file__).with_name(asset.name + ".usda")
        output.write_text(asset.build())
        stage = Usd.Stage.Open(str(output))
        assert stage.GetDefaultPrim().IsValid()
    print(json.dumps(report, indent=2))
