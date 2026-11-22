"""Physical tube, foam blockage, blunt rod and catch tray."""
import math

import numpy as np

import mechkit as m
import layout as g


TUBE_FACETS = 20
PAD_STIFFNESS_N_M = 1150.
PAD_TRAVEL_M = .004
PAD_START_M = .00175
PLUG_RADIUS_M = .0164
TRAY_HALF_M = (.090, .095)
FLOOR_M = .008
RIM_M = .029
LOOKS = {
    "steel": m.Look(m.rgb(.55, .58, .61), metallic=.66, roughness=.47),
    "foam": m.Look(m.rgb(.91, .38, .13), roughness=.86),
    "shaft": m.Look(m.rgb(.73, .58, .39), roughness=.72),
    "handle": m.Look(m.rgb(.12, .43, .52), roughness=.72),
    "tray": m.Look(m.rgb(.27, .39, .42), roughness=.81),
    "rest": m.Look(m.rgb(.34, .42, .34), roughness=.75),
}


def sector(inner, outer, lower, upper, length, count=1):
    """A convex piece of an annular wall, extending along the tube axis."""
    angle = np.linspace(lower, upper, count + 1)
    unit = np.column_stack((np.cos(angle), np.sin(angle)))
    cross_section = np.vstack((outer * unit, inner * unit[::-1]))
    return m.prism(cross_section, -length / 2., length / 2.).turned(90., "x")


def tube(position, *, free=True):
    step = math.tau / TUBE_FACETS
    inner = g.TUBE_BORE_M / (2. * math.cos(step / 2.))
    outer = g.TUBE_OD_M / 2.
    wall = [
        sector(inner, outer, (i - .5) * step, (i + .5) * step, g.TUBE_LENGTH_M)
        for i in range(TUBE_FACETS)
    ]
    visuals = [(f"wall_{i:02}", patch, "steel") for i, patch in enumerate(wall)]
    bore_radius = g.TUBE_BORE_M / 2.
    thickness = outer - bore_radius
    width = 2. * outer * math.tan(step / 2.)
    center = bore_radius + thickness / 2.
    colliders = [
        m.col_box(f"wall_contact_{i:02}", (thickness, g.TUBE_LENGTH_M, width),
                  (center * math.cos(i * step), 0., center * math.sin(i * step)),
                  rotation_y=-math.degrees(i * step))
        for i in range(TUBE_FACETS)
    ]
    if free:
        return m.free("extension_tube", position, visuals, colliders, LOOKS, g.TUBE_MASS_KG,
                      friction=(.27, .21), torsional_patch=None)
    return m.fixed("extension_tube", position, visuals, colliders, LOOKS, friction=(.27, .21))


def plug(position):
    core = m.cylinder(.0128, g.PLUG_LENGTH_M, axis="y")
    # Four spring-loaded contact pads approximate radial compression of the foam.
    pads, starts = [], []
    for name, axis, orientation, sign in (
        ("right", "X", 0., 1), ("top", "Z", math.pi / 2., 1),
        ("left", "X", math.pi, -1), ("bottom", "Z", 3. * math.pi / 2., -1),
    ):
        piece = sector(.0118, PLUG_RADIUS_M, orientation - math.pi / 7.,
                       orientation + math.pi / 7., g.PLUG_LENGTH_M, count=4)
        limits = (-PAD_TRAVEL_M, 0.) if sign > 0 else (0., PAD_TRAVEL_M)
        starts.append((name, -sign * PAD_START_M))
        pads.append(m.slider(name, axis, *limits, ((name, piece, "foam"),),
                             (m.col_hull(name + "_contact", piece),), mass=.00055,
                             spring=PAD_STIFFNESS_N_M, damping=1.1, effort=9., armature=1.e-7,
                             friction=0., gravity=True))
    return m.mechanism("foam_plug", position, pads, LOOKS,
                       frame_parts=(("core", core, "foam"),),
                       frame_colliders=(m.col_cylinder("core_contact", .0128, g.PLUG_LENGTH_M, axis="Y"),),
                       friction=(.57, .45), free_mass=.0015, start=starts,
                       frame_inertia=(1.2e-7, 1.2e-7, 1.2e-7))


def rod(position):
    shank_center = (0., (g.HANDLE_M[1] + g.ROD_SHANK_M) / 2., 0.)
    shape = (
        ("handle", m.box(g.HANDLE_M), "handle"),
        ("shaft", m.cylinder(g.ROD_RADIUS_M, g.ROD_SHANK_M, shank_center, axis="y"), "shaft"),
    )
    columns = (
        m.col_box("handle_contact", g.HANDLE_M),
        m.col_cylinder("shaft_contact", g.ROD_RADIUS_M, g.ROD_SHANK_M, shank_center, axis="Y"),
    )
    return m.free("clearing_rod", position, shape, columns, LOOKS, .063, friction=(.45, .30),
                  torsional_patch=None, com=(0., .137, 0.))


def tray(position):
    hx, hy = TRAY_HALF_M
    floor = ("floor", (2 * hx + .005, 2 * hy + .005, FLOOR_M), (0., 0., FLOOR_M / 2.))
    level = FLOOR_M + RIM_M / 2.
    walls = (
        ("left", (.0025, 2 * hy, RIM_M), (-hx - .00125, 0., level)),
        ("right", (.0025, 2 * hy, RIM_M), (hx + .00125, 0., level)),
        ("front", (2 * hx + .005, .0025, RIM_M), (0., -hy - .00125, level)),
        ("back", (2 * hx + .005, .0025, RIM_M), (0., hy + .00125, level)),
    )
    items = (floor, *walls)
    return m.fixed("waste_tray", position,
                   tuple((name, m.box(size, loc), "tray") for name, size, loc in items),
                   tuple(m.col_box(name + "_contact", size, loc) for name, size, loc in items),
                   LOOKS, friction=(.47, .34))


def rests(name, x, locations, radius, lateral_clearance):
    """Two open-topped bench saddles support the part without end stops."""
    floor = g.TUBE_Z_M - g.TABLE_TOP - radius
    half_opening = radius + lateral_clearance
    blocks = []
    for index, y in enumerate(locations):
        blocks.append((f"base_{index}", (2. * half_opening + .024, .030, floor),
                       (0., y, floor / 2.)))
        for side, sign in (("left", -1), ("right", 1)):
            blocks.append((f"{side}_{index}", (.012, .030, radius + .003),
                           (sign * (half_opening + .006), y, floor + (radius + .003) / 2.)))
    return m.fixed(name, (x, 0., g.TABLE_TOP),
                   tuple((key, m.box(size, loc), "rest") for key, size, loc in blocks),
                   tuple(m.col_box(key + "_contact", size, loc) for key, size, loc in blocks),
                   LOOKS, friction=(.40, .28))
