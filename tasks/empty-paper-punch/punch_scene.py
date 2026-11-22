"""Loose paper punch, compliant cover closure, cardstock scraps and tray."""
from dataclasses import dataclass

import numpy as np

import mechkit as m
from scorer import TOTAL_DISKS
from yam_arena.task_file import TABLE_TOP


@dataclass(frozen=True)
class Box:
    name: str
    size: tuple[float, float, float]
    center: tuple[float, float, float]


DESK_Z = TABLE_TOP["station"]
PUNCH_START = (.285, .120, DESK_Z + .0012)
TRAY_CENTER = (.350, -.040, DESK_Z - .006)
PIVOT = (.032, 0., 0.)
RELEASE_FACE = (-.059, -.041, .00425)
BEAD_Y_M = -.026
BEAD_WIDTH_M = .009
BEAD_XZ_M = ((-.0377, .00295), (-.0422, .00295), (-.0422, .0044),
             (-.0386, .0071), (-.0377, .0071))
KEEPER = Box("keeper", (.0029, .012, .0021), (-.0429, BEAD_Y_M, .00150))
FLEX_LIMIT_M = .00165
STIFFNESS_N_PER_M = 1200.
SCRAP_DIAMETER_M = .0055
SCRAP_THICKNESS_M = .00038
SCRAP_MASS_KG = .000006
SCRAP_NAMES = tuple(f"scrap_{index:02}" for index in range(TOTAL_DISKS))
TRAY_HALF_M = (.110, .095)
TRAY_BOTTOM_Z_M = .012
TRAY_TOP_Z_M = .038
SHALLOW_FLOOR_M = .006
BACKED_FLOOR_M = .020

LOOKS = {
    "blue": m.Look(m.rgb(.18, .42, .57), roughness=.52),
    "steel": m.Look(m.rgb(.52, .60, .64), metallic=.7, roughness=.54),
    "cover": m.Look(m.rgb(.19, .22, .25), roughness=.76),
    "tab": m.Look(m.rgb(.82, .46, .22), roughness=.67),
    "tan": m.Look(m.rgb(.61, .54, .44), roughness=.80),
    "cream": m.Look(m.rgb(.98, .93, .75), roughness=.82),
    "amber": m.Look(m.rgb(.91, .52, .27), roughness=.82),
    "pad": m.Look(m.rgb(.26, .34, .36), roughness=.88),
}


def boxes(parts, look):
    return tuple((part.name, m.box(part.size, part.center), look) for part in parts)


def colliders(parts):
    return tuple(m.col_box(f"{part.name}_col", part.size, part.center) for part in parts)


def across_y(points, width, y=0.):
    outline = np.asarray(points, dtype=float)
    area = np.linalg.det(np.stack((outline, np.roll(outline, -1, axis=0)), axis=1)).sum()
    if area < 0.:
        outline = outline[::-1]
    return m.prism(outline, -width / 2, width / 2).turned(90., "x").moved((0., y, 0.))


def punch():
    chamber = (
        Box("chamber_roof", (.076, .092, .006), (0., 0., .0145)),
        Box("chamber_left", (.076, .003, .0102), (0., .044, .00634)),
        Box("chamber_right", (.076, .003, .0102), (0., -.044, .00634)),
        Box("chamber_front", (.003, .085, .0102), (-.034, 0., .00634)),
        Box("chamber_rear", (.004, .085, .0102), (.035, 0., .00634)),
        Box("steel_housing", (.081, .100, .020), (0., 0., .0275)),
        KEEPER,
    )
    handle = across_y(((-.0405, .035), (-.0405, .056), (-.027, .060), (.023, .069),
                       (.0405, .058), (.0405, .035)), .077)
    cover = (Box("bottom_panel", (.064, .088, .002), (0., 0., 0.)),)
    tab = (
        Box("tab_foot", (.0099, BEAD_WIDTH_M, .0008), (-.0349, BEAD_Y_M, .0002)),
        Box("tab_riser", (.0013, BEAD_WIDTH_M, .0042), (-.0387, BEAD_Y_M, .00230)),
        Box("clip_crossbar", (.0013, .018, .0024), (-.0387, -.03275, .0035)),
        Box("release_link", (.0165, .0045, .0032), (-.04675, -.03925, .0041)),
        Box("release_button", (.006, .016, .0095), (-.056, -.041, .00425)),
    )
    bead = across_y(BEAD_XZ_M, BEAD_WIDTH_M, BEAD_Y_M)
    hinge = m.hinge("waste_cover", "Y", PIVOT, -90., 0., boxes(cover, "cover"), colliders(cover), mass=.012,
                    damping=.0015, friction=.0004, armature=1.e-7, gravity=True, com=(0., 0., 0.),
                    inertia=(7.75e-6, 4.10e-6, 1.19e-5))
    snap = m.slider("elastic_tab", "X", 0., FLEX_LIMIT_M,
                    boxes(tab, "tab") + (("slanted_bead", bead, "tab"),),
                    colliders(tab) + (m.col_hull("bead_col", bead),), at=PIVOT, parent="waste_cover",
                    mass=.00135, spring=STIFFNESS_N_PER_M, damping=1.7, effort=8.,
                    armature=1.e-7, friction=0., gravity=True, com=(-.0385, BEAD_Y_M, .0038))
    solid = m.mechanism(
        "paper_punch", PUNCH_START, [hinge, snap], LOOKS,
        frame_parts=boxes(chamber, "steel") + (("lever", handle, "blue"),),
        frame_colliders=colliders(chamber) + (m.col_hull("lever_col", handle),),
        friction=(.26, .19), free_mass=.25665, frame_com=(0., 0., .035),
        frame_inertia=(.000319, .000245, .000354), self_collide=True,
    )
    return solid


def tray():
    hx, hy = TRAY_HALF_M
    rim = TRAY_TOP_Z_M - TRAY_BOTTOM_Z_M
    z = TRAY_BOTTOM_Z_M + rim / 2
    sides = (
        Box("left_edge", (2 * hx + .005, .0025, rim), (0., hy + .00125, z)),
        Box("right_edge", (2 * hx + .005, .0025, rim), (0., -hy - .00125, z)),
        Box("front_edge", (.0025, 2 * hy, rim), (-hx - .00125, 0., z)),
        Box("back_edge", (.0025, 2 * hy, rim), (hx + .00125, 0., z)),
    )
    def floor(thickness):
        return Box("tray_floor", (2 * hx + .005, 2 * hy + .005, thickness),
                   (0., 0., TRAY_BOTTOM_Z_M - thickness / 2))

    visible = (floor(SHALLOW_FLOOR_M), *sides)
    supported = (floor(BACKED_FLOOR_M), *sides)
    return m.fixed("waste_tray", TRAY_CENTER, boxes(visible, "tan"), colliders(supported), LOOKS, friction=(.26, .19))


def in_tray_mask(relative_positions):
    x, y, z = np.asarray(relative_positions).T
    return ((np.abs(x) <= TRAY_HALF_M[0]) & (np.abs(y) <= TRAY_HALF_M[1])
            & (z >= TRAY_BOTTOM_Z_M - .001) & (z <= TRAY_TOP_Z_M))


def starting_pad():
    frame = (
        Box("front", (.002, .112, .0002), (-.065, 0., 0.)),
        Box("rear", (.002, .112, .0002), (.047, 0., 0.)),
        Box("left", (.112, .002, .0002), (-.009, .056, 0.)),
        Box("right", (.112, .002, .0002), (-.009, -.056, 0.)),
    )
    return m.fixed("starting_pad", (*PUNCH_START[:2], DESK_Z + .0003), boxes(frame, "pad"), looks=LOOKS)


def starting_centers():
    for layer in range(2):
        for column in range(4):
            for row in range(6):
                yield (PUNCH_START[0] + .0074 * (column - 1.5) + layer * .0014,
                       PUNCH_START[1] + .0074 * (row - 2.5),
                       PUNCH_START[2] + .00130 + layer * .00060)


def scraps():
    r = SCRAP_DIAMETER_M / 2
    disk = m.cylinder(r, SCRAP_THICKNESS_M)
    coll = m.col_cylinder("scrap_col", r, SCRAP_THICKNESS_M)
    for index, (name, pos) in enumerate(zip(SCRAP_NAMES, starting_centers(), strict=True)):
        look = "cream" if index % 2 else "amber"
        yield m.free(name, pos, (("disk", disk, look),), (coll,), LOOKS, SCRAP_MASS_KG,
                     friction=(.26, .19), torsional_patch=None)
