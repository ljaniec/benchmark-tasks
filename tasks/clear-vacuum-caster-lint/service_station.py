"""Passive bench supports for opening the original caster without tipping it."""

import numpy as np

import mechkit as m
from caster_fixture import part, split_parts


WORK_TOP = .7755
SLOT_ENTRY = np.array([.255, -.094, WORK_TOP])
SLOT_HOME = np.array([.292, -.094, WORK_TOP])
FIBRE_TRAY = np.array([.198, -.115, .7475])
TIRE_REST = np.array([.246, .072, .768])
CATCH_MIN = np.array([.238, -.104, .752])
CATCH_MAX = np.array([.268, -.084, .774])
COLOURS = {"block": m.Look(color=m.rgb(.81, .74, .49), roughness=.85)}


def retained_in_slot(fork):
    x, y, z = fork.pos
    return (.283 <= x <= .299 and abs(y - SLOT_HOME[1]) <= .002
            and abs(z - WORK_TOP) <= .002 and fork.tilt_deg() <= 8.)


def caster_slot():
    """Slide the stem collar under a keyhole ledge while the foot rests above it."""
    y = SLOT_HOME[1]
    pieces = [part("base", m.box((.077, .044, .006), (.2755, y, .7505)), "block")]
    for side, direction in (("near", -1.), ("far", 1.)):
        outline = np.array([
            [.269, .022], [.269, .010], [.283, .00415], [.314, .00415], [.314, .022],
        ])
        outline[:, 1] = y + direction * outline[:, 1]
        if direction < 0:
            outline = outline[::-1]
        pieces.extend([
            part(f"{side}_support", m.box((.077, .015, .0128), (.2755, y + direction * .0145, .7599)), "block"),
            part(f"{side}_entry", m.box((.032, .012, .0092), (.253, y + direction * .016, .7709)), "block"),
            part(f"{side}_keyway", m.prism(outline, WORK_TOP - .0092, WORK_TOP), "block"),
        ])
    for name, x in (("open_end", .238), ("inner_end", .268)):
        pieces.append(part(f"{name}_catch_lip", m.box((.002, .020, .002), (x, y, .7545)), "block"))
    pieces.append(part("closed_end", m.box((.007, .044, .022), (.312, y, .7645)), "block"))
    visuals, collisions = split_parts(pieces)
    return m.fixed("service_holder", (0., 0., 0.), visuals, collisions, COLOURS)


def wheel_rest():
    """The removed wheel rests between two rails with its axle still attached."""
    x, y = TIRE_REST[:2]
    pieces = [
        part("bottom", m.box((.049, .053, .003), (x, y, .749)), "block"),
        part("approach_rail", m.box((.006, .045, .006), (x - .013, y, .7535)), "block"),
        part("back_rail", m.box((.006, .045, .006), (x + .013, y, .7535)), "block"),
    ]
    visuals, collisions = split_parts(pieces)
    return m.fixed("wheel_rest", (0., 0., 0.), visuals, collisions, COLOURS)
