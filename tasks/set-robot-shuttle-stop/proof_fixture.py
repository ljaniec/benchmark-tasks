"""Manual 5/3 valve, pressure plumbing, and a physical dial indicator mounting."""
import math

import numpy as np

from slide_stop_geometry import contact, LOOKS
import mechkit as m
from scorer import TOLERANCE_M


VALVE_NAME = "proof_valve"
REGULATOR_NAME = "proof_regulator"
VALVE_ACTUATION_NM = .7
PORT_PRESSURE_BAR = 3.
VALVE_ORIGIN_XY = (.340, .227)
PORT_SPACING_M = .021
VALVE_PIVOT = np.array((0., 0., .052))
VALVE_STEM_LENGTH = .056
VALVE_GRIP_LENGTH = .112
VALVE_GRIP_SIZE = (.040, .032, .034)
VALVE_THROW_DEG = 28.
INDICATOR_NAME = "stroke_indicator"
READING_RANGE_M = .001
INDICATOR_RADIUS = .028
UI_LOOKS = dict(LOOKS, **{
    "face": m.Look(m.rgb(.85, .89, .87), roughness=.83),
    "orange": m.Look(m.rgb(.86, .30, .11), roughness=.60),
    "green": m.Look(m.rgb(.13, .56, .31), roughness=.63),
    "dark": m.Look(m.rgb(.16, .20, .23), roughness=.68),
})


def valve(origin):
    case = (
        ("aluminum_valve", m.box((.093, .029, .042), (0., 0., .023)), "aluminum"),
        ("valve_escutcheon", m.box((.091, .031, .002), (0., 0., .045)), "steel"),
        ("lever_hood", m.box((.029, .026, .010), (0., 0., .048)), "dark"),
        ("advance_index", m.box((.009, .021, .0012), (.034, 0., .047)), "green"),
        ("retract_index", m.box((.009, .021, .0012), (-.034, 0., .047)), "orange"),
    )
    handle = np.array((0., 0., VALVE_GRIP_LENGTH)) + VALVE_PIVOT
    stem_end = VALVE_PIVOT + (0., 0., VALVE_STEM_LENGTH)
    extension = m.box((.012, .012, VALVE_GRIP_LENGTH - VALVE_STEM_LENGTH + .012), (stem_end + handle) / 2.)
    lever = (
        ("stem", m.box((.006, .006, VALVE_STEM_LENGTH), (stem_end + VALVE_PIVOT) * .5), "steel"),
        ("extension", extension, "aluminum"),
        ("grip", m.box(VALVE_GRIP_SIZE, handle), "dark"),
    )
    throw = m.hinge("direction", "Y", VALVE_PIVOT, -VALVE_THROW_DEG, VALVE_THROW_DEG,
                    lever, tuple(contact(name + "_c", mesh) for name, mesh, _ in lever),
                    damping=.11, friction=VALVE_ACTUATION_NM, armature=.0003,
                    effort=1., mass=.045, gravity=False)
    return m.mechanism(VALVE_NAME, origin, (throw,), UI_LOOKS, case,
                       tuple(contact(name + "_c", mesh) for name, mesh, _ in case[:2]))


def lever_tip(origin, degrees):
    angle = math.radians(degrees)
    return np.asarray(origin) + VALVE_PIVOT + VALVE_GRIP_LENGTH * np.array((math.sin(angle), 0., math.cos(angle)))


def plumbing(axis_origin, valve_origin, table_height):
    """Fixed physical routes match the A/B and supply lines specified for the rig."""
    ax, ay, az = axis_origin
    vx, vy, vz = valve_origin
    regulator = np.array((vx + .045, vy - .091, table_height + .036))
    case = m.box((.042, .036, .071), regulator)
    interior = [
        ("regulator_body", case, "aluminum"),
        ("regulator_cap", m.cylinder(.013, .017, regulator + (0., 0., .041)), "dark"),
        ("gauge_face", m.cylinder(.018, .005, regulator + (-.029, 0., .016), axis="x"), "face"),
        ("gauge_needle", m.box((.001, .0018, .026), regulator + (-.032, 0., .021)), "orange"),
    ]
    lines = [
        ((vx, vy - .0145, vz + .018), (vx, vy - .047, vz + .018),
         (vx + .045, vy - .060, table_height + .036), regulator + (0., .018, 0.)),
    ]
    for i in (-1, 1):
        ax_port = (ax + .052 + i * .006, ay + .018, az + .0075)
        vx_port = (vx + i * PORT_SPACING_M, vy + .0145, vz + .020)
        lines.append((vx_port, (vx_port[0], vy + .055 + i * .008, table_height + .013),
                      (ax_port[0], ay + .071 + i * .008, table_height + .014),
                      (ax_port[0], ay + .042, az + .0075), ax_port))
    for index, points in enumerate(lines):
        interior.append((f"line_{index}", m.tube(points, .002, sides=8), "dark"))
    return m.fixed(REGULATOR_NAME, (0., 0., 0.), interior, (contact("regulator_c", case),), UI_LOOKS)


def indicator(origin):
    radius = INDICATOR_RADIUS
    face = m.cylinder(radius, .009, (0., 0., .032))
    base = m.box((.038, .048, .012), (0., 0., .006))
    upright = m.box((.021, .024, .022), (0., 0., .019))
    body = [("base", base, "dark"), ("upright", upright, "steel"), ("dial_face", face, "face")]
    half_band = math.tau * TOLERANCE_M / READING_RANGE_M
    limits = np.linspace(math.pi / 2. - half_band, math.pi / 2. + half_band, 13)
    for i, (start, end) in enumerate(zip(limits[:-1], limits[1:], strict=True)):
        section = [(r * math.cos(a), r * math.sin(a)) for r, a in
                   ((.018, start), (.025, start), (.025, end), (.018, end))]
        body.append((f"accepted_band_{i}", m.prism(section, .03661, .03673), "green"))
    for i in range(20):
        angle = math.tau * i / 20.
        loc = (.022 * math.cos(angle), .022 * math.sin(angle), .037)
        mark = m.box((.006, .0020 if i % 5 == 0 else .0012, .0004)).turned(math.degrees(angle)).moved(loc)
        look = "green" if i in (4, 5, 6) else "dark"
        body.append((f"mark_{i}", mark, look))
    needle = m.box((.022, .0021, .0012), (.010, 0., .0381))
    dial = m.hinge("dial", "Z", (0., 0., .0381), -180, 180,
                   (("pointer", needle, "orange"),), (), spring=.45, damping=.030,
                   mass=.007, effort=1., gravity=False)
    return m.mechanism(INDICATOR_NAME, origin, (dial,), UI_LOOKS, body,
                       tuple(contact(name + "_c", mesh) for name, mesh, _ in body[:3]))
