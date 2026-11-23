"""Assembled, unplugged 7-inch tabletop fan with an independent pitch lock."""
import math
import numpy as np

from yam_arena.station import TABLE_TOP
import clamp
import mechkit as m


ORIGIN = np.array((.400, -.018, TABLE_TOP))
PIVOT = np.array((0., 0., .132))
DROP_DEG = -22.
HEAD_MASS = .84
HEAD_COM = (-.038, 0., .176)
GUARD_RADIUS = .089
GUARD_CENTER = np.array((-.067, 0., .210))
KNOB_AXIS = np.array((0., -.052, PIVOT[2]))
KNOB_GRIP = np.array((0., -.062, PIVOT[2]))
SUPPORT = GUARD_CENTER + (0., .071, -.027)
WEIGHT_MASS = .175
WEIGHT_REST = np.array((.308, -.313, TABLE_TOP + .012))
WEIGHT_GRIP = np.array((0., 0., .016))
PROOF_SITE = GUARD_CENTER + (-.003, 0., .090)
WEIGHT_SIDES = (.005, .031, .017)
LOOKS = {
    "enamel": m.Look(m.rgb(.18, .43, .57), roughness=.57),
    "rim": m.Look(m.rgb(.27, .30, .31), metallic=.48, roughness=.53),
    "wire": m.Look(m.rgb(.65, .70, .68), metallic=.66, roughness=.42),
    "blade": m.Look(m.rgb(.87, .83, .68), roughness=.61),
    "rubber": m.Look(m.rgb(.20, .22, .22), roughness=.86),
    "weight": m.Look(m.rgb(.82, .36, .12), metallic=.40, roughness=.53),
    "witness": m.Look(m.rgb(.94, .62, .20), roughness=.66),
}


def rotate_head(point, angle):
    delta = np.asarray(point) - PIVOT
    sine, cosine = math.sin(math.radians(angle)), math.cos(math.radians(angle))
    return PIVOT + np.array((cosine * delta[0] + sine * delta[2], delta[1],
                             -sine * delta[0] + cosine * delta[2]))


def cage():
    visuals = [
        ("motor", m.cylinder(.0375, .078, (-.012, 0., GUARD_CENTER[2]), axis="x"), "enamel"),
        ("head_yoke", m.box((.044, .036, .058), (-.003, 0., .156)), "enamel"),
        ("tilt_pivot_journal", m.cylinder(.012, .041, PIVOT, axis="y"), "rim"),
        ("hub", m.cylinder(.025, .037, GUARD_CENTER + (-.021, 0., 0.), axis="x"), "rim"),
    ]
    phases = np.linspace(0., 2. * math.pi, 49)
    for radius, x, name in ((GUARD_RADIUS, -.085, "front_rim"), (GUARD_RADIUS, -.056, "rear_rim"),
                            (.038, -.086, "finger_ring"), (.062, -.086, "middle_ring"),
                            (.041, -.054, "rear_finger_ring"), (.064, -.054, "rear_middle_ring")):
        points = [(x, radius * math.cos(a), GUARD_CENTER[2] + radius * math.sin(a)) for a in phases]
        visuals.append((name, m.tube(points, .0026 if name.endswith("rim") else .0011, sides=8), "wire"))
    for i, a in enumerate(np.linspace(0., 2. * math.pi, 24, endpoint=False)):
        end = (-.085, GUARD_RADIUS * math.cos(a), GUARD_CENTER[2] + GUARD_RADIUS * math.sin(a))
        start = (-.085, .023 * math.cos(a), GUARD_CENTER[2] + .023 * math.sin(a))
        visuals.append((f"front_wire_{i}", m.tube((start, end), .001, sides=7), "wire"))
        if i % 2 == 0:
            rear = ((-.054, start[1], start[2]), (-.056, end[1], end[2]))
            visuals.append((f"rear_wire_{i}", m.tube(rear, .001, sides=7), "wire"))
    for i, a in enumerate(np.linspace(0., 360., 4, endpoint=False)):
        blade = m.prism(((.020, -.006), (.058, -.018), (.073, -.005), (.067, .013), (.032, .009)),
                        -.0013, .0013).turned(90., "y")
        blade = blade.turned(a, "x").moved(GUARD_CENTER + (-.010, 0., 0.))
        visuals.append((f"blade_{i}", blade, "blade"))
    # The collision envelope keeps fingers outside the intact wire guard.
    contact = (
        m.col_cylinder("guard_envelope", .090, .033, (-.070, 0., GUARD_CENTER[2]), axis="X"),
        m.col_cylinder("motor_envelope", .039, .078, (-.012, 0., GUARD_CENTER[2]), axis="X"),
        m.col_box("yoke_contact", (.044, .036, .058), (-.003, 0., .156)),
    )
    return tuple(visuals), contact


def fan():
    base = m.cylinder(.095, .024, (0., 0., .012))
    neck = m.box((.044, .042, .111), (0., 0., .0755))
    components = (
        ("base", base, "enamel"),
        ("neck", neck, "enamel"),
        ("speed_selector", m.cylinder(.013, .007, (-.052, 0., .0275)), "rubber"),
        ("side_reference", m.box((.003, .005, .037), (.0225, .006, .127)), "witness"),
    )
    contacts = (m.col_cylinder("base_contact", .095, .024, (0., 0., .012)),
                m.col_box("neck_contact", (.044, .042, .111), (0., 0., .0755)))
    visuals, shell = cage()
    head = m.hinge("head", "Y", PIVOT, DROP_DEG, 14., visuals, shell,
                   spring=0., damping=.027, effort=1.9, friction=.018, mass=HEAD_MASS,
                   gravity=True, com=HEAD_COM, inertia=(.0048, .0033, .0036), armature=.0001)
    grip_size = (.038, .013, .012)
    knob_parts = (
        ("shaft", m.cylinder(.0055, .034, (0., -.039, PIVOT[2]), axis="y"), "rim"),
        ("grip", m.box(grip_size, KNOB_GRIP), "rubber"),
        ("index", m.box((.003, .0006, .006), (.015, -.0688, PIVOT[2])), "witness"),
    )
    knob = m.hinge("clamp", "Y", KNOB_AXIS, -50., 32., knob_parts,
                   (m.col_box("grip_contact", grip_size, KNOB_GRIP),), spring=0., damping=.013,
                   effort=1.2, friction=clamp.LOOSE_RUNNING_NM, mass=.021, armature=.0001)
    return m.mechanism("desk_fan", ORIGIN, (head, knob), LOOKS, components, contacts,
                       start=(("head", math.radians(DROP_DEG)), ("clamp", math.radians(clamp.LOOSE_DEG))),
                       free_mass=1.46, frame_com=(0.,0.,.037), frame_inertia=(.006,.006,.008))



def proof_weight():
    rows = (
        ("crown", (.052, .035, .005), (0., 0., .0025), "weight"),
        ("front_leg", WEIGHT_SIDES, (-.024, 0., -.0035), "weight"),
        ("rear_leg", WEIGHT_SIDES, (.024, 0., -.0035), "weight"),
        ("grip", (.030, .030, .021), tuple(WEIGHT_GRIP), "weight"),
    )
    visuals = [(name, m.box(size, center), color) for name, size, center, color in rows]
    collisions = [m.col_box(name + "_c", size, center) for name, size, center, _ in rows]
    for sign, name in ((-1, "near_pad"), (1, "far_pad")):
        pad = m.box((.029, .007, .002)).turned(-8.*sign, "x").moved((.0055, sign*.014, .0004))
        visuals.append((name, pad, "rubber"))
        collisions.append(m.col_hull(name + "_c", pad))
    return m.free("proof_weight", WEIGHT_REST, tuple(visuals), tuple(collisions), LOOKS,
                  WEIGHT_MASS, friction=(.63, .5), com=(0., 0., .0035))
