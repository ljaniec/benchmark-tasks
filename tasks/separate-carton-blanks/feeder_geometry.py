"""Roller and separator contact coupon for thick carton blanks."""
import math

import mechkit as m


THICKNESS = .0042
LENGTH = .095
WIDTH = .085
DECK = .028
BASE_DEPTH = .017
MASS = .020
ROLLER_RADIUS = .018
ROLLER_X = (-.057, -.014, .030, .074, .116)
DRIVE_RAD_S = 4.
ROLLER_EFFORT = .080
ROLLER_DAMPING = .045
GATE_STIFFNESS = 3600.
GATE_DAMPING = 3.6
GATE_EFFORT = 19.
MACHINE = "carton_feeder"
GUARD = "feeder_guard"
STOCK = ("blank_bottom", "blank_top")
GUARD_SPECS = (
    ("clear_top", (.137, .119, .003), (.026, 0., .076), "guard"),
    ("inlet_shroud", (.010, .098, .037), (-.043, 0., .0615), "guard"),
    ("outlet_shroud", (.010, .098, .037), (.095, 0., .0615), "guard"),
)
LOOKS = {
    "deck": m.Look(m.rgb(.34, .38, .42), roughness=.60),
    "roller": m.Look(m.rgb(.13, .18, .21), roughness=.82),
    "panel": m.Look(m.rgb(.66, .72, .75), roughness=.55),
    "gate": m.Look(m.rgb(.73, .51, .19), roughness=.50),
    "guard": m.Look(m.rgb(.73, .84, .94), roughness=.30, opacity=.16),
    "card": m.Look(m.rgb(.76, .59, .36), roughness=.77),
    "cyan": m.Look(m.rgb(.10, .50, .69), roughness=.69),
    "white": m.Look(m.rgb(.97, .91, .77), roughness=.69),
}


def exact_contact(shape):
    return shape.replace("contactOffset = 0.004", "contactOffset = 0.00009")


def frame_specs():
    rails = [(f"rail_{label}", (.272, .013, .063), (.018, side * .050, .0195), "deck")
             for label, side in (("near", -1), ("far", 1))]
    return (
        ("base", (.322, .137, .005), (.025, 0., -BASE_DEPTH + .0025), "panel"),
        *rails,
        ("bridge", (.032, .102, .022), (0., 0., .059), "panel"),
    )


def support_specs():
    return (
        ("infeed", (.037, .091, .006), (-.094, 0., DECK-.003), "deck"),
        ("spacer_a", (.010, .091, .006), (-.035, 0., DECK-.003), "deck"),
        ("spacer_b", (.010, .091, .006), (.008, 0., DECK-.003), "deck"),
        ("spacer_c", (.010, .091, .006), (.052, 0., DECK-.003), "deck"),
        ("spacer_d", (.010, .091, .006), (.095, 0., DECK-.003), "deck"),
        ("exit_deck", (.110, .091, .006), (.185, 0., DECK-.003), "deck"),
        ("exit_stop", (.008, .100, .007), (.242, 0., DECK+.0035), "deck"),
    )


def build_fixed(name, origin, shapes, friction):
    return m.fixed(name, origin, [(label, m.box(size, at), look) for label, size, at, look in shapes],
                   [exact_contact(m.col_box(label + "_contact", size, at)) for label, size, at, _ in shapes],
                   LOOKS, friction=friction)


def machine(origin, start_gap, *, extra_specs=(), controls=(), gap_reference=0.):
    specs = (*frame_specs(), *extra_specs)
    frame = [(name, m.box(dim, position), look) for name, dim, position, look in specs]
    frame_contact = [exact_contact(m.col_box(name+"_contact", dim, position))
                     for name, dim, position, _ in specs]
    rollers = []
    for i, x in enumerate(ROLLER_X, 1):
        name = f"feed_roller_{i}"
        at = (x, 0., DECK - ROLLER_RADIUS + .00025)
        mesh = m.cylinder(ROLLER_RADIUS, .084, at, axis="y", segments=48)
        rollers.append(m.hinge(name, "Y", at, -math.inf, math.inf, ((name, mesh, "roller"),),
                               (exact_contact(m.col_hull(name + "_surface", mesh)),),
                               mass=.078, damping=ROLLER_DAMPING, friction=.0001, effort=ROLLER_EFFORT))
    size = (.015, .093, .021)
    at = (0., 0., DECK+.0105+gap_reference)
    gate = m.slider("separator", "Z", .001-gap_reference, .014-gap_reference,
                    (("separator_face", m.box(size, at), "gate"),),
                    (exact_contact(m.col_box("separator_contact", size, at)),),
                    at=at, mass=.08, damping=GATE_DAMPING, spring=0. if gap_reference else GATE_STIFFNESS,
                    effort=GATE_EFFORT)
    return m.mechanism(MACHINE, origin, (gate, *rollers, *controls), LOOKS, frame, frame_contact,
                       friction=(1.15, .95), start=(("separator", start_gap-gap_reference),))


def cards(origin):
    x = -.063
    for i, name in enumerate(STOCK):
        color = ("cyan", "white")[i]
        size = (LENGTH, WIDTH, THICKNESS)
        body = m.box(size)
        stripe = m.box((.010, WIDTH*.80, .00012), (.033, 0., THICKNESS*.50+.00008))
        # Both sheets begin in the same channel, one lying flat on the other.
        at = (origin[0]+x, origin[1], origin[2]+DECK+.0004+(i+.5)*THICKNESS)
        yield m.free(name, at, (("board", body, "card"), ("lip_mark", stripe, color)),
                     (exact_contact(m.col_box("board_contact", size)),), LOOKS, MASS,
                     friction=(.53, .40), torsional_patch=None)


def secondary_parts(origin):
    guides = build_fixed("stock_guide", origin, support_specs(), (.09, .065))
    guard = build_fixed(GUARD, origin, GUARD_SPECS, (.16, .12))
    return guides, guard, *cards(origin)


def parts(origin, start_gap):
    return machine(origin, start_gap), *secondary_parts(origin)
