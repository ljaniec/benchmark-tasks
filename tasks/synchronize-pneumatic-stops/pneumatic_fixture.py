"""Provisional mounted geometry for the paired-stop diagnostic cycle."""
import math

import mechkit as m
from yam_arena.station import TABLE_TOP


ORIGIN = (.420, 0., TABLE_TOP)
YAW = 15.
STROKE = .080
KNOB = (-.060, .165, .102)
BUTTON = (-.115, -.175, .080)
PLATE_WIDTH = .428
TEETH = 12
CAP_HEIGHT = .006
CAP_LIFT = .0014
COLLAR_HEIGHT = .012
COLLAR_Z = KNOB[2] - CAP_HEIGHT / 2. - COLLAR_HEIGHT / 2. - .0002
LOOKS = {
    "plate": m.Look(m.rgb(.35, .39, .39), roughness=.66),
    "cylinder": m.Look(m.rgb(.74, .78, .78), metallic=.48, roughness=.50),
    "rod": m.Look(m.rgb(.52, .56, .60), metallic=.62, roughness=.43),
    "knob": m.Look(m.rgb(.15, .19, .19), roughness=.72),
    "hose": m.Look(m.rgb(.09, .35, .61), roughness=.67),
    "button": m.Look(m.rgb(.11, .59, .26), roughness=.54),
    "reference": m.Look(m.rgb(.89, .68, .15), roughness=.68),
    "test": m.Look(m.rgb(.80, .16, .12), roughness=.69),
    "guard": m.Look(m.rgb(.70, .79, .81), roughness=.24, opacity=.14),
}


def tight_contact(name, shape):
    return m.col_hull(name + "_collision", shape).replace("contactOffset = 0.004", "contactOffset = 0.00012")


def rods():
    links = []
    for name, y, color in (("reference", -.109, "reference"), ("adjusted", .109, "test")):
        shaft = m.cylinder(.006, .196, (.019, y, .067), axis="x")
        flag = m.box((.014, .033, .017), (-.073, y, .072))
        parts = (("piston_rod", shaft, "rod"), ("moving_flag", flag, color))
        links.append(m.slider(name, "X", -STROKE, 0., parts,
                              [tight_contact(n, mesh) for n, mesh, _ in parts],
                              spring=550., damping=12., effort=32., mass=.149, friction=.2, gravity=False))
    return links


def detent_teeth(center, pitch_offset):
    """Small coarse splines engage only when the push-lock cap is down."""
    for index in range(TEETH):
        angle = 360. * (index + pitch_offset) / TEETH
        tooth = m.box((.0013, .0011, .0007), (.0063, 0., 0.))
        yield tooth.turned(angle, "z").moved(center)


def indicator_collar(center):
    """The housing surrounds the lock splines and leaves room for them to seat."""
    for index in range(TEETH):
        a = math.tau * index / TEETH
        b = math.tau * (index + 1) / TEETH
        corners = [(radius * math.cos(angle), radius * math.sin(angle))
                   for radius, angle in ((.0076, a), (.010, a), (.010, b), (.0076, b))]
        yield m.prism(corners, -COLLAR_HEIGHT / 2., COLLAR_HEIGHT / 2.).moved(center)


def valve_cap():
    x, y, z = KNOB
    grip = m.cylinder(.006, CAP_HEIGHT, KNOB, segments=12)
    index = m.box((.004, .0014, .0004), (x+.002, y, z+CAP_HEIGHT/2.+.0002))
    teeth = list(detent_teeth((x, y, z-.0034), 0.))
    parts = [("grip", grip, "knob"), ("index", index, "reference")]
    parts.extend((f"tooth_{n}", shape, "rod") for n, shape in enumerate(teeth))
    contacts = [tight_contact("grip", grip)]
    contacts.extend(tight_contact(f"face_{n}", shape) for n, shape in enumerate(teeth))
    lift = m.slider("cap_lift", "Z", -.0001, CAP_LIFT, (), (), at=KNOB,
                    spring=65., damping=.25, friction=.35, effort=4., mass=.004)
    turn = m.hinge("flow_knob", "Z", KNOB, -720., 720., parts, contacts,
                   parent="cap_lift", damping=.009, friction=.025, mass=.011, armature=1.e-6)
    return lift, turn


def station_model():
    rows = [
        ("plate", (.408, PLATE_WIDTH, .034), (0., 0., .017), "plate"),
        ("button_box", (.061, .056, .042), (BUTTON[0], BUTTON[1], .055), "plate"),
        ("protective_shield", (.138, .283, .008), (.062, 0., .140), "guard"),
    ]
    for sign in (-1., 1.):
        side = "left" if sign > 0 else "right"
        valve_y = sign * KNOB[1]
        rod_y = sign * .109
        rows += [
            (side+"_barrel_top", (.138, .051, .023), (.093, rod_y, .0865), "cylinder"),
            (side+"_barrel_bottom", (.138, .051, .023), (.093, rod_y, .0475), "cylinder"),
            (side+"_barrel_wall_in", (.138, .0175, .016), (.093, rod_y-.01675, .067), "cylinder"),
            (side+"_barrel_wall_out", (.138, .0175, .016), (.093, rod_y+.01675, .067), "cylinder"),
            (side+"_rod_shield", (.184, .046, .006), (-.086, sign*.109, .121), "guard"),
            (side+"_valve_body", (.031, .024, .034), (-.060, valve_y, .051), "plate"),
            (side+"_valve_hex", (.014, .014, .022), (-.060, valve_y, .079), "rod"),
            (side+"_rail_support", (.027, .030, .103), (.100, sign*.063, .0855), "plate"),
            (side+"_finish_mark", (.004, .038, .002), (-.153, sign*.109, .036), "reference"),
        ]
    frame = [(name, m.box(size, center), color) for name, size, center, color in rows]
    contacts = [m.col_box(name + "_collision", size, center).replace(
        "contactOffset = 0.004", "contactOffset = 0.00012") for name, size, center, _ in rows]
    for side, y in (("reference", -KNOB[1]), ("adjusted", KNOB[1])):
        for n, shape in enumerate(indicator_collar((-.060, y, COLLAR_Z))):
            name = f"{side}_valve_indicator_{n}"
            frame.append((name, shape, "cylinder"))
            contacts.append(tight_contact(name, shape))
        path = ((-.044, y, .081), (-.025, y, .081), (.020, y*.86, .063), (.080, y*.86, .065))
        frame.append((side+"_exhaust_hose", m.tube(path, .003), "hose"))
    lock_center = (KNOB[0], KNOB[1], KNOB[2]-.0034)
    for n, shape in enumerate(detent_teeth(lock_center, .5)):
        frame.append((f"lock_spline_{n}", shape, "rod"))
        contacts.append(tight_contact(f"lock_spline_{n}", shape))
    cover = m.cylinder(.011, .024, (-.060, -KNOB[1], .100))
    reference_cap = m.cylinder(.006, CAP_HEIGHT, (-.060, -KNOB[1], KNOB[2]), segments=12)
    frame.append(("reference_cap", reference_cap, "knob"))
    frame.append(("reference_knob_cover", cover, "guard"))
    contacts.append(tight_contact("reference_knob_cover", cover))
    button = m.button("test_button", BUTTON, outward="+Z", radius=.009, height=.010,
                      travel=.004, spring=700., damping=1.4, look="button")
    return m.mechanism("paired_fixture", ORIGIN, (*rods(), *valve_cap(), button), LOOKS,
                       frame, contacts, yaw_deg=YAW, self_collide=True)
