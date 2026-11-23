"""Compose the guarded feeder with a screw adjustment and test button."""
from dataclasses import replace

import feeder_geometry as f
from yam_arena import station


ORIGIN = (.300, 0., station.TABLE_TOP + f.BASE_DEPTH)
KNOB = (0., -.090, .063)
BUTTON = (.100, .102, .042)
START_GAP = .0099
LEAD_M = .008
ADJUSTMENT_DEG = 198.
MADE_FACES = (
    ("separator_bearing", (.037, .022, .080), (0., -.090, .023), "panel"),
    ("stop_control_base", (.035, .033, .059), (.100, .102, .0125), "deck"),
)
STOCK_STOP = (("aligned_infeed_stop", (.008, .090, .019), (-.1155, 0., f.DECK+.0095), "deck"),)


def couple_separator(stage):
    slider = ('def PhysicsPrismaticJoint "separator_joint" (\n'
              '        prepend apiSchemas = ["PhysicsDriveAPI:linear"]')
    body = "rel physics:body1 = </P_carton_feeder/separator>"
    if stage.count(slider) != 1 or stage.count(body) != 1:
        raise ValueError("The separator slide could not be coupled to its adjustment screw.")
    stage = stage.replace(slider, slider[:-1] + ', "PhysxMimicJointAPI:rotZ"]')
    return stage.replace(body, body + f"""
        uniform token physxMimicJoint:rotZ:referenceJointAxis = "rotZ"
        rel physxMimicJoint:rotZ:referenceJoint = </P_carton_feeder/adjuster_joint>
        float physxMimicJoint:rotZ:gearing = {LEAD_M / 360.:.12g}
        float physxMimicJoint:rotZ:offset = 0
""")


def mounted_machine():
    m = f.m
    knob = m.knob("adjuster", KNOB, look="cyan", mark="white", hub_r=.011, hub_h=.007,
                  wing=(.050, .014, .019), lower=-35., upper=375.,
                  friction=.047, damping=.032, mass=.042)
    knob = replace(knob, colliders=tuple(f.exact_contact(collider) for collider in knob.colliders))
    button = m.button("proof_button", BUTTON, look="gate", radius=.013,
                      height=.012, travel=.006, spring=720., damping=2.)
    base = f.machine(ORIGIN, START_GAP, extra_specs=MADE_FACES, controls=(knob, button),
                     gap_reference=START_GAP)
    return replace(base, build=lambda: couple_separator(base.build()))


def parts():
    m = f.m
    beam = .049
    stripes = [(f"exit_mark_{label}", m.box((.003, .011, .00035), (beam, side*.050, .0512)), "cyan")
               for label, side in (("near", -1), ("far", 1))]
    output_mark = m.fixed("output_witness", ORIGIN, stripes, looks=f.LOOKS)
    stock_stop = f.build_fixed("infeed_stock_stop", ORIGIN, STOCK_STOP, (.16, .12))
    return mounted_machine(), stock_stop, output_mark, *f.secondary_parts(ORIGIN)
