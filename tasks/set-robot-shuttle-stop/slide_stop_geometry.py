"""Model the original adjustment screw and stopped carriage at product scale."""
from dataclasses import replace
import math

import numpy as np

import mechkit as m


NAME = "shuttle_axis"
BODY_LENGTH = .074
BODY_WIDTH = .035
AXIS = (.094, -.011, .017)
STOP_FACE_MM = 81.
THREAD_PITCH = .0005
STROKE_M = .030
NOMINAL_FORCE_N = 15.
SLIDE_MASS = .083
PAYLOAD_MASS = .150
CHECK_SPEED_M_S = .080
LOOKS = {
    "aluminum": m.Look(m.rgb(.69, .74, .77), metallic=.35, roughness=.49),
    "steel": m.Look(m.rgb(.47, .50, .52), metallic=.67, roughness=.58),
    "rubber": m.Look(m.rgb(.23, .23, .24), roughness=.76),
    "payload": m.Look(m.rgb(.12, .36, .61), roughness=.63),
    "reference": m.Look(m.rgb(.82, .53, .12), roughness=.57),
}


def contact(name, mesh):
    return m.col_hull(name, mesh).replace("contactOffset = 0.004", "contactOffset = 0.00008")


def adjuster_socket():
    flats = .00306
    radius = flats / math.sqrt(3.)
    sections = []
    angles = np.linspace(0., 2. * math.pi, 7)
    for i, (start, stop) in enumerate(zip(angles[:-1], angles[1:], strict=True)):
        outline = [(r * math.cos(a), r * math.sin(a))
                   for r, a in ((radius, start), (.0034, start), (.0034, stop), (radius, stop))]
        mesh = m.prism(outline, 0., .0042).aimed("-X").moved(AXIS)
        sections.append((f"socket_wall_{i}", mesh, "steel"))
    shoulder = m.cylinder(.0034, .002, (.0888, AXIS[1], AXIS[2]), axis="x")
    sections.append(("socket_bottom", shoulder, "steel"))
    return sections


def couple(stage):
    tag = 'def PhysicsPrismaticJoint "stop_sleeve_joint" (\n'
    tag += '        prepend apiSchemas = ["PhysicsDriveAPI:linear"]'
    joint = f"rel physics:body1 = </P_{NAME}/stop_sleeve>"
    if stage.count(tag) != 1 or stage.count(joint) != 1:
        raise ValueError("Missing native sleeve adjustment joint")
    stage = stage.replace(tag, tag[:-1] + ', "PhysxMimicJointAPI:rotZ"]')
    return stage.replace(joint, joint + f"""
        uniform token physxMimicJoint:rotZ:referenceJointAxis = "rotZ"
        rel physxMimicJoint:rotZ:referenceJoint = </P_{NAME}/adjuster_joint>
        float physxMimicJoint:rotZ:gearing = {THREAD_PITCH / 360.:.12g}
        float physxMimicJoint:rotZ:offset = 0
""")


def axis(origin=(0., 0., 0.), stop_face_mm=STOP_FACE_MM, yaw_deg=0.):
    body = (
        ("base", m.box((BODY_LENGTH, BODY_WIDTH, .012), (.037, 0., .006)), "aluminum"),
        ("guide", m.box((.069, .015, .007), (.036, .005, .013)), "steel"),
        ("stop_mount", m.box((.013, .012, .008), (.087, -.011, .008)), "aluminum"),
        ("rear_reference", m.box((.0007, .010, .0008), (.053, .017, .0126)), "reference"),
    )
    carriage = (
        ("yoke", m.box((.034, .032, .003), (.037, .001, .0185)), "steel"),
        ("stop_lug", m.box((.010, .009, .006), (.049, -.011, .0168)), "steel"),
        ("bolted_workholding_block", m.box((.031, .026, .016), (.034, 0., .028)), "payload"),
    )
    stroke = m.slider("carriage", "X", 0., STROKE_M + .0004, carriage,
                      tuple(contact(f"{name}_c", mesh) for name, mesh, _ in carriage),
                      at=(.043, 0., .026), mass=SLIDE_MASS + PAYLOAD_MASS, friction=.23,
                      armature=.002, damping=1., effort=NOMINAL_FORCE_N)
    socket = adjuster_socket()
    shank = m.hinge("adjuster", "X", AXIS, -10., 1600., socket,
                    tuple(contact(name + "_c", mesh) for name, mesh, _ in socket),
                    armature=.00002, friction=.007, damping=.002, mass=.014, effort=.13)
    sleeve_x = stop_face_mm * .001 + .0022
    seat = (sleeve_x, AXIS[1], AXIS[2])
    collar = []
    edges = np.linspace(0., 2. * math.pi, 13)
    for i, (a, b) in enumerate(zip(edges[:-1], edges[1:], strict=True)):
        cross_section = [(r * math.cos(t), r * math.sin(t))
                         for r, t in ((.00255, a), (.0034, a), (.0034, b), (.00255, b))]
        wall = m.prism(cross_section, -.0022, .0022).aimed("+X").moved(seat)
        collar.append((f"seat_wall_{i}", wall, "steel"))
    sleeve = m.slider("stop_sleeve", "X", -.0021, .0001, tuple(collar),
                      tuple(contact(name + "_c", mesh) for name, mesh, _ in collar),
                      at=seat, mass=.014, damping=1., friction=.009, effort=20.)
    rubber_x = stop_face_mm * .001 - .00135
    rubber = m.cylinder(.0024, .0027, (rubber_x, AXIS[1], AXIS[2]), axis="x")
    bumper = m.slider("buffer", "X", 0., .0028, (("buffer_pad", rubber, "rubber"),),
                      (contact("buffer_c", rubber),), parent="stop_sleeve",
                      at=(rubber_x, AXIS[1], AXIS[2]), mass=.008, damping=1.7, spring=1500., effort=8.)
    from clamp_load import LOOSE_ANGLE_RAD
    screw, bosses = clamp_geometry(sleeve_x)
    body += bosses
    fixed = tuple(contact(name + "_c", mesh) for name, mesh, _ in body)
    part = m.mechanism(NAME, origin, (stroke, shank, sleeve, bumper, screw), LOOKS, body, fixed,
                       yaw_deg=yaw_deg, start=(("clamp", -LOOSE_ANGLE_RAD),),
                       self_collide=True, solver_iters=36, friction=(.62, .48))
    return replace(part, build=lambda: couple(part.build()))


def clamp_center(sleeve_x):
    return sleeve_x + .0018, AXIS[1] - .0052, .023


def clamp_geometry(sleeve_x):
    """Expose the vertical clamp screw beside the original horizontal stop."""
    at = clamp_center(sleeve_x)
    angles = np.linspace(0., math.tau, 7)
    shell = []
    for i, (a, b) in enumerate(zip(angles[:-1], angles[1:], strict=True)):
        points = [(r * math.cos(t), r * math.sin(t))
                  for r, t in ((.00206 / math.sqrt(3), a), (.00245, a), (.00245, b), (.00206 / math.sqrt(3), b))]
        mesh = m.prism(points, -.0032, 0.).moved(at)
        shell.append((f"clamp_socket_{i}", mesh, "steel"))
    head_floor = m.cylinder(.00245, .0008, (at[0], at[1], at[2] - .0036))
    shell.append(("clamp_socket_bottom", head_floor, "steel"))
    screw = m.hinge("clamp", "Z", at, -29., 92., tuple(shell),
                    tuple(contact(f"{name}_c", mesh) for name, mesh, _ in shell),
                    armature=.00009, damping=.008, friction=.008, mass=.009, effort=.42, gravity=False)
    bosses = (
        ("split_clamp_outer", m.box((.008, .004, .0183), (at[0], at[1] - .0022, .0104)), "aluminum"),
        ("split_clamp_inner", m.box((.008, .0024, .0147), (at[0], at[1] + .0013, .0074)), "aluminum"),
    )
    return screw, bosses
