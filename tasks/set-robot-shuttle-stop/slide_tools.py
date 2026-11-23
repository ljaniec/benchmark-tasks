"""Seated original-size tooling for the mini-slide and its calibrated clamp."""
import math

import numpy as np

import clamp_load as load
import slide_stop_geometry as g
from yam_arena.ik import matrix_to_quat_xyzw
from yam_arena.station import TABLE_TOP
from yam_arena.urdf import axis_angle_matrix


ORIGIN = np.array((.438, .077, TABLE_TOP + .092))
YAW_DEG = -90.
STOP_FACE_MM = 80.125
ANGLE_ADJUST = math.radians(90.)
ADJUST_TOOL_NAME = "adjusting_key"
CLAMP_TOOL_NAME = "torque_key"
ADJUST_SHAFT_VECTOR = np.array((1., 0., 0.))
CLAMP_SHAFT_VECTOR = np.array((0., 0., 1.))
WRENCH_HANDLE_RADIUS = .064
WRENCH_HANDLE_SIZE = (.039, .036, .037)
BEAM_OFFSET_DEG = 35.
WRENCH_Z = .053
WEB_OFFSET = np.array((.027, 0., WRENCH_Z + .0035))
BASIS = axis_angle_matrix(CLAMP_SHAFT_VECTOR, math.radians(YAW_DEG))
ADJUST_TIP = np.asarray(g.AXIS) + (-.0026, 0., 0.)
CLAMP_CENTER = np.asarray(g.clamp_center(STOP_FACE_MM * .001 + .0022))
CLAMP_TIP = CLAMP_CENTER - (0., 0., .0022)
ADJUST_HANDLE = np.array((.093, 0., 0.))
ADJUST_NECK = np.array((.059, 0., 0.))
ADJUST_ROLL = 0.
CLAMP_ROLL = 30.
KEY_PARK_OFFSET = np.array((.022, 0., -.027))
METAL_LOOKS = dict(g.LOOKS, **{
    "plastic": g.m.Look(g.m.rgb(.14, .36, .47), roughness=.64),
    "dial": g.m.Look(g.m.rgb(.89, .89, .86), roughness=.70),
    "pointer": g.m.Look(g.m.rgb(.84, .46, .08), roughness=.65),
    "green": g.m.Look(g.m.rgb(.15, .52, .19), roughness=.65),
})


def point(local):
    return ORIGIN + BASIS @ local


def base():
    slab = g.m.box((.158, .108, .012), (.046, .019, -.006))
    post = g.m.box((.106, .071, ORIGIN[2] - TABLE_TOP - .012), (.048, .020, -(.012 + ORIGIN[2] - TABLE_TOP) / 2.))
    return g.m.fixed("robot_shuttle_pedestal", ORIGIN,
                     (("mount_plate", slab, "aluminum"), ("pedestal", post, "plastic")),
                     (g.contact("mount_plate_c", slab), g.contact("pedestal_c", post)),
                     METAL_LOOKS, yaw_deg=YAW_DEG)


def hex_bit(flats, thickness, offset, axis="z"):
    radius = flats / math.sqrt(3.)
    outline = [(radius * math.cos(a), radius * math.sin(a))
               for a in np.linspace(0., math.tau, 6, endpoint=False)]
    shape = g.m.prism(outline, -thickness / 2., thickness / 2.)
    if axis == "x":
        shape = shape.aimed("+X")
    return shape.moved(offset)


def adjust_key():
    bit = hex_bit(.003, .006, (.003, 0., 0.), axis="x")
    shank = g.m.cylinder(.0022, .070, (.038, 0., 0.), axis="x")
    handle = g.m.box((.045, .017, .038), ADJUST_HANDLE)
    parts = (("engaging_bit", bit, "steel"), ("shank", shank, "steel"), ("flat_handle", handle, "plastic"))
    rotation = BASIS @ axis_angle_matrix(ADJUST_SHAFT_VECTOR, math.radians(ADJUST_ROLL))
    return g.m.free(ADJUST_TOOL_NAME, point(ADJUST_TIP), parts,
                    tuple(g.contact(name + "_c", mesh) for name, mesh, _ in parts),
                    METAL_LOOKS, .087, start_quat=tuple(matrix_to_quat_xyzw(rotation)), friction=(.62, .47))


def adjust_rest():
    ref = point(ADJUST_TIP + ADJUST_NECK)
    pivot_height = ref[2] - TABLE_TOP
    block = g.m.box((.016, .011, pivot_height - .0078), (0., 0., (pivot_height - .0078) / 2.))
    bed_z = pivot_height - .0041
    bed = g.m.box((.015, .016, .004), (0., 0., bed_z))
    sides = [("lip_" + str(i), g.m.box((.002, .015, .0047), (.0042 * side, 0., bed_z + .0023)), "plastic")
             for i, side in enumerate((-1, 1))]
    solid = (("pedestal", block, "plastic"), ("saddle", bed, "plastic"), *sides)
    return g.m.fixed("key_support", (ref[0], ref[1], TABLE_TOP), solid,
                     tuple(g.contact(name + "_c", mesh) for name, mesh, _ in solid), METAL_LOOKS)


def wrench():
    bit = hex_bit(.002, .0044, (0., 0., .0022))
    shank = g.m.cylinder(.003, .049, (0., 0., .028))
    head = g.m.cylinder(.008, .007, (0., 0., WRENCH_Z))
    index = g.m.box((.079, .0024, .0012), (.0395, 0., WRENCH_Z + .0035))
    web = g.m.box((.008, .007, .005), WEB_OFFSET).turned(BEAM_OFFSET_DEG)
    # Put the reading on the heel of the tool, beyond the grasp and both fingers.
    scale_z = WRENCH_Z + .014
    low, high = (torque / load.TOOL_STIFFNESS_NM_PER_RAD for torque in load.TORQUE_BAND_NM)
    band_outline = [(-radius * math.cos(angle), radius * math.sin(angle)) for angle, radius in
                    ((low, .024), (high, .024), (high, .037), (low, .037))]
    band = g.m.prism(band_outline, scale_z + .0007, scale_z + .0011)
    ring = [
        ("tightening_band", band, "green"),
        ("scale_face", g.m.box((.018, .023, .0010), (-.030, .003, scale_z)), "dial"),
        ("scale_mount", g.m.box((.034, .003, .0020), (-.016, .0095, scale_z - .001)), "steel"),
    ]
    base = (("hex", bit, "steel"), ("shaft", shank, "steel"), ("driver_head", head, "steel"), ("support_web", web, "steel"),
            *tuple((name, mesh.turned(BEAM_OFFSET_DEG), color) for name, mesh, color in
                   (("fixed_index", index, "pointer"), *ring)))
    beam = g.m.box((.062, .0046, .006), (.035, 0., WRENCH_Z + .010))
    handle_at = (WRENCH_HANDLE_RADIUS, 0., WRENCH_Z + .012)
    handle = g.m.box(WRENCH_HANDLE_SIZE, handle_at)
    pointer = g.m.box((.034, .0012, .0014), (-.017, 0., scale_z + .002))
    faces = (("elastic_beam", beam.turned(BEAM_OFFSET_DEG), "steel"),
             ("moving_pointer", pointer.turned(BEAM_OFFSET_DEG), "pointer"))
    angle = g.m.hinge("beam", "Z", (0., 0., WRENCH_Z), -12., 12., faces,
                      (g.contact("beam_c", faces[0][1]),),
                      spring=load.TOOL_STIFFNESS_NM_PER_RAD, friction=0., damping=.034,
                      armature=.001, mass=.045, effort=.4, gravity=False,
                      inertia=(6.758e-5,) * 3)
    turn = axis_angle_matrix(CLAMP_SHAFT_VECTOR, math.radians(BEAM_OFFSET_DEG))
    swivel = g.m.hinge("swivel", "Z", turn @ handle_at, -360., 360.,
                        (("rectangular_handle", handle.turned(BEAM_OFFSET_DEG), "plastic"),),
                        (g.contact("handle_c", handle.turned(BEAM_OFFSET_DEG)),), parent="beam",
                        damping=.001, armature=.00005, friction=.002, mass=.027, gravity=False)
    start = BASIS @ axis_angle_matrix(CLAMP_SHAFT_VECTOR, math.radians(CLAMP_ROLL))
    return g.m.mechanism(CLAMP_TOOL_NAME, point(CLAMP_TIP), (angle, swivel), METAL_LOOKS, base,
                        tuple(g.contact(name + "_c", mesh) for name, mesh, _ in base[:4]),
                        start_quat=tuple(matrix_to_quat_xyzw(start)), free_mass=.080,
                        frame_com=(.016, 0., .033), frame_inertia=(.00009, .00008, .00004),
                        friction=(.63, .49))


def wrench_handle(degrees, lift=0.):
    turn = axis_angle_matrix(CLAMP_SHAFT_VECTOR, math.radians(degrees + BEAM_OFFSET_DEG))
    return point(CLAMP_TIP + turn @ (WRENCH_HANDLE_RADIUS, 0., WRENCH_Z + .012 + lift))


def wrench_rest_at():
    turn = axis_angle_matrix(CLAMP_SHAFT_VECTOR, math.radians(CLAMP_ROLL + BEAM_OFFSET_DEG))
    return point(CLAMP_TIP + turn @ WEB_OFFSET)


def wrench_rest():
    pos = wrench_rest_at()
    top = pos[2] - .0028
    body = g.m.box((.012, .013, top - TABLE_TOP))
    return g.m.fixed("beam_support", (pos[0], pos[1], (top + TABLE_TOP) / 2.),
                     (("rest", body, "plastic"),), (g.contact("rest_c", body),), METAL_LOOKS)


def parking_stations():
    adjust_offset = BASIS[:, 0] * .043 + KEY_PARK_OFFSET
    key_handle = point(ADJUST_TIP + ADJUST_HANDLE) + adjust_offset
    key_shank = point(ADJUST_TIP + ADJUST_NECK) + adjust_offset
    components = []
    for label, at, thickness in (("key_handle", key_handle, .019), ("key_shank", key_shank, .0022)):
        height = at[2] - thickness - TABLE_TOP - .0005
        mesh = g.m.box((.015, .017, height), (at[0], at[1], TABLE_TOP + height / 2.))
        components.append((label, mesh, "plastic"))
    return g.m.fixed("service_tool_parking", (0., 0., 0.), components,
                     tuple(g.contact(name + "_c", mesh) for name, mesh, _ in components), METAL_LOOKS)
