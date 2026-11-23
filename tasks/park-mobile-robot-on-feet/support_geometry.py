"""Native support assembly for a compact parked robot and leveling wheels."""
from dataclasses import replace
from itertools import combinations
import math

import mechkit as m
import numpy as np


WHEEL_RADIUS = .025
WHEEL_WIDTH = .025
TOP_PLATE_M = .082
CASTER_OFFSET_M = .038
PAD_RADIUS_M = .016
PAD_HEIGHT_M = .008
PAD_START_CLEARANCE_M = .0012
FRONT_X_M = -.155
REAR_X_M = .155
HALF_TRACK_M = .137
ARM_MASS_KG = 8.0
DECK_SIZE = (.430, .390, .006)
FRAME_MASS_KG = ARM_MASS_KG + 2700. * math.prod(DECK_SIZE)
PAD_DAMPING = 6.5
PAD_EFFORT_N = 130.
PEDAL_X = FRONT_X_M - .055
PEDAL_Z = -.031
PEDAL_MAX_DEG = 102.
PEDAL_LEN = .087
PEDAL_START_DEG = 75.
CAM_RADIUS = .009
CAM_ECCENTRICITY = .0080
NAME = "mobile_robot_stand"
LOOKS = {
    "frame": m.Look(m.rgb(.20, .26, .31), roughness=.60),
    "shell": m.Look(m.rgb(.78, .82, .83), roughness=.56),
    "steel": m.Look(m.rgb(.55, .60, .62), metallic=.42),
    "wheel": m.Look(m.rgb(.13, .16, .19), roughness=.56),
    "pad": m.Look(m.rgb(.19, .24, .25), roughness=.85),
    "mark": m.Look(m.rgb(.13, .60, .70), roughness=.61),
}


def pad_center(y):
    return (FRONT_X_M, y, -TOP_PLATE_M + PAD_START_CLEARANCE_M + PAD_HEIGHT_M / 2.)


def wheel_center(y):
    return (FRONT_X_M + CASTER_OFFSET_M, y, -TOP_PLATE_M + WHEEL_RADIUS)


def contact(name, mesh):
    return m.col_hull(name, mesh).replace("contactOffset = 0.004", "contactOffset = 0.0001")


def restrict_internal_contact(stage):
    """Permit only each cam's contact against its own follower."""
    names = ("frame", *(f"{part}_{side}" for side in ("left", "right")
                        for part in ("wheel", "pedal", "leveler")))
    for a, b in combinations(names, 2):
        if set((a, b)) in ({"pedal_left", "leveler_left"}, {"pedal_right", "leveler_right"}):
            continue
        tag = f'def Xform "{a}" (\n        prepend apiSchemas = ['
        start = stage.index(tag) + len(tag)
        if not stage[start:].startswith('"PhysicsFilteredPairsAPI", '):
            stage = stage[:start] + '"PhysicsFilteredPairsAPI", ' + stage[start:]
        section = stage.index("\n    {\n", start) + len("\n    {\n")
        relation = '        rel physics:filteredPairs = ['
        peer = f'</P_{NAME}/{b}>'
        if stage[section:].startswith(relation):
            end = stage.index("]", section)
            stage = stage[:end] + f", {peer}" + stage[end:]
        else:
            stage = stage[:section] + relation + peer + "]\n" + stage[section:]
    return stage


def pedal_tip(side, angle):
    theta = math.radians(PEDAL_START_DEG + angle)
    return np.array((PEDAL_X - PEDAL_LEN * math.cos(theta), side * HALF_TRACK_M,
                     PEDAL_Z + PEDAL_LEN * math.sin(theta)))


def pedal(side, sign):
    pivot = (PEDAL_X, sign * HALF_TRACK_M, PEDAL_Z)
    cam = m.cylinder(CAM_RADIUS, .013, (PEDAL_X - CAM_ECCENTRICITY, pivot[1], PEDAL_Z),
                     axis="y", segments=48)
    lever = m.box((PEDAL_LEN, .014, .008), (-PEDAL_LEN / 2., 0., 0.)).turned(PEDAL_START_DEG, "y").moved(pivot)
    handle = m.box((.041, .040, .014), (-PEDAL_LEN, 0., 0.)).turned(PEDAL_START_DEG, "y").moved(pivot)
    parts = (("eccentric_cam", cam, "steel"), ("lever", lever, "steel"), ("treadle", handle, "mark"))
    return m.hinge(f"pedal_{side}", "Y", pivot, -PEDAL_MAX_DEG, 0., parts,
                   tuple(contact(name + "_c", mesh) for name, mesh, _ in parts),
                   mass=.138, damping=.012, friction=.045, effort=2.2,
                   armature=.00019, gravity=False)


def fixture(origin):
    deck = m.box(DECK_SIZE, (0., 0., DECK_SIZE[2] / 2.))
    motor_base = m.box((.130, .140, .034), (.016, 0., .022))
    shoulder = m.cylinder(.046, .077, (.016, 0., .0705))
    upper = m.tube([(.016, 0., .099), (.083, .030, .236), (.132, .043, .285)], .027)
    elbow = m.sphere(.031, (.132, .043, .285))
    lower = m.tube([(.132, .043, .285), (.030, -.018, .224), (-.046, -.058, .188)], .023)
    wrist = m.cylinder(.030, .050, (-.046, -.058, .176))
    chassis = [
        ("steel_deck", deck, "frame"),
        ("robot_base", motor_base, "shell"),
        ("shoulder", shoulder, "steel"),
        ("upper_arm", upper, "shell"),
        ("elbow", elbow, "steel"),
        ("forearm", lower, "shell"),
        ("wrist", wrist, "steel"),
    ]
    moving = []
    for label, sign in (("left", 1.), ("right", -1.)):
        y = sign * HALF_TRACK_M
        casting = m.box((.073, .073, .033), (FRONT_X_M, y, -.0185))
        chassis.append((f"caster_housing_{label}", casting, "shell"))
        rear = m.cylinder(.019, TOP_PLATE_M, (REAR_X_M, y, -TOP_PLATE_M / 2.))
        chassis.append((f"rear_support_{label}", rear, "pad"))
        axle = wheel_center(y)
        wheel = m.cylinder(WHEEL_RADIUS, WHEEL_WIDTH, axle, axis="y", segments=48)
        moving.append(m.hinge(f"wheel_{label}", "Y", axle, -math.inf, math.inf,
                              ((f"wheel_{label}", wheel, "wheel"),), (contact(f"wheel_{label}_c", wheel),),
                              mass=.18, damping=.006, friction=.002, armature=.0001))
        moving.append(pedal(label, sign))
        center = pad_center(y)
        pad = m.cylinder(PAD_RADIUS_M, PAD_HEIGHT_M, center, segments=32)
        shaft = m.cylinder(.005, .055, (FRONT_X_M, y, center[2] + .030))
        follower_z = PEDAL_Z - CAM_RADIUS - .00315
        follower = m.box((.086, .015, .006), (FRONT_X_M - .028, y, follower_z))
        parts = ((f"pad_{label}", pad, "pad"), (f"shaft_{label}", shaft, "steel"),
                 (f"follower_{label}", follower, "steel"))
        moving.append(m.slider(f"leveler_{label}", "Z", -.010, .0001, parts,
                               (contact(f"pad_{label}_c", pad), contact(f"follower_{label}_c", follower)),
                               at=center, spring=110., damping=PAD_DAMPING, effort=PAD_EFFORT_N,
                               gravity=False, mass=.18, armature=.001))
    colliders = [contact(name+"_c", mesh) for name, mesh, _ in chassis]
    stand = m.mechanism(NAME, origin, moving, LOOKS, chassis, colliders,
                        free_mass=FRAME_MASS_KG, frame_com=(.025, 0., .067),
                        frame_inertia=(.211, .245, .274), friction=(.85, .66), solver_iters=48,
                        self_collide=True)
    return replace(stand, build=lambda: restrict_internal_contact(stand.build()))


def gaps(root_position, rotation, left_m, right_m):
    """Read the lower contact surfaces against a flat floor from a profile view."""
    floor_normal = np.asarray(rotation)[2]
    root_height = float(root_position[2])
    wheel_margin = (WHEEL_RADIUS * math.sqrt(1. - min(1., floor_normal[1]**2))
                    + WHEEL_WIDTH * .5 * abs(floor_normal[1]))
    pad_margin = (PAD_RADIUS_M * math.sqrt(1. - min(1., floor_normal[2]**2))
                  + PAD_HEIGHT_M * .5 * abs(floor_normal[2]))
    pad_gaps = []
    wheel_gaps = []
    for sign, displacement in ((1., left_m), (-1., right_m)):
        y = sign * HALF_TRACK_M
        pad = np.asarray(pad_center(y)) + (0., 0., displacement)
        pad_gaps.append(float(root_height + floor_normal @ pad - pad_margin))
        wheel_gaps.append(float(root_height + floor_normal @ wheel_center(y) - wheel_margin))
    return pad_gaps, wheel_gaps
