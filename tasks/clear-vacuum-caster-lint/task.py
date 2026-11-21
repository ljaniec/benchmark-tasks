"""Installed caster cleaning with a removable wheel and physical debris."""

import math
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as m
from caster_fixture import (
    LOOKS, METAL, RUBBER, SOCKET, TIRE_OFFSET, WHEEL_RADIUS, detent, part, pin_collar, platform,
    split_parts,
)
from proof import CasterProof, Observation, SpinReference
from service_station import CATCH_MAX, CATCH_MIN, FIBRE_TRAY, SLOT_HOME, caster_slot, wheel_rest
from yam_arena.constants import GRIPPER


SCENE = "station"
INSTRUCTION = "Clear the material jamming the front caster, refit the original parts, and show that the wheel turns freely."
RECREATE_SCENE_ON_RESET = True
SUCCESS_HOLD_S = 0.
GRIPPER["effort"] = 40.

AXLE_RADIUS = .002
AXLE_HALF_LENGTH = .0155
AXLE_END_STOP_Y = .0159
TREAD_HALF_WIDTH = .0085
FORK_INNER_Y = .0125
FORK_OUTER_Y = .018
DEBRIS_ZONE = FIBRE_TRAY
HOLDER_TOP = SLOT_HOME
LOOKS = {
    **LOOKS,
    "debris": m.Look(color=m.rgb(.39, .21, .10), roughness=.95),
    "tray": m.Look(color=m.rgb(.88, .82, .59), roughness=.82),
    "stripe_main": m.Look(color=m.rgb(1., .89, .21), roughness=.77),
    "stripe_aux": m.Look(color=m.rgb(.93, .94, .93), roughness=.80),
}


def wheel_detent():
    """Two spring strips retain both axle ends and flex under each end's ramp."""
    clips = []
    for side, sign in (("near", -1), ("far", 1)):
        # The lower and upper ramps meet at the 3.2 mm throat. The bearing
        # pin is wider than the throat and must force both strips outward.
        outline = np.array([
            [.0048, .0251], [.0016, .0273], [.0016, .0281],
            [.0058, .0324], [.0073, .0324], [.0073, .0251],
        ])
        xz = outline * [sign, -1.]
        if sign < 0:
            xz = xz[::-1]
        leaves = []
        for end, y in (("left", .0143), ("right", -.0143)):
            mesh = m.prism(xz, y - .00165, y + .00165).aimed("+Y")
            leaves.append(part(f"{side}_{end}_leaf", mesh, "fork"))
        visuals, colliders = split_parts(leaves)
        low, high = (-.002, 0.) if sign < 0 else (0., .002)
        clips.append(m.slider(side, "X", low, high, visuals, colliders,
                              mass=.002, spring=3400., damping=1.8, friction=.06,
                              effort=12., armature=.00005))
    return clips


def service_fork():
    feet = [
        part("foot", m.cylinder(.014, .006, (0., 0., .003)), "fork"),
        part("stem", m.cylinder(.00345, .010, (0., 0., -.005)), METAL),
        pin_collar(),
    ]
    width = FORK_OUTER_Y - FORK_INNER_Y
    for name, sign in (("left", 1), ("right", -1)):
        y = sign * (FORK_INNER_Y + width / 2)
        feet.append(part(f"{name}_bearing", m.box((.027, width, .0168), (0., y, .0129)), "fork"))
        # Open bearing slot. Steel remains free to leave through the
        # spring throat without removing or hiding any collision surface.
        for leg, x in (("front", -.007425), ("back", .007425)):
            feet.append(part(f"{name}_{leg}_cheek", m.box((.01015, width, .0056), (x, y, .0242)), "fork"))
        # The closed ends prevent the shaft from sliding out sideways
        # under pressure from the packed lint. The vertical entrance is
        # still unobstructed when the detent spreads.
        # Beveled mouth guides the axle without changing the seated end stop.
        end_profile = np.array([
            [AXLE_END_STOP_Y, .020], [FORK_OUTER_Y, .020],
            [FORK_OUTER_Y, .029], [.0176, .029], [AXLE_END_STOP_Y, .026],
        ])
        end_profile[:, 0] *= sign
        if sign < 0:
            end_profile = end_profile[::-1]
        end_profile = end_profile[:, ::-1] * [-1., 1.]
        cap = m.prism(end_profile, -.008, .008).aimed("+X")
        feet.append(part(f"{name}_end_stop", cap, "fork"))
    # A ledge below the tucked debris supports its own weight in the gap.
    # It stops short of the rotating tire side face.
    feet.append(part("inside_wall_step", m.box((.017, .0032, .0022), (0., .0109, .0171)), "fork"))
    visuals, collisions = split_parts(feet)
    return m.mechanism("caster_module", SOCKET, wheel_detent(), LOOKS, visuals, collisions,
                       free_mass=.030, frame_inertia=(2e-6, 2e-6, 2e-6), friction=(.62, .48))


def removable_wheel():
    axle = part("steel_axle", m.cylinder(AXLE_RADIUS, AXLE_HALF_LENGTH * 2, axis="y"), METAL)
    tire, tire_collision = part(
        "tread", m.cylinder(WHEEL_RADIUS, TREAD_HALF_WIDTH * 2, axis="y"), RUBBER,
    )
    # Paint marks are visual meshes only. They add no mass or collision.
    stripe = m.box((.0038, .0145, .00055), (0., 0., WHEEL_RADIUS))
    marks = [(f"tread_index_{i}", stripe.turned(i * 60., "y"),
              "stripe_main" if i == 0 else "stripe_aux") for i in range(6)]
    mass = .023
    polar_inertia = mass * WHEEL_RADIUS ** 2 / 2
    lateral_inertia = mass * (3 * WHEEL_RADIUS ** 2 + (2 * TREAD_HALF_WIDTH) ** 2) / 12
    # A light painted index marks the position. It does not unbalance the wheel.
    wheel = m.hinge("wheel", "Y", (0., 0., 0.), -math.inf, math.inf, (tire, *marks), (tire_collision,),
                    mass=mass, damping=.000002, friction=.000005, gravity=True, armature=.0000003,
                    com=(0., 0., 0.), inertia=(lateral_inertia, polar_inertia, lateral_inertia))
    return m.mechanism("wheel_and_axle", SOCKET + TIRE_OFFSET, [wheel], LOOKS, [axle[0]], [axle[1]],
                       free_mass=.004, frame_inertia=(3.4e-7, 9.e-9, 3.4e-7),
                       friction=(.68, .52))


def compressed_debris():
    # Two contacting faces and one passive spring approximate the bulk
    # compression of a packed piece of lint. Its spring never drives the
    # wheel. Only friction at the real side face can slow the wheel.
    wad = m.box((.010, .0018, .010), (0., 0., .005))
    flap = m.box((.008, .0015, .008), (-.004, -.00015, .012)).turned(-15., "y")
    frame = [
        part("packed_fibres", wad, "debris"),
        part("loose_end", flap, "debris"),
    ]
    pad, pad_collision = part("rub_face", m.box((.008, .0013, .008), (0., -.00438, .005)), "debris")
    face = m.slider("compression", "Y", 0., .00255, [pad], [pad_collision],
                    mass=.0006, spring=1400., damping=.4, effort=2.5, armature=.00003)
    visuals, collisions = split_parts(frame)
    initial_pos = SOCKET + [0., .01149, .0183]
    return m.mechanism("lint", initial_pos, [face], LOOKS, visuals, collisions,
                       free_mass=.002, frame_inertia=(5e-8, 5e-8, 5e-8), friction=(.88, .72),
                       start=(("compression", .00204),))


def debris_tray():
    bottom = part("bottom", m.box((.074, .074, .003), (0., 0., .0015)), "tray")
    walls = []
    for suffix, sign in (("near", -1), ("far", 1)):
        walls.append(part(f"{suffix}_end", m.box((.003, .074, .006), (sign * .0355, 0., .006)), "tray"))
        walls.append(part(f"{suffix}_side", m.box((.068, .003, .006), (0., sign * .0355, .006)), "tray"))
    visuals, collisions = split_parts([bottom, *walls])
    return m.fixed("debris_tray", DEBRIS_ZONE, visuals, collisions, LOOKS)


OBJECTS = [
    platform(), detent(), service_fork(), removable_wheel(), compressed_debris(),
    debris_tray(), caster_slot(), wheel_rest(),
]
# Provisional engineering thresholds. Replace with measured clean-coast
# values before the full task is eligible for submission.
REFERENCE = SpinReference(.35, 3., math.radians(15.))
_proof = CasterProof(REFERENCE)
_score = 0.


@m.on_reset
def reset(env):
    global _proof, _score
    _proof, _score = CasterProof(REFERENCE), 0.
    # Run after Arena's settling pass to initialize free assemblies to their
    # actual installed poses and clear any contact history from the episode.
    base = env.unwrapped
    for name in ("detent_receiver", "caster_module", "wheel_and_axle", "lint"):
        asset = base.scene[name]
        pose = asset.data.default_root_pose.torch.clone()
        pose[:, :3] += base.scene.env_origins
        asset.write_root_pose_to_sim_index(root_pose=pose)
        asset.write_root_velocity_to_sim_index(root_velocity=asset.data.default_root_vel.torch.clone())
        position = asset.data.default_joint_pos.torch.clone()
        velocity = asset.data.default_joint_vel.torch.clone()
        asset.write_joint_state_to_sim_index(position=position, velocity=velocity)
        asset.actuators.target_command.set_position_index(value=position)
        asset.actuators.target_command.set_velocity_index(value=velocity)
    m.set_target(env, "lint", "compression", 0.)
    base.sim.forward()
    for _ in range(3):
        base.sim.render()
    for name in ("top_cam", "left_cam", "right_cam"):
        base.scene[name].update(0., force_recompute=True)


@m.on_step
def update(env):
    global _score
    fork = m.state(env, "caster_module")
    wheel = m.state(env, "wheel_and_axle")
    lint = m.state(env, "lint")
    fork_rot, wheel_rot = fork.rotation(), wheel.rotation()
    mounted = np.linalg.norm(fork.pos - SOCKET) <= .002 and fork_rot[2, 2] >= .996
    relative = fork_rot.T @ (wheel.pos - fork.pos)
    axles_aligned = abs(float(wheel_rot[:, 1] @ fork_rot[:, 1])) >= .996
    wheel_seated = np.linalg.norm(relative - TIRE_OFFSET) <= .0015 and axles_aligned
    in_tray = np.linalg.norm(lint.pos[:2] - DEBRIS_ZONE[:2]) <= .032 and .749 <= lint.pos[2] <= .766
    in_catch = np.all(lint.pos >= CATCH_MIN) and np.all(lint.pos <= CATCH_MAX)
    parked_lint = ((in_tray or in_catch) and np.linalg.norm(lint.pos - fork.pos) >= .028
                   and np.linalg.norm(lint.lin_vel) <= .015)
    module_removed = fork.pos[2] - SOCKET[2] >= .030 and wheel_seated
    hands_clear = np.all(np.linalg.norm(m.fingertips(env) - wheel.pos, axis=1) >= .042)
    # Measure the visible tread index against the housing. The entire pin
    # is free in its bearing and can take some of the absolute rotation.
    assembly = env.unwrapped.scene["wheel_and_axle"]
    wheel_pose = m._np(assembly.data.body_link_pose_w)[0, assembly.body_names.index("wheel")]
    marker = fork_rot.T @ m.quat_matrix(wheel_pose[3:7])[:, 2]
    angle = math.atan2(marker[0], marker[2])
    parts = np.stack([fork.pos, wheel.pos, lint.pos])
    intact = np.isfinite(parts).all() and np.all(parts >= [.10, -.34, .738]) and np.all(parts <= [.64, .31, 1.15])
    _score = _proof.update(Observation(
        module_removed=module_removed,
        axle_and_wheel_intact=bool(intact),
        debris_cleared=bool(parked_lint),
        original_parts_seated=bool(mounted and wheel_seated),
        hands_clear=bool(hands_clear),
        angle_rad=angle,
    ))


def score(_objects):
    return _score


def success(objects):
    return score(objects) == 1.


def demo():
    from demo import demo as scripted_motion

    return scripted_motion()
