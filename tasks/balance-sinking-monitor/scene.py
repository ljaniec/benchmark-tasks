"""Draft desk-mounted counterbalance mechanics with a physically engaged hex key."""
from dataclasses import replace
import math
import json
import os
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as m
from camera_view import configure
from preflight import LINK_KG, MONITOR_KG, REACH, spring_leverage
from geometry import (BASE, KEY_REST_ROTATION, KEY_START, KEY_TIP_Y, MOUNT_ROTATION,
                      PANEL, PIVOT_Z, REST_LIFT_DEG, SOCKET_CENTER_Y, SOCKET_DEPTH, TABLE, YAW_DEG)
from scorer import ReleaseScore
from yam_arena.constants import GRIPPER
from yam_arena.ik import matrix_to_quat_xyzw


SCENE = "station"
INSTRUCTION = "Adjust the sinking monitor arm so it stays put after release at two working heights."
SOCKET_APOTHEM = .00316
BASE_FORCE_N = .9667 * (MONITOR_KG * REACH + .5 * LINK_KG * REACH) * 9.81 / spring_leverage(0.)
SCREW_PITCH_M = .001
# Provisional simulated preload slope. The modeled 4.2 kg display needs
# about 7.6 N more strut force to balance from this reset.
SCREW_EFFECTIVE_STIFFNESS_N_M = 37000.
_release = ReleaseScore()
_step = 0
GRIPPER["effort"] = 40.
SUCCESS_HOLD_S = 0.
RECREATE_SCENE_ON_RESET = True
configure()

LOOKS = {
    "base": m.Look(color=m.rgb(.19, .23, .26), roughness=.63, metallic=.23),
    "arm": m.Look(color=m.rgb(.32, .37, .39), roughness=.45, metallic=.26),
    "screen": m.Look(color=m.rgb(.076, .09, .11), roughness=.42),
    "bezel": m.Look(color=m.rgb(.16, .18, .21), roughness=.72),
    "steel": m.Look(color=m.rgb(.60, .65, .68), roughness=.36, metallic=.50),
    "grip": m.Look(color=m.rgb(.24, .48, .52), roughness=.68),
}


def close_surface(collider):
    return collider.replace("contactOffset = 0.004", "contactOffset = 0.0002")


def keyed_socket():
    """Open internal hex, with six independently colliding flats."""
    face_z = PIVOT_Z
    y = SOCKET_CENTER_Y
    thickness = .003
    inner_tangent = SOCKET_APOTHEM / math.sqrt(3.)
    mouth_depth = .003
    mouth_start = -SOCKET_DEPTH / 2.
    wall_center = mouth_depth / 2.
    flat_center = SOCKET_APOTHEM + thickness / 2.
    wall_width = 2. * inner_tangent + .004
    wall = m.box((wall_width, SOCKET_DEPTH - mouth_depth, thickness), (0., wall_center, flat_center))
    mouth = m.box((wall_width, mouth_depth, thickness), (0., mouth_start + mouth_depth / 2., flat_center))
    vertices = mouth.v.copy()
    vertices[:, 2] += .0011 * (mouth_start + mouth_depth - vertices[:, 1]) / mouth_depth
    lead = m.Mesh(vertices, mouth.f)
    meshes = []
    for i in range(6):
        angle = i * 60.
        meshes.append((f"socket_flat_{i}", wall.turned(angle, axis="y").moved((0., y, face_z)), "steel"))
        meshes.append((f"socket_mouth_{i}", lead.turned(angle, axis="y").moved((0., y, face_z)), "steel"))
    back = m.cylinder(.009, .004, (0., y + SOCKET_DEPTH / 2. + .002, face_z), axis="y")
    meshes.append(("socket_back", back, "steel"))
    collision = [close_surface(m.col_hull(name + "_shape", mesh)) for name, mesh, _ in meshes]
    return m.hinge("adjuster", "Y", (0., y, face_z), -1080., 1080., meshes, collision,
                   mass=.025, friction=.19, damping=.08, effort=1.4, armature=.003)


def monitor_mount():
    post_top = PIVOT_Z - .020
    post_base = TABLE + .017
    base = [
        ("grommet", m.cylinder(.049, .017, (0., 0., TABLE + .0085)), "base"),
        ("post", m.cylinder(.027, post_top - post_base, (0., 0., (post_base + post_top) / 2.)), "base"),
        ("hinge_support", m.box((.060, .038, .036), (0., .018, post_top + .018)), "base"),
    ]
    base_colliders = [m.col_hull(name + "_shape", mesh) for name, mesh, _ in base]
    beam = m.box((.062, REACH, .037), (0., REACH / 2., PIVOT_Z))
    lift = m.hinge("lift", "X", (0., 0., PIVOT_Z), -21., 21., [("arm", beam, "arm")],
                   [m.col_hull("arm_shape", beam)], mass=LINK_KG, com=(0., REACH / 2., PIVOT_Z),
                   gravity=True, friction=.18, damping=.38, effort=32., armature=.06)

    screen_center = (0., REACH, PIVOT_Z)
    body = m.box(PANEL, screen_center)
    glass = m.box((.509, .002, .280), (0., REACH + PANEL[1] / 2 + .0006, PIVOT_Z))
    vesa = m.box((.113, .007, .113), (0., REACH - .020, PIVOT_Z))
    display = m.hinge("screen_mount", "X", screen_center, -25., 25.,
                      [("case", body, "bezel"), ("glass", glass, "screen"), ("vesa", vesa, "steel")],
                      [m.col_hull("display_shape", body)], parent="lift", mass=MONITOR_KG,
                      com=screen_center, gravity=True, friction=0., damping=0.)
    start = (("lift", math.radians(REST_LIFT_DEG)), ("screen_mount", -math.radians(REST_LIFT_DEG)))
    arm = m.mechanism("monitor_arm", BASE, [lift, display, keyed_socket()], LOOKS, start=start,
                      frame_parts=base, frame_colliders=base_colliders, yaw_deg=YAW_DEG,
                      friction=(.88, .70), solver_iters=40)

    def build():
        text = arm.build()
        block = 'def PhysicsRevoluteJoint "screen_mount_joint" (\n        prepend apiSchemas = ["PhysicsDriveAPI:angular"]'
        assert text.count(block) == 1
        text = text.replace(block, block[:-1] + ', "PhysxMimicJointAPI:rotX"]')
        joint_body = "rel physics:body1 = </P_monitor_arm/screen_mount>"
        assert text.count(joint_body) == 1
        return text.replace(joint_body, joint_body + '''
        uniform token physxMimicJoint:rotX:referenceJointAxis = "rotX"
        rel physxMimicJoint:rotX:referenceJoint = </P_monitor_arm/lift_joint>
        float physxMimicJoint:rotX:gearing = 1
        float physxMimicJoint:rotX:offset = 0
''')

    return replace(arm, build=build)


def hex_key():
    handle = m.box((.024, .030, .025), (0., -.006, 0.))
    radius = .006 / math.sqrt(3.)
    bit = m.lathe([(0., .009), (radius, .009), (radius, KEY_TIP_Y - .0015),
                   (radius - .00055, KEY_TIP_Y), (0., KEY_TIP_Y)], segments=6).aimed("+Y")
    key = m.free("hex_key", tuple(KEY_START),
                 [("grip", handle, "grip"), ("bit", bit, "steel")],
                 [close_surface(m.col_hull("handle_shape", handle)), close_surface(m.col_hull("bit_shape", bit))],
                 LOOKS, mass=.071, friction=(.76, .58), torsional_patch=.003,
                 start_quat=tuple(matrix_to_quat_xyzw(KEY_REST_ROTATION)))
    return replace(key, contact_offset=.00020)


def key_holder():
    x, y, z = KEY_START
    floor_z = z - KEY_TIP_Y - .0004
    side = .018
    opening = .005
    wall = side - opening
    height = .055
    center_z = floor_z + height / 2.
    geometry = [("base", m.box((2. * side, 2. * side, floor_z - TABLE),
                               (x, y, .5 * (floor_z + TABLE))))]
    for sign in (-1., 1.):
        suffix = "negative" if sign < 0. else "positive"
        for axis, width in enumerate(((wall, 2. * side), (2. * side, wall))):
            center = np.array([x, y, center_z])
            center[axis] += sign * (opening + wall / 2.)
            mesh = m.box((*width, height), center)
            vertices = mesh.v.copy()
            inner = np.isclose(vertices[:, axis], KEY_START[axis] + sign * opening)
            vertices[inner, axis] += sign * .008 * (vertices[inner, 2] - floor_z) / height
            geometry.append((f"guide_{axis}_{suffix}", m.Mesh(vertices, mesh.f)))
    return m.fixed("key_holder", (0., 0., 0.),
                   [(name, mesh, "base") for name, mesh in geometry],
                   [close_surface(m.col_hull(name + "_shape", mesh)) for name, mesh in geometry],
                   looks=LOOKS)


OBJECTS = [monitor_mount(), key_holder(), hex_key()]


def support_torque(lift_angle, adjuster_angle):
    setting = SCREW_PITCH_M * adjuster_angle / (2. * math.pi)
    force = BASE_FORCE_N + SCREW_EFFECTIVE_STIFFNESS_N_M * setting
    return max(0., force) * spring_leverage(lift_angle)


@m.on_step
def apply_strut(env):
    asset, index = m._joint(env, "monitor_arm", "lift")
    state = m.state(env, "monitor_arm")
    torque = support_torque(state.joint("lift"), state.joint("adjuster"))
    asset.actuators.target_command.set_effort_index(value=m._tensor(asset, torque), joint_ids=[index])


@m.on_step
def observe_release(env):
    global _step
    mount = m.state(env, "monitor_arm")
    key = m.state(env, "hex_key")
    key_rotation = key.rotation()
    key_tip = key.pos + key_rotation[:, 1] * KEY_TIP_Y
    hands_clear = float(np.max(m.fingertips(env)[:, 0])) < .285
    floor_z = KEY_START[2] - KEY_TIP_Y - .0004
    key_in_holder = (np.max(np.abs(key_tip[:2] - KEY_START[:2])) < .005
                     and abs(key_tip[2] - floor_z) < .008
                     and key_rotation[2, 1] < -math.cos(math.radians(16.)))
    fixture_valid = -.12 <= key.pos[0] <= .70 and abs(key.pos[1]) <= .61 and key.pos[2] >= TABLE - .04
    height = REACH * math.sin(mount.joint("lift"))
    _release.update(height, hands_clear, key_in_holder, fixture_valid)
    path = os.environ.get("MONITOR_TRACE")
    if path:
        from yam_arena.runtime import link_poses, tcp_world

        base = env.unwrapped
        socket = np.asarray(BASE) + MOUNT_ROTATION @ np.array([0., SOCKET_CENTER_Y - SOCKET_DEPTH / 2, PIVOT_Z])
        relative = MOUNT_ROTATION.T @ (key_tip - socket)
        wrists = {}
        for side in ("left", "right"):
            asset = base.scene[f"{side}_robot"]
            ids = asset.find_joints(["joint7", "joint8"])[0]
            wrists[side] = {"tcp": np.asarray(tcp_world(asset)).tolist(),
                            "aperture_m": float(np.abs(m._np(asset.data.joint_pos)[0, ids]).sum())}
            if side == "right":
                wrists[side]["gripper_rotation"] = link_poses(asset)["gripper"][:3, :3].tolist()
        with Path(path).open("a") as stream:
            stream.write(json.dumps({"step": _step, "height_m": height, "dwell": _release.steps,
                                     "adjuster_deg": math.degrees(mount.joint("adjuster")),
                                     "key_deg_frame": key_rotation.tolist(),
                                     "key_pos": key.pos.tolist(), "tip_to_socket_m": relative.tolist(),
                                     "hands_clear": bool(hands_clear), "key_in_holder": bool(key_in_holder),
                                     "arms": wrists, "score": _release.value}, allow_nan=False) + "\n")
    _step += 1


def score(objects):
    return _release.value


def success(objects):
    return _release.value == 1.


@m.on_reset
def reset_strut(env):
    global _release, _step
    _release = ReleaseScore()
    _step = 0
    if path := os.environ.get("MONITOR_TRACE"):
        Path(path).write_text("")
    base = env.unwrapped
    key = base.scene["hex_key"]
    pose = key.data.default_root_pose.torch.clone()
    pose[:, :3] += base.scene.env_origins
    key.write_root_pose_to_sim_index(root_pose=pose)
    key.write_root_velocity_to_sim_index(root_velocity=key.data.default_root_vel.torch.clone())
    arm, index = m._joint(env, "monitor_arm", "lift")
    position = arm.data.default_joint_pos.torch.clone()
    velocity = arm.data.default_joint_vel.torch.clone()
    arm.write_joint_state_to_sim_index(position=position, velocity=velocity)
    torque = support_torque(math.radians(REST_LIFT_DEG), 0.)
    arm.actuators.target_command.set_effort_index(value=m._tensor(arm, torque), joint_ids=[index])
    base.sim.forward()
    # Let temporal lighting and antialiasing resolve at the freshly reset pose.
    for _ in range(12):
        base.sim.render()
    for camera in ("top_cam", "left_cam", "right_cam"):
        base.scene[camera].update(0., force_recompute=True)


def demo():
    from demo import motion

    return motion()


if __name__ == "__main__":
    for part in OBJECTS:
        path = Path(__file__).with_name(f"{part.name}.usda")
        path.write_text(part.build())
