"""Set a sagging desk fan level and tighten its side clamp."""
import json
import math
import os
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import clamp
import geometry as g
import mechkit as m
from fan_route import demo
from scorer import Reading, Scorer
from yam_arena.constants import GRIPPER

SCENE = "station"
INSTRUCTION = "Support the loose fan head, aim it straight ahead, and tighten its tilt knob. Set the test saddle on top of the grille to check the clamp, then take both hands away."
OBJECTS = [g.fan(), g.proof_weight()]
GRIPPER["effort"] = 40.
scorer = Scorer()
last_friction = None


def load_clamp(env, knob_angle):
    global last_friction
    limits = (clamp.breakaway_moment(knob_angle), clamp.running_moment(knob_angle))
    if limits == last_friction:
        return
    asset = env.unwrapped.scene["desk_fan"]
    for joint, moment in zip(("head", "clamp"), limits, strict=True):
        axis = asset.find_joints(f"{joint}_joint")[0][0]
        asset.write_joint_friction_coefficient_to_sim_index(
            joint_friction_coeff=m._tensor(asset, moment),
            joint_dynamic_friction_coeff=m._tensor(asset, .95 * moment), joint_ids=[axis])
    last_friction = limits


@m.on_before_reset
def clear_old_load(env):
    load_clamp(env, math.radians(clamp.LOOSE_DEG))


@m.on_reset
def reset(env):
    scorer.reset()
    base = env.unwrapped
    fan = base.scene["desk_fan"]
    pose = fan.data.default_root_pose.torch.clone()
    pose[:, :3] += base.scene.env_origins
    fan.write_root_pose_to_sim_index(root_pose=pose)
    fan.write_root_velocity_to_sim_index(root_velocity=fan.data.default_root_vel.torch.clone().zero_())
    weight = base.scene["proof_weight"]
    pose = weight.data.default_root_pose.torch.clone()
    pose[:, :3] += base.scene.env_origins
    weight.write_root_pose_to_sim_index(root_pose=pose)
    weight.write_root_velocity_to_sim_index(root_velocity=weight.data.default_root_vel.torch.clone().zero_())
    start = fan.data.default_joint_pos.torch.clone()
    fan.write_joint_state_to_sim_index(position=start, velocity=fan.data.default_joint_vel.torch.clone().zero_())
    fan.set_joint_position_target(start)
    load_clamp(env, math.radians(clamp.LOOSE_DEG))
    base.sim.forward()
    cameras = [base.scene[name] for name in ("top_cam", "left_cam", "right_cam")]
    for camera in cameras:
        camera.reset()
    for _ in range(24):
        base.sim.render()
    for camera in cameras:
        camera.update(0., force_recompute=True)


@m.on_step
def observe(env):
    body = m.state(env, "desk_fan")
    head = body.deg("head")
    angle = body.joint("clamp")
    load_clamp(env, angle)
    rotation = body.rotation()
    tilt_forward = body.to_world(g.rotate_head(g.PIVOT + (-1., 0., 0.), head))[0] - body.to_world(g.PIVOT)[0]
    pitch = math.degrees(math.asin(float(np.clip(tilt_forward[2], -1., 1.))))
    tips = m.fingertips(env)
    fingers = body.to_local(tips)
    guard = g.rotate_head(g.GUARD_CENTER, head)
    clearance = np.linalg.norm(fingers - guard, axis=1).min()
    knob_gap = np.linalg.norm(fingers - g.KNOB_GRIP, axis=1).min()
    # Board-relative finger clearance includes the physical convex guard.
    clear = clearance >= g.GUARD_RADIUS + .028 and knob_gap >= .046
    joints = env.unwrapped.scene["desk_fan"]
    head_index = joints.find_joints("head_joint")[0][0]
    speed = float(m._np(joints.data.joint_vel)[0, head_index])
    steady = abs(speed) <= .065 and np.linalg.norm(body.lin_vel) <= .018
    weight = m.state(env, "proof_weight")
    saddle = body.to_local(weight.pos[None, :])[0]
    offset = saddle - g.rotate_head(g.PROOF_SITE, head)
    proof = (abs(offset[0]) <= .006 and abs(offset[1]) <= .017 and -.004 <= offset[2] <= .007
             and weight.tilt_deg() <= 9. and np.linalg.norm(weight.lin_vel) <= .015)
    clear = clear and np.linalg.norm(tips - weight.pos, axis=1).min() >= .047
    in_area = (.12 <= weight.pos[0] <= .65 and abs(weight.pos[1]) <= .50 and
               weight.pos[2] >= g.TABLE_TOP - .005)
    valid = (in_area and np.linalg.norm(body.pos[:2] - g.ORIGIN[:2]) <= .026
             and abs(body.pos[2] - g.ORIGIN[2]) <= .012 and body.tilt_deg() <= 5.)
    reading = Reading(pitch, bool(steady), bool(proof), bool(clear), bool(valid))
    current = scorer.update(reading)
    trace = os.getenv("FAN_TRACE")
    if trace:
        with Path(trace).open("a") as stream:
            row = dict(vars(reading), score=current, knob_deg=math.degrees(angle), relative_pitch_deg=head,
                       body=body.pos.tolist(), fingers=fingers.tolist(), base_rotation=rotation.tolist(), weight=weight.pos.tolist(),
                       saddle_offset=offset.tolist(), weight_tilt_deg=weight.tilt_deg())
            stream.write(json.dumps(row, allow_nan=False) + "\n")


def score(_objects):
    return scorer.score
