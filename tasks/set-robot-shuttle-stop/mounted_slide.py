"""Physically operated compact shuttle stop and independent clamp witness."""
import json
import math
import os
from pathlib import Path

import numpy as np

import clamp_load as load
from demo_slide import route as demo
import proof_fixture as fixture
from scorer import Frame, StrokeScorer, TARGET_M
import slide_stop_geometry as g
import slide_tools as tools
from yam_arena.constants import GRIPPER
from yam_arena.station import TABLE_TOP


SCENE = "station"
INSTRUCTION = "Set the shuttle stop at the green mark. Tighten the clamp to the green torque band, withdraw the tools, and prove the position twice with the valve."
VALVE_ORIGIN = np.array((*fixture.VALVE_ORIGIN_XY, TABLE_TOP))
DIAL_ORIGIN = np.array((.307, .078, tools.ORIGIN[2] - .004))
OBJECTS = [
    tools.base(), g.axis(tools.ORIGIN, tools.STOP_FACE_MM, tools.YAW_DEG),
    tools.adjust_key(), tools.adjust_rest(), tools.wrench(), tools.wrench_rest(), tools.parking_stations(),
    fixture.valve(VALVE_ORIGIN), fixture.plumbing(tools.ORIGIN, VALVE_ORIGIN, TABLE_TOP),
    fixture.indicator(DIAL_ORIGIN),
]
GRIPPER["effort"] = 40.
scorer = StrokeScorer()
value = 0.
pressure_mode = 0
pressure_target = 0.
clamp_readings = 0
last_clamp = 0.
last_friction = (-1., -1.)


def change_pressure(env, mode, current, seed=False):
    global pressure_mode, pressure_target
    asset, joint = g.m._joint(env, g.NAME, "carriage")
    if mode != pressure_mode or seed:
        stiffness, damping = (6000., 32.) if mode else (0., 1.)
        asset.write_joint_stiffness_to_sim_index(stiffness=g.m._tensor(asset, stiffness), joint_ids=[joint])
        asset.write_joint_damping_to_sim_index(damping=g.m._tensor(asset, damping), joint_ids=[joint])
        pressure_target = current
    pressure_mode = mode
    if mode:
        pressure_target = float(np.clip(pressure_target + mode * g.CHECK_SPEED_M_S * .05, -.005, .042))
        g.m.set_target(env, g.NAME, "carriage", pressure_target)


def apply_clamp_load(env, clamp_angle, reset=False):
    global last_friction
    running = load.clamp_torque(clamp_angle) if clamp_angle > 0. else load.LOOSE_RUNNING_NM
    resistance = .007 + load.sleeve_breakaway_torque(clamp_angle)
    pair = (running, resistance)
    if pair == last_friction and not reset:
        return
    asset = env.unwrapped.scene[g.NAME]
    for joint, effort in zip(("clamp", "adjuster"), pair, strict=True):
        index = asset.find_joints(f"{joint}_joint")[0][0]
        asset.write_joint_friction_coefficient_to_sim_index(
            joint_friction_coeff=g.m._tensor(asset, effort),
            joint_dynamic_friction_coeff=g.m._tensor(asset, .95 * effort), joint_ids=[index])
    last_friction = pair


@g.m.on_reset
def reset(env):
    global value, clamp_readings, last_clamp
    value, clamp_readings, last_clamp = 0., 0, load.LOOSE_ANGLE_RAD
    scorer.reset()
    base = env.unwrapped
    for name in (tools.ADJUST_TOOL_NAME, tools.CLAMP_TOOL_NAME):
        asset = base.scene[name]
        pose = asset.data.default_root_pose.torch.clone()
        pose[:, :3] += base.scene.env_origins
        asset.write_root_pose_to_sim_index(root_pose=pose)
        asset.write_root_velocity_to_sim_index(root_velocity=asset.data.default_root_vel.torch.clone().zero_())
    for name in (g.NAME, fixture.VALVE_NAME, fixture.INDICATOR_NAME, tools.CLAMP_TOOL_NAME):
        asset = base.scene[name]
        pos = asset.data.default_joint_pos.torch.clone()
        vel = asset.data.default_joint_vel.torch.clone().zero_()
        asset.write_joint_state_to_sim_index(position=pos, velocity=vel)
        asset.set_joint_position_target(pos)
    change_pressure(env, 0, 0., seed=True)
    apply_clamp_load(env, load.LOOSE_ANGLE_RAD, reset=True)
    base.sim.forward()
    cameras = [base.scene[name] for name in ("top_cam", "left_cam", "right_cam")]
    for camera in cameras:
        camera.reset()
    for _ in range(24):
        base.sim.render()
    for camera in cameras:
        camera.update(0., force_recompute=True)


@g.m.on_step
def observe(env):
    global value, clamp_readings, last_clamp
    axis = g.m.state(env, g.NAME)
    adjusting = g.m.state(env, tools.ADJUST_TOOL_NAME)
    wrench = g.m.state(env, tools.CLAMP_TOOL_NAME)
    lever = g.m.state(env, fixture.VALVE_NAME).deg("direction")
    stroke, sleeve, buffer = (axis.joint(name) for name in ("carriage", "stop_sleeve", "buffer"))
    clamp = -axis.joint("clamp")
    advance, retract = lever > 18., lever < -18.
    change_pressure(env, int(advance) - int(retract), stroke)
    apply_clamp_load(env, clamp)
    # This drives a visible dial using the same yoke position a contact dial reads.
    error = (stroke - TARGET_M) / fixture.READING_RANGE_M
    pointer_target = math.pi * .5 + float(np.clip(math.tau * error, -1.48, 1.48))
    g.m.set_target(env, fixture.INDICATOR_NAME, "dial", pointer_target)
    joint_error = tools.STOP_FACE_MM * .001 + sleeve - (.054 + stroke)
    metal_contact = abs(joint_error) <= .00018 and buffer >= .00238
    adjust_pad = axis.to_local(np.asarray(adjusting.pos)[None, :])[0] - np.asarray(g.AXIS)
    torque_pad = axis.to_local(np.asarray(wrench.pos)[None, :])[0] - tools.CLAMP_CENTER
    adjusted_by_key = -.0037 <= adjust_pad[0] <= -.0003 and np.linalg.norm(adjust_pad[1:]) <= .00046
    within_socket = -.0032 <= torque_pad[2] <= -.0004 and np.linalg.norm(torque_pad[:2]) <= .00042
    torque = -load.indicated_torque(wrench.joint("beam"))
    web_yaw = tools.axis_angle_matrix(tools.CLAMP_SHAFT_VECTOR, math.radians(tools.BEAM_OFFSET_DEG))
    web_world = wrench.to_world((web_yaw @ tools.WEB_OFFSET)[None, :])[0]
    wrench_unbraced = np.linalg.norm((web_world - tools.wrench_rest_at())[:2]) >= .014
    stable_read = (within_socket and wrench_unbraced and load.TORQUE_BAND_NM[0] <= torque <= load.TORQUE_BAND_NM[1]
                   and abs(clamp - last_clamp) <= math.radians(.22))
    clamp_readings = clamp_readings + 1 if stable_read else 0
    last_clamp = clamp
    tips = g.m.fingertips(env)
    yoke = tools.point((.037 + stroke, .001, .025))
    tools_out = not adjusted_by_key and not within_socket
    hands_clear = tools_out and np.linalg.norm(tips - yoke, axis=1).min() >= .044
    safe = (math.isfinite(stroke) and (not (advance or retract) or tools_out and hands_clear)
            and abs(torque) <= .25 and all(float(body.pos[2]) >= TABLE_TOP - .003 for body in (adjusting, wrench)))
    value = scorer.update(Frame(stroke, bool(adjusted_by_key), bool(metal_contact), bool(clamp_readings >= 10),
                               clamp, bool(hands_clear), bool(advance), bool(safe)))
    if destination := os.getenv("SHUTTLE_TRACE"):
        beam_angle = wrench.joint("beam")
        wrench_tip = tools.axis_angle_matrix(tools.CLAMP_SHAFT_VECTOR, math.radians(tools.BEAM_OFFSET_DEG) + beam_angle)
        handle_offset = wrench_tip @ np.array((tools.WRENCH_HANDLE_RADIUS, 0., tools.WRENCH_Z + .012))
        handle_world = wrench.to_world(handle_offset[None, :])[0]
        right_fingers = tips[12:]
        valve_grip = fixture.lever_tip(VALVE_ORIGIN, lever)
        with Path(destination).open("a", encoding="utf-8") as out:
            out.write(json.dumps(dict(stroke_m=stroke, clamp_torque_nm=torque, metal_contact=bool(metal_contact),
                                      adjustment_in_socket=bool(adjusted_by_key), clamp_in_socket=bool(within_socket),
                                      hands_clear=bool(hands_clear), valve_deg=lever,
                                      clamp_readings=clamp_readings, wrench_unbraced=bool(wrench_unbraced),
                                      safe=bool(safe), score=value, clamp_deg=math.degrees(clamp),
                                      valve_grip_m=valve_grip.tolist(), left_finger_mean_m=tips[:12].mean(axis=0).tolist(),
                                      wrench_pos=np.asarray(wrench.pos).tolist(), wrench_quat=wrench.quat_xyzw.tolist(),
                                      wrench_handle=handle_world.tolist(),
                                      right_finger_gaps_mm=(1000. * np.linalg.norm(right_fingers - handle_world, axis=1)).tolist()),
                                 allow_nan=False) + "\n")
    return value


def score(_objects):
    return value
