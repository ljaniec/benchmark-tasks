"""Set a mobile robot base on its real load-bearing pads using both pedals."""
from dataclasses import astuple
import json
import math
import os
from pathlib import Path

import numpy as np

from scorer import Frame, SupportTransferScorer
import support_geometry as g
from yam_arena import demo as motion
from yam_arena.constants import FINGER_CLOSED, GRIPPER
from yam_arena.station import TABLE_TOP
from yam_arena.urdf import axis_angle_matrix


SCENE = "station"
INSTRUCTION = "Press the two leveling pedals so the front wheels clear the worktop. Leave the robot base parked on its feet."
ORIGIN = np.array((.480, 0., TABLE_TOP + g.TOP_PLATE_M))
OBJECTS = [g.fixture(ORIGIN)]
GRIPPER["effort"] = 40.
scorer = SupportTransferScorer()
value = 0.


@g.m.on_reset
def reset(env):
    global value
    value = 0.
    scorer.reset()
    base = env.unwrapped
    asset = base.scene[g.NAME]
    initial = asset.data.default_root_pose.torch.clone()
    initial[:, :3] += base.scene.env_origins
    asset.write_root_pose_to_sim_index(root_pose=initial)
    asset.write_root_velocity_to_sim_index(root_velocity=asset.data.default_root_vel.torch.clone().zero_())
    pos = asset.data.default_joint_pos.torch.clone()
    vel = asset.data.default_joint_vel.torch.clone().zero_()
    asset.write_joint_state_to_sim_index(position=pos, velocity=vel)
    asset.set_joint_position_target(pos)
    base.sim.forward()
    cameras = [base.scene[view] for view in ("top_cam", "left_cam", "right_cam")]
    for camera in cameras:
        camera.reset()
    for _ in range(24):
        base.sim.render()
    for camera in cameras:
        camera.update(0., force_recompute=True)


@g.m.on_step
def observe(env):
    global value
    state = g.m.state(env, g.NAME)
    position = np.asarray(state.pos)
    rel_table = position - (0., 0., TABLE_TOP)
    pad, wheel = g.gaps(rel_table, state.rotation(), state.joint("leveler_left"), state.joint("leveler_right"))
    tips = state.to_local(g.m.fingertips(env))
    near_base = (np.abs(tips[:, 0]) <= .224) & (np.abs(tips[:, 1]) <= .213)
    near_support = near_base & (tips[:, 2] < -.039)
    pedal_clear = all(np.linalg.norm(tips - g.pedal_tip(sign, state.deg(f"pedal_{side}")), axis=1).min() >= .028
                      for side, sign in (("left", 1.), ("right", -1.)))
    hands_clear = bool(np.all(~near_base | (tips[:, 2] > .035))) and pedal_clear
    blocked = bool(near_support.any() or rel_table[2] > g.TOP_PLATE_M + .012)
    drift = float(np.linalg.norm(position[:2] - ORIGIN[:2]))
    frame = Frame(tuple(pad), tuple(wheel), hands_clear, float(np.linalg.norm(state.lin_vel)) < .020,
                  drift, state.tilt_deg(), blocked)
    value = scorer.update(frame)
    if destination := os.getenv("STAND_TRACE"):
        row = {name: item.item() if isinstance(item, np.generic) else item
               for name, item in zip(Frame.__annotations__, astuple(frame), strict=True)}
        row.update(score=value, pedal_deg=[state.deg(f"pedal_{name}") for name in ("left", "right")],
                   nearest_fingers_mm=[1000. * float(np.linalg.norm(
                       tips - g.pedal_tip(sign, state.deg(f"pedal_{name}")), axis=1).min())
                       for name, sign in (("left", 1.), ("right", -1.))])
        with Path(destination).open("a", encoding="utf-8") as out:
            out.write(json.dumps(row, allow_nan=False) + "\n")
    return value


def score(_objects):
    return value


class Wrist(float):
    def __new__(cls, matrix):
        value = float.__new__(cls, 0.)
        value.matrix = matrix
        return value


def demo():
    original = motion.grip_rotation
    if not getattr(original, "pedal_wrist", False):
        def grip(angle=0.):
            return angle.matrix if isinstance(angle, Wrist) else original(angle)
        grip.pedal_wrist = True
        motion.grip_rotation = grip
    y_axis, z_axis = np.array((0., 1., 0.)), np.array((0., 0., 1.))
    front = np.array(((0., 0., 1.), (-1., 0., 0.), (0., -1., 0.)))
    inspection = {
        'left': ((0.3068675, 0.2455718, 0.7820763), 67.0, 6.0),
        'right': ((0.3086265, -0.24517, 0.7819651), 67.0, 6.0),
    }
    p = motion.Waypoint
    arms = {}
    for name, sign in (("left", 1.), ("right", -1.)):
        home = ORIGIN + g.pedal_tip(sign, 0.)
        end = ORIGIN + g.pedal_tip(sign, -g.PEDAL_MAX_DEG)
        path = [
            p(1.18, home + (-.003, 0., .069), -.036, "approach the full-width treadle"),
            p(.59, home + (0., 0., .001), -.036, "straddle the two flat sides"),
            p(.44, home + (0., 0., .001), FINGER_CLOSED, "close on the treadle"),
        ]
        for degrees in range(-8, -105, -8):
            degrees = max(-g.PEDAL_MAX_DEG, degrees)
            at = ORIGIN + g.pedal_tip(sign, degrees)
            lift = .007 * max(0., math.sin(math.radians(-degrees)) - .15) / .85
            path.append(p(.18, at + (0., 0., lift), FINGER_CLOSED, "pull the cam lever through its full throw"))
        path.extend((
            p(.30, end + (0., 0., .007), FINGER_CLOSED, "settle the leveling lever"),
            p(.31, end + (0., 0., .007), -.036, "release the deployed lever"),
            p(.56, end + (-.023, 0., .082), -.030, "withdraw without supporting the base"),
        ))
        center, yaw, pitch = inspection[name]
        witness = Wrist(axis_angle_matrix(z_axis, math.radians(-sign * yaw))
                        @ axis_angle_matrix(y_axis, math.radians(pitch)) @ front)
        eye = np.asarray(center)
        path.extend((
            p(.65, eye + (-.034, sign * .035, .060), -.022, "move outside the stand for inspection", witness),
            p(.67, eye, -.022, "show the wheel and pad after release", witness),
        ))
        arms[name] = path
    return arms
