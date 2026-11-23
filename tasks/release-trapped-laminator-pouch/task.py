"""Release a trapped pouch from an unplugged desktop laminator."""
import json
import math
import os
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import geometry as g
import mechkit as m
from scorer import Reading, Scorer
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, GRIPPER, READY


SCENE = 'station'
INSTRUCTION = 'Hold the laminator’s rear release lever down, grip the folded edge and draw the trapped pouch straight out through the entry slot. Lay it in the marked area. Let the lever return.'
GRIPPER['effort'] = 40.
OBJECTS = [*g.OBJECTS]
scorer = Scorer()
PRESS_HEADING_DEG = 0.
PRESS_PITCH_DEG = 6.
POUCH_SIZE = np.array((g.POUCH_LENGTH, g.POUCH_WIDTH, g.COLLISION_THICKNESS))
POUCH_POINTS = m.box(POUCH_SIZE).v
ENVELOPE = np.concatenate((POUCH_POINTS, g.FOLD.v))


@m.on_reset
def reset(env):
    scorer.reset()
    base = env.unwrapped
    paper = base.scene['pouch']
    pose = paper.data.default_root_pose.torch.clone()
    pose[:, :3] += base.scene.env_origins
    paper.write_root_pose_to_sim_index(root_pose=pose)
    paper.write_root_velocity_to_sim_index(root_velocity=paper.data.default_root_vel.torch.clone().zero_())
    lever = base.scene['laminator']
    start = lever.data.default_joint_pos.torch.clone()
    lever.write_joint_state_to_sim_index(position=start, velocity=lever.data.default_joint_vel.torch.clone().zero_())
    lever.set_joint_position_target(start)
    base.sim.forward()
    cameras = [base.scene[name] for name in ('top_cam', 'left_cam', 'right_cam')]
    for camera in cameras:
        camera.reset()
    for _ in range(24):
        base.sim.render()
    for camera in cameras:
        camera.update(0., force_recompute=True)


@m.on_step
def observe(env):
    device = m.state(env, 'laminator')
    paper = m.state(env, 'pouch')
    lever_deg = device.deg('release')
    world = paper.to_world(POUCH_POINTS)
    local = (world - g.ORIGIN) @ g.ROTATION
    average = local.mean(axis=0)
    draw = float(g.POUCH_START[0] - average[0])
    along_pouch = paper.rotation()[:, 0] @ g.ROTATION
    along_entry = local[:, 0].min() <= g.ENTRY and local[:, 0].max() >= g.ENTRY - .015
    straight = False
    if along_entry and along_pouch[0] >= math.cos(math.radians(16.)):
        mark = average + ((g.ENTRY - average[0]) / along_pouch[0]) * along_pouch
        straight = bool(abs(mark[1]) <= .024 and .027 <= mark[2] <= .040)
    cleared = bool(local[:, 0].max() <= g.ENTRY - .010)
    fingertips = m.fingertips(env)
    clear_lever = np.linalg.norm(fingertips - g.world(g.PRESS_PAD), axis=1).min() >= .040
    outside_paper = np.maximum(np.abs(paper.to_local(fingertips)) - POUCH_SIZE / 2., 0.)
    fold_center = paper.to_world(g.FOLD_GRIP[None, :])[0]
    clear_paper = (np.linalg.norm(outside_paper, axis=1).min() >= .020
                   and np.linalg.norm(fingertips - fold_center, axis=1).min() >= .025)
    envelope = paper.to_world(ENVELOPE)
    rest = world - g.LAYDOWN
    laid = (bool(np.all(np.abs(envelope[:, :2] - g.LAYDOWN[:2]) <= (.081, .114)))
            and -.002 <= rest[:, 2].min() <= .004 and rest[:, 2].max() <= .005
            and np.linalg.norm(paper.lin_vel) <= .015)
    valid = (np.isfinite(world).all() and device.tilt_deg() < 4. and np.all((envelope[:, 0] >= .135) & (envelope[:, 0] <= .735))
             and np.all(np.abs(envelope[:, 1]) <= .515)
             and envelope[:, 2].min() >= g.TABLE_TOP - .010)
    reading = Reading(lever_deg >= 7., draw, bool(straight), cleared, bool(laid), abs(lever_deg) <= 2.,
                      bool(clear_lever and clear_paper), bool(valid))
    current = scorer.update(reading)
    trace = os.getenv('LAMINATOR_TRACE')
    if trace:
        with open(trace, 'a', encoding='utf-8') as stream:
            right_tips_local = (fingertips[12:] - g.ORIGIN) @ g.ROTATION
            sample = dict(vars(reading), score=current, lever_deg=lever_deg, pouch_center=average.tolist(),
                          pouch_rotation=paper.rotation().tolist(), right_tips_local=right_tips_local.tolist(),
                          fold_center=fold_center.tolist())
            stream.write(json.dumps(sample, allow_nan=False) + '\n')


def score(_objects):
    return scorer.score


class WristPose(float):
    def __new__(cls, rotation):
        result = float.__new__(cls, 0.)
        result.rotation = rotation
        return result


def demo():
    from yam_arena import demo as motion, ik, station

    original = motion.grip_rotation
    if not getattr(original, 'laminator_pose', False):
        def grip_rotation(yaw=0.):
            return yaw.rotation if isinstance(yaw, WristPose) else original(yaw)
        grip_rotation.laminator_pose = True
        motion.grip_rotation = grip_rotation

    left_home, left_rotation = ik.tcp_pose(READY, station.BASES['left'])
    right_home, right_rotation = ik.tcp_pose(READY, station.BASES['right'])
    yaw = math.radians(PRESS_HEADING_DEG)
    c, s = math.cos(yaw), math.sin(yaw)
    tilt = math.radians(PRESS_PITCH_DEG)
    u, v = math.cos(tilt), math.sin(tilt)
    lean = np.array(((1.,0.,0.),(0.,u,-v),(0.,v,u)))
    down = WristPose(lean @ np.array(((c,s,0.),(s,-c,0.),(0.,0.,-1.))))
    grasp = WristPose(np.diag((1., -1., -1.)))
    paddle = g.world(g.PRESS_PAD + (0., 0., .007))
    press_center = paddle + (0., .010, 0.)
    high = press_center + (0., 0., .074)
    pressed = press_center + (0., 0., -.021)
    bite = g.world(g.POUCH_START + g.FOLD_GRIP)
    reach = bite + (0., 0., .064)
    draw = bite + (0., -.096, -.002)
    back = -g.FOLD_GRIP[0]
    raised = g.LAYDOWN + (0., -back, .096)
    rest = g.LAYDOWN + (0., -back, .017)
    w = motion.Waypoint
    left = [
        w(1.2, high, FINGER_CLOSED, 'approach the rear release paddle', down),
        w(.9, pressed, FINGER_CLOSED, 'press the release paddle down', down),
        w(5.10, pressed, FINGER_CLOSED, 'keep the pressure rollers released', down),
        w(.75, high, FINGER_CLOSED, 'let the rear release return', down),
        w(1.1, left_home, FINGER_OPEN, 'clear the laminator', WristPose(left_rotation)),
    ]
    right = [
        w(2.2, right_home, FINGER_OPEN, 'wait for the rollers to release', WristPose(right_rotation)),
        w(1.15, reach, -.028, 'approach the folded pouch edge', grasp),
        w(.65, bite, -.028, 'straddle the folded edge', grasp),
        w(.55, bite, FINGER_CLOSED, 'grip the folded edge', grasp),
        w(1.4, draw, FINGER_CLOSED, 'draw the pouch clear of the entry slot', grasp),
        w(.85, raised, FINGER_CLOSED, 'move the released pouch to the rest area', grasp),
        w(.7, rest, FINGER_CLOSED, 'lower the pouch flat over the bench', grasp),
        w(.55, rest, FINGER_OPEN, 'release the pouch', grasp),
        w(.8, raised, FINGER_OPEN, 'leave the pouch clear', grasp),
        w(.9, right_home, FINGER_OPEN, 'return the pouch hand to rest', WristPose(right_rotation)),
    ]
    return {'left': left, 'right': right}
