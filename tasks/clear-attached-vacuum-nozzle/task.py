"""Clear an attached canister vacuum's unpowered floor head."""
import json
import math
import os
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as m
import scene_fixture as g
from scorer import Reading, Scorer
from yam_arena.constants import FINGER_OPEN, GRIPPER, READY
from yam_arena.station import TABLE_TOP


SCENE = 'station'
INSTRUCTION = 'Hold the raised floor nozzle steady. Pull out the wedged paperboard, put it in the tray, and lower the connected nozzle.'
GRIPPER['effort'] = 40.
TRAY = np.array((.250,-.218,TABLE_TOP))
TRAY_HALF = np.array((.094,.079))
BUMPER = np.array((-.093,.117,.025))
scorer = Scorer()


def tray():
    walls = [
        ('floor',(.202,.172,.009),(0.,0.,.0045),'body'),
        ('inner_left',(.200,.006,.023),(0.,.082,.017),'body'),
        ('inner_right',(.200,.006,.023),(0.,-.082,.017),'body'),
        ('front',(.006,.164,.023),(-.097,0.,.017),'body'),
        ('back',(.006,.164,.023),(.097,0.,.017),'body'),
    ]
    visuals, collisions = g.solids(walls)
    return m.fixed('collection_tray',TRAY,visuals,collisions,g.LOOKS)


OBJECTS = [*g.OBJECTS, tray()]


@m.on_reset
def reset(env):
    scorer.reset()
    base = env.unwrapped
    part = base.scene['folded_paperboard']
    pose = part.data.default_root_pose.torch.clone()
    pose[:,:3] += base.scene.env_origins
    part.write_root_pose_to_sim_index(root_pose=pose)
    part.write_root_velocity_to_sim_index(root_velocity=part.data.default_root_vel.torch.clone().zero_())
    tool = base.scene['connected_floor_tool']
    initial = tool.data.default_joint_pos.torch.clone()
    tool.write_joint_state_to_sim_index(position=initial,velocity=tool.data.default_joint_vel.torch.clone().zero_())
    tool.set_joint_position_target(initial)
    base.sim.forward()
    cameras=[base.scene[name] for name in ('top_cam','left_cam','right_cam')]
    for camera in cameras:
        camera.reset()
    for _ in range(24):
        base.sim.render()
    for camera in cameras:
        camera.update(0.,force_recompute=True)


@m.on_step
def observe(env):
    tool = m.state(env,'connected_floor_tool')
    card = m.state(env,'folded_paperboard')
    angle = tool.deg('head')
    axis = g.pitch(angle)
    point = (card.pos-g.PIVOT) @ axis
    draw = float(g.CARD_CENTER[2]-point[2])
    passed = bool(angle>=50. and .010<=draw<=.066 and abs(point[0]-g.MOUTH_X)<=.026 and abs(point[1])<=.028)
    vertices = card.to_world(g.card_mesh().v)
    relative = vertices-TRAY
    in_tray = bool(np.all(np.abs(relative[:,:2]) <= TRAY_HALF) and relative[:,2].min()>=.006
                   and relative[:,2].max()<=.039 and np.linalg.norm(card.lin_vel)<=.020)
    fingers = m.fingertips(env)
    clear_card = float(np.linalg.norm(fingers - card.pos,axis=1).min()) >= .060
    head_ends = g.PIVOT + np.array(((-.067,-.118,-.025),(-.067,0.,-.025),(-.067,.118,-.025))) @ axis.T
    clear_head = float(np.linalg.norm(fingers[:,None,:]-head_ends,axis=2).min()) >= .047
    valid = bool(np.isfinite(vertices).all() and .070<=card.pos[0]<=.72 and abs(card.pos[1])<=.50
                 and card.pos[2]>=TABLE_TOP-.009 and tool.tilt_deg()<4.)
    result = Reading(passed,in_tray,bool(abs(angle)<=5.),bool(clear_card and clear_head),valid)
    current = scorer.update(result)
    trace = os.getenv('NOZZLE_TRACE')
    if trace:
        row = dict(vars(result),score=current,head_deg=angle,axial_draw_m=draw,card_m=card.pos.tolist())
        with open(trace,'a',encoding='utf-8') as stream:
            stream.write(json.dumps(row,allow_nan=False)+'\n')


def score(_objects):
    return scorer.score


class WristPose(float):
    def __new__(cls, rotation):
        pose = float.__new__(cls,0.)
        pose.rotation = rotation
        return pose


def grip(elevation):
    radians = math.radians(elevation)
    jaw = np.array((0.,-1.,0.))
    approach = np.array((math.cos(radians),0.,math.sin(radians)))
    return WristPose(np.column_stack((jaw,np.cross(approach,jaw),approach)))


def demo():
    from yam_arena import demo as motion, ik, station

    original = motion.grip_rotation
    if not getattr(original,'nozzle_pose',False):
        def grip_rotation(yaw=0.):
            return yaw.rotation if isinstance(yaw,WristPose) else original(yaw)
        grip_rotation.nozzle_pose=True
        motion.grip_rotation = grip_rotation

    axis = g.pitch(g.START_PITCH_DEG)
    left_pose = grip(-82.)
    right_pose = grip(-72.)
    left_axis = left_pose.rotation[:,2]
    right_axis = right_pose.rotation[:,2]
    bumper = lambda angle: g.PIVOT+g.pitch(angle)@BUMPER-.010*left_axis
    held = bumper(g.START_PITCH_DEG)
    card = g.PIVOT+axis@(g.CARD_CENTER-np.array((0.,0.,.019)))-.010*right_axis
    withdraw = card-.059*axis[:,2]
    right_home, right_rotation = ik.tcp_pose(READY, station.BASES['right'])
    left_home, left_rotation = ik.tcp_pose(READY, station.BASES['left'])
    waypoint = motion.Waypoint
    left = [
        waypoint(1.25,held-.057*left_axis,-.031,'approach the raised nozzle bumper',left_pose),
        waypoint(.80,held,-.031,'straddle the original bumper',left_pose),
        waypoint(.65,held,-.019,'hold the nozzle',left_pose),
        waypoint(7.15,held,-.019,'stabilize the raised soleplate',left_pose),
        waypoint(.70,bumper(43.),-.019,'lower the nozzle halfway',left_pose),
        waypoint(.85,bumper(2.),-.019,'lay the soleplate down',left_pose),
        waypoint(.65,bumper(2.),-.034,'release the nozzle',left_pose),
        waypoint(.70,bumper(2.)-.105*left_axis,-.031,'lift clear of the nozzle',left_pose),
        waypoint(1.1,left_home,FINGER_OPEN,'return the nozzle hand to rest',WristPose(left_rotation)),
    ]
    right = [
        waypoint(2.75,right_home,FINGER_OPEN,'wait until the nozzle is held',WristPose(right_rotation)),
        waypoint(1.35,card-.046*right_axis,-.036,'approach the paperboard edge',right_pose),
        waypoint(.80,card,-.036,'straddle the protruding fold',right_pose),
        waypoint(.75,card,-.024,'grip the paperboard',right_pose),
        waypoint(1.35,withdraw,-.024,'draw the fold out of the nozzle',right_pose),
        waypoint(1.25,TRAY+(0.,0.,.120),-.024,'move over the collection tray',right_pose),
        waypoint(.60,TRAY+(0.,0.,.073),-.024,'lower the fragment to the tray',right_pose),
        waypoint(.55,TRAY+(0.,0.,.073),-.039,'let the fragment fall into the tray',right_pose),
        waypoint(.8,right_home,FINGER_OPEN,'leave the card and nozzle clear',WristPose(right_rotation)),
    ]
    return {'left':left,'right':right}
