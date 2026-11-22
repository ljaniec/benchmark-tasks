"""Seat and secure a guarded cutter assembly, then exercise its crank."""
import json
import math
import os
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parent))
import cartridge_geometry as g
import mechkit as m
from scorer import Reading, Scorer
from yam_arena.constants import FINGER_OPEN, GRIPPER, READY
from yam_arena.urdf import axis_angle_matrix


SCENE='station'
INSTRUCTION='Seat the replacement cutter, turn its retaining ring clockwise, and turn the crank to check that the cutter stays in the sharpener.'
GRIPPER['effort']=40.
OBJECTS=[*g.OBJECTS]
scorer=Scorer()
X_AXIS=np.array((1.,0.,0.))
CRANK_KNOB=np.array((-.043,0.,-.051))


def roll(degrees):
    return axis_angle_matrix(X_AXIS,math.radians(degrees))


@m.on_reset
def reset(env):
    scorer.reset()
    base=env.unwrapped
    art=base.scene['mounted_sharpener']
    start=art.data.default_joint_pos.torch.clone()
    art.write_joint_state_to_sim_index(position=start,velocity=art.data.default_joint_vel.torch.clone().zero_())
    art.set_joint_position_target(start)
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
    device=m.state(env,'mounted_sharpener')
    gap=device.joint('cutter_sleeve')
    collar=device.deg('lock_collar')
    crank=device.deg('crank')
    points=g.BORE+np.array(((-.011,0.,0.),roll(crank)@CRANK_KNOB))+(gap,0.,0.)
    margins=np.linalg.norm(m.fingertips(env)[:,None,:]-points,axis=2).min(axis=0)
    state=Reading(abs(gap)<=.0015,abs(collar-g.LOCK_ANGLE)<=8.,crank,
                  bool(margins[0]>=.047 and margins[1]>=.038),
                  bool(np.isfinite((gap,collar,crank)).all() and device.tilt_deg()<4.))
    score_now=scorer.update(state)
    log=os.getenv('SHARPENER_TRACE')
    if log:
        start = scorer.proof_start_deg
        post_lock_travel = crank - start if start is not None else 0.
        trace=dict(vars(state),score=score_now,axial_gap_m=gap,collar_deg=collar,
                   post_lock_crank_deg=post_lock_travel,post_lock_crank_proved=scorer.crank_proved)
        with open(log,'a',encoding='utf-8') as stream:
            stream.write(json.dumps(trace,allow_nan=False)+'\n')


def score(_objects):
    return scorer.score


class WristPose(float):
    def __new__(cls, rotation):
        value=float.__new__(cls,0.)
        value.rotation=rotation
        return value


def demo():
    from yam_arena import demo as motion, ik, station
    old=motion.grip_rotation
    if not getattr(old,'sharpener_pose',False):
        def grip_rotation(yaw=0.):
            return yaw.rotation if isinstance(yaw,WristPose) else old(yaw)
        grip_rotation.sharpener_pose=True
        motion.grip_rotation=grip_rotation

    front=np.array(((0.,0.,1.),(0.,1.,0.),(-1.,0.,0.)))
    face=WristPose(front)
    final=WristPose(roll(g.LOCK_ANGLE)@front)
    wrist_pitch=axis_angle_matrix(np.array((0.,1.,0.)),math.radians(11.))
    drive_rotation=np.column_stack(((0.,-1.,0.),(0.,0.,-1.),(1.,0.,0.)))
    drive_rotation=wrist_pitch @ drive_rotation
    ring=g.BORE+(-.021,0.,0.)
    incoming=ring+(g.SLEEVE_START_M,0.,0.)
    left_home,left_rotation=ik.tcp_pose(READY,station.BASES['left'])
    right_home,right_rotation=ik.tcp_pose(READY,station.BASES['right'])
    def crank_pose(degrees):
        axis=roll(g.CRANK_START_DEG+degrees)
        wrist=WristPose(axis@drive_rotation)
        return g.BORE+axis@CRANK_KNOB-.010*wrist.rotation[:,2],wrist

    knob,drive_start=crank_pose(0.)
    halfway,drive_middle=crank_pose(29.)
    proof,drive_finish=crank_pose(59.)
    w=motion.Waypoint
    left=[
        w(1.4,incoming-(.054,0.,0.),-.03,'approach the cutter assembly',face),
        w(.7,incoming,-.03,'straddle the retaining collar',face),
        w(.65,incoming,-.017,'hold the collar',face),
        w(1.2,ring,-.017,'seat the replacement cutter in its bore',face),
        w(1.15,ring,-.017,'turn the retaining collar',final),
        w(.68,ring,-.031,'release the retained assembly',final),
        w(.80,ring-(.080,0.,0.),-.031,'withdraw from the collar',final),
        w(1.10,left_home,FINGER_OPEN,'return the assembly hand to rest',WristPose(left_rotation)),
    ]
    right=[
        w(7.7,right_home,FINGER_OPEN,'wait until the cutter is retained',WristPose(right_rotation)),
        w(1.25,knob-.063*drive_start.rotation[:,2]+(0.,0.,.020),-.023,'approach the crank knob',drive_start),
        w(.70,knob,-.023,'straddle the crank knob',drive_start),
        w(.65,knob,-.008,'hold the crank knob',drive_start),
        w(.73,halfway,-.008,'start the clockwise proof turn',drive_middle),
        w(.75,proof,-.008,'turn the crank through the proof mark',drive_finish),
        w(.65,proof,-.024,'release the crank',drive_finish),
        w(.75,proof-.082*drive_finish.rotation[:,2]+(0.,0.,.033),-.024,'lift clear of the sharpener',drive_finish),
    ]
    return {'left':left,'right':right}
