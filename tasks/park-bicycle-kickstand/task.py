"""Pilot task for parking a full-size bicycle on its spring side stand."""
import math
import json
import os
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as m
from bicycle import BUMP_FACE, FRONT, PARK_ZONE, REAR, START_LEAN, bicycle, parking_deck
from camera_view import configure
from preflight import (BRACE_POINT, DECK_HEIGHT_M, FOLDED, FRAME_CENTER, SHOE_RADIUS_M,
                       STAND_LENGTH_M, STAND_PIVOT, SWEEP_RAD, TIRE_RADIUS_M,
                       TIRE_SECTION_M, stand_point, statics, tilted)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, GRIPPER

SCENE = 'station'
RECREATE_SCENE_ON_RESET = True
INSTRUCTION = 'Park the bicycle on its kickstand and leave it standing independently.'
SUCCESS_HOLD_S = 0.
GRIPPER['effort'] = 40.
configure()
BIKE, LINK_BOUNDS = bicycle()
OBJECTS = [parking_deck(), BIKE]
SHOE_CENTER = STAND_PIVOT + FOLDED * (1.-SHOE_RADIUS_M/STAND_LENGTH_M)
PARK_ROLL = statics()['park_roll_rad']
SPRING_PEAK_NM = .85

_score = 0.
_void = False
_dwell = 0


def stand_torque(angle):
    """Passive detent spring, approximated by its over-center torque curve."""
    travel = np.clip(angle / SWEEP_RAD,0.,1.)
    return -SPRING_PEAK_NM * math.cos(math.pi * travel)


@m.on_reset
def reset(env):
    global _score, _void, _dwell
    _score = 0.
    _void = False
    _dwell = 0
    if log := os.environ.get('BIKE_TRACE'):
        Path(log).write_text('')
    body, stand_id = m._joint(env,'bicycle','kickstand')
    base = env.unwrapped
    pose = body.data.default_root_pose.torch.clone()
    pose[:, :3] += base.scene.env_origins
    body.write_root_pose_to_sim_index(root_pose=pose)
    body.write_root_velocity_to_sim_index(root_velocity=body.data.default_root_vel.torch.clone())
    position = body.data.default_joint_pos.torch.clone()
    velocity = body.data.default_joint_vel.torch.clone()
    body.write_joint_state_to_sim_index(position=position,velocity=velocity)
    body.actuators.target_command.set_effort_index(value=m._tensor(body,stand_torque(0.)),joint_ids=[stand_id])
    base.sim.forward()
    for _ in range(3):
        base.sim.render()
    for view in ('top_cam','left_cam','right_cam'):
        base.scene[view].update(0.,force_recompute=True)


@m.on_step
def evaluate(env):
    global _score, _void, _dwell
    stand_angle = m.state(env,'bicycle').joint('kickstand')
    asset, stand_id = m._joint(env,'bicycle','kickstand')
    if not math.isfinite(stand_angle):
        _void = True
        _score = 0.
        return
    asset.actuators.target_command.set_effort_index(value=m._tensor(asset,stand_torque(stand_angle)),joint_ids=[stand_id])
    base = env.unwrapped
    origin = m._np(base.scene.env_origins[0])
    state = m._np(asset.data.body_link_pose_w)[0]
    ids = tuple(asset.body_names.index(key) for key in ('frame','rear','front','kickstand'))
    frame,rear,front,stand = state[list(ids)]
    if not np.isfinite([frame,rear,front,stand]).all():
        _void = True
        _score = 0.
        return
    frame_rotation = m.quat_matrix(frame[3:7])
    roll = math.degrees(math.atan2(frame_rotation[0,2],frame_rotation[2,2]))
    tip_from_floor = (stand[:3]-origin + m.quat_matrix(stand[3:7]) @ SHOE_CENTER)[2]-SHOE_RADIUS_M-DECK_HEIGHT_M
    patches = []
    up = np.array([0.,0.,1.])
    for pose, center in ((rear,REAR),(front,FRONT)):
        rotation = m.quat_matrix(pose[3:7])
        axis = rotation[:,0]
        hub = pose[:3] - origin + rotation @ center
        radial = axis * float(axis @ up) - up
        radial /= max(float(np.linalg.norm(radial)),1e-8)
        patches.append(hub + radial * (TIRE_RADIUS_M-TIRE_SECTION_M) - up * TIRE_SECTION_M)
    xa,xb,ya,yb = PARK_ZONE
    wheels_within = all(xa <= patch[0] <= xb and ya <= patch[1] <= yb for patch in patches)
    tires = [float(patch[2]-DECK_HEIGHT_M) for patch in patches]
    tip_over = bool(frame_rotation[2,2] < math.cos(math.radians(17.)))
    lifted = max(tires) > .015
    _void |= not wheels_within or lifted or tip_over
    brace = frame[:3]-origin + frame_rotation @ BRACE_POINT
    gap = BUMP_FACE - brace[0] - .018
    deployed = abs(stand_angle-SWEEP_RAD)<math.radians(5.)
    tips = m.fingertips(env)
    clearance = math.inf
    for name, bounds in LINK_BOUNDS.items():
        pose = state[asset.body_names.index(name)]
        local = (tips - pose[:3] + origin) @ m.quat_matrix(pose[3:7])
        offset = np.maximum(bounds[:,None,0,:] - local, local - bounds[:,None,1,:])
        gap_to_box = np.linalg.norm(np.maximum(offset,0.),axis=-1).min()
        clearance = min(clearance,float(gap_to_box))
    hands_clear = clearance >= .012 and bool(np.max(tips[:,0]) < .20)
    standing = (gap >= .030 and deployed and all(-.009 <= z <= .003 for z in tires)
                and abs(tip_from_floor) <= .004 and -14. <= roll <= -8.
                and np.linalg.norm(m.state(env,'bicycle').lin_vel)<.012 and hands_clear)
    _dwell = _dwell + 1 if standing and not _void else 0
    _score = 0.
    if not _void and gap >= .010:
        _score = .5 if deployed else .25
        if _dwell >= 10:
            _score = 1.
    log = os.environ.get('BIKE_TRACE')
    if log:
        with Path(log).open('a') as output:
            print(json.dumps(dict(score=_score, roll=roll, deck_tire_heights_m=tires,
                                  tire_patches=[p.tolist() for p in patches],
                                  wheels_within=wheels_within, tipped=tip_over, lifted=lifted,
                                  stand_deg=math.degrees(stand_angle), shoe_height_m=tip_from_floor,
                                  hands_clear=hands_clear, clearance_m=clearance, wall_gap_m=gap, void=bool(_void), dwell=_dwell)),
                  file=output)


def score(_objects):
    return _score


def success(objects):
    return score(objects)==1.


class Grip(float):
    def __new__(cls, yaw, pitch):
        obj = float.__new__(cls,math.radians(yaw))
        obj.pitch = math.radians(pitch)
        return obj


def demo():
    from yam_arena import demo as motion
    from yam_arena.urdf import axis_angle_matrix

    grip_rotation = getattr(motion.grip_rotation,'bike_stock',motion.grip_rotation)

    def rotation(yaw=0.):
        ordinary = grip_rotation(float(yaw))
        return axis_angle_matrix(np.array([0.,1.,0.]),yaw.pitch) @ ordinary if isinstance(yaw,Grip) else ordinary

    rotation.bike_stock = grip_rotation
    motion.grip_rotation = rotation
    wp = motion.Waypoint
    brace_grip = Grip(-67.5,-80.)
    closed = FINGER_CLOSED
    open_gap = -.030

    def brace_at(roll):
        return tuple(FRAME_CENTER + tilted(BRACE_POINT,roll)-[.008,0.,0.])

    def hold_at(angle):
        return tuple(FRAME_CENTER+stand_point(angle,.62))

    start = brace_at(START_LEAN)
    standing = brace_at(0.)
    resting = brace_at(PARK_ROLL + math.radians(2.))
    parked = (*resting[:2], resting[2] - .007)
    left = [wp(2.0,(.22,start[1],start[2]),open_gap,'reach for the bicycle frame',brace_grip),
            wp(1.2,start,open_gap,'position the pads around the top tube',brace_grip),
            wp(.8,start,closed,'grasp the top tube',brace_grip),
            wp(1.6,standing,closed,'bring the bicycle off the wall',brace_grip),
            wp(4.4,standing,closed,'brace upright as the stand opens',brace_grip),
            wp(2.1,parked,closed,'lean the bicycle onto its shoe',brace_grip),
            wp(.9,parked,closed,'transfer load to the three floor contacts',brace_grip),
            wp(.55,parked,open_gap,'release the bicycle',brace_grip),
            wp(1.6,(.164,.28,1.07),open_gap,'withdraw the supporting hand',Grip(-67.5,-65.))]
    right_start = hold_at(0.)
    right_grip = Grip(-90.,-20.)
    right = [wp(4.5,(.19,-.43,.85),FINGER_OPEN,'wait clear of the bicycle',right_grip),
             wp(1.3,right_start,open_gap,'approach the folded stand',right_grip),
             wp(.65,right_start,closed,'grasp the stand',right_grip)]
    for i in range(1,8):
        grip = right_grip if i<5 else Grip(-112.5,-20.)
        right.append(wp(.24,hold_at(SWEEP_RAD*i/12),closed,'swing past the spring midpoint',grip))
    right += [wp(.5,right[-1].pos,open_gap,'release the spring-loaded stand',right[-1].yaw),
              wp(1.05,(.135,-.31,.83),FINGER_OPEN,'withdraw from the support triangle',right_grip)]
    return {'left':left,'right':right}
