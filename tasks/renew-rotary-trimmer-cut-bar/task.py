"""Turn a worn backing bar on an assembled rotary paper trimmer."""
import json
import math
import os
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parent))
import mechkit as m
from scorer import Reading, Scorer
from yam_arena.constants import FINGER_OPEN, GRIPPER, READY
from yam_arena.ik import matrix_to_quat_xyzw
from yam_arena.station import TABLE_TOP
from yam_arena.urdf import axis_angle_matrix

SCENE = "station"
INSTRUCTION = "Turn the trimmer's worn cutting bar to a fresh face. Seat it flush and close the cutting guide."
GRIPPER["effort"] = 40.
CUTTER = np.array((.335,.012,TABLE_TOP))
BAR_LENGTH, SIDE = .312,.008
START_TILT = -4.
OFFSET_Y = -.048
START = CUTTER+(0.,OFFSET_Y,.0244)
TARGET = CUTTER+(0.,0.,.013)
GUIDE_AXIS = np.array((.054,0.,.021))
OPEN_DEG = 98.
GRASP_Y = -.149
LOOKS = {
    "deck": m.Look(m.rgb(.26,.40,.45),roughness=.65),
    "rail": m.Look(m.rgb(.14,.23,.27),roughness=.58),
    "guard": m.Look(m.rgb(.37,.57,.61),roughness=.31,opacity=.57),
    "base": m.Look(m.rgb(.15,.16,.17),roughness=.71),
    "bar": m.Look(m.rgb(.93,.66,.22),roughness=.61),
    "line": m.Look(m.rgb(.31,.24,.15),roughness=.80),
    "tool": m.Look(m.rgb(.36,.30,.22),roughness=.82),
    "marker": m.Look(m.rgb(.83,.90,.88),roughness=.65),
}


def rot(axis, degrees):
    return axis_angle_matrix(np.eye(3)[axis],math.radians(degrees))


def solids(rows):
    return (tuple((n,m.box(size,at),look) for n,size,at,look in rows),
            tuple(m.col_box(n+"_contact",size,at) for n,size,at,_ in rows))


def deck():
    rows = [
        ("body",(.242,.348,.009),(.082,0.,.0045),"base"),
        ("large_deck",(.192,.348,.008),(.102,0.,.013),"deck"),
        ("narrow_deck",(.024,.315,.008),(-.018,0.008,.013),"deck"),
        ("near_stop",(.018,.010,.008),(0.,-.162,.013),"rail"),
        ("far_stop",(.018,.010,.008),(0.,.162,.013),"rail"),
        ("near_pivot",(.032,.015,.021),(.042,-.166,.016),"rail"),
        ("far_pivot",(.032,.015,.021),(.042,.166,.016),"rail"),
    ]
    visual, collider = solids(rows)
    guide = (
        ("guide_bar",(.032,.322,.010),(.008,0.,.024),"guard"),
        ("guarded_carriage",(.040,.027,.022),(.007,-.137,.040),"deck"),
        ("handle_plate",(.023,.019,.007),(-.017,.106,.027),"rail"),
        ("near_ear",(.034,.008,.008),(.037,-.161,.023),"rail"),
        ("far_ear",(.034,.008,.008),(.037,.161,.023),"rail"),
    )
    faces, contacts = solids(guide)
    hinge = m.hinge("guide","Y",GUIDE_AXIS,-1.,100.,faces,contacts,mass=.047,gravity=True,
                    spring=0.,damping=.03,friction=.025,effort=.7,armature=.00001,
                    inertia=(.0004,.000053,.0004))
    lines = tuple((f"scale_mark_{j}",m.box((.066,.0005,.0002),(.056,y,.0171)),"marker")
                  for j,y in enumerate(np.linspace(-.145,.145,16)))
    return m.mechanism("trimmer",CUTTER,[hinge],LOOKS,visual+lines,collider,friction=(.53,.39),
                       solver_iters=32,start=(("guide",math.radians(OPEN_DEG)),))


def cutting_bar():
    faces = (
        ("core",(.008,BAR_LENGTH,.0065),(0.,0.,-.00075),"bar"),
        ("worn_left",(.00335,BAR_LENGTH,.0015),(-.002325,0.,.00325),"bar"),
        ("worn_right",(.00335,BAR_LENGTH,.0015),(.002325,0.,.00325),"bar"),
        ("wear_line",(.0013,BAR_LENGTH-.014,.00025),(0.,0.,.00257),"line"),
    )
    visual = solids(faces)[0]
    orientation = tuple(matrix_to_quat_xyzw(rot(0,START_TILT)))
    return m.free("cutting_bar",START,visual,(m.col_box("bar_contact",(SIDE,BAR_LENGTH,SIDE)),),
                  LOOKS,.019,friction=(.66,.51),start_quat=orientation,torsional_patch=.003)


def service_tool():
    rows = (("handle",(.058,.016,.012),(-.029,0.,.017),"tool"),
            ("end",(.012,.015,.004),(.006,0.,.0295),"tool"))
    faces, contacts = solids(rows)
    return m.fixed("service_lever",CUTTER+(-.007,-.201,0.),faces,contacts,LOOKS)


OBJECTS = [deck(), cutting_bar(), service_tool()]
scorer = Scorer()


@m.on_reset
def reset(env):
    scorer.reset()
    base = env.unwrapped
    bar = base.scene["cutting_bar"]
    pose = bar.data.default_root_pose.torch.clone()
    pose[:,:3] += base.scene.env_origins
    bar.write_root_pose_to_sim_index(root_pose=pose)
    bar.write_root_velocity_to_sim_index(root_velocity=bar.data.default_root_vel.torch.clone().zero_())
    asset = base.scene["trimmer"]
    initial = asset.data.default_joint_pos.torch.clone()
    asset.write_joint_state_to_sim_index(position=initial,velocity=asset.data.default_joint_vel.torch.clone().zero_())
    asset.set_joint_position_target(initial)
    base.sim.forward()
    cameras = [base.scene[name] for name in ("top_cam","left_cam","right_cam")]
    for camera in cameras:
        camera.reset()
    # Refill temporal history at the restored poses before delivering RGB.
    for _ in range(64):
        base.sim.render()
    for camera in cameras:
        camera.update(0.,force_recompute=True)


@m.on_step
def observe(env):
    bar = m.state(env,"cutting_bar")
    guide = m.state(env,"trimmer")
    ends = bar.to_world([(0.,-BAR_LENGTH/2.,0.),(0.,BAR_LENGTH/2.,0.)])
    delta = ends-TARGET
    rotation = bar.rotation()
    fresh_up = abs(float(rotation[2,0])) >= .98 or float(rotation[2,2]) <= -.98
    clear_track = bool(np.all(delta[:,2] > .015) or np.all(delta[:,0] > .027)
                       or np.all(delta[:,0] < -.027))
    within_track = np.max(np.abs(delta[:,0])) <= .0017 and np.max(np.abs(delta[:,1])) <= BAR_LENGTH/2.+.003
    height = np.abs(delta[:,2]).max()
    axis = ends[1]-ends[0]
    offsets = m.fingertips(env)-ends[0]
    fractions = np.clip((offsets @ axis)/(axis @ axis),0.,1.)
    clear = float(np.linalg.norm(offsets-fractions[:,None]*axis,axis=1).min()) >= .035
    valid = bool(np.all(np.isfinite(ends)) and np.all((ends[:,0] > .035) & (ends[:,0] < .72))
                 and np.all(np.abs(ends[:,1])<.50) and np.all(ends[:,2]>TABLE_TOP-.004))
    point_velocity = np.linalg.norm(bar.lin_vel)
    outcome = Reading(clear_track,bool(fresh_up and within_track and height < .009),bool(height <= .0009),
                      bool(guide.deg("guide") <= 5.),bool(clear and point_velocity < .020),valid)
    score_now = scorer.update(outcome)
    trace = os.getenv("TRIMMER_TRACE")
    if trace:
        with open(trace,"a",encoding="utf-8") as stream:
            row = dict(vars(outcome), score=score_now, bar_m=bar.pos.tolist(), ends_m=ends.tolist(),
                       rotation=rotation.tolist(), guide_deg=guide.deg("guide"),
                       fingertips_m=m.fingertips(env).tolist())
            stream.write(json.dumps(row,allow_nan=False)+"\n")


def score(_objects):
    return scorer.score


class WristPose(float):
    def __new__(cls, rotation):
        instance = float.__new__(cls,0.)
        instance.rotation = rotation
        return instance


def demo():
    from yam_arena import demo as motion, ik, station

    original = motion.grip_rotation
    if not getattr(original,"trimmer_pose",False):
        def grip_rotation(yaw=0.):
            return yaw.rotation if isinstance(yaw,WristPose) else original(yaw)
        grip_rotation.trimmer_pose = True
        motion.grip_rotation = grip_rotation

    down = np.diag((1.,-1.,-1.))
    start = WristPose(rot(0,START_TILT) @ down)
    turned = WristPose(rot(1,-90.) @ down)
    reference = np.array((0.,GRASP_Y,.014))
    grasp = START + rot(0,START_TILT) @ reference
    upper = grasp + (0.,0.,.069)
    handoff = np.array((.347,.010,.881))
    pass_bar = handoff + rot(1,-90.) @ reference
    take_ref = np.array((0.,.104,.0155))
    # A fixed handoff pose measured in the seed-zero rehearsal with the right grip held.
    supported_center = np.array((.354281,.009119,.875697))
    supported_rotation = np.array(((.0077786,.0565793,-.9983677),
                                   (.0200334,.9981888,.0567253),
                                   (.9997689,-.0204420,.0066311)))
    correction = supported_rotation @ rot(1,-90.).T
    left_down = np.diag((-1.,1.,-1.))
    take = supported_center + correction @ take_ref
    taking_pose = WristPose(correction @ left_down)
    # Center the released ends inside the stop-to-stop clearance measured in the smoke.
    release = TARGET + take_ref + (0.,-.0015,.008)
    rest, rest_rotation = ik.tcp_pose(READY,station.BASES["left"])
    # Push a free section of the guide after the bar has settled.
    handle = np.array((-.017,.106,.0305))
    from_pivot = handle-GUIDE_AXIS
    top = CUTTER+GUIDE_AXIS+rot(1,OPEN_DEG) @ from_pivot
    outside_handle = top + (.032,0.,.011)
    middle = CUTTER+GUIDE_AXIS+rot(1,56.) @ from_pivot
    middle += (.006,0.,.011)
    bottom = CUTTER+handle+(0.,0.,.0115)
    camera = WristPose(rot(2,25.) @ rot(1,-10.) @ np.diag((-1.,1.,-1.)))
    waypoint = motion.Waypoint
    right = [
        waypoint(1.50,upper,FINGER_OPEN,"approach the raised bar end",start),
        waypoint(1.05,grasp,FINGER_OPEN,"reach its sides",start),
        waypoint(1.30,grasp,-.0015,"grip the raised end",start),
        waypoint(1.05,upper,-.0015,"lift the original bar",start),
        waypoint(1.45,pass_bar,-.0015,"turn a fresh side up",turned),
        waypoint(3.35,pass_bar,-.0015,"hold while the other hand takes the bar",turned),
        waypoint(.65,pass_bar,-.009,"open the first grip",turned),
        waypoint(1.20,(.225,-.138,.9205),-.009,"clear the deck and face the track",camera),
    ]
    left_pose = WristPose(left_down)
    left = [
        waypoint(6.35,rest,FINGER_OPEN,"keep clear of the bar",WristPose(rest_rotation)),
        waypoint(1.20,take+(0.,0.,.028),FINGER_OPEN,"approach the other end",taking_pose),
        waypoint(.85,take,FINGER_OPEN,"straddle the turned bar",taking_pose),
        waypoint(.85,take,-.0015,"hold the far end",taking_pose),
        waypoint(2.30,take,-.0015,"hold while the first grip opens",taking_pose),
        waypoint(1.05,release+(0.,0.,.043),-.0015,"align along the track",left_pose),
        waypoint(.90,release,-.0015,"lower the bar near the deck",left_pose),
        waypoint(.70,release,-.008,"release into the track",left_pose),
        waypoint(.50,release+(0.,0.,.133),-.003,"withdraw vertically above the bar",left_pose),
        waypoint(.53,outside_handle+(0.,0.,.022),-.003,"move above and outside the guide handle",left_pose),
        waypoint(.37,outside_handle,-.003,"approach outside the guide handle",left_pose),
        waypoint(.85,middle,-.003,"push the open guide from the side",left_pose),
        waypoint(1.15,bottom,-.003,"close the cutting guide",left_pose),
        waypoint(.84,bottom+(0.,0.,.075),-.003,"release the guide",left_pose),
    ]
    return {"left":left,"right":right}
