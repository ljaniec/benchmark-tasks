"""Empty the scraps from a desk punch and secure its hinged cover."""
import json
import math
import os
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as m
import punch_scene as scene
from scorer import Outcome, Scorer
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, GRIPPER, PAD_TIP_PAST_TCP, READY


SCENE = "station"
INSTRUCTION = "Empty the paper scraps into the waste tray. Close the punch's bottom cover while holding it, then leave it upright on its starting pad."
GRIPPER["effort"] = 40.
OBJECTS = [scene.punch(), scene.tray(), scene.starting_pad(), *scene.scraps()]
SUCCESS_HOLD_S = 0.

scorer = Scorer()
_cover_ticks = 0
_cover_verified = False
_previous_scraps = None


def containment(world, local):
    in_tray = scene.in_tray_mask(world - scene.TRAY_CENTER)
    in_chamber = (np.all(np.abs(local[:, :2]) <= (.036, .046), axis=1)
                  & (local[:, 2] >= -.001) & (local[:, 2] <= .013) & ~in_tray)
    return in_tray, in_chamber


@m.on_reset
def reset(env):
    global _cover_ticks, _cover_verified, _previous_scraps
    scorer.reset()
    _cover_ticks, _cover_verified, _previous_scraps = 0, False, None
    base = env.unwrapped
    for name in ("paper_punch", *scene.SCRAP_NAMES):
        part = base.scene[name]
        pose = part.data.default_root_pose.torch.clone()
        pose[:, :3] += base.scene.env_origins
        velocity = part.data.default_root_vel.torch.clone().zero_()
        part.write_root_pose_to_sim_index(root_pose=pose)
        part.write_root_velocity_to_sim_index(root_velocity=velocity)
    punch = base.scene["paper_punch"]
    punch.write_joint_state_to_sim_index(
        position=punch.data.default_joint_pos.torch.clone(),
        velocity=punch.data.default_joint_vel.torch.clone().zero_(),
    )
    cameras = [base.scene[name] for name in ("top_cam", "left_cam", "right_cam")]
    base.sim.forward()
    for camera in cameras:
        camera.reset()
    # Fill the renderer's temporal history while the restored scene stays fixed.
    for _ in range(32):
        base.sim.render()
    for camera in cameras:
        camera.update(0., force_recompute=True)


@m.on_step
def observe(env):
    global _cover_ticks, _cover_verified, _previous_scraps
    punch = m.state(env, "paper_punch")
    disks = np.asarray([m.state(env, name).pos for name in scene.SCRAP_NAMES])
    in_tray, in_chamber = containment(disks, punch.to_local(disks))
    tray_count, chamber_count = int(in_tray.sum()), int(in_chamber.sum())
    angle, flex = punch.deg("waste_cover"), punch.joint("elastic_tab")
    closed = abs(angle) <= 2. and abs(flex) <= .00035
    tips = m.fingertips(env)
    closed_tab = punch.to_world([scene.RELEASE_FACE])[0]
    tip_local = punch.to_local(tips)
    cover_lo, cover_hi = np.array([-.059, -.050, -.003]), np.array([.034, .050, .010])
    cover_distances = np.linalg.norm(np.maximum(np.maximum(cover_lo - tip_local, tip_local - cover_hi), 0.), axis=1)
    air_test = (closed and tray_count == len(disks) and punch.pos[2] >= scene.DESK_Z + .045
                and cover_distances[12:].min() >= .035 and cover_distances[:12].min() >= .006)
    _cover_ticks = _cover_ticks + 1 if air_test else 0
    _cover_verified = closed and (_cover_verified or _cover_ticks >= 10)
    motion = (np.linalg.norm(disks - _previous_scraps, axis=1).max()
              if _previous_scraps is not None else float("inf"))
    _previous_scraps = disks.copy()
    base_z = -.003 if closed else -.096
    lo, hi = np.asarray([-.059, -.050, base_z]), np.asarray([.041, .050, .070])
    punch_distance = np.linalg.norm(np.maximum(np.maximum(lo - tip_local, tip_local - hi), 0.), axis=1).min()
    disk_distance = np.linalg.norm(tips[:, None, :] - disks[None, :, :], axis=2).min() - scene.SCRAP_DIAMETER_M / 2
    clearance = min(punch_distance, disk_distance)
    flat = (punch.tilt_deg() <= 5. and abs(punch.pos[2] - scene.PUNCH_START[2]) <= .004
            and np.linalg.norm(punch.pos[:2] - np.asarray(scene.PUNCH_START[:2])) <= .018)
    within = bool(np.all((disks[:, 0] >= .08) & (disks[:, 0] <= .62)
                         & (np.abs(disks[:, 1]) <= .43) & (disks[:, 2] >= scene.DESK_Z - .014))
                  and .08 <= punch.pos[0] <= .62 and abs(punch.pos[1]) <= .43 and punch.pos[2] >= scene.DESK_Z - .025)
    intact = bool(-94. < angle <= 4. and -.0004 < flex < .0021)
    still = bool(motion <= .0015 and np.linalg.norm(punch.lin_vel) <= .025)
    value = scorer.update(Outcome(tray_count, chamber_count, float(clearance), _cover_verified,
                                  flat, intact, within, still))
    path = os.getenv("PUNCH_TRACE")
    if path:
        evidence = {
            "score": value, "tray": tray_count, "chamber": chamber_count,
            "cover_deg": angle, "tab_mm": 1000. * flex, "cover_verified": bool(_cover_verified),
            "punch_m": punch.pos.tolist(), "punch_quat_xyzw": punch.quat_xyzw.tolist(),
            "tilt_deg": punch.tilt_deg(), "clearance_m": float(clearance),
            "release_face_m": closed_tab.tolist(), "right_tip_points_m": tips[12::2].tolist(),
            "outside_m": {scene.SCRAP_NAMES[i]: disks[i].tolist() for i in np.flatnonzero(~(in_tray | in_chamber))},
            "flat_on_pad": bool(flat), "intact": intact, "within_workspace": within, "still": still,
            "cover_clearance_m": [float(cover_distances[:12].min()), float(cover_distances[12:].min())],
        }
        with open(path, "a", encoding="utf-8") as trace:
            trace.write(json.dumps(evidence, allow_nan=False) + "\n")


def score(_objects):
    return scorer.score


class WristPose(float):
    def __new__(cls, rotation):
        pose = float.__new__(cls, 0.)
        pose.rotation = rotation
        return pose


def wrist_orientation(yaw_deg, pitch_deg, roll_deg=0.):
    from yam_arena.urdf import axis_angle_matrix

    yaw, pitch = map(math.radians, (yaw_deg, pitch_deg))
    axis = np.array([math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw), math.sin(pitch)])
    jaw = np.array([0., 0., 1.]) - axis * axis[2]
    jaw /= np.linalg.norm(jaw)
    return axis_angle_matrix(axis, math.radians(roll_deg)) @ np.column_stack((jaw, np.cross(axis, jaw), axis))


def release_route(yaw_deg=0., pitch_deg=0., roll_deg=0., retreat=.068, rise=.028):
    from yam_arena import demo as motion, ik, station

    rotation = wrist_orientation(yaw_deg, pitch_deg, roll_deg)
    level = WristPose(rotation)
    axis = rotation[:, 2]
    support = np.array([.347, -.020, .898])
    contact = support + scene.RELEASE_FACE - PAD_TIP_PAST_TCP * axis + (0., 0., -.004)
    before = contact - .006 * axis
    entry = contact - retreat * axis + (0., 0., rise)
    pressed = contact + (.010, 0., 0.)
    lowered = pressed + (.001, 0., -.012)
    withdrawn = before + (0., 0., -.012)
    park, idle_rotation = ik.tcp_pose(np.asarray(READY), station.BASES["right"])
    way = motion.Waypoint
    return [
        way(6.55, park, FINGER_CLOSED, "wait clear of the pickup", WristPose(idle_rotation)),
        way(1.1, entry, FINGER_CLOSED, "approach from the release side", level),
        way(1.2, before, FINGER_CLOSED, "line up on the button", level),
        way(1.65, pressed, FINGER_CLOSED, "press the button inward", level),
        way(.90, lowered, FINGER_CLOSED, "guide the depressed release down", level),
        way(.70, withdrawn, FINGER_CLOSED, "give the scraps space to fall", level),
        way(.8, entry, FINGER_CLOSED, "withdraw from the open cover", level),
        way(1.1, park, FINGER_CLOSED, "clear of the punch", WristPose(idle_rotation)),
    ]


def demo():
    from yam_arena import demo as motion
    from yam_arena.urdf import quat_xyzw_to_matrix

    if not getattr(motion.grip_rotation, "punch_wrist_frames", False):
        original = motion.grip_rotation

        def grip_rotation(yaw=0.):
            return yaw.rotation if isinstance(yaw, WristPose) else original(yaw)

        grip_rotation.punch_wrist_frames = True
        motion.grip_rotation = grip_rotation

    way = motion.Waypoint
    left_wrist = WristPose(quat_xyzw_to_matrix((0., .906307787, -.422618262, 0.)))
    pickup = np.asarray(scene.PUNCH_START) + (0., 0., .0405)
    lift = pickup + (0., 0., .105)
    carry = (.350, -.025, .954)
    check_height = pickup + (0., 0., .078)
    level_return = (*pickup[:2], carry[2])
    latch_seat = pickup - (0., 0., .004)
    left = [
        way(1.5, lift, FINGER_OPEN, "above the punch", left_wrist),
        way(1.4, pickup, FINGER_OPEN, "line up on the housing", left_wrist),
        way(.8, pickup, -.038, "grip the housing", left_wrist),
        way(1.4, lift, -.038, "lift the punch", left_wrist),
        way(1.4, carry, -.038, "hold over the tray", left_wrist),
    ]
    right = release_route()
    hold = sum(w.duration for w in right) + 1.2 - sum(w.duration for w in left)
    left.extend([
        way(hold, carry, -.038, "support the punch while emptying it", left_wrist),
        way(1.5, level_return, -.038, "carry the open cover over the tray rim", left_wrist),
        way(2.7, pickup, -.038, "press the cover shut against the desk", left_wrist),
        way(.45, latch_seat, -.038, "seat the return latch", left_wrist),
        way(1.2, check_height, -.038, "lift to show the cover holds", left_wrist),
        way(.68, check_height, -.038, "display the released cover", left_wrist),
        way(1.4, pickup, -.038, "set it on the pad", left_wrist),
        way(.65, pickup, FINGER_OPEN, "release the punch", left_wrist),
        way(1.1, lift, FINGER_OPEN, "leave the finish clear", left_wrist),
    ])
    park = right[-1]
    side_view = (.235, 0., .815)
    side_wrist = WristPose(wrist_orientation(80., -18.))
    right.extend([
        way(2.7, park.pos, park.finger, "wait while the punch is returned", park.yaw),
        way(2.2, side_view, FINGER_CLOSED, "observe the suspended cover", side_wrist),
        way(3.25, side_view, FINGER_CLOSED, "hold the cover in view", side_wrist),
        way(1.6, park.pos, park.finger, "leave the setup clear", park.yaw),
    ])
    return {"left": left, "right": right}
