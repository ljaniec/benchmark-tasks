"""Clear a physical obstruction from a detached vacuum extension."""
import json
import os
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as m
import layout as g
import parts
from scorer import Outcome, Scorer
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, GRIPPER
from yam_arena.urdf import axis_angle_matrix


SCENE = "station"
INSTRUCTION = "Clear the blockage from the vacuum tube. Leave the debris in the tray and return the clearing rod to its rest."
GRIPPER["effort"] = 40.
PLAN = g.geometry()
TRAY = np.array((PLAN["tube"][0], PLAN["outlet"] + .086, g.TABLE_TOP))
OBJECTS = [
    parts.tube(PLAN["tube"]), parts.plug(PLAN["plug"]), parts.rod(PLAN["rod_rest"]), parts.tray(TRAY),
    parts.rests("tube_rest", PLAN["tube"][0], g.TUBE_REST_Y_M, g.TUBE_OD_M / 2., .002),
    parts.rests("rod_rest", PLAN["rod_rest"][0], g.ROD_REST_Y_M, g.ROD_RADIUS_M, .007),
]
MOVABLE = ("extension_tube", "foam_plug", "clearing_rod")

scorer = Scorer()


@m.on_reset
def reset(env):
    scorer.reset()
    base = env.unwrapped
    for name in MOVABLE:
        asset = base.scene[name]
        pose = asset.data.default_root_pose.torch.clone()
        pose[:, :3] += base.scene.env_origins
        asset.write_root_pose_to_sim_index(root_pose=pose)
        asset.write_root_velocity_to_sim_index(root_velocity=asset.data.default_root_vel.torch.clone().zero_())
    plug = base.scene["foam_plug"]
    initial_pads = plug.data.default_joint_pos.torch.clone()
    plug.write_joint_state_to_sim_index(position=initial_pads,
                                        velocity=plug.data.default_joint_vel.torch.clone().zero_())
    plug.set_joint_position_target(initial_pads.clone().zero_())
    base.sim.forward()
    cameras = [base.scene[name] for name in ("top_cam", "left_cam", "right_cam")]
    for camera in cameras:
        camera.reset()
    for _ in range(24):
        base.sim.render()
    for camera in cameras:
        camera.update(0., force_recompute=True)


def separation(points, a, b):
    """Smallest distance from the sampled points to the finite shaft axis."""
    axis = b - a
    s = np.clip((points - a) @ axis / (axis @ axis), 0., 1.)
    return np.linalg.norm(points - a - s[:, None] * axis, axis=1).min()


def on_both_saddles(ends, rest_x, rest_y, lateral_tolerance):
    """Check that the shaft spans both saddles and sits between their sides."""
    rear, front = sorted(ends, key=lambda end: end[1])
    if rear[1] > rest_y[0] - .015 or front[1] < rest_y[1] + .015:
        return False
    axis = front - rear
    fraction = (np.asarray(rest_y) - rear[1]) / axis[1]
    cross_sections = rear + fraction[:, None] * axis
    return bool(np.all(np.abs(cross_sections[:, 0] - rest_x) <= lateral_tolerance)
                and np.all(np.abs(cross_sections[:, 2] - g.TUBE_Z_M) <= .003))


@m.on_step
def observe(env):
    tube, plug, rod = (m.state(env, name) for name in MOVABLE)
    half_tube = g.TUBE_LENGTH_M / 2.
    tube_ends = tube.to_world([(0., -half_tube, 0.), (0., half_tube, 0.)])
    rod_ends = rod.to_world([(0., -g.HANDLE_M[1] / 2., 0.), (0., g.HANDLE_M[1] / 2. + g.ROD_SHANK_M, 0.)])
    local_plug = tube.to_local([plug.pos])[0]
    plug_extent = parts.PLUG_RADIUS_M + .004
    outside = abs(local_plug[1]) - plug_extent >= half_tube
    visible = abs(local_plug[1]) + g.PLUG_LENGTH_M / 2. >= half_tube + .005
    visible &= not outside and np.linalg.norm(local_plug[[0, 2]]) < .022
    tray_delta = plug.pos - TRAY
    inside = outside and np.all(np.abs(tray_delta[:2]) <= np.asarray(parts.TRAY_HALF_M) - .008)
    inside &= parts.FLOOR_M <= tray_delta[2] <= parts.FLOOR_M + parts.RIM_M
    tube_on_rest = on_both_saddles(tube_ends, PLAN["tube"][0], g.TUBE_REST_Y_M, g.TUBE_REST_CENTER_TOL_M)
    rod_shank_rear = rod.to_world([(0., g.HANDLE_M[1] / 2., 0.)])[0]
    rod_on_rest = on_both_saddles((rod_shank_rear, rod_ends[1]), PLAN["rod_rest"][0],
                                  g.ROD_REST_Y_M, g.ROD_REST_CENTER_TOL_M)
    tips = m.fingertips(env)
    debris_clearance = float(np.linalg.norm(tips - plug.pos, axis=1).min()) - plug_extent
    inside &= debris_clearance >= .035 and float(np.linalg.norm(plug.lin_vel)) <= .025
    clearance = min(separation(tips, *tube_ends) - g.TUBE_OD_M / 2.,
                    separation(tips, *rod_ends) - max(g.HANDLE_M[0], g.HANDLE_M[2]) / 2.,
                    debris_clearance)
    extent = np.vstack((tube_ends, rod_ends, plug.pos[None]))
    bounded = np.all((extent[:, 0] >= .01) & (extent[:, 0] <= .72) & (np.abs(extent[:, 1]) <= .64)
                     & (extent[:, 2] >= g.TABLE_TOP - .005) & (extent[:, 2] <= 1.135))
    pad_limits = np.array([abs(plug.joint(part)) for part in ("right", "left", "top", "bottom")])
    intact = bool(np.isfinite(extent).all() and pad_limits.max() < parts.PAD_TRAVEL_M + .0006)
    still = max(float(np.linalg.norm(o.lin_vel)) for o in (tube, plug, rod)) <= .025
    outcome = Outcome(bool(visible), bool(inside), bool(tube_on_rest), bool(rod_on_rest),
                      bool(clearance >= .035), bool(still), intact, bool(bounded))
    score = scorer.update(outcome)
    trace_path = os.getenv("VACUUM_TRACE")
    if trace_path:
        row = dict(vars(outcome), score=score, tube_m=tube.pos.tolist(), tube_axis=tube_ends.tolist(),
                   rod_m=rod.pos.tolist(), rod_tip_m=rod_ends[-1].tolist(), plug_m=plug.pos.tolist(),
                   plug_local_m=local_plug.tolist(), jaw_clearance_m=float(clearance), pad_travel_m=pad_limits.tolist())
        with open(trace_path, "a", encoding="utf-8") as stream:
            stream.write(json.dumps(row, allow_nan=False) + "\n")


def score(_objects):
    return scorer.score


class WristPose(float):
    def __new__(cls, rotation):
        obj = float.__new__(cls, 0.)
        obj.rotation = rotation
        return obj


def demo():
    from yam_arena import demo as motion

    original = motion.grip_rotation
    if not getattr(original, "vacuum_wrist", False):
        def grip_rotation(yaw=0.):
            return yaw.rotation if isinstance(yaw, WristPose) else original(yaw)

        grip_rotation.vacuum_wrist = True
        motion.grip_rotation = grip_rotation
    waypoint = motion.Waypoint
    left_wrist = WristPose(np.diag((-1., 1., -1.)))
    tray_view = WristPose(axis_angle_matrix(np.array((0., 0., 1.)), np.pi / 3.) @ left_wrist.rotation)
    left_hold = PLAN["hold"] + (0., 0., .009)
    left_hover = left_hold + (0., 0., .055)
    grasp = PLAN["rod_rest"] + (0., 0., .010)
    rest_hover = grasp + (0., 0., .035)
    # The grasp settles below the target TCP when it carries the long rod.
    rod_tcp_offset = (0., 0., .024)
    align = PLAN["rod_aligned"] + rod_tcp_offset
    pre = PLAN["rod_aligned"] + (0., 0., .039)
    inserted = PLAN["rod_inserted"] + rod_tcp_offset
    right = [
        waypoint(1.6, rest_hover, FINGER_OPEN, "above the rear handle"),
        waypoint(1.25, grasp, FINGER_OPEN, "descend onto the handle"),
        waypoint(.72, grasp, -.011, "grip the rectangular handle"),
        waypoint(.8, rest_hover, -.011, "lift the clearing rod"),
        waypoint(1.1, pre, -.011, "align to the tube mouth"),
        waypoint(.9, align, -.011, "level the shank"),
        waypoint(4.1, inserted, -.011, "push out the obstruction"),
        waypoint(1., inserted, -.011, "let the obstruction fall"),
        waypoint(3.5, align, -.011, "withdraw the rod"),
        waypoint(.8, pre, -.011, "clear the tube"),
        waypoint(.86, rest_hover, -.011, "return toward the rest"),
        waypoint(1., grasp, -.011, "lower the rod onto both saddles"),
        waypoint(.65, grasp, FINGER_OPEN, "release the rod"),
        waypoint(1.2, rest_hover + (0., 0., .020), FINGER_OPEN, "clear the bench"),
    ]
    left = [
        waypoint(1.4, left_hover, FINGER_OPEN, "above the tube", left_wrist),
        waypoint(1.35, left_hold, FINGER_OPEN, "straddle the tube", left_wrist),
        waypoint(.7, left_hold, -.007, "grip the tube", left_wrist),
        waypoint(12.3, left_hold, -.007, "support the tube against the rod", left_wrist),
        waypoint(.65, left_hold, FINGER_OPEN, "release the tube onto its supports", left_wrist),
        waypoint(1.2, left_hover, FINGER_OPEN, "clear the tube", left_wrist),
        waypoint(1.6, (.325, .423, .955), FINGER_OPEN, "show the caught debris", tray_view),
    ]
    return {"left": left, "right": right}
