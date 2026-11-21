"""Load sizing and isolated YAM reach for a desk monitor arm counterbalance repair."""
import json
from math import cos, radians, sin

import numpy as np

from yam_arena import demo, ik, station
from yam_arena.constants import READY
from yam_arena.urdf import axis_angle_matrix


MONITOR_KG = 4.2
REACH = .300
LINK_KG = .60
SPRING_TIP_YZ = np.array([.180, 0.])
SPRING_BASE_YZ = np.array([.019, -.055])
TEST_ANGLES_DEG = (-12., 0., 12.)


def spring_leverage(angle):
    r = np.array(((cos(angle), -sin(angle)), (sin(angle), cos(angle))))
    attach = r @ SPRING_TIP_YZ
    shaft = attach - SPRING_BASE_YZ
    direction = shaft / np.linalg.norm(shaft)
    return attach[0] * direction[1] - attach[1] * direction[0]


def load_estimates():
    gravity_torque = (MONITOR_KG * REACH + LINK_KG * REACH / 2.) * 9.81
    balanced_force = gravity_torque / spring_leverage(0.)
    rows = []
    for degrees in TEST_ANGLES_DEG:
        angle = radians(degrees)
        gain = spring_leverage(angle)
        required = gravity_torque * cos(angle)
        balanced = (required - balanced_force * gain) / (REACH * cos(angle))
        weak = (required - .85 * balanced_force * gain) / (REACH * cos(angle))
        rows.append({"angle_deg": degrees, "screen_height_from_neutral_mm": 1000. * REACH * sin(angle),
                     "balanced_manual_support_n": balanced, "reset_manual_support_n": weak})
    assert max(abs(r["balanced_manual_support_n"]) for r in rows) < 3.5
    return {"horizontal_hinge_torque_nm": gravity_torque,
            "illustrative_spring_force_n": balanced_force, "poses": rows}


def try_pose(side, target, rotation):
    q, pe, re = ik.solve(target, ik.matrix_to_quat_xyzw(rotation), READY, base_pos=station.BASES[side], iters=80)
    margin = float(np.rad2deg(np.min(np.minimum(q - ik.LOWER, ik.UPPER - q))))
    if pe <= .003 and re < radians(2.) and margin > 4.:
        return {"target_xyz_m": target, "position_error_mm": pe * 1000.,
                "angle_error_deg": float(np.rad2deg(re)), "joint_margin_deg": margin}
    return None


def wrench_poses():
    results = []
    for spin in (-30., 0., 30.):
        options = []
        for x in (.28, .32, .36, .40):
            for y in (-.22, -.18, -.14):
                for yaw_deg in (55., 65., 75., 85.):
                    yaw = radians(yaw_deg)
                    tool_axis = np.array((cos(yaw), sin(yaw), 0.))
                    close_axis = np.array((sin(yaw), -cos(yaw), 0.))
                    vertical = np.cross(tool_axis, close_axis)
                    rot = axis_angle_matrix(tool_axis, radians(spin)) @ np.column_stack((close_axis, vertical, tool_axis))
                    pose = try_pose("right", (x, y, 1.035), rot)
                    if pose is not None:
                        pose["socket_axis_ground_yaw_deg"] = yaw_deg
                        options.append(pose)
        results.append((spin, options))

    def key(row):
        return (*row["target_xyz_m"], row["socket_axis_ground_yaw_deg"])

    common = set.intersection(*(set(key(r) for r in options) for _, options in results))
    feasible = [[next(r for r in options if key(r) == target) for _, options in results] for target in common]
    return max(feasible, default=[], key=lambda triplet: min(row["joint_margin_deg"] for row in triplet))


def support_pose():
    best = None
    for pitch in (-95., -110., -125., -140.):
        tilt = axis_angle_matrix(np.array((0., 1., 0.)), radians(pitch))
        for yaw in (-135., -90., -45., 0., 45., 90., 135.):
            for x in (.28, .33, .38):
                pose = try_pose("left", (x, .198, .927), tilt @ demo.grip_rotation(radians(yaw)))
                if pose is not None and (best is None or pose["joint_margin_deg"] > best["joint_margin_deg"]):
                    best = pose | {"pitch_deg": pitch, "yaw_deg": yaw}
    return best


if __name__ == "__main__":
    print(json.dumps({"load": load_estimates(), "wrench_arc_samples": wrench_poses(),
                      "bezel_support_pose": support_pose()}, indent=2))
