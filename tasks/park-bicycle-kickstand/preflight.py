"""Check support, torque and arm reach for a full-size side-stand bicycle."""
from math import acos, cos, degrees, radians, sin
import json

import numpy as np
from scipy.optimize import brentq
from yam_arena import demo, ik, station
from yam_arena.urdf import axis_angle_matrix

MASS_KG = 16.78
WHEELBASE_M = 1.195
DECK_HEIGHT_M = .44
TIRE_RADIUS_M = .334
TIRE_SECTION_M = .027
FRAME_CENTER = np.array([.390, 0., DECK_HEIGHT_M])
CENTER_OF_MASS = np.array([0., -.015, .55])
BRACE_POINT = np.array([0., .102, .736])
STAND_PIVOT = np.array([-.052, -.245, .291])
STAND_LENGTH_M = .300
SHOE_RADIUS_M = .013
FOLDED = np.array([-.035, -.294, -.012])
DEPLOYED = np.array([-.156, .050, -.250])
for vector in (FOLDED, DEPLOYED):
    vector *= STAND_LENGTH_M / np.linalg.norm(vector)

SWEEP_AXIS = np.cross(FOLDED, DEPLOYED)
SWEEP_AXIS /= np.linalg.norm(SWEEP_AXIS)
SWEEP_RAD = acos(float(FOLDED @ DEPLOYED) / STAND_LENGTH_M**2)


def tilted(point, roll):
    # A round tire cross section moves the frame up slightly as the bike leans.
    lift = TIRE_SECTION_M * (1. - cos(roll))
    return axis_angle_matrix(np.array([0., 1., 0.]), roll) @ point + [0., 0., lift]


def stand_point(angle, fraction):
    return STAND_PIVOT + fraction * (axis_angle_matrix(SWEEP_AXIS, angle) @ FOLDED)


def statics():
    shoe = stand_point(SWEEP_RAD, 1. - SHOE_RADIUS_M / STAND_LENGTH_M)
    parked = brentq(lambda roll: tilted(shoe, roll)[2] - SHOE_RADIUS_M, -.40, -.03)
    foot = tilted(shoe, parked) - [0., 0., SHOE_RADIUS_M]
    mass_projection = tilted(CENTER_OF_MASS, parked)[:2]
    tire_x = TIRE_SECTION_M * sin(parked)
    contact_points = np.array([[tire_x, -WHEELBASE_M/2], [tire_x, WHEELBASE_M/2], foot[:2]])
    shares = np.linalg.solve(np.vstack((contact_points.T, np.ones(3))), [*mass_projection, 1.])
    if np.any(shares <= .06):
        raise ValueError('A contact is too close to unloading in the nominal parked pose')
    angles = [radians(a) for a in (5, 0, degrees(parked))]
    brace_force = max(abs(MASS_KG * 9.81 * (tilted(CENTER_OF_MASS, a)[0] - TIRE_SECTION_M*sin(a))
                          / tilted(BRACE_POINT, a)[2]) for a in angles)
    assert brace_force <= 32.
    loads = shares * MASS_KG * 9.81
    rotation = axis_angle_matrix(np.array([0., 1., 0.]), parked)
    pivot_arm = rotation @ (shoe - STAND_PIVOT)
    stand_torque = float(np.cross(pivot_arm,[0.,0.,loads[-1]]) @ (rotation @ SWEEP_AXIS))
    if stand_torque < .5:
        raise ValueError('The nominal ground load must press the leg into its deployed mechanical stop')
    return dict(park_deg=degrees(parked), park_roll_rad=parked, loads_n=loads.tolist(),
                max_brace_force_n=brace_force, stop_torque_nm=stand_torque,
                sweep_deg=degrees(SWEEP_RAD), foot_xyz_m=foot.tolist())


def probe_pose(position, tangent, arm, previous=None):
    base = station.BASES[arm]
    best = None
    tangent = tangent / np.linalg.norm(tangent)
    # Search discrete tool orientations in the free aisle. Carry continuous
    # solution seeds into adjacent poses to avoid unrelated IK branches.
    for pitch in (-20, -35, -50, -65, -80, -95, -110, -125):
        pitch_rot = axis_angle_matrix(np.array([0., 1., 0.]), radians(pitch))
        for yaw in np.deg2rad(np.arange(-180., 180., 22.5)):
            rotation = pitch_rot @ demo.grip_rotation(yaw)
            if abs(float(rotation[:, 0] @ tangent)) > .45:
                continue
            quat = ik.matrix_to_quat_xyzw(rotation)
            seed = previous if previous is not None else [0., 1., 1., -.6, 0., 0.]
            q, pe, re = ik.solve(position, quat, seed, base_pos=base, iters=80)
            margin = float(np.min(np.minimum(q-ik.LOWER,ik.UPPER-q)))
            if pe > .004 or re > radians(3.) or margin < radians(6.):
                continue
            cost = 100. * pe + re - .25 * margin
            if previous is not None:
                cost += .25 * np.linalg.norm(q-previous)
            if best is None or cost < best[0]:
                best = (cost, q, pe, re, margin, pitch, yaw)
    if best is None:
        raise ValueError(f'No reachable tool pose for {arm} at {position}')
    return best[1:]


def reach(parked):
    targets = {
        'left': [(FRAME_CENTER + tilted(BRACE_POINT, a), np.array([.01, 1., -.12]))
                 for a in np.linspace(radians(5), parked, 9)],
        'right': [(FRAME_CENTER + stand_point(a, .62), axis_angle_matrix(SWEEP_AXIS, a) @ FOLDED)
                  for a in np.linspace(0., SWEEP_RAD, 13)],
    }
    results = {}
    for arm, points in targets.items():
        q = None
        rows = []
        for point,tangent in points:
            q, error, angle, margin, pitch, yaw = probe_pose(point, tangent, arm, q)
            rows.append((point.tolist(), error * 1000., degrees(angle), degrees(margin), pitch, degrees(yaw)))
        results[arm] = rows
    return results


if __name__ == '__main__':
    result = statics()
    print(json.dumps(result), flush=True)
    print(json.dumps(reach(result['park_roll_rad'])))
