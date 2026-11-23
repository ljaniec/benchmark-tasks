"""Fixed-layout RGB-only rehearsal; offline IK, no live object-pose feedback.

The right hand comes in level from the arms' side, fingers pointing along the row and closing side to side
on ball 1's equator, under its lines. It lifts the ball 35 degrees towards the arms along its swing circle
(the ball turns about the pinch axis as it goes), holds it still, opens, and backs away.
The left arm stays parked.
"""
import math
import numpy as np
import spec

OPEN = .045
SQUEEZE = .015
LIFT_DEG = 35.
PARK_LEFT = np.array([.18, .36, .95])
BALL = np.array(spec.ball_pose(0, 0.)[0])
START = np.array([.26, -.06, BALL[2] + .060])
PRE = BALL + [-.080, 0., 0.]
BACK_OFF = np.array([-.080, 0., 0.])
# Times (s).
PRE_T, GRASP_T, CLOSE_T, LIFT_T, HOLD_T, OPEN_T, AWAY_T = 2.5, 4.0, 5.0, 7.5, 8.5, 9.0, 10.5
END = 12.0


def smooth(u):
    u = min(max(u, 0.), 1.)
    return u ** 3 * (10 - 15 * u + 6 * u * u)


def lifted(angle):
    """Ball 1's centre swung `angle` degrees out, aimed 6 mm outside its swing circle: the arm settles a few mm short
    of its targets, and the strings must stay taut."""
    a = math.radians(angle)
    return np.array([spec.ball_rest_x(0), 0., spec.RAIL_Z]) + (spec.SWING_R + .006) * np.array([-math.sin(a), 0., -math.cos(a)])


def right_path(t):
    """Commanded right-hand pinch point (m) and aperture (m) at time t (s)."""
    if t < PRE_T:
        return START + smooth(t / PRE_T) * (PRE - START), OPEN
    if t < GRASP_T:
        return PRE + smooth((t - PRE_T) / (GRASP_T - PRE_T)) * (BALL - PRE), OPEN
    if t < CLOSE_T:
        return BALL, OPEN + smooth((t - GRASP_T) / (CLOSE_T - GRASP_T)) * (SQUEEZE - OPEN)
    if t < LIFT_T:
        return lifted(LIFT_DEG * smooth((t - CLOSE_T) / (LIFT_T - CLOSE_T))), SQUEEZE
    top = lifted(LIFT_DEG)
    if t < HOLD_T:
        return top, SQUEEZE
    if t < OPEN_T:
        return top, SQUEEZE + smooth((t - HOLD_T) / (OPEN_T - HOLD_T)) * (OPEN - SQUEEZE)
    return top + smooth((t - OPEN_T) / (AWAY_T - OPEN_T)) * BACK_OFF, OPEN


# Fingers point forward (+x) and close along world y; the hand's y axis points down.
GRIP = np.column_stack([(0., -1., 0.), (0., 0., -1.), (1., 0., 0.)])
DOWN = np.diag([-1., 1., -1.])
SEED = [0., 1.67, .31, 1.35, 0., 1.57]


def plan(ik, constants, station):
    steps = int(END * spec.CONTROL_HZ)
    q_left, ep, er = ik.solve(PARK_LEFT, ik.matrix_to_quat_xyzw(DOWN), constants.READY, station.BASES['left'],
                              iters=400, damping=.015)
    errors = [{'time': 0., 'side': 'left', 'position_m': float(ep), 'rotation_rad': float(er)}]
    f_left = spec.finger_joint_position(.040)
    left = [*q_left, f_left, f_left]
    result = []
    quat = ik.matrix_to_quat_xyzw(GRIP)
    q = np.array(SEED, float)
    for k in range(steps + 1):
        t = k / spec.CONTROL_HZ
        p, gap = right_path(t)
        q, ep, er = ik.solve(p, quat, q, station.BASES['right'], iters=600 if k == 0 else 80, damping=.015)
        errors.append({'time': t, 'side': 'right', 'position_m': float(ep), 'rotation_rad': float(er),
                       'target': p.tolist()})
        f = spec.finger_joint_position(gap)
        result.append((k, {'left': list(left), 'right': [*q, f, f]}))
    bad = [e for e in errors if e['position_m'] > .002 or e['rotation_rad'] > .03]
    if bad:
        raise RuntimeError('Unreachable nominal waypoint: ' + str(bad[:8]))
    return result, errors


class Controller:
    def __init__(self, ik, constants, station):
        self.plan, self.errors = plan(ik, constants, station)
        self.observations = []

    def action(self, step, rgb):
        if set(rgb) != {'overhead_cam', 'left_wrist_cam', 'right_wrist_cam'}:
            raise ValueError('Three RGB views required')
        if step % 20 == 0:
            self.observations.append({'step': step})
        k = min(step + 1, len(self.plan) - 1)
        return {s: np.asarray(v) for s, v in self.plan[k][1].items()}
