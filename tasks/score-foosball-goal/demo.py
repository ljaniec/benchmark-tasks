"""Fixed-layout RGB-only rehearsal; offline IK, no live object-pose feedback.

The right hand slides its open fingers over the end of the three-man rod's handle, along the rod's
axis, and closes on the handle (fingers left and right of it). It pushes the rod 50 mm in so the
middle man stands behind the ball, rolls the wrist back 70 degrees to lift the men's feet, then
rolls forward to 60 degrees past vertical in one quick stroke, striking the ball towards the goal.
The left arm stays parked.
"""
import math
import numpy as np
import spec

OPEN = .066
SQUEEZE = .012
GRIP = np.column_stack([(0., -1., 0.), (0., 0., -1.), (1., 0., 0.)])     # tool z along the rod, fingers along y
HANDLE = np.array(spec.HANDLE_CENTRE)
PRE = HANDLE + [-.080, 0., 0.]
PARK_LEFT = np.array([.18, .36, .95])
DOWN = np.diag([-1., 1., -1.])
SLIDE = spec.BALL_OFFSET_X
WIND_DEG = 70.
STRIKE_DEG = -60.
WIND_START, WIND_END, STRIKE_AT = 7.0, 8.0, 8.6
END = 12.0


def smooth(u):
    u = min(max(u, 0.), 1.)
    return u ** 3 * (10 - 15 * u + 6 * u * u)


def right_path(t):
    """Commanded right-hand pinch point (m), aperture (m) and wrist roll (deg) at time t (s)."""
    if t < 2.5:
        return PRE + [0., 0., .06] - smooth(t / 2.5) * np.array([0., 0., .06]), OPEN, 0.
    if t < 4.0:
        return PRE + smooth((t - 2.5) / 1.5) * (HANDLE - PRE), OPEN, 0.
    if t < 5.0:
        return HANDLE, OPEN + smooth(t - 4.) * (SQUEEZE - OPEN), 0.
    pushed = HANDLE + [SLIDE, 0., 0.]
    if t < 7.0:
        return HANDLE + smooth((t - 5.) / 2.) * (pushed - HANDLE), SQUEEZE, 0.
    if t < STRIKE_AT:
        return pushed, SQUEEZE, WIND_DEG * smooth((t - WIND_START) / (WIND_END - WIND_START))
    return pushed, SQUEEZE, STRIKE_DEG


def plan(ik, constants, station):
    steps = int(END * spec.CONTROL_HZ)
    q_left, ep, er = ik.solve(PARK_LEFT, ik.matrix_to_quat_xyzw(DOWN), constants.READY, station.BASES['left'],
                              iters=400, damping=.015)
    errors = [{'time': 0., 'side': 'left', 'position_m': float(ep), 'rotation_rad': float(er)}]
    f_left = spec.finger_joint_position(OPEN)
    left = [*q_left, f_left, f_left]
    quat = ik.matrix_to_quat_xyzw(GRIP)
    # Continuation seed in the elbow-up family that reaches along the rod (rehearsed).
    q = np.array([0., 1.67, .31, 1.35, 0., 0.])
    result = []
    for k in range(steps + 1):
        t = k / spec.CONTROL_HZ
        p, gap, roll = right_path(t)
        q, ep, er = ik.solve(p, quat, q, station.BASES['right'], iters=400 if k == 0 else 80, damping=.015)
        errors.append({'time': t, 'side': 'right', 'position_m': float(ep), 'rotation_rad': float(er),
                       'target': p.tolist()})
        f = spec.finger_joint_position(gap)
        cmd = np.array(q, float)
        cmd[5] += math.radians(roll)       # joint 6 turns the tool about its own axis, the rod's axis
        result.append((k, {'left': list(left), 'right': [*cmd, f, f]}))
    bad = [e for e in errors if e['position_m'] > .002 or e['rotation_rad'] > .03]
    if bad:
        raise RuntimeError('Unreachable nominal waypoint: ' + str(bad[:8]))
    lim = [(r[1]['right'][5]) for r in result]
    if min(lim) < ik.LOWER[5] or max(lim) > ik.UPPER[5]:
        raise RuntimeError('Wrist roll outside its range')
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
