"""Fixed-layout RGB-only rehearsal; offline IK, no live object-pose feedback.

The right hand closes its fingers and comes down behind the striker where it lies on the base lines, fingers
pointing down with their tips 2 mm above the board and the closed pair facing along the line from the striker
through the queen to the far left pocket. It strikes: the hand speeds up along that line to SPEED m/s over
ACCEL m and meets the striker's back edge with the outer face of its front finger while still at full speed.
The striker flies off the finger faster than the hand moves; the hand stops and lifts away. The left arm stays
parked beside the board.
"""
import math
import numpy as np
import spec

TCP_ABOVE_TIPS = .013
PAD_CLEAR = .002                 # finger tips above the board
FACE = .0048                     # the front finger's outer face, ahead of the tool point at the striker's top edge
AIM = math.atan2(spec.QUEEN_START[1] - spec.STRIKER_START[1], spec.QUEEN_START[0] - spec.STRIKER_START[0])
PARK_LEFT = np.array([.18, .36, .95])
ABOVE = .060
# Stroke (rehearsed in this scene): the hand speeds up to SPEED over ACCEL m, keeps that speed for COAST m
# (meeting the striker LEAD m into it), then stops within STOP s.
STRIKE = dict(speed=.545, accel=.050, coast=.085, lead=.050, stop=.08, yaw=math.radians(-.20))
STRIKE_START = 5.0
END = 14.0


def smooth(u):
    u = min(max(u, 0.), 1.)
    return u ** 3 * (10 - 15 * u + 6 * u * u)


def direction(st=None):
    st = st or STRIKE
    a = AIM + st['yaw']
    return np.array([math.cos(a), math.sin(a), 0.])


def stroke_profile(t, st=None):
    """Hand travel (m) along the stroke t s after it starts."""
    st = st or STRIKE
    v, accel, coast, stop = st['speed'], st['accel'], st['coast'], st['stop']
    t_acc = 2 * accel / v
    if t <= t_acc:
        u = t / t_acc
        return v * t_acc * (u ** 4 * 2.5 - u ** 5 * 3 + u ** 6)
    t_coast = coast / v
    if t <= t_acc + t_coast:
        return accel + v * (t - t_acc)
    s1 = accel + coast
    if t <= t_acc + t_coast + stop:
        u = (t - t_acc - t_coast) / stop
        return s1 + v * stop * (u - (u ** 4 * 2.5 - u ** 5 * 3 + u ** 6))
    return s1 + v * stop / 2


def run_up_start(st=None):
    st = st or STRIKE
    d = direction(st)
    contact = np.array([*spec.STRIKER_START[:2], spec.BOARD_Z + PAD_CLEAR + TCP_ABOVE_TIPS]) \
        - (spec.STRIKER_R + FACE) * d
    return contact - (st['accel'] + st['lead']) * d


def right_path(t, st=None):
    """Commanded right-hand tool point (m) at time t (s); the fingers stay closed."""
    st = st or STRIKE
    start = run_up_start(st)
    above = start + [0., 0., ABOVE]
    if t < 2.5:
        return above + [-.05, 0., .05] + smooth(t / 2.5) * np.array([.05, 0., -.05])
    if t < 4.0:
        return above + smooth((t - 2.5) / 1.5) * (start - above)
    if t < STRIKE_START:
        return start
    tt = t - STRIKE_START
    p = start + stroke_profile(tt, st) * direction(st)
    v = st['speed']
    done = 2 * st['accel'] / v + st['coast'] / v + st['stop'] + .5
    if tt > done:
        p = p + smooth((tt - done) / 1.5) * np.array([-.05, 0., .10])
    return p


def tool(st=None):
    """Fingers down, closing along the stroke."""
    x = direction(st)
    z = np.array([0., 0., -1.])
    return np.column_stack([x, np.cross(z, x), z])


DOWN = np.diag([-1., 1., -1.])


def plan(ik, constants, station, st=None):
    steps = int(END * spec.CONTROL_HZ)
    q_left, ep, er = ik.solve(PARK_LEFT, ik.matrix_to_quat_xyzw(DOWN), constants.READY, station.BASES['left'],
                              iters=400, damping=.015)
    errors = [{'time': 0., 'side': 'left', 'position_m': float(ep), 'rotation_rad': float(er)}]
    f_left = spec.finger_joint_position(.045)
    left = [*q_left, f_left, f_left]
    result = []
    quat = ik.matrix_to_quat_xyzw(tool(st))
    q = np.array(constants.READY, float)
    closed = spec.finger_joint_position(0.)
    for k in range(steps + 1):
        t = k / spec.CONTROL_HZ
        p = right_path(t, st)
        q, ep, er = ik.solve(p, quat, q, station.BASES['right'], iters=400 if k == 0 else 80, damping=.015)
        errors.append({'time': t, 'side': 'right', 'position_m': float(ep), 'rotation_rad': float(er),
                       'target': p.tolist()})
        result.append((k, {'left': list(left), 'right': [*q, closed, closed]}))
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
