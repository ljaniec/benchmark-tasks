"""Fixed-layout RGB-only rehearsal; offline IK, no live object-pose feedback.

The right hand pinches the handle from the left side (fingers closing front-to-back), lifts the toy out
of its sleeve, carries it to the right of the stand, lets the ball settle and swings it up in the
front-to-back plane, then holds the cup still. The left arm stays parked.
"""
import math
import numpy as np
import spec

OPEN = .060
SQUEEZE = .012
GRASP_Z = spec.SLEEVE_BOTTOM_Z + spec.SLEEVE_HEIGHT + .046      # middle of the exposed handle
GRASP = np.array([spec.STAND_X, spec.STAND_Y, GRASP_Z])
LIFT = .090
PLAY = np.array([.330, -.560, .980])
PARK_LEFT = np.array([.18, .36, .95])
UP = np.array([0., 0., 1.])
# Small-swing period of the ball on its string (knot to ball centre).
PERIOD = 2 * math.pi * math.sqrt((spec.STRING_LEN + spec.BALL_R) / 9.81)
CARRY_START = 6.5
CARRY = 2 * PERIOD                   # a raised-cosine speed profile two periods long leaves no residual swing
SWING_START = 10.5
# Swing (rehearsed offline on a pendulum model, then checked in this scene): hand offsets from PLAY
# along x and z. Pumping at the swing frequency (x) and twice it (z) for CYCLES periods, then straight
# segments through six via points (x, z) reached after the listed intervals, then a hold at the last one.
PUMP = dict(ax=.099417, phase=1.615801, az=.060859, zphase=-.799549, cycles=2.360977)
VIAS = [(.026584, .050399), (.019818, -.005966), (-.116298, -.034953),
        (-.018509, .050083), (-.024501, .006183), (.053836, -.033493)]
INTERVALS = [.206893, .12, .258301, .252710, .407715, .127253]
END = 30.0


def smooth(u):
    u = min(max(u, 0.), 1.)
    return u ** 3 * (10 - 15 * u + 6 * u * u)


def cosine_carry(u):
    u = min(max(u, 0.), 1.)
    return u - math.sin(2 * math.pi * u) / (2 * math.pi)


def swing_offset(t, pump=None, vias=None, intervals=None):
    """Commanded (x, z) hand offset from PLAY, t seconds after the swing starts."""
    pump, vias, intervals = pump or PUMP, vias or VIAS, intervals or INTERVALS
    ax, phase, az, zphase = pump['ax'], pump['phase'], pump['az'], pump['zphase']
    t1 = max(.5, pump['cycles']) * PERIOD
    if t <= t1:
        ramp = min(1., t / PERIOD)
        x = ax * ramp * math.sin(2 * math.pi * t / PERIOD + phase) - ax * ramp * math.sin(phase) * (1 - ramp)
        z = az * ramp * math.sin(4 * math.pi * t / PERIOD + zphase)
        return x, z
    times = t1 + np.cumsum(np.clip(intervals, .12, .8))
    start = (ax * math.sin(2 * math.pi * t1 / PERIOD + phase), az * math.sin(4 * math.pi * t1 / PERIOD + zphase))
    ts = np.r_[t1, times]
    pts = np.vstack([start, vias])
    return float(np.interp(t, ts, pts[:, 0])), float(np.interp(t, ts, pts[:, 1]))


def right_path(t):
    """Commanded right-hand pinch point (m) and aperture (m) at time t (s)."""
    side = np.array([0., .1, .05])
    pre = GRASP + [0., .06, 0.]
    if t < 2.0:
        return GRASP + side + smooth(t / 2.) * (pre - GRASP - side), OPEN
    if t < 3.5:
        return pre + smooth((t - 2.) / 1.5) * (GRASP - pre), OPEN
    if t < 5.0:
        return GRASP, OPEN + smooth((t - 3.5) / 1.) * (SQUEEZE - OPEN)
    lifted = GRASP + LIFT * UP
    if t < CARRY_START:
        return GRASP + smooth((t - 5.) / (CARRY_START - 5.)) * LIFT * UP, SQUEEZE
    if t < SWING_START:
        return lifted + cosine_carry((t - CARRY_START) / CARRY) * (PLAY - lifted), SQUEEZE
    dx, dz = swing_offset(t - SWING_START)
    return PLAY + [dx, 0., dz], SQUEEZE


def vertical_ik(ik, p, seed, base, iters=60, damping=.02, weight=.3):
    """Joint angles placing the pinch point at p with the tool's y axis (the held handle) vertical; yaw free."""
    def error(q):
        pos, R = ik.tcp_pose(q, base)
        return np.concatenate([p - pos, weight * (R[:, 1] - UP)])
    q = np.array(seed, float)
    for _ in range(iters):
        e = error(q)
        if np.linalg.norm(e) < 1e-6:
            break
        J = np.zeros((6, 6))
        for i in range(6):
            dq = q.copy()
            dq[i] += 1e-6
            J[:, i] = (error(dq) - e) / -1e-6
        q = np.clip(q + J.T @ np.linalg.solve(J @ J.T + damping ** 2 * np.eye(6), e), ik.LOWER, ik.UPPER)
    pos, R = ik.tcp_pose(q, base)
    return q, float(np.linalg.norm(p - pos)), float(np.arccos(np.clip(R[:, 1] @ UP, -1., 1.)))


# Continuation seed for the handle-vertical solution family that pinches from the left (rehearsed).
SEED = [.05, 2.46, 2.17, -1.06, 1.57, -1.79]


def plan(ik, constants, station):
    steps = int(END * spec.CONTROL_HZ)
    down_left = np.diag([-1., 1., -1.])
    q_left, ep, er = ik.solve(PARK_LEFT, ik.matrix_to_quat_xyzw(down_left), constants.READY, station.BASES['left'],
                              iters=400, damping=.015)
    errors = [{'time': 0., 'side': 'left', 'position_m': float(ep), 'rotation_rad': float(er)}]
    f_left = spec.finger_joint_position(OPEN)
    left = [*q_left, f_left, f_left]
    result = []
    q = np.array(SEED, float)
    for k in range(steps + 1):
        t = k / spec.CONTROL_HZ
        p, gap = right_path(t)
        q, ep, er = vertical_ik(ik, p, q, station.BASES['right'], iters=200 if k == 0 else 60)
        errors.append({'time': t, 'side': 'right', 'position_m': ep, 'rotation_rad': er, 'target': p.tolist()})
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
