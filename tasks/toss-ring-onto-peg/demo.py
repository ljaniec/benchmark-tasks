"""Fixed-layout RGB-only rehearsal; offline IK, no live object-pose feedback.

The right hand pinches the ring's back rim where it hangs over the side table's edge (fingers pointing
down, closing front-to-back), lifts it, carries it in front of the right shoulder and throws it forward
and up along a straight line, opening at the throw's peak speed and stopping well behind the throw line.
The left arm stays parked.
"""
import math
import numpy as np
import spec

OPEN = .040
SQUEEZE = .002
GRASP = np.array([spec.RING_START[0] - spec.RING_MID_R, spec.RING_START[1], spec.RING_START[2]])
ABOVE = .080
PARK_LEFT = np.array([.18, .36, .95])
# Throw (rehearsed in this scene): the hand starts at START, accelerates along a line ELEV degrees above
# horizontal (yawed by YAW degrees) to SPEED m/s over ACCEL m, then stops within STOP s. The fingers are
# commanded open RELEASE s after peak speed.
THROW = dict(start=(.080, -.305, .870), elev=25., yaw=0., speed=2.40, accel=.150, stop=.06, release=.05)
CARRY_START = 6.0
THROW_START = 10.0
RETREAT = np.array([.200, -.305, .900])
END = 16.0


def smooth(u):
    u = min(max(u, 0.), 1.)
    return u ** 3 * (10 - 15 * u + 6 * u * u)


def throw_profile(t, throw=None):
    """Hand displacement (m) along the throw line t s after the throw starts; and the open time."""
    th = throw or THROW
    v, accel, stop = th['speed'], th['accel'], th['stop']
    t_acc = 2 * accel / v
    if t <= t_acc:
        u = t / t_acc
        # integral of v*smooth(u) over the acceleration
        s = v * t_acc * (u ** 4 * 2.5 - u ** 5 * 3 + u ** 6)
        return s
    s1 = accel
    if t <= t_acc + stop:
        u = (t - t_acc) / stop
        return s1 + v * stop * (u - (u ** 4 * 2.5 - u ** 5 * 3 + u ** 6))
    return s1 + v * stop / 2


def throw_direction(throw=None):
    th = throw or THROW
    e, y = math.radians(th['elev']), math.radians(th['yaw'])
    return np.array([math.cos(e) * math.cos(y), math.cos(e) * math.sin(y), math.sin(e)])


def release_time(throw=None):
    th = throw or THROW
    return 2 * th['accel'] / th['speed'] + th['release']


def right_path(t, throw=None):
    """Commanded right-hand pinch point (m) and aperture (m) at time t (s)."""
    th = throw or THROW
    above = GRASP + [0., 0., ABOVE]
    start = np.array(th['start'])
    if t < 2.5:
        return above + [-.03, .10, 0.] + smooth(t / 2.5) * np.array([.03, -.10, 0.]), OPEN
    if t < 4.0:
        return above + smooth((t - 2.5) / 1.5) * (GRASP - above), OPEN
    if t < 5.0:
        return GRASP, OPEN + smooth(t - 4.) * (SQUEEZE - OPEN)
    lifted = GRASP + [0., 0., .06]
    if t < CARRY_START:
        return GRASP + smooth(t - 5.) * (lifted - GRASP), SQUEEZE
    if t < THROW_START:
        return lifted + smooth((t - CARRY_START) / (THROW_START - 1. - CARRY_START)) * (start - lifted), SQUEEZE
    tt = t - THROW_START
    p = start + throw_profile(tt, th) * throw_direction(th)
    settle = 2 * th['accel'] / th['speed'] + th['stop'] + .5
    if tt > settle:
        p = p + smooth((tt - settle) / 1.5) * (RETREAT - p)
    return p, (OPEN if tt >= release_time(th) else SQUEEZE)


DOWN = np.diag([-1., 1., -1.])


def plan(ik, constants, station, throw=None):
    steps = int(END * spec.CONTROL_HZ)
    q_left, ep, er = ik.solve(PARK_LEFT, ik.matrix_to_quat_xyzw(DOWN), constants.READY, station.BASES['left'],
                              iters=400, damping=.015)
    errors = [{'time': 0., 'side': 'left', 'position_m': float(ep), 'rotation_rad': float(er)}]
    f_left = spec.finger_joint_position(OPEN)
    left = [*q_left, f_left, f_left]
    result = []
    quat = ik.matrix_to_quat_xyzw(DOWN)
    q = np.array(constants.READY, float)
    for k in range(steps + 1):
        t = k / spec.CONTROL_HZ
        p, gap = right_path(t, throw)
        q, ep, er = ik.solve(p, quat, q, station.BASES['right'], iters=400 if k == 0 else 80, damping=.015)
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
