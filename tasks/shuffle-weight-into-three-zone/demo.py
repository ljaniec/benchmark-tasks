"""Fixed-layout RGB-only rehearsal; offline IK, no live object-pose feedback.

The right hand comes down over the weight where it stands at the near end and takes it by its sides (fingers
pointing down, closing across the board). Keeping it on the board, the hand slides it straight down the centre
line, speeding up to SPEED m/s over ACCEL m, opens at that speed, coasts a moment, stops, and then lifts away.
The left arm stays parked beside the table.
"""
import numpy as np
import spec

OPEN = .072
SQUEEZE = 0.
PAD_CLEAR = .002                 # pad tips above the board
TCP_ABOVE_TIPS = .013
GRASP = np.array([spec.WEIGHT_START[0], spec.WEIGHT_START[1], spec.BOARD_Z + PAD_CLEAR + TCP_ABOVE_TIPS])
ABOVE = .080
PARK_LEFT = np.array([.18, .36, .95])
# Shot (rehearsed in this scene): from the grasp, the hand speeds up along +x to SPEED over ACCEL m, keeps that
# speed for COAST s, then stops within STOP s. The fingers are commanded open RELEASE s after full speed.
SHOT = dict(speed=1.225, accel=.090, coast=.06, stop=.05, release=.03)
SHOT_START = 6.0
END = 20.0


def smooth(u):
    u = min(max(u, 0.), 1.)
    return u ** 3 * (10 - 15 * u + 6 * u * u)


def shot_profile(t, shot=None):
    """Hand travel (m) along +x t s after the shot starts."""
    sh = shot or SHOT
    v, accel, coast, stop = sh['speed'], sh['accel'], sh['coast'], sh['stop']
    t_acc = 2 * accel / v
    if t <= t_acc:
        u = t / t_acc
        return v * t_acc * (u ** 4 * 2.5 - u ** 5 * 3 + u ** 6)
    if t <= t_acc + coast:
        return accel + v * (t - t_acc)
    s1 = accel + v * coast
    if t <= t_acc + coast + stop:
        u = (t - t_acc - coast) / stop
        return s1 + v * stop * (u - (u ** 4 * 2.5 - u ** 5 * 3 + u ** 6))
    return s1 + v * stop / 2


def release_time(shot=None):
    sh = shot or SHOT
    return 2 * sh['accel'] / sh['speed'] + sh['release']


def right_path(t, shot=None):
    """Commanded right-hand tool point (m) and aperture (m) at time t (s)."""
    sh = shot or SHOT
    above = GRASP + [0., 0., ABOVE]
    if t < 2.5:
        return above + [-.06, 0., .04] + smooth(t / 2.5) * np.array([.06, 0., -.04]), OPEN
    if t < 4.0:
        return above + smooth((t - 2.5) / 1.5) * (GRASP - above), OPEN
    if t < 5.0:
        return GRASP, OPEN + smooth(t - 4.) * (SQUEEZE - OPEN)
    if t < SHOT_START:
        return GRASP, SQUEEZE
    tt = t - SHOT_START
    p = GRASP + [shot_profile(tt, sh), 0., 0.]
    done = 2 * sh['accel'] / sh['speed'] + sh['coast'] + sh['stop'] + .5
    if tt > done:
        p = p + smooth((tt - done) / 1.5) * np.array([-.10, 0., .12])
    return p, (OPEN if tt >= release_time(sh) else SQUEEZE)


# Fingers down, closing along y (across the board).
TOOL = np.column_stack([[0., -1., 0.], [-1., 0., 0.], [0., 0., -1.]])
DOWN = np.diag([-1., 1., -1.])


def plan(ik, constants, station, shot=None):
    steps = int(END * spec.CONTROL_HZ)
    q_left, ep, er = ik.solve(PARK_LEFT, ik.matrix_to_quat_xyzw(DOWN), constants.READY, station.BASES['left'],
                              iters=400, damping=.015)
    errors = [{'time': 0., 'side': 'left', 'position_m': float(ep), 'rotation_rad': float(er)}]
    f_left = spec.finger_joint_position(.045)
    left = [*q_left, f_left, f_left]
    result = []
    quat = ik.matrix_to_quat_xyzw(TOOL)
    q = np.array(constants.READY, float)
    for k in range(steps + 1):
        t = k / spec.CONTROL_HZ
        p, gap = right_path(t, shot)
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
