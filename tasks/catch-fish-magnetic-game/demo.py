"""Fixed-layout RGB-only rehearsal; offline IK, no live object-pose feedback.

The right hand pinches the rod's handle where it overhangs the game box (fingers pointing down, closing
side to side), lifts the rod level, carries the dangling magnet over the red fish's washer and waits for
the swing to die down, lowers it onto the washer, then lifts the fish well clear of the pond.
The left arm stays parked.
"""
import math
import numpy as np
import spec

OPEN = .040
SQUEEZE = .002
PINCH_FROM_END = .015
PINCH = np.array([spec.ROD_START[0] - spec.ROD_LEN / 2 + PINCH_FROM_END, spec.ROD_START[1], spec.ROD_START[2]])
PINCH_TO_TIP = spec.ROD_LEN - PINCH_FROM_END
PARK_LEFT = np.array([.18, .36, .95])
WASHER = np.array(spec.washer_start(spec.TARGET))
HANG = spec.CORD_LEN + spec.MAGNET_T             # rod tip to the magnet's face, cord taut
HOVER_Z = WASHER[2] + HANG + .060
TOUCH_Z = WASHER[2] + HANG - .012
LIFT_Z = .920
# Times (s).
GRASP_T, LIFT_T, CARRY_T, SETTLE_T, LOWER_T, WAIT_T, RAISE_T = 4.0, 5.0, 6.5, 9.5, 11.5, 13.0, 14.0
END = 19.0


def smooth(u):
    u = min(max(u, 0.), 1.)
    return u ** 3 * (10 - 15 * u + 6 * u * u)


def over(z):
    """Pinch point that puts the rod's tip straight above the washer at height z."""
    return np.array([WASHER[0] - PINCH_TO_TIP, WASHER[1], z])


def right_path(t):
    """Commanded right-hand pinch point (m) and aperture (m) at time t (s)."""
    above = PINCH + [0., 0., .080]
    if t < 2.5:
        return above + [-.03, .10, 0.] + smooth(t / 2.5) * np.array([.03, -.10, 0.]), OPEN
    if t < GRASP_T:
        return above + smooth((t - 2.5) / (GRASP_T - 2.5)) * (PINCH - above), OPEN
    if t < LIFT_T:
        return PINCH, OPEN + smooth(t - GRASP_T) * (SQUEEZE - OPEN)
    lifted = PINCH + [0., 0., .150]
    if t < CARRY_T:
        return PINCH + smooth((t - LIFT_T) / (CARRY_T - LIFT_T)) * (lifted - PINCH), SQUEEZE
    hover = over(HOVER_Z)
    if t < SETTLE_T:
        return lifted + smooth((t - CARRY_T) / (SETTLE_T - CARRY_T)) * (hover - lifted), SQUEEZE
    if t < LOWER_T:
        return hover, SQUEEZE
    touch = over(TOUCH_Z)
    if t < WAIT_T:
        return hover + smooth((t - LOWER_T) / (WAIT_T - LOWER_T)) * (touch - hover), SQUEEZE
    if t < RAISE_T:
        return touch, SQUEEZE
    return touch + smooth((t - RAISE_T) / 2.5) * (over(LIFT_Z) - touch), SQUEEZE


# Fingers point down and close along world y; the rod lies along the hand's -y axis (world +x).
GRIP = np.column_stack([(0., -1., 0.), (-1., 0., 0.), (0., 0., -1.)])
DOWN = np.diag([-1., 1., -1.])


def plan(ik, constants, station):
    steps = int(END * spec.CONTROL_HZ)
    q_left, ep, er = ik.solve(PARK_LEFT, ik.matrix_to_quat_xyzw(DOWN), constants.READY, station.BASES['left'],
                              iters=400, damping=.015)
    errors = [{'time': 0., 'side': 'left', 'position_m': float(ep), 'rotation_rad': float(er)}]
    f_left = spec.finger_joint_position(OPEN)
    left = [*q_left, f_left, f_left]
    result = []
    quat = ik.matrix_to_quat_xyzw(GRIP)
    q, _, _ = ik.solve(right_path(0.)[0], ik.matrix_to_quat_xyzw(DOWN), constants.READY, station.BASES['right'],
                       iters=400, damping=.015)
    q[5] -= math.pi / 2
    for k in range(steps + 1):
        t = k / spec.CONTROL_HZ
        p, gap = right_path(t)
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
