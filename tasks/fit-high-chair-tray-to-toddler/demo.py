"""Fixed-layout RGB-only rehearsal; offline IK, no live object-pose feedback."""
import math
import numpy as np
import spec

PITCH = math.radians(20)
YAW = math.radians(22)
# Pinch centre in tray coordinates: 4 mm under the button and 11 mm over the plate when open,
# so closing lifts the button before the upper pad lands on the plate.
PINCH_LOCAL = np.array([-.1455, -.005, -.0055])
OPEN = .061
SQUEEZE = .030
# The fingers close along an axis tilted PITCH forward, and pad friction exceeds tan(PITCH), so a
# lower pad rising under the button would drag the tray forward. Backing the hand off by the pad's
# horizontal travel while closing makes the lower pad rise straight up.
SQUEEZE_BACKOFF = (OPEN - SQUEEZE) / 2 * math.sin(PITCH)
# Rehearsed: arrival, squeeze and release contacts leave the tray 3 mm past the commanded pinch,
# and a finger only enters its slot within +/-3 mm of the slot centre.
SETTLE_DRIFT = .003
PARK_LEFT = np.array([.18, .36, .95])


def orientation(pitch, yaw):
    a = np.array([math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw), -math.sin(pitch)])
    x = np.array([math.sin(pitch) * math.cos(yaw), math.sin(pitch) * math.sin(yaw), math.cos(pitch)])
    return np.column_stack([x, np.cross(a, x), a])


def pinch_point(tray_x):
    return np.array([tray_x, 0., spec.TRAY_Z]) + PINCH_LOCAL


def plan(ik, constants, station):
    approach = np.array([math.cos(YAW), math.sin(YAW), 0.])
    backoff = SQUEEZE_BACKOFF * approach
    start = pinch_point(spec.TRAY_START_X)
    target = spec.POSITION_X[spec.TARGET_POSITION - 1] - SETTLE_DRIFT
    rows = [
        (0.0, start - .07 * approach + [0, -.04, .05], OPEN),
        (1.5, start - .05 * approach, OPEN),
        (3.0, start, OPEN),
        (4.0, start - backoff, SQUEEZE),
        (5.0, start - backoff, SQUEEZE),
    ]
    travel = target - spec.TRAY_START_X
    steps = int(round(travel / .01))
    for i in range(1, steps + 1):
        rows.append((5.0 + 3.0 * i / steps, pinch_point(spec.TRAY_START_X + travel * i / steps) - backoff, SQUEEZE))
    end = pinch_point(target)
    rows += [
        (9.0, end - backoff, SQUEEZE),
        (10.0, end, OPEN),
        (11.0, end, OPEN),
        (12.5, end - .07 * approach, OPEN),
        (14.5, end - .09 * approach + [0, -.06, .06], OPEN),
        (16.0, end - .09 * approach + [0, -.06, .06], OPEN),
    ]
    q = {s: np.array(constants.READY) for s in station.BASES}
    down_left = np.diag([-1., 1., -1.])
    result, errors = [], []
    rng = np.random.default_rng(19)
    for sec, right_xyz, gap in rows:
        poses = {}
        for side, p, R, g in [('left', PARK_LEFT, down_left, OPEN), ('right', right_xyz, orientation(PITCH, YAW), gap)]:
            p = np.asarray(p, dtype=float)
            candidates = []
            for seed in [q[side], constants.READY, *[rng.uniform(ik.LOWER, ik.UPPER) for _ in range(20)]]:
                trial = ik.solve(p, ik.matrix_to_quat_xyzw(R), seed, station.BASES[side], iters=400, damping=.015)
                candidates.append(trial)
                if trial[1] < .001 and trial[2] < .01:
                    break
            q[side], ep, er = min(candidates, key=lambda v: v[1] + .25 * v[2])
            errors.append({'time': sec, 'side': side, 'position_m': float(ep), 'rotation_rad': float(er),
                           'target': p.tolist()})
            f = spec.finger_joint_position(g)
            poses[side] = [*q[side], f, f]
        result.append((int(round(sec * 20)), poses))
    bad = [e for e in errors if e['position_m'] > .004 or e['rotation_rad'] > .05]
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
        for (a, p), (b, q) in zip(self.plan[:-1], self.plan[1:]):
            if step <= b:
                u = np.clip((step - a) / max(1, b - a), 0, 1)
                u = u * u * (3 - 2 * u)
                return {s: (1 - u) * np.array(p[s]) + u * np.array(q[s]) for s in p}
        return {s: np.asarray(v) for s, v in self.plan[-1][1].items()}
