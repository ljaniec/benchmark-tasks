"""Convert the measured fixture trajectories to ordinary Inspect joint targets."""

import numpy as np
from yam_arena.constants import FINGER_OPEN


def _target(waypoint):
    grip = waypoint['finger_m'] / FINGER_OPEN
    if not 0. <= grip <= 1.:
        raise ValueError('The aperture is outside the Inspect action range')
    action = np.asarray([value for side in ('left', 'right')
                         for value in (*waypoint['targets'][side]['joint_rad'], grip)])
    if action.shape != (14,) or not np.isfinite(action).all():
        raise ValueError('Expected six finite joint targets and one gripper target per arm')
    return action


def actions(plan):
    """Yield one 14-element command per tick, based on fixture calibration."""
    previous = _target(plan['waypoints'][0])
    for waypoint in plan['waypoints'][1:]:
        target = _target(waypoint)
        duration = waypoint['steps']
        for tick in range(1, duration + 1):
            phase = tick / duration
            blend = phase * phase * (3. - 2. * phase)
            yield waypoint['phase'], previous + blend * (target - previous)
        previous = target
