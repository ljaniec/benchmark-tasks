"""Physical web geometry and observable towel-feed scoring."""

from dataclasses import dataclass
import math

import numpy as np


WIDTH = .2032
ROLL_CENTER = np.array([.461, 0., 1.065])
ROLL_RADIUS = .075
ROLLER_RADIUS = .0198
NIP_X = .428
NIP_Z = .934
OUTLET_Z = .820
OUTLET_X = (.399, .438)
WEB_THICKNESS = .0014
LEADER_TIP_X = .311
LEADER_TIP_Z = .9815
CONTROL_DT = .05
FEED_PER_LEVER_RAD = .084
BACKLASH = math.radians(2.)
REARM = math.radians(1.)


def reset_path():
    """Web centreline from its attachment on the roll to the free edge."""
    angle = np.linspace(.79, math.pi, 14)
    wrap = ROLL_CENTER + np.column_stack((
        ROLL_RADIUS * np.cos(angle),
        np.zeros(len(angle)),
        ROLL_RADIUS * np.sin(angle),
    ))
    leader = np.array([
        [.384, 0., 1.047],
        [.379, 0., 1.028],
        [.372, 0., 1.010],
        [.366, 0., .992],
        [.347, 0., .984],
        [.333, 0., LEADER_TIP_Z],
        [LEADER_TIP_X, 0., LEADER_TIP_Z],
    ])
    return np.concatenate((wrap, leader))


@dataclass
class Ratchet:
    depth: float = 0.

    def roller_speed(self, lever_angle):
        """Apply the one-way ratio to lever movement, with one stroke per return."""
        if not math.isfinite(lever_angle):
            return 0.
        depressed = -lever_angle
        if depressed <= REARM:
            self.depth = 0.
        advance = max(0., depressed - max(self.depth, BACKLASH))
        self.depth = max(0., depressed, self.depth)
        return -FEED_PER_LEVER_RAD / ROLLER_RADIUS * advance / CONTROL_DT


def crossing(path, height, limits):
    """Does the web pass downward through this opening for its full width?"""
    above, below = path[:-1], path[1:]
    pairs = np.flatnonzero(np.all((above[:, :, 2] >= height) & (below[:, :, 2] < height), axis=1))
    for i in pairs:
        fraction = (above[i, :, 2] - height) / (above[i, :, 2] - below[i, :, 2])
        section = above[i] + fraction[:, None] * (below[i] - above[i])
        if (np.all((section[:, 0] >= limits[0]) & (section[:, 0] <= limits[1]))
                and np.max(np.abs(section[:, 1])) <= .112
                and abs(section[0, 1] - section[1, 1]) >= .195):
            return True
    return False


@dataclass
class FeedScore:
    previous_tip_z: float | None = None
    previous_roller_angle: float | None = None
    previous_idler_angle: float | None = None
    driven_m: float = 0.
    dwell: int = 0
    score: float = 0.
    engaged: bool = False
    exposed: float = 0.
    invalid: bool = False

    def observe(self, path, lever_angle, roller_angle, idler_angle, hands_clear, powered, *, nip_closed=True):
        path = np.asarray(path)
        self.invalid |= (path.ndim != 3 or path.shape[1:] != (2, 3) or len(path) < 2
                         or not np.isfinite(path).all()
                         or not all(map(math.isfinite, (lever_angle, roller_angle, idler_angle))))
        if not self.invalid:
            self.invalid |= bool(
                np.any(path[:, :, 0] < .245) or np.any(path[:, :, 0] > .541)
                or np.any(np.abs(path[:, :, 1]) > .144)
                or np.any(path[:, :, 2] > 1.218) or np.any(path[:, :, 2] < .748)
            )
        if self.invalid:
            self.engaged = False
            self.exposed = 0.
            self.dwell = 0
            self.score = 0.
            return

        tip_z = float(path[-1, :, 2].max())
        self.engaged = nip_closed and crossing(path, NIP_Z, (NIP_X - .003, NIP_X + .003))
        at_outlet = self.engaged and crossing(path, OUTLET_Z, OUTLET_X)
        self.exposed = max(0., OUTLET_Z - tip_z) if at_outlet else 0.
        if not self.engaged:
            self.driven_m = 0.
        elif powered and self.previous_tip_z is not None:
            travel = max(0., self.previous_tip_z - tip_z)
            driven = max(0., self.previous_roller_angle - roller_angle) * ROLLER_RADIUS
            idler = max(0., idler_angle - self.previous_idler_angle) * ROLLER_RADIUS
            # Count only simultaneous film motion and counterrotation of
            # both roller surfaces. A spinning powered roll by itself
            # cannot establish that the interposed paper was advanced.
            self.driven_m += min(travel, driven, idler)
        self.previous_tip_z = tip_z
        self.previous_roller_angle = roller_angle
        self.previous_idler_angle = idler_angle

        square = np.ptp(path[-1, :, 2]) <= .008
        released = hands_clear and abs(lever_angle) <= math.radians(3.)
        full = self.exposed >= .050 and self.driven_m >= .035 and released and square
        self.dwell = self.dwell + 1 if full else 0
        self.score = 0.
        if self.engaged and square:
            self.score = .25
        if self.exposed >= .015 and self.driven_m >= .010 and square:
            self.score = .5
        if self.dwell >= 10:
            self.score = 1.
