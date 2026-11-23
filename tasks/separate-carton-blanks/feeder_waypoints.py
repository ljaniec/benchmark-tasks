"""Use the exposed separator wing and the proof button in the fixed scene."""
import numpy as np

from mounted_feeder_fixture import ADJUSTMENT_DEG, BUTTON, KNOB, ORIGIN
from yam_arena.constants import FINGER_CLOSED
from yam_arena.demo import Waypoint


START_YAW_DEG = -84.
KNOB_TCP_Z_M = .016
PRESS_TCP_Z_M = .019


def demo(start_yaw_deg=START_YAW_DEG):
    knob = np.asarray(ORIGIN) + KNOB + (0., 0., KNOB_TCP_Z_M)
    above_knob = knob + (0., 0., .047)
    button = np.asarray(ORIGIN) + BUTTON + (0., 0., PRESS_TCP_Z_M)
    above_button = button + (0., 0., .045)
    start, center, end = np.radians(start_yaw_deg + np.array((0., ADJUSTMENT_DEG / 2., ADJUSTMENT_DEG)))
    return {
        "left": [
            Waypoint(1.35, above_button, FINGER_CLOSED, "approach the proof control"),
            Waypoint(5.2, above_button, FINGER_CLOSED, "wait for the separator adjustment"),
            Waypoint(.55, button, FINGER_CLOSED, "start the feeder proof cycle"),
            Waypoint(.40, button, FINGER_CLOSED, "press the proof button"),
            Waypoint(.55, above_button, FINGER_CLOSED, "withdraw from the proof control"),
            Waypoint(6.5, above_button, FINGER_CLOSED, "watch both blanks exit"),
        ],
        "right": [
            Waypoint(1.35, above_knob, -.022, "approach the separator adjuster", start),
            Waypoint(.65, knob, -.022, "straddle the rectangular wing", start),
            Waypoint(.5, knob, FINGER_CLOSED, "grip the wing", start),
            Waypoint(1.45, knob, FINGER_CLOSED, "turn the adjustment screw", center),
            Waypoint(1.45, knob, FINGER_CLOSED, "finish the gap adjustment", end),
            Waypoint(.45, knob, -.021, "release the adjuster", end),
            Waypoint(.58, above_knob, -.021, "retreat outside the guard", end),
            Waypoint(8., above_knob, -.021, "watch the proof cycle", end),
        ],
    }
