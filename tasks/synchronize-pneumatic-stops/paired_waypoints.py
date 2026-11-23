"""A fixed-layout path using the physical button and push-lock control."""
import numpy as np

from flow_estimate import ADJUSTMENT_DEG
from pneumatic_fixture import BUTTON, CAP_LIFT, KNOB, ORIGIN, YAW
from yam_arena import demo as motion
from yam_arena.constants import FINGER_CLOSED


def demo():
    p = motion.Waypoint
    approach_height = np.array((0., 0., .057))
    press_point = np.array((BUTTON[0], BUTTON[1], .099))
    button = local_to_world(press_point)
    hover_button = button + approach_height
    cap = local_to_world(np.array(KNOB) + (0., 0., .013))
    hover_cap = cap + (0., 0., .048)
    raised = cap + (0., 0., CAP_LIFT)
    # The stock downward wrist has a -90 degree heading in the station.
    start, midpoint, stop = np.radians([YAW - 150. + value for value in (0., ADJUSTMENT_DEG / 2., ADJUSTMENT_DEG)])
    dial_opening = -.014
    button_yaw = np.radians(YAW)

    left = [
        p(1.4, hover_cap, dial_opening, "approach the speed controller", start),
        p(3.3, hover_cap, dial_opening, "hold above the controller during the first diagnostic cycle", start),
        p(.65, cap, dial_opening, "approach the top of the exposed cap", start),
        p(.4, cap, FINGER_CLOSED, "grip the cap above its collar", start),
        p(.55, raised, FINGER_CLOSED, "pull the cap out of its lock", start),
        p(1.5, raised, FINGER_CLOSED, "open the speed control counterclockwise", midpoint),
        p(1.5, raised, FINGER_CLOSED, "finish the adjustment", stop),
        p(.45, cap, FINGER_CLOSED, "bring the adjusted cap down toward the lock", stop),
        p(.35, cap, dial_opening, "release the side grip", stop),
        p(.40, cap + (0., 0., .012), FINGER_CLOSED, "approach directly above the cap", stop),
        p(.45, cap + (0., 0., .001), FINGER_CLOSED, "press the cap into the lock from above", stop),
        p(.55, hover_cap, FINGER_CLOSED, "withdraw from the controller", stop),
    ]
    right = [
        p(1.3, hover_button, FINGER_CLOSED, "approach the test button", button_yaw),
        p(.65, button, FINGER_CLOSED, "start the diagnostic cycle", button_yaw),
        p(.6, hover_button, FINGER_CLOSED, "release the button", button_yaw),
        p(8.60, hover_button, FINGER_CLOSED, "wait until the speed control is relocked", button_yaw),
        p(.55, button, FINGER_CLOSED, "start the first verification cycle", button_yaw),
        p(.6, hover_button, FINGER_CLOSED, "release the verification button", button_yaw),
        p(2.8, hover_button, FINGER_CLOSED, "let both stops return", button_yaw),
        p(.55, button, FINGER_CLOSED, "start the second verification cycle", button_yaw),
        p(.6, hover_button, FINGER_CLOSED, "release the verification button again", button_yaw),
    ]
    return {"left": left, "right": right}


def local_to_world(point):
    angle = np.radians(YAW)
    x, y, z = point
    dx = np.cos(angle) * x - np.sin(angle) * y
    dy = np.sin(angle) * x + np.cos(angle) * y
    return np.asarray(ORIGIN) + (dx, dy, z)
