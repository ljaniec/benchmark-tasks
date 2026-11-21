"""Fixed robot trajectory. Targets come from the authored fixture, with no object-pose reads."""
import math

import numpy as np
from yam_arena import demo as planner
from yam_arena.urdf import axis_angle_matrix

from geometry import (BASE, KEY_FRAME, KEY_HANDLE_Y, KEY_REST_ROTATION, KEY_START, KEY_TIP_Y,
                      MOUNT_ROTATION, PANEL, PIVOT_Z, REACH, REST_LIFT_DEG, SOCKET_CENTER_Y, SOCKET_DEPTH)


OPEN = -.034
CLOSED = 0.


class GripOrientation(float):
    def __new__(cls, rotation):
        value = float.__new__(cls, 0.)
        value.rotation = rotation
        return value


def install_orientation():
    if getattr(planner.grip_rotation, "uses_explicit_rotation", False):
        return
    stock = planner.grip_rotation

    def convert(yaw=0.):
        if isinstance(yaw, GripOrientation):
            return yaw.rotation
        return stock(yaw)

    convert.uses_explicit_rotation = True
    planner.grip_rotation = convert


def rigid_pose(duration, xyz, finger, description, rotation):
    return planner.Waypoint(duration, tuple(xyz), finger, description, GripOrientation(rotation))


def world(local):
    return np.asarray(BASE) + MOUNT_ROTATION @ np.asarray(local)


def motion():
    install_orientation()
    down = KEY_REST_ROTATION @ KEY_FRAME
    shaft = MOUNT_ROTATION[:, 1]
    radial = MOUNT_ROTATION[:, 0]
    # Fixed offset measured with the key at the socket on this replica.
    socket_alignment = radial * .0012 + np.array([0., 0., .0076])
    key_handle = KEY_START + KEY_REST_ROTATION @ np.array([0., KEY_HANDLE_Y, 0.])
    above_key = key_handle + np.array([0., 0., .081])
    return_offset = np.array([.009, 0., 0.])
    rest_right = (.205, -.306, .941)
    socket_depth = .0115
    socket_mouth = SOCKET_CENTER_Y - SOCKET_DEPTH / 2.
    seated_y = socket_mouth + socket_depth - (KEY_TIP_Y - KEY_HANDLE_Y)
    seated = world((0., seated_y, PIVOT_Z)) + socket_alignment
    just_outside = seated - shaft * (.013 + socket_depth)
    approach = seated - shaft * .067

    def wrench_rotation(degrees):
        pitch = axis_angle_matrix(radial, math.radians(.8))
        roll = axis_angle_matrix(shaft, math.radians(degrees + .6))
        return pitch @ roll @ MOUNT_ROTATION @ KEY_FRAME

    start = wrench_rotation(-60.)
    end = wrench_rotation(82.)
    right = [
        rigid_pose(1.45, above_key, OPEN, "approach hex key", down),
        rigid_pose(1.15, key_handle, OPEN, "align pads with grip", down),
        rigid_pose(.65, key_handle, CLOSED, "grasp hex key", down),
        rigid_pose(1.05, above_key, CLOSED, "lift the key from the holder", down),
        rigid_pose(1.15, approach, CLOSED, "align the bit with the socket", start),
        rigid_pose(1.20, just_outside, CLOSED, "center the hex bit", start),
        rigid_pose(1.65, seated, CLOSED, "insert the hex bit", start),
        rigid_pose(.35, seated, CLOSED, "seat the wrench", start),
    ]
    for fraction in np.linspace(1. / 6., 1., 6):
        right.append(rigid_pose(.72, seated, CLOSED, "turn the counterbalance adjuster",
                                wrench_rotation(-60. + 142. * fraction)))
    right.extend([
        rigid_pose(.28, seated, CLOSED, "finish the adjustment", end),
        rigid_pose(1.10, approach, CLOSED, "withdraw the key", end),
        rigid_pose(.85, approach, CLOSED, "turn the withdrawn key clear of the hardware", start),
        rigid_pose(1.45, above_key, CLOSED, "return the upright key", down),
        rigid_pose(1.20, key_handle + return_offset, CLOSED, "return key into holder", down),
        rigid_pose(.55, key_handle + return_offset, OPEN, "release the key", down),
        rigid_pose(1.15, rest_right, OPEN, "park right hand clear", down),
    ])

    close = -shaft
    fixed_frame = np.column_stack((close, np.cross(radial, close), radial))
    bezel_rotation = axis_angle_matrix(close, math.radians(-25.)) @ fixed_frame
    parked_left = (.212, .337, 1.015)
    mid = 0.
    low, high = -12., 12.

    def bezel(lift, outside=0.):
        angle = math.radians(lift)
        return world((-PANEL[0] / 2. - .004 - outside, REACH * math.cos(angle),
                      PIVOT_Z + REACH * math.sin(angle) + .010))

    def left_pose(duration, xyz, finger, description):
        return rigid_pose(duration, xyz, finger, description, bezel_rotation)

    left = [
        left_pose(1.45, bezel(REST_LIFT_DEG, .050), OPEN, "approach the side bezel"),
        left_pose(.85, bezel(REST_LIFT_DEG), OPEN, "surround the side bezel"),
        left_pose(.65, bezel(REST_LIFT_DEG), CLOSED, "hold the bezel clear of the glass"),
    ]
    for angle in (-14., -7., mid):
        left.append(left_pose(.58, bezel(angle), CLOSED, "support display nearer horizontal"))
    screw_done_s = sum(w.duration for w in right[:15])
    hold_s = screw_done_s - sum(w.duration for w in left) + .15
    left.append(left_pose(hold_s, bezel(mid), CLOSED, "counterhold display while the key turns"))
    for angle in (-6., low):
        left.append(left_pose(.56, bezel(angle), CLOSED, "position the display at the lower test height"))
    left.extend([
        left_pose(.5, bezel(low), OPEN, "release the balanced display"),
        left_pose(.65, bezel(low, .058), OPEN, "clear the left bezel"),
        left_pose(.85, parked_left, OPEN, "park for the first released hold"),
    ])
    wait_s = max(sum(w.duration for w in right) - sum(w.duration for w in left), 0.) + 2.4
    left.extend([
        left_pose(wait_s, parked_left, OPEN, "leave the display unsupported"),
        left_pose(1.0, bezel(low, .05), OPEN, "reapproach the lower bezel"),
        left_pose(.8, bezel(low), OPEN, "align with the bezel"),
        left_pose(.58, bezel(low), CLOSED, "grasp for repositioning"),
    ])
    for angle in (-6., mid, 6., high):
        left.append(left_pose(.57, bezel(angle), CLOSED, "raise the balanced arm"))
    left.extend([
        left_pose(.70, bezel(high), CLOSED, "steady the upper height"),
        left_pose(.75, bezel(high), OPEN, "release the upper height"),
        left_pose(.75, bezel(high, .058), OPEN, "clear the screen"),
        left_pose(.85, parked_left, OPEN, "park for the second released hold"),
        left_pose(2.5, parked_left, OPEN, "leave the display unsupported"),
    ])
    return {"left": left, "right": right}
