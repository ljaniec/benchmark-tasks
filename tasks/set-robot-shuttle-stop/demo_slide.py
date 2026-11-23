"""Fixed rehearsal using only the public arm action interface."""
import math

import numpy as np

import proof_fixture as fixture
import slide_tools as tools
from yam_arena import demo as motion
from yam_arena.constants import FINGER_CLOSED
from yam_arena.station import TABLE_TOP
from yam_arena.urdf import axis_angle_matrix


FRONT = np.array(((0., 0., 1.), (-1., 0., 0.), (0., -1., 0.)))
SIDE = np.array(((1., 0., 0.), (0., 0., 1.), (0., -1., 0.)))


class Wrist(float):
    def __new__(cls, matrix):
        angle = float.__new__(cls, 0.)
        angle.matrix = matrix
        return angle


def route():
    old = motion.grip_rotation
    if not getattr(old, "shuttle_wrist", False):
        def turn(angle=0.):
            return angle.matrix if isinstance(angle, Wrist) else old(angle)
        turn.shuttle_wrist = True
        motion.grip_rotation = turn

    yoke_wrist = Wrist(axis_angle_matrix(tools.CLAMP_SHAFT_VECTOR, math.radians(-65.))
                      @ axis_angle_matrix(np.array((0., 1., 0.)), math.radians(57.)) @ FRONT)
    key_twist = (axis_angle_matrix(tools.CLAMP_SHAFT_VECTOR, math.pi * .5) @ SIDE
                 @ axis_angle_matrix(np.array((1., 0., 0.)), math.radians(-5.)))

    def adjust_wrist(degrees):
        turn = axis_angle_matrix(tools.ADJUST_SHAFT_VECTOR, math.radians(degrees))
        return Wrist(tools.BASIS @ turn @ key_twist)

    wrench_pitch = axis_angle_matrix(np.array((0., 1., 0.)), math.radians(58.))
    wrench_pose = Wrist(wrench_pitch @ FRONT)

    def beam_wrist(degrees):
        return wrench_pose

    pointer_block = tools.point((.034, 0., .028))
    wanted = pointer_block + .02598 * tools.BASIS[:, 0]
    key_center = tools.point(tools.ADJUST_TIP + tools.ADJUST_HANDLE)
    key_out = tools.BASIS[:, 0] * .043
    key_stand = key_center + key_out + tools.KEY_PARK_OFFSET
    beam_start = tools.CLAMP_ROLL
    beam_mid = beam_start - 110.
    beam_far = beam_mid - 47.
    beam_gauge = beam_far - 11.5
    beam_test = beam_gauge - 11.
    beam_probe = beam_test - 5.7
    beam_press = beam_probe - 3.6
    beam_end = beam_press - 4.8
    valve_origin = np.array((*fixture.VALVE_ORIGIN_XY, TABLE_TOP))
    theta = fixture.VALVE_THROW_DEG
    valve_neutral = fixture.lever_tip(valve_origin, 0.)
    valve_wrist = motion.grip_rotation(math.pi / 2.)
    valve_neutral_wrist = Wrist(valve_wrist)
    valve_depth = np.array((0., 0., -.006))

    def valve_pose(degrees):
        swing = axis_angle_matrix(np.array((0., 1., 0.)), math.radians(degrees))
        return fixture.lever_tip(valve_origin, degrees) + swing @ valve_depth, Wrist(swing @ valve_wrist)
    first = tools.wrench_handle(beam_start)
    last = tools.wrench_handle(beam_end)
    first_key, last_key = adjust_wrist(0.), adjust_wrist(math.degrees(tools.ANGLE_ADJUST))
    first_beam, last_beam = beam_wrist(beam_start), beam_wrist(beam_end)
    proof_camera_wrist = Wrist(axis_angle_matrix(tools.CLAMP_SHAFT_VECTOR, math.radians(70.)) @ last_beam.matrix)
    w = motion.Waypoint
    left = [
        w(1.30, pointer_block + (0., .009, .039), -.023, "approach the bolted yoke grip", yoke_wrist),
        w(.60, pointer_block, -.023, "straddle the yoke grip", yoke_wrist),
        w(.48, pointer_block, FINGER_CLOSED, "grip the yoke", yoke_wrist),
        w(.70, wanted, FINGER_CLOSED, "hold the yoke on the reading line", yoke_wrist),
        w(3.55, wanted, FINGER_CLOSED, "hold the yoke while the original screw is set", yoke_wrist),
        w(.40, wanted, -.023, "release the vented yoke", yoke_wrist),
        w(.68, wanted + (.025, .065, .048), -.023, "clear access to the clamp", yoke_wrist),
    ]
    right = [
        w(1.35, key_center + key_out, -.021, "approach the original adjustment key", first_key),
        w(.68, key_center, -.021, "straddle the flat key handle", first_key),
        w(.50, key_center, FINGER_CLOSED, "grip the adjustment key", first_key),
        w(.64, key_center, FINGER_CLOSED, "let the other arm position the yoke", first_key),
        w(3.00, key_center, FINGER_CLOSED, "turn the original sleeve to contact", last_key),
        w(.72, key_center + key_out, FINGER_CLOSED, "withdraw the adjusting bit", last_key),
        w(.70, key_stand, FINGER_CLOSED, "lay the adjusting tool aside", last_key),
        w(.30, key_stand, -.021, "release the adjusting tool", last_key),
        w(.54, np.array((.467, -.155, TABLE_TOP + .162)), -.022, "clear the tools before changing wrist pose", first_beam),
        w(.89, first + (.012, -.019, .026), -.035, "approach the indicating wrench", first_beam),
        w(.65, first, -.035, "straddle the wrench handle", first_beam),
        w(.60, first, FINGER_CLOSED, "grip the wrench handle", first_beam),
    ]
    beam_path = np.r_[np.linspace(beam_start, beam_mid, 12)[1:], np.linspace(beam_mid, beam_far, 6)[1:],
                       np.linspace(beam_far, beam_gauge, 4)[1:],
                       np.linspace(beam_gauge, beam_test, 4)[1:],
                       np.linspace(beam_test, beam_probe, 4)[1:]]
    for angle in beam_path:
        right.append(w(.18, tools.wrench_handle(float(angle)), FINGER_CLOSED,
                       "turn the actual clamp socket", beam_wrist(float(angle))))
    right += [
        w(.36, tools.wrench_handle(beam_press), FINGER_CLOSED, "increase pressure on the seated beam", last_beam),
        w(.48, last, FINGER_CLOSED, "settle the clamp at the indicated torque", last_beam),
        w(.85, last, FINGER_CLOSED, "hold the calibrated reading", last_beam),
        w(.59, last + (0., 0., .025), FINGER_CLOSED, "pull the torque bit clear", last_beam),
        w(.70, np.array((.390, -.035, TABLE_TOP + .260)), FINGER_CLOSED,
          "hold the removed wrench and frame the proof dial", proof_camera_wrist),
    ]
    waiting = sum(x.duration for x in right) - sum(x.duration for x in left) + .80
    left += [
        w(waiting, wanted + (.025, .065, .048), -.023, "wait for both tools to be clear", yoke_wrist),
        w(1.03, valve_neutral + (0., 0., .050), -.032, "approach the exhaust-center valve", valve_neutral_wrist),
        w(.59, valve_neutral + valve_depth, -.032, "straddle the manual valve lever", valve_neutral_wrist),
        w(.42, valve_neutral + valve_depth, FINGER_CLOSED, "take hold of the valve", valve_neutral_wrist),
    ]
    last_direction = 0.
    for direction, label, duration in ((theta, "prove the first advance", 1.35),
                                        (-theta, "retract fully", 1.38),
                                        (theta, "prove the second advance", 1.45)):
        for angle in np.linspace(last_direction, direction, 5)[1:]:
            target, wrist = valve_pose(float(angle))
            left.append(w(.1625, target, FINGER_CLOSED, label, wrist))
        left.append(w(duration, target, FINGER_CLOSED, "hold the proof valve", wrist))
        last_direction = direction
    left += [
        w(.33, target, -.032, "release the latched valve", wrist),
        w(.58, target + (0., -.022, .045), -.032, "leave the finished shuttle clear", wrist),
    ]
    return {"left": left, "right": right}
