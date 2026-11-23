"""A fixed target path for supporting the loaded head while setting its clamp."""
import math

import numpy as np

import clamp
import geometry as g
from yam_arena.constants import FINGER_OPEN, READY
from yam_arena.urdf import axis_angle_matrix


class WristPose(float):
    def __new__(cls, rotation):
        pose = float.__new__(cls, 0.)
        pose.rotation = rotation
        return pose


def pitch(angle):
    return axis_angle_matrix(np.array((0., 1., 0.)), math.radians(angle))


def demo():
    from yam_arena import demo as motion, ik, station

    original = motion.grip_rotation
    if not getattr(original, "fan_pose", False):
        def grip_rotation(yaw=0.):
            return yaw.rotation if isinstance(yaw, WristPose) else original(yaw)
        grip_rotation.fan_pose = True
        motion.grip_rotation = grip_rotation

    left_home, original_left_rot = ik.tcp_pose(READY, station.BASES["left"])
    right_home, original_right_rot = ik.tcp_pose(READY, station.BASES["right"])
    toward_fan = axis_angle_matrix(np.array((1., 0., 0.)), math.radians(136.))
    inward = np.array(((1., 0., 0.), (0., 0., 1.), (0., -1., 0.)))

    loose_support = g.ORIGIN + g.rotate_head(g.SUPPORT, g.DROP_DEG)
    level_support = g.ORIGIN + g.SUPPORT
    knob_pos = g.ORIGIN + g.KNOB_GRIP
    loose_head = WristPose(pitch(g.DROP_DEG) @ toward_fan)
    level_head = WristPose(toward_fan)
    loose_knob = WristPose(pitch(clamp.LOOSE_DEG) @ inward)
    tight_knob = WristPose(pitch(24.) @ inward)
    front = knob_pos + (0., -.062, 0.)
    weight_rotation = axis_angle_matrix(np.array((1., 0., 0.)), math.radians(58.)) @ np.diag((1., -1., -1.))
    weight_pose = WristPose(weight_rotation)
    pick = g.WEIGHT_REST + g.WEIGHT_GRIP + (0., 0., .003)
    place = g.ORIGIN + g.PROOF_SITE + g.WEIGHT_GRIP + (0., 0., .010)
    over_rim = place + (0., 0., .029)
    w = motion.Waypoint
    left = [
        w(1.35, loose_support - loose_head.rotation[:, 2] * .055, FINGER_OPEN, "approach the lowered head", loose_head),
        w(.75, loose_support, FINGER_OPEN, "place the jaws around the guard rim", loose_head),
        w(.55, loose_support, -.010, "support the guard", loose_head),
        w(1.15, level_support, -.010, "bring the front of the fan to level", level_head),
        w(3.6, level_support, -.010, "hold the loaded head through tightening", level_head),
        w(.4, level_support, FINGER_OPEN, "release the head", level_head),
        w(.85, level_support - level_head.rotation[:, 2] * .060, FINGER_OPEN, "clear the fan", level_head),
        w(1., left_home, FINGER_OPEN, "return the support hand", WristPose(original_left_rot)),
    ]
    right = [
        w(1.5, front, FINGER_OPEN, "approach the side adjustment knob", loose_knob),
        w(1.85, front, FINGER_OPEN, "wait for the head to be supported", loose_knob),
        w(.65, knob_pos, FINGER_OPEN, "straddle the knob", loose_knob),
        w(.50, knob_pos, -.009, "grip the adjustment knob", loose_knob),
        w(1.2, knob_pos, -.009, "tighten the loaded pitch clamp", tight_knob),
        w(.4, knob_pos, FINGER_OPEN, "release the knob", tight_knob),
        w(.7, front, FINGER_OPEN, "withdraw from the neck", tight_knob),
        w(1.15, right_home, FINGER_OPEN, "clear the neck before picking up the proof weight", WristPose(original_right_rot)),
        w(.90, pick + (0., -.018, .072), FINGER_OPEN, "approach the calibrated weight", weight_pose),
        w(.65, pick, FINGER_OPEN, "straddle the gauge grip", weight_pose),
        w(.50, pick, -.006, "pick up the weight", weight_pose),
        w(.65, pick + (0., -.018, .075), -.006, "lift the proof weight", weight_pose),
        w(.70, (.315, -.214, 1.02), -.006, "lift the weight above the side of the bench", weight_pose),
        w(1.0, over_rim + (0., -.042, 0.), -.006, "bring the weight to the side of the rim", weight_pose),
        w(.70, over_rim, -.006, "center the saddle from above", weight_pose),
        w(.70, place, -.006, "lower to just above the grille", weight_pose),
        w(.50, place, FINGER_OPEN, "release the proof weight", weight_pose),
        w(.70, over_rim + (0., -.042, .006), FINGER_OPEN, "clear the loaded head", weight_pose),
        w(1.10, right_home, FINGER_OPEN, "return the proof hand", WristPose(original_right_rot)),
    ]
    return {"left": left, "right": right}


def main():
    from yam_arena import demo as motion, station

    for side, route in demo().items():
        plan = motion.ArmPlan(route, station.BASES[side])
        errors = {}
        for t in np.arange(0., plan.duration + .05, .1):
            target, _, jaw, label = plan.target(t)
            plan.action7(t)
            actual = motion.ik.tcp_pose(plan.q, station.BASES[side])[0]
            delta = float(np.linalg.norm(actual - target) * 1000.)
            errors[label] = max(delta, errors.get(label, 0.))
        print(side, plan.duration, errors)


if __name__ == "__main__":
    main()
