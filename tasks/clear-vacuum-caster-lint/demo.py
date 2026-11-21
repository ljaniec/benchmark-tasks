"""Fixed tool poses for cleaning and refitting a robot vacuum's original caster."""

from pathlib import Path
import math
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from caster_fixture import SOCKET, TIRE_OFFSET
from service_station import SLOT_ENTRY, SLOT_HOME, TIRE_REST
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, READY


class ForkGrip(float):
    pass


def release_plan(mat_target, setdown_height):
    from yam_arena import demo as motion
    from yam_arena.urdf import axis_angle_matrix

    active = motion.grip_rotation
    stock = getattr(active, 'caster_stock', active)
    down = axis_angle_matrix(np.array([0., 1., 0.]), math.radians(-25.)) @ stock(0.)

    def rotation(yaw=0.):
        return down if isinstance(yaw, ForkGrip) else stock(yaw)

    rotation.caster_stock = stock
    motion.grip_rotation = rotation
    Waypoint = motion.Waypoint
    grip = ForkGrip(0.)

    x, y, z = SOCKET
    open_gap = -.030
    seated = (x - .002, y, z + .024)
    lifted = (x, y, z + .063)
    touchdown = (*mat_target, setdown_height)
    return {'right': [
        Waypoint(1.2, (x, y, z + .072), open_gap, 'above installed caster', grip),
        Waypoint(1.2, seated, open_gap, 'around fork', grip),
        Waypoint(.6, seated, FINGER_CLOSED, 'grip fork', grip),
        Waypoint(1.8, lifted, FINGER_CLOSED, 'withdraw pivot', grip),
        Waypoint(1.4, (*mat_target, lifted[2]), FINGER_CLOSED, 'move over mat', grip),
        Waypoint(1.2, touchdown, FINGER_CLOSED, 'lower caster onto the mat', grip),
        Waypoint(.5, touchdown, FINGER_CLOSED, 'settle the stem on the mat', grip),
        Waypoint(.8, touchdown, open_gap, 'release caster', grip),
        Waypoint(1.1, (*mat_target, .881), open_gap, 'withdraw', grip),
    ]}


def slot_transfer():
    from yam_arena.demo import Waypoint

    entry = (*SLOT_ENTRY[:2], .802)
    alignment_y = SLOT_HOME[1] + .002
    seat = (SLOT_HOME[0], alignment_y, .802)
    hover = (*SLOT_ENTRY[:2], .886)
    approach = release_plan(tuple(SLOT_ENTRY[:2]), entry[2])["right"][:4]
    grip = approach[-1].yaw
    return {"right": [
        *approach,
        Waypoint(1.75, hover, FINGER_CLOSED, "move above the open entry", grip),
        Waypoint(1.15, hover, FINGER_CLOSED, "let the held caster settle above the opening", grip),
        Waypoint(1.8, entry, FINGER_CLOSED, "lower the stem through the open entry", grip),
        Waypoint(.65, entry, FINGER_CLOSED, "support the caster foot", grip),
        Waypoint(1.2, (.269, SLOT_HOME[1], .802), FINGER_CLOSED, "slide to the beginning of the taper", grip),
        Waypoint(.75, (.269, alignment_y, .802), FINGER_CLOSED, "center the stem in the taper", grip),
        Waypoint(1.9, seat, FINGER_CLOSED, "slide the stem collar into the keyway", grip),
        Waypoint(.85, seat, -.030, "release the retained caster", grip),
        Waypoint(1., (*SLOT_HOME[:2], .892), -.030, "clear the caster and slot", grip),
    ]}


class TireGrip(float):
    """An ordinary down grip tilted forward to leave clearance at the tread."""


class ReadyGrip(float):
    """Keep the idle arm in the Arena's own ready pose."""

    def __new__(cls, side):
        grip = super().__new__(cls, 0.)
        grip.side = side
        return grip


def wheel_removal():
    from yam_arena import demo as motion
    from yam_arena import ik, station
    from yam_arena.urdf import axis_angle_matrix

    right = slot_transfer()["right"]
    rest = {side: ik.tcp_pose(np.array(READY, dtype=float), station.BASES[side])
            for side in ("left", "right")}
    right.append(motion.Waypoint(1.65, tuple(rest["right"][0]), FINGER_OPEN,
                                 "park the right arm at ready", ReadyGrip("right")))
    slot_rotation = motion.grip_rotation
    tilt = axis_angle_matrix(np.array([0., 1., 0.]), math.radians(-30.))
    def rotation(yaw=0.):
        if isinstance(yaw, ReadyGrip):
            return rest[yaw.side][1]
        if isinstance(yaw, TireGrip):
            return tilt @ slot_rotation(float(yaw))
        return slot_rotation(yaw)

    rotation.caster_stock = slot_rotation.caster_stock
    motion.grip_rotation = rotation
    wp = motion.Waypoint
    grip = TireGrip(0.)
    x, y, z = SLOT_HOME
    # The pads close across the tread width above the fork cheeks. Both
    # pads enter at the same height as the jaws close.
    pick = (x - .0045, y, z + .0445)
    wx, wy, wz = TIRE_REST
    wheel_drop = (wx - .0045, wy, wz + .021)
    left = [
        wp(sum(p.duration for p in right), tuple(rest["left"][0]), FINGER_OPEN,
           "keep the left arm at ready until the fork is in the slot", ReadyGrip("left")),
        wp(1.4, (.291, .044, .939), -.020, "approach well above the work surface", grip),
        wp(1.5, (x - .002, y, z + .104), -.020, "hover over the wheel", grip),
        wp(.85, (x - .002, y, z + .104), -.020, "let the wrist settle", grip),
        wp(1.65, pick, -.020, "place pads around the tread", grip),
        wp(.7, pick, FINGER_CLOSED, "grasp the tread above the fork", grip),
        wp(1.4, (pick[0], y, z + .103), FINGER_CLOSED, "pull the axle through both retaining clips", grip),
        wp(1.5, (wx, wy, z + .095), FINGER_CLOSED, "carry the wheel to its rest", grip),
        wp(1.15, wheel_drop, FINGER_CLOSED, "support the wheel between the rails", grip),
        wp(.7, wheel_drop, -.027, "release the wheel", grip),
        wp(1.1, (wx, wy, z + .118), -.027, "clear the tread", grip),
    ]
    return {"left": left, "right": right}


class FlickGrip(float):
    """Keep the pads closed while sweeping the tread."""


def demo():
    from yam_arena import demo as motion
    from yam_arena.urdf import axis_angle_matrix

    paths = wheel_removal()
    left, right = paths["left"], paths["right"]
    fork_grip = next(w.yaw for w in right if isinstance(w.yaw, ForkGrip))
    left_ready = left[0]
    stock = motion.grip_rotation
    flick_tilt = axis_angle_matrix(np.array([0., 1., 0.]), math.radians(-20.))

    def rotate(yaw=0.):
        if isinstance(yaw, FlickGrip):
            return flick_tilt @ stock(float(yaw))
        return stock(yaw)

    rotate.caster_stock = stock.caster_stock
    motion.grip_rotation = rotate
    wp = motion.Waypoint

    def hold_for(other, path):
        duration = sum(w.duration for w in other) - sum(w.duration for w in path)
        if duration > 1e-8:
            last = path[-1]
            path.append(wp(duration, last.pos, last.finger, "hold clear while the other arm works", last.yaw))

    x, y, z = SLOT_HOME
    tire_grip = TireGrip(0.)
    wx, wy, wz = TIRE_REST
    pickup = (wx - .0045, wy + .006, wz + .022)
    tire_hover = (x - .010, y + .0006, .878)
    tire_entry = (x - .0095, y + .0006, .845)
    tire_seat = (tire_entry[0], tire_entry[1] - .0012, z + .033)
    press_hover = (x - .0108, y + .0005, z + .094)
    press_tread = (*press_hover[:2], z + .0505)
    left.extend([
        wp(.6, left[-1].pos, left[-1].finger, "let the freed lint settle in the catch tray", tire_grip),
        wp(1., pickup, -.027, "approach the wheel in its rest", tire_grip),
        wp(.6, pickup, FINGER_CLOSED, "grasp the original wheel", tire_grip),
        wp(.9, (*pickup[:2], .871), FINGER_CLOSED, "lift the wheel and axle together", tire_grip),
        wp(1.3, tire_hover, FINGER_CLOSED, "align the axle over the fork", tire_grip),
        wp(.65, tire_hover, FINGER_CLOSED, "hold the alignment", tire_grip),
        wp(.95, tire_entry, FINGER_CLOSED, "center both ends over the molded guides", tire_grip),
        wp(1.2, tire_entry, FINGER_CLOSED, "settle over the retaining clips", tire_grip),
        wp(2.2, tire_seat, FINGER_CLOSED, "press the axle past the retaining clips", tire_grip),
        wp(.65, tire_seat, FINGER_CLOSED, "hold the wheel on its bearing seat", tire_grip),
        wp(.65, tire_seat, -.027, "release the wheel across both clip mouths", tire_grip),
        wp(.60, press_hover, -.027, "clear the tread and orient the pads down", fork_grip),
        wp(.35, press_hover, FINGER_CLOSED, "close the pads above the tread", fork_grip),
        wp(.90, press_tread, FINGER_CLOSED, "press straight down on the tread", fork_grip),
        wp(.50, press_tread, FINGER_CLOSED, "hold the axle against the bearing seat", fork_grip),
        wp(.55, press_hover, FINGER_CLOSED, "withdraw the pads vertically", fork_grip),
        wp(1.35, left_ready.pos, left_ready.finger, "park at ready for caster refit", left_ready.yaw),
    ])
    hold_for(left, right)
    fork_pick = (x - .0075, y + .0005, .801)
    free_of_slot = (*SLOT_ENTRY[:2], .802)
    sx, sy, sz = SOCKET
    mount = (sx - .0082, sy + .0016, sz + .013)
    right.extend([
        wp(1.1, (*fork_pick[:2], .882), -.030, "reach above the reassembled caster", fork_grip),
        wp(.95, fork_pick, -.030, "place pads outside the fork", fork_grip),
        wp(.55, fork_pick, FINGER_CLOSED, "pinch the fork", fork_grip),
        wp(1.65, free_of_slot, FINGER_CLOSED, "slide the stem out of the slot", fork_grip),
        wp(.85, (*free_of_slot[:2], .890), FINGER_CLOSED, "lift the intact caster", fork_grip),
        wp(1.3, (*mount[:2], .889), FINGER_CLOSED, "center the stem over the chassis seat", fork_grip),
        wp(.9, (*mount[:2], .889), FINGER_CLOSED, "hold the mounting alignment", fork_grip),
        wp(1.65, mount, FINGER_CLOSED, "press the stem through its detent", fork_grip),
        wp(.7, mount, FINGER_CLOSED, "hold the fork on the chassis seat", fork_grip),
        wp(.5, mount, -.030, "release the refitted caster", fork_grip),
        wp(.8, (.345, -.033, .918), -.030, "withdraw before checking the wheel", fork_grip),
    ])
    flick = FlickGrip(0.)
    crown = sz + TIRE_OFFSET[2]
    fx = sx - .0037
    fy = sy + .0019
    right.extend([
        wp(.95, (fx - .013, fy, crown + .0582), FINGER_CLOSED, "approach the tread", flick),
        wp(1.1, (fx - .010, fy, crown + .0242), FINGER_CLOSED, "touch the front of the tread", flick),
        wp(.3, (fx, fy, crown + .0272), FINGER_CLOSED, "flick the wheel", flick),
        wp(.25, (fx + .011, fy, crown + .0232), FINGER_CLOSED, "finish the flick", flick),
        wp(.32, (fx + .011, fy, crown + .0612), FINGER_CLOSED, "withdraw from the turning wheel", flick),
        wp(1.2, (fx - .020, fy - .060, crown + .0772), FINGER_CLOSED, "observe the released wheel", flick),
    ])
    return paths
