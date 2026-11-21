"""Wooden cup-and-ball toy on a floor stand beside two YAM arms; SI units.

World frame: x forward from the line between the arm bases, y to the left, z up from the floor.
Toy frame: origin on the handle axis where the handle meets the cup's base, z up the handle.
"""
import math

CONTROL_HZ = 20
PHYSICS_HZ = 240
DWELL_STEPS = 20
FINGER_FORCE_LIMIT = 40.
PAD_TORSIONAL_RADIUS = .002

# Handmade wooden cup-and-ball (RoyaltyRoute, Amazon ASIN B00DPSU8OC): 7.5 in overall, 3.5 in cup,
# 1.25 in ball on a 21 in string tied to the top of the handle, 120 g. Wall, depth and handle
# thickness are not published and are estimates.
OVERALL_LEN = .1905
CUP_OUTER_R = .04445
CUP_INNER_R = .0380
CUP_HEIGHT = .050
CUP_FLOOR_T = .010
HANDLE_R = .012
HANDLE_LEN = OVERALL_LEN - CUP_HEIGHT
BALL_R = .015875
BALL_MASS = .011
TOY_MASS = .120 - BALL_MASS
STRING_LEN = .533
STRING_D = .0024
# PhysX distance joints engage only this far beyond their maximum distance (measured: a 0.550 m limit
# holds the ball at 0.575 m), so the authored limit is shortened by it.
JOINT_TOLERANCE = .025
# The knot sits on the handle's right side (-y) 10 mm under the cup.
TIE_LOCAL = (0., -(HANDLE_R + STRING_D / 2), -.010)
RIM_LOCAL_Z = CUP_HEIGHT
CUP_FLOOR_LOCAL_Z = CUP_FLOOR_T
WALL_SEGMENTS = 20

# Lacquered wood on lacquered wood.
WOOD_STATIC = .45
WOOD_DYNAMIC = .40
WOOD_RESTITUTION = .40
BALL_LINEAR_DAMPING = .02

# Floor stand: the handle stands in a 27 mm sleeve held out on a bracket from a post behind it; the
# toy is upright with the ball hanging still on its full string, which leaves the sleeve through a
# 10 mm slot on its right side.
STAND_X = .500
STAND_Y = -.450
SLEEVE_BOTTOM_Z = .860               # the handle's end rests on the sleeve's floor
SLEEVE_HEIGHT = .040
SLEEVE_INNER_R = .0135
SLEEVE_WALL = .006
SLEEVE_SEGMENTS = 12
SLEEVE_SLOT = 9                      # wall segment k faces 2 pi k / 12; 9 faces -y
POST_X = STAND_X + .160
POST_SIZE = .040
BASE_SIZE = (.300, .300, .020)
TOY_START = (STAND_X, STAND_Y, SLEEVE_BOTTOM_Z + HANDLE_LEN)

# The toy counts as lifted once the handle's end is this far above the sleeve's rim.
LIFT_CLEAR = .010
# Upright: the handle within this angle of vertical.
UPRIGHT_DEG = 30.
# Caught: the ball's centre stays inside the cup's inner wall and below the rim for DWELL_STEPS frames.
# PhysX gives a sphere no rolling resistance, so a caught ball can keep circling the cup's floor.
TOUCH_MARGIN = .001
DROP_Z = .300

WORKSPACE = ((-.40, -1.20, -.01), (1.20, .80, 1.60))

INSTRUCTION_FALLBACK = (
    "Take the wooden cup-and-ball toy out of its stand by the handle, swing the ball up and catch it in "
    "the cup, then keep the toy upright with the ball in the cup. Only the toy may touch the ball."
)


def tie_world(toy_pos):
    return (toy_pos[0] + TIE_LOCAL[0], toy_pos[1] + TIE_LOCAL[1], toy_pos[2] + TIE_LOCAL[2])


def ball_start():
    x, y, z = tie_world(TOY_START)
    return (x, y, z - STRING_LEN - BALL_R)


def sleeve_rim_z():
    return SLEEVE_BOTTOM_Z + SLEEVE_HEIGHT


def in_cup(local, margin=.002):
    """Ball centre (toy frame) inside the cup: within the inner wall and between the floor and the rim."""
    r = math.hypot(local[0], local[1])
    return (r <= CUP_INNER_R - BALL_R + margin and CUP_FLOOR_LOCAL_Z + BALL_R - .003 <= local[2] <= RIM_LOCAL_Z)


def finger_joint_position(aperture):
    if not 0 <= aperture <= .075:
        raise ValueError('Aperture exceeds 75 mm')
    return -min(aperture, .0749) / 2
