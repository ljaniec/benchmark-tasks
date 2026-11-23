"""Single source of truth for the 9-ft table shuffleboard, its weight, the score lines and the score ladder."""
import math

CONTROL_HZ = 20
PHYSICS_HZ = 960
FINGER_FORCE_LIMIT = 40.
PAD_TORSIONAL_RADIUS = .002

# Board: a 9-ft table's playfield, 7 ft 8 in x 20 in, 30 in above the floor (Shuffleboard Federation).
BOARD_Z = .762
BOARD_L = 2.3368
BOARD_W = .508
X0 = .280                       # near end of the playfield
X1 = X0 + BOARD_L               # far end
LANE_Y = -.305                  # the board's centre line, straight ahead of the right arm
Y0, Y1 = LANE_Y - BOARD_W / 2, LANE_Y + BOARD_W / 2
# Cradle: 31 in wide and 9 ft long overall, with gutters round the playfield.
END_GUTTER = .200
SIDE_GUTTER = .100
WALL = .040
GUTTER_Z = BOARD_Z - .065
WALL_TOP_Z = BOARD_Z + .025
CRADLE = ((X0 - END_GUTTER, Y0 - SIDE_GUTTER - WALL), (X1 + END_GUTTER, Y1 + SIDE_GUTTER + WALL))

# Lines, from each end of the playfield: the 3-line at 6 in and the 2-line at 12 in; on tables of 9, 12 and
# 14 ft a single foul line crosses the middle. Each is a painted strip LINE_W wide centred on its distance.
LINE_W = .006
LINE3 = X1 - .1524
LINE2 = X1 - .3048
FOUL = X0 + BOARD_L / 2
NEAR_LINES = (X0 + .1524, X0 + .3048)

# Weight: chrome-plated steel puck 2 5/16 in across and 0.75 in high, with a plastic cap; 345 g in all.
PUCK_R = .058738 / 2
PUCK_H = .01905
CAP_R = .0275                   # cap size: our approximation
CAP_H = .013
WEIGHT_MASS = .345
WEIGHT_COM_Z = .0116            # most of the mass is in the steel puck
WEIGHT_START = (X0 + .035, LANE_Y, BOARD_Z)   # bottom centre

# (static friction, dynamic friction, restitution). The board's friction combines by the minimum, so the
# waxed playfield sets the slide for anything on it; the weight's own steel sets its grip in the fingers.
FRICTION = {
    'wax': (.02, .02, .20),
    'steel': (.30, .25, .30),
    'cap': (.40, .35, .30),
    'wood': (.45, .40, .30),
}

# Score ladder.
SHOT_SPEED = .10                # the weight slides at least this fast with nothing touching it: it has been shot
LIFT_LIMIT = .005               # free weight more than this above the board: thrown, not slid
REST_SPEED = .005
REST_STEPS = 20
FLAT_DEG = 10.
ZONES = ((LINE3 + LINE_W / 2, 1.), (LINE2 + LINE_W / 2, .5), (FOUL + LINE_W / 2, .25))

WORKSPACE = ((-.60, -1.80, 0.), (3.40, 1.80, 1.80))
INSTRUCTION_FALLBACK = ('Slide the red-capped weight down the shuffleboard with one push so that it stops in the '
                        '3-point zone at the far end: completely past the 3-line, which is 6 inches from the end, '
                        'without falling off. Keep it on the board: slide it, do not throw it, and do not touch it '
                        'again once it has left your hand.')


def zone_score(x_centre):
    """Score for a weight resting flat on the board with its centre at x_centre."""
    rear = x_centre - PUCK_R
    for line, score in ZONES:
        if rear > line:
            return score
    return 0.


def on_board(x, y, lift):
    return X0 <= x <= X1 and Y0 <= y <= Y1 and lift > -.003


def finger_joint_position(aperture):
    if not 0 <= aperture <= .075:
        raise ValueError('Aperture exceeds 75 mm')
    return -min(aperture, .0749) / 2


def tilt_deg(quat_xyzw):
    x, y, z, w = quat_xyzw
    # angle between the weight's axis and vertical
    c = 1 - 2 * (x * x + y * y)
    return math.degrees(math.acos(max(-1., min(1., c))))
