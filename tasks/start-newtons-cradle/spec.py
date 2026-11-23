"""A desktop Newton's cradle on a table in front of two YAM arms; SI units.

World frame: x forward from the line between the arm bases, y to the left, z up from the floor.
The row of balls runs along x, away from the arms; the balls swing in the x-z plane.
"""
import math

CONTROL_HZ = 20
PHYSICS_HZ = 960
FINGER_FORCE_LIMIT = 40.
PAD_TORSIONAL_RADIUS = .002

# Table.
TABLE_CENTRE = (.520, 0.)
TABLE_SIZE = (.500, .600)
TABLE_TOP_Z = .760

# Cradle base: a black wooden plinth.
BASE_SIZE = (.260, .180, .020)
BASE_CENTRE = (.520, 0.)
BASE_TOP_Z = TABLE_TOP_Z + BASE_SIZE[2]

# Balls: five 25 mm chrome steel balls (64 g), hanging in a row along x, 1 mm apart.
BALLS = 5
BALL_D = .025
BALL_R = BALL_D / 2
BALL_MASS = .064
BALL_GAP = .001                      # between neighbours at rest, wider than their contact distance, so impacts pass ball to ball
PITCH = BALL_D + BALL_GAP
BALL_CLEARANCE = .030                # from the base's top to the bottom of a hanging ball
BALL_Z = BASE_TOP_Z + BALL_CLEARANCE + BALL_R
ROW_X0 = BASE_CENTRE[0] - 2 * PITCH  # the ball nearest the arms ("ball 1"); ball 5 is the far end


def ball_rest_x(k):
    return ROW_X0 + k * PITCH


# Each ball hangs from two nylon lines in a V to two chrome rails 90 mm apart along x, so it swings only in
# the x-z plane, about the line through its two anchors.
RAIL_Y = .045
RAIL_Z = BALL_Z + .1175              # anchor line height; 117.5 mm from anchor line to ball centre
RAIL_D = .006
SWING_R = RAIL_Z - BALL_Z
LINE_LEN = math.hypot(RAIL_Y, SWING_R - BALL_R)
LINE_D = .0006
# PhysX distance joints engage only this far beyond their maximum distance, so the authored limit is
# shortened by it.
JOINT_TOLERANCE = .025
RAIL_OVERHANG = .070                 # rails run 70 mm past each end ball's centre to the end frames
FRAME_X_ENDS = (ball_rest_x(0) - RAIL_OVERHANG, ball_rest_x(BALLS - 1) + RAIL_OVERHANG)
LEG_FOOT_Y = .090                    # end frames' legs splay out to 90 mm either side at the base
FRAME_MASS = .80                     # wooden plinth and chrome frame; it stands loose on the table

BALL_RESTITUTION = .97
BALL_FRICTION = .10
STEEL_ROLL_DAMPING = .02             # air drag on a ball, roughly (1/s)

# Scoring.
LIFTED_DEG = 20.                     # ball 1 lifted at least this far on taut lines
TAUT_TOL = .0025                     # a ball's centre within 2.5 mm of its swing circle: a 114 mm string sagging under about 1 cm
STILL_DEG = 3.                       # balls 2-5 hanging still while ball 1 is lifted
RELEASE_SPEED = .10                  # ball 1 let go, not pushed: slower than this at the last touch
FAR_HALF = 10.                       # ball 5's swing-out for 0.5
FAR_FULL = 15.                       # ball 5's swing-out for 1.0 (a clean release from 20 degrees gives about 18.6)
MIDDLE_LIMIT = 6.                    # balls 2-4 must stay within this while ball 5 swings out (clean transfer)
FRAME_MOVE_LIMIT = .003              # largest shift of any plinth corner or rail end
WORKSPACE = ((-.40, -1.40, 0.), (1.40, 1.00, 1.60))

INSTRUCTION_FALLBACK = (
    "Start the Newton's cradle: lift the ball nearest you out along its swing with its strings taut, and let it "
    "go so that only the ball at the far end swings out. Don't touch the other balls."
)


def ball_pose(k, angle_deg):
    """Centre and quaternion (w, x, y, z) of ball k swung angle_deg towards the arms (negative x)."""
    a = math.radians(angle_deg)
    x = ball_rest_x(k) - SWING_R * math.sin(a)
    z = RAIL_Z - SWING_R * math.cos(a)
    return (x, 0., z), (math.cos(a / 2), 0., math.sin(a / 2), 0.)


def swing_angle_deg(k, centre):
    """Angle of ball k towards the arms (positive) or away from them (negative), from its centre."""
    dx = ball_rest_x(k) - centre[0]
    dz = RAIL_Z - centre[2]
    return math.degrees(math.atan2(dx, dz))


def radius_error(k, centre):
    dx = centre[0] - ball_rest_x(k)
    dz = RAIL_Z - centre[2]
    return math.hypot(dx, dz) - SWING_R


def frame_points():
    """Plinth corners (bottom) and rail ends, where the frame's movement is measured."""
    bx, by = BASE_CENTRE
    hx, hy = BASE_SIZE[0] / 2, BASE_SIZE[1] / 2
    corners = [(bx + sx * hx, by + sy * hy, TABLE_TOP_Z) for sx in (-1, 1) for sy in (-1, 1)]
    return corners + [(x, sy * RAIL_Y, RAIL_Z) for x in FRAME_X_ENDS for sy in (-1, 1)]


def finger_joint_position(aperture):
    if not 0 <= aperture <= .075:
        raise ValueError('Aperture exceeds 75 mm')
    return -min(aperture, .0749) / 2
