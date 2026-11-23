"""A foosball table across the front of two YAM arms; SI units.

World frame: x forward from the line between the arm bases, y to the left, z up from the floor.
The table's long axis runs along y; the robot plays from the table's near long side, like a person.
"""
CONTROL_HZ = 20
PHYSICS_HZ = 240
DWELL_STEPS = 10
FINGER_FORCE_LIMIT = 40.
PAD_TORSIONAL_RADIUS = .002

# Table sized like a Tornado Sport foosball table: 1.20 x 0.75 m playfield (47 x 29.5 in), eight
# 5/8 in steel rods 150 mm apart, 203 mm (8 in) goal mouths, 35 mm (1-3/8 in) balls.
PF_X0 = .600                         # near (robot-side) inner wall
PF_W = .750                          # playfield across, along x
PF_L = 1.200                         # playfield length, along y
PF_YC = -.080
PF_Z = .830                          # playfield surface above the floor
X_C = PF_X0 + PF_W / 2
Y_GOAL = PF_YC - PF_L / 2            # the opponents' goal line (inner face of the far end wall)
WALL = .020
WALL_H = .120                        # side walls above the playfield
END_H = .100
GOAL_W = .203
GOAL_H = .075
ROD_SPACING = .150
ROD_Z = PF_Z + .070                  # rod axes above the floor
ROD_D = .0159
HANDLE_D = .032
HANDLE_LEN = .110
HANDLE_OUT = .150                    # handle centre outside the near wall's inner face, rod centred
MAN_FOOT_BELOW = .067                # toe below the rod axis (3 mm above the playfield)
MAN_HEAD_ABOVE = .030
MAN_WIDTH = .022                     # along the rod
MAN_THICK = .016                     # along the play direction
FOOT_WIDTH = .025
FOOT_THICK = .020
FOOT_HEIGHT = .022
MAN_MASS = .030
ROD_MASS = .550                      # hollow steel rod, bumpers and handle
ROD_TRAVEL = .095                    # each way from centred
BALL_D = .035
BALL_MASS = .020
PLASTIC_STATIC = .30
PLASTIC_DYNAMIC = .25
BALL_RESTITUTION = .50


def rod_y(k):
    """Rod k (1-8) counted from the robot team's own goal at +y; 1 goalie ... 6 three-bar ... 8 opponents' goalie."""
    return PF_YC + PF_L / 2 - ROD_SPACING / 2 - ROD_SPACING * (k - 1)


# Rod k: (team, men offsets along x from the table's centre line). Robot's team handles are on the near side.
RODS = {
    1: ('robot', [0.]),
    2: ('robot', [-.1205, .1205]),
    3: ('opponent', [-.184, 0., .184]),
    4: ('robot', [-.240, -.120, 0., .120, .240]),
    5: ('opponent', [-.240, -.120, 0., .120, .240]),
    6: ('robot', [-.184, 0., .184]),
    7: ('opponent', [-.1205, .1205]),
    8: ('opponent', [-.060]),       # the opponents' keeper, held off-centre
}
ACTIVE_ROD = 6
ROD6_Y = rod_y(ACTIVE_ROD)
HANDLE_CENTRE = (PF_X0 - HANDLE_OUT, ROD6_Y, ROD_Z)
BALL_OFFSET_X = .050                 # ball's centre line from the middle man, rod centred
BALL_START = (X_C + BALL_OFFSET_X, ROD6_Y - FOOT_THICK / 2 - BALL_D / 2 - .004, PF_Z + BALL_D / 2)

# Scoring.
PROGRESS = .050                      # ball moved this far towards the goal
SPIN_LIMIT_DEG = 360.
WORKSPACE = ((-.40, -1.40, .40), (1.80, 1.00, 1.60))

INSTRUCTION_FALLBACK = (
    "Play foosball: using the handle of your three-man rod in front of the right arm, shoot the ball into the "
    "opponents' goal past their keeper. Hold the rod only by its handle and don't spin it."
)


def in_goal(p):
    """Ball centre through the goal mouth and behind the goal line."""
    x, y, z = p
    return y < Y_GOAL - BALL_D / 2 and abs(x - X_C) < GOAL_W / 2 and z < PF_Z + GOAL_H


def finger_joint_position(aperture):
    if not 0 <= aperture <= .075:
        raise ValueError('Aperture exceeds 75 mm')
    return -min(aperture, .0749) / 2
