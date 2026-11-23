"""Single source of truth for the carrom board, the striker, the queen and the score ladder."""
import math

CONTROL_HZ = 20
PHYSICS_HZ = 960
FINGER_FORCE_LIMIT = 40.
PAD_TORSIONAL_RADIUS = .002

# Board (International Carrom Federation laws): a plywood playing surface 74 cm square and at least 8 mm thick,
# inside a hardwood frame 6.35 to 7.6 cm broad standing 1.9 to 2.54 cm above it. It lies on a stand, 30 in up.
BOARD_Z = .762
BOARD_T = .010
PLAY = .740
X0 = .220                       # inner edge of the frame on the robot's side
X1 = X0 + PLAY
BOARD_Y = 0.                    # the board is centred between the arms
Y0, Y1 = BOARD_Y - PLAY / 2, BOARD_Y + PLAY / 2
FRAME_W = .070
FRAME_H = .022
OUTER = ((X0 - FRAME_W, Y0 - FRAME_W), (X1 + FRAME_W, Y1 + FRAME_W))
# Four corner pockets 4.45 cm across, round inwardly, the plywood cut away (here 2 mm clear of the frame).
POCKET_R = .0445 / 2
POCKET_INSET = POCKET_R + .002
POCKETS = tuple((x, y) for x in (X0 + POCKET_INSET, X1 - POCKET_INSET) for y in (Y0 + POCKET_INSET, Y1 - POCKET_INSET))
CORNER = .060                   # side of the square corner pieces that carry the pocket holes
SEAM_STEP = .0002               # the corner pieces and end strips lie this much below the middle
NET_DEPTH = .055                # the net under each pocket catches what falls in

# Base lines on each side: two lines 47 cm long, the outer one 5 to 6.5 mm thick and 10.15 cm from the frame,
# the inner one 3.18 cm from it, closed at both ends by base circles 3.18 cm across.
BASE_OUTER = (X0 + .1015, X0 + .1075)
BASE_INNER = (X0 + .1333, X0 + .1348)
BASE_HALF = .235
BASE_CIRCLE_R = .0318 / 2
BASE_CIRCLE_X = X0 + .1015 + BASE_CIRCLE_R
BASE_CIRCLE_Y = BASE_HALF - BASE_CIRCLE_R

# Striker: smooth, at most 4.13 cm across and 15 g. Carrom men: wood, 3.02 to 3.18 cm across, 7 to 9 mm thick,
# 5.0 to 5.5 g; the queen is the red one.
BEVEL = .0008                  # the discs' top and bottom edges are bevelled
STRIKER_R = .0413 / 2
STRIKER_H = .009
STRIKER_MASS = .015
QUEEN_R = .0315 / 2
QUEEN_H = .008
QUEEN_MASS = .0055

# The striker touches both base lines while its centre is in this window; and it must lie between the base
# circles' centres.
WINDOW_X = (BASE_INNER[0] - STRIKER_R, BASE_OUTER[1] + STRIKER_R)
WINDOW_Y = (BOARD_Y - BASE_CIRCLE_Y, BOARD_Y + BASE_CIRCLE_Y)
# The queen stands 6 cm short of the far left pocket. The striker lies on the base lines where the straight
# line from that pocket through the queen crosses them, which is also straight out from the right arm's base.
TARGET_POCKET = (X1 - POCKET_INSET, Y1 - POCKET_INSET)
_ARM = (0., -.305)
_dx, _dy = TARGET_POCKET[0] - _ARM[0], TARGET_POCKET[1] - _ARM[1]
_n = math.hypot(_dx, _dy)
_sx = X0 + .1205
STRIKER_START = (_sx, _ARM[1] + (_sx - _ARM[0]) * _dy / _dx, BOARD_Z)          # bottom centre, on the base lines
QUEEN_START = (TARGET_POCKET[0] - .060 * _dx / _n, TARGET_POCKET[1] - .060 * _dy / _n, BOARD_Z)

# (static friction, dynamic friction, restitution). The powdered board's friction and restitution combine by the
# minimum, so anything on it slides at 0.04 and does not bounce. Restitution combines by the average elsewhere:
# striker on queen 0.82; either on the frame 0.6.
FRICTION = {
    'powder': (.04, .04, 0.),
    'striker_disc': (.20, .20, .82),
    'queen_disc': (.20, .20, .82),
    'rosewood': (.30, .25, .38),
    'net': (.80, .70, 0.),
    'wood': (.45, .40, .30),
}

# Score ladder.
PUSH_LIMIT = .010               # the hand may touch the striker only within this distance of the base-line window
LIFT_LIMIT = .005               # a disc more than this above the board with nothing touching it is in the air
HIT_MOVE = .010                 # the queen has moved this far: it has been hit
REST_SPEED = .005
REST_STEPS = 20

WORKSPACE = ((-.60, -1.20, 0.), (1.60, 1.20, 1.60))


def window_outside(x, y):
    """Distance (m) of the striker's centre outside the base-line window; 0 inside it."""
    dx = max(WINDOW_X[0] - x, 0., x - WINDOW_X[1])
    dy = max(WINDOW_Y[0] - y, 0., y - WINDOW_Y[1])
    return math.hypot(dx, dy)


def in_net(x, y):
    return any(abs(x - px) <= CORNER and abs(y - py) <= CORNER for px, py in POCKETS)


def over_pocket(x, y):
    return any(math.hypot(x - px, y - py) < POCKET_R + .020 for px, py in POCKETS)


def pocketed(x, y, lift):
    """Fallen through a pocket into its net: below the underside of the board."""
    return in_net(x, y) and -(BOARD_T + NET_DEPTH + .010) < lift < -(BOARD_T + .002)


def on_board(x, y, lift):
    """On the playing surface, or dropping through a pocket."""
    if over_pocket(x, y) and -(BOARD_T + .002) <= lift < .030:
        return True
    return X0 <= x <= X1 and Y0 <= y <= Y1 and -.003 < lift < .030


def finger_joint_position(aperture):
    if not 0 <= aperture <= .075:
        raise ValueError('Aperture exceeds 75 mm')
    return -min(aperture, .0749) / 2
