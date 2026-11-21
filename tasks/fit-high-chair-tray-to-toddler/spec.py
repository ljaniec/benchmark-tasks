"""High chair with a four-position sliding tray and a seated toddler manikin; SI units.

World frame: x forward from the line between the arm bases, y to the left, z up from the floor.
The chair faces the arms, so the tray slides along +x toward the toddler.
"""
import math

CONTROL_HZ = 20
PHYSICS_HZ = 240
DWELL_STEPS = 10
FINGER_FORCE_LIMIT = 40.
PAD_TORSIONAL_RADIUS = .002

# Armrest rails (the tray runs on these). Top surface height, rail section and extent.
ARM_TOP = .720
ARM_H = .030
ARM_W = .032
ARM_Y = .170
ARM_X0 = .460
ARM_X1 = .800
# Four locking notches cut 8 mm up into each rail's underside, 18 mm long.
NOTCH_DEPTH = .008
NOTCH_LEN = .018

# Tray: plate with a raised rim; its local origin is the centre of the plate's underside.
TRAY_LEN = .300   # x, front to back
TRAY_WIDTH = .440  # y
PLATE_T = .012
RIM_H = .020
RIM_T = .010
TRAY_MASS = .65
CHANNEL_DEPTH = .044
CHANNEL_GAP = .003
CHANNEL_WALL = .004
CHANNEL_X = (-.120, .130)

# Release bail: one rigid part pivoting about a lateral axis under the tray. The front button
# rises when squeezed toward the front rim, which lowers both locking fingers out of their notches.
BAIL_MASS = .060
BAIL_PIVOT = (-.040, 0., -.043)
FINGER_X = .100
FINGER_LEN = .012
FINGER_WIDTH = .020
FINGER_TOP = -.024          # tray-local z of a finger's top face at rest (engaged)
BAIL_ARM_Z = (-.046, -.040)
# The button sticks out 22 mm under the front edge; the front rim has a 120 mm thumb gap above it.
BUTTON_X = (-.172, -.132)
RISER_X = (-.137, -.131)
BUTTON_HALF_WIDTH = .040
BUTTON_Z = (-.030, -.022)
RIM_GAP_HALF = .060
BAIL_LIMIT_DEG = 6.
BAIL_STIFFNESS = .25        # N m per degree (UsdPhysics angular drive units)
BAIL_DAMPING = .02

# Tray centre x for each of the four positions. The rear rim clears the belly by 80, 50 and 20 mm
# at positions 1-3; position 4 would need -10 mm, so the tray stops against the manikin. The tray starts
# locked in position 1.
POSITION_X = (.520, .550, .580, .610)
NOTCH_X = tuple(x + FINGER_X for x in POSITION_X)
TRAY_START_X = POSITION_X[0]
TARGET_POSITION = 3
RELEASE_TRAVEL = .010
TRAY_Z = ARM_TOP

# Seated toddler manikin (12-18-month size), rigid and harnessed. Each part is a capsule
# (segment end points, radius) or a sphere (equal end points); BELLY_X is the front of the belly.
SEAT_TOP = .560
BELLY_X = .750


def _toddler():
    b, parts = BELLY_X, {}
    parts['head'] = ((b + .070, 0., .940), (b + .070, 0., .940), .075)
    parts['neck'] = ((b + .065, 0., .850), (b + .068, 0., .880), .036)
    for sign, n in [(1, 'left'), (-1, 'right')]:
        parts['torso_' + n] = ((b + .060, sign * .025, .620), (b + .062, sign * .025, .800), .060)
        parts['thigh_' + n] = ((b - .080, sign * .055, .590), (b + .040, sign * .055, .590), .030)
        parts['shin_' + n] = ((b - .086, sign * .057, .430), (b - .084, sign * .056, .560), .025)
        parts['foot_' + n] = ((b - .125, sign * .057, .404), (b - .080, sign * .057, .404), .021)
        parts['arm_' + n] = ((b + .062, sign * .125, .805), (b + .052, sign * .128, .685), .022)
        parts['hand_' + n] = ((b + .046, sign * .128, .660), (b + .046, sign * .128, .660), .022)
    return parts


TODDLER = _toddler()
HEAD_CENTRE, _, HEAD_RADIUS = TODDLER['head']

# Chair parts not tied to the rails or toddler are placed relative to this seat-front line.
SEAT_FRONT_X = .660

TRAY_CONTACT_STATIC = .35
TRAY_CONTACT_DYNAMIC = .30

# A finger counts as locked in a notch when its centre is within this distance of the notch
# centre and its top is at least ENGAGE_DEPTH above the rail's underside.
ENGAGE_X_TOL = .004
ENGAGE_DEPTH = .003
TOUCH_MARGIN = .001
DROP_Z = .600

WORKSPACE = ((-.30, -.80, -.01), (1.30, .80, 1.40))

INSTRUCTION_FALLBACK = (
    "A toddler manikin is buckled into the high chair and the tray is locked in its outermost position. "
    "Using the release button under the tray's front edge, slide the tray in to the closest of its four "
    "positions that does not touch the manikin, make sure both locking fingers have clicked in, and let go."
)


def rail_bottom():
    return ARM_TOP - ARM_H


def finger_top_rest_world():
    return TRAY_Z + FINGER_TOP


def finger_drop(bail_rad):
    """Finger lowering (m) for a bail rotation; positive rotation is the squeezed direction."""
    return (FINGER_X - BAIL_PIVOT[0]) * math.sin(bail_rad)


def button_rise(bail_rad):
    return (BAIL_PIVOT[0] - BUTTON_X[0]) * math.sin(bail_rad)


def rim_gap(position_index):
    """Gap (m) between the tray's rear rim and the toddler at a 1-based position."""
    return BELLY_X - (POSITION_X[position_index - 1] + TRAY_LEN / 2)


def finger_joint_position(aperture):
    if not 0 <= aperture <= .075:
        raise ValueError('Aperture exceeds 75 mm')
    return -min(aperture, .0749) / 2
