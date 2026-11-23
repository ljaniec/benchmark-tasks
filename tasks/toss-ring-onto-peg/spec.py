"""Kiddie Play ring toss set on the floor in front of two YAM arms; SI units.

World frame: x forward from the line between the arm bases, y to the left, z up from the floor.
"""
import math

CONTROL_HZ = 20
PHYSICS_HZ = 240
DWELL_STEPS = 20
FINGER_FORCE_LIMIT = 40.
PAD_TORSIONAL_RADIUS = .002

# Kiddie Play Ring Toss Game (Amazon ASIN B0CMR4MDYF): a two-piece plastic cross base 16 x 16 in
# (406 mm), five pegs, 7 in (178 mm) tall assembled, five 5 in (127 mm) rings; 9.6 oz (272 g) in all.
# The ring's tube, the pegs' diameter and the base's section are not published and are estimates.
RING_OUTER_D = .127
RING_TUBE_D = .010
RING_MID_R = (RING_OUTER_D - RING_TUBE_D) / 2          # centre line of the tube
RING_INNER_R = RING_MID_R - RING_TUBE_D / 2
RING_MASS = .016
RING_SEGMENTS = 16
BASE_SPAN = .406
BASE_BAR = (.040, .022)                                 # width and height of each arm of the cross
PEG_D = .025
PEG_TOP_Z = .178                                        # rounded tip, above the floor
PEG_OFFSET = .180                                       # outer pegs from the centre peg
SET_MASS = .272 - 5 * RING_MASS

# Plastic on plastic and on the floor.
PLASTIC_STATIC = .35
PLASTIC_DYNAMIC = .30
PLASTIC_RESTITUTION = .30

# Layout: the set lies on the floor straight ahead of the right arm; the ring lies flat on a side
# table to the right of that arm, its back edge hanging 30 mm over the table's back edge.
SET_X = .820
SET_Y = -.305
LINE_X = .450                                            # throw line (front of the robot's side), taped on the floor
TABLE_SIZE = (.200, .240)
TABLE_X = .330
TABLE_Y = -.600
TABLE_TOP_Z = .840
TABLE_BACK_X = TABLE_X - TABLE_SIZE[0] / 2
RING_OVERHANG = .030                                     # ring centre in front of the table's back edge
RING_START = (TABLE_BACK_X + RING_OVERHANG, TABLE_Y, TABLE_TOP_Z + RING_TUBE_D / 2)

# Scoring.
TOUCH_MARGIN = .001
WORKSPACE = ((-.40, -1.40, -.01), (2.20, 1.00, 1.80))

INSTRUCTION_FALLBACK = (
    "Play ring toss: pick up the pink ring from the small table and land it over the red centre peg of the "
    "ring toss game. Stay behind the blue tape line."
)


def peg_positions(set_pose_xy=(SET_X, SET_Y)):
    """Peg axes (x, y) in the set's own frame order: centre, front, back, left, right."""
    x, y = set_pose_xy
    return [(x, y), (x + PEG_OFFSET, y), (x - PEG_OFFSET, y), (x, y + PEG_OFFSET), (x, y - PEG_OFFSET)]


def around_peg(ring_pos, ring_axis, peg_base, peg_top):
    """True when the peg's axis passes through the ring's hole below the peg's tip."""
    import numpy as np
    n = np.asarray(ring_axis, float)
    a, b = np.asarray(peg_base, float), np.asarray(peg_top, float)
    d = b - a
    denom = float(n @ d)
    if abs(denom) < 1e-6:
        return False
    s = float(n @ (np.asarray(ring_pos) - a)) / denom
    if not 0. <= s <= 1.:
        return False
    hit = a + s * d
    return bool(np.linalg.norm(hit - ring_pos) <= RING_INNER_R - PEG_D / 2)


def finger_joint_position(aperture):
    if not 0 <= aperture <= .075:
        raise ValueError('Aperture exceeds 75 mm')
    return -min(aperture, .0749) / 2
