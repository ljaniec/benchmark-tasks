"""A wooden magnetic fishing game on a children's table in front of two YAM arms; SI units.

World frame: x forward from the line between the arm bases, y to the left, z up from the floor.
"""
import math

CONTROL_HZ = 20
PHYSICS_HZ = 240
DWELL_STEPS = 20
FINGER_FORCE_LIMIT = 40.
PAD_TORSIONAL_RADIUS = .002

# Children's table.
TABLE_CENTRE = (.500, -.300)
TABLE_SIZE = (.600, .800)
TABLE_TOP_Z = .550

# Pond: a round wooden tray.
POND_CENTRE = (.560, -.230)
POND_INNER_R = .150
POND_WALL = .008
POND_WALL_H = .040                   # above the tray's floor
POND_FLOOR_T = .006
POND_FLOOR_Z = TABLE_TOP_Z + POND_FLOOR_T
RIM_Z = POND_FLOOR_Z + POND_WALL_H

# Fish: flat painted wooden fish, each with a steel washer glued on its back behind the head.
FISH_LEN = .080                      # body, nose to the tail's root
FISH_W = .040
FISH_T = .012
TAIL = (.025, .032)                  # tail length and width; the tail overlaps the body by 4 mm
WASHER_D = .020
WASHER_T = .0015
WASHER_LOCAL_X = -.009               # over the fish's balance point (the tail moves it 9 mm back)
FISH_MASS = .020
WASHER_MASS = .003
# name: (x, y, heading in degrees, colour). The red fish lies in the middle; the others lie around it,
# 105 mm out, at least 12 mm from each other and 15 mm from the wall.
FISH = {
    'red': (POND_CENTRE[0], POND_CENTRE[1], 20., (.85, .12, .10)),
    'blue': (POND_CENTRE[0] - .0458, POND_CENTRE[1] + .0951, -160., (.12, .35, .85)),
    'green': (POND_CENTRE[0] + .0800, POND_CENTRE[1] + .0688, 125., (.15, .65, .25)),
    'yellow': (POND_CENTRE[0] - .0260, POND_CENTRE[1] - .1023, -20., (.95, .80, .10)),
    'orange': (POND_CENTRE[0] + .0872, POND_CENTRE[1] - .0594, 50., (.95, .50, .10)),
}
TARGET = 'red'

# Rod: a flat wooden stick, 280 mm long, 15 mm wide and 8 mm thick, with a 200 mm cord and a 16 x 8 mm
# magnet. It lies flat along x on the game's box, its handle end 45 mm past the box's back edge; the
# magnet rests on the table.
BOX_CENTRE = (.370, -.560)
BOX_SIZE = (.200, .120, .060)
BOX_TOP_Z = TABLE_TOP_Z + BOX_SIZE[2]
ROD_LEN = .280
ROD_W = .015
ROD_T = .008
ROD_MASS = .020
HANDLE_OVERHANG = .045
ROD_START = (BOX_CENTRE[0] - BOX_SIZE[0] / 2 - HANDLE_OVERHANG + ROD_LEN / 2, BOX_CENTRE[1], BOX_TOP_Z + ROD_T / 2)
CORD_LEN = .200
CORD_D = .002
MAGNET_D = .016
MAGNET_T = .008
MAGNET_MASS = .006
MAGNET_LINEAR_DAMPING = .30          # air drag and cord friction, roughly
# PhysX distance joints engage only this far beyond their maximum distance, so the authored limit is
# shortened by it.
JOINT_TOLERANCE = .025
MAGNET_START = (ROD_START[0] + ROD_LEN / 2 + .050, BOX_CENTRE[1], TABLE_TOP_Z + MAGNET_T / 2)

# Magnet pull on a washer: F = PULL / (1 + gap / PULL_SCALE)^3 along the line between the magnet's face
# and the washer, zero beyond PULL_RANGE. PULL is the pull-off force at contact.
PULL = 2.0
PULL_SCALE = .004
PULL_RANGE = .025

WOOD_STATIC = .45
WOOD_DYNAMIC = .40

# Scoring.
TOUCH_MARGIN = .001
CORD_PROBES = 100                    # spheres every 2 mm along the drawn cord find robot colliders touching it
ATTACHED_GAP = .002
LIFT_OFF = .020
ABOVE_RIM = .050
WORKSPACE = ((-.40, -1.40, 0.), (1.40, 1.00, 1.60))
FISH_MIN_Z = TABLE_TOP_Z - .050      # a fish lower than this has fallen off the table

INSTRUCTION_FALLBACK = (
    "Play the fishing game: pick up the fishing rod and, touching only the rod, catch the red fish on its "
    "magnet and lift the fish up out of the pond."
)


def washer_start(name):
    """Centre of the top of a fish's washer in the starting layout."""
    x, y, heading, _ = FISH[name]
    h = math.radians(heading)
    return (x + WASHER_LOCAL_X * math.cos(h), y + WASHER_LOCAL_X * math.sin(h), POND_FLOOR_Z + FISH_T + WASHER_T)


def magnet_pull(gap):
    if gap >= PULL_RANGE:
        return 0.
    return PULL / (1 + max(gap, 0.) / PULL_SCALE) ** 3


def finger_joint_position(aperture):
    if not 0 <= aperture <= .075:
        raise ValueError('Aperture exceeds 75 mm')
    return -min(aperture, .0749) / 2
