"""An assembled desktop laminator with an external nip release."""
import math

import numpy as np
from yam_arena.station import TABLE_TOP

import mechkit as m


RISER_HEIGHT = .043
ORIGIN = np.array((.410, .010, TABLE_TOP + RISER_HEIGHT))
WIDTH, DEPTH, HEIGHT = .356, .158, .067
ENTRY = -.079
NIP_X = -.046
PATH_Z = .033
POUCH_LENGTH, POUCH_WIDTH, COLLISION_THICKNESS = .164, .117, .002
INSERTION = .061
POUCH_START = np.array((ENTRY + INSERTION - POUCH_LENGTH / 2., 0., PATH_Z))
HINGE = np.array((.005, .120, .045))
PRESS_PAD = np.array((.095, .120, .060))
LAYDOWN = ORIGIN + np.array((.039, -.395, -RISER_HEIGHT))
ROTATION = np.array(((0., -1., 0.), (1., 0., 0.), (0., 0., 1.)))

LOOKS = {
    'body': m.Look(m.rgb(.87, .86, .79), roughness=.52),
    'trim': m.Look(m.rgb(.16, .23, .24), roughness=.68),
    'roller': m.Look(m.rgb(.20, .19, .17), roughness=.91),
    'switch': m.Look(m.rgb(.65, .22, .11), roughness=.60),
    'film': m.Look(m.rgb(.65, .83, .84), roughness=.21),
    'print': m.Look(m.rgb(.92, .60, .24), roughness=.56),
    'fold': m.Look(m.rgb(.21, .60, .66), roughness=.44),
    'mark': m.Look(m.rgb(.70, .40, .23), roughness=.45),
}

BODY = (
    ('base', (.158, .356, .019), (0., 0., .0095), 'trim'),
    ('top_shell', (.118, .356, .018), (-.020, 0., .0575), 'body'),
    ('rear_shell_main', (.040, .263, .018), (.059, -.0465, .0575), 'body'),
    ('rear_shell_end', (.040, .018, .018), (.059, .169, .0575), 'body'),
    ('right_cap', (.158, .058, .031), (0., -.149, .034), 'body'),
    ('left_cap', (.158, .058, .031), (0., .149, .034), 'body'),
    ('entry_lip', (.017, .240, .009), (-.0705, 0., .0235), 'trim'),
    ('upper_entry_bezel', (.016, .240, .011), (-.071, 0., .045), 'trim'),
    ('rear_lower_bezel', (.017, .240, .009), (.0705, 0., .0235), 'trim'),
    ('rear_upper_bezel', (.016, .240, .011), (.071, 0., .042), 'trim'),
)


def world(local):
    return ORIGIN + ROTATION @ local


def solid(rows):
    visual = tuple((n, m.box(size, at), look) for n, size, at, look in rows)
    contact = tuple(m.col_box(n + '_contact', size, at) for n, size, at, _ in rows)
    return visual, contact


def laminator():
    shell, collisions = solid(BODY)
    lower = m.cylinder(.0097, .240, (NIP_X, 0., .0220), axis='y', segments=32)
    higher = m.cylinder(.0097, .240, (NIP_X, 0., .0429), axis='y', segments=32)
    paddle = (.034, .026, .014)
    lever_parts, lever_collision = solid((('release_paddle', paddle, tuple(PRESS_PAD), 'switch'),))
    # Rear paddle down, front end of the pressure arm up.
    release = m.hinge('release', 'Y', tuple(HINGE), -2., 12.,
                      lever_parts + (('upper_roller', higher, 'roller'),),
                      lever_collision + (m.col_hull('upper_roller_contact', higher),),
                      spring=5.2, damping=.08, effort=3.2, friction=.022,
                      mass=.092, armature=4.e-5)
    indicator = m.box((.013, .002, .006), (.076, .136, .058))
    return m.mechanism('laminator', ORIGIN, (release,), LOOKS,
                       shell + (('lower_roller', lower, 'roller'), ('release_reference', indicator, 'mark')),
                       collisions + (m.col_hull('lower_roller_contact', lower),),
                       yaw_deg=90., self_collide=False, friction=(.69, .51))


# A buckled, unsealed end gives both fingers a broad face to grasp.
FOLD_GRIP = np.array((-.090, 0., .0105))
FOLD_WIDTH = .035
FOLD = m.prism(((-.101, .0007), (-.077, .0007), (-.081, .0165),
                (-.087, .0195), (-.097, .0195), (-.101, .0145)),
               -FOLD_WIDTH / 2, FOLD_WIDTH / 2).turned(90., "x")


def pouch():
    sheet = m.box((POUCH_LENGTH, POUCH_WIDTH, COLLISION_THICKNESS))
    insert = m.box((.148, .105, .0003), (0., 0., COLLISION_THICKNESS / 2. + .0002))
    seam = m.box((.003, .117, .00035), (.080, 0., .00125))
    ridge_size = (.009, .075, .0026)
    ridge_center = (.069, 0., .0021)
    ridge = m.box(ridge_size, ridge_center)
    contacts = (
        m.col_box('pouch_contact', (POUCH_LENGTH, POUCH_WIDTH, COLLISION_THICKNESS)),
        m.col_box('creased_leading_lip_contact', ridge_size, ridge_center),
        m.col_hull('outer_fold_contact', FOLD),
    )
    return m.free('pouch', world(POUCH_START), (('pouch', sheet, 'film'), ('insert', insert, 'print'),
                  ('sealed_edge', seam, 'mark'), ('creased_leading_lip', ridge, 'film'),
                  ('outer_fold', FOLD, 'fold')), contacts, LOOKS, .009,
                  friction=(.66, .47), yaw_deg=90.)


def rest_marks():
    pieces = []
    for x in (-1., 1.):
        for y in (-1., 1.):
            for axis, direction in enumerate(((x, 0.), (0., y))):
                at = np.array((x * .079, y * .112, .00025))
                at[:2] -= .008 * np.asarray(direction)
                shape = (.018, .003, .0005) if axis == 0 else (.003, .018, .0005)
                pieces.append((f'mark_{len(pieces)}', m.box(shape, at), 'mark'))
    return m.fixed('pouch_rest_marks', LAYDOWN, tuple(pieces), looks=LOOKS)


SUPPORT_HEIGHT = PATH_Z - COLLISION_THICKNESS / 2. - .0003
ENTRY_GUIDES = tuple((f"entry_guide_{i}", (.205, .008, .009),
                      (-.195, side * .051, SUPPORT_HEIGHT - .0045), "body")
                     for i, side in enumerate((-1., 1.)))


def recovery_support():
    # Two low shelf edges hold the exposed tail with clear access between them.
    deck = (*ENTRY_GUIDES, *tuple((f"guide_foot_{i}", (.185, .014, SUPPORT_HEIGHT - .009 + RISER_HEIGHT),
                                 (-.195, side * .051, (SUPPORT_HEIGHT - .009 - RISER_HEIGHT) / 2.), "trim")
                                for i, side in enumerate((-1., 1.))))
    visible, contacts = solid(deck)
    return m.fixed("recovery_support", ORIGIN, visible, contacts, LOOKS,
                   yaw_deg=90., friction=(.34, .25))


RISER_BODY = (("laminator_riser", (.148, .340, RISER_HEIGHT), (0., 0., -.5 * RISER_HEIGHT), "trim"),)


def riser():
    parts, contacts = solid(RISER_BODY)
    return m.fixed("riser", ORIGIN, parts, contacts, LOOKS, yaw_deg=90.)


OBJECTS = (laminator(), pouch(), rest_marks(), recovery_support(), riser())


def marker_drop_mm(degrees):
    a = math.radians(degrees)
    pivot_to_paddle = PRESS_PAD - HINGE
    return 1000. * (pivot_to_paddle[0] * math.sin(a) + pivot_to_paddle[2] * (1. - math.cos(a)))
