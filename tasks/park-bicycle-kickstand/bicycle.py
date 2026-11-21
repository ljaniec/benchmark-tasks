"""Procedural full-size bicycle, weight-bearing deck and passive starting support."""
from dataclasses import replace
from math import cos, pi, sin

import numpy as np

import mechkit as m
from preflight import (BRACE_POINT, CENTER_OF_MASS, DECK_HEIGHT_M, FOLDED, FRAME_CENTER,
                       MASS_KG, SHOE_RADIUS_M, STAND_LENGTH_M, STAND_PIVOT, SWEEP_AXIS, SWEEP_RAD, TIRE_RADIUS_M,
                       TIRE_SECTION_M, WHEELBASE_M)
from yam_arena.urdf import axis_angle_matrix
from yam_arena import ik

LOOKS = {name: m.Look(m.rgb(*color), roughness=roughness) for name, color, roughness in (
    ('frame', (.25,.49,.53), .43), ('rim', (.69,.70,.71), .39),
    ('rubber', (.13,.14,.15), .81), ('stand', (.68,.71,.73), .57),
    ('deck', (.76,.69,.52), .77), ('bumper', (.29,.36,.41), .83),
    ('accent', (.9,.65,.16), .63),
)}
START_LEAN = np.deg2rad(5.)
BUMP_FACE = float(FRAME_CENTER[0] + BRACE_POINT[2] * sin(START_LEAN) + .018 * cos(START_LEAN))
FRONT = np.array([0., WHEELBASE_M/2, TIRE_RADIUS_M])
REAR = np.array([0., -WHEELBASE_M/2, TIRE_RADIUS_M])
PARK_ZONE = (.10,.74,-.90,.90)
STAND_MASS = .295
WHEEL_MASS = 1.55


def geometry(parts):
    return tuple((name, mesh, look) for name, mesh, look in parts), tuple(
        m.col_hull(name + '_collision', mesh) for name, mesh, _ in parts
    )


def parking_deck():
    base = m.box((.92, 1.99, .040), (.40, 0., DECK_HEIGHT_M - .020))
    tire_lines = [(f'tire_{i}', m.box((.094,.010,.0002), (.39, y, DECK_HEIGHT_M+.0001)), 'accent')
                  for i, y in enumerate((REAR[1],FRONT[1]))]
    xa,xb,ya,yb = PARK_ZONE
    mark_z = DECK_HEIGHT_M + .00015
    for i, y in enumerate((ya,yb)):
        tire_lines.append((f'boundary_y_{i}',m.box((xb-xa,.006,.0003),((xa+xb)/2.,y,mark_z)),'accent'))
    for i, x in enumerate((xa,xb)):
        tire_lines.append((f'boundary_x_{i}',m.box((.006,yb-ya,.0003),(x,(ya+yb)/2.,mark_z)),'accent'))
    posts = []
    for i, y in enumerate((-.305,.305)):
        posts.append((f'arm_pedestal_{i}', m.box((.24,.24,.60), (-.05,y,.445)), 'bumper'))
    bumper = m.box((.065,.235,.068), (BUMP_FACE+.0325,BRACE_POINT[1],DECK_HEIGHT_M+.731))
    wall = m.box((.068,.20,.76), (BUMP_FACE+.09,BRACE_POINT[1],DECK_HEIGHT_M+.38))
    visuals, collisions = geometry([('parking_slab',base,'deck'),('rubber_bumper',bumper,'bumper'),
                                   ('wall_support',wall,'deck'),*posts])
    return m.fixed('parking_deck',(0.,0.,0.),[*visuals,*tire_lines],collisions,LOOKS,friction=(.88,.78))


def wheel(name, center, parent=None):
    angles = np.linspace(-pi, pi, 19)
    tire = m.lathe([(TIRE_RADIUS_M-TIRE_SECTION_M + TIRE_SECTION_M*cos(a), TIRE_SECTION_M*sin(a))
                    for a in angles], 64).aimed('+X').moved(center)
    hub = m.cylinder(.026,.074,center,axis='x')
    rim = m.lathe([(.289,-.016),(.311,-.016),(.311,.016),(.289,.016),(.289,-.016)],64).aimed('+X').moved(center)
    spokes = [(f'spoke_{i:02d}',m.tube([center,center+[0.,.294*cos(a),.294*sin(a)]],.0018, sides=6),'rim')
              for i, a in enumerate(np.linspace(0.,2*pi,18,endpoint=False))]
    ixx = WHEEL_MASS*.31**2
    return m.hinge(name,'X',center,-np.inf,np.inf,
                   [('tire',tire,'rubber'),('hub',hub,'rim'),('rim',rim,'rim'),*spokes],
                   [m.col_hull('tire_collision',tire)], parent=parent, mass=WHEEL_MASS,
                   damping=.015, gravity=True, inertia=(ixx,ixx/2,ixx/2))


def stand_link():
    tip = STAND_PIVOT + FOLDED * (1. - SHOE_RADIUS_M / STAND_LENGTH_M)
    shoe = m.sphere(SHOE_RADIUS_M,tip)
    leg = m.tube([STAND_PIVOT,tip],.010)
    collar = m.cylinder(.018,.023,STAND_PIVOT,axis='x')
    visuals, collisions = geometry([('leg',leg,'stand'),('shoe',shoe,'rubber'),('pivot',collar,'stand')])
    return m.hinge('kickstand','X',STAND_PIVOT,0.,np.degrees(SWEEP_RAD),visuals,collisions,
                   mass=STAND_MASS, damping=.045, gravity=True, effort=4., armature=.0005)


def oblique_joint(usd, joint, axis):
    """Orient both native joint frames so local X follows the hinge pin."""
    source = np.array([1.,0.,0.])
    normal = np.cross(source, axis)
    rotation = axis_angle_matrix(normal / np.linalg.norm(normal), np.arccos(float(source @ axis)))
    x,y,z,w = ik.matrix_to_quat_xyzw(rotation)
    marker = f'def PhysicsRevoluteJoint "{joint}"'
    before, found, tail = usd.partition(marker)
    assert found
    block, close, after = tail.partition('\n    }')
    assert close and block.count('(1, 0, 0, 0)') == 2
    return before + found + block.replace('(1, 0, 0, 0)', f'({w:.8f}, {x:.8f}, {y:.8f}, {z:.8f})') + close + after


def bicycle():
    crank = np.array([0., -.145, .286])
    seat_top = np.array([0., -.245, .786])
    head_top = np.array([0., .385, .714])
    head_bottom = np.array([0., .439, .560])
    frame_parts = [('top_tube',m.tube([seat_top,(0.,.102,.736),head_top],.018),'frame'),
                   ('down_tube',m.tube([crank,head_bottom],.022),'frame'),
                   ('seat_tube',m.tube([crank,seat_top],.017),'frame'),
                   ('head_tube',m.tube([head_bottom,head_top],.022),'frame'),
                   ('saddle',m.box((.162,.245,.045),(0.,-.281,.872)),'rubber'),
                   ('seatpost',m.tube([seat_top,(0.,-.281,.855)],.015),'rim'),
                   ('stand_mount',m.box((.060,.070,.038),STAND_PIVOT+[.014,0.,0.]),'frame'),
                   ('crank',m.cylinder(.075,.016,crank+[.046,0.,0.],axis='x'),'rim')]
    fork = [('stem',m.tube([head_bottom,head_top,(0.,.405,.846)],.016),'rim'),
            ('handlebar',m.tube([(-.28,.375,.874),(0.,.405,.846),(.28,.375,.874)],.013),'rim')]
    for i, side in enumerate((-1.,1.)):
        offset = np.array([side*.039,0.,0.])
        frame_parts.extend([(f'seat_stay_{i}',m.tube([seat_top,REAR+offset],.011),'frame'),
                            (f'chain_stay_{i}',m.tube([crank+offset,REAR+offset],.012),'frame')])
        fork.append((f'blade_{i}',m.tube([head_bottom,head_bottom+offset,[side*.044,*FRONT[1:]]],.012),'frame'))
    visuals, collisions = geometry(frame_parts)
    crown, crown_collision = geometry(fork)
    steering = m.hinge('steering','Z',head_bottom,-55.,55.,crown,crown_collision,
                       mass=1.,gravity=True,damping=.09,friction=.08)
    steer_axis = head_top-head_bottom
    steer_axis /= np.linalg.norm(steer_axis)
    # The linkage uses a true slanted pin, represented by local X.
    steering = replace(steering,axis='X')
    root_mass = MASS_KG-STAND_MASS-2*WHEEL_MASS-1.
    lift = TIRE_SECTION_M*(1.-cos(START_LEAN))
    rear_wheel = wheel('rear',REAR)
    front_wheel = wheel('front',FRONT,parent='steering')
    kickstand = stand_link()
    bike = m.mechanism('bicycle', FRAME_CENTER+[0.,0.,lift],
                      [rear_wheel,steering,front_wheel,kickstand],LOOKS,
                      visuals,collisions,free_mass=root_mass,frame_com=CENTER_OF_MASS,
                      frame_inertia=(2.84,1.04,2.25),friction=(.88,.78),solver_iters=48,
                      start_quat=(0.,sin(START_LEAN/2),0.,cos(START_LEAN/2)))
    joint_asset = replace(bike,build=lambda: oblique_joint(oblique_joint(bike.build(),'kickstand_joint',SWEEP_AXIS),
                                                           'steering_joint',steer_axis))
    # These boxes enclose the same link geometry used for contact. Clearance
    # outside a box conservatively establishes clearance from the shape.
    parts = {'frame': frame_parts, 'steering': fork, 'rear': [rear_wheel.parts[0]],
             'front': [front_wheel.parts[0]], 'kickstand': kickstand.parts}
    bounds = {link: np.array([(mesh.v.min(axis=0),mesh.v.max(axis=0)) for _,mesh,_ in pieces])
              for link,pieces in parts.items()}
    return joint_asset,bounds
