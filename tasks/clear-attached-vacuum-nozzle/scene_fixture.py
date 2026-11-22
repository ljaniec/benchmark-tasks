"""Dimensioned nozzle and complete vacuum layout for a clearance check."""
import math

import numpy as np
from yam_arena.ik import matrix_to_quat_xyzw
from yam_arena.station import TABLE_TOP
from yam_arena.urdf import axis_angle_matrix

import mechkit as m


PIVOT = np.array((.420,-.045,TABLE_TOP+.052))
HEAD_WIDTH = .280
START_PITCH_DEG = 67.
MOUTH_X = -.074
MOUTH_WIDTH = .088
LIP_GAP = .020
CARD_WIDTH = .062
CARD_CENTER = np.array((MOUTH_X,0.,-.055))
CARD_LENGTH = .058
CARD_FOLD_WIDTH = .023
Y_AXIS = np.array((0.,1.,0.))
LOOKS = {
    'body': m.Look(m.rgb(.56,.15,.18),roughness=.56),
    'sole': m.Look(m.rgb(.17,.20,.22),roughness=.62),
    'flex': m.Look(m.rgb(.25,.29,.30),roughness=.82),
    'metal': m.Look(m.rgb(.65,.71,.74),roughness=.29,metallic=.72),
    'hose': m.Look(m.rgb(.17,.19,.21),roughness=.76),
    'flake': m.Look(m.rgb(.90,.68,.32),roughness=.84),
    'rubber': m.Look(m.rgb(.24,.29,.25),roughness=.95),
}


def pitch(degrees):
    return axis_angle_matrix(Y_AXIS,math.radians(degrees))


def solids(rows):
    return (tuple((name,m.box(size,center),look) for name,size,center,look in rows),
            tuple(m.col_box(f'{name}_contact',size,center) for name,size,center,_ in rows))


def canister():
    center=(.652,PIVOT[1]-.359,TABLE_TOP)
    body=m.cylinder(.143,.231,(0.,0.,.144),segments=32)
    base=m.cylinder(.150,.041,(0.,0.,.037),segments=32)
    cap=m.cylinder(.146,.027,(0.,0.,.271),segments=32)
    grip=m.tube(((-.067,0.,.279),(-.061,0.,.308),(.061,0.,.308),(.067,0.,.279)),.014)
    hose_port=m.cylinder(.027,.026,(0.,.143,.116),axis='y')
    wheel_l=m.cylinder(.038,.022,(.081,.138,.039),axis='y')
    wheel_r=m.cylinder(.038,.022,(.081,-.138,.039),axis='y')
    visuals=(('canister',body,'body'),('lower_trim',base,'sole'),('motor_cap',cap,'sole'),
             ('carry_handle',grip,'sole'),('hose_collar',hose_port,'hose'),
             ('left_rear_wheel',wheel_l,'sole'),('right_rear_wheel',wheel_r,'sole'))
    colliders=(m.col_cylinder('canister_contact',.150,.276,(0.,0.,.143)),m.col_hull('handle_contact',grip))
    return m.fixed('canister',center,visuals,colliders,LOOKS)


def vacuum():
    plastic, plastic_c = solids((
        ('head_shroud',(.122,HEAD_WIDTH,.024),(-.061,0.,-.013),'sole'),
        ('front_skirt',(.037,HEAD_WIDTH,.020),(-.104,0.,-.044),'flex'),
        ('rear_skirt',(.036,HEAD_WIDTH,.020),(-.036,0.,-.044),'flex'),
        ('left_edge',(.032,.096,.018),(MOUTH_X,+.092,-.043),'sole'),
        ('right_edge',(.032,.096,.018),(MOUTH_X,-.092,-.043),'sole'),
        ('upper_shoulder',(.030,.028,.026),(-.093,.117,.011),'sole'),
        ('left_bumper',(.021,.049,.015),(-.093,.117,.025),'rubber'),
    ))
    neck=m.cylinder(.025,.052,(-.008,0.,-.010),axis='y')
    head=m.hinge('head','Y',(0.,0.,0.),0.,86.,plastic+(('neck',neck,'sole'),),
                 plastic_c+(m.col_hull('neck_contact',neck),),mass=.38,gravity=True,
                 spring=0.,damping=.05,friction=.21,effort=.78,inertia=(.0035,.0015,.0040))

    assembly=[]
    for name,sign,limits in (('front_lip',-1.,(-.008,0.)),('rear_lip',1.,(0.,.008))):
        center=(MOUTH_X+sign*(LIP_GAP/2.+.0015),0.,-.047)
        visual,colliders=solids(((name,(.003,MOUTH_WIDTH-.008,.010),center,'rubber'),))
        assembly.append(m.slider(name,'X',*limits,visual,colliders,parent='head',
                                 mass=.019,spring=1700.,damping=1.8,effort=9.,friction=.02))

    # The full hose connects the visible appliance to the wand behind the work area.
    wand=m.cylinder(.016,.431,(.216,0.,0.),axis='x')
    elbow=m.tube(((.420,0.,0.),(.461,0.,.004),(.477,-.034,.028),(.460,-.125,.093),
                  (.406,-.202,.110),(.335,-.230,.091),(.256,-.216,.066),(.232,-.206,.064)),.020,16)
    collar=m.cylinder(.024,.052,(.022,0.,0.),axis='x')
    fixed=(('wand',wand,'metal'),('flexible_hose',elbow,'hose'),('wand_collar',collar,'sole'))
    fixed_c=(m.col_hull('wand_contact',wand),m.col_hull('collar_contact',collar))
    return m.mechanism('connected_floor_tool',PIVOT,(head,*assembly),LOOKS,fixed,fixed_c,
                       friction=(.53,.43),self_collide=False,start=(('head',math.radians(START_PITCH_DEG)),))


def card_mesh():
    height=CARD_LENGTH/2.
    width=CARD_FOLD_WIDTH/2.
    # A measured fold envelope. Native retention and the real release force still need testing.
    outline=((-width,0.),(-.004,-height),(.004,-height),(width,0.),(.005,height),(-.005,height))
    return m.prism(outline,-CARD_WIDTH/2.,CARD_WIDTH/2.).turned(90.,'x')


def obstruction():
    mesh=card_mesh()
    # Map the profile to the insertion axis, preserving the width across the nozzle.
    world_rotation=pitch(START_PITCH_DEG)
    position=PIVOT+world_rotation@CARD_CENTER
    return m.free('folded_paperboard',position,(('card',mesh,'flake'),),(m.col_hull('card_contact',mesh),),
                  LOOKS,.006,friction=(.47,.33),start_quat=tuple(matrix_to_quat_xyzw(world_rotation)))


OBJECTS=(canister(),vacuum(),obstruction())
