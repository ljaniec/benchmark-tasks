"""Guarded sharpener housing and a contact-based cutter-collar concept."""
import math

import numpy as np
from yam_arena.station import TABLE_TOP

import mechkit as m


BORE = np.array((.408,.036,TABLE_TOP+.091))
SLEEVE_START_M = -.043
EAR_ANGLE = 8.
ENTRY_HALF_ANGLE = 17.
LOCK_ANGLE = 67.
EAR_RADIUS = (.0174,.0220)
EAR_AXIAL = (.0064,.0098)
LOOKS = {
    'green': m.Look(m.rgb(.12,.42,.37),roughness=.51),
    'polymer': m.Look(m.rgb(.16,.18,.19),roughness=.79),
    'steel': m.Look(m.rgb(.61,.66,.69),roughness=.38,metallic=.62),
    'marker': m.Look(m.rgb(.91,.70,.22),roughness=.61),
}


def solids(rows):
    return (tuple((name,m.box(size,at),look) for name,size,at,look in rows),
            tuple(m.col_box(f'{name}_contact',size,at) for name,size,at,_ in rows))


def arc_band(radii, start_deg, finish_deg, x0, x1):
    # A convex annular segment extruded along the cartridge's axial direction.
    a,b=np.radians([start_deg,finish_deg]); lo,hi=radii
    ring=[(z*math.cos(th),z*math.sin(th)) for z,th in ((lo,a),(hi,a),(hi,b),(lo,b))]
    # The prism's first two axes map to the housing's Y and Z axes.
    return m.prism([(-u,v) for v,u in ring],x0,x1).turned(90.,'y')


def jaw_outline():
    frame, frame_collisions=solids((
        ('top',(.118,.075,.035),(.055,0.,.042),'green'),
        ('left_wall',(.118,.017,.114),(.055,.031,-.008),'green'),
        ('right_wall',(.118,.017,.114),(.055,-.031,-.008),'green'),
        ('lower_panel',(.118,.076,.025),(.055,0.,-.078),'green'),
        ('shavings_drawer',(.073,.050,.040),(.057,0.,-.048),'polymer'),
        ('mount_foot',(.130,.083,.007),(.058,0.,-.0875),'polymer'),
        ('rear_axle_wall',(.010,.077,.104),(.116,0.,-.012),'green'),
    ))
    # Two coarse cam tracks stand in for the original screw-secured collar.
    arcs=[]
    for axis in (0.,180.):
        for k in range(6):
            at=axis+ENTRY_HALF_ANGLE+k*12.
            back=.0032+.00054*k
            arcs.append((f'track_{int(axis)}_{k}',arc_band((.0168,.0231),at,at+12.2,-.002,back),'polymer'))
    arc_collisions=tuple(m.col_hull(f'{name}_contact',shape) for name,shape,_ in arcs)
    return frame+tuple(arcs),frame_collisions+arc_collisions


def cartridge():
    frame,frame_collisions=jaw_outline()
    guide=m.cylinder(.0142,.061,(.042,0.,0.),axis='x')
    sleeve=m.slider('cutter_sleeve','X',SLEEVE_START_M,.001,
                    (('cutter_guard',guide,'steel'),),(m.col_hull('cutter_guard_contact',guide),),
                    damping=1.2,mass=.088,effort=23.,friction=.04,armature=1.e-6)
    grip=m.cylinder(.022,.016,(-.011,0.,0.),axis='x',segments=12)
    hub=m.cylinder(.0162,.030,(-.0035,0.,0.),axis='x')
    ears=tuple((f'collar_ear_{i}',arc_band(EAR_RADIUS,a-EAR_ANGLE,a+EAR_ANGLE,*EAR_AXIAL),'polymer')
               for i,a in enumerate((0.,180.)))
    collar_parts=(('black_collar',grip,'polymer'),('ring_pilot',hub,'polymer'),*ears)
    collar_collision=tuple(m.col_hull(f'{name}_contact',part) for name,part,_ in collar_parts)
    lock=m.hinge('lock_collar','X',(0.,0.,0.),-7.,76.,collar_parts,collar_collision,parent='cutter_sleeve',mass=.033,
                  friction=.021,damping=.012,effort=.22,armature=1.e-6,inertia=(1.1e-5,9.e-6,9.e-6))
    crank, crank_col=solids((
        ('crank_web',(.012,.016,.061),(-.033,0.,-.022),'steel'),
        ('crank_knob',(.035,.022,.022),(-.043,0.,-.051),'polymer'),
        ('reference_dot',(.013,.008,.006),(-.033,.010,-.030),'marker'),
    ))
    shaft=m.cylinder(.006,.104,(.017,0.,0.),axis='x')
    axle=m.hinge('crank','X',(0.,0.,0.),-375.,375.,crank+(('drive_shaft',shaft,'steel'),),
                  crank_col,parent='cutter_sleeve',mass=.066,
                  friction=.026,damping=.018,effort=.36,armature=1.e-6,inertia=(4.e-5,4.e-5,7.e-6))
    return m.mechanism('mounted_sharpener',BORE,(sleeve,lock,axle),LOOKS,frame,frame_collisions,
                       yaw_deg=0.,friction=(.55,.38),self_collide=True,
                       start=(('cutter_sleeve',SLEEVE_START_M),))


def geometric_witness():
    free_at_entry=EAR_ANGLE < ENTRY_HALF_ANGLE and EAR_AXIAL[0]+SLEEVE_START_M < -.002
    covers_lock=(LOCK_ANGLE-EAR_ANGLE>=ENTRY_HALF_ANGLE
                 and LOCK_ANGLE+EAR_ANGLE<=ENTRY_HALF_ANGLE+72.)
    release_before_cam=EAR_AXIAL[1]-abs(SLEEVE_START_M)<-.002
    final_ear_gap=EAR_AXIAL[0]-(.0032+.00054*5)
    return dict(open_entry=free_at_entry,quarters_capture_ears=covers_lock,
                staging_is_outside_tracks=release_before_cam,final_modelled_axial_play_mm=1000.*final_ear_gap)


OBJECTS=(cartridge(),)
