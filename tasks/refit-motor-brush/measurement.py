"""Evaluator-only geometric checks; no readings are sent to the policy."""
import math
import numpy as np
import spec
from scorer import Evidence

def evaluate(w):
    from pxr import Gf
    import carb
    from omni.physx import get_physx_scene_query_interface
    query=get_physx_scene_query_interface()
    bm=w.matrix(w.path('/World/brush'));fm=w.matrix(w.path('/World/finger'))
    bp=np.array(bm.ExtractTranslation());br=np.array(bm.ExtractRotationMatrix()).T
    fp=np.array(fm.ExtractTranslation());fr=np.array(fm.ExtractRotationMatrix()).T
    offset=bp-np.array(spec.CENTER);tilt=math.degrees(math.acos(float(np.clip(br[2,2],-1,1))))
    depth=spec.BOX_GAP+spec.BOX_HEIGHT-offset[2]
    centered=bool(np.all(abs(offset[:2])<.001))
    entered=centered and tilt<4 and depth>=.010 and offset[2]>=-.001
    gaps=[]
    for x in [-spec.BRUSH_T/2,0,spec.BRUSH_T/2]:
        z=math.sqrt(spec.COLLECTOR_RADIUS**2-x*x)-spec.COLLECTOR_RADIUS
        for y in [-spec.BRUSH_A/2,spec.BRUSH_A/2]:
            p=bp+br@np.array([x,y,z]);dx=p[0]-spec.CENTER[0]
            surface=spec.CENTER[2]-spec.COLLECTOR_RADIUS+math.sqrt(max(0,spec.COLLECTOR_RADIUS**2-dx*dx))
            gaps.append(float(p[2]-surface))
    seated=entered and max(gaps)<=.0005 and min(gaps)>=-.0005 and tilt<1.
    toe=br.T@(fp+fr@np.array([.060,0,-.032])-bp)
    toe_corners=[br.T@(fp+fr@np.array([x,y,-.032])-bp) for x in [.0535,.0665] for y in [-.008,.008]]
    toe_gaps=[float(p[2]-spec.BRUSH_R) for p in toe_corners]
    finger_angle=math.degrees(math.atan2(fr[0,2],fr[0,0]))
    pressing=all(abs(p[0])<spec.BRUSH_T/2+.0001 and abs(p[1])<spec.BRUSH_A/2+.0001 for p in toe_corners) and min(toe_gaps)>=-.0005 and min(toe_gaps)<=.0005 and abs(finger_angle)<5.
    def near(matrix,center,half):
        hits=[];q=matrix.ExtractRotationQuat();c=matrix.Transform(Gf.Vec3d(*center))
        def hit(h):
            if 'Arm/' in str(h.rigid_body):hits.append(str(h.rigid_body))
            return True
        query.overlap_box(carb.Float3(*map(float,half)),carb.Float3(*map(float,c)),carb.Float4(*map(float,q.GetImaginary()),float(q.GetReal())),hit,False)
        return sorted(set(hits))
    brush_near=near(bm,(0,0,.025),(.0175,.021,.0315))
    finger_near=near(fm,(.034,0,-.015),(.038,.013,.022))
    lo,hi=w.bounds(w.path('/World/brush'))
    inside=bool(np.all(lo>=spec.WORKSPACE[0]) and np.all(hi<=spec.WORKSPACE[1]))
    hm=w.matrix(w.path('/World/housing'));up=np.array(hm.ExtractRotationMatrix())[2]
    chassis=math.degrees(math.acos(float(np.clip(up[2]/np.linalg.norm(up),-1,1))))
    intact=bool(np.linalg.norm(fp-np.array(spec.PIVOT))<.001 and -86<finger_angle<24)
    shortcut=any(np.linalg.norm(np.array(w.matrix(w.path('/World/'+name)).ExtractTranslation())-pos)>.002 for name,pos in w.fixed_positions.items())
    e=Evidence(bool(entered),bool(seated),bool(pressing),not(brush_near or finger_near),chassis,inside,intact,bool(shortcut))
    diagnostics={'brush_xyz':bp.tolist(),'brush_offset':offset.tolist(),'brush_tilt_deg':tilt,'insertion_depth_m':depth,
        'collector_gaps_m':gaps,'toe_in_brush':toe.tolist(),'toe_gaps_m':toe_gaps,'finger_angle_deg':finger_angle,
        'brush_matrix':np.array(bm).tolist(),'finger_matrix':np.array(fm).tolist(),'brush_near_robot':brush_near,'finger_near_robot':finger_near,
        'pad_centers_usd':[list(w.matrix(p).ExtractTranslation()) for p in w.pads]}
    return e,diagnostics
