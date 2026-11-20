"""Evaluator-only geometry. Nothing in this module is a policy input."""
import math
import numpy as np
from scorer import Evidence
import spec

def evaluate(w):
    from pxr import Gf
    import carb
    from omni.physx import get_physx_scene_query_interface
    query=get_physx_scene_query_interface()
    nm=w.matrix(w.path('/World/nut'));cm=w.matrix(w.path('/World/collet'))
    npos=np.asarray(nm.ExtractTranslation());cpos=np.asarray(cm.ExtractTranslation())
    nr=np.asarray(nm.ExtractRotationMatrix()).T;cr=np.asarray(cm.ExtractRotationMatrix()).T
    relative=nr.T@(cpos-npos);axis_angle=math.degrees(math.acos(float(np.clip(np.dot(nr[:,2],cr[:,2]),-1,1))))
    tilt=math.degrees(math.acos(float(np.clip(nr[2,2],-1,1))))
    radial=float(np.linalg.norm(relative[:2]))
    entered=radial<=.006 and -.005<=relative[2]<=.020 and axis_angle<=35.
    seated=radial<=.0015 and .0015<=relative[2]<=.0048 and axis_angle<=5.
    def near_robot(matrix,center,half):
        hits=[];q=matrix.ExtractRotationQuat();c=matrix.Transform(Gf.Vec3d(*center))
        def hit(h):
            if 'Arm/' in str(h.rigid_body):hits.append(str(h.rigid_body))
            return True
        query.overlap_box(carb.Float3(*map(float,half)),carb.Float3(*map(float,c)),
                          carb.Float4(*map(float,q.GetImaginary()),float(q.GetReal())),hit,False)
        return sorted(set(hits))
    collet_near=near_robot(cm,(0,0,.020),(.0215,.0215,.025))
    nut_near=near_robot(nm,(0,0,spec.NUT_HEIGHT/2),(.034,.034,spec.NUT_HEIGHT/2+.005))
    bounds=[w.bounds(w.path(path)) for path in w.objects.values()]
    lo=np.min([b[0] for b in bounds],axis=0);hi=np.max([b[1] for b in bounds],axis=0)
    inside=bool(np.all(lo>=spec.WORKSPACE[0]) and np.all(hi<=spec.WORKSPACE[1]))
    # Clears every supporting surface, including the service block.
    raised=lo[2]>=spec.SUPPORT_TOP+.050
    parked=bool(np.all(lo[:2]>=np.array(spec.PAD[:2])-.045) and np.all(hi[:2]<=np.array(spec.PAD[:2])+.045)
                and abs(lo[2]-(spec.PAD[2]+.001))<=.003)
    displacements=[]
    for i in range(spec.NOSE_SECTORS):
        pos=np.asarray(w.matrix(w.path(f'/World/nose_{i}')).ExtractTranslation())
        displacements.append((cr.T@(pos-cpos)).tolist())
    intact=all(np.linalg.norm(x)<.002 for x in displacements)
    table=w.matrix(w.path('/World/table'));up=np.asarray(table.ExtractRotationMatrix())[2]
    fixture_tilt=math.degrees(math.acos(float(np.clip(up[2]/np.linalg.norm(up),-1,1))))
    shortcut=any(np.linalg.norm(np.asarray(w.matrix(w.path('/World/'+name)).ExtractTranslation())-pos)>.002
                 for name,pos in w.fixed_positions.items())
    e=Evidence(bool(entered),bool(seated),not collet_near,bool(raised),tilt,parked,not(collet_near or nut_near),
               fixture_tilt,inside,bool(intact),bool(shortcut))
    diagnostics={'nut_xyz':npos.tolist(),'collet_xyz':cpos.tolist(),'collet_relative_nut':relative.tolist(),
                 'axis_angle_deg':axis_angle,'radial_offset_m':radial,'nose_displacements':displacements,
                 'nut_matrix':np.asarray(nm).tolist(),'collet_matrix':np.asarray(cm).tolist(),
                 'collet_near_robot':collet_near,'nut_near_robot':nut_near,
                 'pad_centers_usd':[list(w.matrix(p).ExtractTranslation()) for p in w.pads]}
    return e,diagnostics
