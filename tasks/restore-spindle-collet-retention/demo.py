"""Fixed-layout RGB-only rehearsal; no live object-state feedback."""
import math
import numpy as np
from scipy.optimize import least_squares
import spec


def ry(degrees):
    a=math.radians(degrees);c,s=math.cos(a),math.sin(a)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]])


DOWN=np.array([[0.,-1.,0.],[-1.,0.,0.],[0.,0.,-1.]])
RIGHT_SIDE=np.array([[1.,0.,0.],[0.,0.,1.],[0.,-1.,0.]])


def plan(ik,constants,station):
    t=spec.TABLE_Z
    nut_origin=np.asarray(spec.NUT_START)
    collet_origin=np.asarray(spec.COLLET_START)
    # Both components start inverted. The supported collet needs no free-air
    # tapered grasp; the moving hand holds the nut's larger hexagonal exterior.
    nut_origin_in_tcp=np.array([0.,0.,-.005])
    nut_rotation_in_tcp=DOWN.T@ry(180)
    def nut_grasp(origin,angle):
        R=ry(angle)@nut_rotation_in_tcp.T
        return np.asarray(origin)-R@nut_origin_in_tcp,R
    def over_collet(offset,angle):
        return nut_grasp(collet_origin-ry(angle)@np.asarray(offset),angle)
    left_rest=np.array([.20,.22,t+.18])
    right_rest=np.array([.23,-.20,t+.18])
    right_grasp=collet_origin+[0,0,-.031]
    inspection=right_grasp+[0,-.09,0]
    pickup=nut_origin+[0,0,-.005]
    rows=[]
    def row(sec,lp,lr,lg,rp,rr,rg):rows.append((sec,np.asarray(lp),lr,lg,np.asarray(rp),rr,rg))
    row(0,left_rest,DOWN,.075,right_rest,DOWN,.075)
    row(2,pickup+[0,0,.08],DOWN,.075,right_grasp+[0,-.08,0],RIGHT_SIDE,.045)
    row(4,pickup,DOWN,.075,right_grasp,RIGHT_SIDE,.045)
    row(6,pickup,DOWN,0.,right_grasp,RIGHT_SIDE,0.)
    row(9,pickup+[0,0,.09],DOWN,0.,right_grasp,RIGHT_SIDE,0.)
    # Stabilize the standing part, then clear the wide nut opening before
    # pressing. The support and seating load keep the collet upright.
    row(10,pickup+[0,0,.09],DOWN,0.,right_grasp,RIGHT_SIDE,.060)
    row(12,pickup+[0,0,.09],DOWN,0.,inspection,RIGHT_SIDE,.075)
    pp,rr=nut_grasp([.27,.16,t+.20],150)
    row(15,pp,rr,0.,inspection,RIGHT_SIDE,.075)
    pp,rr=over_collet([-.0009,0,.060],165)
    row(18,pp,rr,0.,inspection,RIGHT_SIDE,.075)
    # Native effort limits bound the seating load; overtravel accounts for
    # loaded-arm deflection. There are no object-pose commands or snap events.
    pp,rr=over_collet([-.0009,0,-.025],165)
    row(23,pp,rr,0.,inspection,RIGHT_SIDE,.075)
    pp,rr=over_collet([0,0,-.020],180)
    row(26,pp,rr,0.,inspection,RIGHT_SIDE,.075)
    pp,rr=over_collet([0,0,.003],180)
    row(27,pp,rr,0.,inspection,RIGHT_SIDE,.075)
    row(28,pp,rr,0.,inspection,RIGHT_SIDE,.075)
    # The unused wrist follows the published inspection pose so the collet's
    # exposed tail stays in view during the unsupported retention check.
    def inspection_follow(origin,angle):
        center=np.asarray(origin)+ry(angle)@np.array([0.,0.,.0242])
        return center+np.array([0.,-.09,-.011])
    pp,rr=nut_grasp([.30,-.02,t+.17],150)
    row(31,pp,rr,0.,inspection_follow([.30,-.02,t+.17],150),RIGHT_SIDE,.075)
    pp,rr=nut_grasp([.30,-.02,t+.20],120)
    row(34,pp,rr,0.,inspection_follow([.30,-.02,t+.20],120),RIGHT_SIDE,.075)
    row(35,pp,rr,0.,inspection_follow([.30,-.02,t+.20],120),RIGHT_SIDE,.075)
    park=np.array([spec.PAD[0],spec.PAD[1],spec.PAD[2]+.001+.0443])
    pp,rr=nut_grasp(park+[0,0,.06],180)
    row(38,pp,rr,0.,inspection,RIGHT_SIDE,.075)
    pp,rr=nut_grasp(park,180)
    row(41,pp,rr,0.,inspection,RIGHT_SIDE,.075)
    row(43,pp,rr,.075,inspection,RIGHT_SIDE,.075)
    row(46,pp+[0,0,.08],rr,.075,inspection,RIGHT_SIDE,.075)
    # Dense Cartesian samples avoid the low arcs of interpolating only the
    # distant joint-space endpoints. Every sample still passes the IK gate.
    dense=[rows[0]]
    for a,b in zip(rows[:-1],rows[1:]):
        count=math.ceil((b[0]-a[0])/.25)
        for i in range(1,count+1):
            u=i/count;v=u*u*(3-2*u)
            dense.append((a[0]+(b[0]-a[0])*u,
                          a[1]*(1-v)+b[1]*v,ik.slerp_matrix(a[2],b[2],v),a[3]*(1-v)+b[3]*v,
                          a[4]*(1-v)+b[4]*v,ik.slerp_matrix(a[5],b[5],v),a[6]*(1-v)+b[6]*v))
    rows=dense
    q={s:np.array(constants.READY) for s in ['left','right']};result=[];errors=[]
    for sec,lp,lr,lg,rp,rr,rg in rows:
        poses={}
        for side,p,R,g in [('left',lp,lr,lg),('right',rp,rr,rg)]:
            seed=q[side].copy(); candidates=[]
            # Offline branch search avoids a joint-limit local minimum. No
            # simulator state is used for this deterministic kinematic solve.
            for start in [seed,np.array(constants.READY),np.array([0.,.5,1.5,-1.,0.,0.]),
                          np.array([0.,1.5,1.,-1.,1.,0.])]:
                def residual(joints):
                    position,rotation=ik.tcp_pose(joints,station.BASES[side])
                    return np.r_[position-p,.3*ik._rot_error(R,rotation)]
                fit=least_squares(residual,np.clip(start,ik.LOWER+1e-8,ik.UPPER-1e-8),
                                  bounds=(ik.LOWER,ik.UPPER),max_nfev=200)
                solved=fit.x;ep=np.linalg.norm(fit.fun[:3]);er=np.linalg.norm(fit.fun[3:])/.3
                candidates.append((ep+.3*er,solved,ep,er))
                if ep<.003 and er<.02:break
            _,q[side],ep,er=min(candidates,key=lambda item:item[0])
            errors.append({'time':sec,'side':side,'position_m':ep,'rotation_rad':er})
            f=spec.finger_joint_position(g);poses[side]=[*q[side].tolist(),f,f]
        result.append((int(sec*spec.CONTROL_HZ),poses))
    bad=[e for e in errors if e['position_m']>.003 or e['rotation_rad']>.02]
    if bad:raise RuntimeError('Offline IK reachability failure: '+str(bad))
    return result,errors

class Controller:
    def __init__(self,ik,constants,station):
        self.plan,self.errors=plan(ik,constants,station);self.observations=[]
    def action(self,step,rgb):
        if set(rgb)!={'overhead_cam','left_wrist_cam','right_wrist_cam'}:raise ValueError('Three RGB streams required')
        if step%20==0:self.observations.append({'step':step,'frame_shapes':{k:list(v.shape) for k,v in rgb.items()}})
        for (a,p),(b,q) in zip(self.plan[:-1],self.plan[1:]):
            if step<=b:
                u=np.clip((step-a)/max(1,b-a),0,1)
                return {s:(1-u)*np.array(p[s])+u*np.array(q[s]) for s in p}
        return {s:np.asarray(v) for s,v in self.plan[-1][1].items()}
