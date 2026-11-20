"""Fixed-layout RGB-only rehearsal; no live object-state feedback."""
import math
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import spec


def ry(degrees):
    a=math.radians(degrees);c,s=math.cos(a),math.sin(a)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]])


DOWN=np.array([[0.,-1.,0.],[-1.,0.,0.],[0.,0.,-1.]])
LEFT_SIDE=np.array([[-1.,0.,0.],[0.,0.,-1.],[0.,-1.,0.]])
RIGHT_SIDE=np.array([[1.,0.,0.],[0.,0.,1.],[0.,-1.,0.]])


def plan(ik,constants,station):
    t=spec.TABLE_Z
    c=np.array(spec.CENTER);pivot=np.array(spec.PIVOT)
    left_rest=np.array([.32,.08,t+.26]);right_rest=np.array([.32,-.10,t+.22])
    # Fixed offline action script: no measured object pose enters the plan.
    def lever(angle):
        R=ry(angle)
        return pivot+R@np.array([.052,0,.004]),ry(-55+.30*(angle-15))@DOWN
    start=np.array(spec.BRUSH_START);start[2]=t+.048
    dest=c+np.array([0.,-.00018,.0471])
    # Fixed-layout rehearsal calibration, applied before approaching the box.
    # These small biases are constants, not live pose corrections.
    align=Rotation.from_euler('xyz',[.12,.315,-.06],degrees=True).as_matrix()@RIGHT_SIDE
    rows=[]
    def row(sec,lp,lr,lg,rp,rr,rg):rows.append((sec,np.asarray(lp),lr,lg,np.asarray(rp),rr,rg))
    lp,lr=lever(15)
    row(0,left_rest,LEFT_SIDE,.075,right_rest,RIGHT_SIDE,.075)
    row(2,lp+[-.04,0,.02],lr,.050,start+[0,0,.21],RIGHT_SIDE,.035)
    row(4,lp,lr,.050,start,RIGHT_SIDE,.035)
    row(6,lp,lr,0.,start,RIGHT_SIDE,0.)
    for i in range(1,17):
        u=i/16;v=u*u*(3-2*u)
        lp,lr=lever(15+(-82-15)*v)
        row(6+4*u,lp,lr,0.,start+[0,0,.21*v],RIGHT_SIDE,0.)
    row(14,lp,lr,0.,dest+[0,0,.07],align,0.)
    row(19,lp,lr,0.,dest,align,0.)
    row(21,lp,lr,0.,dest,align,.065)
    row(24,lp,lr,0.,dest+[0,-.10,.08],RIGHT_SIDE,.075)
    for i in range(1,17):
        u=i/16;v=u*u*(3-2*u)
        lp,lr=lever(-82+80*v)
        row(24+4*u,lp,lr,0.,dest+[0,-.10,.08],RIGHT_SIDE,.075)
    row(30,lp,lr,.060,dest+[0,-.10,.08],RIGHT_SIDE,.075)
    row(33,lp+[-.06,0,.04],lr,.075,dest+[0,-.10,.08],RIGHT_SIDE,.075)
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
    q={s:np.array(constants.READY) for s in ['left','right']};result=[];errors=[];unbiased=[]
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
            command=q[side].copy()
            if side=='left' and 6<sec<29:
                # Offline feed-forward target bias compensates the ordinary
                # joint-position controller's deflection under the known spring.
                # It uses planned angle/kinematics only, never measured state.
                if sec<=10:
                    u=(sec-6)/4;v=u*u*(3-2*u);angle=15-97*v
                elif sec<=24:angle=-82
                else:
                    u=min(1,(sec-24)/4);v=u*u*(3-2*u);angle=-82+80*v
                r=ry(angle)@np.array([.052,0,.004])
                torque=spec.SPRING_STIFFNESS*math.radians(spec.SPRING_TARGET_DEG-angle)
                force=torque*np.array([-r[2],0,r[0]])/np.dot(r,r)
                jac=np.empty((3,6));basep,_=ik.tcp_pose(q[side],station.BASES[side])
                for k in range(6):
                    dq=q[side].copy();dq[k]+=1e-5
                    pp,_=ik.tcp_pose(dq,station.BASES[side]);jac[:,k]=(pp-basep)/1e-5
                weight=min(1.,max(0.,sec-6))*min(1.,max(0.,29-sec))
                command += weight*(jac.T@force)/np.array([80.,80.,80.,10.,10.,10.])
                if np.any(command<ik.LOWER) or np.any(command>ik.UPPER):
                    raise RuntimeError('Offline spring compensation exceeds a robot joint limit')
            if side=='right' and 6<sec<20:
                jac=np.empty((3,6));basep,_=ik.tcp_pose(q[side],station.BASES[side])
                for k in range(6):
                    dq=q[side].copy();dq[k]+=1e-5
                    pp,_=ik.tcp_pose(dq,station.BASES[side]);jac[:,k]=(pp-basep)/1e-5
                weight=min(1.,sec-6)*min(1.,20-sec)
                command+=weight*(jac.T@np.array([0.,0.,spec.BRUSH_MASS*9.81]))/np.array([80.,80.,80.,10.,10.,10.])
            f=spec.finger_joint_position(g);poses[side]=[*command.tolist(),f,f]
        result.append((int(sec*spec.CONTROL_HZ),poses))
        unbiased.append({side:q[side].copy() for side in q})
    bad=[e for e in errors if e['position_m']>.003 or e['rotation_rad']>.02]
    if bad:raise RuntimeError('Offline IK reachability failure: '+str(bad[:6])+f'; bad count={len(bad)}')
    # Fixed velocity feed-forward offsets the native drive's zero-velocity
    # damping while following the planned path. No measured state is involved.
    for i in range(1,len(result)-1):
        dt=(result[i+1][0]-result[i-1][0])/spec.CONTROL_HZ
        for side in ['left','right']:
            velocity=(unbiased[i+1][side]-unbiased[i-1][side])/dt
            command=np.array(result[i][1][side][:6])+velocity*np.array([5/80]*3+[1.5/10]*3)
            limited=np.clip(command,ik.LOWER,ik.UPPER)
            errors[2*i+(side=='right')]['feedforward_clipped_rad']=float(np.max(abs(command-limited)))
            if np.max(abs(command-limited))>.03:raise RuntimeError('Excessive offline feed-forward clipping')
            result[i][1][side][:6]=limited.tolist()
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
