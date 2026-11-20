"""Fixed-layout RGB-only rehearsal: offline kinematics, no live object state."""
import math
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import spec
DOWN=np.diag([-1.,1.,-1.])
FRONT=np.array([[0.,0.,1.],[0.,1.,0.],[-1.,0.,0.]])
RIGHT=Rotation.from_euler('z',180,degrees=True).as_matrix()@DOWN
HANDLE=Rotation.from_euler('y',15,degrees=True).as_matrix()@FRONT@Rotation.from_euler('z',180,degrees=True).as_matrix()

def plan(ik,constants,station):
    left=(np.array([.16,.35,.95]),DOWN,.075)
    right=(np.array([.16,-.35,.95]),RIGHT,.075)
    rows=[]
    def row(t,l=None,r=None):
        nonlocal left,right
        if l is not None:left=(np.array(l[0]),l[1],l[2])
        if r is not None:right=(np.array(r[0]),r[1],r[2])
        rows.append((t,*left,*right))
    row(0)
    row(2,l=([.310,.070,.850],DOWN,.060))
    row(4,l=([.310,.070,.696],DOWN,.060))
    row(6,l=([.310,.070,.696],DOWN,0.))
    row(8,l=([.310,.070,.866],DOWN,0.))
    row(10,l=([.30,.43,.84],DOWN,0.))
    row(12,l=([.30,.43,.660],DOWN,0.),r=([.14,0,.610],HANDLE,.045))
    row(14,l=([.30,.43,.660],DOWN,.075),r=([.200,0,.610],HANDLE,.045))
    row(16,l=([.16,.35,.95],DOWN,.075),r=([.200,0,.610],HANDLE,0.))
    row(19,r=([.440,0,.610],HANDLE,0.))
    row(21,r=([.440,0,.610],HANDLE,.060))
    row(22,r=([.32,0,.760],HANDLE,.060))
    row(24,r=([.400,0,.760],HANDLE,.060))
    row(26,r=([.400,0,.760],HANDLE,0.))
    row(30,r=([.150,0,.760],HANDLE,0.))
    row(32,r=([.150,0,.760],HANDLE,.060))
    row(34,r=([.150,0,.980],HANDLE,.060))
    row(36,r=([.354,-.069,.900],RIGHT,.050))
    row(38,r=([.354,-.069,.710],RIGHT,.050))
    row(40,r=([.354,-.069,.710],RIGHT,0.))
    row(42,r=([.354,-.069,.870],RIGHT,0.))
    row(44,r=([.230,-.420,.670],RIGHT,0.))
    row(46,r=([.230,-.420,.523],RIGHT,0.))
    row(48,r=([.230,-.420,.523],RIGHT,.060))
    row(50,r=([.16,-.35,.95],RIGHT,.075))
    row(54)
    dense=[rows[0]]
    for a,b in zip(rows[:-1],rows[1:]):
        count=math.ceil((b[0]-a[0])/.25)
        for i in range(1,count+1):
            u=i/count;v=u*u*(3-2*u)
            dense.append((a[0]+(b[0]-a[0])*u,a[1]*(1-v)+b[1]*v,ik.slerp_matrix(a[2],b[2],v),a[3]*(1-v)+b[3]*v,
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
            if np.max(abs(command-limited))>.03:raise RuntimeError(f'Excessive offline feed-forward clipping at step {result[i][0]}, {side}: {np.max(abs(command-limited))}')
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
