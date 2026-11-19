"""Fixed-layout action rehearsal; no simulator/object-state input.

IK uses the published robot model and scene setup before execution. Only RGB is
accepted at execution time. This is a development controller, not validated
feasibility evidence until a scored camera-recorded run succeeds.
"""
import numpy as np
import spec


def plan(ik, constants, station, pick_xy=None):
    # Each row: seconds, left TCP, left aperture, right TCP, right aperture.
    t=spec.TABLE_Z
    rows=[
        (0, (.22,.19,t+.15),.075,(.25,-.18,t+.15),.075),
        (3, (.375,.15,t+.0825),.025,(.349,-.0924,t+.0530),.026),
        (4, (.375,.080,t+.0600),.020,(.349,-.0642,t+.04276),.026),
        (5, (.375,.048,t+.0525),.015,(.349,-.036,t+.0325),.026),
        (7, (.375,.048,t+.0525),0.,(.349,-.036,t+.0325),0.),
        (12,(.375,.048,t+.0525),0.,(.330,-.10,t+.0925),0.),
        (17,(.375,.048,t+.0525),0.,(.310,-.09,t+.1025),0.),
        (19,(.375,.048,t+.0525),0.,(.310,-.09,t+.1025),.075),
        (22,(.375,.048,t+.0525),0.,(.342,0,t+.12),.06),
        (25,(.375,.048,t+.0525),0.,(.342,0,t+.0615),.06),
        (27,(.375,.048,t+.0525),0.,(.342,0,t+.0615),.025),
        (29,(.375,.048,t+.0525),0.,(.29,0,t+.14),.025),
        (31,(.375,.048,t+.0525),0.,(.30,-.04,t+.14),.025),
        (35,(.22,.19,t+.15),.075,(.27,-.20,t+.14),.025),
        (39,(.22,.19,t+.15),.075,(.27,-.20,t+.0525),.025),
        (41,(.22,.19,t+.15),.075,(.27,-.20,t+.0525),.075),
        (44,(.22,.19,t+.15),.075,(.27,-.20,t+.16),.075)]
    result=[];q={side:np.array(constants.READY) for side in ['left','right']}; errors=[]
    # Gripper +Z approaches down; jaws close along Y with this rotation.
    rotation=ik.matrix_to_quat_xyzw(np.array([[0.,-1.,0.],[-1.,0.,0.],[0.,0.,-1.]]))
    side_rotation=ik.matrix_to_quat_xyzw(np.array([[-1.,0.,0.],[0.,0.,-1.],[0.,-1.,0.]]))
    a=np.deg2rad(-25.)
    pickup_tilt=np.array([[np.cos(a),0,np.sin(a)],[0,1,0],[-np.sin(a),0,np.cos(a)]])
    pickup_rotation=ik.matrix_to_quat_xyzw(pickup_tilt@np.array([[0.,-1.,0.],[-1.,0.,0.],[0.,0.,-1.]]))
    b=np.deg2rad(-20.)
    side_tilt=np.array([[1,0,0],[0,np.cos(b),-np.sin(b)],[0,np.sin(b),np.cos(b)]])
    peel_rotation=ik.matrix_to_quat_xyzw(side_tilt@np.array([[0.,-1.,0.],[0.,0.,1.],[-1.,0.,0.]]))
    for sec,lp,lg,rp,rg in rows:
        if 0 < sec <= 31:lp=(lp[0]+spec.LAYOUT_X_SHIFT,*lp[1:])
        if 0 < sec <= 27:rp=(rp[0]+spec.LAYOUT_X_SHIFT,*rp[1:])
        if pick_xy is not None and sec in (22,25,27):rp=(*pick_xy,rp[2])
        poses={}
        for side,p,g in [('left',lp,lg),('right',rp,rg)]:
            if side=='left' and sec in (3,35):
                q[side] = np.array(constants.READY)
            if side=='right' and sec==3:
                q[side] = np.array(constants.READY)
            orientation = side_rotation if side=='left' and 0<sec<=31 else rotation
            if side=='right' and 0<sec<=19:
                orientation = peel_rotation
            if side=='right' and 22<=sec<=29:
                orientation = pickup_rotation
            q[side],ep,er=ik.solve(p,orientation,q[side],station.BASES[side],iters=400,damping=.015)
            errors.append({'time':sec,'side':side,'position_m':ep,'rotation_rad':er})
            # URDF q=0 is closed; negative finger coordinates OPEN the jaws.
            # Mesh/FK checks give aperture ~= -2*q + 0.063 mm.
            finger=spec.finger_joint_position(g)
            poses[side]=[*q[side].tolist(),finger,finger]
        result.append((int(sec*spec.CONTROL_HZ),poses))
    critical = [e for e in errors if (e['side']=='right' and e['time'] in (5,7,25,27))
                or (e['side']=='left' and e['time'] in (5,7))]
    if any(e['position_m']>.002 or e['rotation_rad']>.02 for e in critical):
        raise RuntimeError('A contact waypoint is outside the verified IK tolerance')
    return result,errors


def locate_battery(rgb, camera):
    """Intersect blue-object RGB rays with the known upright battery upper face."""
    from yam_arena.urdf import quat_xyzw_to_matrix
    im=np.asarray(rgb,dtype=float);r,g,b=np.moveaxis(im,-1,0)
    mask=(b>65)&(b>r*1.45)&(b>g*1.30)
    yy,xx=np.nonzero(mask)
    if len(xx)<4:return None
    lens=camera.lens.resized(im.shape[1],im.shape[0])
    ray=quat_xyzw_to_matrix(np.array(camera.rot_xyzw))@np.array([(np.median(xx)-lens.cx)/lens.fx,-(np.median(yy)-lens.cy)/lens.fy,-1.])
    if abs(ray[2])<.1:return None
    # Visible blue silhouette combines the top and front faces. This plane is
    # 6.35 mm below the top, checked against the known reset camera image.
    z=spec.BATTERY_POS[2]+.020
    camera_pos=np.array(camera.pos);xy=(camera_pos+ray*((z-camera_pos[2])/ray[2]))[:2]
    if not (.20<xy[0]<.42 and -.18<xy[1]<.12):return None
    return xy


class Controller:
    def __init__(self,ik,constants,station):
        self.ik,self.constants,self.station=ik,constants,station
        self.plan,self.errors=plan(ik,constants,station)
        self.observations=[]

    def action(self,step,rgb):
        if step in (380,385) and rgb is not None:
            xy=locate_battery(rgb['overhead_cam'],self.station.top_camera())
            self.observations.append(None if xy is None else xy.tolist())
            if step==385:
                if any(v is None for v in self.observations) or np.linalg.norm(np.diff(np.array(self.observations),axis=0))>.01:
                    raise RuntimeError('Battery not reliably localized in overhead RGB')
                self.plan,self.errors=plan(self.ik,self.constants,self.station,np.mean(self.observations,axis=0))
        return action(step,self.plan,rgb)


def action(step, plan, rgb):
    if rgb is not None and set(rgb)!={'overhead_cam','left_wrist_cam','right_wrist_cam'}:
        raise ValueError('Only the three RGB camera streams are permitted')
    for (a,p),(b,q) in zip(plan[:-1],plan[1:]):
        if step<=b:
            u=np.clip((step-a)/max(1,b-a),0,1);u=u*u*(3-2*u)
            return {s:(1-u)*np.array(p[s])+u*np.array(q[s]) for s in p}
    return {s:np.array(v) for s,v in plan[-1][1].items()}
