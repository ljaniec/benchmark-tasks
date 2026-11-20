"""Visible tread clearance from the floor or sill, without contact-force telemetry."""
import numpy as np
from scipy.spatial.transform import Rotation
import spec


def tread_points(pose):
    # Sample the actual 64-sided tire faces, not a sphere or inflated bounds.
    # Eight subdivisions per face bound the point-spacing error below 0.4 mm.
    angle=np.arange(64)*2*np.pi/64
    ring=np.c_[spec.WHEEL_RADIUS*np.cos(angle),spec.WHEEL_RADIUS*np.sin(angle)]
    u=np.arange(8)/8
    xy=(ring[:,None,:]*(1-u[None,:,None])+np.roll(ring,-1,axis=0)[:,None,:]*u[None,:,None]).reshape(-1,2)
    local=np.concatenate([np.c_[xy,np.full(len(xy),z)] for z in [-spec.WHEEL_WIDTH/2,0,spec.WHEEL_WIDTH/2]])
    return local@Rotation.from_quat(pose[3:]).as_matrix().T+np.asarray(pose[:3])


def clearance(pose):
    points=tread_points(pose)
    floor=max(0.,float(points[:,2].min()))
    low=np.array([spec.SILL_X[0],-.65,0.])
    high=np.array([spec.SILL_X[1],.65,spec.SILL_HEIGHT])
    delta=np.maximum(np.maximum(low-points,points-high),0.)
    sill=float(np.linalg.norm(delta,axis=1).min())
    return min(floor,sill)


def support_gap(poses):
    """At least one tire must remain within the visible support tolerance."""
    return min(clearance(poses['wheel_'+side]) for side in ['left','right'])
