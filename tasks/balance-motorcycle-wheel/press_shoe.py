"""Geometry and passive magnetic retention for a wheel-weight applicator."""

import numpy as np

from probe_strip_press import THICK_M


FACE_THICKNESS_M = .007
FACE_WIDTH_M = .026
FACE_LENGTH_M = .031
GRIP_WIDTH_M = .019
GRIP_LENGTH_M = .023
GRIP_DEPTH_M = .024
SHOE_MASS_KG = .024
TCP_FROM_STRIP_M = .025
MAGNET_HOLD_N = .95
MAGNET_REACH_M = .008


def shoe_boxes():
    backing = THICK_M / 2.
    shoulder = backing + FACE_THICKNESS_M
    return (
        ('face', (0., backing + FACE_THICKNESS_M / 2., 0.),
         (FACE_WIDTH_M, FACE_THICKNESS_M, FACE_LENGTH_M)),
        ('handle', (0., shoulder + GRIP_DEPTH_M / 2., 0.),
         (GRIP_WIDTH_M, GRIP_DEPTH_M, GRIP_LENGTH_M)),
    )


def magnetic_reaction(strip_center, strip_rotation, shoe_center, shoe_rotation):
    """Pull the strip against the shoe, with an equal reaction on the shoe."""
    normal = shoe_rotation[:, 1]
    local = (strip_center - shoe_center) @ shoe_rotation
    separation = max(0., -local[1])
    if (separation >= MAGNET_REACH_M or local[1] > .002
            or abs(local[0]) > .009 or abs(local[2]) > .013
            or strip_rotation[:, 1] @ normal < .85):
        return np.zeros(3)
    scale = (1. - separation / MAGNET_REACH_M) ** 2
    return MAGNET_HOLD_N * scale * normal


def author_shoe(stage, path, center, quat_xyzw):
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    root = UsdGeom.Xform.Define(stage, path)
    root.AddTranslateOp().Set(Gf.Vec3d(*center))
    x, y, z, w = quat_xyzw
    root.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, Gf.Vec3d(x, y, z)))
    prim = root.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(SHOE_MASS_KG)
    for name, center, extent in shoe_boxes():
        box = UsdGeom.Cube.Define(stage, f'{path}/{name}')
        box.CreateSizeAttr(1.)
        box.AddTranslateOp().Set(Gf.Vec3d(*center))
        box.AddScaleOp().Set(Gf.Vec3f(*extent))
        box.CreateDisplayColorAttr([Gf.Vec3f(.17, .50, .58)])
        UsdPhysics.CollisionAPI.Apply(box.GetPrim())
        PhysxSchema.PhysxCollisionAPI.Apply(box.GetPrim()).CreateContactOffsetAttr(.0004)
    return prim
