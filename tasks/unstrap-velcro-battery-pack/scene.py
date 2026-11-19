"""Procedural native PhysX scene. Import only after starting Isaac Sim.

Flexible-link pattern informed by accepted three-device-charging (PR #1939).
This implementation is independent; all task geometry is procedural.
"""
import math
import numpy as np
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema
import spec


def material(stage, path, color):
    m = UsdShade.Material.Define(stage, path)
    s = UsdShade.Shader.Define(stage, path + '/surface')
    s.CreateIdAttr('UsdPreviewSurface')
    s.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    s.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(.65)
    m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(), 'surface')
    return m


def box(stage, path, size, pos, color, mass=None, yaw=0):
    root = UsdGeom.Xform.Define(stage, path)
    root.AddTranslateOp().Set(Gf.Vec3d(*pos))
    root.AddOrientOp().Set(Gf.Quatf(math.cos(yaw/2), 0, 0, math.sin(yaw/2)))
    if mass is not None:
        UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
        UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(mass)
        api = PhysxSchema.PhysxRigidBodyAPI.Apply(root.GetPrim())
        api.CreateSolverPositionIterationCountAttr(32)
        api.CreateSolverVelocityIterationCountAttr(8)
        api.CreateMaxDepenetrationVelocityAttr(.3)
        api.CreateEnableCCDAttr(True)
    shape = UsdGeom.Cube.Define(stage, path + '/shape')
    shape.CreateSizeAttr(1.)
    shape.AddScaleOp().Set(Gf.Vec3f(*size))
    UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
    phys = PhysxSchema.PhysxCollisionAPI.Apply(shape.GetPrim())
    phys.CreateContactOffsetAttr(.0002)
    phys.CreateRestOffsetAttr(0.)
    UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(material(stage, path+'/look', color))
    UsdShade.MaterialBindingAPI(shape.GetPrim()).Bind(UsdShade.Material(stage.GetPrimAtPath('/World/contact')), materialPurpose='physics')
    return root


def hinge(stage, name, b0, b1, p0, p1, yaw0=0., yaw1=0.):
    j = UsdPhysics.Joint.Define(stage, '/World/joints/' + name)
    j.CreateBody0Rel().SetTargets([b0]); j.CreateBody1Rel().SetTargets([b1])
    j.CreateLocalPos0Attr(Gf.Vec3f(*p0)); j.CreateLocalPos1Attr(Gf.Vec3f(*p1))
    # Coincident joint frames at the initial bent path: no initial spring impulse.
    j.CreateLocalRot0Attr(Gf.Quatf(math.cos(yaw0/2),0,0,math.sin(yaw0/2)))
    j.CreateLocalRot1Attr(Gf.Quatf(math.cos(yaw1/2),0,0,math.sin(yaw1/2)))
    j.CreateCollisionEnabledAttr(False)
    j.CreateExcludeFromArticulationAttr(True)
    for axis in ['transX','transY','transZ']:
        limit=UsdPhysics.LimitAPI.Apply(j.GetPrim(),axis)
        limit.CreateLowAttr(1.);limit.CreateHighAttr(-1.)
    for axis,room in [('rotX',45.),('rotY',120.),('rotZ',170.)]:
        limit=UsdPhysics.LimitAPI.Apply(j.GetPrim(),axis)
        limit.CreateLowAttr(-room);limit.CreateHighAttr(room)
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(),axis)
        d.CreateTypeAttr('force')
        # USD angular drive coefficients are per degree; specification is SI.
        d.CreateStiffnessAttr(spec.BEND_STIFFNESS_NM_RAD * math.pi / 180)
        d.CreateDampingAttr(spec.BEND_DAMPING_NMS_RAD * math.pi / 180)
        d.CreateMaxForceAttr(spec.BEND_TORQUE_LIMIT_NM)
        d.CreateTargetPositionAttr(0.)
    return j


def build(stage):
    UsdGeom.SetStageMetersPerUnit(stage, 1.); UsdGeom.SetStageUpAxis(stage, 'Z')
    ph = UsdPhysics.Scene.Define(stage, '/World/physics')
    ph.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1)); ph.CreateGravityMagnitudeAttr(9.81)
    api = PhysxSchema.PhysxSceneAPI.Apply(ph.GetPrim())
    api.CreateTimeStepsPerSecondAttr(spec.PHYSICS_HZ)
    api.CreateEnableGPUDynamicsAttr(False); api.CreateBroadphaseTypeAttr('MBP')
    api.CreateEnableCCDAttr(True)
    mat = UsdShade.Material.Define(stage, '/World/contact')
    m = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    m.CreateStaticFrictionAttr(spec.STATIC_FRICTION)
    m.CreateDynamicFrictionAttr(spec.DYNAMIC_FRICTION); m.CreateRestitutionAttr(0.)
    box(stage, '/World/table', (1.1,1.3,.06), (.36,0,spec.TABLE_Z-.03), (.42,.29,.18))
    box(stage, '/World/mat', spec.MAT_SIZE, spec.MAT_CENTER, (.06,.25,.09))
    module = box(stage, '/World/module', (.09,.11,.012), spec.MODULE_POS, (.35,.39,.42), .45)
    # Components on the same rigid body. Static geometry in its own local coordinates.
    for name,size,pos,color in [
        ('backplate',(.004,.085,.082),(.010,0,.047),(.06,.19,.10)),
        ('shelf',(.048,.048,.004),(-.020,0,.004),(.45,.48,.50)),
        ('side_mount',(.030,.005,.024),(-.003,-.0211,.035),(.25,.28,.31))]:
        box(stage, '/World/module/'+name, size, pos, color)
    box(stage, '/World/battery', spec.BATTERY_SIZE, spec.BATTERY_POS, (.04,.12,.65), spec.BATTERY_MASS)
    points = []
    path = [np.array(p) for p in spec.STRAP_PATH]
    for a,b in zip(path[:-1],path[1:]):
        n = math.ceil(np.linalg.norm(b-a)/.007)
        points.extend(a+(b-a)*i/n for i in range(n))
    points.append(path[-1])
    links=[]; bonds=[]; hinges=[]; previous='/World/module'; prev_center=np.array(spec.MODULE_POS); prev_yaw=0.
    for i,(a,b) in enumerate(zip(points[:-1],points[1:])):
        center=np.array([*(.5*(a+b)),spec.STRAP_Z]); length=float(np.linalg.norm(b-a))
        yaw=math.atan2(*(b-a)[::-1]); name=f'/World/strap_{i:02d}'
        box(stage,name,(length*.97,spec.STRAP_THICKNESS,spec.STRAP_WIDTH),center,(.70,.055,.035),length*.97*spec.STRAP_THICKNESS*spec.STRAP_WIDTH*spec.STRAP_DENSITY,yaw)
        pivot=np.array([*a,spec.STRAP_Z])
        def local(delta, angle):
            c,s=math.cos(angle),math.sin(angle)
            return (c*delta[0]+s*delta[1],-s*delta[0]+c*delta[1],delta[2])
        p0=local(pivot-prev_center,prev_yaw);p1=(-length/2,0,0)
        hinge(stage,f'bend_{i:02d}',previous,name,p0,p1,-prev_yaw,-yaw)
        hinges.append((previous,name,p0,p1))
        # Independent local bonds along the overlap on the side mounting pad.
        if abs(center[1]+.0242)<1e-7 and .348+spec.LAYOUT_X_SHIFT < center[0] < .372+spec.LAYOUT_X_SHIFT:
            j=UsdPhysics.FixedJoint.Define(stage,f'/World/joints/overlap_{i:02d}')
            j.CreateBody0Rel().SetTargets(['/World/module']);j.CreateBody1Rel().SetTargets([name])
            j.CreateLocalPos0Attr(Gf.Vec3f(*(center-np.array(spec.MODULE_POS))))
            j.CreateLocalPos1Attr(Gf.Vec3f(0));j.CreateCollisionEnabledAttr(False)
            j.CreateBreakForceAttr(spec.BOND_FORCE);j.CreateBreakTorqueAttr(spec.BOND_TORQUE)
            bonds.append((str(j.GetPath()),name,(center-np.array(spec.MODULE_POS)).tolist()))
        links.append(name);previous=name;prev_center=center;prev_yaw=yaw
    return {'links':links,'bonds':bonds,'hinges':hinges,'module':'/World/module','battery':'/World/battery'}
