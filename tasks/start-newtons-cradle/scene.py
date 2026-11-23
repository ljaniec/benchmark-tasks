"""A desktop Newton's cradle on a table: five steel balls, each on two lines in a V from two chrome rails."""
import math
import os
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema
import spec

TEST_LIFT_DEG = float(os.environ.get('CRADLE_TEST_LIFT_DEG', '0'))


def material(stage, path, color, roughness=.45, metallic=0.):
    m = UsdShade.Material.Define(stage, path)
    s = UsdShade.Shader.Define(stage, path + '/surface')
    s.CreateIdAttr('UsdPreviewSurface')
    s.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    s.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(roughness)
    s.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(metallic)
    m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(), 'surface')
    return m


def look(stage, prim, path, color, roughness=.45, metallic=0.):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material(stage, path, color, roughness, metallic))


def root(stage, path, pos, mass=None, quat=None, damping=None):
    r = UsdGeom.Xform.Define(stage, path)
    r.AddTranslateOp().Set(Gf.Vec3d(*pos))
    r.AddOrientOp().Set(quat or Gf.Quatf(1.))
    if mass is not None:
        UsdPhysics.RigidBodyAPI.Apply(r.GetPrim())
        UsdPhysics.MassAPI.Apply(r.GetPrim()).CreateMassAttr(mass)
        api = PhysxSchema.PhysxRigidBodyAPI.Apply(r.GetPrim())
        api.CreateSolverPositionIterationCountAttr(32)
        api.CreateSolverVelocityIterationCountAttr(4)
        api.CreateMaxDepenetrationVelocityAttr(.5)
        api.CreateEnableCCDAttr(True)
        api.CreateSleepThresholdAttr(0.)
        api.CreateStabilizationThresholdAttr(0.)
        if damping is not None:
            api.CreateLinearDampingAttr(damping)
            api.CreateAngularDampingAttr(damping)
        PhysxSchema.PhysxContactReportAPI.Apply(r.GetPrim()).CreateThresholdAttr(0.)
    return r


def collide(stage, prim, contact='/World/contact', offset=.001):
    UsdPhysics.CollisionAPI.Apply(prim)
    p = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    p.CreateContactOffsetAttr(offset)
    p.CreateRestOffsetAttr(0.)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        UsdShade.Material(stage.GetPrimAtPath(contact)), materialPurpose='physics')


def cube(stage, path, size, pos, color, collision=True, contact='/World/contact', quat=None, metallic=0.):
    shape = UsdGeom.Cube.Define(stage, path)
    shape.CreateSizeAttr(1.)
    shape.AddTranslateOp().Set(Gf.Vec3d(*pos))
    if quat is not None:
        shape.AddOrientOp().Set(quat)
    shape.AddScaleOp().Set(Gf.Vec3f(*size))
    if collision:
        collide(stage, shape.GetPrim(), contact)
    look(stage, shape.GetPrim(), path + '_look', color, metallic=metallic)
    return shape


def box(stage, path, size, pos, color, collision=True):
    root(stage, path, pos)
    return cube(stage, path + '/shape', size, (0., 0., 0.), color, collision)


def rod(stage, path, p0, p1, radius, color, collision=True, contact='/World/contact', metallic=0.):
    """A cylinder from p0 to p1."""
    p0, p1 = Gf.Vec3d(*p0), Gf.Vec3d(*p1)
    d = p1 - p0
    length = d.GetLength()
    c = UsdGeom.Cylinder.Define(stage, path)
    c.CreateRadiusAttr(radius)
    c.CreateHeightAttr(length)
    c.CreateAxisAttr('Z')
    c.AddTranslateOp().Set((p0 + p1) / 2)
    rot = Gf.Rotation(Gf.Vec3d(0, 0, 1), d / length).GetQuat()
    c.AddOrientOp().Set(Gf.Quatf(rot))
    if collision:
        collide(stage, c.GetPrim(), contact)
    look(stage, c.GetPrim(), path + '_look', color, roughness=.25, metallic=metallic)
    return c


def anchors(k):
    x = spec.ball_rest_x(k)
    return [(x, s * spec.RAIL_Y, spec.RAIL_Z) for s in (1, -1)]


def build(stage):
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdGeom.SetStageUpAxis(stage, 'Z')
    ph = UsdPhysics.Scene.Define(stage, '/World/physics')
    ph.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    ph.CreateGravityMagnitudeAttr(9.81)
    api = PhysxSchema.PhysxSceneAPI.Apply(ph.GetPrim())
    api.CreateTimeStepsPerSecondAttr(spec.PHYSICS_HZ)
    api.CreateEnableGPUDynamicsAttr(False)
    api.CreateBroadphaseTypeAttr('MBP')
    api.CreateEnableCCDAttr(True)
    api.CreateBounceThresholdAttr(.005)
    for name, static, dynamic, restitution in [('contact', .65, .55, 0.), ('pad_contact', 1.2, 1., 0.),
                                               ('steel', spec.BALL_FRICTION, spec.BALL_FRICTION, spec.BALL_RESTITUTION)]:
        m = UsdPhysics.MaterialAPI.Apply(UsdShade.Material.Define(stage, '/World/' + name).GetPrim())
        m.CreateStaticFrictionAttr(static)
        m.CreateDynamicFrictionAttr(dynamic)
        m.CreateRestitutionAttr(restitution)
    steel = '/World/steel'
    PhysxSchema.PhysxMaterialAPI.Apply(stage.GetPrimAtPath(steel)).CreateRestitutionCombineModeAttr('max')

    box(stage, '/World/floor', (3.2, 2.8, .06), (.80, -.20, -.03), (.62, .60, .56))
    for side, sign in [('left', 1), ('right', -1)]:
        box(stage, '/World/pedestal_' + side, (.15, .15, .750), (0, sign * .305, .375), (.35, .39, .42))
    tx, ty = spec.TABLE_CENTRE
    sx, sy = spec.TABLE_SIZE
    box(stage, '/World/table', (sx, sy, spec.TABLE_TOP_Z), (tx, ty, spec.TABLE_TOP_Z / 2), (.78, .74, .68))

    # Cradle frame (fixed): plinth, two end frames with splayed legs, and two rails.
    bx, by = spec.BASE_CENTRE
    frame = root(stage, '/World/cradle', (0., 0., 0.), mass=2.)
    UsdPhysics.RigidBodyAPI.Apply(frame.GetPrim()).CreateKinematicEnabledAttr(True)
    cube(stage, '/World/cradle/base', spec.BASE_SIZE, (bx, by, spec.TABLE_TOP_Z + spec.BASE_SIZE[2] / 2),
         (.08, .07, .07))
    chrome = (.80, .81, .83)
    x_ends = (spec.ball_rest_x(0) - spec.RAIL_OVERHANG, spec.ball_rest_x(spec.BALLS - 1) + spec.RAIL_OVERHANG)
    for i, xe in enumerate(x_ends):
        for s in (1, -1):
            rod(stage, f'/World/cradle/leg_{i}_{"l" if s > 0 else "r"}', (xe, s * spec.LEG_FOOT_Y, spec.BASE_TOP_Z),
                (xe, s * spec.RAIL_Y, spec.RAIL_Z), spec.RAIL_D / 2, chrome, metallic=.9)
        rod(stage, f'/World/cradle/top_{i}', (xe, spec.RAIL_Y, spec.RAIL_Z), (xe, -spec.RAIL_Y, spec.RAIL_Z),
            spec.RAIL_D / 2, chrome, metallic=.9)
    for s in (1, -1):
        rod(stage, f'/World/cradle/rail_{"l" if s > 0 else "r"}', (x_ends[0], s * spec.RAIL_Y, spec.RAIL_Z),
            (x_ends[1], s * spec.RAIL_Y, spec.RAIL_Z), spec.RAIL_D / 2, chrome, metallic=.9)

    # Balls, each on two lines that can go slack (distance joints with no minimum).
    for k in range(spec.BALLS):
        pos, q = spec.ball_pose(k, TEST_LIFT_DEG if k == 0 else 0.)
        root(stage, f'/World/ball_{k}', pos, spec.BALL_MASS, quat=Gf.Quatf(*q), damping=spec.STEEL_ROLL_DAMPING)
        sphere = UsdGeom.Sphere.Define(stage, f'/World/ball_{k}/shape')
        sphere.CreateRadiusAttr(spec.BALL_R)
        collide(stage, sphere.GetPrim(), steel, offset=.0003)
        look(stage, sphere.GetPrim(), f'/World/ball_{k}/shape_look', chrome, .15, 1.)
        for j, a in enumerate(anchors(k)):
            joint = UsdPhysics.DistanceJoint.Define(stage, f'/World/joints/line_{k}_{j}')
            joint.CreateBody1Rel().SetTargets([f'/World/ball_{k}'])
            joint.CreateLocalPos0Attr(Gf.Vec3f(*a))
            joint.CreateLocalPos1Attr(Gf.Vec3f(0., 0., spec.BALL_R))
            joint.CreateLocalRot0Attr(Gf.Quatf(1.))
            joint.CreateLocalRot1Attr(Gf.Quatf(1.))
            joint.CreateMinDistanceAttr(-1.)
            joint.CreateMaxDistanceAttr(spec.LINE_LEN - spec.JOINT_TOLERANCE)
            joint.CreateExcludeFromArticulationAttr(True)

    # Drawn lines (no collision), re-placed every control step.
    UsdGeom.Xform.Define(stage, '/World/lines')
    for k in range(spec.BALLS):
        for j, a in enumerate(anchors(k)):
            seg = UsdGeom.Cylinder.Define(stage, f'/World/lines/line_{k}_{j}')
            seg.CreateAxisAttr('Z')
            seg.CreateRadiusAttr(spec.LINE_D / 2 + .0002)
            seg.CreateHeightAttr(1.)
            seg.AddTranslateOp().Set(Gf.Vec3d(*a))
            seg.AddOrientOp().Set(Gf.Quatf(1.))
            seg.AddScaleOp().Set(Gf.Vec3f(1., 1., spec.LINE_LEN))
            look(stage, seg.GetPrim(), f'/World/lines/line_{k}_{j}_look', (.92, .92, .90), .6)
    return {f'ball_{k}': f'/World/ball_{k}' for k in range(spec.BALLS)}


def segment_pose(p0, p1):
    """(centre, quaternion wxyz, length) of a cylinder from p0 to p1."""
    import numpy as np
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    length = float(np.linalg.norm(d))
    u = d / length if length > 1e-9 else np.array([0., 0., 1.])
    axis = np.cross([0., 0., 1.], u)
    s = float(np.linalg.norm(axis))
    angle = math.atan2(s, float(u[2]))
    axis = axis / s if s > 1e-9 else np.array([1., 0., 0.])
    return (p0 + p1) / 2, (math.cos(angle / 2), *(axis * math.sin(angle / 2))), max(length, 1e-5)


FIXED = ['floor', 'pedestal_left', 'pedestal_right', 'table', 'cradle']
