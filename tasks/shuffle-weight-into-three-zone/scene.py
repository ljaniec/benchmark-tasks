"""A 9-ft table shuffleboard in its guttered cradle, with one red-capped weight at the near end."""
import math
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema
import spec

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
        api.CreateMaxDepenetrationVelocityAttr(.2)
        api.CreateEnableCCDAttr(True)
        api.CreateSleepThresholdAttr(0.)
        api.CreateStabilizationThresholdAttr(0.)
        if damping is not None:
            api.CreateLinearDampingAttr(damping)
            api.CreateAngularDampingAttr(damping)
        PhysxSchema.PhysxContactReportAPI.Apply(r.GetPrim()).CreateThresholdAttr(0.)
    return r


def collide(stage, prim, contact='/World/contact', offset=.001, rest=0.):
    UsdPhysics.CollisionAPI.Apply(prim)
    p = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    p.CreateContactOffsetAttr(offset)
    p.CreateRestOffsetAttr(rest)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        UsdShade.Material(stage.GetPrimAtPath(contact)), materialPurpose='physics')


def cube(stage, path, size, pos, color, collision=True, contact='/World/contact', roughness=.45):
    shape = UsdGeom.Cube.Define(stage, path)
    shape.CreateSizeAttr(1.)
    shape.AddTranslateOp().Set(Gf.Vec3d(*pos))
    shape.AddScaleOp().Set(Gf.Vec3f(*size))
    if collision:
        collide(stage, shape.GetPrim(), contact)
    look(stage, shape.GetPrim(), path + '_look', color, roughness)
    return shape


def span(stage, path, lo, hi, color, contact='/World/contact', roughness=.45):
    """A static box between corners lo and hi."""
    size = [h - l for l, h in zip(lo, hi)]
    if min(size) <= 0.:
        return
    root(stage, path, [(l + h) / 2 for l, h in zip(lo, hi)])
    cube(stage, path + '/shape', size, (0., 0., 0.), color, contact=contact, roughness=roughness)


def cylinder(stage, path, radius, length, centre_x, color, collision=False, contact='/World/contact', roughness=.4):
    """A cylinder along the parent's x axis."""
    c = UsdGeom.Cylinder.Define(stage, path)
    c.CreateAxisAttr('X')
    c.CreateRadiusAttr(radius)
    c.CreateHeightAttr(length)
    c.AddTranslateOp().Set(Gf.Vec3d(centre_x, 0., 0.))
    if collision:
        collide(stage, c.GetPrim(), contact)
    look(stage, c.GetPrim(), path + '_look', color, roughness)
    return c


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
    api.CreateBounceThresholdAttr(.01)
    materials = {'contact': (.65, .55, 0.), 'pad_contact': (1.2, 1., 0.), **spec.FRICTION}
    for name, (static, dynamic, restitution) in materials.items():
        prim = UsdShade.Material.Define(stage, '/World/' + name).GetPrim()
        m = UsdPhysics.MaterialAPI.Apply(prim)
        m.CreateStaticFrictionAttr(static)
        m.CreateDynamicFrictionAttr(dynamic)
        m.CreateRestitutionAttr(restitution)
        if name == 'wax':
            PhysxSchema.PhysxMaterialAPI.Apply(prim).CreateFrictionCombineModeAttr('min')

    span(stage, '/World/floor', (-.90, -1.80, -.06), (3.40, 1.80, 0.), (.62, .60, .56))
    for side, sign in [('left', 1), ('right', -1)]:
        span(stage, '/World/pedestal_' + side, (-.075, sign * .305 - .075, 0.), (.075, sign * .305 + .075, .750),
             (.35, .39, .42))

    (cx0, cy0), (cx1, cy1) = spec.CRADLE
    X0, X1, Y0, Y1, W = spec.X0, spec.X1, spec.Y0, spec.Y1, spec.WALL
    oak, dark, maple = (.42, .25, .11), (.10, .07, .05), (.78, .58, .34)
    # Legs and a skirt under the cradle, the gutter floor, then the cradle's walls.
    for k, x in enumerate((cx0 + .10, (cx0 + cx1) / 2, cx1 - .10)):
        for s, y in (('r', cy0 + .08), ('l', cy1 - .08)):
            span(stage, f'/World/leg_{k}{s}', (x - .04, y - .04, 0.), (x + .04, y + .04, .560), oak, contact='/World/wood')
    span(stage, '/World/skirt', (cx0, cy0, .560), (cx1, cy1, spec.GUTTER_Z - .030), oak, contact='/World/wood')
    span(stage, '/World/gutter', (cx0, cy0, spec.GUTTER_Z - .030), (cx1, cy1, spec.GUTTER_Z), dark, contact='/World/wood',
         roughness=.9)
    for name, lo, hi in [('wall_near', (cx0, cy0), (cx0 + W, cy1)), ('wall_far', (cx1 - W, cy0), (cx1, cy1)),
                         ('wall_right', (cx0, cy0), (cx1, cy0 + W)), ('wall_left', (cx0, cy1 - W), (cx1, cy1))]:
        span(stage, '/World/' + name, (*lo, spec.GUTTER_Z), (*hi, spec.WALL_TOP_Z), oak, contact='/World/wood')
    # The playfield: waxed maple.
    span(stage, '/World/board', (X0, Y0, spec.GUTTER_Z), (X1, Y1, spec.BOARD_Z), maple, contact='/World/wax',
         roughness=.25)
    # Painted lines, drawn only (a weight slides over them).
    lines = [('line3', spec.LINE3), ('line2', spec.LINE2), ('foul', spec.FOUL),
             ('near3', spec.NEAR_LINES[0]), ('near2', spec.NEAR_LINES[1])]
    for name, x in lines:
        cube(stage, '/World/' + name, (spec.LINE_W, spec.BOARD_W, .0004), (x, spec.LANE_Y, spec.BOARD_Z + .0002),
             (.01, .01, .01), collision=False)
    # Zone marks: one black dot per point, drawn in the far zones.
    for points, (a, b) in [(3, (spec.LINE3, X1)), (2, (spec.LINE2, spec.LINE3)), (1, (spec.FOUL, spec.LINE2))]:
        x = (a + b) / 2 if points > 1 else spec.LINE2 - .08
        for k in range(points):
            y = spec.LANE_Y + (k - (points - 1) / 2) * .045
            d = UsdGeom.Cylinder.Define(stage, f'/World/mark_{points}_{k}')
            d.CreateAxisAttr('Z')
            d.CreateRadiusAttr(.012)
            d.CreateHeightAttr(.0004)
            d.AddTranslateOp().Set(Gf.Vec3d(x, y, spec.BOARD_Z + .0002))
            look(stage, d.GetPrim(), f'/World/mark_{points}_{k}_look', (.02, .02, .02))

    # The weight: origin at the centre of its base.
    w = root(stage, '/World/weight', spec.WEIGHT_START, spec.WEIGHT_MASS)
    mass = UsdPhysics.MassAPI(w.GetPrim())
    mass.CreateCenterOfMassAttr(Gf.Vec3f(0., 0., spec.WEIGHT_COM_Z))
    for name, r, h, z, color, contact, metal in [
            ('puck', spec.PUCK_R, spec.PUCK_H, spec.PUCK_H / 2, (.80, .80, .82), '/World/steel', 1.),
            ('cap', spec.CAP_R, spec.CAP_H, spec.PUCK_H + spec.CAP_H / 2, (.75, .08, .06), '/World/cap', 0.)]:
        c = UsdGeom.Cylinder.Define(stage, f'/World/weight/{name}')
        c.CreateAxisAttr('Z')
        c.CreateRadiusAttr(r)
        c.CreateHeightAttr(h)
        c.AddTranslateOp().Set(Gf.Vec3d(0., 0., z))
        collide(stage, c.GetPrim(), contact)
        look(stage, c.GetPrim(), f'/World/weight/{name}_look', color, .15 if metal else .35, metal)
    return {'weight': '/World/weight'}


FIXED = ['floor', 'pedestal_left', 'pedestal_right', 'gutter', 'board', 'wall_far', 'wall_right']
