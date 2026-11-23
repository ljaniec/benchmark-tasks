"""A plastic ring toss set lying loose on the floor, one ring on a side table and a taped throw line."""
import math
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema
import spec

PEG_COLORS = [(.85, .12, .10), (.10, .35, .85), (.95, .80, .10), (.15, .65, .25), (.95, .50, .10)]


def material(stage, path, color, roughness=.45):
    m = UsdShade.Material.Define(stage, path)
    s = UsdShade.Shader.Define(stage, path + '/surface')
    s.CreateIdAttr('UsdPreviewSurface')
    s.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    s.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(roughness)
    m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(), 'surface')
    return m


def look(stage, prim, path, color, roughness=.45):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material(stage, path, color, roughness))


def root(stage, path, pos, mass=None, quat=None):
    r = UsdGeom.Xform.Define(stage, path)
    r.AddTranslateOp().Set(Gf.Vec3d(*pos))
    r.AddOrientOp().Set(quat or Gf.Quatf(1.))
    if mass is not None:
        UsdPhysics.RigidBodyAPI.Apply(r.GetPrim())
        UsdPhysics.MassAPI.Apply(r.GetPrim()).CreateMassAttr(mass)
        api = PhysxSchema.PhysxRigidBodyAPI.Apply(r.GetPrim())
        api.CreateSolverPositionIterationCountAttr(32)
        api.CreateSolverVelocityIterationCountAttr(0)
        api.CreateMaxDepenetrationVelocityAttr(.5)
        api.CreateEnableCCDAttr(True)
        api.CreateSleepThresholdAttr(0.)
        api.CreateStabilizationThresholdAttr(0.)
        PhysxSchema.PhysxContactReportAPI.Apply(r.GetPrim()).CreateThresholdAttr(0.)
    return r


def collide(stage, prim, contact='/World/contact'):
    UsdPhysics.CollisionAPI.Apply(prim)
    p = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    p.CreateContactOffsetAttr(.001)
    p.CreateRestOffsetAttr(0.)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        UsdShade.Material(stage.GetPrimAtPath(contact)), materialPurpose='physics')


def cube(stage, path, size, pos, color, collision=True, contact='/World/contact', quat=None):
    shape = UsdGeom.Cube.Define(stage, path)
    shape.CreateSizeAttr(1.)
    shape.AddTranslateOp().Set(Gf.Vec3d(*pos))
    if quat is not None:
        shape.AddOrientOp().Set(quat)
    shape.AddScaleOp().Set(Gf.Vec3f(*size))
    if collision:
        collide(stage, shape.GetPrim(), contact)
    look(stage, shape.GetPrim(), path + '_look', color)
    return shape


def box(stage, path, size, pos, color, collision=True):
    root(stage, path, pos)
    return cube(stage, path + '/shape', size, (0., 0., 0.), color, collision)


def zquat(angle):
    return Gf.Quatf(math.cos(angle / 2), 0., 0., math.sin(angle / 2))


def capsule(stage, path, radius, height, pos, axis, color, contact, quat=None):
    c = UsdGeom.Capsule.Define(stage, path)
    c.CreateRadiusAttr(radius)
    c.CreateHeightAttr(height)
    c.CreateAxisAttr(axis)
    c.AddTranslateOp().Set(Gf.Vec3d(*pos))
    if quat is not None:
        c.AddOrientOp().Set(quat)
    collide(stage, c.GetPrim(), contact)
    look(stage, c.GetPrim(), path + '_look', color)
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
    for name, static, dynamic, bounce in [('contact', .65, .55, 0.), ('pad_contact', 1.2, 1., 0.),
                                          ('plastic', spec.PLASTIC_STATIC, spec.PLASTIC_DYNAMIC,
                                           spec.PLASTIC_RESTITUTION)]:
        m = UsdPhysics.MaterialAPI.Apply(UsdShade.Material.Define(stage, '/World/' + name).GetPrim())
        m.CreateStaticFrictionAttr(static)
        m.CreateDynamicFrictionAttr(dynamic)
        m.CreateRestitutionAttr(bounce)
    plastic = '/World/plastic'

    box(stage, '/World/floor', (3.2, 2.8, .06), (.80, -.20, -.03), (.62, .60, .56))
    for side, sign in [('left', 1), ('right', -1)]:
        box(stage, '/World/pedestal_' + side, (.15, .15, .750), (0, sign * .305, .375), (.35, .39, .42))
    # Throw line: 50 mm blue painter's tape on the floor (drawn only).
    box(stage, '/World/throw_line', (.050, 2.2, .001), (spec.LINE_X + .025, -.20, .0005), (.20, .45, .85),
        collision=False)
    sx, sy = spec.TABLE_SIZE
    box(stage, '/World/side_table', (sx, sy, spec.TABLE_TOP_Z), (spec.TABLE_X, spec.TABLE_Y, spec.TABLE_TOP_Z / 2),
        (.55, .42, .30))

    # The ring toss set: one loose plastic body (cross base and five round-tipped pegs).
    root(stage, '/World/game', (spec.SET_X, spec.SET_Y, 0.), spec.SET_MASS)
    w, h = spec.BASE_BAR
    cube(stage, '/World/game/bar_x', (spec.BASE_SPAN, w, h), (0., 0., h / 2), (.92, .92, .90), contact=plastic)
    cube(stage, '/World/game/bar_y', (w, spec.BASE_SPAN, h), (0., 0., h / 2), (.92, .92, .90), contact=plastic)
    r = spec.PEG_D / 2
    length = spec.PEG_TOP_Z - r - h / 2
    for k, ((px, py), color) in enumerate(zip(spec.peg_positions((0., 0.)), PEG_COLORS)):
        capsule(stage, f'/World/game/peg_{k}', r, length, (px, py, h / 2 + length / 2), 'Z', color, plastic)

    # The ring: a 127 mm hoop of 10 mm plastic tube, as 16 overlapping capsules in one body.
    root(stage, '/World/ring', spec.RING_START, spec.RING_MASS)
    n = spec.RING_SEGMENTS
    chord = 2 * spec.RING_MID_R * math.sin(math.pi / n)
    for k in range(n):
        a = 2 * math.pi * (k + .5) / n
        capsule(stage, f'/World/ring/seg_{k:02d}', spec.RING_TUBE_D / 2, chord,
                (spec.RING_MID_R * math.cos(a), spec.RING_MID_R * math.sin(a), 0.), 'Y', (.98, .45, .70), plastic,
                quat=zquat(a))
    return {'ring': '/World/ring', 'game': '/World/game'}


FIXED = ['floor', 'side_table', 'pedestal_left', 'pedestal_right']
