"""A wooden magnetic fishing game on a children's table: a round pond tray with five fish, and a rod lying on the game's box."""
import math
import numpy as np
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema
import spec

CORD_SEGMENTS = 12
WALL_SEGMENTS = 32


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


def root(stage, path, pos, mass=None, quat=None, damping=None):
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
        if damping is not None:
            api.CreateLinearDampingAttr(damping)
            api.CreateAngularDampingAttr(damping)
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


def cylinder(stage, path, radius, height, pos, axis, color, collision=True, contact='/World/contact'):
    c = UsdGeom.Cylinder.Define(stage, path)
    c.CreateRadiusAttr(radius)
    c.CreateHeightAttr(height)
    c.CreateAxisAttr(axis)
    c.AddTranslateOp().Set(Gf.Vec3d(*pos))
    if collision:
        collide(stage, c.GetPrim(), contact)
    look(stage, c.GetPrim(), path + '_look', color)
    return c


def zquat(angle):
    return Gf.Quatf(math.cos(angle / 2), 0., 0., math.sin(angle / 2))


def fish_start(name):
    x, y, heading, _ = spec.FISH[name]
    return (x, y, spec.POND_FLOOR_Z + spec.FISH_T / 2), zquat(math.radians(heading))


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
    for name, static, dynamic in [('contact', .65, .55), ('pad_contact', 1.2, 1.),
                                  ('wood', spec.WOOD_STATIC, spec.WOOD_DYNAMIC)]:
        m = UsdPhysics.MaterialAPI.Apply(UsdShade.Material.Define(stage, '/World/' + name).GetPrim())
        m.CreateStaticFrictionAttr(static)
        m.CreateDynamicFrictionAttr(dynamic)
        m.CreateRestitutionAttr(0.)
    wood = '/World/wood'

    box(stage, '/World/floor', (3.2, 2.8, .06), (.80, -.20, -.03), (.62, .60, .56))
    for side, sign in [('left', 1), ('right', -1)]:
        box(stage, '/World/pedestal_' + side, (.15, .15, .750), (0, sign * .305, .375), (.35, .39, .42))
    tx, ty = spec.TABLE_CENTRE
    sx, sy = spec.TABLE_SIZE
    box(stage, '/World/kids_table', (sx, sy, spec.TABLE_TOP_Z), (tx, ty, spec.TABLE_TOP_Z / 2), (.66, .48, .32))

    # Pond: a round wooden tray with a painted blue floor.
    px, py = spec.POND_CENTRE
    root(stage, '/World/pond', (px, py, spec.TABLE_TOP_Z))
    r_out = spec.POND_INNER_R + spec.POND_WALL
    cylinder(stage, '/World/pond/floor', r_out, spec.POND_FLOOR_T, (0., 0., spec.POND_FLOOR_T / 2), 'Z',
             (.25, .55, .85), contact=wood)
    r_mid = spec.POND_INNER_R + spec.POND_WALL / 2
    chord = 2 * r_out * math.tan(math.pi / WALL_SEGMENTS)
    for k in range(WALL_SEGMENTS):
        a = 2 * math.pi * k / WALL_SEGMENTS
        cube(stage, f'/World/pond/wall_{k:02d}', (spec.POND_WALL, chord, spec.POND_WALL_H),
             (r_mid * math.cos(a), r_mid * math.sin(a), spec.POND_FLOOR_T + spec.POND_WALL_H / 2),
             (.80, .62, .40), contact=wood, quat=zquat(a))

    # The game's box, closed, with the rod lying on it.
    bx, by = spec.BOX_CENTRE
    box(stage, '/World/game_box', spec.BOX_SIZE, (bx, by, spec.TABLE_TOP_Z + spec.BOX_SIZE[2] / 2), (.95, .85, .30))

    # Fish: a flat body, a tail and a steel washer on the back.
    for name, (_, _, _, color) in spec.FISH.items():
        pos, quat = fish_start(name)
        root(stage, f'/World/fish_{name}', pos, spec.FISH_MASS + spec.WASHER_MASS, quat=quat)
        cube(stage, f'/World/fish_{name}/body', (spec.FISH_LEN, spec.FISH_W, spec.FISH_T), (0., 0., 0.), color,
             contact=wood)
        tl, tw = spec.TAIL
        cube(stage, f'/World/fish_{name}/tail', (tl, tw, spec.FISH_T * .8),
             (-spec.FISH_LEN / 2 - tl / 2 + .004, 0., 0.), color, contact=wood)
        cylinder(stage, f'/World/fish_{name}/washer', spec.WASHER_D / 2, spec.WASHER_T,
                 (spec.WASHER_LOCAL_X, 0., spec.FISH_T / 2 + spec.WASHER_T / 2), 'Z', (.72, .74, .76), contact=wood)
        cube(stage, f'/World/fish_{name}/eye', (.008, spec.FISH_W + .001, .008),
             (spec.FISH_LEN / 2 - .014, 0., .0), (.05, .05, .05), collision=False)

    # Rod: a flat wooden stick with a red-painted handle end (drawn band).
    root(stage, '/World/rod', spec.ROD_START, spec.ROD_MASS)
    cube(stage, '/World/rod/stick', (spec.ROD_LEN, spec.ROD_W, spec.ROD_T), (0., 0., 0.), (.86, .72, .52),
         contact=wood)
    cube(stage, '/World/rod/handle', (.080, spec.ROD_W + .001, spec.ROD_T + .001),
         (-spec.ROD_LEN / 2 + .040, 0., 0.), (.80, .15, .12), collision=False)

    # Magnet: a 16 x 8 mm disc.
    root(stage, '/World/magnet', spec.MAGNET_START, spec.MAGNET_MASS, damping=spec.MAGNET_LINEAR_DAMPING)
    cylinder(stage, '/World/magnet/shape', spec.MAGNET_D / 2, spec.MAGNET_T, (0., 0., 0.), 'Z', (.20, .20, .22),
             contact=wood)

    # Cord: an inextensible tether from the rod's tip to the magnet's top. It can go slack.
    j = UsdPhysics.DistanceJoint.Define(stage, '/World/joints/cord')
    j.CreateBody0Rel().SetTargets(['/World/rod'])
    j.CreateBody1Rel().SetTargets(['/World/magnet'])
    j.CreateLocalPos0Attr(Gf.Vec3f(spec.ROD_LEN / 2, 0., 0.))
    j.CreateLocalPos1Attr(Gf.Vec3f(0., 0., spec.MAGNET_T / 2))
    j.CreateLocalRot0Attr(Gf.Quatf(1.))
    j.CreateLocalRot1Attr(Gf.Quatf(1.))
    j.CreateMinDistanceAttr(-1.)
    j.CreateMaxDistanceAttr(spec.CORD_LEN - spec.JOINT_TOLERANCE)
    j.CreateCollisionEnabledAttr(True)
    j.CreateExcludeFromArticulationAttr(True)

    # Rendered cord (no collision), re-shaped every control step.
    UsdGeom.Xform.Define(stage, '/World/cord')
    for k in range(CORD_SEGMENTS):
        seg = UsdGeom.Cylinder.Define(stage, f'/World/cord/seg_{k:02d}')
        seg.CreateAxisAttr('Z')
        seg.CreateRadiusAttr(.0015)
        seg.CreateHeightAttr(1.)
        seg.AddTranslateOp().Set(Gf.Vec3d(*spec.MAGNET_START))
        seg.AddOrientOp().Set(Gf.Quatf(1.))
        seg.AddScaleOp().Set(Gf.Vec3f(1., 1., spec.CORD_LEN / CORD_SEGMENTS))
        look(stage, seg.GetPrim(), f'/World/cord/seg_{k:02d}_look', (.10, .10, .12), .8)
    return {'rod': '/World/rod', 'magnet': '/World/magnet'}


def cord_points(tip, top, n=CORD_SEGMENTS):
    """Points along the cord from the rod's tip to the magnet's top: straight when taut, sagging when slack,
    and lying on the table where the sag would pass below its top."""
    tip, top = np.asarray(tip, float), np.asarray(top, float)
    chord = top - tip
    c = float(np.linalg.norm(chord))
    t = np.linspace(0., 1., n + 1)[:, None]
    pts = tip + t * chord
    slack = spec.CORD_LEN - c
    if slack > 1e-4 and c > 1e-6:
        s = math.sqrt(3 * c * slack / 8)
        pts[:, 2] -= (4 * s * t * (1 - t))[:, 0]
        pts[:, 2] = np.maximum(pts[:, 2], spec.TABLE_TOP_Z + spec.CORD_D / 2)
    return pts


def segment_poses(points):
    """(centre, quaternion wxyz, length) for each cylinder between consecutive points."""
    out = []
    for p0, p1 in zip(points[:-1], points[1:]):
        d = p1 - p0
        length = float(np.linalg.norm(d))
        u = d / length if length > 1e-9 else np.array([0., 0., 1.])
        axis = np.cross([0., 0., 1.], u)
        s = float(np.linalg.norm(axis))
        angle = math.atan2(s, float(u[2]))
        axis = axis / s if s > 1e-9 else np.array([1., 0., 0.])
        out.append(((p0 + p1) / 2, (math.cos(angle / 2), *(axis * math.sin(angle / 2))), max(length, 1e-5)))
    return out


FIXED = ['floor', 'pedestal_left', 'pedestal_right', 'kids_table', 'pond', 'game_box']
