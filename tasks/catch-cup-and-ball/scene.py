"""Wooden cup-and-ball toy standing in a floor stand's sleeve, its ball hanging on a 533 mm string."""
import math
import numpy as np
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema
import spec

STRING_SEGMENTS = 16
STRING_RADIUS = .0012


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
        api.CreateSolverPositionIterationCountAttr(48)
        api.CreateSolverVelocityIterationCountAttr(0)
        api.CreateMaxDepenetrationVelocityAttr(.2)
        api.CreateEnableCCDAttr(True)
        # A slowly swinging ball is below PhysX's default sleep energy, and bodies about to sleep are
        # damped; the toy and ball must never sleep.
        api.CreateSleepThresholdAttr(0.)
        api.CreateStabilizationThresholdAttr(0.)
        PhysxSchema.PhysxContactReportAPI.Apply(r.GetPrim()).CreateThresholdAttr(0.)
    return r


def collide(stage, prim, contact='/World/contact'):
    UsdPhysics.CollisionAPI.Apply(prim)
    p = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    p.CreateContactOffsetAttr(.0005)
    p.CreateRestOffsetAttr(0.)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        UsdShade.Material(stage.GetPrimAtPath(contact)), materialPurpose='physics')


def box(stage, path, size, pos, color, mass=None, quat=None, collision=True, contact='/World/contact'):
    r = root(stage, path, pos, mass, quat)
    shape = UsdGeom.Cube.Define(stage, path + '/shape')
    shape.CreateSizeAttr(1.)
    shape.AddScaleOp().Set(Gf.Vec3f(*size))
    if collision:
        collide(stage, shape.GetPrim(), contact)
    look(stage, shape.GetPrim(), path + '/look', color)
    return r


def zquat(angle):
    return Gf.Quatf(math.cos(angle / 2), 0., 0., math.sin(angle / 2))


def prism(stage, path, radius, z0, z1, n, color, contact):
    """Convex n-gon prism about the local z axis (a disc or rod), centred on the parent's axis."""
    mesh = UsdGeom.Mesh.Define(stage, path)
    ring = [(radius * math.cos(2 * math.pi * k / n), radius * math.sin(2 * math.pi * k / n)) for k in range(n)]
    mesh.CreatePointsAttr([Gf.Vec3f(x, y, z0) for x, y in ring] + [Gf.Vec3f(x, y, z1) for x, y in ring])
    faces, counts = [], []
    for k in range(n):
        a, b = k, (k + 1) % n
        faces += [a, b, n + b, n + a]
        counts.append(4)
    faces += list(range(n - 1, -1, -1)) + list(range(n, 2 * n))
    counts += [n, n]
    mesh.CreateFaceVertexIndicesAttr(faces)
    mesh.CreateFaceVertexCountsAttr(counts)
    collide(stage, mesh.GetPrim(), contact)
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexHull')
    look(stage, mesh.GetPrim(), path + '_look', color)
    return mesh


def ring(stage, parent, name, inner, outer, z0, z1, n, color, contact, skip=()):
    """Tube wall as n boxes whose outer corners meet, centred on the parent's z axis; box k faces angle 2 pi k / n."""
    half = math.pi / n
    length = 2 * outer * math.tan(half)
    for k in range(n):
        if k in skip:
            continue
        a = 2 * half * k
        r = (inner + outer) / 2
        part = UsdGeom.Cube.Define(stage, f'{parent}/{name}_{k:02d}')
        part.CreateSizeAttr(1.)
        part.AddTranslateOp().Set(Gf.Vec3d(r * math.cos(a), r * math.sin(a), (z0 + z1) / 2))
        part.AddOrientOp().Set(zquat(a))
        part.AddScaleOp().Set(Gf.Vec3f(outer - inner, length, z1 - z0))
        collide(stage, part.GetPrim(), contact)
        look(stage, part.GetPrim(), f'{parent}/{name}_{k:02d}_look', color)


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
                                          ('wood', spec.WOOD_STATIC, spec.WOOD_DYNAMIC, spec.WOOD_RESTITUTION)]:
        m = UsdPhysics.MaterialAPI.Apply(UsdShade.Material.Define(stage, '/World/' + name).GetPrim())
        m.CreateStaticFrictionAttr(static)
        m.CreateDynamicFrictionAttr(dynamic)
        m.CreateRestitutionAttr(bounce)
    wood = '/World/wood'
    pine = (.80, .66, .46)
    dark = (.30, .27, .24)

    box(stage, '/World/floor', (2.4, 2.4, .06), (.40, -.20, -.03), (.62, .60, .56))
    for side, sign in [('left', 1), ('right', -1)]:
        box(stage, '/World/pedestal_' + side, (.15, .15, .750), (0, sign * .305, .375), (.35, .39, .42))

    # Floor stand (fixed): base board, square post, bracket and the slotted sleeve the handle stands in.
    sx, sy = spec.STAND_X, spec.STAND_Y
    bx, by, bz = spec.BASE_SIZE
    box(stage, '/World/stand_base', spec.BASE_SIZE, (spec.POST_X - .05, sy, bz / 2), dark)
    post_top = spec.SLEEVE_BOTTOM_Z + spec.SLEEVE_HEIGHT / 2
    box(stage, '/World/stand_post', (spec.POST_SIZE, spec.POST_SIZE, post_top - bz),
        (spec.POST_X, sy, bz + (post_top - bz) / 2), pine)
    outer = spec.SLEEVE_INNER_R + spec.SLEEVE_WALL
    start = sx + outer - .004
    arm_len = spec.POST_X - spec.POST_SIZE / 2 - start
    box(stage, '/World/stand_bracket', (arm_len, .024, .020), (start + arm_len / 2, sy, post_top - .010), pine)
    root(stage, '/World/stand_sleeve', (sx, sy, spec.SLEEVE_BOTTOM_Z))
    floor = UsdGeom.Cube.Define(stage, '/World/stand_sleeve/floor')
    floor.CreateSizeAttr(1.)
    floor_y = (spec.TIE_LOCAL[1] + .006, outer)
    floor.AddTranslateOp().Set(Gf.Vec3d(0., sum(floor_y) / 2, -.004))
    floor.AddScaleOp().Set(Gf.Vec3f(2 * outer, floor_y[1] - floor_y[0], .008))
    collide(stage, floor.GetPrim())
    look(stage, floor.GetPrim(), '/World/stand_sleeve/floor_look', pine)
    ring(stage, '/World/stand_sleeve', 'wall', spec.SLEEVE_INNER_R, outer, 0., spec.SLEEVE_HEIGHT,
         spec.SLEEVE_SEGMENTS, pine, '/World/contact', skip=(spec.SLEEVE_SLOT,))

    # The toy: one rigid body (lacquered cup on a turned handle).
    root(stage, '/World/toy', spec.TOY_START, spec.TOY_MASS)
    red, natural = (.72, .10, .08), (.86, .72, .52)
    prism(stage, '/World/toy/cup_floor', spec.CUP_OUTER_R, 0., spec.CUP_FLOOR_T, spec.WALL_SEGMENTS, red, wood)
    ring(stage, '/World/toy', 'cup_wall', spec.CUP_INNER_R, spec.CUP_OUTER_R, spec.CUP_FLOOR_T, spec.CUP_HEIGHT,
         spec.WALL_SEGMENTS, red, wood)
    handle = UsdGeom.Capsule.Define(stage, '/World/toy/handle')
    handle.CreateAxisAttr('Z')
    handle.CreateRadiusAttr(spec.HANDLE_R)
    handle.CreateHeightAttr(spec.HANDLE_LEN - 2 * spec.HANDLE_R)
    handle.AddTranslateOp().Set(Gf.Vec3d(0., 0., -spec.HANDLE_LEN / 2))
    collide(stage, handle.GetPrim(), wood)
    look(stage, handle.GetPrim(), '/World/toy/handle_look', natural)

    # The ball, hanging still on its full string.
    root(stage, '/World/ball', spec.ball_start(), spec.BALL_MASS)
    PhysxSchema.PhysxRigidBodyAPI(stage.GetPrimAtPath('/World/ball')).CreateLinearDampingAttr(spec.BALL_LINEAR_DAMPING)
    ball = UsdGeom.Sphere.Define(stage, '/World/ball/shape')
    ball.CreateRadiusAttr(spec.BALL_R)
    collide(stage, ball.GetPrim(), wood)
    look(stage, ball.GetPrim(), '/World/ball/look', (.95, .78, .10), .35)

    # The string: an inextensible tether from the knot to the ball's centre. It can go slack, and the
    # ball and toy still collide with each other.
    j = UsdPhysics.DistanceJoint.Define(stage, '/World/joints/string')
    j.CreateBody0Rel().SetTargets(['/World/toy'])
    j.CreateBody1Rel().SetTargets(['/World/ball'])
    j.CreateLocalPos0Attr(Gf.Vec3f(*spec.TIE_LOCAL))
    j.CreateLocalPos1Attr(Gf.Vec3f(0., 0., 0.))
    j.CreateLocalRot0Attr(Gf.Quatf(1.))
    j.CreateLocalRot1Attr(Gf.Quatf(1.))
    j.CreateMinDistanceAttr(-1.)
    j.CreateMaxDistanceAttr(spec.STRING_LEN + spec.BALL_R - spec.JOINT_TOLERANCE)
    j.CreateCollisionEnabledAttr(True)
    j.CreateExcludeFromArticulationAttr(True)

    # Rendered string (no collision), re-shaped every control step from the knot and the ball.
    UsdGeom.Xform.Define(stage, '/World/string')
    for k in range(STRING_SEGMENTS):
        seg = UsdGeom.Cylinder.Define(stage, f'/World/string/seg_{k:02d}')
        seg.CreateAxisAttr('Z')
        seg.CreateRadiusAttr(STRING_RADIUS)
        seg.CreateHeightAttr(1.)
        seg.AddTranslateOp().Set(Gf.Vec3d(*spec.ball_start()))
        seg.AddOrientOp().Set(Gf.Quatf(1.))
        seg.AddScaleOp().Set(Gf.Vec3f(1., 1., spec.STRING_LEN / STRING_SEGMENTS))
        look(stage, seg.GetPrim(), f'/World/string/seg_{k:02d}_look', (.93, .91, .86), .8)
    return {'toy': '/World/toy', 'ball': '/World/ball'}


def string_points(tie, ball, n=STRING_SEGMENTS):
    """Points along the string from the knot to the ball's top: straight when taut, sagging when slack."""
    tie, ball = np.asarray(tie, float), np.asarray(ball, float)
    d = ball - tie
    dist = float(np.linalg.norm(d))
    end = ball - (d / dist if dist > 1e-9 else np.array([0., 0., 1.])) * spec.BALL_R
    chord = end - tie
    c = float(np.linalg.norm(chord))
    t = np.linspace(0., 1., n + 1)[:, None]
    pts = tie + t * chord
    slack = spec.STRING_LEN - c
    if slack > 1e-4 and c > 1e-6:
        # Parabolic sag with the string's length: arc ~ c + 8 s^2 / (3 c) for sag s at midspan.
        s = math.sqrt(3 * c * slack / 8)
        pts[:, 2] -= (4 * s * t * (1 - t))[:, 0]
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


FIXED = ['floor', 'stand_post', 'stand_sleeve']
