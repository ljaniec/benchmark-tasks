"""A foosball table: playfield, walls, goals, eight rods of men (only the robot's three-man rod moves) and a ball."""
import math
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema
import spec

ROBOT_COLOR = (.85, .12, .10)
OPPONENT_COLOR = (.10, .30, .85)


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


def cube(stage, path, size, pos, color, collision=True, contact='/World/contact'):
    shape = UsdGeom.Cube.Define(stage, path)
    shape.CreateSizeAttr(1.)
    shape.AddTranslateOp().Set(Gf.Vec3d(*pos))
    shape.AddScaleOp().Set(Gf.Vec3f(*size))
    if collision:
        collide(stage, shape.GetPrim(), contact)
    look(stage, shape.GetPrim(), path + '_look', color)
    return shape


def box(stage, path, size, pos, color, collision=True, contact='/World/contact'):
    """A static box given by its size and centre."""
    root(stage, path, pos)
    return cube(stage, path + '/shape', size, (0., 0., 0.), color, collision, contact)


def cylinder(stage, path, radius, height, pos, axis, color, collision=False, contact='/World/contact', capsule=False):
    c = (UsdGeom.Capsule if capsule else UsdGeom.Cylinder).Define(stage, path)
    c.CreateRadiusAttr(radius)
    c.CreateHeightAttr(height)
    c.CreateAxisAttr(axis)
    c.AddTranslateOp().Set(Gf.Vec3d(*pos))
    if collision:
        collide(stage, c.GetPrim(), contact)
    look(stage, c.GetPrim(), path + '_look', color, .3)
    return c


def man(stage, path, x, color, collision, contact):
    """One foosball man hanging from a rod whose axis is the parent's x axis, local x offset `x`."""
    top, toe = spec.MAN_HEAD_ABOVE, -spec.MAN_FOOT_BELOW
    body_low = toe + spec.FOOT_HEIGHT
    cube(stage, path + '/body', (spec.MAN_WIDTH, spec.MAN_THICK, top - body_low), (x, 0., (top + body_low) / 2),
         color, collision, contact)
    cube(stage, path + '/foot', (spec.FOOT_WIDTH, spec.FOOT_THICK, spec.FOOT_HEIGHT), (x, 0., toe + spec.FOOT_HEIGHT / 2),
         color, collision, contact)


def rod(stage, path, k, mass=None):
    team, offsets = spec.RODS[k]
    y = spec.rod_y(k)
    near = spec.PF_X0
    far = spec.PF_X0 + spec.PF_W
    color = spec_color(team)
    root(stage, path, (spec.X_C, y, spec.ROD_Z), mass)
    contact = '/World/plastic'
    # Shaft (drawn only) from the handle side through both walls.
    if team == 'robot':
        start, end = near - spec.HANDLE_OUT - spec.HANDLE_LEN / 2, far + .06
    else:
        start, end = near - .06, far + spec.HANDLE_OUT + spec.HANDLE_LEN / 2
    cylinder(stage, path + '/shaft', spec.ROD_D / 2, end - start, ((start + end) / 2 - spec.X_C, 0., 0.), 'X',
             (.75, .76, .78))
    handle_x = (near - spec.HANDLE_OUT if team == 'robot' else far + spec.HANDLE_OUT) - spec.X_C
    cylinder(stage, path + '/handle', spec.HANDLE_D / 2, spec.HANDLE_LEN - spec.HANDLE_D, (handle_x, 0., 0.), 'X',
             (.12, .12, .12), collision=True, contact='/World/grip', capsule=True)
    for i, x in enumerate(offsets):
        man(stage, f'{path}/man_{i}', x, color, True, contact)


def spec_color(team):
    return ROBOT_COLOR if team == 'robot' else OPPONENT_COLOR


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
                                          ('grip', .9, .8, 0.),
                                          ('plastic', spec.PLASTIC_STATIC, spec.PLASTIC_DYNAMIC, spec.BALL_RESTITUTION)]:
        m = UsdPhysics.MaterialAPI.Apply(UsdShade.Material.Define(stage, '/World/' + name).GetPrim())
        m.CreateStaticFrictionAttr(static)
        m.CreateDynamicFrictionAttr(dynamic)
        m.CreateRestitutionAttr(bounce)
        if name == 'plastic':
            PhysxSchema.PhysxMaterialAPI.Apply(m.GetPrim()).CreateRestitutionCombineModeAttr('max')
    plastic = '/World/plastic'

    box(stage, '/World/floor', (3.2, 2.8, .06), (.60, -.20, -.03), (.62, .60, .56))
    for side, sign in [('left', 1), ('right', -1)]:
        box(stage, '/World/pedestal_' + side, (.15, .15, .750), (0, sign * .305, .375), (.35, .39, .42))

    x0, w, l, yc, z = spec.PF_X0, spec.PF_W, spec.PF_L, spec.PF_YC, spec.PF_Z
    wood = (.40, .26, .15)
    # Playfield (green) on a cabinet with four legs.
    box(stage, '/World/table/playfield', (w, l, .020), (spec.X_C, yc, z - .010), (.12, .48, .20), contact=plastic)
    box(stage, '/World/table/cabinet', (w + 2 * spec.WALL, l + 2 * spec.WALL, .160), (spec.X_C, yc, z - .100), wood)
    for i, (sx, sy) in enumerate([(-1, -1), (-1, 1), (1, -1), (1, 1)]):
        box(stage, f'/World/table/leg_{i}', (.08, .08, z - .180),
            (spec.X_C + sx * (w / 2 - .04), yc + sy * (l / 2 - .04), (z - .180) / 2), wood)
    # Side walls (rods pass through drilled bearings; the drawn shafts have no collision).
    for name, x in [('near_wall', x0 - spec.WALL / 2), ('far_wall', x0 + w + spec.WALL / 2)]:
        box(stage, '/World/table/' + name, (spec.WALL, l + 2 * spec.WALL, spec.WALL_H),
            (x, yc, z + spec.WALL_H / 2), wood, contact=plastic)
    # End walls with goal mouths, and a closed goal box behind each mouth.
    side = (w - spec.GOAL_W) / 2
    for end, sign in [('opp', -1), ('own', 1)]:
        y = yc + sign * (l / 2 + spec.WALL / 2)
        for part, cx in [('a', x0 + side / 2), ('b', x0 + w - side / 2)]:
            box(stage, f'/World/table/{end}_end_{part}', (side, spec.WALL, spec.END_H), (cx, y, z + spec.END_H / 2), wood,
                contact=plastic)
        box(stage, f'/World/table/{end}_end_top', (spec.GOAL_W, spec.WALL, spec.END_H - spec.GOAL_H),
            (spec.X_C, y, z + (spec.END_H + spec.GOAL_H) / 2), wood, contact=plastic)
        back = y + sign * .10
        box(stage, f'/World/table/{end}_goal_floor', (spec.GOAL_W + .04, .18, .010),
            (spec.X_C, y + sign * .095, z - .040), (.08, .08, .08), contact=plastic)
        box(stage, f'/World/table/{end}_goal_back', (spec.GOAL_W + .04, .010, .12), (spec.X_C, back + sign * .085, z),
            (.08, .08, .08), contact=plastic)
        for gs in (-1, 1):
            box(stage, f'/World/table/{end}_goal_side_{"ab"[gs > 0]}', (.010, .18, .12),
                (spec.X_C + gs * (spec.GOAL_W / 2 + .025), y + sign * .095, z), (.08, .08, .08), contact=plastic)

    for k in spec.RODS:
        if k == spec.ACTIVE_ROD:
            rod(stage, '/World/rod_active', k, spec.ROD_MASS + spec.MAN_MASS * len(spec.RODS[k][1]))
        else:
            rod(stage, f'/World/table/rod_{k}', k)
    # The active rod slides along and turns about its own axis (x), between its bumpers.
    j = UsdPhysics.Joint.Define(stage, '/World/rod_active_bearing')
    j.CreateBody1Rel().SetTargets(['/World/rod_active'])
    j.CreateLocalPos0Attr().Set(Gf.Vec3f(spec.X_C, spec.ROD6_Y, spec.ROD_Z))
    j.CreateLocalRot0Attr().Set(Gf.Quatf(1.))
    j.CreateLocalPos1Attr().Set(Gf.Vec3f(0., 0., 0.))
    j.CreateLocalRot1Attr().Set(Gf.Quatf(1.))
    for dof in ['transY', 'transZ', 'rotY', 'rotZ']:
        lim = UsdPhysics.LimitAPI.Apply(j.GetPrim(), dof)
        lim.CreateLowAttr(1.)
        lim.CreateHighAttr(-1.)
    lim = UsdPhysics.LimitAPI.Apply(j.GetPrim(), 'transX')
    lim.CreateLowAttr(-spec.ROD_TRAVEL)
    lim.CreateHighAttr(spec.ROD_TRAVEL)
    for dof, damping in [('transX', .5), ('rotX', 1e-5)]:  # angular damping per degree
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), dof)
        d.CreateTypeAttr('force')
        d.CreateStiffnessAttr(0.)
        d.CreateDampingAttr(damping)
        d.CreateTargetVelocityAttr(0.)

    root(stage, '/World/ball', spec.BALL_START, spec.BALL_MASS)
    s = UsdGeom.Sphere.Define(stage, '/World/ball/shape')
    s.CreateRadiusAttr(spec.BALL_D / 2)
    collide(stage, s.GetPrim(), plastic)
    look(stage, s.GetPrim(), '/World/ball/shape_look', (.96, .96, .92), .6)
    return {'ball': '/World/ball', 'rod_active': '/World/rod_active'}


FIXED = ['floor', 'pedestal_left', 'pedestal_right', 'table']
