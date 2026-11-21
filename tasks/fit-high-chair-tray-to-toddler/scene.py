"""High chair with a sliding four-position tray, release bail and a seated toddler manikin."""
import math
import numpy as np
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema
import spec


def material(stage, path, color):
    m = UsdShade.Material.Define(stage, path)
    s = UsdShade.Shader.Define(stage, path + '/surface')
    s.CreateIdAttr('UsdPreviewSurface')
    s.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    s.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(.55)
    m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(), 'surface')
    return m


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
    UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(material(stage, path + '/look', color))
    return r


def span(lo, hi):
    """Size and centre of an axis-aligned box from per-axis (low, high) pairs."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return tuple(hi - lo), tuple((hi + lo) / 2)


def block(stage, path, x, y, z, color, **kwargs):
    size, centre = span((x[0], y[0], z[0]), (x[1], y[1], z[1]))
    return box(stage, path, size, centre, color, **kwargs)


def beam(stage, path, p0, p1, thickness, color):
    """Static square tube between two points (chair legs)."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    length = float(np.linalg.norm(d))
    u = d / length
    axis = np.cross([1., 0., 0.], u)
    s = float(np.linalg.norm(axis))
    angle = math.atan2(s, float(u[0]))
    axis = axis / s if s > 1e-9 else np.array([0., 0., 1.])
    q = Gf.Quatf(math.cos(angle / 2), *(axis * math.sin(angle / 2)))
    return box(stage, path, (length, thickness, thickness), tuple((p0 + p1) / 2), color, quat=q)


def capsule(stage, path, p0, p1, radius, color):
    """Static capsule between two points, or a sphere when they coincide."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    length = float(np.linalg.norm(d))
    if length < 1e-9:
        shape = UsdGeom.Sphere.Define(stage, path)
        shape.CreateRadiusAttr(radius)
        shape.AddTranslateOp().Set(Gf.Vec3d(*p0))
    else:
        shape = UsdGeom.Capsule.Define(stage, path)
        shape.CreateAxisAttr('Z')
        shape.CreateRadiusAttr(radius)
        shape.CreateHeightAttr(length)
        shape.AddTranslateOp().Set(Gf.Vec3d(*((p0 + p1) / 2)))
        u = d / length
        axis = np.cross([0., 0., 1.], u)
        s = float(np.linalg.norm(axis))
        angle = math.atan2(s, float(u[2]))
        axis = axis / s if s > 1e-9 else np.array([1., 0., 0.])
        shape.AddOrientOp().Set(Gf.Quatf(math.cos(angle / 2), *(axis * math.sin(angle / 2))))
    collide(stage, shape.GetPrim())
    UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(material(stage, path + '_look', color))
    return shape


def joint(stage, name, kind, b0, b1, p0, p1, axis):
    j = kind.Define(stage, '/World/joints/' + name)
    j.CreateBody0Rel().SetTargets([b0])
    j.CreateBody1Rel().SetTargets([b1])
    j.CreateLocalPos0Attr(Gf.Vec3f(*p0))
    j.CreateLocalPos1Attr(Gf.Vec3f(*p1))
    j.CreateLocalRot0Attr(Gf.Quatf(1.))
    j.CreateLocalRot1Attr(Gf.Quatf(1.))
    j.CreateCollisionEnabledAttr(False)
    j.CreateExcludeFromArticulationAttr(True)
    j.CreateAxisAttr(axis)
    return j


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
                                  ('tray_contact', spec.TRAY_CONTACT_STATIC, spec.TRAY_CONTACT_DYNAMIC)]:
        m = UsdPhysics.MaterialAPI.Apply(UsdShade.Material.Define(stage, '/World/' + name).GetPrim())
        m.CreateStaticFrictionAttr(static)
        m.CreateDynamicFrictionAttr(dynamic)
        m.CreateRestitutionAttr(0.)

    frame = (.93, .93, .91)
    pad = (.42, .46, .50)
    tray_white = (.95, .95, .93)
    dark = (.22, .23, .25)
    skin = (.87, .69, .56)
    onesie = (.96, .78, .82)
    tc = '/World/tray_contact'

    box(stage, '/World/floor', (2.2, 1.8, .06), (.50, 0, -.03), (.62, .55, .45))
    for side, sign in [('left', 1), ('right', -1)]:
        box(stage, '/World/pedestal_' + side, (.15, .15, .750), (0, sign * .305, .375), (.35, .39, .42))

    # Chair frame (fixed): splayed legs, seat shell, backrest, footrest, armrest posts.
    seat_lo = spec.SEAT_TOP - .030
    F, B = spec.SEAT_FRONT_X, spec.BELLY_X
    for sign, n in [(1, 'left'), (-1, 'right')]:
        beam(stage, f'/World/leg_front_{n}', (F - .28, sign * .36, .015), (F, sign * .17, seat_lo), .028, frame)
        beam(stage, f'/World/leg_rear_{n}', (F + .50, sign * .36, .015), (F + .26, sign * .17, seat_lo), .028, frame)
        # Side bolsters stay below the squeezed bail's fingers (lowest about 0.655 m) under the tray.
        block(stage, f'/World/seat_side_{n}', (F, F + .26), tuple(sorted((sign * .150, sign * .158))),
              (spec.SEAT_TOP, .640), pad)
        block(stage, f'/World/arm_post_{n}', (spec.ARM_X1 - .030, spec.ARM_X1),
              (sign * spec.ARM_Y - .012, sign * spec.ARM_Y + .012), (spec.SEAT_TOP, spec.rail_bottom()), frame)
    block(stage, '/World/seat', (F, F + .28), (-.160, .160), (seat_lo, spec.SEAT_TOP), pad)
    block(stage, '/World/backrest', (F + .24, F + .28), (-.165, .165), (spec.SEAT_TOP, 1.08), pad)
    block(stage, '/World/footrest', (F - .05, F + .02), (-.13, .13), (.36, .38), frame)

    # Armrest rails: solid upper part, lower 8 mm segmented by the four notches.
    bottom = spec.rail_bottom()
    for sign, name in [(1, 'left'), (-1, 'right')]:
        y = (sign * spec.ARM_Y - spec.ARM_W / 2, sign * spec.ARM_Y + spec.ARM_W / 2)
        block(stage, f'/World/rail_{name}_upper', (spec.ARM_X0, spec.ARM_X1), y,
              (bottom + spec.NOTCH_DEPTH, spec.ARM_TOP), frame, contact=tc)
        edges = [spec.ARM_X0]
        for x in spec.NOTCH_X:
            edges += [x - spec.NOTCH_LEN / 2, x + spec.NOTCH_LEN / 2]
        edges.append(spec.ARM_X1)
        for i in range(0, len(edges), 2):
            block(stage, f'/World/rail_{name}_lower_{i // 2}', (edges[i], edges[i + 1]), y,
                  (bottom, bottom + spec.NOTCH_DEPTH), frame, contact=tc)
        # Moulded position ticks on the rail's outer face (visual only).
        face = sign * (spec.ARM_Y + spec.ARM_W / 2)
        for k, x in enumerate(spec.NOTCH_X, 1):
            block(stage, f'/World/rail_{name}_tick_{k}', (x - .002, x + .002), (face - .0005, face + .0005),
                  (spec.ARM_TOP - .022, spec.ARM_TOP - .004), dark, collision=False)

    # Toddler manikin, fixed in the seat (harnessed).
    UsdGeom.Xform.Define(stage, '/World/toddler')
    for name, (p0, p1, radius) in spec.TODDLER.items():
        bare = name.split('_')[0] in ('head', 'neck', 'arm', 'hand', 'shin', 'foot')
        capsule(stage, '/World/toddler/' + name, p0, p1, radius, skin if bare else onesie)

    # Tray: plate, rim with a thumb gap at the front centre, and two channels over the rails.
    half_l, half_w = spec.TRAY_LEN / 2, spec.TRAY_WIDTH / 2
    tray = root(stage, '/World/tray', (spec.TRAY_START_X, 0., spec.TRAY_Z), spec.TRAY_MASS)
    block(stage, '/World/tray/plate', (-half_l, half_l), (-half_w, half_w), (0., spec.PLATE_T), tray_white, contact=tc)
    rim = (spec.PLATE_T, spec.PLATE_T + spec.RIM_H)
    block(stage, '/World/tray/rim_back', (half_l - spec.RIM_T, half_l), (-half_w, half_w), rim, tray_white, contact=tc)
    block(stage, '/World/tray/rim_left', (-half_l, half_l), (half_w - spec.RIM_T, half_w), rim, tray_white, contact=tc)
    block(stage, '/World/tray/rim_right', (-half_l, half_l), (-half_w, -half_w + spec.RIM_T), rim, tray_white, contact=tc)
    block(stage, '/World/tray/rim_front_left', (-half_l, -half_l + spec.RIM_T), (spec.RIM_GAP_HALF, half_w), rim,
          tray_white, contact=tc)
    block(stage, '/World/tray/rim_front_right', (-half_l, -half_l + spec.RIM_T), (-half_w, -spec.RIM_GAP_HALF), rim,
          tray_white, contact=tc)
    block(stage, '/World/tray/insert', (-half_l + .03, half_l - .03), (-half_w + .03, half_w - .03),
          (spec.PLATE_T, spec.PLATE_T + .001), (.62, .80, .90), collision=False)
    for sign, name in [(1, 'left'), (-1, 'right')]:
        inner = spec.ARM_Y - spec.ARM_W / 2 - spec.CHANNEL_GAP
        outer = spec.ARM_Y + spec.ARM_W / 2 + spec.CHANNEL_GAP
        for label, a, b in [('inner', inner - spec.CHANNEL_WALL, inner), ('outer', outer, outer + spec.CHANNEL_WALL)]:
            block(stage, f'/World/tray/channel_{name}_{label}', spec.CHANNEL_X, tuple(sorted((sign * a, sign * b))),
                  (-spec.CHANNEL_DEPTH, 0.), tray_white, contact=tc)
        # Pointer marking the locking finger's position (visual only).
        face = sign * (outer + spec.CHANNEL_WALL)
        block(stage, f'/World/tray/pointer_{name}', (spec.FINGER_X - .002, spec.FINGER_X + .002),
              (face - .0005, face + .0005), (-.020, -.004), (.85, .30, .20), collision=False)

    # Release bail (one rigid part): front button, riser, cross bar, two arms and two fingers.
    pivot = np.array(spec.BAIL_PIVOT)
    pivot_world = (spec.TRAY_START_X + pivot[0], pivot[1], spec.TRAY_Z + pivot[2])
    root(stage, '/World/bail', pivot_world, spec.BAIL_MASS)

    def part(name, x, y, z, color=dark):
        block(stage, '/World/bail/' + name, (x[0] - pivot[0], x[1] - pivot[0]), y,
              (z[0] - pivot[2], z[1] - pivot[2]), color, contact=tc)

    az = spec.BAIL_ARM_Z
    part('button', spec.BUTTON_X, (-spec.BUTTON_HALF_WIDTH, spec.BUTTON_HALF_WIDTH), spec.BUTTON_Z, (.30, .55, .80))
    rx = spec.RISER_X
    part('riser', rx, (-spec.BUTTON_HALF_WIDTH, spec.BUTTON_HALF_WIDTH), (az[1], spec.BUTTON_Z[0]))
    part('crossbar', (rx[1], rx[1] + .010), (-spec.ARM_Y - .006, spec.ARM_Y + .006), az)
    for sign, name in [(1, 'left'), (-1, 'right')]:
        y_arm = (sign * spec.ARM_Y - .006, sign * spec.ARM_Y + .006)
        part(f'arm_{name}', (rx[0] + .004, spec.FINGER_X + spec.FINGER_LEN / 2), y_arm, az)
        y_f = (sign * spec.ARM_Y - spec.FINGER_WIDTH / 2, sign * spec.ARM_Y + spec.FINGER_WIDTH / 2)
        part(f'finger_{name}', (spec.FINGER_X - spec.FINGER_LEN / 2, spec.FINGER_X + spec.FINGER_LEN / 2), y_f,
             (az[1], spec.FINGER_TOP))

    j = joint(stage, 'bail_pivot', UsdPhysics.RevoluteJoint, '/World/tray', '/World/bail',
              tuple(pivot), (0., 0., 0.), 'Y')
    # Revolute limits and angular drive values are degrees in UsdPhysics; 0 = fingers up (locked).
    j.CreateLowerLimitAttr(0.)
    j.CreateUpperLimitAttr(spec.BAIL_LIMIT_DEG)
    d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), 'angular')
    d.CreateTypeAttr('force')
    d.CreateStiffnessAttr(spec.BAIL_STIFFNESS)
    d.CreateDampingAttr(spec.BAIL_DAMPING)
    d.CreateTargetPositionAttr(0.)
    d.CreateMaxForceAttr(5.)

    return {'tray': '/World/tray', 'bail': '/World/bail'}


FIXED = ['floor', 'seat', 'backrest', 'footrest', 'rail_left_upper', 'rail_right_upper',
         'toddler/torso_left', 'toddler/head']
