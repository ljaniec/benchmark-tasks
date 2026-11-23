"""A tournament carrom board on its stand, with the striker on the robot's base lines and the red queen."""
import math
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema, Vt
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


def cube(stage, path, size, pos, color, collision=True, contact='/World/contact', roughness=.45, yaw_deg=0.):
    shape = UsdGeom.Cube.Define(stage, path)
    shape.CreateSizeAttr(1.)
    shape.AddTranslateOp().Set(Gf.Vec3d(*pos))
    if yaw_deg:
        shape.AddRotateZOp().Set(yaw_deg)
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


def disc(stage, path, radius, height, xyz, color, roughness=.5):
    """A drawn disc lying on the board (no collision)."""
    d = UsdGeom.Cylinder.Define(stage, path)
    d.CreateAxisAttr('Z')
    d.CreateRadiusAttr(radius)
    d.CreateHeightAttr(height)
    d.AddTranslateOp().Set(Gf.Vec3d(*xyz))
    look(stage, d.GetPrim(), path + '_look', color, roughness)


def strip(stage, path, x0, x1, y0, y1, color=(.02, .02, .02), z=.0002):
    cube(stage, path, (x1 - x0, y1 - y0, .0004), ((x0 + x1) / 2, (y0 + y1) / 2, spec.BOARD_Z + z), color,
         collision=False)


def ring_polygon(lo, hi, centre, r, n=64):
    """Outer square (with its corners) and inner circle points at matching angles around centre."""
    cx, cy = centre
    corners = [math.atan2(y - cy, x - cx) for x, y in [(lo[0], lo[1]), (hi[0], lo[1]), (hi[0], hi[1]), (lo[0], hi[1])]]
    angles = sorted({round(2 * math.pi * k / n - math.pi, 12) for k in range(n)} |
                    {round(a, 12) for a in corners})
    outer, inner = [], []
    for a in angles:
        c, s = math.cos(a), math.sin(a)
        t = min(((hi[0] if c > 0 else lo[0]) - cx) / c if abs(c) > 1e-12 else math.inf,
                ((hi[1] if s > 0 else lo[1]) - cy) / s if abs(s) > 1e-12 else math.inf)
        outer.append((cx + t * c, cy + t * s))
        inner.append((cx + r * c, cy + r * s))
    return outer, inner


def corner_piece(stage, path, lo, hi, centre):
    """A square of the playing surface with a pocket hole cut through it: one welded triangle mesh."""
    z, T = spec.BOARD_Z - spec.SEAM_STEP, spec.BOARD_T
    points, index, faces = [], {}, []

    def v(x, y, zz):
        key = (round(x, 7), round(y, 7), round(zz, 7))
        if key not in index:
            index[key] = len(points)
            points.append(Gf.Vec3f(*key))
        return index[key]

    outer, inner = ring_polygon(lo, hi, centre, spec.POCKET_R)
    n = len(outer)
    for i in range(n):
        j = (i + 1) % n
        o0, o1, i0, i1 = v(*outer[i], z), v(*outer[j], z), v(*inner[i], z), v(*inner[j], z)
        faces += [(i0, o0, o1), (i0, o1, i1)]                      # top, facing up
        b0, b1 = v(*inner[i], z - T), v(*inner[j], z - T)
        faces += [(i0, i1, b1), (i0, b1, b0)]                      # hole wall, facing the hole's axis
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray(points))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(faces)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([i for f in faces for i in f]))
    mesh.CreateSubdivisionSchemeAttr('none')
    mesh.CreateDoubleSidedAttr(True)
    collide(stage, mesh.GetPrim(), '/World/powder')
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('none')
    look(stage, mesh.GetPrim(), path + '_look', BOARD_COLOR, .30)


BOARD_COLOR = (.80, .63, .40)


def playing_surface(stage, path):
    """The plywood playing surface: three flat boxes over the open board, and the four corner pieces with the
    pocket holes. The end strips and corner pieces sit SEAM_STEP below the middle, so a disc sliding out of the
    middle steps down onto them instead of catching on the seam between two collision shapes."""
    X0, X1, Y0, Y1, C, z, T = spec.X0, spec.X1, spec.Y0, spec.Y1, spec.CORNER, spec.BOARD_Z, spec.BOARD_T
    for name, lo, hi, top in [('middle', (X0 + C, Y0), (X1 - C, Y1), z),
                              ('near', (X0, Y0 + C), (X0 + C, Y1 - C), z - spec.SEAM_STEP),
                              ('far', (X1 - C, Y0 + C), (X1, Y1 - C), z - spec.SEAM_STEP)]:
        cube(stage, f'{path}_{name}', (hi[0] - lo[0], hi[1] - lo[1], T), ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2,
             top - T / 2), BOARD_COLOR, contact='/World/powder', roughness=.30)
    for i, (px, py) in enumerate(spec.POCKETS):
        lo = (X0 if px < (X0 + X1) / 2 else X1 - C, Y0 if py < (Y0 + Y1) / 2 else Y1 - C)
        corner_piece(stage, f'{path}_corner_{i}', lo, (lo[0] + C, lo[1] + C), (px, py))


def markings(stage):
    """Base lines, base circles, arrows, centre and outer circles, drawn on all four sides."""
    X0, X1, Y0, Y1, cy = spec.X0, spec.X1, spec.Y0, spec.Y1, spec.BOARD_Y
    cx = (X0 + X1) / 2
    black, red = (.02, .02, .02), (.70, .05, .04)
    half = spec.BASE_CIRCLE_Y
    # Offsets of the two lines from a frame edge.
    lines = [(spec.BASE_OUTER[0] - X0, spec.BASE_OUTER[1] - X0), (spec.BASE_INNER[0] - X0, spec.BASE_INNER[1] - X0)]
    k = 0
    for a, b in lines:
        strip(stage, f'/World/line_{k}', X0 + a, X0 + b, cy - half, cy + half); k += 1
        strip(stage, f'/World/line_{k}', X1 - b, X1 - a, cy - half, cy + half); k += 1
        strip(stage, f'/World/line_{k}', cx - half, cx + half, Y0 + a, Y0 + b); k += 1
        strip(stage, f'/World/line_{k}', cx - half, cx + half, Y1 - b, Y1 - a); k += 1
    off = spec.BASE_CIRCLE_X - X0
    circles = []
    for s in (-1, 1):
        circles += [(X0 + off, cy + s * half), (X1 - off, cy + s * half),
                    (cx + s * half, Y0 + off), (cx + s * half, Y1 - off)]
    for i, (x, y) in enumerate(circles):
        disc(stage, f'/World/base_circle_{i}', spec.BASE_CIRCLE_R, .0004, (x, y, spec.BOARD_Z + .0002), black)
        disc(stage, f'/World/base_circle_{i}_red', .0254 / 2, .0004, (x, y, spec.BOARD_Z + .0004), red)
    # Arrows at 45 degrees, pointing at each pocket and stopping 5 cm short of its edge.
    for i, (px, py) in enumerate(spec.POCKETS):
        ux, uy = (cx - px) / math.hypot(cx - px, cy - py), (cy - py) / math.hypot(cx - px, cy - py)
        start, length = spec.POCKET_R + .050, .200
        mx, my = px + ux * (start + length / 2), py + uy * (start + length / 2)
        cube(stage, f'/World/arrow_{i}', (length, .0015, .0004), (mx, my, spec.BOARD_Z + .0002), black,
             collision=False, yaw_deg=math.degrees(math.atan2(uy, ux)))
    disc(stage, '/World/outer_circle', .085, .0004, (cx, cy, spec.BOARD_Z + .0002), black)
    disc(stage, '/World/outer_circle_in', .0835, .0004, (cx, cy, spec.BOARD_Z + .0004), BOARD_COLOR, .30)
    disc(stage, '/World/centre_circle', .0318 / 2, .0004, (cx, cy, spec.BOARD_Z + .0006), black)
    disc(stage, '/World/centre_circle_red', .0288 / 2, .0004, (cx, cy, spec.BOARD_Z + .0008), red)


def piece(stage, path, start, radius, height, mass, color, contact, roughness):
    """A disc whose origin is the centre of its base."""
    r = root(stage, path, start, mass)
    UsdPhysics.MassAPI(r.GetPrim()).CreateCenterOfMassAttr(Gf.Vec3f(0., 0., height / 2))
    c = UsdGeom.Cylinder.Define(stage, path + '/body')
    c.CreateAxisAttr('Z')
    c.CreateRadiusAttr(radius)
    c.CreateHeightAttr(height)
    c.AddTranslateOp().Set(Gf.Vec3d(0., 0., height / 2))
    look(stage, c.GetPrim(), path + '/body_look', color, roughness)
    # Collision: a 48-sided prism with its top and bottom edges bevelled by spec.BEVEL.
    b, n, rings = spec.BEVEL, 48, []
    for z, rr in ((0., radius - b), (b, radius), (height - b, radius), (height, radius - b)):
        rings.append([Gf.Vec3f(rr * math.cos(2 * math.pi * k / n), rr * math.sin(2 * math.pi * k / n), z)
                      for k in range(n)])
    faces = [[k for k in range(n)][::-1], [3 * n + k for k in range(n)]]
    for i in range(3):
        faces += [[i * n + k, i * n + (k + 1) % n, (i + 1) * n + (k + 1) % n, (i + 1) * n + k] for k in range(n)]
    m = UsdGeom.Mesh.Define(stage, path + '/hull')
    m.CreatePointsAttr(Vt.Vec3fArray([p for ring in rings for p in ring]))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([len(f) for f in faces]))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray([i for f in faces for i in f]))
    m.CreateVisibilityAttr('invisible')
    collide(stage, m.GetPrim(), contact, offset=.001)
    UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr('convexHull')
    PhysxSchema.PhysxConvexHullCollisionAPI.Apply(m.GetPrim()).CreateHullVertexLimitAttr(4 * n)
    return r


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
        if name == 'powder':
            px = PhysxSchema.PhysxMaterialAPI.Apply(prim)
            px.CreateFrictionCombineModeAttr('min')
            px.CreateRestitutionCombineModeAttr('min')

    span(stage, '/World/floor', (-.90, -1.20, -.06), (1.60, 1.20, 0.), (.62, .60, .56))
    for side, sign in [('left', 1), ('right', -1)]:
        span(stage, '/World/pedestal_' + side, (-.075, sign * .305 - .075, 0.), (.075, sign * .305 + .075, .750),
             (.35, .39, .42))

    (ox0, oy0), (ox1, oy1) = spec.OUTER
    X0, X1, Y0, Y1, W = spec.X0, spec.X1, spec.Y0, spec.Y1, spec.FRAME_W
    under = spec.BOARD_Z - spec.BOARD_T
    rosewood, stand, net = (.30, .12, .07), (.20, .14, .10), (.06, .06, .07)
    # Stand: four legs under the frame's corners.
    for i, (x, y) in enumerate([(ox0 + .04, oy0 + .04), (ox1 - .04, oy0 + .04), (ox0 + .04, oy1 - .04),
                                (ox1 - .04, oy1 - .04)]):
        span(stage, f'/World/leg_{i}', (x - .03, y - .03, 0.), (x + .03, y + .03, under - .030), stand,
             contact='/World/wood')
    # Frame: rosewood, overlapping the board's edges by the corners only.
    top = spec.BOARD_Z + spec.FRAME_H
    for name, lo, hi in [('frame_near', (ox0, oy0), (X0, oy1)), ('frame_far', (X1, oy0), (ox1, oy1)),
                         ('frame_right', (X0, oy0), (X1, Y0)), ('frame_left', (X0, Y1), (X1, oy1))]:
        span(stage, '/World/' + name, (*lo, under - .030), (*hi, top), rosewood, contact='/World/rosewood',
             roughness=.35)
    # Pocket nets: a closed box under each corner piece.
    C = spec.CORNER
    for i, (px, py) in enumerate(spec.POCKETS):
        x0 = X0 if px < (X0 + X1) / 2 else X1 - C
        y0 = Y0 if py < (Y0 + Y1) / 2 else Y1 - C
        bottom = under - spec.NET_DEPTH
        span(stage, f'/World/net_{i}_floor', (x0, y0, bottom - .004), (x0 + C, y0 + C, bottom), net,
             contact='/World/net', roughness=.9)
        for j, (lo, hi) in enumerate([((x0, y0), (x0 + .004, y0 + C)), ((x0 + C - .004, y0), (x0 + C, y0 + C)),
                                      ((x0, y0), (x0 + C, y0 + .004)), ((x0, y0 + C - .004), (x0 + C, y0 + C))]):
            span(stage, f'/World/net_{i}_wall_{j}', (*lo, bottom), (*hi, under - .0005), net, contact='/World/net',
                 roughness=.9)
    root(stage, '/World/board', (0., 0., 0.))
    playing_surface(stage, '/World/board/surface')
    markings(stage)

    piece(stage, '/World/striker', spec.STRIKER_START, spec.STRIKER_R, spec.STRIKER_H, spec.STRIKER_MASS,
          (.08, .20, .55), '/World/striker_disc', .25)
    piece(stage, '/World/queen', spec.QUEEN_START, spec.QUEEN_R, spec.QUEEN_H, spec.QUEEN_MASS,
          (.78, .06, .05), '/World/queen_disc', .40)
    return {'striker': '/World/striker', 'queen': '/World/queen'}


FIXED = ['floor', 'pedestal_left', 'pedestal_right', 'board', 'frame_near', 'frame_far', 'frame_right', 'frame_left']
