"""Rigid motor service geometry and a passive, native spring pressure finger."""
import math
from pxr import Gf,Sdf,UsdGeom,UsdPhysics,UsdShade,PhysxSchema
import spec

def material(stage,path,color):
    m=UsdShade.Material.Define(stage,path);s=UsdShade.Shader.Define(stage,path+'/surface')
    s.CreateIdAttr('UsdPreviewSurface');s.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    s.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.55)
    m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(),'surface');return m

def root(stage,path,pos,mass=None):
    r=UsdGeom.Xform.Define(stage,path);r.AddTranslateOp().Set(Gf.Vec3d(*pos));r.AddOrientOp().Set(Gf.Quatf(1.))
    if mass is not None:
        UsdPhysics.RigidBodyAPI.Apply(r.GetPrim());UsdPhysics.MassAPI.Apply(r.GetPrim()).CreateMassAttr(mass)
        p=PhysxSchema.PhysxRigidBodyAPI.Apply(r.GetPrim());p.CreateSolverPositionIterationCountAttr(64)
        p.CreateSolverVelocityIterationCountAttr(0);p.CreateMaxDepenetrationVelocityAttr(.2);p.CreateEnableCCDAttr(True)
    return r

def collision(stage,prim,mesh=False):
    UsdPhysics.CollisionAPI.Apply(prim)
    if mesh:UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('convexHull')
    p=PhysxSchema.PhysxCollisionAPI.Apply(prim);p.CreateContactOffsetAttr(.000025);p.CreateRestOffsetAttr(0.)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(UsdShade.Material(stage.GetPrimAtPath('/World/contact')),materialPurpose='physics')

def box(stage,path,size,pos,color,collide=True):
    r=root(stage,path,pos);s=UsdGeom.Cube.Define(stage,path+'/shape');s.CreateSizeAttr(1.);s.AddScaleOp().Set(Gf.Vec3f(*size))
    if collide:collision(stage,s.GetPrim())
    UsdShade.MaterialBindingAPI.Apply(s.GetPrim()).Bind(material(stage,path+'/look',color));return r

def cylinder(stage,path,radius,height,pos,color,axis='Y',collide=True):
    r=root(stage,path,pos);c=UsdGeom.Cylinder.Define(stage,path+'/shape');c.CreateRadiusAttr(radius);c.CreateHeightAttr(height);c.CreateAxisAttr(axis)
    if collide:collision(stage,c.GetPrim())
    UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(material(stage,path+'/look',color));return r

def build(stage,table=True):
    for name,stat,dyn in [('contact',spec.FRICTION,spec.FRICTION),('pad_contact',spec.PAD_STATIC_FRICTION,spec.PAD_DYNAMIC_FRICTION)]:
        m=UsdShade.Material.Define(stage,'/World/'+name);p=UsdPhysics.MaterialAPI.Apply(m.GetPrim())
        p.CreateStaticFrictionAttr(stat);p.CreateDynamicFrictionAttr(dyn);p.CreateRestitutionAttr(0.)
    t=spec.TABLE_Z;c=spec.CENTER;brass=(.65,.45,.16);steel=(.42,.47,.52)
    if table:box(stage,'/World/table',(1.1,1.3,.06),(.36,0,t-.03),(.42,.29,.18))
    # The motor is bolted to its service stand. Its rear body, end shield,
    # end bearing and side cheeks preserve installed access around the holder.
    housing=root(stage,'/World/housing',(0,0,0))
    box(stage,'/World/housing/foot',(.24,.30,.020),(.32,.035,t+.010),(.18,.23,.27))
    cylinder(stage,'/World/housing/body',.105,.145,(.32,.1125,t+.105),(.18,.23,.27))
    cylinder(stage,'/World/housing/bearing',.030,.020,(.32,-.112,t+.105),steel)
    for x in [.205,.435]:box(stage,f'/World/housing/cheek_{int(x*1000)}',(.010,.150,.175),(x,-.035,t+.1075),(.18,.23,.27))
    box(stage,'/World/housing/end_bottom',(.24,.010,.055),(.32,-.107,t+.0475),(.18,.23,.27))
    cylinder(stage,'/World/collector',spec.COLLECTOR_RADIUS,.130,(c[0],-.035,c[2]-spec.COLLECTOR_RADIUS),(.64,.30,.12))
    # Fine commutator segmentation is visual only; running electrical behavior
    # is outside this isolated, stationary mechanical refitting task.
    for i in range(24):
        a=i*math.tau/24
        z=c[2]-spec.COLLECTOR_RADIUS+(.0601)*math.sin(a);x=c[0]+.0601*math.cos(a)
        b=box(stage,f'/World/segment_{i}',(.00035,.128,.00035),(x,-.035,z),(.18,.12,.09),False)
    holder=root(stage,'/World/holder',c)
    innerx=spec.BRUSH_T+spec.BOX_CLEARANCE;innery=spec.BRUSH_A+spec.BOX_CLEARANCE;wall=.003
    z=spec.BOX_GAP+spec.BOX_HEIGHT/2
    for sign in [-1,1]:
        box(stage,f'/World/holder/x_{sign+1}',(wall,innery+2*wall,spec.BOX_HEIGHT),(sign*(innerx+wall)/2,0,z),brass)
        box(stage,f'/World/holder/y_{sign+1}',(innerx,wall,spec.BOX_HEIGHT),(0,sign*(innery+wall)/2,z),brass)
    box(stage,'/World/holder/mount',(.018,.065,.012),(-.025,0,.009),brass)
    box(stage,'/World/housing/holder_post',(.018,.025,.106),(c[0]-.025,c[1]+.045,t+.115),steel)
    box(stage,'/World/housing/holder_arm',(.018,.065,.012),(c[0]-.025,c[1]+.020,c[2]+.009),brass)
    cylinder(stage,'/World/housing/hinge',.010,.025,spec.PIVOT,brass,collide=False)
    box(stage,'/World/housing/hinge_post',(.012,.022,.074),(spec.PIVOT[0],spec.PIVOT[1],spec.PIVOT[2]-.047),brass)
    box(stage,'/World/housing/spring_bracket',(.060,.022,.014),(.275,-.035,t+.170),brass)
    # Pre-bedded 25 x 32 x 50 mm brush; ten convex strips form the shallow
    # concave contact face rather than replacing it with a flat floating cube.
    brush=root(stage,'/World/brush',spec.BRUSH_START,spec.BRUSH_MASS)
    for i in range(10):
        x0=-spec.BRUSH_T/2+i*spec.BRUSH_T/10;x1=x0+spec.BRUSH_T/10
        z0=math.sqrt(spec.COLLECTOR_RADIUS**2-x0*x0)-spec.COLLECTOR_RADIUS
        z1=math.sqrt(spec.COLLECTOR_RADIUS**2-x1*x1)-spec.COLLECTOR_RADIUS
        pts=[(x,y,z) for x,z in [(x0,z0),(x1,z1)] for y in [-spec.BRUSH_A/2,spec.BRUSH_A/2]]
        pts += [(x,y,spec.BRUSH_R) for x in [x0,x1] for y in [-spec.BRUSH_A/2,spec.BRUSH_A/2]]
        faces=[[0,1,3,2],[4,6,7,5],[0,4,5,1],[2,3,7,6],[0,2,6,4],[1,5,7,3]]
        m=UsdGeom.Mesh.Define(stage,f'/World/brush/strip_{i}');m.CreatePointsAttr(pts);m.CreateFaceVertexCountsAttr([4]*6);m.CreateFaceVertexIndicesAttr(sum(faces,[]));m.CreateSubdivisionSchemeAttr('none')
        collision(stage,m.GetPrim(),True);UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(material(stage,f'/World/brush/look_{i}',(.11,.12,.13)))
    # Pressure finger is a rigid lever driven ONLY by its passive torsion spring.
    finger=root(stage,'/World/finger',spec.PIVOT,spec.FINGER_MASS)
    a=math.radians(spec.FINGER_START_DEG);finger.GetPrim().GetAttribute('xformOp:orient').Set(Gf.Quatf(math.cos(a/2),0,math.sin(a/2),0))
    box(stage,'/World/finger/arm',(.064,.014,.005),(.032,0,0),steel)
    box(stage,'/World/finger/toe',(.009,.014,.030),(.060,0,-.0175),steel)
    box(stage,'/World/finger/pad',(.013,.016,.004),(.060,0,-.030),steel)
    j=UsdPhysics.RevoluteJoint.Define(stage,'/World/pressure_hinge');j.CreateBody1Rel().SetTargets(['/World/finger'])
    j.CreateAxisAttr('Y');j.CreateLocalPos0Attr(Gf.Vec3f(*spec.PIVOT));j.CreateLocalPos1Attr(Gf.Vec3f(0.))
    j.CreateLowerLimitAttr(-85.);j.CreateUpperLimitAttr(22.);j.CreateExcludeFromArticulationAttr(True)
    # USD angular stiffness/damping are per degree; spec is SI per radian.
    d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'angular');d.CreateTypeAttr('force');d.CreateStiffnessAttr(spec.SPRING_STIFFNESS*math.pi/180);d.CreateDampingAttr(spec.SPRING_DAMPING*math.pi/180)
    d.CreateTargetPositionAttr(spec.SPRING_TARGET_DEG);d.CreateTargetVelocityAttr(0.);d.CreateMaxForceAttr(3.)
    return {'brush':'/World/brush','finger':'/World/finger'}
