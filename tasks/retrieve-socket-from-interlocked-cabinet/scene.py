"""Native sliding drawers and a contact-driven stacked-bar interlock."""
import math
from pxr import Gf,Sdf,UsdGeom,UsdPhysics,UsdShade,PhysxSchema
import spec

def root(stage,path,pos,mass=None):
    x=UsdGeom.Xform.Define(stage,path);x.AddTranslateOp().Set(Gf.Vec3d(*pos));x.AddOrientOp().Set(Gf.Quatf(1.))
    if mass is not None:
        UsdPhysics.RigidBodyAPI.Apply(x.GetPrim());UsdPhysics.MassAPI.Apply(x.GetPrim()).CreateMassAttr(mass)
        p=PhysxSchema.PhysxRigidBodyAPI.Apply(x.GetPrim());p.CreateSolverPositionIterationCountAttr(96);p.CreateSolverVelocityIterationCountAttr(0);p.CreateSleepThresholdAttr(0.);p.CreateStabilizationThresholdAttr(0.)
    return x

def look(stage,path,color):
    m=UsdShade.Material.Define(stage,path);s=UsdShade.Shader.Define(stage,path+'/surface');s.CreateIdAttr('UsdPreviewSurface')
    s.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color));s.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.45)
    m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(),'surface');return m

def collision(stage,prim):
    UsdPhysics.CollisionAPI.Apply(prim)
    if prim.IsA(UsdGeom.Mesh):UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('convexHull')
    p=PhysxSchema.PhysxCollisionAPI.Apply(prim);p.CreateContactOffsetAttr(.0001);p.CreateRestOffsetAttr(0.)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(UsdShade.Material(stage.GetPrimAtPath('/World/contact')),materialPurpose='physics')

def box(stage,path,size,pos,color):
    x=root(stage,path,pos);s=UsdGeom.Cube.Define(stage,path+'/shape');s.CreateSizeAttr(1.);s.AddScaleOp().Set(Gf.Vec3f(*size));collision(stage,s.GetPrim())
    UsdShade.MaterialBindingAPI.Apply(s.GetPrim()).Bind(look(stage,path+'/look',color));return x

def mesh(stage,path,points,faces,color):
    m=UsdGeom.Mesh.Define(stage,path);m.CreatePointsAttr(points);m.CreateFaceVertexCountsAttr([len(f) for f in faces]);m.CreateFaceVertexIndicesAttr(sum(faces,[]));m.CreateSubdivisionSchemeAttr('none');collision(stage,m.GetPrim())
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(look(stage,path+'_look',color));return m

def prism(stage,path,profile,y0,y1,color):
    pts=[(x,y,z) for y in [y0,y1] for x,z in profile];n=len(profile)
    faces=[list(range(n-1,-1,-1)),list(range(n,2*n))]+[[i,(i+1)%n,(i+1)%n+n,i+n] for i in range(n)]
    return mesh(stage,path,pts,faces,color)

def polygon(stage,path,radius,height,center,color,n=24):
    pts=[(center[0]+radius*math.cos(2*math.pi*i/n),center[1]+radius*math.sin(2*math.pi*i/n),center[2]+z) for z in [-height/2,height/2] for i in range(n)]
    faces=[list(range(n-1,-1,-1)),list(range(n,2*n))]+[[i,(i+1)%n,(i+1)%n+n,i+n] for i in range(n)]
    return mesh(stage,path,pts,faces,color)

def slide(stage,path,body,pos,axis,lo,hi,damping=0.,force=3.):
    j=UsdPhysics.PrismaticJoint.Define(stage,path);j.CreateBody1Rel().SetTargets([body]);j.CreateLocalPos0Attr(Gf.Vec3f(*pos));j.CreateLocalPos1Attr(Gf.Vec3f(0.));j.CreateAxisAttr(axis)
    j.CreateLowerLimitAttr(lo);j.CreateUpperLimitAttr(hi);j.CreateExcludeFromArticulationAttr(True)
    if damping:
        d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'linear');d.CreateTypeAttr('force');d.CreateStiffnessAttr(0.);d.CreateDampingAttr(damping);d.CreateTargetVelocityAttr(0.);d.CreateMaxForceAttr(force)
    return j

def ring(stage,path,outer,inner,z0,z1,n,color,phase=0.):
    # Circular exterior and exact polygon bore; convex sectors keep the hole open.
    count=math.lcm(n,36)
    def inner_radius(a):
        if n>=24:return inner
        normal=phase+math.pi/n
        d=(a-normal+math.pi/n)%(2*math.pi/n)-math.pi/n
        return inner*math.cos(math.pi/n)/math.cos(d)
    for i in range(count):
        angles=[phase+2*math.pi*(i+k)/count for k in [0,1]]
        radii=[outer,outer,inner_radius(angles[1]),inner_radius(angles[0])]
        az=[angles[0],angles[1],angles[1],angles[0]]
        pts=[(r*math.cos(a),r*math.sin(a),z) for z in [z0,z1] for r,a in zip(radii,az)]
        mesh(stage,f'{path}_{i}',pts,[[0,3,2,1],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]],color)


def build(stage):
    for name,mu in [('contact',.25),('pad_contact',1.2)]:
        m=UsdShade.Material.Define(stage,'/World/'+name);p=UsdPhysics.MaterialAPI.Apply(m.GetPrim());p.CreateStaticFrictionAttr(mu);p.CreateDynamicFrictionAttr(1. if name=='pad_contact' else mu);p.CreateRestitutionAttr(0.)
    steel=(.45,.5,.54);blue=(.08,.28,.34);light=(.73,.78,.8);orange=(.90,.47,.08)
    box(stage,'/World/floor',(1.6,1.5,.06),(.5,0,-.03),(.43,.43,.4))
    box(stage,'/World/table',(1.1,1.3,.06),(.63,0,.47),(.48,.35,.22))
    for i,(x,y) in enumerate([(.13,-.59),(.13,.59),(1.1,-.59),(1.1,.59)]):
        box(stage,f'/World/bench_leg_{i}',(.06,.06,.44),(x,y,.22),steel)
    for sign in [-1,1]:box(stage,f'/World/pedestal_{str(sign).replace('-','n')}',(.15,.15,.75),(0,sign*.305,.375),steel)
    root(stage,'/World/cabinet',(0,0,0))
    for sign in [-1,1]:box(stage,f'/World/cabinet/side_{str(sign).replace('-','n')}',(.525,.004,.400),(.7125,sign*.2605,.700),blue)
    for z in [.502,.898]:box(stage,'/World/cabinet/plate_'+str(z).replace('.','_'),(.525,.525,.004),(.7125,0,z),blue)
    box(stage,'/World/cabinet/back',(.004,.525,.400),(.973,0,.700),blue)
    box(stage,'/World/cabinet/header',(.020,.517,.062),(.460,0,.865),blue)
    # Native guide joints reduce the commercial telescoping ball slides.
    objects={}
    for name,z in spec.DRAWER_Z.items():
        q=spec.INITIAL_OPEN[name];path='/World/'+name;root(stage,path,(spec.FRONT-q,0,z),2.5);objects[name]=path
        box(stage,path+'/floor',(.400,.400,.004),(.210,0,.002),light)
        box(stage,path+'/front',(.008,.500,.145),(0,0,.075),light)
        box(stage,path+'/back',(.004,.400,.095),(.408,0,.0495),light)
        for sign in [-1,1]:box(stage,path+f'/wall_{str(sign).replace('-','n')}',(.400,.004,.095),(.210,sign*.202,.0495),light)
        box(stage,path+'/handle',(.020,.120,.020),(-.050,0,.075),(.08,.09,.10))
        for sign in [-1,1]:box(stage,path+f'/handle_stem_{str(sign).replace('-','n')}',(.045,.010,.010),(-.025,sign*.05,.075),steel)
        cz=spec.CAM_Z[name]-z
        # A 20 mm ramp inserts a 6 mm plate between vertically stacked bars.
        prism(stage,path+'/cam',[(.078,cz),(.498,cz),(.498,cz+.006),(.098,cz+.006)],.229,.241,steel)
        slide(stage,'/World/joint_'+name,path,(spec.FRONT,0,z),'X',-.400,0.,30.,3.)
    # Fixed cupboard side cover protects the interlock; bar travel is native.
    for name,z,h,mass in [('middle',.550,.150,.12),('top',.7008,.020,.04)]:
        path='/World/bar_'+name;root(stage,path,(spec.BAR_X,spec.BAR_Y,z+.0061),mass);objects['bar_'+name]=path
        box(stage,path+'/steel',(.010,.012,h),(0,0,h/2),steel)
        slide(stage,'/World/joint_bar_'+name,path,(spec.BAR_X,spec.BAR_Y,z),'Z',0.,spec.BAR_TRAVEL,.2,.2)
    box(stage,'/World/cabinet/interlock_ceiling',(.025,.020,.004),(spec.BAR_X,spec.BAR_Y,.7293),steel)
    # Upright misplaced screwdriver: a real shaft, tip and octagonal handle.
    root(stage,'/World/screwdriver',spec.SCREW_START,.120);objects['screwdriver']='/World/screwdriver'
    box(stage,'/World/screwdriver/tip',(.006,.001,.012),(0,0,.006),steel)
    polygon(stage,'/World/screwdriver/shaft',.003,.095,(0,0,.0595),steel,12)
    polygon(stage,'/World/screwdriver/handle',.016,.100,(0,0,.157),orange,8)
    # Four walls hold only the shaft; no grasp latch or tool attachment.
    for sign in [-1,1]:
        box(stage,f'/World/lower/holder_x_{str(sign).replace('-','n')}',(.008,.024,.050),(.060+sign*.008,.070,.029),blue)
        box(stage,f'/World/lower/holder_y_{str(sign).replace('-','n')}',(.008,.008,.050),(.060,.070+sign*.008,.029),blue)
    root(stage,'/World/socket',spec.SOCKET_START,.080);objects['socket']='/World/socket'
    ring(stage,'/World/socket/hex',.013,.017/math.sqrt(3),0.,.020,6,steel,math.pi/6)
    ring(stage,'/World/socket/drive',.013,.0127/math.sqrt(2),.023,.037,4,steel,math.pi/4)
    # Central web with a small through passage; no solid cube stand-in.
    ring(stage,'/World/socket/web',.013,.003,.020,.023,24,steel)
    box(stage,'/World/mat',spec.MAT_SIZE,spec.MAT_CENTER,(.14,.5,.22))
    box(stage,'/World/tool_pad',(.250,.250,.002),(.250,.430,.501),(.25,.35,.45))
    # Open square tool cup: no latch, only ordinary rigid walls.
    for sign in [-1,1]:
        box(stage,f'/World/tool_pad/cup_x_{str(sign).replace("-","n")}',(.004,.044,.120),(.050+sign*.020,0,.061),blue)
        box(stage,f'/World/tool_pad/cup_y_{str(sign).replace("-","n")}',(.036,.004,.120),(.050,sign*.020,.061),blue)
    return objects
