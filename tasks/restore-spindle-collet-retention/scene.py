"""Native eccentric-lip contacts with a reduced, spring-loaded collet nose."""
import math
import numpy as np
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema
import spec

def material(stage,path,color):
    m=UsdShade.Material.Define(stage,path);s=UsdShade.Shader.Define(stage,path+'/surface')
    s.CreateIdAttr('UsdPreviewSurface')
    s.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    s.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.55)
    m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(),'surface');return m

def root(stage,path,pos,mass=None):
    r=UsdGeom.Xform.Define(stage,path);r.AddTranslateOp().Set(Gf.Vec3d(*pos));r.AddOrientOp().Set(Gf.Quatf(1.))
    if mass is not None:
        UsdPhysics.RigidBodyAPI.Apply(r.GetPrim());UsdPhysics.MassAPI.Apply(r.GetPrim()).CreateMassAttr(mass)
        p=PhysxSchema.PhysxRigidBodyAPI.Apply(r.GetPrim())
        p.CreateSolverPositionIterationCountAttr(64);p.CreateSolverVelocityIterationCountAttr(0)
        p.CreateMaxDepenetrationVelocityAttr(.2);p.CreateEnableCCDAttr(True)
    return r

def collision(stage,prim,mesh=False):
    UsdPhysics.CollisionAPI.Apply(prim)
    if mesh:UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('convexHull')
    p=PhysxSchema.PhysxCollisionAPI.Apply(prim);p.CreateContactOffsetAttr(.000025);p.CreateRestOffsetAttr(0.)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(UsdShade.Material(stage.GetPrimAtPath('/World/contact')),materialPurpose='physics')

def box(stage,path,size,pos,color,collide=True):
    r=root(stage,path,pos);s=UsdGeom.Cube.Define(stage,path+'/shape')
    s.CreateSizeAttr(1.);s.AddScaleOp().Set(Gf.Vec3f(*size))
    if collide:collision(stage,s.GetPrim())
    UsdShade.MaterialBindingAPI.Apply(s.GetPrim()).Bind(material(stage,path+'/look',color));return r

def wedge(stage,path,z0,z1,ri0,ri1,ro0,ro1,a0,a1,color,inner_offset=0.):
    angles=[a0,(a0+a1)/2,a1]
    points=[]
    for z,ri,ro in [(z0,ri0,ro0),(z1,ri1,ro1)]:
        points += [(inner_offset+ri*math.cos(a),ri*math.sin(a),z) for a in angles]
        points += [(ro*math.cos(a),ro*math.sin(a),z) for a in angles]
    # Convex hulls preserve the bore as each wedge is a separate shape.
    faces=[[0,1,2,5,4,3],[6,9,10,11,8,7],[0,3,9,6],[2,8,11,5],
           [0,6,7,1],[1,7,8,2],[3,4,10,9],[4,5,11,10]]
    m=UsdGeom.Mesh.Define(stage,path);m.CreatePointsAttr(points)
    m.CreateFaceVertexCountsAttr([len(f) for f in faces]);m.CreateFaceVertexIndicesAttr(sum(faces,[]))
    m.CreateSubdivisionSchemeAttr('none');collision(stage,m.GetPrim(),True)
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(material(stage,path+'_look',color))

def annulus(stage,path,z0,z1,ri0,ri1,ro,color,offset=0.,hexagonal=False):
    for i in range(48):
        a0=i*2*math.pi/48;a1=(i+1)*2*math.pi/48
        # Piecewise convex approximation of the nut's six-flat outer envelope.
        outer=ro if not hexagonal else ro/math.cos(((a0+a1)/2+math.pi/6)%(math.pi/3)-math.pi/6)
        wedge(stage,f'{path}_{i}',z0,z1,ri0,ri1,outer,outer,a0,a1,color,offset)

def build(stage,table=True):
    contact=UsdShade.Material.Define(stage,'/World/contact');m=UsdPhysics.MaterialAPI.Apply(contact.GetPrim())
    m.CreateStaticFrictionAttr(spec.FRICTION);m.CreateDynamicFrictionAttr(spec.FRICTION);m.CreateRestitutionAttr(0.)
    pads=UsdShade.Material.Define(stage,'/World/pad_contact');m=UsdPhysics.MaterialAPI.Apply(pads.GetPrim())
    m.CreateStaticFrictionAttr(spec.PAD_STATIC_FRICTION);m.CreateDynamicFrictionAttr(spec.PAD_DYNAMIC_FRICTION);m.CreateRestitutionAttr(0.)
    if table:
        box(stage,'/World/table',(1.1,1.3,.06),(.36,0,spec.TABLE_Z-.03),(.42,.29,.18))
        box(stage,'/World/pad',(.09,.09,.002),spec.PAD,(.15,.33,.16))
        box(stage,'/World/support',spec.SUPPORT_SIZE,spec.SUPPORT,(.32,.34,.36))
        # A shallow, open locating cup supports the narrow collet tail. It
        # carries lateral assembly loads without latching either workpiece.
        r=spec.BODY_TOP_R+spec.SUPPORT_CUP_CLEARANCE
        annulus(stage,'/World/support/cup',.040,.040+spec.SUPPORT_CUP_DEPTH,
                r,r+spec.SUPPORT_CUP_DEPTH*math.tan(math.radians(8)),
                .020,(.32,.34,.36))
    nut=root(stage,'/World/nut',spec.NUT_START,spec.NUT_MASS)
    nut.GetPrim().GetAttribute('xformOp:orient').Set(Gf.Quatf(0.,0,1,0))
    dark=(.12,.13,.15)
    # Real nut's unused screw threads are represented by their clear bore.
    annulus(stage,'/World/nut/wall',0.,spec.NUT_HEIGHT,spec.NUT_BORE_R,spec.NUT_BORE_R,spec.NUT_AF/2,dark,hexagonal=True)
    annulus(stage,'/World/nut/base',0.,.003,.0135,.0135,.0201,dark)
    annulus(stage,'/World/nut/seat',.003,.0085,.0135,.016675,.0201,dark)
    annulus(stage,'/World/nut/lip',spec.RING_Z-spec.RING_THICKNESS/2,spec.RING_Z+spec.RING_THICKNESS/2,
            spec.RING_INNER_R,spec.RING_INNER_R,.0201,(.30,.32,.35),spec.RING_ECCENTRICITY)
    collet=root(stage,'/World/collet',spec.COLLET_START,spec.COLLET_BODY_MASS)
    collet.GetPrim().GetAttribute('xformOp:orient').Set(Gf.Quatf(0.,0,1,0))
    z0=spec.NOSE_HEIGHT+spec.GROOVE_HEIGHT
    objects={'nut':'/World/nut','collet':'/World/collet'}
    for i in range(spec.NOSE_SECTORS):
        angle=i*2*math.pi/spec.NOSE_SECTORS
        gap=math.radians(.75);a0=angle-math.pi/spec.NOSE_SECTORS+gap;a1=angle+math.pi/spec.NOSE_SECTORS-gap
        wedge(stage,f'/World/collet/body_{i}',z0,spec.COLLET_LENGTH,spec.COLLET_BORE_R,spec.COLLET_BORE_R,
              spec.BODY_MAX_R,spec.BODY_TOP_R,a0,a1,(.62,.65,.69))
        name=f'/World/nose_{i}';nose=root(stage,name,spec.COLLET_START,spec.NOSE_SECTOR_MASS)
        nose.GetPrim().GetAttribute('xformOp:orient').Set(Gf.Quatf(0.,0,1,0))
        wedge(stage,name+'/face',0.,spec.NOSE_HEIGHT,spec.COLLET_BORE_R,spec.COLLET_BORE_R,
              spec.NOSE_MIN_R,spec.NOSE_MAX_R,a0,a1,(.62,.65,.69))
        wedge(stage,name+'/groove',spec.NOSE_HEIGHT,z0,spec.COLLET_BORE_R,spec.COLLET_BORE_R,
              spec.GROOVE_R,spec.GROOVE_R,a0,a1,(.42,.45,.48))
        j=UsdPhysics.PrismaticJoint.Define(stage,f'/World/joints/nose_{i}')
        j.CreateBody0Rel().SetTargets(['/World/collet']);j.CreateBody1Rel().SetTargets([name])
        j.CreateAxisAttr('X');j.CreateLocalPos0Attr(Gf.Vec3f(0.));j.CreateLocalPos1Attr(Gf.Vec3f(0.))
        q=Gf.Quatf(math.cos(angle/2),0,0,math.sin(angle/2))
        j.CreateLocalRot0Attr(q);j.CreateLocalRot1Attr(q)
        j.CreateLowerLimitAttr(-spec.NOSE_RADIAL_LIMIT);j.CreateUpperLimitAttr(.00005)
        j.CreateCollisionEnabledAttr(False);j.CreateExcludeFromArticulationAttr(True)
        d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'linear');d.CreateTypeAttr('force')
        d.CreateStiffnessAttr(spec.NOSE_STIFFNESS);d.CreateDampingAttr(spec.NOSE_DAMPING)
        d.CreateTargetPositionAttr(0.);d.CreateTargetVelocityAttr(0.);d.CreateMaxForceAttr(100.)
        objects[f'nose_{i}']=name
    return objects
