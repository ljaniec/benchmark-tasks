"""Native contacts for the mount, loose driver and optical test block."""
import math
import numpy as np
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema
import spec

def material(stage,path,color,emission=False):
    m=UsdShade.Material.Define(stage,path);s=UsdShade.Shader.Define(stage,path+'/surface')
    s.CreateIdAttr('UsdPreviewSurface');s.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    s.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.65)
    if emission:s.CreateInput('emissiveColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(),'surface');return m

def root(stage,path,pos,mass=None,yaw=0.,quat=None):
    r=UsdGeom.Xform.Define(stage,path);r.AddTranslateOp().Set(Gf.Vec3d(*pos))
    r.AddOrientOp().Set(quat or Gf.Quatf(math.cos(yaw/2),0,0,math.sin(yaw/2)))
    if mass is not None:
        UsdPhysics.RigidBodyAPI.Apply(r.GetPrim());UsdPhysics.MassAPI.Apply(r.GetPrim()).CreateMassAttr(mass)
        api=PhysxSchema.PhysxRigidBodyAPI.Apply(r.GetPrim())
        api.CreateSolverPositionIterationCountAttr(64);api.CreateSolverVelocityIterationCountAttr(16)
        # Limit recovery speed for transient tool/finger penetrations.
        api.CreateMaxDepenetrationVelocityAttr(.2)
        api.CreateEnableCCDAttr(True)
        PhysxSchema.PhysxContactReportAPI.Apply(r.GetPrim()).CreateThresholdAttr(0.)
    return r

def box(stage,path,size,pos,color,mass=None,yaw=0.,collision=True,mat=None):
    r=root(stage,path,pos,mass,yaw);shape=UsdGeom.Cube.Define(stage,path+'/shape')
    shape.CreateSizeAttr(1.);shape.AddScaleOp().Set(Gf.Vec3f(*size))
    if collision:
        UsdPhysics.CollisionAPI.Apply(shape.GetPrim());p=PhysxSchema.PhysxCollisionAPI.Apply(shape.GetPrim())
        p.CreateContactOffsetAttr(.00005);p.CreateRestOffsetAttr(0.)
        UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(UsdShade.Material(stage.GetPrimAtPath('/World/contact')),materialPurpose='physics')
    UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(mat or material(stage,path+'/look',color))
    return r

def cylinder(stage,path,radius,height,pos,color,mass=None,sides=32,quat=None,collision=True):
    r=root(stage,path,pos,mass,quat=quat)
    points=[(radius*math.cos(a),radius*math.sin(a),z) for z in [-height/2,height/2] for a in np.arange(sides)*2*math.pi/sides]
    faces=[list(reversed(range(sides))),list(range(sides,2*sides))]
    faces += [[i,(i+1)%sides,(i+1)%sides+sides,i+sides] for i in range(sides)]
    m=UsdGeom.Mesh.Define(stage,path+'/shape');m.CreatePointsAttr(points)
    m.CreateFaceVertexCountsAttr([len(f) for f in faces]);m.CreateFaceVertexIndicesAttr(sum(faces,[]));m.CreateSubdivisionSchemeAttr('none')
    if collision:
        UsdPhysics.CollisionAPI.Apply(m.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr('convexHull')
        ph=PhysxSchema.PhysxCollisionAPI.Apply(m.GetPrim());ph.CreateContactOffsetAttr(.00003);ph.CreateRestOffsetAttr(0.)
        UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(UsdShade.Material(stage.GetPrimAtPath('/World/contact')),materialPurpose='physics')
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(material(stage,path+'/look',color));return r

def joint(stage,name,kind,b0,b1,p0,p1,axis='Z',angle0=0.):
    j=kind.Define(stage,'/World/joints/'+name)
    j.CreateBody0Rel().SetTargets([b0] if b0 else []);j.CreateBody1Rel().SetTargets([b1])
    j.CreateLocalPos0Attr(Gf.Vec3f(*p0));j.CreateLocalPos1Attr(Gf.Vec3f(*p1))
    j.CreateLocalRot0Attr(Gf.Quatf(math.cos(angle0/2),0,0,math.sin(angle0/2)))
    j.CreateLocalRot1Attr(Gf.Quatf(1.));j.CreateCollisionEnabledAttr(True)
    j.CreateExcludeFromArticulationAttr(True)
    if hasattr(j,'CreateAxisAttr'):j.CreateAxisAttr(axis)
    return j

def socket_wall(stage,path,angle):
    """Estimated 0.3 mm radial, 1 mm deep entry bevel on an 8 mm socket."""
    root(stage,path,(0,0,0),yaw=angle)
    section=[(.00545,0.),(.00545,.012),(.00405,.012),(.00405,.001),(.00435,0.)]
    points=[(r,t,z) for t in [-.00275,.00275] for r,z in section]
    faces=[list(range(5)),list(reversed(range(5,10)))]
    faces += [[i,i+5,(i+1)%5+5,(i+1)%5] for i in range(5)]
    shape=UsdGeom.Mesh.Define(stage,path+'/shape');shape.CreatePointsAttr(points)
    shape.CreateFaceVertexCountsAttr([len(f) for f in faces]);shape.CreateFaceVertexIndicesAttr(sum(faces,[]))
    shape.CreateSubdivisionSchemeAttr('none')
    UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(shape.GetPrim()).CreateApproximationAttr('convexHull')
    ph=PhysxSchema.PhysxCollisionAPI.Apply(shape.GetPrim());ph.CreateContactOffsetAttr(.00003);ph.CreateRestOffsetAttr(0.)
    UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(UsdShade.Material(stage.GetPrimAtPath('/World/contact')),materialPurpose='physics')
    UsdShade.MaterialBindingAPI(shape.GetPrim()).Bind(material(stage,path+'/look',(.55,.58,.60)))

def build(stage):
    UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,'Z')
    ph=UsdPhysics.Scene.Define(stage,'/World/physics');ph.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1));ph.CreateGravityMagnitudeAttr(9.81)
    api=PhysxSchema.PhysxSceneAPI.Apply(ph.GetPrim());api.CreateTimeStepsPerSecondAttr(spec.PHYSICS_HZ)
    api.CreateEnableGPUDynamicsAttr(False);api.CreateBroadphaseTypeAttr('MBP');api.CreateEnableCCDAttr(True)
    contact=UsdShade.Material.Define(stage,'/World/contact');m=UsdPhysics.MaterialAPI.Apply(contact.GetPrim())
    m.CreateStaticFrictionAttr(spec.FRICTION);m.CreateDynamicFrictionAttr(spec.FRICTION);m.CreateRestitutionAttr(0.)
    pad=UsdShade.Material.Define(stage,'/World/pad_contact')
    m=UsdPhysics.MaterialAPI.Apply(pad.GetPrim())
    m.CreateStaticFrictionAttr(spec.PAD_STATIC_FRICTION);m.CreateDynamicFrictionAttr(spec.PAD_DYNAMIC_FRICTION);m.CreateRestitutionAttr(0.)
    axial=UsdShade.Material.Define(stage,'/World/axial_contact')
    m=UsdPhysics.MaterialAPI.Apply(axial.GetPrim())
    m.CreateStaticFrictionAttr(0.);m.CreateDynamicFrictionAttr(0.);m.CreateRestitutionAttr(0.)
    PhysxSchema.PhysxMaterialAPI.Apply(axial.GetPrim()).CreateFrictionCombineModeAttr('min')
    box(stage,'/World/table',(1.1,1.3,.06),(.36,0,spec.TABLE_Z-.03),(.42,.29,.18))
    # Fixed conveyor section and bracket pedestal; fixed reflector at its opposite side.
    # Conveyor travel is along Y; the beam crosses its width along X.
    box(stage,'/World/frame',(.56,.36,.045),(.620,.09,spec.TABLE_Z+.0225),(.35,.39,.42))
    box(stage,'/World/belt',(.51,.34,.003),(.635,.09,spec.TABLE_Z+.0465),(.055,.06,.065))
    box(stage,'/World/pedestal',(.065,.045,.059),(.33,0,spec.TABLE_Z+.0295),(.28,.31,.34))
    # Foot envelope follows a 30 x 23.5 mm slotted commercial bracket. Its
    # pivot and arcuate-slot guide are represented by the revolute constraint.
    mount=root(stage,'/World/mount',spec.PIVOT,.140,spec.INITIAL_YAW)
    load=PhysxSchema.PhysxForceAPI.Apply(mount.GetPrim())
    load.CreateModeAttr('force');load.CreateWorldFrameEnabledAttr(True)
    load.CreateForceEnabledAttr(True);load.CreateForceAttr(Gf.Vec3f(0.))
    load.CreateTorqueAttr(Gf.Vec3f(0.))
    box(stage,'/World/mount/foot',(.030,.0235,.002),(-.006,0,0),(.5,.52,.55))
    box(stage,'/World/mount/upright',(.002,.019,.0465),(.009,0,.02425),(.5,.52,.55))
    box(stage,'/World/mount/housing',spec.SENSOR_SIZE,spec.SENSOR_CENTER_LOCAL,(.08,.22,.55))
    box(stage,'/World/mount/lens',(.001,.012,.020),spec.LENS_LOCAL,(.10,.012,.018),collision=False)
    led=material(stage,'/World/led_material',(.025,.02,.0),emission=True)
    box(stage,'/World/mount/indicator',(.005,.005,.001),(.031,0,.050),(0,0,0),collision=False,mat=led)
    j=joint(stage,'mount',UsdPhysics.RevoluteJoint,None,'/World/mount',spec.PIVOT,(0,0,0))
    j.CreateLowerLimitAttr(-30.);j.CreateUpperLimitAttr(30.)
    d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'angular');d.CreateTypeAttr('force')
    d.CreateStiffnessAttr(0.)
    d.CreateDampingAttr(spec.CLAMP_BRAKE_DAMPING_NMS_RAD*math.pi/180)
    d.CreateTargetVelocityAttr(0.);d.CreateMaxForceAttr(0.)
    # Captive washer axial micromotion is omitted. Rotational clamp friction
    # is represented by the native torque-limited drive, with no angle target.
    washer_z=spec.CLAMP[2]+.0015
    cylinder(stage,'/World/washer',.005,.001,(*spec.CLAMP[:2],washer_z),(.55,.58,.60))
    UsdShade.MaterialBindingAPI(stage.GetPrimAtPath('/World/washer/shape')).Bind(axial,materialPurpose='physics')
    cylinder(stage,'/World/bolt',.008/math.sqrt(3),.0035,(*spec.CLAMP[:2],spec.CLAMP[2]+.00395),(.45,.48,.50),.004,sides=6)
    box(stage,'/World/bolt/witness',(.003,.0008,.0002),(.002,0,.00185),(.015,.015,.015),collision=False)
    j=joint(stage,'bolt',UsdPhysics.RevoluteJoint,None,'/World/bolt',(*spec.CLAMP[:2],spec.CLAMP[2]+.00395),(0,0,0))
    j.CreateLowerLimitAttr(-170.);j.CreateUpperLimitAttr(20.)
    d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'angular');d.CreateTypeAttr('force');d.CreateStiffnessAttr(0.)
    d.CreateDampingAttr(1.);d.CreateTargetVelocityAttr(0.);d.CreateMaxForceAttr(.002)
    box(stage,'/World/reflector_post',(.012,.018,.059),(.904,0,spec.TABLE_Z+.0295),(.35,.39,.42))
    box(stage,'/World/reflector',spec.REFLECTOR_SIZE,spec.REFLECTOR,(.50,.045,.02))
    # A standard loose socket driver. Its socket mouth is a six-wall hexagon.
    driver=root(stage,'/World/driver',spec.DRIVER_POS,.10,quat=Gf.Quatf(math.sqrt(.5),0,math.sqrt(.5),0))
    # Simple replaceable flat-sided grip. Optical alignment and clamp retention
    # are the task's challenge; a round handle is not required.
    box(stage,'/World/driver/handle',(.020,.020,.060),(0,0,.110),(.10,.10,.11))
    cylinder(stage,'/World/driver/shaft',.003,.065,(0,0,.0475),(.55,.58,.60))
    for i in range(6):
        socket_wall(stage,f'/World/driver/socket_{i}',math.pi/3+i*math.pi/3)
    box(stage,'/World/card_pad',(.075,.075,.001),(*spec.CARD_START[:2],spec.TABLE_Z+.0005),(.12,.30,.10))
    box(stage,'/World/card',spec.CARD_SIZE,spec.CARD_START,(.80,.22,.025),.045)
    return {'mount':'/World/mount','washer':'/World/washer','bolt':'/World/bolt','driver':'/World/driver','card':'/World/card'}
