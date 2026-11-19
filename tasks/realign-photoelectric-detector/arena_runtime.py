"""The procedural photoelectric scene inside an actual Isaac Lab-Arena environment.

Policy observations are produced separately from evaluator measurements. Reset
rebuilds the environment, including every clamp and optical proof state and camera product.
"""
from pathlib import Path
from dataclasses import asdict
import json
import hashlib
import math
import time
import numpy as np
import spec
from scorer import Evidence, Scorer

APP = None
CAMERAS = {'overhead_cam': 'top_cam', 'left_wrist_cam': 'left_cam', 'right_wrist_cam': 'right_cam'}


def array(value):
    value = value.torch if hasattr(value, 'torch') else value
    return value.detach().cpu().numpy() if hasattr(value, 'detach') else np.asarray(value)


class PhotoWorld:
    def __init__(self, output, seed=0):
        global APP
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=False)
        self.env = None
        self.steps = 0
        self.scorer = Scorer()
        self.history = []
        self.started = time.monotonic()
        self.trace = (self.output/'trace.jsonl').open('w')
        if APP is None:
            from isaaclab.app import AppLauncher
            APP = AppLauncher(headless=True, enable_cameras=True, device='cpu').app
        from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics, PhysxSchema
        import omni.usd
        import torch
        from isaaclab_arena.assets.object import Object
        from isaaclab_arena.assets.object_type import ObjectType
        from isaaclab_arena.utils.pose import Pose
        from isaaclab_arena.scene.scene import Scene
        from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
        from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
        from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
        from yam_arena import station, constants, ik
        from yam_arena.embodiment import BimanualYamEmbodiment
        from yam_arena.scene import env_cfg_callback
        import scene
        import demo
        self.torch, self.constants = torch, constants
        omni.usd.get_context().new_stage()
        asset_stage = Usd.Stage.CreateInMemory()
        root = UsdGeom.Xform.Define(asset_stage, '/World').GetPrim()
        asset_stage.SetDefaultPrim(root)
        objects = scene.build(asset_stage)
        # Application/renderer warm-up must not consume any task dynamics.
        # Restore every authored rigid pose and zero velocity before step zero.
        authored = {}
        for prim in asset_stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
                q = matrix.ExtractRotationQuat()
                authored[str(prim.GetPath())] = [*matrix.ExtractTranslation(), *q.GetImaginary(), q.GetReal()]
                PhysxSchema.PhysxRigidBodyAPI(prim).CreateDisableGravityAttr(True)
        asset_stage.RemovePrim('/World/physics')
        light = UsdLux.DomeLight.Define(asset_stage, '/World/light')
        light.CreateIntensityAttr(800.)
        sun = UsdLux.DistantLight.Define(asset_stage, '/World/sun')
        sun.CreateIntensityAttr(1600.)
        sun.AddRotateXYZOp().Set(Gf.Vec3f(25, -30, 20))
        usd = self.output/'workbench.usda'
        asset_stage.Export(str(usd))
        self.prefix = '/World/envs/env_0/workbench'
        self.objects = objects
        assembly = Object(name='workbench', prim_path='{ENV_REGEX_NS}/workbench',
                          usd_path=str(usd), object_type=ObjectType.BASE,
                          initial_pose=Pose(position_xyz=(0., 0., 0.), rotation_xyzw=(0., 0., 0., 1.)))
        rig = station.rig()
        embodiment = BimanualYamEmbodiment(enable_cameras=True, rig=rig,
                        gravity_compensation=True, cam_height=360, cam_width=640,
                        robot_contact_offset=.0002, contact_sensors=False)
        embodiment.camera_config.set_use_tiled_camera(False)
        initial, _ = demo.plan(ik, constants, station)
        self.initial_command = initial[0][1]
        for side in ['left', 'right']:
            arm = getattr(embodiment.scene_config, side+'_robot')
            arm.init_state.joint_pos = dict(zip(constants.ALL_JOINTS, self.initial_command[side]))
            arm.init_state.joint_vel = {'.*': 0.}
            arm.actuators['gripper'].joint_effort_limit = spec.FINGER_FORCE_LIMIT
            arm.spawn.articulation_props.solver_position_iteration_count = 32
            arm.spawn.articulation_props.solver_velocity_iteration_count = 8
            original_spawn = arm.spawn.func

            def spawn_arm(prim_path, cfg, *args, _spawn=original_spawn, **kwargs):
                from pxr import UsdPhysics, UsdShade, PhysxSchema
                prim = _spawn(prim_path, cfg, *args, **kwargs)
                stage = prim.GetStage()
                matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
                # Keep the articulation container at identity. With a translated
                # non-rigid container, this Lab/PhysX pair adds its translation
                # again when it synchronizes the fixed root back to USD.
                prim.GetAttribute('xformOp:translate').Set(Gf.Vec3d(0,0,0))
                for child in Usd.PrimRange(prim):
                    if child.GetName()=='root_joint' and child.IsA(UsdPhysics.FixedJoint):
                        joint = UsdPhysics.FixedJoint(child)
                        # The asset's non-rigid body0 parent makes Lab's anchor
                        # update add the base offset repeatedly. Anchor to the
                        # world explicitly before physics is initialized.
                        joint.GetBody0Rel().SetTargets([])
                        joint.GetLocalPos0Attr().Set(Gf.Vec3f(*matrix.ExtractTranslation()))
                        joint.GetLocalRot0Attr().Set(Gf.Quatf(matrix.ExtractRotationQuat()))
                        base = stage.GetPrimAtPath(joint.GetBody1Rel().GetTargets()[0])
                        UsdGeom.Xformable(base).MakeMatrixXform().Set(matrix)
                    if child.HasAPI(UsdPhysics.CollisionAPI):
                        contact = PhysxSchema.PhysxCollisionAPI.Apply(child)
                        contact.CreateContactOffsetAttr(.0002)
                        contact.CreateRestOffsetAttr(0.)
                        pad = any(x in str(child.GetPath()).split('/') for x in ('tip_left','tip_right'))
                        if pad:
                            contact.CreateTorsionalPatchRadiusAttr(spec.PAD_TORSIONAL_RADIUS)
                            contact.CreateMinTorsionalPatchRadiusAttr(spec.PAD_TORSIONAL_RADIUS)
                        UsdShade.MaterialBindingAPI.Apply(child).Bind(
                            UsdShade.Material(stage.GetPrimAtPath(self.path('/World/pad_contact' if pad else '/World/contact'))), materialPurpose='physics')
                # Imported links are nested. Make every rigid link's transform
                # independent while preserving its world pose and joint frames;
                # this avoids parent transforms being applied twice by RTX.
                bodies = [(child, UsdGeom.Xformable(child).ComputeLocalToWorldTransform(0))
                          for child in Usd.PrimRange(prim) if child.HasAPI(UsdPhysics.RigidBodyAPI)]
                for child, pose in bodies:
                    xf = UsdGeom.Xformable(child)
                    xf.MakeMatrixXform().Set(pose)
                    xf.SetResetXformStack(True)
                return prim

            arm.spawn.func = spawn_arm
        standard = env_cfg_callback(spec.CONTROL_HZ)

        def configure(cfg):
            cfg = standard(cfg)
            cfg.seed = seed
            cfg.sim.dt = 1/spec.PHYSICS_HZ
            cfg.decimation = spec.PHYSICS_HZ//spec.CONTROL_HZ
            cfg.sim.render_interval = cfg.decimation
            cfg.sim.use_fabric = False
            cfg.sim.enable_scene_query_support = True
            cfg.sim.physics.enable_scene_query_support = True
            cfg.sim.physics.enable_ccd = True
            cfg.sim.physics.solver_type = 1
            # TGS supports finite-patch torsional finger friction. Zero velocity
            # iterations avoid the documented TGS joint-drive interaction.
            cfg.sim.physics.min_velocity_iteration_count = 0
            cfg.sim.physics.max_velocity_iteration_count = 0
            # Contact dimensions are millimetres; the default 25 mm anchor
            # correlation radius spans the entire clamp and finger pads.
            cfg.sim.physics.friction_correlation_distance = .0005
            cfg.sim.physics.friction_offset_threshold = .0001
            cfg.episode_length_s = 130.
            return cfg

        description = IsaacLabArenaEnvironment(name='photoelectric', scene=Scene([assembly]),
                            embodiment=embodiment, env_cfg_callback=configure)
        self.env = ArenaEnvBuilder(description, ArenaEnvBuilderCfg(num_envs=1,
                              solve_relations=False, device='cpu', disable_fabric=True)).make_registered()
        self.base = self.env.unwrapped
        self.stage = self.base.sim.stage
        # Lab disables reporting globally unless a ContactSensor is created.
        # These evaluator-only native views need the same reporting switch.
        self.base.sim.set_setting('/physics/disableContactProcessing',False)
        self.env.reset(seed=seed)
        import omni.physics.tensors as tensors
        from isaaclab.sim.utils.stage import get_current_stage_id
        self.reset_sim_view = tensors.create_simulation_view('numpy', stage_id=get_current_stage_id())
        self.reset_sim_view.set_subspace_roots('/')
        self.prop_views = {}
        reset_state = {}
        for original, pose in authored.items():
            path = self.path(original)
            view = self.reset_sim_view.create_rigid_body_view(path)
            ids = np.array([0], dtype=np.uint32)
            view.set_transforms(np.asarray([pose], dtype=np.float32), ids)
            view.set_velocities(np.zeros((1,6), dtype=np.float32), ids)
            view.set_disable_gravities(np.zeros((1,1), dtype=np.uint8), ids)
            PhysxSchema.PhysxRigidBodyAPI(self.stage.GetPrimAtPath(path)).GetDisableGravityAttr().Set(False)
            self.prop_views[original] = view
            reset_state[original] = {'pose':view.get_transforms().tolist(), 'velocity':view.get_velocities().tolist()}
        (self.output/'reset-state.json').write_text(json.dumps(reset_state, indent=2)+'\n')
        # Lab's current backend initializes fixed-root transforms separately
        # from joint state; the pinned YAM reset event resets only joints.
        for side in ['left','right']:
            arm = self.base.scene[side+'_robot']
            arm.write_root_pose_to_sim_index(root_pose=arm.data.default_root_pose.torch.clone())
            arm.write_root_velocity_to_sim_index(root_velocity=arm.data.default_root_vel.torch.clone())
        self.base.scene.write_data_to_sim()
        self.base.sim.forward()
        robot_reset = {}
        for side in ['left','right']:
            arm = self.base.scene[side+'_robot']
            robot_reset[side] = {'root_pose':array(arm.data.root_pose_w).tolist(),
                                 'root_velocity':array(arm.data.root_vel_w).tolist(),
                                 'joint_position':array(arm.data.joint_pos).tolist(),
                                 'joint_velocity':array(arm.data.joint_vel).tolist(),
                                 'joint_names':list(arm.joint_names)}
        (self.output/'reset-robot-state.json').write_text(json.dumps(robot_reset,indent=2)+'\n')
        self.pads = []
        self.robot_colliders = []
        self.hands = []
        from pxr import UsdPhysics, UsdShade, PhysxSchema
        for prim in Usd.PrimRange.Stage(self.stage, Usd.TraverseInstanceProxies()):
            if any(str(prim.GetPath()).startswith('/World/envs/env_0/'+s) for s in ['LeftArm','RightArm']):
                if prim.HasAPI(UsdPhysics.CollisionAPI):self.robot_colliders.append(str(prim.GetPath()))
                if prim.GetName() in ['tip_left','tip_right'] and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    self.pads.append(str(prim.GetPath()))
                if prim.GetName() in ['gripper','tip_left','tip_right'] and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    self.hands.append(str(prim.GetPath()))
        if len(self.pads) != 4:
            raise RuntimeError('Expected four physical finger pads')
        if len(self.hands)!=6:raise RuntimeError('Expected two gripper bodies and four fingers')
        if not self.robot_colliders:raise RuntimeError('Robot collision geometry missing')
        self.fixed_positions={name:np.asarray(self.matrix(self.path('/World/'+name)).ExtractTranslation())
                              for name in ['frame','reflector']}
        for _ in range(4):
            self.base.sim.render()
        for name in CAMERAS.values():
            self.base.scene[name].update(0., force_recompute=True)
        self.stage.Export(str(self.output/'reset.usda'))
        self.update_clamp()
        from omni.physx import get_physx_scene_query_interface
        probe=get_physx_scene_query_interface().raycast_closest(
            Gf.Vec3f(spec.CLAMP[0],spec.CLAMP[1],spec.CLAMP[2]+.10),Gf.Vec3f(0,0,-1),.15)
        if not probe['hit'] or str(probe['rigidBody'])!=self.path('/World/bolt'):
            raise RuntimeError('Native ray query failed the known-bolt self-test')
        (self.output/'query-self-test.json').write_text(json.dumps({'known_bolt_hit':True,'distance_m':probe['distance']})+'\n')
        self.save_frames('00000')
        (self.output/'reset-cameras.json').write_text(json.dumps(
            {name:{'shape':list(im.shape),'sha256':hashlib.sha256(im.tobytes()).hexdigest()}
             for name,im in self.images().items()},indent=2)+'\n')

    def path(self, original):
        return original.replace('/World', self.prefix, 1)

    def matrix(self, path):
        from pxr import UsdGeom
        return UsdGeom.Xformable(self.stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(0)

    def bounds(self, path):
        from pxr import UsdGeom
        bound = UsdGeom.BBoxCache(0, ['default','render','proxy','guide']).ComputeWorldBound(self.stage.GetPrimAtPath(path)).ComputeAlignedRange()
        return np.array(bound.GetMin()), np.array(bound.GetMax())

    def images(self, native=False):
        from PIL import Image
        frames = {name:array(self.base.scene[key].data.output['rgb'])[0,:,:,:3].astype(np.uint8).copy()
                  for name,key in CAMERAS.items()}
        if not native:
            frames = {name:np.asarray(Image.fromarray(im).resize((320,176), Image.Resampling.BILINEAR)) for name,im in frames.items()}
        return frames

    def joints(self):
        result = []
        for side in ['left','right']:
            arm = self.base.scene[side+'_robot']
            ids, _ = arm.find_joints(self.constants.ARM_JOINTS, preserve_order=True)
            fingers, _ = arm.find_joints(self.constants.FINGER_JOINTS, preserve_order=True)
            q = array(arm.data.joint_pos)[0]
            result.extend([*q[ids], float(np.clip(q[fingers].mean()/self.constants.FINGER_OPEN,0,1))])
        return np.asarray(result, dtype=np.float32)

    def save_frames(self, label):
        from PIL import Image
        for name,im in self.images(native=True).items():
            folder = self.output/name
            folder.mkdir(exist_ok=True)
            Image.fromarray(im).save(folder/(label+'.png'))
            if label == '00000' and name == 'overhead_cam':
                Image.fromarray(im).save(self.output/'setup.png')

    def update_clamp(self):
        from pxr import Gf, UsdPhysics, PhysxSchema
        matrix=self.matrix(self.path('/World/bolt'))
        rotation=np.asarray(matrix.ExtractRotationMatrix())
        angle=math.atan2(rotation[0,1],rotation[0,0])
        # Clockwise viewed from above advances a right-hand M5 thread.
        advance=-angle*spec.THREAD_PITCH/(2*math.pi)
        z=float(self.matrix(self.path('/World/washer')).ExtractTranslation()[2])
        # Reduced thread compression and Coulomb clamp-torque model.
        compression=max(0.,advance-spec.CLAMP_GAP)
        force=min(spec.MAX_CLAMP_FORCE,spec.CLAMP_STIFFNESS*compression)
        torque=.002+spec.NUT_FACTOR*spec.BOLT_DIAMETER*force
        bolt=UsdPhysics.DriveAPI(self.stage.GetPrimAtPath(self.path('/World/joints/bolt')),'angular')
        bolt.GetMaxForceAttr().Set(torque)
        cap=spec.FRICTION*force*spec.CLAMP_FRICTION_RADIUS
        mount=UsdPhysics.DriveAPI(self.stage.GetPrimAtPath(self.path('/World/joints/mount')),'angular')
        mount.GetMaxForceAttr().Set(cap)
        mount_rotation=np.asarray(self.matrix(self.path('/World/mount')).ExtractRotationMatrix())
        yaw=math.atan2(mount_rotation[0,1],mount_rotation[0,0])
        omega=float(self.prop_views['/World/mount'].get_velocities()[0,5])
        cable=float(np.clip(-spec.CABLE_STIFFNESS_NM_RAD*(yaw-spec.INITIAL_YAW)-spec.CABLE_DAMPING_NMS_RAD*omega,-.03,.03))
        load=PhysxSchema.PhysxForceAPI(self.stage.GetPrimAtPath(self.path('/World/mount')))
        load.GetTorqueAttr().Set(Gf.Vec3f(0.,0.,cable))
        self.clamp_data={'bolt_angle_deg':math.degrees(angle),'advance_m':advance,'washer_z':z,
                         'clamp_force_estimate_n':force,'thread_resistance_nm':torque,
                         'friction_torque_cap_nm':cap,'cable_torque_nm':cable}

    def measure(self):
        from pxr import Gf, UsdShade
        import optics
        matrix=self.matrix(self.path('/World/mount'))
        rot=np.asarray(matrix.ExtractRotationMatrix()).T
        origin=np.asarray(matrix.Transform(Gf.Vec3d(*spec.LENS_LOCAL)))
        yaw=math.degrees(math.atan2(rot[1,0],rot[0,0]))
        clo,chi=self.bounds(self.path('/World/card'))
        dlo,dhi=self.bounds(self.path('/World/driver'))
        from omni.physx import get_physx_scene_query_interface
        query=get_physx_scene_query_interface()
        def occlusion(origin,direction,distance):
            hit=query.raycast_closest(Gf.Vec3f(*origin),Gf.Vec3f(*direction),float(distance-.003))
            if not hit['hit']:return None
            body=str(hit.get('rigidBody',hit.get('collision','unknown')))
            if '/card' in body:return 'card'
            if '/driver' in body:return 'driver'
            if 'Arm/' in body:return 'robot'
            return 'fixture'
        optical=optics.response(origin,rot,spec.REFLECTOR,[],occlusion=occlusion)
        # The physical output indicator is visible to every camera; no score lamp.
        led=UsdShade.Shader(self.stage.GetPrimAtPath(self.path('/World/led_material/surface')))
        color=Gf.Vec3f(1.,.7,.01) if optical['on'] else Gf.Vec3f(.025,.02,0.)
        led.GetInput('diffuseColor').Set(color);led.GetInput('emissiveColor').Set(color)
        mlo,mhi=self.bounds(self.path('/World/mount'))
        distance=lambda lo,hi,a,b:float(np.linalg.norm(np.maximum(0,np.maximum(lo-b,a-hi))))
        # Query actual robot shapes against each mount component's oriented
        # 5 mm margin. A single AABB around an angled gripper fills empty space
        # between its palm and fingers and can falsely reject the test pose.
        import carb
        near_robot=[]
        def overlap(hit):
            body=str(hit.rigid_body)
            if 'Arm/' in body:near_robot.append(body)
            return True
        orientation=matrix.ExtractRotationQuat()
        q=carb.Float4(*map(float,orientation.GetImaginary()),float(orientation.GetReal()))
        for local,size in [((-.006,0,0),(.030,.0235,.002)),
                           ((.009,0,.02425),(.002,.019,.0465)),
                           (spec.SENSOR_CENTER_LOCAL,spec.SENSOR_SIZE)]:
            center=matrix.Transform(Gf.Vec3d(*local))
            half=np.asarray(size)/2+.005
            query.overlap_box(carb.Float3(*map(float,half)),carb.Float3(*map(float,center)),q,overlap,False)
        hands_clear=not near_robot
        near_card=[]
        def overlap_card(hit):
            body=str(hit.rigid_body)
            if 'Arm/' in body:near_card.append(body)
            return True
        card_matrix=self.matrix(self.path('/World/card'))
        card_q=card_matrix.ExtractRotationQuat()
        query.overlap_box(carb.Float3(*map(float,np.array(spec.CARD_SIZE)/2+.005)),
                          carb.Float3(*map(float,card_matrix.ExtractTranslation())),
                          carb.Float4(*map(float,card_q.GetImaginary()),float(card_q.GetReal())),
                          overlap_card,False)
        tool_clear=distance(mlo,mhi,dlo,dhi)>=.005
        all_bounds=[self.bounds(self.path(path)) for path in self.objects.values()]
        inside=all(np.all(lo>=spec.WORKSPACE[0]) and np.all(hi<=spec.WORKSPACE[1]) for lo,hi in all_bounds)
        position=np.asarray(matrix.ExtractTranslation());intact=np.linalg.norm(position-np.asarray(spec.PIVOT))<.002
        parked=bool(np.all(clo[:2]>=np.array(spec.CARD_START[:2])-.0375) and np.all(chi[:2]<=np.array(spec.CARD_START[:2])+.0375) and abs(clo[2]-(spec.TABLE_Z+.001))<.003)
        # Quarter-turn-loose reset: the visible bolt index must pass the seat
        # angle. Retention is judged separately by the hands-off optical test;
        # internal spring-load estimates are diagnostic only.
        closed=self.clamp_data['bolt_angle_deg']<=-spec.CLAMP_MIN_TURN_DEG
        frame=self.matrix(self.path('/World/frame'))
        up=np.asarray(frame.ExtractRotationMatrix())[2,:]
        tilt=math.degrees(math.acos(float(np.clip(up[2]/np.linalg.norm(up),-1,1))))
        shortcut=any(np.linalg.norm(np.asarray(self.matrix(self.path('/World/'+name)).ExtractTranslation())-pos)>.002
                     for name,pos in self.fixed_positions.items())
        evidence=Evidence(optical['on'],optical['geometric_hits']>=7 and optical['blocked_by']==['card'],closed,hands_clear,parked,tool_clear,yaw,tilt,bool(inside),bool(intact),bool(shortcut),not near_card)
        diagnostics={'sensor_xyz':position.tolist(),'sensor_yaw_deg':yaw,'card_xyz':list(self.matrix(self.path('/World/card')).ExtractTranslation()),'driver_xyz':list(self.matrix(self.path('/World/driver')).ExtractTranslation()),'driver_matrix':np.asarray(self.matrix(self.path('/World/driver'))).tolist(),'optical':optical,'clamp':self.clamp_data,'robot_inside_5mm_margin':sorted(set(near_robot)),'pad_centers_usd':[list(self.matrix(p).ExtractTranslation()) for p in self.pads]}
        return evidence,diagnostics

    def step(self, action):
        action = np.asarray(action, dtype=np.float32)
        if action.shape != (14,) or not np.isfinite(action).all():
            raise ValueError('Expected 14 finite joint/gripper targets')
        self.update_clamp()
        self.env.step(self.torch.tensor(action[None,:], device=self.base.device))
        self.update_clamp()
        evidence, diagnostics = self.measure()
        score = self.scorer.update(evidence)
        row = {'step':self.steps,'score':score,'dwell':self.scorer.dwell,'invalid':self.scorer.invalid,
               'evidence':asdict(evidence),'joint_targets':action.tolist(),'measured_joints':self.joints().tolist(),**diagnostics}
        self.trace.write(json.dumps(row)+'\n')
        self.trace.flush()
        self.steps += 1
        # Render the updated physical LED without advancing physics.
        self.base.sim.render()
        for key in CAMERAS.values():self.base.scene[key].update(0., force_recompute=True)
        self.save_frames(f'{self.steps:05d}')
        self.last = row
        if self.steps % 50 == 0:
            print('PHOTO_STEP',self.steps,'score',score,'invalid',self.scorer.invalid,flush=True)
        return row

    def close(self):
        result = {'steps':self.steps,'score':self.scorer.score,'invalid':self.scorer.invalid,
                  'dwell':self.scorer.dwell,'wall_seconds':time.monotonic()-self.started,
                  'control_hz':spec.CONTROL_HZ,'physics_hz':spec.PHYSICS_HZ}
        (self.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        self.trace.close()
        if self.env is not None:
            self.stage.Export(str(self.output/'terminal.usda'))
            self.env.close()
            self.env = None
