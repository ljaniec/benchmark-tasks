"""The procedural strap scene inside an actual Isaac Lab-Arena environment.

Policy observations are produced separately from evaluator measurements. Reset
rebuilds the environment, including every breakable bond and camera product.
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


class StrapWorld:
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
                        UsdShade.MaterialBindingAPI.Apply(child).Bind(
                            UsdShade.Material(stage.GetPrimAtPath(self.path('/World/contact'))), materialPurpose='physics')
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
            cfg.sim.physics.solver_type = 1
            cfg.episode_length_s = 130.
            return cfg

        description = IsaacLabArenaEnvironment(name='battery_strap', scene=Scene([assembly]),
                            embodiment=embodiment, env_cfg_callback=configure)
        self.env = ArenaEnvBuilder(description, ArenaEnvBuilderCfg(num_envs=1,
                              solve_relations=False, device='cpu', disable_fabric=True)).make_registered()
        self.base = self.env.unwrapped
        self.stage = self.base.sim.stage
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
        from pxr import UsdPhysics, UsdShade, PhysxSchema
        for prim in self.stage.Traverse():
            if any(str(prim.GetPath()).startswith('/World/envs/env_0/'+s) for s in ['LeftArm','RightArm']):
                if prim.GetName() in ['tip_left','tip_right'] and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    self.pads.append(str(prim.GetPath()))
        if len(self.pads) != 4:
            raise RuntimeError('Expected four physical finger pads')
        for _ in range(4):
            self.base.sim.render()
        for name in CAMERAS.values():
            self.base.scene[name].update(0., force_recompute=True)
        self.stage.Export(str(self.output/'reset.usda'))
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
        bound = UsdGeom.BBoxCache(0, ['default','render']).ComputeWorldBound(self.stage.GetPrimAtPath(path)).ComputeAlignedRange()
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

    def measure(self):
        from pxr import Gf
        from yam_arena import station
        for side in ['left','right']:
            if np.linalg.norm(array(self.base.scene[side+'_robot'].data.root_pos_w)[0]-station.BASES[side])>.001:
                raise RuntimeError('Fixed robot base moved by more than 1 mm')
        center = lambda path: np.array(self.matrix(path).ExtractTranslation())
        battery = self.path('/World/battery')
        module = self.matrix(self.path('/World/module'))
        bp = center(battery)
        tilt = math.degrees(math.acos(np.clip(float(module.TransformDir(Gf.Vec3d(0,0,1))[2]),-1,1)))
        gaps = [float(np.linalg.norm(center(self.path(link))-np.array(module.Transform(Gf.Vec3d(*anchor)))))
                for _,link,anchor in self.objects['bonds']]
        opened = bool(gaps) and min(gaps)>=.006
        blo,bhi = self.bounds(battery)
        mlo,mhi = self.bounds(self.path('/World/module'))
        clear = bool(np.any(bhi<mlo-.005) or np.any(blo>mhi+.005))
        onmat = bool(blo[0]>=.21 and bhi[0]<=.33 and blo[1]>=-.26 and bhi[1]<=-.14 and abs(blo[2]-(spec.TABLE_Z+.002))<=.002)
        hand_clear = all(np.linalg.norm(np.maximum(0,np.maximum(blo-hi,lo-bhi)))>=.005 for lo,hi in map(self.bounds,self.pads))
        all_bounds = [self.bounds(self.path(path)) for path in ['/World/module','/World/battery',*self.objects['links']]]
        inside = all(np.all(lo>=spec.WORKSPACE[0]) and np.all(hi<=spec.WORKSPACE[1]) for lo,hi in all_bounds)
        join_gaps = [float(np.linalg.norm(np.array(self.matrix(self.path(b0)).Transform(Gf.Vec3d(*p0)))-np.array(self.matrix(self.path(b1)).Transform(Gf.Vec3d(*p1)))))
                     for b0,b1,p0,p1 in self.objects['hinges']]
        self.history.append(bp.copy())
        self.history = self.history[-spec.DWELL_STEPS:]
        motion = float(max(np.linalg.norm(v-self.history[0]) for v in self.history)*1000)
        evidence = Evidence(opened,clear,onmat,hand_clear,motion,tilt,bool(inside),max(join_gaps,default=0.)<.004,bool(clear and not opened))
        return evidence, {'battery_xyz':bp.tolist(),'module_tilt_deg':tilt,'overlap_gaps_m':gaps,
                          'module_xyz':list(module.ExtractTranslation()),
                          'strap_xyz':[center(self.path(link)).tolist() for link in self.objects['links']],
                          'strap_tail_xyz':[center(self.path(link)).tolist() for link in self.objects['links'][-4:]],
                          'max_strap_joint_gap_m':max(join_gaps,default=0.),
                          'robot_roots':{s:array(self.base.scene[s+'_robot'].data.root_pos_w).tolist()
                                         for s in ['left','right']},
                          'pad_centers_usd':[center(p).tolist() for p in self.pads]}

    def step(self, action):
        action = np.asarray(action, dtype=np.float32)
        if action.shape != (14,) or not np.isfinite(action).all():
            raise ValueError('Expected 14 finite joint/gripper targets')
        self.env.step(self.torch.tensor(action[None,:], device=self.base.device))
        evidence, diagnostics = self.measure()
        score = self.scorer.update(evidence)
        row = {'step':self.steps,'score':score,'dwell':self.scorer.dwell,'invalid':self.scorer.invalid,
               'evidence':asdict(evidence),'joint_targets':action.tolist(),'measured_joints':self.joints().tolist(),**diagnostics}
        self.trace.write(json.dumps(row)+'\n')
        self.trace.flush()
        self.steps += 1
        self.save_frames(f'{self.steps:05d}')
        self.last = row
        if self.steps % 50 == 0:
            print('STRAP_STEP',self.steps,'score',score,'invalid',self.scorer.invalid,flush=True)
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
