"""The cup-and-ball toy, its floor stand and two YAM arms inside an Isaac Lab-Arena environment.

Policy observations are produced separately from evaluator measurements. Reset
rebuilds the environment, including every body, the string joint, scorer counters and camera products.
"""
from pathlib import Path
from dataclasses import asdict
from functools import lru_cache
import json
import hashlib
import math
import time
import numpy as np
import spec
from scipy.spatial.transform import Rotation
from scorer import Evidence, Scorer

APP = None
CAMERAS = {'overhead_cam': 'top_cam', 'left_wrist_cam': 'left_cam', 'right_wrist_cam': 'right_cam'}
FIXTURES = ['floor', 'pedestal_left', 'pedestal_right', 'stand_base', 'stand_post', 'stand_bracket', 'stand_sleeve']


def array(value):
    value = value.torch if hasattr(value, 'torch') else value
    return value.detach().cpu().numpy() if hasattr(value, 'detach') else np.asarray(value)


class CupBallWorld:
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
        self.prop_views = {}
        self.ball_contacts = None
        self.fixture_subscription = None
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
        from yam_arena import constants, ik
        import task_station as station
        from yam_arena.embodiment import BimanualYamEmbodiment
        from yam_arena.scene import env_cfg_callback
        import scene
        import demo
        self.torch, self.constants, self.scene_module, self.Gf = torch, constants, scene, Gf
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
            arm.spawn.articulation_props.solver_velocity_iteration_count = 0
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
                fingers={child.GetName():child for child in Usd.PrimRange(prim)
                         if child.GetName() in ('joint7','joint8') and child.IsA(UsdPhysics.PrismaticJoint)}
                if len(fingers)!=2:raise RuntimeError('Missing parallel-gripper joints')
                for finger in fingers.values():
                    UsdPhysics.PrismaticJoint(finger).GetLowerLimitAttr().Set(-.0375)
                mimic=PhysxSchema.PhysxMimicJointAPI.Apply(fingers['joint8'],'rotX')
                mimic.CreateReferenceJointRel().SetTargets([fingers['joint7'].GetPath()])
                mimic.CreateReferenceJointAxisAttr('rotX');mimic.CreateGearingAttr(-1.)
                mimic.CreateOffsetAttr(0.);mimic.CreateNaturalFrequencyAttr(0.)
                return prim

            arm.spawn.func = spawn_arm
        standard = env_cfg_callback(spec.CONTROL_HZ)

        def configure(cfg):
            cfg = standard(cfg)
            cfg.seed = seed
            cfg.sim.dt = 1/spec.PHYSICS_HZ
            cfg.decimation = spec.PHYSICS_HZ//spec.CONTROL_HZ
            cfg.sim.render_interval = cfg.decimation
            # With USD write-back instead of Fabric, this Lab/PhysX pair zeroes every free body's
            # velocity at each control step (a dropped ball falls 12 mm per step at constant speed).
            cfg.sim.use_fabric = True
            cfg.sim.enable_scene_query_support = True
            cfg.sim.physics.enable_scene_query_support = True
            cfg.sim.physics.enable_ccd = True
            cfg.sim.physics.solver_type = 1
            cfg.sim.physics.min_velocity_iteration_count = 0
            cfg.sim.physics.max_velocity_iteration_count = 0
            # Contact dimensions are millimetres; the default 25 mm anchor
            # correlation radius spans the entire handle and finger pads.
            cfg.sim.physics.friction_correlation_distance = .0005
            cfg.sim.physics.friction_offset_threshold = .0001
            cfg.episode_length_s = 130.
            return cfg

        description = IsaacLabArenaEnvironment(name='cup_and_ball', scene=Scene([assembly]),
                            embodiment=embodiment, env_cfg_callback=configure)
        self.env = ArenaEnvBuilder(description, ArenaEnvBuilderCfg(num_envs=1,
                              solve_relations=False, device='cpu', disable_fabric=False)).make_registered()
        self.base = self.env.unwrapped
        self.stage = self.base.sim.stage
        # Lab disables reporting globally unless a ContactSensor is created.
        # These evaluator-only native views need the same reporting switch.
        self.base.sim.set_setting('/physics/disableContactProcessing',False)
        self.string_ops = []
        for k in range(scene.STRING_SEGMENTS):
            ops = UsdGeom.Xformable(self.stage.GetPrimAtPath(self.path(f'/World/string/seg_{k:02d}'))).GetOrderedXformOps()
            self.string_ops.append(ops)
        # Lab renders once at the end of every control step; re-shape the drawn string just before.
        original_render = self.base.sim.render

        def render(*args, **kwargs):
            self.update_string()
            return original_render(*args, **kwargs)
        self.base.sim.render = render
        # Lab updates the scene after every physics substep; poll ball contacts there so brief
        # touches between control steps still count.
        original_update = self.base.scene.update

        def update(*args, **kwargs):
            self.poll_contacts()
            return original_update(*args, **kwargs)
        self.base.scene.update = update
        self.env.reset(seed=seed)
        import omni.physics.tensors as tensors
        from isaaclab.sim.utils.stage import get_current_stage_id
        self.reset_sim_view = tensors.create_simulation_view('numpy', stage_id=get_current_stage_id())
        self.reset_sim_view.set_subspace_roots('/')
        views = {}
        reset_state = {}
        for original, pose in authored.items():
            path = self.path(original)
            view = self.reset_sim_view.create_rigid_body_view(path)
            ids = np.array([0], dtype=np.uint32)
            view.set_transforms(np.asarray([pose], dtype=np.float32), ids)
            view.set_velocities(np.zeros((1,6), dtype=np.float32), ids)
            view.set_disable_gravities(np.zeros((1,1), dtype=np.uint8), ids)
            PhysxSchema.PhysxRigidBodyAPI(self.stage.GetPrimAtPath(path)).GetDisableGravityAttr().Set(False)
            views[original] = view
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
        gripper_contract={}
        for side in ['left','right']:
            arm=self.base.scene[side+'_robot']
            ids,_=arm.find_joints(constants.FINGER_JOINTS,preserve_order=True)
            force=arm.root_view.get_dof_max_forces().numpy()[0,ids]
            limits=arm.root_view.get_dof_limits().numpy()[0,ids,:]
            if not np.allclose(force,40.) or not np.allclose(limits,[[-.0375,0.],[-.0375,0.]],atol=1e-7):
                raise RuntimeError('Native gripper force or aperture contract differs from the specification')
            gripper_contract[side]={'force_caps_n':force.tolist(),'position_limits_m':limits.tolist()}
        (self.output/'reset-gripper-physics.json').write_text(json.dumps(gripper_contract,indent=2)+'\n')
        self.robot_bodies = []
        from pxr import UsdPhysics
        for prim in Usd.PrimRange.Stage(self.stage, Usd.TraverseInstanceProxies()):
            if any(str(prim.GetPath()).startswith('/World/envs/env_0/'+s) for s in ['LeftArm','RightArm']):
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    self.robot_bodies.append(str(prim.GetPath()))
        if len(self.robot_bodies) < 12:
            raise RuntimeError('Robot link bodies missing')
        self.ball_contacts = self.reset_sim_view.create_rigid_contact_view(
            [self.path('/World/ball')], filter_patterns=[self.robot_bodies], max_contact_data_count=256)
        self.fixture_boxes = self.boxes(FIXTURES)
        self.fixed_positions={name:np.asarray(self.matrix(self.path('/World/'+name)).ExtractTranslation()) for name in scene.FIXED}
        self.touch = {'robot': False, 'fixture': False}
        self.prop_views = views
        self.monitor_fixture_contacts()
        self.update_string()
        for _ in range(4):self.base.sim.render()
        for name in CAMERAS.values():
            # Discard the RGB cache captured before the final pose restoration.
            self.base.scene[name].reset()
            self.base.scene[name].update(0.,force_recompute=True)
        self.stage.Export(str(self.output/'reset.usda'))
        self.save_frames('00000')
        (self.output/'reset-cameras.json').write_text(json.dumps({name:{'shape':list(im.shape),'sha256':hashlib.sha256(im.tobytes()).hexdigest()} for name,im in self.images().items()},indent=2)+'\n')

    def path(self, original):
        return original.replace('/World', self.prefix, 1)

    def matrix(self, path):
        from pxr import UsdGeom
        return UsdGeom.Xformable(self.stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(0)

    def boxes(self, names):
        """World matrix (row-vector convention) and inverse of every cube collider under the named fixtures."""
        from pxr import Usd, UsdGeom, UsdPhysics
        out = []
        for name in names:
            for prim in Usd.PrimRange(self.stage.GetPrimAtPath(self.path('/World/'+name))):
                if prim.HasAPI(UsdPhysics.CollisionAPI) and prim.IsA(UsdGeom.Cube):
                    if UsdGeom.Cube(prim).GetSizeAttr().Get() != 1.:
                        raise RuntimeError('Fixture cubes are authored with unit size')
                    m = np.asarray(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0))
                    out.append((name, m, np.linalg.inv(m)))
        return out

    def fixture_gap(self, point):
        """Distance (m) from a point to the nearest fixture collider and that fixture's name."""
        p = np.r_[point, 1.]
        best, which = math.inf, None
        for name, m, inv in self.fixture_boxes:
            local = (p @ inv)[:3]
            closest = np.r_[np.clip(local, -.5, .5), 1.] @ m
            d = float(np.linalg.norm(closest[:3] - point))
            if d < best:
                best, which = d, name
        return best, which

    def poses(self):
        return {name: self.prop_views['/World/'+name].get_transforms()[0].astype(float) for name in ['toy', 'ball']}

    def update_string(self):
        if not self.prop_views:
            return
        Gf = self.Gf
        poses = self.poses()
        tie = poses['toy'][:3] + Rotation.from_quat(poses['toy'][3:]).apply(spec.TIE_LOCAL)
        segments = self.scene_module.segment_poses(self.scene_module.string_points(tie, poses['ball'][:3]))
        for (centre, q, length), ops in zip(segments, self.string_ops):
            ops[0].Set(Gf.Vec3d(*map(float, centre)))
            ops[1].Set(Gf.Quatf(float(q[0]), Gf.Vec3f(*map(float, q[1:]))))
            ops[2].Set(Gf.Vec3f(1., 1., float(length)))

    def poll_contacts(self):
        if not self.prop_views or self.ball_contacts is None:
            return
        forces = np.asarray(self.ball_contacts.get_contact_force_matrix(dt=1/spec.PHYSICS_HZ))
        if forces.size and float(np.abs(forces).max()) > 0.:
            self.touch['robot'] = True

    def monitor_fixture_contacts(self):
        """Latch a physical ball impact against any fixture, including static meshes."""
        from omni.physx import get_physx_simulation_interface
        from pxr import PhysicsSchemaTools

        ball_root = self.path('/World/ball')
        fixture_roots = tuple(self.path('/World/' + name) for name in FIXTURES)
        self.fixture_contact_pairs = set()

        @lru_cache(maxsize=256)
        def collider_path(identifier):
            return str(PhysicsSchemaTools.intToSdfPath(identifier))

        def collect(headers, contacts):
            for header in headers:
                if not header.num_contact_data:
                    continue
                pair = tuple(collider_path(identifier) for identifier in (header.collider0, header.collider1))
                if not spec.ball_hit_fixture(pair, ball_root, fixture_roots):
                    continue
                start = header.contact_data_offset
                stop = start + header.num_contact_data
                if any(math.hypot(*contacts[i].impulse) > 0. for i in range(start, stop)):
                    self.touch['fixture'] = True
                    self.fixture_contact_pairs.add(tuple(sorted(pair)))

        self.fixture_subscription = get_physx_simulation_interface().subscribe_contact_report_events(collect)

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
        poses = self.poses()
        toy, ball = poses['toy'], poses['ball']
        R = Rotation.from_quat(toy[3:])
        axis = R.apply([0., 0., 1.])
        tilt = math.degrees(math.acos(float(np.clip(axis[2], -1., 1.))))
        handle_end = toy[:3] + R.apply([0., 0., -spec.HANDLE_LEN])
        rim = toy[:3] + R.apply([0., 0., spec.RIM_LOCAL_Z])
        local = R.inv().apply(ball[:3] - toy[:3])
        tie = toy[:3] + R.apply(spec.TIE_LOCAL)
        low, high = map(np.asarray, spec.WORKSPACE)
        inside = all(bool(np.all(p >= low) and np.all(p <= high)) for p in (toy[:3], ball[:3]))
        intact = all(np.linalg.norm(np.asarray(self.matrix(self.path('/World/'+name)).ExtractTranslation())-p) < .003
                     for name,p in self.fixed_positions.items())
        gap, nearest = self.fixture_gap(ball[:3])
        touch_robot, touch_fixture = self.touch['robot'], self.touch['fixture']
        fixture_contact_pairs = sorted(self.fixture_contact_pairs)
        self.fixture_contact_pairs.clear()
        self.touch = {'robot': False, 'fixture': False}
        e = Evidence(float(handle_end[2] - spec.sleeve_rim_z()), float(tilt), float(ball[2] - rim[2]),
                     bool(spec.in_cup(local)), bool(touch_robot), bool(touch_fixture),
                     float(toy[2]), bool(inside), bool(intact))
        velocities = {name: self.prop_views['/World/'+name].get_velocities()[0].tolist() for name in ['toy', 'ball']}
        return e, {'poses': {k: v.tolist() for k, v in poses.items()}, 'velocities': velocities,
                   'ball_in_toy_frame_m': local.tolist(), 'string_chord_m': float(np.linalg.norm(ball[:3] - tie)),
                   'ball_fixture_gap_m': gap - spec.BALL_R, 'nearest_fixture': nearest,
                   'fixture_contact_pairs': fixture_contact_pairs}

    def step(self,action):
        action=np.asarray(action,dtype=np.float32)
        if action.shape!=(14,) or not np.isfinite(action).all():raise ValueError('Expected 14 finite joint targets')
        self.env.step(self.torch.tensor(action[None,:],device=self.base.device))
        measured=self.joints()
        if not np.isfinite(measured).all() or np.max(np.abs(measured))>7.:raise RuntimeError('Native robot dynamics diverged')
        evidence,diagnostics=self.measure();score=self.scorer.update(evidence)
        row={'step':self.steps,'score':score,'dwell':self.scorer.dwell,'invalid':self.scorer.invalid,
             'lifted':self.scorer.lifted,'raised':self.scorer.raised,
             'evidence':asdict(evidence),'joint_targets':action.tolist(),'measured_joints':measured.tolist(),**diagnostics}
        row['applied_joint_effort']={s:array(self.base.scene[s+'_robot'].data.applied_torque)[0].tolist() for s in ['left','right']}
        self.trace.write(json.dumps(row)+'\n');self.trace.flush();self.steps+=1
        self.save_frames(f'{self.steps:05d}');self.last=row
        if self.steps%50==0:print('CUPBALL_STEP',self.steps,'score',score,'invalid',self.scorer.invalid,flush=True)
        return row

    def close(self):
        self.fixture_subscription = None
        result={'steps':self.steps,'score':self.scorer.score,'invalid':self.scorer.invalid,'dwell':self.scorer.dwell,
                'wall_seconds':time.monotonic()-self.started,'control_hz':spec.CONTROL_HZ,'physics_hz':spec.PHYSICS_HZ}
        (self.output/'result.json').write_text(json.dumps(result,indent=2)+'\n');self.trace.close()
        if self.env is not None:
            self.stage.Export(str(self.output/'terminal.usda'));self.env.close();self.env=None
