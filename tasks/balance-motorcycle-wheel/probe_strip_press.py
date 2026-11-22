"""Measure face-specific application pressure and check a tape bond after release."""

import argparse
import json
from math import cos, radians
from pathlib import Path
import traceback

import numpy as np


RADIAL_M, THICK_M, TANGENT_M = .015, .007, .0172
STRIP_KG = .014174
LAND_RADIUS_M = .233
FACE_OFFSET_M = .047
PRESS_N = 5. * 6894.757 * RADIAL_M * TANGENT_M
PRESS_SUBSTEPS = 12
DT = 1. / 240.
CASES = {'left': (1., 22., False), 'right': (-1., 22., False),
         'low': (1., 6., False), 'reversed': (1., 22., True)}


def measure_patch(points, normals, loads, strip, strip_rot, wheel, wheel_rot, side):
    """Return normal pressure and footprint spans for the actual adhesive face."""
    tape_points = (points - strip) @ strip_rot
    rim_points = (points - wheel) @ wheel_rot
    rim_out = side * wheel_rot[:, 1]
    radial = np.linalg.norm(rim_points[:, (0, 2)], axis=1)
    on_tape = np.abs(tape_points[:, 1] + THICK_M / 2.) <= .0012
    on_tape &= np.abs(tape_points[:, 0]) <= RADIAL_M / 2. + .0005
    on_tape &= np.abs(tape_points[:, 2]) <= TANGENT_M / 2. + .0005
    on_rim = (np.abs(rim_points[:, 1] - side * FACE_OFFSET_M) <= .0012) & (radial >= .219) & (radial <= .249)
    projection = np.abs(normals @ rim_out)
    accepted = on_tape & on_rim & (projection >= cos(radians(12.))) & (loads > 0.)
    footprint = tape_points[accepted][:, (0, 2)]
    spans = np.ptp(footprint, axis=0) if len(footprint) else np.zeros(2)
    normal_force = float(np.sum(loads[accepted] * projection[accepted]))
    aligned = float(strip_rot[:, 1] @ rim_out) >= cos(radians(5.))
    ready = (aligned and normal_force >= PRESS_N and spans[0] >= .50 * RADIAL_M
             and spans[1] >= .50 * TANGENT_M)
    return normal_force, spans, bool(ready)


def author_strip(stage, path, side, reversed_face=False, land_radius=LAND_RADIUS_M):
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    strip = UsdGeom.Xform.Define(stage, path)
    center = (.38 - land_radius, side * (FACE_OFFSET_M + THICK_M / 2. + .0003), .83)
    strip.AddTranslateOp().Set(Gf.Vec3d(*center))
    flip = (side < 0.) != reversed_face
    strip.AddRotateXOp().Set(180. if flip else 0.)
    prim = strip.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(STRIP_KG)
    box = UsdGeom.Cube.Define(stage, path + '/Shape')
    box.CreateSizeAttr(1.)
    box.AddScaleOp().Set(Gf.Vec3f(RADIAL_M, THICK_M, TANGENT_M))
    UsdPhysics.CollisionAPI.Apply(box.GetPrim())
    PhysxSchema.PhysxCollisionAPI.Apply(box.GetPrim()).CreateContactOffsetAttr(.0005)
    PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
    return prim


def read_contacts(view):
    loads, positions, normals, _, counts, starts = view.get_contact_data(DT)
    count = int(counts.numpy().flatten()[0])
    start = int(starts.numpy().flatten()[0])
    patch = slice(start, start + count)
    return positions.numpy().reshape(-1, 3)[patch], normals.numpy().reshape(-1, 3)[patch], loads.numpy().flatten()[patch]


def author_bond(stage, path, wheel, strip, relative, rotation, *, retain_contact=False):
    from pxr import Gf, UsdPhysics
    from yam_arena.ik import matrix_to_quat_xyzw

    joint = UsdPhysics.FixedJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([wheel.GetPath()])
    joint.CreateBody1Rel().SetTargets([strip.GetPath()])
    joint.CreateLocalPos0Attr(Gf.Vec3f(*map(float, relative)))
    x, y, z, w = matrix_to_quat_xyzw(rotation)
    joint.CreateLocalRot0Attr(Gf.Quatf(float(w), Gf.Vec3f(float(x), float(y), float(z))))
    joint.CreateCollisionEnabledAttr(retain_contact)
    joint.CreateBreakForceAttr(5.)
    joint.CreateBreakTorqueAttr(.05)


def run(asset, case, output):
    from isaaclab.app import AppLauncher

    app = AppLauncher(headless=True).app
    try:
        import omni.physics.tensors as tensors
        import warp as wp
        from isaaclab.sim import SimulationCfg, SimulationContext
        from isaaclab.sim.utils.stage import get_current_stage
        from isaaclab_physx.physics import PhysxManager
        from pxr import PhysxSchema, UsdGeom, UsdPhysics
        from yam_arena.urdf import quat_xyzw_to_matrix

        side, load, reversed_face = CASES[case]
        sim = SimulationContext(SimulationCfg(dt=DT, render_interval=12, device='cuda:0'))
        stage = get_current_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.)
        base = UsdGeom.Xform.Define(stage, '/World/Service')
        if not base.GetPrim().GetReferences().AddReference(str(asset.resolve())):
            raise RuntimeError('Could not attach the mounted wheel fixture')
        wheel = stage.GetPrimAtPath('/World/Service/Wheel')
        if not wheel or not wheel.HasAPI(UsdPhysics.RigidBodyAPI):
            raise RuntimeError('Expected the mounted wheel rigid body')

        strip_prim = author_strip(stage, '/World/Strip', side, reversed_face)
        PhysxSchema.PhysxContactReportAPI.Apply(wheel).CreateThresholdAttr(0.)

        wheel_path, strip_path = str(wheel.GetPath()), str(strip_prim.GetPath())
        sim.reset()
        view = PhysxManager.get_physics_sim_view()
        bodies = view.create_rigid_body_view([wheel_path, strip_path])
        impacts = view.create_rigid_contact_view(strip_path, filter_patterns=[wheel_path], max_contact_data_count=64)
        if bodies.count != 2 or impacts.filter_count != 1:
            raise RuntimeError('Expected both bodies and their single contact filter')
        wheel_id, strip_id = [bodies.prim_paths.index(path) for path in (wheel_path, strip_path)]
        indices = wp.array([strip_id], dtype=wp.int32, device='cuda:0')
        forces = np.zeros((2, 3), dtype=np.float32)
        forces[strip_id, 1] = -side * load
        press = wp.array(forces.flatten(), dtype=wp.float32, device='cuda:0')
        dwell = 0
        bond_step = None
        with output.open('w', buffering=1) as stream:
            for step in range(210):
                running_press = step < 48 and bond_step is None
                if running_press:
                    bodies.apply_forces(press, indices, True)
                sim.step(render=False)
                states = bodies.get_transforms().numpy().reshape(2, 7)
                wheel_xyz, strip_xyz = states[wheel_id, :3], states[strip_id, :3]
                wheel_rot = quat_xyzw_to_matrix(states[wheel_id, 3:])
                strip_rot = quat_xyzw_to_matrix(states[strip_id, 3:])
                relative = wheel_rot.T @ (strip_xyz - wheel_xyz)
                normal_force, all_contact_force, spans, qualified = 0., 0., np.zeros(2), False
                if running_press:
                    positions, normals, forces_n = read_contacts(impacts)
                    all_contact_force = float(forces_n.sum())
                    normal_force, spans, qualified = measure_patch(
                        positions, normals, forces_n, strip_xyz, strip_rot, wheel_xyz, wheel_rot, side)
                    dwell = dwell + 1 if qualified else 0
                    if dwell >= PRESS_SUBSTEPS:
                        bond_step = step
                        author_bond(stage, '/World/TapeBond', wheel, strip_prim, relative, wheel_rot.T @ strip_rot)
                        sim.forward()
                        if not view.is_valid:
                            view = tensors.create_simulation_view('warp')
                            bodies = view.create_rigid_body_view([wheel_path, strip_path])
                            wheel_id, strip_id = [bodies.prim_paths.index(path) for path in (wheel_path, strip_path)]
                row = {'case': case, 'step': step, 'pressing': running_press,
                       'normal_n': normal_force, 'rim_contact_n': all_contact_force,
                       'spans_mm': (1000. * spans).tolist(),
                       'qualified': qualified, 'bond_step': bond_step,
                       'face_outward_cos': float(side * (strip_rot[:, 1] @ wheel_rot[:, 1])),
                       'relative_xyz_m': relative.tolist(), 'strip_xyz_m': strip_xyz.tolist()}
                stream.write(json.dumps(row, allow_nan=False) + '\n')
        print(f'{case}: retained full trace', flush=True)
    except Exception:
        traceback.print_exc()
        app.close(exit_code=1)
        raise
    app.close()


if __name__ == '__main__':
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('asset', type=Path)
    cli.add_argument('case', choices=tuple(CASES))
    cli.add_argument('output', type=Path)
    args = cli.parse_args()
    run(args.asset, args.case, args.output)
