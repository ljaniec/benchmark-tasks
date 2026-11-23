"""Compare an untouched and a pressed nip under the same bounded axial pull."""
import argparse
import json
import math
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np

import geometry as g


def assemble(stage, directory):
    from pxr import Gf, UsdGeom, UsdPhysics

    stage.DefinePrim('/World', 'Xform')
    bench = UsdGeom.Cube.Define(stage, '/World/bench')
    bench.AddTranslateOp().Set(Gf.Vec3d(0., 0., -g.RISER_HEIGHT - .009))
    bench.AddScaleOp().Set(Gf.Vec3d(.45, .50, .018))
    UsdPhysics.CollisionAPI.Apply(bench.GetPrim())
    files = []
    for part in (g.OBJECTS[0], g.OBJECTS[1], g.recovery_support(), g.riser()):
        file = directory / (part.name + '.usda')
        file.write_text(part.build())
        files.append(file)
    cases = {}
    for name, lateral in (('untouched', -.240), ('pressed', .240)):
        parent = f'/World/{name}'
        body = UsdGeom.Xform.Define(stage, parent)
        body.AddTranslateOp().Set(Gf.Vec3d(0., lateral, 0.))
        laminator = UsdGeom.Xform.Define(stage, parent + '/laminator')
        pouch = UsdGeom.Xform.Define(stage, parent + '/pouch')
        pouch.AddTranslateOp().Set(Gf.Vec3d(*map(float, g.POUCH_START)))
        guides = UsdGeom.Xform.Define(stage, parent + "/entry_guides")
        riser = UsdGeom.Xform.Define(stage, parent + "/riser")
        for organ, file in zip((laminator, pouch, guides, riser), files, strict=True):
            organ.GetPrim().GetReferences().AddReference(str(file))
        joint, (stiffness, damping, *_rest) = g.OBJECTS[0].drives[0]
        drive = UsdPhysics.DriveAPI(laminator.GetPrim().GetChild(joint), 'angular')
        drive.GetStiffnessAttr().Set(math.radians(stiffness))
        drive.GetDampingAttr().Set(math.radians(damping))
        cases[name] = tuple(str(organ.GetPath()) for organ in (laminator, pouch))
    return cases


def ease(value):
    f = min(1., max(0., value))
    return f * f * (3. - 2. * f)


def native(directory, output):
    from isaaclab.app import AppLauncher
    app = AppLauncher(headless=True).app
    try:
        def milestone(name):
            print(name, flush=True)

        from isaaclab.sim import SimulationCfg, SimulationContext
        from isaaclab.sim.utils.stage import get_current_stage
        from isaaclab_physx.physics import PhysxManager
        from pxr import UsdGeom
        import warp as wp

        sim = SimulationContext(SimulationCfg(dt=1./480., render_interval=24, device='cuda:0'))
        stage = get_current_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.)
        cases = assemble(stage, directory)
        milestone("Assembled both complete cases")
        sim.reset()
        milestone("Reset finished")
        backend = PhysxManager.get_physics_sim_view()
        controls = {name: (backend.create_articulation_view(machine), backend.create_rigid_body_view(pouch))
                    for name, (machine, pouch) in cases.items()}
        milestone("Created articulation and rigid-body views")
        index = wp.array([0], dtype=wp.int32, device='cuda:0')
        zero_torque = wp.zeros((1, 3), dtype=wp.float32, device='cuda:0')
        records = {name: [] for name in cases}
        for step in range(670):
            for name, (machine, pouch) in controls.items():
                radians = float(machine.get_dof_positions().numpy()[0, 0])
                feed = float(pouch.get_transforms().numpy()[0, 0])
                speed = float(pouch.get_velocities().numpy()[0, 0])
                if step == 0:
                    milestone(f"Observed initial {name} state")
                if name == 'pressed':
                    # An applied 0.96 Nm moment corresponds to approximately 11 N at the paddle center.
                    effort = .96 * ease((step - 20) / 100.)
                    drive = wp.array2d(np.array([[effort]], dtype=np.float32), dtype=wp.float32, device='cuda:0')
                    machine.set_dof_actuation_forces(drive, index)
                wanted = g.POUCH_START[0] - .093 * ease((step - 170) / 400.)
                force = np.clip(550. * (wanted - feed) - 5. * speed, -4., 4.)
                wrench = wp.array2d(np.array([[force, 0., 0.]], dtype=np.float32), dtype=wp.float32, device='cuda:0')
                pouch.apply_forces_and_torques_at_position(force_data=wrench, torque_data=zero_torque,
                                                         position_data=None, indices=index, is_global=True)
                records[name].append([float(1000. * (g.POUCH_START[0] - feed)), math.degrees(radians)])
            sim.step(render=False)
            if step in (0, 20, 170, 669):
                milestone(f"Completed physics step {step}")
        results = {name: dict(draw_mm=trace[-1][0], lever_deg=trace[-1][1]) for name, trace in records.items()}
        checks = dict(unpressed_retained=results['untouched']['draw_mm'] < 5.,
                      pressed_released=results['pressed']['draw_mm'] > 71.,
                      paddle_travel_reached=results['pressed']['lever_deg'] >= 7.)
        result = dict(passed=all(checks.values()), checks=checks, states=results, pull_limit_n=4.)
        output.write_text(json.dumps(result, indent=2) + '\n')
        output.with_suffix('.trace.json').write_text(json.dumps(records) + '\n')
        print(result)
    except Exception:
        import traceback
        traceback.print_exc()
        raise
    finally:
        app.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check-usd', action='store_true')
    parser.add_argument('--output', type=Path, default=Path('release-load.json'))
    args = parser.parse_args()
    with TemporaryDirectory(prefix='laminator-nip-') as directory:
        directory = Path(directory)
        if args.check_usd:
            from pxr import Usd, UsdPhysics
            stage = Usd.Stage.CreateInMemory()
            cases = assemble(stage, directory)
            assert all(stage.GetPrimAtPath(path).IsValid() for pair in cases.values() for path in pair)
            for housing, _ in cases.values():
                drive = UsdPhysics.DriveAPI(stage.GetPrimAtPath(housing + '/release_joint'), 'angular')
                assert math.isclose(drive.GetStiffnessAttr().Get(), math.radians(5.2), rel_tol=1e-6)
            print('Composed samples for pressed and untouched release levers')
        else:
            native(directory, args.output)


if __name__ == '__main__':
    main()
