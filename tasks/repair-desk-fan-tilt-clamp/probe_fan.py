"""Compare a free pitch joint and a tightened pitch joint under gravity and proof load."""
import argparse
import json
import math
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np

import clamp
import geometry


CASES = {"loose": clamp.LOOSE_DEG, "tight": 24.}
SETTLE_STEPS = 480
PROOF_STEPS = 240
PROOF_END = SETTLE_STEPS + PROOF_STEPS


def assemble(stage, source):
    from pxr import Gf, UsdGeom, UsdPhysics

    bench = UsdGeom.Cube.Define(stage, "/World/bench")
    bench.AddTranslateOp().Set(Gf.Vec3d(0., 0., -.012))
    bench.AddScaleOp().Set(Gf.Vec3d(1., .58, .024))
    UsdPhysics.CollisionAPI.Apply(bench.GetPrim())
    paths = {}
    for name, x in (("loose", -.24), ("tight", .24)):
        path = "/World/" + name
        item = UsdGeom.Xform.Define(stage, path)
        item.AddTranslateOp().Set(Gf.Vec3d(x, 0., 0.))
        item.GetPrim().GetReferences().AddReference(str(source))
        paths[name] = path
    return paths


def run(source):
    from isaaclab.app import AppLauncher

    app = AppLauncher(headless=True).app
    try:
        from isaaclab.sim import SimulationCfg, SimulationContext
        from isaaclab.sim.utils.stage import get_current_stage
        from isaaclab_physx.physics import PhysxManager
        from pxr import UsdGeom
        import warp as wp

        sim = SimulationContext(SimulationCfg(dt=1. / 480., render_interval=24, device="cuda:0"))
        stage = get_current_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.)
        paths = assemble(stage, source)
        sim.reset()
        backend = PhysxManager.get_physics_sim_view()
        cpu_indices = wp.array([0], dtype=wp.int32, device="cpu")
        gpu_indices = wp.array([0], dtype=wp.int32, device="cuda:0")
        active = {}
        for name, path in paths.items():
            view = backend.create_articulation_view(path)
            joint_names = view.shared_metatype.dof_names
            pitch = joint_names.index("head_joint")
            knob = joint_names.index("clamp_joint")
            angle = math.radians(CASES[name])
            poses = np.zeros((1, len(joint_names)), dtype=np.float32)
            poses[0, knob] = angle
            view.set_dof_positions(wp.array2d(poses, dtype=wp.float32, device="cuda:0"), gpu_indices)
            view.set_dof_velocities(wp.zeros_like(wp.array2d(poses, dtype=wp.float32, device="cuda:0")), gpu_indices)
            friction = view.get_dof_friction_properties().numpy().copy()
            for axis, moment in ((pitch, clamp.breakaway_moment(angle)), (knob, clamp.running_moment(angle))):
                friction[0, axis, 0] = moment
                friction[0, axis, 1] = .95 * moment
            view.set_dof_friction_properties(wp.array(friction, dtype=wp.float32, device="cpu"), cpu_indices)
            active[name] = (view, pitch, knob)
        trace = {name: [] for name in active}
        for step in range(PROOF_END + 240):
            for name, (view, pitch, knob) in active.items():
                load = np.zeros((1, 2), dtype=np.float32)
                if SETTLE_STEPS <= step < PROOF_END:
                    load[0, pitch] = -clamp.PROOF_MOMENT_NM
                view.set_dof_actuation_forces(wp.array2d(load, dtype=wp.float32, device="cuda:0"), gpu_indices)
            sim.step(render=False)
            for name, (view, pitch, knob) in active.items():
                angle = view.get_dof_positions().numpy()[0]
                trace[name].append((math.degrees(float(angle[pitch])), math.degrees(float(angle[knob]))))
        write_report(trace)
    except Exception:
        import traceback
        traceback.print_exc()
        raise
    finally:
        app.close()


def write_report(data):
    tight = data["tight"]
    checks = {
        "loose_sags_under_head_weight": data["loose"][SETTLE_STEPS - 1][0] < -10.,
        "tight_holds_before_proof": abs(tight[SETTLE_STEPS - 1][0]) <= 2.,
        "tight_holds_during_proof": max(abs(x[0]) for x in tight[SETTLE_STEPS:PROOF_END]) <= 2.,
        "tight_holds_after_proof": abs(tight[-1][0]) <= 2.,
    }
    result = {"passed": all(checks.values()), "checks": checks,
              "loose_before_proof_deg": data["loose"][SETTLE_STEPS - 1][0],
              "tight_peak_deg": max(abs(x[0]) for x in tight),
              "proof_moment_nm": clamp.PROOF_MOMENT_NM}
    Path("fan-contact.json").write_text(json.dumps(result, indent=2) + "\n")
    Path("fan-contact-trace.json").write_text(json.dumps(data) + "\n")
    print(json.dumps(result, indent=2), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--author-only", action="store_true")
    args = parser.parse_args()
    with TemporaryDirectory(prefix="desk-fan-pitch-") as folder:
        source = Path(folder) / "fan.usda"
        source.write_text(geometry.fan().build())
        if args.author_only:
            from pxr import Usd
            stage = Usd.Stage.CreateInMemory()
            assert all(stage.GetPrimAtPath(p).IsValid() for p in assemble(stage, source).values())
            print("Both guarded fan fixtures compose")
            return
        run(source)


if __name__ == "__main__":
    main()
