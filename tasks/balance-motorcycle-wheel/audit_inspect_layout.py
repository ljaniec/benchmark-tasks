"""Check scene reset and two image sizes through Inspect's YAM embodiment."""

import argparse
import json
from pathlib import Path

import numpy as np


CAMERAS = ('top_cam', 'left_cam', 'right_cam')
PROPS = ('strip_left', 'strip_right', 'shoe_left', 'shoe_right')


def cpu(value):
    return getattr(value, 'torch', value).detach().cpu().numpy().copy()


def snapshot(arena, observation, path, label):
    import cv2
    from isaaclab_physx.physics import PhysxManager

    roots = {}
    for name in PROPS:
        state = arena._base.scene[name].data
        roots[name] = np.concatenate((cpu(state.root_link_pose_w), cpu(state.root_link_vel_w)), axis=1)
    backend = PhysxManager.get_physics_sim_view()
    wheel = backend.create_rigid_body_view('/World/envs/env_0/*/Wheel')
    if wheel.count != 1:
        raise RuntimeError(f'Expected one passive wheel, found {wheel.prim_paths}')
    roots['wheel'] = np.concatenate((wheel.get_transforms().numpy(), wheel.get_velocities().numpy()), axis=1)
    for side in ('left', 'right'):
        state = arena._base.scene[f'{side}_robot'].data
        roots[f'{side}_arm'] = np.concatenate((cpu(state.joint_pos), cpu(state.joint_vel)), axis=1)
    if not all(np.isfinite(row).all() for row in roots.values()):
        raise ValueError('Nonfinite robot or object state')
    frames = {}
    for name in CAMERAS:
        native = cpu(arena._base.scene[name].data.output['rgb'])[0, :, :, :3]
        delivered = observation.images[name].copy()
        assert native.shape == (360, 640, 3) and delivered.shape == (176, 320, 3)
        np.testing.assert_array_equal(delivered, cv2.resize(native, (320, 176)))
        for resolution, frame in (('native', native), ('delivered', delivered)):
            assert frame.dtype == np.uint8 and frame.std() > 2.
            key = f'{name}-{resolution}'
            frames[key] = frame
            np.save(path / f'{label}-{key}.npy', frame)
    times = {name: float(value) for name, value in observation.image_times.items()}
    return roots, frames, (float(observation.state_time), times)


def audit(task_path, output):
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from yam_arena.inspect_embodiment import YamArenaEmbodiment
    from wheel_index_rgb import WheelIndexReader

    output.mkdir(parents=True, exist_ok=True)
    arena = YamArenaEmbodiment(task=str(task_path), control_hz=20, cam_width=320, cam_height=176, object_jitter=0.)
    scene = Scene(id='wheel-balance-layout', instruction=arena.task.module.INSTRUCTION)
    try:
        observed = arena.reset(scene, seed=0)
        initial = snapshot(arena, observed, output, 'initial')
        if not np.allclose(initial[0]['wheel'][0, :3], [.38, 0., .83], atol=.001):
            raise ValueError('The wheel did not settle at its axle')
        command = np.asarray(observed.state['joint_pos'], dtype=float).copy().reshape(14)
        command[[0, 7]] += [-.025, .025]
        for _ in range(12):
            moved = arena.step(Action(data=command)).observation
        terminal = snapshot(arena, moved, output, 'moved')
        repeated = snapshot(arena, arena.reset(scene, seed=0), output, 'repeated')
        physical = {key: float(np.max(np.abs(value - repeated[0][key]))) for key, value in initial[0].items()}
        images = {key: float(np.mean(np.abs(value.astype(float) - repeated[1][key]))) for key, value in initial[1].items()}
        moved_joints = float(np.max(np.abs(initial[0]['left_arm'] - terminal[0]['left_arm'])))
        reader = WheelIndexReader()
        angle = [reader.angle_deg(state[1]['top_cam-delivered']) for state in (initial, terminal, repeated)]
        reset_passed = (moved_joints > .005 and initial[2] == repeated[2]
                        and max(physical.values()) <= .002 and max(images.values()) <= 5.1)
        index_passed = all(value is not None and abs(value - 270.) <= 2. for value in angle)
        report = {'max_physical_delta': physical, 'mean_rgb_delta': images, 'initial_times': initial[2],
                  'repeated_times': repeated[2], 'moved_joint_delta': moved_joints,
                  'delivered_index_deg': angle, 'passed': reset_passed and index_passed}
        (output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
        if not reset_passed:
            raise RuntimeError('Inspect failed to reproduce the initial state after an arm movement')
        if not index_passed:
            raise RuntimeError(f'The stationary 270° mark was unreadable in an Inspect frame: {angle}')
        print('Completed repeated Inspect reset and delivered camera check', flush=True)
    finally:
        arena.close()


if __name__ == '__main__':
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('task', type=Path)
    cli.add_argument('output', type=Path)
    args = cli.parse_args()
    audit(args.task, args.output)
