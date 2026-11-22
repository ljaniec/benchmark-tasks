"""Apply adhesive wheel weights and demonstrate hands-off static balance."""

import json
import os
from dataclasses import asdict
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from inspect_layout import m, overhead_camera, task_objects
from wheel_physics import WheelPhysics
from wheel_readout import WheelReadout
from scorer import WheelScore
from yam_arena import station
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, GRIPPER


SCENE = 'station'
INSTRUCTION = 'Press a balance weight onto each rim face, then release the wheel at each marked position and check that it stays put.'
RECREATE_SCENE_ON_RESET = True
SUCCESS_HOLD_S = 0.05
SCORE_OVER = 'end'
GRIPPER['effort'] = 40.
PLAN = json.loads((HERE / 'robot-press-shoe-plan.json').read_text())
OBJECTS = task_objects(PLAN)
_score = 0.

stock_camera = station.top_camera
stock_camera_stand = station.camera_stand


def wheel_camera(variant='calibrated'):
    return overhead_camera(stock_camera(variant))


def wheel_camera_stand(optical_camera):
    # The station normally puts its colliding mast underneath the camera.
    # Keep the established stand placement when shifting the view.
    return stock_camera_stand(overhead_camera(optical_camera, 'station_stand'))


station.top_camera = wheel_camera
station.camera_stand = wheel_camera_stand


def log_readout(entry):
    path = os.environ.get('WHEEL_TRIAL_TRACE')
    if path:
        with open(path, 'a', encoding='utf-8') as out:
            out.write(json.dumps(entry, allow_nan=False) + '\n')


@m.on_before_reset
def clear_loads(env):
    base = env.unwrapped
    if hasattr(base, 'wheel_physics'):
        base.sim.step = base.wheel_physics.original_step
        del base.wheel_physics


@m.on_reset
def reset_grippers(env):
    import torch

    global _score
    _score = 0.
    base = env.unwrapped
    start = PLAN['waypoints'][0]
    for side in ('left', 'right'):
        robot = base.scene[f'{side}_robot']
        ids = robot.find_joints([*ARM_JOINTS, *FINGER_JOINTS], preserve_order=True)[0]
        joints = robot.data.default_joint_pos.torch.clone()
        angles = [*start['targets'][side]['joint_rad'], start['finger_m'], start['finger_m']]
        joints[:, ids] = torch.tensor(angles, device=joints.device, dtype=joints.dtype)
        robot.write_joint_state_to_sim_index(position=joints, velocity=torch.zeros_like(joints))
        robot.reset()
    base.sim.forward()
    for _ in range(8):
        base.sim.render()
    for name in ('top_cam', 'left_cam', 'right_cam'):
        base.scene[name].update(0., force_recompute=True)
    physics = WheelPhysics(base)
    physics.original_step = base.sim.step
    base.wheel_physics = physics
    base.wheel_readout = WheelReadout()
    base.wheel_score = WheelScore()
    log_readout({'reset': True, 'score': _score})

    def step_with_contact(*args, **kwargs):
        physics.apply_loads()
        result = physics.original_step(*args, **kwargs)
        physics.sample_contact()
        return result

    base.sim.step = step_with_contact


@m.on_step
def measure_release(env):
    global _score
    base = env.unwrapped
    base.wheel_reading = base.wheel_readout.observe(base)
    _score = base.wheel_score.update(base.wheel_reading)
    log_readout({'substeps': base.wheel_physics.substeps,
                 **asdict(base.wheel_reading), 'score': _score,
                 'completed_releases': base.wheel_score.checks})


def score(objects):
    return _score


def success(objects):
    return _score == 1.
