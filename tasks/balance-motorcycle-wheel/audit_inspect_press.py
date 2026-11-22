"""Exercise native weight application through Inspect's normal action interface."""

import argparse
from dataclasses import asdict
import json
from itertools import chain
from pathlib import Path

import numpy as np

from demo_actions import actions
from demo import park_applicators
from wheel_index_rgb import WheelIndexReader


def audit(task_path, output, max_steps, observe_park=False):
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from yam_arena.inspect_embodiment import YamArenaEmbodiment

    output.mkdir(parents=True, exist_ok=True)
    arena = YamArenaEmbodiment(task=str(task_path), control_hz=20, cam_width=320, cam_height=176, object_jitter=0.)
    scene = Scene(id='wheel-weight-application', instruction=arena.task.module.INSTRUCTION)
    reader = WheelIndexReader()
    plan = arena.task.module.PLAN
    commands = actions(plan)
    if observe_park:
        commands = chain(commands, actions(park_applicators(plan)))
    try:
        start = arena.reset(scene, seed=0)
        initial = arena._base.wheel_physics.snapshot()
        phases = {}
        with (output / 'trace.jsonl').open('w', buffering=1) as trace:
            for tick, (phase, command) in enumerate(commands, start=1):
                observed = arena.step(Action(data=command)).observation
                contact = arena._base.wheel_physics.snapshot()
                contact['step'] = tick
                contact['phase'] = phase
                contact['rgb_deg'] = reader.angle_deg(observed.images['top_cam'])
                if observe_park:
                    contact['score_reading'] = asdict(arena._base.wheel_reading)
                trace.write(json.dumps(contact, allow_nan=False) + '\n')
                phases[phase] = observed.images
                if tick >= max_steps:
                    break
        for phase, images in phases.items():
            for camera, frame in images.items():
                np.save(output / f'{phase}-{camera}.npy', frame)
        repeated = arena.reset(scene, seed=0)
        reset = arena._base.wheel_physics.snapshot()
        reset_ok = reset == initial and all(np.array_equal(image, repeated.images[name])
                                             for name, image in start.images.items())
        cadence_ok = contact['substeps'] == tick * 12
        both_applied = all(part['bond_active'] and part['clearance_m'] >= .045
                           for part in contact['parts'].values())
        result = {
            'reset_ok': reset_ok,
            'cadence_ok': cadence_ok,
            'both_applied_and_applicators_withdrawn': both_applied,
            'observed_parking': observe_park,
            'passed': reset_ok and cadence_ok and both_applied,
            'terminal': contact,
        }
        (output / 'press-report.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
        if not result['passed']:
            raise RuntimeError('Weight application audit did not reproduce both pressure bonds, release and reset')
        phase_name = 'pressure, parking' if observe_park else 'pressure, withdrawal'
        print(f'Completed native Inspect {phase_name} and reset audit', flush=True)
    finally:
        arena.close()


if __name__ == '__main__':
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('task', type=Path)
    cli.add_argument('output', type=Path)
    cli.add_argument('--max-steps', type=int, metavar='STEPS')
    cli.add_argument('--observe-park', action='store_true', help='Trace strip geometry through applicator parking')
    args = cli.parse_args()
    default_limit = 282 if args.observe_park else 196
    max_steps = default_limit if args.max_steps is None else args.max_steps
    if not 1 <= max_steps <= (282 if args.observe_park else 200):
        cli.error('The step limit must be within the selected diagnostic')
    audit(args.task, args.output, max_steps, args.observe_park)
