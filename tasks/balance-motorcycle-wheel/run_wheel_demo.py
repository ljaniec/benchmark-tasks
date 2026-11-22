"""Record a wheel demo with Inspect's standard scorer and RGB frame store."""

import argparse
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
RUN_LIMIT = 1600


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log-dir', required=True, type=Path)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--max-steps', type=int, default=RUN_LIMIT)
    args = parser.parse_args()
    if not 1 <= args.max_steps <= RUN_LIMIT:
        parser.error(f'Use between 1 and {RUN_LIMIT} control steps')

    from demo import register_policy
    from inspect_robots.cli import main as inspect_main

    instruction = json.loads((HERE / 'task.json').read_text())['instruction']
    return inspect_main([
        'run', '--embodiment', 'yam_arena', '--policy', register_policy(),
        '-E', f'task={HERE / "inspect_wheel_task.py"}',
        '-E', 'control_hz=20', '-E', 'object_jitter=0',
        '-E', 'cam_width=320', '-E', 'cam_height=176',
        '--no-prompt', '--grader', 'none', '--scorer', 'task_score',
        '--max-steps', str(args.max_steps), '--seed', str(args.seed),
        '--store-frames', '--log-dir', str(args.log_dir.resolve()),
        '--instruction', instruction,
    ])


if __name__ == '__main__':
    raise SystemExit(main())
