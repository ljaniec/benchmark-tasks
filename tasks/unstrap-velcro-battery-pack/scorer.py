"""State-based evaluator; controllers never receive this measurement channel.

Each predicate corresponds to a ruler/photo/video check described in README.md.
Invalid events are permanent for the episode; terminal dwell is consecutive.
"""
from dataclasses import dataclass
import math
from spec import DWELL_STEPS


@dataclass(frozen=True)
class Evidence:
    restraint_open: bool = False
    battery_clear: bool = False
    battery_on_mat: bool = False
    hands_clear: bool = False
    battery_motion_mm: float = math.inf
    module_tilt_deg: float = 0.0
    within_workspace: bool = True
    intact: bool = True
    shortcut: bool = False


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.invalid = False
        self.dwell = 0
        self.score = 0.0

    def update(self, e: Evidence):
        self.invalid |= (not math.isfinite(e.module_tilt_deg) or e.module_tilt_deg > 15
                         or not e.within_workspace or not e.intact or e.shortcut)
        if self.invalid:
            self.dwell = 0
            self.score = 0.0
            return self.score
        full = (e.restraint_open and e.battery_clear and e.battery_on_mat
                and e.hands_clear and 0 <= e.battery_motion_mm <= 1.0)
        self.dwell = self.dwell + 1 if full else 0
        self.score = (1.0 if self.dwell >= DWELL_STEPS else
                      0.5 if e.restraint_open and e.battery_clear else
                      0.25 if e.restraint_open else 0.0)
        return self.score


if __name__ == '__main__':
    import argparse
    import json
    from pathlib import Path

    parser = argparse.ArgumentParser(description='Replay saved evaluator measurements through the task rubric.')
    parser.add_argument('trace', type=Path, help='Per-episode evaluator trace.jsonl')
    args = parser.parse_args()
    scorer = Scorer()
    count = 0
    for count, line in enumerate(args.trace.read_text().splitlines(), start=1):
        row = json.loads(line)
        if row['step'] != count - 1:
            raise ValueError('Missing or reordered control-step evidence')
        score = scorer.update(Evidence(**row['evidence']))
        if (score, scorer.dwell, scorer.invalid) != (row['score'], row['dwell'], row['invalid']):
            raise ValueError(f'Recorded grade differs at step {row["step"]}')
    if not count:
        raise ValueError('Empty evidence trace')
    print(json.dumps({'steps': count, 'score': scorer.score, 'dwell': scorer.dwell,
                      'invalid': scorer.invalid, 'recorded_grades_match': True}))
