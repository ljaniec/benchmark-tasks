"""Numeric checks for catching the red fish of a magnetic fishing game and lifting it out of the pond."""
from dataclasses import dataclass, asdict
import math
import spec


@dataclass
class Evidence:
    magnet_touched_target: bool  # the magnet's face came within 1 mm of the red fish's washer during the last step
    target_attached: bool        # the magnet's face is within 2 mm of the red fish's washer
    target_lift_m: float         # lowest point of the red fish above the pond's floor
    target_above_rim_m: float    # lowest point of the red fish above the pond's rim
    other_fish_attached: bool    # another fish's washer is within 2 mm of the magnet's face
    robot_touched_game: bool     # a robot link touched a fish or the magnet during the last step
    inside_workspace: bool       # the rod, the magnet and every fish are in the workspace, and no fish is off the table
    intact: bool


class Scorer:
    def __init__(self):
        self.score = 0.
        self.dwell = 0
        self.invalid = False
        self.touched = False
        self.lifted = False

    def update(self, e: Evidence):
        finite = all(math.isfinite(v) for v in asdict(e).values() if isinstance(v, float))
        self.invalid |= (not finite) or (not e.inside_workspace) or (not e.intact) or e.robot_touched_game
        if self.invalid:
            self.score = 0.
            self.dwell = 0
            return 0.
        self.touched |= e.magnet_touched_target
        self.lifted |= e.target_attached and e.target_lift_m >= spec.LIFT_OFF
        caught = e.target_attached and e.target_above_rim_m >= spec.ABOVE_RIM and not e.other_fish_attached
        self.dwell = self.dwell + 1 if caught else 0
        if self.score == 1. or self.dwell >= spec.DWELL_STEPS:
            self.score = 1.
        elif self.lifted:
            self.score = .5
        elif self.touched:
            self.score = .25
        else:
            self.score = 0.
        return self.score


def regrade(path):
    import json
    from pathlib import Path
    scorer = Scorer()
    count = 0
    peak = 0.
    for line in Path(path).read_text().splitlines():
        row = json.loads(line)
        score = scorer.update(Evidence(**row['evidence']))
        if score != row['score'] or scorer.invalid != row['invalid'] or scorer.dwell != row['dwell']:
            raise ValueError(f"Recorded score mismatch at step {row['step']}")
        count += 1
        peak = max(peak, score)
    if not count:
        raise ValueError('Empty evidence trace')
    return {
        'steps': count,
        'terminal_score': scorer.score,
        'peak_score': peak,
        'dwell': scorer.dwell,
        'invalid': scorer.invalid,
        'recorded_scores_match': True,
    }


if __name__ == '__main__':
    import argparse
    import json
    parser = argparse.ArgumentParser()
    parser.add_argument('trace')
    print(json.dumps(regrade(parser.parse_args().trace), indent=2))
