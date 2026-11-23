"""Numeric checks for tossing a ring onto the centre peg of a ring toss set."""
from dataclasses import dataclass, asdict
import math
import spec


@dataclass
class Evidence:
    robot_past_line: bool       # a robot collider overlapped the space beyond the throw line during the last step
    ring_touched_set: bool      # the ring came within 1 mm of the base or any peg during the last step
    ring_touched_target: bool   # the ring came within 1 mm of the centre peg during the last step
    ring_around_target: bool    # the centre peg's axis passes through the ring's hole below the peg's tip
    ring_landed: bool           # the ring has touched the floor or the set since it left the table
    ring_touched_robot: bool    # a robot link touched the ring during the last step
    ring_z_m: float
    inside_workspace: bool
    intact: bool


class Scorer:
    def __init__(self):
        self.score = 0.
        self.dwell = 0
        self.invalid = False
        self.hit_set = False
        self.hit_target = False

    def update(self, e: Evidence):
        finite = all(math.isfinite(v) for v in asdict(e).values() if isinstance(v, float))
        # A ring that has landed may not be picked up, pushed or placed by the robot.
        self.invalid |= ((not finite) or (not e.inside_workspace) or (not e.intact) or e.robot_past_line
                         or (e.ring_landed and e.ring_touched_robot))
        if self.invalid:
            self.score = 0.
            self.dwell = 0
            return 0.
        self.hit_set |= e.ring_touched_set
        self.hit_target |= e.ring_touched_target
        self.dwell = self.dwell + 1 if e.ring_around_target else 0
        if self.score == 1. or self.dwell >= spec.DWELL_STEPS:
            self.score = 1.
        elif self.hit_target:
            self.score = .5
        elif self.hit_set:
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
