"""Numeric checks for catching the ball of a wooden cup-and-ball toy in its cup."""
from dataclasses import dataclass, asdict
import math
import spec


@dataclass
class Evidence:
    handle_clear_m: float       # handle's end above the sleeve's rim
    tilt_deg: float             # handle axis from vertical
    ball_above_rim_m: float     # ball centre above the cup rim's centre
    ball_in_cup: bool           # ball centre inside the cup, below the rim
    ball_touched_robot: bool    # any robot link touched the ball during the last control step
    ball_touched_fixture: bool  # the ball made physical contact with the floor, stand or a pedestal
    toy_z_m: float
    inside_workspace: bool
    intact: bool


class Scorer:
    def __init__(self):
        self.score = 0.
        self.dwell = 0
        self.invalid = False
        self.lifted = False
        self.raised = False

    def update(self, e: Evidence):
        finite = all(math.isfinite(v) for v in asdict(e).values() if isinstance(v, float))
        dropped = e.toy_z_m < spec.DROP_Z
        # Only the toy may touch the ball, and the toy must stay off the floor.
        self.invalid |= ((not finite) or (not e.inside_workspace) or (not e.intact) or dropped
                         or e.ball_touched_robot or e.ball_touched_fixture)
        if self.invalid:
            self.score = 0.
            self.dwell = 0
            return 0.
        upright = e.tilt_deg <= spec.UPRIGHT_DEG
        self.lifted |= e.handle_clear_m >= spec.LIFT_CLEAR
        self.raised |= self.lifted and upright and e.ball_above_rim_m > 0.
        caught = self.lifted and upright and e.ball_in_cup
        self.dwell = self.dwell + 1 if caught else 0
        if self.score == 1. or self.dwell >= spec.DWELL_STEPS:
            self.score = 1.
        elif self.raised:
            self.score = .5
        elif self.lifted:
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
