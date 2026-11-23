"""Numeric checks for a foosball shot into the opponents' goal."""
from dataclasses import dataclass, asdict
import math
import spec


@dataclass
class Evidence:
    ball_progress_m: float      # ball's centre moved towards the opponents' goal (along -y) from its start
    ball_past_keeper: bool      # ball's centre beyond the opponents' keeper rod
    ball_in_goal: bool          # ball's centre through the goal mouth and behind the goal line
    rod_turn_deg: float         # largest turn of the three-man rod from its starting angle so far, either way
    robot_touched_ball: bool    # a robot link touched the ball during the last step
    robot_over_table: bool      # a robot collider overlapped the space over the playfield during the last step
    ball_z_m: float
    inside_workspace: bool
    intact: bool


class Scorer:
    def __init__(self):
        self.score = 0.
        self.dwell = 0
        self.invalid = False
        self.moved = False
        self.past_keeper = False

    def update(self, e: Evidence):
        finite = all(math.isfinite(v) for v in asdict(e).values() if isinstance(v, float))
        off_table = e.ball_z_m < spec.PF_Z - .10 and not e.ball_in_goal
        self.invalid |= ((not finite) or (not e.inside_workspace) or (not e.intact) or off_table
                         or e.rod_turn_deg > spec.SPIN_LIMIT_DEG or e.robot_touched_ball or e.robot_over_table)
        if self.invalid:
            self.score = 0.
            self.dwell = 0
            return 0.
        self.moved |= e.ball_progress_m >= spec.PROGRESS
        self.past_keeper |= e.ball_past_keeper
        self.dwell = self.dwell + 1 if e.ball_in_goal else 0
        if self.score == 1. or e.ball_in_goal:
            self.score = 1.
        elif self.past_keeper:
            self.score = .5
        elif self.moved:
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
