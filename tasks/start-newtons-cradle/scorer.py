"""Numeric checks for starting a Newton's cradle: lift ball 1 on taut lines, let it go, ball 5 alone swings out."""
from dataclasses import dataclass, asdict
import math
import spec


@dataclass
class Evidence:
    ball1_angle_deg: float       # ball 1's swing towards the arms (degrees from hanging)
    ball1_taut: bool             # ball 1's centre is within 3 mm of its swing circle (both lines taut)
    others_max_deg: float        # largest swing of balls 2-5, either way
    ball1_speed_mps: float
    far_out_deg: float           # ball 5's swing away from the arms
    middle_max_deg: float        # largest swing of balls 2-4, either way
    robot_touched_ball1: bool    # a robot link touched ball 1 during the last step
    robot_touched_other: bool    # a robot link touched balls 2-5 during the last step
    inside_workspace: bool
    intact: bool


class Scorer:
    def __init__(self):
        self.score = 0.
        self.invalid = False
        self.lifted = False
        self.armed = False
        self.held = None             # (angle, taut, others still, speed) at the last step ball 1 was touched
        self.far_max = 0.
        self.middle_max = 0.

    def update(self, e: Evidence):
        finite = all(math.isfinite(v) for v in asdict(e).values() if isinstance(v, float))
        self.invalid |= (not finite) or (not e.inside_workspace) or (not e.intact) or e.robot_touched_other
        if self.invalid:
            self.score = 0.
            self.armed = False
            return 0.
        if e.robot_touched_ball1:
            still = e.others_max_deg <= spec.STILL_DEG
            self.lifted |= e.ball1_angle_deg >= spec.LIFTED_DEG and e.ball1_taut and still
            self.held = (e.ball1_angle_deg, e.ball1_taut, still, e.ball1_speed_mps)
            self.armed = False
        elif self.held is not None:
            angle, taut, still, speed = self.held
            self.held = None
            # Let go from at least 20 degrees, lines taut, the other balls hanging still, the ball not pushed.
            self.armed = angle >= spec.LIFTED_DEG and taut and still and speed <= spec.RELEASE_SPEED
            self.far_max = 0.
            self.middle_max = 0.
        if self.armed and not e.robot_touched_ball1:
            self.far_max = max(self.far_max, e.far_out_deg)
            self.middle_max = max(self.middle_max, e.middle_max_deg)
            if self.far_max >= 2. and e.far_out_deg < self.far_max - 1.:
                # Ball 5 has turned at the top of its first swing out: judge that swing.
                if self.far_max >= spec.FAR_FULL and self.middle_max <= spec.MIDDLE_LIMIT:
                    self.score = 1.
                elif self.far_max >= spec.FAR_HALF:
                    self.score = max(self.score, .5)
                self.armed = False
        if self.score < .25 and self.lifted:
            self.score = .25
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
        if score != row['score'] or scorer.invalid != row['invalid']:
            raise ValueError(f"Recorded score mismatch at step {row['step']}")
        count += 1
        peak = max(peak, score)
    if not count:
        raise ValueError('Empty evidence trace')
    return {
        'steps': count,
        'terminal_score': scorer.score,
        'peak_score': peak,
        'invalid': scorer.invalid,
        'recorded_scores_match': True,
    }


if __name__ == '__main__':
    import argparse
    import json
    parser = argparse.ArgumentParser()
    parser.add_argument('trace')
    print(json.dumps(regrade(parser.parse_args().trace), indent=2))
