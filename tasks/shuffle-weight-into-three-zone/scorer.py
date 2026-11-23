"""Score ladder for one shuffleboard shot; pure Python so it can be re-run on a recorded trace."""
from dataclasses import dataclass
import json
import math
import sys
import spec


@dataclass
class Evidence:
    robot_on_weight: bool       # an arm or gripper touched the weight during the step
    weight_x: float             # centre of the weight's base, along the board
    weight_y: float             # and across it
    weight_lift: float          # height of the base's centre above the playfield
    weight_tilt_deg: float      # the weight's axis from vertical
    weight_speed: float
    inside: bool                # the weight is in the workspace
    intact: bool                # table and pedestals unmoved


class Scorer:
    def __init__(self):
        self.score = 0.
        self.invalid = False
        self.shot = False           # the weight has slid at SHOT_SPEED or faster with nothing touching it
        self.dead = False           # after the shot it left the playfield (into the gutter)
        self.resting = 0            # consecutive steps at rest, flat on the board, after the shot

    def foul(self, e):
        """Events that score 0 for the trial."""
        if not all(math.isfinite(x) for x in (e.weight_x, e.weight_y, e.weight_lift, e.weight_tilt_deg, e.weight_speed)):
            return True
        if not e.inside or not e.intact:
            return True
        if e.weight_lift > spec.LIFT_LIMIT and not e.robot_on_weight:   # in the air with nothing holding it: thrown
            return True
        if self.shot and e.robot_on_weight:                               # touched again after it was let go
            return True
        return False

    def update(self, e):
        if self.invalid:
            return 0.
        if self.foul(e):
            self.invalid = True
            self.score = 0.
            return 0.
        if not e.robot_on_weight and e.weight_speed >= spec.SHOT_SPEED:
            self.shot = True
        if self.shot and not spec.on_board(e.weight_x, e.weight_y, e.weight_lift):
            self.dead = True
        at_rest = (self.shot and not self.dead and e.weight_speed < spec.REST_SPEED
                   and e.weight_tilt_deg < spec.FLAT_DEG and not e.robot_on_weight)
        self.resting = self.resting + 1 if at_rest else 0
        self.score = spec.zone_score(e.weight_x) if self.resting >= spec.REST_STEPS else 0.
        return self.score


def regrade(path):
    scorer = Scorer()
    score = 0.
    for line in open(path):
        row = json.loads(line)
        score = scorer.update(Evidence(**row['evidence']))
    return score, scorer.invalid


if __name__ == '__main__':
    score, invalid = regrade(sys.argv[1])
    print(json.dumps({'score': score, 'invalid': invalid}))
