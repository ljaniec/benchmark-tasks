"""Score ladder for one carrom stroke at the queen; pure Python so it can be re-run on a recorded trace."""
from dataclasses import dataclass
import json
import math
import sys
import spec


@dataclass
class Evidence:
    robot_on_striker: bool      # an arm or gripper touched the striker during the step
    robot_on_queen: bool        # or the queen
    touch_outside: float        # furthest the striker's centre was outside the base-line window while touched (m)
    striker_x: float            # centre of the striker's base
    striker_y: float
    striker_lift: float         # height of its base above the playing surface (negative: fallen into a pocket)
    striker_speed: float
    queen_x: float
    queen_y: float
    queen_lift: float
    queen_speed: float
    inside: bool                # both discs in the workspace
    intact: bool                # board, frame and pedestals unmoved


class Scorer:
    def __init__(self):
        self.score = 0.
        self.invalid = False
        self.stroke = False         # the striker has left the base lines untouched: the one stroke has been played
        self.hit = False            # the queen has moved from its spot
        self.resting = 0            # consecutive steps with both discs at rest, untouched, after the stroke

    def foul(self, e):
        """Events that score 0 for the trial."""
        values = (e.touch_outside, e.striker_x, e.striker_y, e.striker_lift, e.striker_speed,
                  e.queen_x, e.queen_y, e.queen_lift, e.queen_speed)
        if not all(math.isfinite(x) for x in values):
            return True
        if not e.inside or not e.intact:
            return True
        if e.robot_on_queen:                                         # the queen is played only with the striker
            return True
        if e.robot_on_striker and (self.stroke or e.touch_outside > spec.PUSH_LIMIT):
            return True                                              # pushed, or touched again after the stroke
        for x, y, lift, touched in ((e.striker_x, e.striker_y, e.striker_lift, e.robot_on_striker),
                                    (e.queen_x, e.queen_y, e.queen_lift, False)):
            if spec.pocketed(x, y, lift):
                continue
            if not touched and lift > spec.LIFT_LIMIT:                # in the air: thrown or jumped
                return True
            if not spec.on_board(x, y, lift) and not touched:          # over the frame
                return True
        return False

    def update(self, e):
        if self.invalid:
            return 0.
        if self.foul(e):
            self.invalid = True
            self.score = 0.
            return 0.
        if not e.robot_on_striker and spec.window_outside(e.striker_x, e.striker_y) > spec.PUSH_LIMIT:
            self.stroke = True
        if math.hypot(e.queen_x - spec.QUEEN_START[0], e.queen_y - spec.QUEEN_START[1]) >= spec.HIT_MOVE \
                or spec.pocketed(e.queen_x, e.queen_y, e.queen_lift):
            self.hit = True
        at_rest = (self.stroke and not e.robot_on_striker and e.striker_speed < spec.REST_SPEED
                   and e.queen_speed < spec.REST_SPEED)
        self.resting = self.resting + 1 if at_rest else 0
        self.score = 0.
        if self.resting >= spec.REST_STEPS:
            if spec.pocketed(e.queen_x, e.queen_y, e.queen_lift):
                self.score = .5 if spec.pocketed(e.striker_x, e.striker_y, e.striker_lift) else 1.
            elif self.hit and not spec.pocketed(e.striker_x, e.striker_y, e.striker_lift):
                self.score = .25
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
