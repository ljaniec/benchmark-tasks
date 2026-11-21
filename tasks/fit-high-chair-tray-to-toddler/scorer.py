"""Numeric checks for fitting the high-chair tray to the seated toddler."""
from dataclasses import dataclass, asdict
import math
import spec


@dataclass
class Evidence:
    tray_x_m: float
    tray_z_m: float
    tray_yaw_deg: float
    bail_deg: float
    left_position: int      # notch (1-4) holding the left finger, 0 if none
    right_position: int
    touching_child: bool    # tray, bail or a gripper touching the toddler
    hands_clear: bool
    stable: bool
    inside_workspace: bool
    intact: bool


class Scorer:
    def __init__(self):
        self.score = 0.
        self.dwell = 0
        self.invalid = False

    def update(self, e: Evidence):
        finite = all(math.isfinite(v) for v in asdict(e).values() if isinstance(v, float))
        dropped = e.tray_z_m < spec.DROP_Z
        # Touching the manikin (forcing the tray against it) zeros the trial for good.
        self.invalid |= (not finite) or (not e.inside_workspace) or (not e.intact) or dropped or e.touching_child
        if self.invalid:
            self.score = 0.
            self.dwell = 0
            return 0.
        latched = e.left_position if e.left_position and e.left_position == e.right_position else 0
        full = latched == spec.TARGET_POSITION and e.hands_clear and e.stable
        self.dwell = self.dwell + 1 if full else 0
        if self.dwell >= spec.DWELL_STEPS:
            self.score = 1.
        elif 1 < latched <= spec.TARGET_POSITION:
            self.score = .5
        elif e.tray_x_m >= spec.POSITION_X[0] + spec.RELEASE_TRAVEL:
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
