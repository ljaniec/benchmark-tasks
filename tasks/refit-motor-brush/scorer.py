"""Practical motor-brush refitting rubric, with invalid-event gating."""
from dataclasses import dataclass
import math
import spec
@dataclass
class Evidence:
    entered: bool
    seated: bool
    pressing: bool
    hands_clear: bool
    chassis_tilt_deg: float
    inside_workspace: bool
    intact: bool
    shortcut: bool
class Scorer:
    def __init__(self):self.score=0.;self.dwell=0;self.invalid=False
    def update(self,e):
        self.invalid |= not math.isfinite(e.chassis_tilt_deg) or e.chassis_tilt_deg>15 or not e.inside_workspace or not e.intact or e.shortcut
        if self.invalid:self.score=0.;self.dwell=0;return 0.
        full=e.entered and e.seated and e.pressing and e.hands_clear
        self.dwell=self.dwell+1 if full else 0
        self.score=1. if self.dwell>=spec.DWELL_STEPS else .5 if e.entered and e.seated else .25 if e.entered else 0.
        return self.score

def regrade(path):
    import json
    from pathlib import Path
    s=Scorer();n=0;peak=0.
    for line in Path(path).read_text().splitlines():
        row=json.loads(line);score=s.update(Evidence(**row['evidence']))
        if (score,s.dwell,s.invalid)!=(row['score'],row['dwell'],row['invalid']):raise ValueError('Recorded grade mismatch')
        n+=1;peak=max(peak,score)
    if not n:raise ValueError('Empty trace')
    return {'steps':n,'terminal_score':s.score,'peak_score':peak,'dwell':s.dwell,'invalid':s.invalid,'recorded_scores_match':True}
if __name__=='__main__':
    import argparse,json
    p=argparse.ArgumentParser();p.add_argument('trace');print(json.dumps(regrade(p.parse_args().trace),indent=2))
