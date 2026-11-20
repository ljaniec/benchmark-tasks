"""Physical insertion, independent retention proof, and released placement."""
from dataclasses import dataclass
import math
import spec

@dataclass
class Evidence:
    entered: bool
    seated: bool
    collet_clear: bool
    assembly_raised: bool
    nut_tilt_deg: float
    parked: bool
    hands_clear: bool
    chassis_tilt_deg: float
    inside_workspace: bool
    intact: bool
    shortcut: bool

class Scorer:
    def __init__(self):
        self.score=0.;self.dwell=0;self.invalid=False
        self.proof_steps=0;self.verified=False

    def update(self,e):
        if not all(math.isfinite(v) for v in [e.nut_tilt_deg,e.chassis_tilt_deg]):self.invalid=True
        self.invalid |= e.chassis_tilt_deg>15 or not e.inside_workspace or not e.intact or e.shortcut
        if self.invalid:self.score=0.;self.dwell=0;return 0.
        if not e.seated or not e.collet_clear:
            self.proof_steps=0;self.verified=False
        test=e.seated and e.collet_clear and e.assembly_raised and e.nut_tilt_deg>=110.
        self.proof_steps=self.proof_steps+1 if test else 0
        if self.proof_steps>=spec.DWELL_STEPS:self.verified=True
        complete=self.verified and e.seated and e.collet_clear and e.parked and e.hands_clear
        self.dwell=self.dwell+1 if complete else 0
        self.score=1. if self.dwell>=spec.DWELL_STEPS else .5 if e.seated and e.collet_clear else .25 if e.entered else 0.
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
