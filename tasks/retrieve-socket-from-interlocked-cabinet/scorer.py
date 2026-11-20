"""Ruler/photo-equivalent current-state scoring and permanent damage gates."""
from dataclasses import dataclass,asdict
import math
import spec

@dataclass
class Evidence:
    lower_open_m:float
    upper_open_m:float
    socket_parked:bool
    hands_clear:bool
    stable:bool
    inside_workspace:bool
    intact:bool
    chassis_tilt_deg:float

class Scorer:
    def __init__(self):self.score=0.;self.dwell=0;self.invalid=False
    def update(self,e):
        self.invalid|=not all(math.isfinite(v) for v in asdict(e).values()) or not e.inside_workspace or not e.intact or abs(e.chassis_tilt_deg)>15
        if self.invalid:self.score=0.;self.dwell=0;return 0.
        complete=e.socket_parked and e.hands_clear and e.stable
        self.dwell=self.dwell+1 if complete else 0
        self.score=1. if self.dwell>=spec.DWELL_STEPS else .5 if e.upper_open_m>=.200 else .25 if e.lower_open_m<=.005 else 0.
        return self.score

def regrade(path):
    import json
    from pathlib import Path
    s=Scorer();rows=[json.loads(x) for x in Path(path).read_text().splitlines()]
    for r in rows:
        assert s.update(Evidence(**r['evidence']))==r['score'] and s.dwell==r['dwell'] and s.invalid==r['invalid']
    assert rows
    return dict(steps=len(rows),terminal_score=s.score,peak_score=max(r['score'] for r in rows),dwell=s.dwell,invalid=s.invalid,recorded_scores_match=True)

if __name__=='__main__':
    import argparse,json
    p=argparse.ArgumentParser();p.add_argument('trace');print(json.dumps(regrade(p.parse_args().trace),indent=2))
