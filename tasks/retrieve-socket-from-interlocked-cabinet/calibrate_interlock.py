"""Calibration only: native forces, no task-policy success claim."""
from pathlib import Path
import argparse,json,fcntl
import numpy as np
lock=open('/tmp/robocurve-shared-gpu.lock','a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
import arena_runtime,spec
parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
r=args.output;r.mkdir(parents=True,exist_ok=False)
w=arena_runtime.CabinetWorld(r/'scene')
ids=np.array([0],dtype=np.uint32)
report={'calibration_only':True,'controlled_changes':'Screwdriver removed between obstruction and closure tests; not a policy demo. All tested drawer motion comes from applied force, not pose setters.','cases':[]}
def push(name,force,seconds,label):
    v=w.prop_views['/World/'+name];samples=[]
    for k in range(round(seconds*spec.PHYSICS_HZ)):
        v.apply_forces_and_torques_at_position(np.array([[force,0,0]],np.float32),None,None,ids,True)
        w.base.sim.step(render=False)
        if k%48==0:
            samples.append({n:w.prop_views['/World/'+n].get_transforms()[0].tolist() for n in w.objects})
    report['cases'].append({'name':label,'force_n':force,'seconds':seconds,'samples':samples})
    print(label,{n:round(float(spec.FRONT-w.prop_views['/World/'+n].get_transforms()[0,0]),5) for n in ['lower','upper']},flush=True)
push('upper',-40,1.,'upper_blocked_with_lower_open')
push('lower',40,1.,'lower_blocked_by_screwdriver')
v=w.prop_views['/World/screwdriver'];v.set_transforms(np.array([[.3,.43,.503,0,0,0,1]],np.float32),ids);v.set_velocities(np.zeros((1,6),np.float32),ids)
push('upper',10,.5,'upper_return_to_closed')
push('lower',20,2.,'lower_closes_after_tool_removed')
push('upper',-40,2.,'upper_opens_after_lower_closes')
push('lower',-40,1.,'lower_blocked_with_upper_open')
summary={c['name']:{'lower_open_m':spec.FRONT-c['samples'][-1]['lower'][0],'upper_open_m':spec.FRONT-c['samples'][-1]['upper'][0]} for c in report['cases']}
assert abs(summary['upper_blocked_with_lower_open']['upper_open_m'])<.005
assert summary['lower_blocked_by_screwdriver']['lower_open_m']>.030
assert abs(summary['lower_closes_after_tool_removed']['lower_open_m'])<.005
assert summary['upper_opens_after_lower_closes']['upper_open_m']>.200
assert summary['lower_blocked_with_upper_open']['lower_open_m']<.030
report['passed']=True;report['summary']=summary
(r/'interlock.json').write_text(json.dumps(report,indent=2)+'\n')
w.close();arena_runtime.APP.close()
