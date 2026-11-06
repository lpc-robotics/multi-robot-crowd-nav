"""Offline committed behavior/force intervals; actual metrics stay in acceptance.json."""
import collections
import hashlib
import json
from pathlib import Path
import sys
run = Path(sys.argv[1])
paths = sorted(run.rglob('psychology_*.jsonl'))
if not paths: raise SystemExit('no psychology trace')
path = paths[0]
summary = dict(source=str(path), source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(), people={})
last = {}
for line in path.open():
    d = json.loads(line)
    if 'states' not in d: continue
    t = d['stamp_ns']/1e9
    for i, s in d['states'].items():
        p = summary['people'].setdefault(i, dict(modes={}, phases={}, events=[], intervals=[], max_latents={'curiosity':0.,'fear':0.,'surprise':0.}, max_robot_social_force=0., max_robot_static_force=0.))
        mode = s['behavior']['mode']; phase = s['phase']
        for k, v in [('modes',mode),('phases',phase)]:p[k][v]=p[k].get(v,0)+1
        if s['event']: p['events'].append(dict(sim_s=t,event=s['event'],reason=s['exit_reason'],outcome=s.get('playful_outcome','')))
        key=(mode,phase)
        if last.get(i) != key:
            if p['intervals']: p['intervals'][-1]['end_sim_s']=t
            p['intervals'].append(dict(start_sim_s=t,end_sim_s=t,mode=mode,phase=phase))
            last[i]=key
        else:p['intervals'][-1]['end_sim_s']=t
        for k in ('curiosity','fear'):p['max_latents'][k]=max(p['max_latents'][k],s['variables'][k])
        p['max_latents']['surprise']=max(p['max_latents']['surprise'],s['surprise'])
    for m in d['motion']:
        p=summary['people'][str(m['agent_id'])]
        for k in ('robot_social_force','robot_static_force'):
            f=m[k];p['max_'+k]=max(p['max_'+k],sum(x*x for x in f)**.5)
print(json.dumps(summary,indent=2))
