import json
from pathlib import Path
import sys
p=next(Path(sys.argv[1]).rglob('psychology_*.jsonl'))
with p.open('rb') as f:
    f.seek(max(0,p.stat().st_size-65536));lines=f.read().splitlines()
if lines:
    try:d=json.loads(lines[-1])
    except json.JSONDecodeError:d=json.loads(lines[-2])
    print('sim_s',round(d['stamp_ns']/1e9,3),'people',{i:(s['behavior']['mode'],s['phase']) for i,s in d['states'].items()},'robots',{i:(round(r['x'],2),round(r['y'],2)) for i,r in d['robots'].items()})
else:print('waiting for simulation')
