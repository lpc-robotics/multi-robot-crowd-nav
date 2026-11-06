"""Additional showcase gates on top of the existing actual V2 acceptance."""
import json
from pathlib import Path
import sys
root=Path(sys.argv[1]);out=Path(sys.argv[2]);reports={};fail=[]
for name in ('immune','static','playful'):
    p=root/name
    a=json.loads((p/'acceptance.json').read_text());s=json.loads((p/'summary.json').read_text())
    if not a['passed']:fail.append(name+': base actual acceptance failed')
    if name!='playful':
        x=s['people']['1']
        if any(x['max_latents'].values()) or set(x['modes'])!={'REGULAR'}:fail.append(name+': psychological response')
        expected='robot_social_force' if name=='immune' else 'robot_static_force'
        excluded='robot_static_force' if name=='immune' else 'robot_social_force'
        if x['max_'+expected]<=.001 or x['max_'+excluded]>1e-9:fail.append(name+': response force not demonstrated')
        if a['minimum_robot_gap_m']>2.:fail.append(name+': no close exposure')
    else:
        windows=[]
        for i in ('13','14'):
            if a['actual_occupy_at_target_s'].get(i,0)<5.:fail.append('playful: insufficient actual occupancy '+i)
            intervals=[(x['start_sim_s'],x['end_sim_s']) for x in s['people'][i]['intervals'] if x['mode']=='THREATENING']
            windows.append(intervals)
        if any(max(a0,b0)<min(a1,b1) for a0,a1 in windows[0] for b0,b1 in windows[1]):fail.append('playful: overlapping actor interactions')
    reports[name]={k:a[k] for k in ('wall_s','sim_s','minimum_robot_gap_m','minimum_human_gap_m','max_display_error_m','actual_occupy_at_target_s','modes')}
result=dict(passed=not fail,failures=fail,reports=reports,scope='full actual V2 gates plus zero latents/observed forces and >=5s actual occupancy per playful actor; no population or child-behavior identification')
with out.open('x') as f:json.dump(result,f,indent=2);f.write('\n')
print(json.dumps(result,indent=2));raise SystemExit(int(not result['passed']))
