#!/usr/bin/env python3
"""Matched blocking ablation: report changes, without presuming delay must increase."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path


def metrics(run):
    run=Path(run); meta=json.loads((run/'source_manifest.json').read_text())
    demo=json.loads((run/'psychology_demo.json').read_text())
    if not demo['passed']: raise ValueError('demonstration incomplete: '+str(run))
    stats={}; previous={}; interactions=[]
    for path in sorted(run.glob('psychology_*.jsonl')):
        for line in path.open():
            row=json.loads(line); t=row['input_stamp_ns']/1e9
            for person,state in row['states'].items():
                if state.get('event')!='playful_started':continue
                target=str(state.get('event_target_robot_id') or state['interaction_target'])
                name=row['robots'][target]['name'];event_ns=row['stamp_ns']
                keys=[f"round_{r['round']}:{name}" for r in demo['rounds']
                      if r['robots'][name]['sent_ns']<=event_ns<=r['robots'][name]['completed_ns']]
                interactions.append(dict(participant_id=person,target_robot=name,stamp_ns=event_ns,goal_keys=keys))
            for key,p in row['robots'].items():
                name=p['name']; lane=7. if name=='robot_1' else 17.
                old=previous.get(name); previous[name]=(t,p)
                if old is None or not 18<=p['x']<=28 or abs(p['y']-lane)>4: continue
                dt=t-old[0]
                if not 0<dt<=.026: raise ValueError('discontinuous robot record')
                s=stats.setdefault(name,dict(duration_s=0.,path_m=0.,speed_integral=0.,stopped_s=0.,maximum_lateral_deviation_m=0.))
                speed=math.hypot(p['vx'],p['vy']); s['duration_s']+=dt; s['speed_integral']+=speed*dt
                s['path_m']+=math.hypot(p['x']-old[1]['x'],p['y']-old[1]['y'])
                s['stopped_s']+=dt if speed<.05 else 0.
                s['maximum_lateral_deviation_m']=max(s['maximum_lateral_deviation_m'],abs(p['y']-lane))
    for s in stats.values(): s['mean_speed_mps']=s.pop('speed_integral')/s['duration_s']
    tasks={f"round_{r['round']}:{name}":(v['completed_ns']-v['sent_ns'])/1e9 for r in demo['rounds'] for name,v in r['robots'].items()}
    intervals={f"round_{r['round']}:{name}":dict(sent_ns=v['sent_ns'],completed_ns=v['completed_ns']) for r in demo['rounds'] for name,v in r['robots'].items()}
    return meta, dict(interactions=interactions,goal_intervals_ns=intervals,corridor=stats,goal_duration_sim_s=tasks,total_goal_duration_sim_s=sum(tasks.values()),
        demo_duration_sim_s=(demo['sim_stamp_ns']-min(v['sent_ns'] for r in demo['rounds'] for v in r['robots'].values()))/1e9,
        demo_wall_s=demo['wall_duration_s'])


def main():
    p=argparse.ArgumentParser();p.add_argument('active_run');p.add_argument('control_run');p.add_argument('--output',required=True);a=p.parse_args()
    active, am=metrics(a.active_run); control, cm=metrics(a.control_run)
    if cm['interactions']: raise ValueError('disabled control contains blocking episodes')
    ac=copy.deepcopy(active['resolved_individuals']);cc=copy.deepcopy(control['resolved_individuals'])
    for key in ac:
        if ac[key]['parameters'].pop('playful_enabled')!=1 or cc[key]['parameters'].pop('playful_enabled')!=0:
            raise ValueError('incorrect ablation switch')
    sa=copy.deepcopy(active['scenario']);sc=copy.deepcopy(control['scenario'])
    sa.pop('psychology_config');sc.pop('psychology_config')
    if ac!=cc or sa!=sc: raise ValueError('unmatched effective parameters/scenario')
    if active['provenance']['source_sha256']!=control['provenance']['source_sha256']:
        raise ValueError('source changed between ablation runs')
    deltas={name:{k:am['corridor'][name][k]-cm['corridor'][name][k] for k in am['corridor'][name]} for name in am['corridor'] if name in cm['corridor']}
    first_interaction=min((e['stamp_ns'] for e in am['interactions']),default=0)
    before=[k for k,v in am['goal_intervals_ns'].items() if v['completed_ns']<=first_interaction]
    report=dict(passed=True,scope='matched simulation ablation, no empirical prevalence or statistical significance claim',
        active=am,control=cm,delta_corridor=deltas,
        interpretation='descriptive paired simulation; same seed/configuration does not make asynchronous Nav2/Isaac a bitwise replay; pre-interaction differences prevent attributing the whole-demo delta solely to blocking',
        pre_interaction_goal_deltas_sim_s={k:am['goal_duration_sim_s'][k]-cm['goal_duration_sim_s'][k] for k in before},
        delta_goal_duration_sim_s={k:am['goal_duration_sim_s'][k]-cm['goal_duration_sim_s'][k] for k in am['goal_duration_sim_s']},
        interaction_navigation_legs=[dict(**episode,active_duration_sim_s={k:am['goal_duration_sim_s'][k] for k in episode['goal_keys']},
            control_duration_sim_s={k:cm['goal_duration_sim_s'][k] for k in episode['goal_keys']}) for episode in am['interactions']],
        delta_total_goal_duration_sim_s=am['total_goal_duration_sim_s']-cm['total_goal_duration_sim_s'],
        delta_demo_duration_sim_s=am['demo_duration_sim_s']-cm['demo_duration_sim_s'],
        analysis_script_sha256=hashlib.file_digest(Path(__file__).open('rb'),'sha256').hexdigest(),
        source_sha256=active['provenance']['source_sha256'])
    with Path(a.output).open('x') as f:json.dump(report,f,indent=2,allow_nan=False)
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
