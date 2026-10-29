import bisect,csv,json,math,sys
from pathlib import Path
root=Path(sys.argv[1]);out=Path(sys.argv[2]);episodes={};robots={};completed=set()
with (root/'events.csv').open() as f:
 for e in csv.DictReader(f):
  if e['kind']=='transition' and e['label']=='surprise_completed':completed.add((e['participant_id'],int(e['start_ns']),'robot_'+str(-int(e['target_robot_id']))))
  if e['kind']=='mode' and e['label']=='SURPRISED':
   episodes.setdefault(e['participant_id'],[]).append(dict(start_ns=int(e['start_ns']),end_ns=int(e['end_ns']),target='robot_'+str(-int(e['target_robot_id'])),exit_reason=e['reason'],max_aligned_s=0.,aligned_s=0.,last_ns=None))
with (root/'observations.csv').open() as f:
 for r in csv.DictReader(f):
  if r['source']=='robot_observation':robots.setdefault(r['participant_id'],[]).append((int(r['stamp_ns']),float(r['x']),float(r['y'])))
times={k:[v[0] for v in values] for k,values in robots.items()}
with (root/'observations.csv').open() as f:
 for r in csv.DictReader(f):
  if r['source']!='isaac_actual':continue
  t=int(r['stamp_ns']);who=r['participant_id']
  for e in episodes.get(who,[]):
   if not e['start_ns']<=t<e['end_ns']:continue
   index=bisect.bisect_right(times[e['target']],t-25_000_000)-1
   if index<0:continue
   rt,x,y=robots[e['target']][index];dt=0 if e['last_ns'] is None else (t-e['last_ns'])/1e9;e['last_ns']=t
   angle=abs(math.remainder(math.atan2(y-float(r['y']),x-float(r['x']))-float(r['body_yaw']),2*math.pi))
   aligned=math.hypot(float(r['vx']),float(r['vy']))<=.05 and angle<=math.radians(15) and 0<=t-rt<=250_000_000
   e['aligned_s']=e['aligned_s']+dt if aligned and 0<dt<=.15 else 0.
   e['max_aligned_s']=max(e['max_aligned_s'],e['aligned_s'])
for who,values in episodes.items():
 for e in values:
  e['normally_completed']=(who,e['end_ns'],e['target']) in completed
  e.pop('last_ns');e.pop('aligned_s');e['meets_one_second']=e['max_aligned_s']>=1.
report=dict(scope='all exported SURPRISED episodes, actual pose/velocity/yaw with prior robot timeline sample; 25ms core input offset',episodes=episodes,
 completed_episode_count=len(completed),
 passed=bool(completed) and all(e['meets_one_second'] for v in episodes.values() for e in v if e['normally_completed']))
out.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
