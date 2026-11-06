"""Plot full actual trial and committed interaction intervals; no cherry-picked success frames."""
import argparse,json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('raw_run');p.add_argument('report_dir');a=p.parse_args()
raw=Path(a.raw_run);out=Path(a.report_dir)
summary=json.loads((out/'summary.json').read_text());acceptance=json.loads((out/'acceptance.json').read_text())
if not acceptance['passed']:raise SystemExit('plot is reserved for passed complete trials')
people={};robot=[]
for line in (raw/'acceptance.actual.jsonl').open():
 d=json.loads(line)
 for name,v in d['people'].items():people.setdefault(name,[]).append((v['x'],v['y']))
trace=next(raw.rglob('psychology_*.jsonl'))
for n,line in enumerate(trace.open()):
 if n%8==0:
  r=json.loads(line)['robots']['-1'];robot.append((r['x'],r['y']))
fig,(ax,tl)=plt.subplots(1,2,figsize=(12,5),layout='constrained')
for name,points in people.items():ax.plot(*zip(*points),lw=1.2,label=name+' (Isaac actual)')
ax.plot(*zip(*robot),'k--',lw=1,label='robot_1 (observed odometry)')
ax.set(xlim=(6,18),ylim=(2,20),xlabel='x (m)',ylabel='y (m)',title='Full trial: observed trajectories',aspect='equal');ax.grid(alpha=.2);ax.legend(fontsize=8)
colors={'SURPRISED':'#b8860b','CURIOUS':'#1787b8','THREATENING':'#8e44ad','SCARED':'#d43c3c','REGULAR':'#aaa'}
for y,i in enumerate(('13','14')):
 for seg in summary['people'][i]['intervals']:
  start,end=seg['start_sim_s'],seg['end_sim_s']
  tl.broken_barh([(start,end-start)],(y-.3,.6),facecolors=colors[seg['mode']],alpha=1. if seg['phase']=='occupy' else .4)
 for ev in summary['people'][i]['events']:
  if ev['event']=='playful_occupied':tl.annotate('occupy: 8 s',(ev['sim_s'],y+.32),fontsize=9)
 tl.text(0,y-.48,f'actual at-target occupancy: {acceptance["actual_occupy_at_target_s"][i]:.2f} s',fontsize=8)
from matplotlib.patches import Patch
tl.legend(handles=[Patch(color=c,label=k,alpha=.6) for k,c in colors.items()],loc='upper right',fontsize=8)
tl.set(yticks=(0,1),yticklabels=('person_13','person_14'),xlabel='Simulation time (s)',ylim=(-.6,1.9),title='Committed modes; opaque purple = occupy')
fig.suptitle('Isolated playful showcase: sequential interaction, original curiosity model')
for ext in ('png','svg'):fig.savefig(out/f'playful_actual.{ext}',dpi=160)
