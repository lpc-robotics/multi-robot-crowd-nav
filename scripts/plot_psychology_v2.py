#!/usr/bin/env python3
"""Standalone figures from versioned CSV exports; latent activations are not human labels."""
import argparse
import csv
import json
import os
from pathlib import Path
os.environ.setdefault('MPLCONFIGDIR','/tmp/arena-psychology-v2-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

MODES={'REGULAR':0,'CURIOUS':1,'SCARED':2,'SURPRISED':3,'THREATENING':4}
COLORS=['#8c8c8c','#e69f00','#d73027','#56b4e9','#984ea3']


def main():
    p=argparse.ArgumentParser();p.add_argument('export_dir');p.add_argument('--output-dir',required=True)
    p.add_argument('--agents',nargs='+',default=['13','14']);a=p.parse_args()
    root=Path(a.export_dir);out=Path(a.output_dir);out.mkdir(parents=True,exist_ok=True)
    manifest=json.loads((root/'manifest.json').read_text());meta=manifest['provenance']
    names=[meta['participants'].get(i,i) for i in a.agents]
    with (root/'events.csv').open() as f:events=list(csv.DictReader(f))
    windows={}
    for name in names:
        spans=[e for e in events if e['participant_id']==name and e['kind']=='mode']
        selected=[e for e in events if e['participant_id']==name and e['kind']=='transition']
        if not spans:raise ValueError('No recorded participant: '+name)
        windows[name]=(max(0.,min(int(e['start_ns'])/1e9 for e in (selected or spans))-3),
                       max(int(e['end_ns'])/1e9 for e in (selected or spans))+5)
    latent={n:[] for n in names};tracks={};last={}
    with (root/'latents.csv').open() as f:
        for row in csv.DictReader(f):
            name=row['participant_id'];t=int(row['stamp_ns'])/1e9
            if name in latent and windows[name][0]<=t<=windows[name][1] and t-last.get(name,-1e9)>=.1:
                latent[name].append(row);last[name]=t
    with (root/'observations.csv').open() as f:
        for row in csv.DictReader(f):
            name=row['participant_id'];t=int(row['stamp_ns'])/1e9
            if name not in names and row['entity_type']!='robot':continue
            key=(name,row['source'])
            if t-last.get(key,-1e9)<.1:continue
            tracks.setdefault(key,[]).append((t,float(row['x']),float(row['y'])));last[key]=t
    fig,axes=plt.subplots(3,len(names),figsize=(6*len(names),10),squeeze=False)
    for col,name in enumerate(names):
        rows=latent[name];t=[int(r['stamp_ns'])/1e9 for r in rows]
        for key in ('curiosity','fear','surprise'):axes[0,col].plot(t,[float(r[key]) for r in rows],label=key)
        axes[0,col].set(title=name+' : unfitted simulator activations',ylim=(0,1),ylabel='Activation');axes[0,col].legend()
        axes[1,col].step(t,[MODES[r['mode']] for r in rows],where='post')
        axes[1,col].set(yticks=list(MODES.values()),yticklabels=list(MODES),xlabel='Simulation time (s)')
        actual=(name,'isaac_actual') in tracks
        for (who,source),points in tracks.items():
            if who==name and source!=('isaac_actual' if actual else 'core_committed'):continue
            if who!=name and source!='robot_observation':continue
            selected=[v for v in points if windows[name][0]<=v[0]<=windows[name][1]]
            if selected:axes[2,col].plot([v[1] for v in selected],[v[2] for v in selected],label=who+' ('+source+')')
        axes[2,col].set(xlabel='Map x (m)',ylabel='Map y (m)',title='Selected interaction window')
        axes[2,col].set_aspect('equal',adjustable='datalim');axes[2,col].legend(fontsize=8)
    for ax in axes.flat:ax.grid(alpha=.2)
    fig.tight_layout()
    for ext in ('png','svg'):fig.savefig(out/('interactions.'+ext),dpi=160)
    plt.close(fig)
    people=sorted(meta['participants'],key=int);fig,ax=plt.subplots(figsize=(13,max(3,.45*len(people))))
    for index,i in enumerate(people):
        name=meta['participants'][i]
        for e in events:
            if e['participant_id']==name and e['kind']=='mode':
                start=int(e['start_ns'])/1e9;length=(int(e['end_ns'])-int(e['start_ns']))/1e9
                ax.broken_barh([(start,length)],(index-.35,.7),facecolors=COLORS[MODES[e['label']]])
    ax.set(yticks=range(len(people)),yticklabels=[meta['participants'][i]+' / '+meta['resolved_individuals'][i]['profile'] for i in people],
           xlabel='Simulation time (s)',title='All recorded mode intervals, including non-triggering participants')
    ax.legend(handles=[Patch(color=COLORS[v],label=k) for k,v in MODES.items()],ncol=5,fontsize=8)
    fig.tight_layout()
    for ext in ('png','svg'):fig.savefig(out/('cohort_modes.'+ext),dpi=160)
    plt.close(fig)


if __name__=='__main__':main()
