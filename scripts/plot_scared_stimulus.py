#!/usr/bin/env python3
"""Plot full fear experiments, including recovery and non-triggering controls."""
import argparse
import csv
import json
import hashlib
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR','/tmp/arena-scared-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

MODES = {'REGULAR':0,'CURIOUS':1,'SCARED':2,'SURPRISED':3,'THREATENING':4}


def read_experiment(directory, participant):
    root = Path(directory)
    with (root/'latents.csv').open() as f:
        rows = [r for r in csv.DictReader(f) if r['participant_id']==participant]
    if not rows:
        raise ValueError('missing simulator participant: '+participant)
    start = int(rows[0]['stamp_ns'])/1e9
    sampled = []
    previous = -1e9
    for row in rows:
        t = int(row['stamp_ns'])/1e9-start
        if t-previous >= .1:
            sampled.append((t,row)); previous=t
    tracks = {}
    seen = {}
    with (root/'observations.csv').open() as f:
        for row in csv.DictReader(f):
            if row['participant_id']!=participant and row['entity_type']!='robot':
                continue
            if row['participant_id']==participant and row['source']!='isaac_actual':
                continue
            key = row['participant_id']+' / '+row['source']
            t = int(row['stamp_ns'])/1e9-start
            if t-seen.get(key,-1e9) < .1:
                continue
            tracks.setdefault(key,[]).append((float(row['x']),float(row['y'])))
            seen[key]=t
    return sampled,tracks


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('exports',nargs='+')
    p.add_argument('--labels',nargs='+')
    p.add_argument('--participant',default='person_1')
    p.add_argument('--output-dir',required=True)
    a = p.parse_args()
    labels = a.labels or [Path(v).name for v in a.exports]
    if len(labels)!=len(a.exports):
        p.error('one label required per export')
    out = Path(a.output_dir); out.mkdir(parents=True,exist_ok=False)
    fig,axes = plt.subplots(3,len(a.exports),figsize=(6*len(a.exports),9),squeeze=False,
                           gridspec_kw={'height_ratios':[1,1,.65]})
    map_points = []
    for col,(directory,label) in enumerate(zip(a.exports,labels)):
        rows,tracks = read_experiment(directory,a.participant)
        times = [t for t,_ in rows]
        for key,color in [('fear','#d73027'),('curiosity','#e69f00'),('surprise','#56b4e9')]:
            axes[0,col].plot(times,[float(r[key]) for _,r in rows],label=key,color=color)
        axes[0,col].axhline(.65,color='#d73027',ls='--',lw=.8,label='fear_on = 0.65')
        axes[0,col].axhline(.35,color='#888888',ls=':',lw=.8,label='fear_off = 0.35')
        axes[0,col].set(title=label+'\nUnfitted simulator activations',ylim=(0,1),ylabel='Activation')
        axes[0,col].legend(fontsize=8,ncol=2)
        axes[1,col].step(times,[MODES[r['mode']] for _,r in rows],where='post',color='#333333')
        axes[1,col].set(yticks=list(MODES.values()),yticklabels=list(MODES),xlabel='Simulation seconds since first committed sample')
        for name,points in tracks.items():
            map_points.extend(points)
            line, = axes[2,col].plot([v[0] for v in points],[v[1] for v in points],label=name,lw=1.)
            axes[2,col].scatter(*points[0],marker='o',color=line.get_color(),s=20)
            axes[2,col].scatter(*points[-1],marker='x',color=line.get_color(),s=30)
        axes[2,col].set(xlabel='Map x (m)',ylabel='Map y (m)',title='Actual human / observed robot paths\nCircle = start; x = end')
        axes[2,col].set_aspect('equal',adjustable='box')
        axes[2,col].legend(fontsize=8)
        for row in range(3):
            axes[row,col].grid(alpha=.2)
    if map_points:
        xs,ys = zip(*map_points)
        pad = max(1.,(max(xs)-min(xs))*.08)
        for col in range(len(a.exports)):
            axes[2,col].set(xlim=(min(xs)-pad,max(xs)+pad),ylim=(min(ys)-pad,max(ys)+pad))
    fig.suptitle(a.participant+' : full experimental windows, including recovery',fontsize=13)
    fig.tight_layout()
    for ext in ('png','svg'):
        fig.savefig(out/('fear_comparison.'+ext),dpi=170)
    plt.close(fig)
    (out/'plot_inputs.json').write_text(json.dumps(dict(exports=[str(Path(v).resolve()) for v in a.exports],
        input_manifest_sha256={str(Path(v).resolve()):hashlib.sha256((Path(v)/'manifest.json').read_bytes()).hexdigest() for v in a.exports},
        processor_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        labels=labels,participant=a.participant,semantics='simulator latents are not human emotion measurements'),indent=2)+'\n')


if __name__=='__main__':
    main()
