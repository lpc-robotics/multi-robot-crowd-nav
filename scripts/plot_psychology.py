#!/usr/bin/env python3
"""Standalone research plots from the CPU evidence, no live model changes."""
import argparse
import json
from pathlib import Path
import os
os.environ.setdefault('MPLCONFIGDIR','/tmp/arena-psychology-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    ap=argparse.ArgumentParser();ap.add_argument('cpu_directory');ap.add_argument('--output-dir',required=True);args=ap.parse_args()
    cpu=Path(args.cpu_directory);out=Path(args.output_dir);out.mkdir(parents=True,exist_ok=True)
    report=json.loads((cpu/'cpu.json').read_text())
    fig,axes=plt.subplots(1,2,figsize=(11,4),sharey=True)
    for ax,key,title in zip(axes,['frozen_default','frozen_enhanced'],['Recoverable weak propagation','Enhanced propagation']):
        curve=report[key]['curve'];t=[r['time_s'] for r in curve]
        for i,label in enumerate(['A: direct source','B: first hop','C: second hop']):
            ax.plot(t,[r['fear'][i] for r in curve],label=label)
        ax.axvline(50,color='gray',linestyle=':',label='Robot stimulus removed')
        ax.axhline(.08,color='gray',linestyle='--',alpha=.5);ax.set(xlabel='Simulation time (s)',title=title,ylim=(0,1))
        ax.grid(alpha=.2)
    axes[0].set_ylabel('Fear activation');axes[1].legend(fontsize=8);fig.tight_layout()
    fig.savefig(out/'propagation.png',dpi=180);fig.savefig(out/'propagation.svg');plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(11,7))
    for name,column in [('single',0),('threat',1)]:
        rows=[json.loads(line) for line in (cpu/(name+'_sis.jsonl')).open()]
        t=[r['time_s'] for r in rows]
        for q,color in [('curiosity','tab:orange'),('fear','tab:red')]:
            axes[0,column].plot(t,[r['states']['1']['variables'][q] for r in rows],color=color,label=q)
        modes={'REGULAR':0,'CURIOUS':1,'SCARED':2}
        axes[0,column].step(t,[modes[r['states']['1']['behavior']['mode']]/2 for r in rows],where='post',alpha=.5,label='Mode (scaled)')
        axes[0,column].set(title=name+' closed loop',xlabel='Released-motion time (s)',ylim=(0,1));axes[0,column].legend(fontsize=8)
        axes[1,column].plot([r['people']['1']['position'][0] for r in rows],[r['people']['1']['position'][1] for r in rows],label='Person')
        axes[1,column].plot([r['robots'][0][0] for r in rows],[r['robots'][0][1] for r in rows],label='Robot')
        axes[1,column].set(xlabel='Map x (m)',ylabel='Map y (m)');axes[1,column].legend(fontsize=8)
    for ax in axes.flat:ax.grid(alpha=.2)
    fig.tight_layout();fig.savefig(out/'single_motion.png',dpi=180);fig.savefig(out/'single_motion.svg');plt.close(fig)
    fig,ax=plt.subplots(figsize=(9,6))
    for suffix,style in [('noop','--'),('sis','-')]:
        rows=[json.loads(line) for line in (cpu/('six_'+suffix+'.jsonl')).open()]
        for i in rows[0]['people']:
            ax.plot([r['people'][i]['position'][0] for r in rows],[r['people'][i]['position'][1] for r in rows],style,label=f'{suffix} person {i}')
    ax.set(xlabel='Map x (m)',ylabel='Map y (m)',title='Six-person same-input closed-loop comparison')
    ax.legend(fontsize=7,ncol=2);ax.set_aspect('equal',adjustable='datalim');ax.grid(alpha=.2);fig.tight_layout()
    fig.savefig(out/'six_comparison.png',dpi=180);fig.savefig(out/'six_comparison.svg');plt.close(fig)


if __name__=='__main__':main()
