#!/usr/bin/env python3
"""Repeatable profiled 14-person transactions, not a real-time acceptance claim."""
import argparse
import cProfile
import json
from pathlib import Path
import pstats
import sys
import time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/arena_multi_hunav'))
from arena_multi_hunav.multi_adapter import load_people,entity
from arena_multi_hunav.psychology_config import load_psychology_config
from arena_multi_hunav.psychology_v2 import SocialPsychologyV2
from arena_multi_hunav.psychology import prepare_step,make_context,Entity

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--steps',type=int,default=1000);a=p.parse_args()
    m=SocialPsychologyV2();agents,_=load_people(ROOT/'config/hunav/multi_14_psychology_v2_multigoal.yaml')
    people=[entity(p) for p in agents.agents]
    m.initialize(load_psychology_config(ROOT/'config/psychology/sis_cf_v2.yaml').document(),tuple(p.id for p in people),11)
    states=m.initial_states();robots=[Entity(-1,'robot_1',18,7,.26,0,.35),Entity(-2,'robot_2',3,7,0,0,.35)]
    profiler=cProfile.Profile();profiler.enable();start=time.perf_counter()
    for k in range(a.steps):
        proposal,snapshot=prepare_step(m,make_context(k*25_000_000,.025,people,robots),states)
        m.restore(snapshot);states=proposal.states
    elapsed=time.perf_counter()-start;profiler.disable()
    with Path(a.output).open('x') as stream:
        stream.write(json.dumps(dict(steps=a.steps,profiled_wall_s=elapsed,snapshot_bytes=len(json.dumps(m.snapshot()))))+'\n')
        pstats.Stats(profiler,stream=stream).sort_stats('cumtime').print_stats(18)
    print(json.dumps(dict(steps=a.steps,profiled_wall_s=elapsed)))
