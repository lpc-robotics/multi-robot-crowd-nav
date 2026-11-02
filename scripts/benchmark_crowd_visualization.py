#!/usr/bin/env python3
"""Bounded CPU projection benchmark; not a GPU/live acceptance test."""
import argparse
import json
from pathlib import Path
import statistics
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/arena_crowd_visualization'))
from arena_crowd_visualization.model import project, arrows
p=argparse.ArgumentParser();p.add_argument('trace',type=Path);p.add_argument('--frames',type=int,default=1000)
p.add_argument('--output',type=Path,required=True);a=p.parse_args()
times=[];count=0
with a.trace.open() as stream:
    for line in stream:
        start=time.process_time()
        doc=json.loads(line);rows=project(doc)
        for row in rows:arrows(doc,row['id'])
        times.append((time.process_time()-start)*1000);count+=1
        if count>=a.frames:break
if not times:raise SystemExit('empty trace')
report=dict(scope='CPU JSON decode + all-person projections; no ROS/rendering',frames=count,
    mean_ms=statistics.mean(times),p95_ms=sorted(times)[int(.95*(len(times)-1))],
    one_core_percent_at_5hz=statistics.mean(times)*.5)
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report))
