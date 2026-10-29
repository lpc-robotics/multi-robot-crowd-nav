#!/usr/bin/env python3
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/arena_multi_hunav'))
from arena_multi_hunav.data_v2 import export_data

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run_dir');p.add_argument('--output-dir',required=True)
    p.add_argument('--experiment-id',required=True);p.add_argument('--actual')
    a=p.parse_args();print(json.dumps(export_data(a.run_dir,a.output_dir,a.experiment_id,a.actual),indent=2))
