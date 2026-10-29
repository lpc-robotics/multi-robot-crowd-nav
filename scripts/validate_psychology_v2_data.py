#!/usr/bin/env python3
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/arena_multi_hunav'))
from arena_multi_hunav.data_v2 import validate_data

if __name__=='__main__': print(json.dumps(validate_data(sys.argv[1]),indent=2))
