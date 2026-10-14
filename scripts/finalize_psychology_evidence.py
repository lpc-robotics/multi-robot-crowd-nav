#!/usr/bin/env python3
"""Separate psychology acceptance; cannot reuse the noop release gate."""
import argparse
import hashlib
import json
from pathlib import Path
from multigoal_evidence import provenance, ROOT


def main():
    ap=argparse.ArgumentParser();ap.add_argument('evidence_directory');args=ap.parse_args();root=Path(args.evidence_directory)
    names=['validation/offline.json','validation/cpu/cpu.json','validation/faults/faults.json',
           'validation/isaac_two_people.json','validation/isaac_six_people_30min.json','validation/psychology_demo.json']
    failures=[];records={};current=provenance(ROOT/'config/scenarios/two_robots_multi_sfm_6_multigoal_psychology_nav2.yaml',ROOT/'config/world/arena.yaml')
    for name in names:
        path=root/name
        if not path.is_file():failures.append('missing:'+name);continue
        report=json.loads(path.read_text());records[name]={'sha256':hashlib.file_digest(path.open('rb'),'sha256').hexdigest()}
        if not report.get('passed'):failures.append('failed:'+name)
        if 'provenance' in report and report['provenance']['source_sha256']!=current['source_sha256']:
            failures.append('source mismatch:'+name)
    long=root/'validation/isaac_six_people_30min.json'
    if long.is_file() and json.loads(long.read_text()).get('wall_duration_s',0)<1800:failures.append('six-person endurance below 30 minutes')
    acceptance={'schema_version':1,'passed':not failures,'scope':'sis_cf_v1 development, no formal release or human-data validation',
                'source_commit':current['source_commit'],'source_sha256':current['source_sha256'],
                'reports':records,'failures':failures}
    (root/'acceptance.json').write_text(json.dumps(acceptance,indent=2)+'\n');print(json.dumps(acceptance,indent=2))
    return 0 if acceptance['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
