#!/usr/bin/env python3
"""Evaluate a completed control run without rewriting its original probe report."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/arena_multi_hunav'))
from arena_multi_hunav import control_validation_v2


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('acceptance', type=Path)
    parser.add_argument('source_manifest', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = control_validation_v2.assess_control(
        json.loads(args.acceptance.read_text()), json.loads(args.source_manifest.read_text()))
    report['inputs'] = {str(p.resolve()): digest(p) for p in (args.acceptance, args.source_manifest)}
    report['processor_sha256'] = {Path(p).name: digest(p) for p in (__file__, control_validation_v2.__file__)}
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(report, indent=2))
    return int(not report['passed'])


if __name__ == '__main__':
    raise SystemExit(main())
