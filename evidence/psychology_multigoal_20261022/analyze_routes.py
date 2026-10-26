#!/usr/bin/env python3
"""Verify complete goal ordering and psychological task interruption from a saved trace."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/arena_multi_hunav'))
import yaml
from arena_multi_hunav.routes import compile_route
from arena_multi_hunav.route_tracking import RouteTracker


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    config = ROOT / 'config/hunav/multi_regular_8_psychology_multigoal.yaml'
    specs = yaml.safe_load(config.read_text())['hunav_loader']['ros__parameters']
    tracks = {str(specs[n]['id']): RouteTracker(compile_route(specs[n], person=n, world_bounds=(30, 23)),
                                             specs[n]['goal_radius']) for n in specs['agents']}
    modes = {i: Counter() for i in tracks}
    returns = Counter(); pauses = Counter(); previous = {}; peaks = {}; final = {}; failures = []
    count = 0; first = None; last = None
    try:
        with args.trace.open() as stream:
            for line in stream:
                data = json.loads(line); count += 1
                if first is None: first = data['stamp_ns']
                last = data['stamp_ns']
                if set(data['agents']) != set(tracks): raise ValueError('agent identity mismatch')
                for i, tracker in tracks.items():
                    agent = data['agents'][i]; person = data['people'][i]; goals = data['task_goals'][i]
                    tracker.observe(last, person['x'], person['y'], [tuple(g) for g in goals])
                    behavior = agent['behavior']; modes[i][behavior['mode']] += 1
                    if not agent['intent']['allow_task_progress']:
                        pauses[i] += 1
                        if i in previous and previous[i][0] != goals:
                            raise ValueError('paused task advanced: ' + i)
                    if i in previous:
                        returns[i] += previous[i][1] and not behavior['returning_to_task']
                    previous[i] = (goals, behavior['returning_to_task'])
                    peaks.setdefault(i, {'curiosity': 0., 'fear': 0.})
                    for q, value in agent['new'].items(): peaks[i][q] = max(peaks[i][q], value)
                    final[i] = agent['new']
        if not count: failures.append('empty trace')
        for i, tracker in tracks.items():
            if tracker.report()['completed_cycles'] < 2: failures.append('fewer than two full cycles: ' + i)
        if not any('CURIOUS' in seen for seen in modes.values()): failures.append('no curiosity behavior')
        if not any(returns.values()): failures.append('no completed task return')
    except (ValueError, KeyError, TypeError) as error:
        failures.append(str(error))
    result = {'passed': not failures, 'scope': 'committed full trace; actual geometry checked separately',
              'samples': count, 'first_stamp_ns': first, 'last_stamp_ns': last,
              'trace': str(args.trace.resolve()), 'trace_sha256': hashlib.file_digest(args.trace.open('rb'), 'sha256').hexdigest(),
              'config_sha256': hashlib.sha256(config.read_bytes()).hexdigest(),
              'modes': modes, 'peak_activation': peaks, 'final_activation': final,
              'paused_steps': dict(pauses), 'completed_task_returns': dict(returns),
              'routes': {i: t.report() for i, t in tracks.items()}, 'failures': failures}
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'routes'}, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
