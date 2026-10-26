#!/usr/bin/env python3
"""Stream committed Isaac-run diagnostics into an audit summary and static figure.

Geometry here is the core's committed geometry; the independent runtime report
is the evidence for actual Character Graph clearance and display errors.
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/arena-psychology-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('trace')
    ap.add_argument('--output-dir', required=True)
    args = ap.parse_args()
    path = Path(args.trace); out = Path(args.output_dir); out.mkdir(parents=True, exist_ok=True)
    rows = []; modes = defaultdict(Counter); rotations = Counter(); returns = Counter()
    avoids = Counter(); infeasible = Counter(); previous = {}; edges = set(); first = None; last = None
    count = 0; maximum_step_gap = 0; last_step = None
    with path.open() as stream:
        for line in stream:
            data = json.loads(line); count += 1
            if first is None:
                first = data['stamp_ns']
            last = data['stamp_ns']
            if last_step is not None:
                maximum_step_gap = max(maximum_step_gap, data['step']-last_step)
            last_step = data['step']
            for i, a in data['agents'].items():
                modes[i][a['behavior']['mode']] += 1
                avoids[i] += bool(a['avoidance_conflicts'])
                infeasible[i] += not a['avoidance_feasible']
                if i in previous:
                    goals, returning = previous[i]
                    rotations[i] += goals != data['task_goals'][i]
                    returns[i] += returning and not a['behavior']['returning_to_task']
                previous[i] = (data['task_goals'][i], a['behavior']['returning_to_task'])
            edges.update((e['source_id'], e['target_id']) for e in data['edges'])
            if count % 20 != 1:
                continue
            people = data['people']; robots = data['robots']
            human_gap = min(math.hypot(a['x']-b['x'], a['y']-b['y'])-a['radius']-b['radius']
                            for i, a in people.items() for j, b in people.items() if i < j)
            robot_gap = min(math.hypot(a['x']-b['x'], a['y']-b['y'])-a['radius']-b['radius']
                            for a in people.values() for b in robots.values())
            rows.append({'time': (last-first)/1e9, 'agents': data['agents'], 'people': people,
                         'robots': robots, 'human_gap': human_gap, 'robot_gap': robot_gap,
                         'edges': len(data['edges'])})
    summary = {'scope': 'committed model diagnostics; actual display geometry is in actual.json',
               'trace': str(path.resolve()), 'trace_sha256': hashlib.file_digest(path.open('rb'), 'sha256').hexdigest(),
               'committed_samples': count, 'first_stamp_ns': first, 'last_stamp_ns': last,
               'maximum_step_increment': maximum_step_gap, 'mode_samples': dict(modes),
               'task_queue_advances': dict(rotations), 'completed_task_returns': dict(returns),
               'avoidance_steps': dict(avoids), 'infeasible_avoidance_steps': dict(infeasible),
               'observed_directed_edges': sorted(edges)}
    (out/'actual_trace_summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    colors = plt.get_cmap('tab10'); time_axis = [r['time'] for r in rows]
    for index, i in enumerate(sorted(rows[0]['people'], key=int)):
        color = colors(index)
        axes[0, 0].plot(time_axis, [r['agents'][i]['new']['curiosity'] for r in rows], color=color, label=f'Person {i}')
        axes[0, 1].plot(time_axis, [r['agents'][i]['new']['fear'] for r in rows], color=color)
        axes[1, 0].plot([r['people'][i]['x'] for r in rows], [r['people'][i]['y'] for r in rows], color=color)
    for i in sorted(rows[0]['robots']):
        axes[1, 0].plot([r['robots'][i]['x'] for r in rows], [r['robots'][i]['y'] for r in rows], '--', label=f'Robot {-int(i)}')
    axes[0, 0].set(title='Curiosity activation', xlabel='Simulation time since first commit (s)', ylim=(0, 1))
    axes[0, 0].legend(ncol=3, fontsize=8)
    axes[0, 1].set(title='Fear activation', xlabel='Simulation time since first commit (s)', ylim=(0, 1))
    axes[1, 0].set(title='Committed motion during actual Nav2 run', xlabel='Map x (m)', ylabel='Map y (m)')
    axes[1, 0].set_aspect('equal', adjustable='datalim'); axes[1, 0].legend(fontsize=8)
    axes[1, 1].plot(time_axis, [r['edges'] for r in rows], label='Visible directed edges')
    axes[1, 1].set(title='Changing psychological contact graph', xlabel='Simulation time since first commit (s)', ylabel='Edge count')
    for ax in axes.flat:
        ax.grid(alpha=.2)
    fig.tight_layout()
    for suffix in ('png', 'svg'):
        path_out = out/f'actual_closed_loop.{suffix}'
        fig.savefig(path_out, dpi=180)
        if suffix == 'svg':
            path_out.write_text('\n'.join(line.rstrip() for line in path_out.read_text().splitlines())+'\n')
    plt.close(fig)
    print(json.dumps({k: v for k, v in summary.items() if k != 'observed_directed_edges'}, indent=2))


if __name__ == '__main__':
    main()
