#!/usr/bin/env python3
"""Plot configured routes; this is a layout diagram, not an actual-motion trace."""
import os
from pathlib import Path
os.environ.setdefault('MPLCONFIGDIR', '/tmp/arena-route-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import yaml

root = Path(__file__).resolve().parents[2]
output = Path(__file__).resolve().parent
specs = yaml.safe_load((root/'config/hunav/multi_regular_8_psychology_multigoal.yaml').read_text())['hunav_loader']['ros__parameters']
fig, ax = plt.subplots(figsize=(11, 9))
for index, name in enumerate(specs['agents']):
    person = specs[name]; points = [(person[k]['x'], person[k]['y']) for k in person['goals']]
    color = plt.get_cmap('tab10')(index)
    plotted = points + [points[0]] if person['route_mode'] == 'loop' else points
    ax.plot(*zip(*plotted), '-o', color=color, label=f"Person {index+1}: {person['route_mode']}", markersize=4)
    for j, (x, y) in enumerate(points):
        ax.annotate(f'g{j}', (x, y), xytext=(3, 7), textcoords='offset points', color=color, fontsize=8)
    ax.scatter(person['init_pose']['x'], person['init_pose']['y'], marker='*', color=color, s=120, zorder=3)
for i, point in enumerate([(3, 3), (3, 7)], 1):
    ax.scatter(*point, marker='s', color='black', s=70)
    ax.annotate(f'Robot {i} start', point, xytext=(-30, -18), textcoords='offset points', fontsize=9)
ax.set(title='Eight-person psychology scenario: configured four-waypoint routes', xlabel='Map x (m)', ylabel='Map y (m)',
       xlim=(0, 23), ylim=(1, 21))
ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(loc='upper right', fontsize=9)
fig.tight_layout()
for extension in ['png', 'svg']:
    path=output/f'configured_routes.{extension}';fig.savefig(path, dpi=180)
    if extension == 'svg': path.write_text('\n'.join(line.rstrip() for line in path.read_text().splitlines())+'\n')
