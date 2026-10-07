"""Render the measured core force scan to standalone PNG/PDF figures."""
import csv
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parent
with (root/"scan.csv").open() as stream:
    rows = list(csv.DictReader(stream))

def series(layout, speed=.8, scale=0):
    return [r for r in rows if r["layout"] == layout and float(r["initial_speed"]) == speed and float(r["robot_scale"]) == scale]

fig, axes = plt.subplots(1, 3, figsize=(14, 4.2), constrained_layout=True)
near = series("front")
axes[0].plot([float(r["distance"]) for r in near], [float(r["near_sum_magnitudes"]) for r in near], color="#176b91")
axes[0].axhline(3, color="#c33e34", linestyle="--", label="Total acceleration cap")
axes[0].set(xlabel="Center distance (m)", ylabel="Single-robot near magnitude (m/s²)", title="Unchanged physical defaults")
axes[0].legend(fontsize=8)

for layout, label, color in [("front", "One robot ahead", "#176b91"), ("rear", "One robot behind", "#bc6935"), ("dual_front", "Two robots ahead ±30°", "#7b589b")]:
    data = series(layout)
    axes[1].plot([float(r["distance"]) for r in data], [math.hypot(float(r["raw_ax"]), float(r["raw_ay"])) for r in data], label=label, color=color)
axes[1].axhline(3, color="#c33e34", linestyle="--")
axes[1].set(xlabel="Center distance (m)", ylabel="Total acceleration before cap (m/s²)", title="Walking 0.8 m/s; robot_scale=0")
axes[1].legend(fontsize=8)

with (root/"full_circle.csv").open() as stream:
    full = list(csv.DictReader(stream))
ratios = []
for layout in ["front", "side", "edge", "blind", "rear"]:
    selected = lambda r: r["layout"] == layout and float(r["initial_speed"]) == .8 and float(r["robot_scale"]) == 1 and abs(float(r["distance"])-1.5) < 1e-10
    new = next(r for r in rows if selected(r)); old = next(r for r in full if selected(r))
    ratios.append(math.hypot(float(new["social_x"]), float(new["social_y"])) / math.hypot(float(old["social_x"]), float(old["social_y"])))
axes[2].bar(["0°", "90°", "95°", "110°", "180°"], ratios, color="#176b91")
axes[2].set(xlabel="Robot bearing relative to pedestrian yaw", ylabel="Social force / full-circle social force", ylim=(0, 1.1), title="200° FOV; 10° fade on each edge")
for ax in axes:
    ax.grid(axis="y", alpha=.2)
fig.savefig(root/"force_scan.png", dpi=180)
fig.savefig(root/"force_scan.pdf")
