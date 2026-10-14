"""Plot configured goals and approach legs; these are not measured trajectories."""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
spec = yaml.safe_load((ROOT / "config/hunav/multi_regular_6_multigoal.yaml").read_text())["hunav_loader"]["ros__parameters"]
fig, ax = plt.subplots(figsize=(11, 7), layout="constrained")
for index, name in enumerate(spec["agents"]):
    item = spec[name]
    points = [(item[k]["x"], item[k]["y"]) for k in item["goals"]]
    color = plt.get_cmap("tab10")(index)
    route = points + points[:1] if item["route_mode"] == "loop" else points
    ax.plot(*zip(*route), color=color, lw=1.6, alpha=.8,
            label=f"{name}: {item['route_mode']}")
    for i, (x, y) in enumerate(points):
        ax.scatter(x, y, color=color, s=48, zorder=3)
        ax.annotate(f"{index+1}:g{i}", (x, y), xytext=(4, 6), textcoords="offset points", color=color, fontsize=8)
    p = item["init_pose"]
    ax.scatter(p["x"], p["y"], color=color, marker="x", s=75, zorder=4)
    ax.plot([p["x"], points[0][0]], [p["y"], points[0][1]], "--", color=color, alpha=.45)
    for a, b in zip(route, route[1:]):
        mid = ((a[0]+b[0])/2, (a[1]+b[1])/2)
        start = (mid[0]-.13*(b[0]-a[0]), mid[1]-.13*(b[1]-a[1]))
        arrow = "->" if item["route_mode"] == "loop" else "<->"
        ax.annotate("", xy=mid, xytext=start, arrowprops={"arrowstyle":arrow, "color":color})
ax.scatter([3, 3], [3, 7], marker="s", color="black", s=45, label="Robot spawn")
ax.set(xlim=(2, 21), ylim=(2.5, 12), xlabel="Map x (m)", ylabel="Map y (m)",
       title="Configured multi-goal routes (not measured paths)\nDashed: initial approach; x: pedestrian spawn")
ax.set_aspect("equal"); ax.grid(alpha=.2)
ax.legend(loc="upper center", ncol=3, fontsize=8)
fig.savefig(HERE / "routes.png", dpi=170)
fig.savefig(HERE / "routes.svg")
