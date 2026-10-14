"""Shared provenance and route definitions for multi-goal acceptance tools."""
import hashlib
import json
from pathlib import Path
import subprocess

import yaml

from arena_multi_hunav.routes import compile_route
from arena_multi_hunav.route_tracking import RouteTracker

ROOT = Path(__file__).resolve().parents[1]


def provenance(scenario_path, world_path):
    names = subprocess.check_output([
        "git", "-C", str(ROOT), "ls-files", "--cached", "--others", "--exclude-standard", "-z",
        "src", "scripts", "config"], text=True).split("\0")
    files = {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
             for name in sorted(set(names) - {""})}
    paths = [Path(scenario_path).resolve(), Path(world_path).resolve()]
    return {"source_commit": subprocess.check_output(
                ["git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True).strip(),
            "source_sha256": hashlib.sha256(json.dumps(files, sort_keys=True).encode()).hexdigest(),
            "source_files": files,
            "inputs": {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}}


def trackers(config_path, bounds):
    spec = yaml.safe_load(Path(config_path).read_text())["hunav_loader"]["ros__parameters"]
    return {name: RouteTracker(compile_route(spec[name], person=name, world_bounds=bounds),
                               float(spec[name]["goal_radius"])) for name in spec["agents"]}


def observe(trackers_by_name, message):
    stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
    if message.header.frame_id != "map" or {a.name for a in message.agents} != set(trackers_by_name):
        raise ValueError("route frame or pedestrian identity changed")
    for agent in message.agents:
        p = agent.position.position
        trackers_by_name[agent.name].observe(
            stamp, p.x, p.y, [(g.position.x, g.position.y) for g in agent.goals])


def save_report(path, report):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps({k: v for k, v in report.items() if k not in ("provenance", "people")}, indent=2))
