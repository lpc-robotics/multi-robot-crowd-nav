#!/usr/bin/env python3
"""Exercise configured routes against a private real core service, without Isaac."""
import argparse
import copy
import math
from pathlib import Path
import subprocess
import time
import types
import uuid

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message, deserialize_message
from ament_index_python.packages import get_package_prefix
from hunav_msgs.msg import Agent, Agents
from arena_multi_hunav_msgs.srv import ComputeMultiAgents
from arena_multi_hunav_msgs.msg import BehaviorModifiers
from arena_multi_control.scenario import load_scenario
from arena_multi_hunav.multi_adapter import load_people, validate_response
import yaml

from multigoal_evidence import ROOT, observe, provenance, save_report, trackers


def wire_equal(first, second):
    # CDR alignment padding is not a message field and may contain allocator
    # residue. Compare decoded wire values, retaining ROS float32 rounding.
    return (deserialize_message(serialize_message(first), Agents)
            == deserialize_message(serialize_message(second), Agents))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario")
    parser.add_argument("--world", default=str(ROOT / "config/world/arena.yaml"))
    parser.add_argument("--duration-sim", type=float, default=180)
    parser.add_argument("--cycles", type=int, default=2)
    parser.add_argument("--baseline-ref", default="backup-multigoal-20261010")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    if not math.isfinite(args.duration_sim) or args.duration_sim < .025 or args.cycles < 1:
        parser.error("duration must be >=0.025 seconds and cycles >=1")
    scenario = load_scenario(args.scenario)
    world = yaml.safe_load(Path(args.world).read_text()); bounds = (world["width"], world["height"])
    people, _ = load_people(scenario.pedestrian_config, world_bounds=bounds)
    progress = trackers(scenario.pedestrian_config, bounds)
    proof = provenance(args.scenario, args.world)
    robots = Agents(); robots.header.frame_id = "map"
    for i, configured in enumerate(sorted(scenario.robots, key=lambda r: r.name), 1):
        a = Agent(); a.id = -i; a.name = configured.name; a.type = Agent.ROBOT
        a.radius = .35; a.group_id = -1; a.yaw = configured.yaw
        a.position.position.x = configured.x; a.position.position.y = configured.y
        a.position.orientation.w = 1.; robots.agents.append(a)
    rclpy.init(); node = Node("multigoal_cpu_probe_" + uuid.uuid4().hex[:8])
    service = "/multigoal_cpu_" + uuid.uuid4().hex + "/compute"
    executable = Path(get_package_prefix("arena_multi_hunav_core")) / "lib/arena_multi_hunav_core/multi_sfm_server"
    failures = []; max_speed = 0.; min_clearance = math.inf; max_acceleration = 0.
    old_comparisons = {}; steps = 0; epoch = 1; request_count = 0
    log_path = Path(args.output).with_suffix(".server.log"); log_path.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    with log_path.open("w") as log:
        server = subprocess.Popen([str(executable), "--ros-args", "-r", "__node:=multigoal_cpu_core",
            "-r", "/multirobot/hunav/compute_agents:=" + service,
            "-p", f"world_width:={float(bounds[0])}", "-p", f"world_height:={float(bounds[1])}"], stdout=log, stderr=log)
        client = node.create_client(ComputeMultiAgents, service)
        def advance(state):
            nonlocal request_count
            request_count += 1
            snapshot = copy.deepcopy(robots); snapshot.header.stamp = state.header.stamp
            req = ComputeMultiAgents.Request(epoch=epoch, step=request_count, dt=.025,
                current_agents=state, robots=snapshot,
                modifiers=[BehaviorModifiers(agent_id=a.id) for a in state.agents])
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            if not future.done(): raise RuntimeError("CPU compute service timeout")
            result = future.result(); validate_response(req, result)
            return result.updated_agents
        try:
            if not client.wait_for_service(timeout_sec=20): raise RuntimeError("CPU core unavailable")
            observe(progress, people)
            for steps in range(1, round(args.duration_sim / .025) + 1):
                previous = people; people = advance(people); observe(progress, people)
                for before, a in zip(previous.agents, people.agents):
                    p = a.position.position
                    max_speed = max(max_speed, math.hypot(a.velocity.linear.x, a.velocity.linear.y))
                    max_acceleration = max(max_acceleration, math.hypot(
                        a.velocity.linear.x-before.velocity.linear.x, a.velocity.linear.y-before.velocity.linear.y) / .025)
                    if not (a.radius <= p.x <= bounds[0]-a.radius and a.radius <= p.y <= bounds[1]-a.radius):
                        raise ValueError("pedestrian left world envelope")
                    for robot in robots.agents:
                        r = robot.position.position
                        min_clearance = min(min_clearance, math.hypot(p.x-r.x, p.y-r.y)-a.radius-robot.radius)
                if steps % 1200 == 0:
                    print(f"CPU simulation {steps*.025:.0f}s: cycles=" + str({n:t.report()['completed_cycles'] for n,t in progress.items()}), flush=True)
            if any(t.report()["completed_cycles"] < args.cycles for t in progress.values()):
                failures.append("some pedestrians did not complete required route cycles")
            if min_clearance < .1-1e-5: failures.append("CPU robot clearance below 0.10 m")
            if max_speed > 1.00001 or max_acceleration > 3.00001: failures.append("CPU motion limit exceeded")
            baseline = types.ModuleType("arena_multi_hunav._baseline_adapter")
            baseline.__package__ = "arena_multi_hunav"
            source = subprocess.check_output(["git", "-C", str(ROOT), "show",
                args.baseline_ref + ":src/arena_multi_hunav/arena_multi_hunav/multi_adapter.py"], text=True)
            exec(compile(source, "baseline_multi_adapter.py", "exec"), baseline.__dict__)
            for count in (1, 6):
                config = ROOT / f"config/hunav/multi_regular_{count}.yaml"
                old, old_models = baseline.load_people(config)
                new, new_models = load_people(config, world_bounds=bounds)
                if not wire_equal(old, new) or old_models != new_models:
                    raise ValueError("legacy loader output changed")
                for _ in range(1200):
                    old, new = advance(old), advance(new)
                    if not wire_equal(old, new):
                        raise ValueError("legacy two-goal trajectory changed")
                old_comparisons[str(count)] = {"identical": True, "duration_sim_s": 30., "steps": 1200}
        except Exception as error:
            failures.append(str(error))
        finally:
            server.terminate()
            try: server.wait(timeout=10)
            except subprocess.TimeoutExpired: server.kill(); server.wait()
            node.destroy_node(); rclpy.shutdown()
    report = {"passed": not failures, "scope": "CPU core with stationary robots; no rendered geometry",
        "duration_sim_s": steps*.025, "duration_wall_s": time.monotonic()-started,
        "minimum_clearance_m": min_clearance if math.isfinite(min_clearance) else None,
        "maximum_speed_mps": max_speed, "maximum_acceleration_mps2": max_acceleration,
        "people": {n:t.report() for n,t in progress.items()}, "legacy_comparisons": old_comparisons,
        "provenance": proof, "failures": failures}
    if provenance(args.scenario, args.world) != proof:
        report["passed"] = False; report["failures"].append("source changed during validation")
    save_report(args.output, report)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
