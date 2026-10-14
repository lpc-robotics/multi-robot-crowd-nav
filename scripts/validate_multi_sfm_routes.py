#!/usr/bin/env python3
"""Read-only route order, rendered geometry and continuous-health acceptance."""
import argparse
import copy
import math
from pathlib import Path
import time

import rclpy
from arena_multi_control.scenario import load_scenario
import yaml

from multigoal_evidence import ROOT, observe, provenance, save_report, trackers
from validate_multi_sfm_runtime import Probe, stamp


class RouteProbe(Probe):
    def __init__(self, scenario, bounds, duration_sim):
        self.progress = trackers(scenario.pedestrian_config, bounds)
        self.bounds = bounds
        self.route_start = None; self.route_elapsed = 0.; self.route_deadline_report = None
        self.route_duration = duration_sim; self.maximum_speed = 0.
        self.epoch = None
        super().__init__(scenario)

    def people(self, message):
        super().people(message)
        if self.started is None:
            return
        try:
            observe(self.progress, message)
            current = stamp(message)
            if self.route_start is None: self.route_start = current
            self.route_elapsed = (current-self.route_start)/1e9
            for a in message.agents:
                speed = math.hypot(a.velocity.linear.x, a.velocity.linear.y)
                if not math.isfinite(speed) or speed > 1.00001:
                    raise ValueError("pedestrian speed exceeds 1 m/s or is nonfinite")
                self.maximum_speed = max(self.maximum_speed, speed)
                p = a.position.position
                if not (a.radius <= p.x <= self.bounds[0]-a.radius and a.radius <= p.y <= self.bounds[1]-a.radius):
                    raise ValueError("pedestrian outside world envelope")
            if self.route_elapsed >= self.route_duration and self.route_deadline_report is None:
                self.route_deadline_report = copy.deepcopy({n:t.report() for n,t in self.progress.items()})
        except ValueError as error:
            self.failures.add(str(error))

    def on_status(self, message):
        super().on_status(message)
        if self.started is None or not self.status:
            return
        epoch = self.status[-1].get("epoch")
        if self.epoch is not None and epoch != self.epoch:
            self.failures.add("backend epoch changed during route validation")
        self.epoch = epoch


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario")
    parser.add_argument("--world", default=str(ROOT / "config/world/arena.yaml"))
    parser.add_argument("--duration-sim", type=float, default=180)
    parser.add_argument("--duration-wall", type=float, default=0,
                        help="minimum healthy wall time; use 1800 for the endurance gate")
    parser.add_argument("--wall-timeout", type=float, default=5400)
    parser.add_argument("--ready-timeout", type=float, default=300)
    parser.add_argument("--cycles", type=int, default=2)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    if (not all(math.isfinite(v) for v in (args.duration_sim, args.duration_wall, args.wall_timeout, args.ready_timeout))
            or args.duration_sim <= 0 or args.duration_wall < 0 or args.ready_timeout <= 0
            or args.wall_timeout <= args.duration_wall or args.cycles < 1):
        parser.error("invalid duration, timeout or cycle count")
    scenario = load_scenario(args.scenario)
    if scenario.pedestrian_backend != "multi_sfm": parser.error("multi_sfm scenario required")
    world = yaml.safe_load(Path(args.world).read_text())
    proof = provenance(args.scenario, args.world)
    rclpy.init(); node = RouteProbe(scenario, (world["width"], world["height"]), args.duration_sim)
    duration_wall = 0.; last_progress = time.monotonic()
    try:
        until = time.monotonic() + args.ready_timeout
        while time.monotonic() < until and not node.ready() and not node.failures:
            rclpy.spin_once(node, timeout_sec=.02)
        if not node.ready(): raise RuntimeError("route probe readiness timeout")
        node.started = time.monotonic()
        while not node.failures:
            rclpy.spin_once(node, timeout_sec=.01); node.check_geometry()
            now = time.monotonic(); duration_wall = now-node.started
            if now-node.health_seen > .6: node.failures.add("health heartbeat missing")
            if duration_wall >= args.wall_timeout: raise RuntimeError("route probe wall timeout")
            if node.route_elapsed >= args.duration_sim and duration_wall >= args.duration_wall: break
            if now-last_progress >= 30:
                print(f"ROUTE_PROGRESS wall={duration_wall:.1f}s sim={node.route_elapsed:.1f}s cycles="
                      + str({n:t.report()['completed_cycles'] for n,t in node.progress.items()}), flush=True)
                last_progress = now
        # Stop adding rendered samples and let the last ones acquire aligned truth.
        node.destroy_subscription(next(s for s in node.subscriptions if s.topic_name == "/multirobot/hunav/actual_people"))
        until = time.monotonic() + 1
        while node.pending and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.02); node.check_geometry()
        if node.pending: node.failures.add("unmatched rendered route samples")
        if node.route_deadline_report is None:
            node.failures.add("route simulation duration not completed")
        elif any(v["completed_cycles"] < args.cycles for v in node.route_deadline_report.values()):
            node.failures.add("some pedestrians did not complete required cycles by simulation deadline")
        if duration_wall < args.duration_wall: node.failures.add("endurance wall duration not completed")
        if node.checked_samples < 10 or any(t.samples < 10 for t in node.progress.values()):
            node.failures.add("insufficient route or actual geometry samples")
        if node.max_display_error > .25: node.failures.add("display error above 0.25 m")
        if len(node.influences) != len(node.radii)*len(node.names): node.failures.add("missing interaction pairs")
        if any(v < 2 for v in node.guard_samples.values()): node.failures.add("missing guard diagnostics")
        graph = [name for name, _ in node.get_node_names_and_namespaces()]
        if graph.count("multi_sfm_server") != 1 or graph.count("multi_sfm_adapter") != 1:
            node.failures.add("nonunique backend")
        if len(node.get_publishers_info_by_topic("/multirobot/hunav/agents")) != 1:
            node.failures.add("nonunique pedestrian state publisher")
    except Exception as error:
        node.failures.add(str(error))
    if provenance(args.scenario, args.world) != proof: node.failures.add("source changed during validation")
    report = {"passed": not node.failures, "scope": "GPU route order, synchronized rendered geometry and health",
        "duration_wall_s": duration_wall, "duration_sim_s": node.route_elapsed,
        "route_deadline_sim_s": args.duration_sim, "people_at_route_deadline": node.route_deadline_report,
        "people": {n:t.report() for n,t in node.progress.items()}, "maximum_speed_mps": node.maximum_speed,
        "minimum_actual_clearance_m": node.min_clearance if math.isfinite(node.min_clearance) else None,
        "maximum_display_error_m": node.max_display_error, "maximum_error_sample": node.maximum_error_sample,
        "checked_samples": node.checked_samples, "skipped_samples": node.skipped_samples,
        "last_status": node.status[-1] if node.status else None,
        "failures": sorted(node.failures), "failure_events": node.failures.events, "provenance": proof}
    save_report(args.output, report)
    node.destroy_node(); rclpy.shutdown()
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
