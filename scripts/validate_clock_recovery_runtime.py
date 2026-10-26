#!/usr/bin/env python3
"""Actual geometry checks with explicitly verified safe clock-hold intervals.

The original strict continuous-health validator remains unchanged. Here an
unhealthy interval is accepted only when bracketed by clock-hold status, zero
commands and subsequent recovery; all geometric thresholds remain identical.
"""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Twist
from arena_multi_control.scenario import load_scenario
from validate_psychology_runtime import PsychologyProbe
from multigoal_evidence import provenance


class RecoveryProbe(PsychologyProbe):
    def __init__(self, scenario):
        self.unhealthy_start = None
        self.health_windows = []
        self.pause_start = None
        self.pause_compute = None
        self.pause_windows = []
        self.guard_events = []
        self.command_counts = {r.name: 0 for r in scenario.robots}
        self.last_recovery_wall = 0.0
        super().__init__(scenario)
        for name in self.names:
            self.create_subscription(Twist, f"/{name}/cmd_vel",
                lambda msg, name=name: self.command(name, msg), 10)

    def health(self, msg):
        self.healthy = msg.data
        self.health_seen = time.monotonic()
        if self.started is None:
            return
        if not msg.data and self.unhealthy_start is None:
            self.unhealthy_start = self.health_seen
        if msg.data and self.unhealthy_start is not None:
            self.health_windows.append((self.unhealthy_start, self.health_seen))
            self.unhealthy_start = None

    def on_status(self, msg):
        super().on_status(msg)  # A real latched fault always fails acceptance.
        if self.started is None or not self.status:
            return
        data = self.status[-1]
        now = time.monotonic()
        if data.get("paused"):
            if self.pause_start is None:
                self.pause_start, self.pause_compute = now, data.get("compute")
            elif data.get("compute") != self.pause_compute:
                self.failures.add("psychology committed during clock hold")
        elif self.pause_start is not None:
            self.pause_windows.append((self.pause_start, now))
            self.pause_start = None
            self.last_recovery_wall = now

    def command(self, name, msg):
        self.command_counts[name] += 1
        if self.started is not None and self.unhealthy_start is not None:
            if time.monotonic()-self.unhealthy_start > .6 and (
                    abs(msg.linear.x)>1e-8 or abs(msg.angular.z)>1e-8):
                self.failures.add(name+" moved during unhealthy hold")

    def _on_guard_diagnostics(self, msg):
        if self.started is None:
            return
        for status in msg.status:
            for name in self.names:
                if status.name == f"multirobot/{name}/command_guard":
                    self.guard_samples[name] += 1
                    if status.message not in ("ok", "command_stale"):
                        self.guard_events.append((time.monotonic(), name, status.message))

    def finish_holds(self):
        if self.pause_start is not None or self.unhealthy_start is not None or not self.healthy:
            self.failures.add("not healthy at end of recovery acceptance")
        for start, end in self.health_windows:
            if not any(p-.2 <= start <= end <= q+.6 for p,q in self.pause_windows):
                self.failures.add("unrecognized unhealthy interval")
        for when, name, reason in self.guard_events:
            expected = reason == "pedestrian_unhealthy" or reason == "lidar_stale" or reason.startswith("peer_stale:")
            if not expected or not any(start-.2 <= when <= end+.6 for start,end in self.pause_windows):
                self.failures.add(f"{name} unexpected guard stop: {reason}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario")
    parser.add_argument("--duration-wall", type=float, default=600)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    before = provenance(args.scenario, root/"config/world/arena.yaml")
    rclpy.init()
    node = RecoveryProbe(load_scenario(args.scenario))
    try:
        deadline = time.monotonic()+300
        while time.monotonic()<deadline and not (node.ready() and node.psych_count>=40):
            rclpy.spin_once(node, timeout_sec=.05)
            if node.failures:
                break
        if not (node.ready() and node.psych_count>=40):
            node.failures.add("readiness timeout")
        else:
            node.started = time.monotonic()
            sim_start = node.last_psych_stamp
            deadline = node.started+args.duration_wall
            while time.monotonic()<deadline or (node.pause_start is not None and time.monotonic()<deadline+35):
                rclpy.spin_once(node, timeout_sec=.02)
                node.check_geometry()
                now = time.monotonic()
                if now-node.health_seen > .6:
                    node.failures.add("health heartbeat missing")
                if node.unhealthy_start is not None and now-node.unhealthy_start>1.2 and node.pause_start is None:
                    node.failures.add("unhealthy without clock hold")
                if node.pause_start is None and now-node.psych_seen>1 and now-node.last_recovery_wall>1:
                    node.failures.add("psychology stream missing outside clock hold")
                if node.failures:
                    break
        drain = time.monotonic()+1
        node.destroy_subscription(next(s for s in node.subscriptions if s.topic_name=="/multirobot/hunav/actual_people"))
        while node.pending and time.monotonic()<drain:
            rclpy.spin_once(node, timeout_sec=.02); node.check_geometry()
        node.finish_holds()
        if node.pending or node.checked_samples<2:
            node.failures.add("insufficient matched actual samples")
        if node.min_clearance<.1:
            node.failures.add("actual robot clearance below 0.10 m")
        if node.max_display_error>.25:
            node.failures.add("actual display error above 0.25 m")
        if set(node.live_inputs)!={str(-i) for i in range(1,len(node.names)+1)}:
            node.failures.add("not all robots perceived")
        if min(node.command_counts.values())<10:
            node.failures.add("insufficient command observations")
        if before!=provenance(args.scenario,root/"config/world/arena.yaml"):
            node.failures.add("source/configuration changed during acceptance")
        report = {"passed":not node.failures,
            "scope":"actual Isaac geometry, clock holds and safe command recovery",
            "wall_duration_s":0 if node.started is None else time.monotonic()-node.started,
            "sim_duration_s":0 if node.started is None else (node.last_psych_stamp-sim_start)/1e9,
            "provenance":before, "matched_samples":node.checked_samples,
            "minimum_robot_gap_m":None if math.isinf(node.min_clearance) else node.min_clearance,
            "minimum_human_gap_m":None if math.isinf(node.minimum_human_gap) else node.minimum_human_gap,
            "maximum_display_error_m":node.max_display_error,
            "closed_clock_holds":len(node.pause_windows), "closed_unhealthy_intervals":len(node.health_windows),
            "command_counts":node.command_counts, "failures":sorted(node.failures)}
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+"\n")
        print(json.dumps({k:v for k,v in report.items() if k!="provenance"},indent=2),flush=True)
        return 0 if report["passed"] else 1
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
