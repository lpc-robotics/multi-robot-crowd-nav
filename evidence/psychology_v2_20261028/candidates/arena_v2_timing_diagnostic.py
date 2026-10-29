#!/usr/bin/env python3
"""Read-only timing evidence; never sends goals, commands or service requests."""
import argparse
from collections import deque
import json
from pathlib import Path
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=600)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("arena_runtime_timing_monitor")
    started = time.monotonic()
    samples, seen, events, status = {}, {}, deque(maxlen=500), {}
    subscriptions = []

    def receive(topic, message):
        now = time.monotonic()
        sample = samples.setdefault(topic, {"count": 0, "max_wall_gap_s": 0.0})
        sample["count"] += 1
        if topic in seen:
            gap = now-seen[topic]
            sample["max_wall_gap_s"] = max(gap, sample["max_wall_gap_s"])
            if gap > (1.5 if topic.endswith("/status") else .4):
                events.append({"topic": topic, "wall_elapsed_s": now-started, "gap_s": gap})
        seen[topic] = now
        if isinstance(message, Clock):
            stamp = message.clock.sec+message.clock.nanosec*1e-9
            sample.setdefault("first_sim_s", stamp)
            sample["last_sim_s"] = stamp
        elif isinstance(message, Odometry):
            pos = [message.pose.pose.position.x, message.pose.pose.position.y]
            sample.setdefault("first_position", pos)
            sample["last_position"] = pos
        elif isinstance(message, Bool):
            sample["last_health"] = message.data
        else:
            status.clear()
            status.update(json.loads(message.data))

    for topic, kind in [("/clock", Clock), ("/robot_1/odom", Odometry),
                        ("/robot_2/odom", Odometry), ("/multirobot/hunav/status", String), ("/multirobot/hunav/health", Bool)]:
        subscriptions.append(node.create_subscription(
            kind, topic, lambda msg, topic=topic: receive(topic, msg), qos_profile_sensor_data))

    args.output.parent.mkdir(parents=True, exist_ok=True)

    def save():
        report = {"wall_duration_s": time.monotonic()-started, "samples": samples,
                  "gaps_over_0_4s": list(events), "status": status}
        temp = args.output.with_suffix(".tmp")
        temp.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
        temp.replace(args.output)

    last_save = started
    last_loop = started
    try:
        while time.monotonic()-started < args.duration:
            now=time.monotonic()
            if now-last_loop>.2:events.append({"topic":"monitor_scheduling", "wall_elapsed_s":now-started,"gap_s":now-last_loop})
            last_loop=now
            rclpy.spin_once(node, timeout_sec=.05)
            if time.monotonic()-last_save >= 1:
                save()
                last_save = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
