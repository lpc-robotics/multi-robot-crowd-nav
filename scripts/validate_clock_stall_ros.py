#!/usr/bin/env python3
"""Actual ROS adapter/core/guard: coherent clock pause must stop and recover."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from validate_multi_sfm_faults import World, ROOT


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") in (None, "51", "71"):
        raise SystemExit("Use a private ROS domain")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    commands = [
        ("core", ["ros2", "run", "arena_multi_hunav_core", "multi_sfm_server"]),
        ("adapter", ["ros2", "run", "arena_multi_hunav", "multi_sfm_adapter", "--ros-args",
            "-p", "use_sim_time:=true", "-p", f"agent_config_file:={ROOT}/config/hunav/multi_regular_1.yaml",
            "-p", "psychology_model:=sis_cf_v1", "-p", f"psychology_config_file:={ROOT}/config/psychology/sis_cf_v1.yaml"]),
        ("guard", ["ros2", "run", "arena_multi_control", "command_guard", "--ros-args",
            "-p", "robot_name:=robot_1", "-p", "control_mode:=external", "-p", "peer_names:=[robot_2]",
            "-p", "require_pedestrian_health:=true"]),
    ]
    children, logs = [], []
    rclpy.init()
    node = World()
    try:
        for name, command in commands:
            log = (args.output_dir / (name+".log")).open("w")
            logs.append(log)
            children.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        sim, started, injected, resumed = 10.0, time.monotonic(), None, None
        frozen = None
        healthy_again = None
        seen_paused = False
        zero_when_held = False
        moved_after_recovery = False
        fresh_command_required = True
        frozen_model = True
        while time.monotonic()-started < 20:
            now = time.monotonic()
            if injected is None and node.health and node.psychology.get("step", 0)>5 and any(abs(v)>.05 for _,v in node.commands):
                injected = now
            pausing = injected is not None and now-injected < 2.0
            if not pausing:
                sim += .005
                # Keep fresh sensors, but withhold external commands after the
                # health transition to prove old queued commands cannot restart.
                if injected is None or (healthy_again is not None and now-healthy_again >= .3):
                    node.tick(sim)
                else:
                    publisher = node.cmd
                    node.cmd = type("SilentPublisher", (), {"publish": lambda self,msg:None})()
                    node.tick(sim)
                    node.cmd = publisher
            else:
                cmd = Twist(); cmd.linear.x = .1; node.cmd.publish(cmd)
            for _ in range(12):
                rclpy.spin_once(node, timeout_sec=0)
            seen_paused |= bool(node.status.get("paused"))
            if pausing and node.status.get("paused"):
                if frozen is None:
                    frozen = node.psychology
                frozen_model &= frozen == node.psychology
                zero_when_held |= any(t>injected+.7 and abs(v)<1e-8 for t,v in node.commands)
            if injected and not pausing and resumed is None:
                resumed = now
            if resumed and node.health and healthy_again is None:
                healthy_again = now
            if healthy_again and now-healthy_again < .25:
                fresh_command_required &= not any(t>healthy_again and abs(v)>.01 for t,v in node.commands)
            if healthy_again and now-healthy_again > .5:
                moved_after_recovery = any(t>healthy_again+.3 and abs(v)>.05 for t,v in node.commands)
                if moved_after_recovery:
                    break
            if node.status.get("fault"):
                break
            time.sleep(.005)
        report = {"scope":"real ROS nodes and synthetic clock/sensors; no Isaac motion",
            "pause_observed":seen_paused, "guard_zero_during_hold":zero_when_held,
            "psychology_frozen_during_hold":frozen_model and frozen is not None,
            "fresh_command_required_after_recovery":fresh_command_required,
            "motion_after_new_command":moved_after_recovery,
            "final_fault":node.status.get("fault"), "final_health":node.health,
            "wall_duration_s":time.monotonic()-started}
        report["passed"] = all(report[k] for k in ["pause_observed", "guard_zero_during_hold",
            "psychology_frozen_during_hold", "fresh_command_required_after_recovery", "motion_after_new_command"])
        report["passed"] &= not report["final_fault"] and report["final_health"]
        (args.output_dir/"result.json").write_text(json.dumps(report,indent=2)+"\n")
        print(json.dumps(report,indent=2),flush=True)
        return 0 if report["passed"] else 1
    finally:
        node.destroy_node(); rclpy.shutdown()
        for child in children:
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
        for child in children:
            try: child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGKILL);child.wait()
        for log in logs:log.close()


if __name__ == "__main__":
    raise SystemExit(main())
