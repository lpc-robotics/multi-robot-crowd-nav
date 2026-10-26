#!/usr/bin/env python3
"""Pause ONLY the registered Isaac process, then always resume it.

No ROS motion commands or goals are published. Used to reproduce a stopped
frame source while the independent adapter and command guards keep running.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def isaac_pid(run):
    group = int((run/"launch.pid").read_text().strip())
    found = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            pid = int(entry.name)
            if entry.stat().st_uid != os.getuid() or os.getpgid(pid) != group:
                continue
            args = (entry/"cmdline").read_bytes().split(b"\0")
            if not any(a.endswith(b"/arena_isaac/run_isaacsim") for a in args[:2]):
                continue
            env = dict(s.split(b"=",1) for s in (entry/"environ").read_bytes().split(b"\0") if b"=" in s)
            if env.get(b"ARENA_MULTI_RUN_DIR", b"").decode() != str(run):
                raise RuntimeError("Isaac run identity mismatch")
            found.append(pid)
        except (FileNotFoundError, ProcessLookupError):
            continue
    if len(found) != 1:
        raise RuntimeError(f"Expected one registered Isaac process, found {found}")
    return found[0]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    run = args.run_directory.resolve()
    rclpy.init()
    node = rclpy.create_node("isaac_clock_stall_injection_probe")
    status, commands, statuses = {}, [], []

    def on_status(msg):
        status.clear(); status.update(json.loads(msg.data))
        statuses.append((time.monotonic(), dict(status)))

    def on_command(msg):
        commands.append((time.monotonic(), msg.linear.x, msg.angular.z))

    subscriptions = [node.create_subscription(String,"/multirobot/hunav/status",on_status,10),
                     node.create_subscription(Twist,"/robot_1/cmd_vel",on_command,10)]
    paused_pid = None
    paused_start_ticks = None
    try:
        end = time.monotonic()+300
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
            if status.get("fault"):
                raise RuntimeError(status["fault"])
            if status.get("healthy") and commands and time.monotonic()-commands[-1][0]<.2 and abs(commands[-1][1])>.05:
                break
        else:
            raise RuntimeError("No healthy moving robot before injection")
        pid = isaac_pid(run)
        paused_start_ticks = Path(f"/proc/{pid}/stat").read_text().split()[21]
        recoveries = status.get("clock_stall_recoveries",0)
        injected = time.monotonic()
        os.kill(pid,signal.SIGSTOP); paused_pid = pid
        while time.monotonic()-injected<2:
            rclpy.spin_once(node,timeout_sec=.01)
        if Path(f"/proc/{pid}/stat").read_text().split()[21] != paused_start_ticks:
            raise RuntimeError("Isaac process identity changed")
        os.kill(pid,signal.SIGCONT); paused_pid = None
        resumed = time.monotonic()
        end = resumed+20
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
            if status.get("fault"):
                break
            if status.get("healthy") and status.get("clock_stall_recoveries",0)>recoveries and any(
                    t>resumed and abs(v)>.05 for t,v,w in commands):
                break
        zero = [t for t,v,w in commands if t>=injected and abs(v)<1e-8 and abs(w)<1e-8]
        held = any(t>=injected and s.get("paused") for t,s in statuses)
        moving = any(t>resumed and abs(v)>.05 for t,v,w in commands)
        held_commands = [(v,w) for t,v,w in commands if injected+.8<=t<=resumed]
        safely_stopped = bool(held_commands) and all(abs(v)<1e-8 and abs(w)<1e-8 for v,w in held_commands)
        report = {"scope":"actual registered Isaac process suspension; no new goals or commands",
            "isaac_pid":pid,"pause_wall_s":resumed-injected,"paused_status_observed":held,
            "first_zero_wall_s":None if not zero else zero[0]-injected,
            "all_commands_zero_during_hold":safely_stopped,
            "movement_resumed":moving,"final_status":status}
        report["passed"] = bool(held and zero and zero[0]-injected<1 and safely_stopped and moving and
            status.get("healthy") and not status.get("fault") and
            status.get("clock_stall_recoveries",0)>recoveries)
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2)+"\n")
        print(json.dumps(report,indent=2),flush=True)
        return 0 if report["passed"] else 1
    finally:
        if paused_pid is not None:
            try:
                if Path(f"/proc/{paused_pid}/stat").read_text().split()[21] == paused_start_ticks:
                    os.kill(paused_pid,signal.SIGCONT)
            except FileNotFoundError:
                pass
        node.destroy_node();rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
