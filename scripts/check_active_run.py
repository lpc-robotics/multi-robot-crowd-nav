#!/usr/bin/env python3
"""Reject another Arena runtime in the same ROS domain, including orphans."""
import os
from pathlib import Path
import sys


def active_runs():
    domain = os.environ.get("ROS_DOMAIN_ID", "71")
    own_group = os.getpgrp()
    matches = {}
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            if entry.stat().st_uid != os.getuid():
                continue
            pid = int(entry.name)
            group = os.getpgid(pid)
            if group == own_group:
                continue
            args = [s.decode(errors="replace") for s in
                    (entry / "cmdline").read_bytes().split(b"\0") if s]
            if not args or "-c" in args[:2]:
                continue
            env = dict(s.split(b"=", 1) for s in
                       (entry / "environ").read_bytes().split(b"\0") if b"=" in s)
            if env.get(b"ROS_DOMAIN_ID", b"0").decode() != domain:
                continue
            if not env.get(b"ARENA_MULTI_WS") or not env.get(b"ARENA_MULTI_RUN_DIR"):
                continue
            # Runtime executables, not terminals, editors or diagnostic scripts.
            runtime = any("/install/" in s or "/.conda/arena_ros/lib/" in s
                          or s.endswith("/ros2") for s in args[:2])
            if runtime:
                matches.setdefault(group, []).append((pid, Path(args[0]).name))
        except (OSError, ValueError):
            continue
    return matches


if __name__ == "__main__":
    found = active_runs()
    if found:
        print("Existing Arena runtime in this ROS domain; clean it up before restarting.",
              file=sys.stderr)
        for group, processes in sorted(found.items()):
            print(f"  PGID={group} processes={processes}", file=sys.stderr)
        print("Use scripts/cleanup.sh with the previous run's ARENA_MULTI_LOG_ROOT.",
              file=sys.stderr)
        sys.exit(1)
