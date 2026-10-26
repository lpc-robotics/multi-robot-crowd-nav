#!/usr/bin/env python3
"""Choose CPUs local to a GPU without widening an existing caller CPU mask."""
import os
from pathlib import Path
import subprocess
import sys


def cpu_set(value):
    result = set()
    for item in value.split(","):
        first, _, last = item.partition("-")
        low, high = int(first), int(last or first)
        if low < 0 or high < low:
            raise ValueError("invalid CPU range")
        result.update(range(low, high+1))
    return result


def select_cpus(gpu):
    allowed = os.sched_getaffinity(0)
    requested = os.environ.get("ARENA_CPU_AFFINITY", "auto")
    if requested == "none":
        return allowed
    if requested != "auto":
        selected = cpu_set(requested) & allowed
        if not selected:
            raise ValueError("requested CPU affinity has no allowed CPUs")
        return selected
    try:
        bus = subprocess.check_output([
            "nvidia-smi", "--id="+gpu, "--query-gpu=pci.bus_id",
            "--format=csv,noheader,nounits"], text=True).strip().lower()
        domain, bus_device, function = bus.split(":")
        pci = f"{domain[-4:]}:{bus_device}:{function}"
        node = int((Path("/sys/bus/pci/devices") / pci / "numa_node").read_text())
        if node >= 0:
            local = cpu_set((Path("/sys/devices/system/node") / f"node{node}" / "cpulist").read_text().strip())
            return local & allowed or allowed
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f"GPU NUMA discovery unavailable; retaining caller affinity: {error}", file=sys.stderr)
    return allowed


if __name__ == "__main__":
    print(",".join(map(str, sorted(select_cpus(sys.argv[1])))))
