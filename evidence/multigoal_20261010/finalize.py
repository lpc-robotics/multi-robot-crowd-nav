"""Bind completed development acceptance to current source and preserved baseline."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from multigoal_evidence import provenance

BACKUP = ROOT.parent / "arena5_multi_backups/20261010T090406Z-before-multigoal"
SCENARIO = ROOT / "config/scenarios/two_robots_multi_sfm_6_multigoal_nav2.yaml"
WORLD = ROOT / "config/world/arena.yaml"


def sha(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024*1024), b""): digest.update(block)
    return digest.hexdigest()


def main():
    proof = provenance(SCENARIO, WORLD)
    reports = {}; values = {}
    for name in ("offline", "cpu_routes", "isaac_routes_30min", "navigation", "restore_check",
                 "preservation", "installed_source_check"):
        path = HERE / "validation" / (name + ".json")
        data = json.loads(path.read_text())
        assert data["passed"] is True, (name, data.get("failures"))
        if name in ("offline", "cpu_routes", "isaac_routes_30min"):
            assert data["provenance"] == proof, name + ": source/input mismatch"
        reports[name] = {"path": str(path.relative_to(ROOT)), "sha256": sha(path), "passed": True}
        values[name] = data
    assert values["isaac_routes_30min"]["duration_wall_s"] >= 1800
    assert values["isaac_routes_30min"]["route_deadline_sim_s"] == 180
    assert len(values["isaac_routes_30min"]["people_at_route_deadline"]) == 6
    assert all(p["completed_cycles"] >= 2 for p in values["isaac_routes_30min"]["people_at_route_deadline"].values())
    assert values["isaac_routes_30min"]["minimum_actual_clearance_m"] >= .1 - 1e-5
    assert values["isaac_routes_30min"]["maximum_display_error_m"] <= .25
    assert values["isaac_routes_30min"]["maximum_speed_mps"] <= 1.00001
    assert values["cpu_routes"]["duration_sim_s"] >= 180
    assert len(values["cpu_routes"]["people"]) == 6
    assert all(p["completed_cycles"] >= 2 for p in values["cpu_routes"]["people"].values())
    assert all(values["cpu_routes"]["legacy_comparisons"][str(count)] ==
               {"identical": True, "duration_sim_s": 30., "steps": 1200} for count in (1, 6))
    nav = values["navigation"]
    assert nav["rounds_requested"] == nav["rounds_completed"] == 10
    assert nav["source_commit"] == proof["source_commit"]
    assert nav["scenario_sha256"] == sha(SCENARIO)
    assert nav["tasks_sha256"] == sha(ROOT / "config/tasks/two_robot_lanes.yaml")
    assert all(len(r["robots"]) == 2 and all(p["passed"] for p in r["robots"].values()) for r in nav["results"])
    underlay = json.loads(subprocess.check_output([sys.executable, str(ROOT / "scripts/verify_baseline.py")], text=True))
    assert underlay["passed"]
    baseline = json.loads((BACKUP / "manifest.json").read_text())
    prefixes = ("config/", "src/arena_multi_hunav_core/src/", "src/arena_multi_hunav_core/include/",
                "src/arena_multi_hunav_core/vendor/", "src/arena_multi_hunav_msgs/")
    protected = {name: item["sha256"] for name, item in baseline.items()
                 if item["type"] == "file" and (name == "command.txt" or name.startswith(prefixes))}
    assert all(sha(ROOT / name) == digest for name, digest in protected.items()), "protected file changed"
    report = {"schema_version": 1, "passed": True,
        "scope": "multi-goal development acceptance; not a release certification",
        "source_commit": proof["source_commit"], "source_sha256": proof["source_sha256"],
        "baseline_ref": "backup-multigoal-20261010", "backup": str(BACKUP),
        "underlay": underlay, "preserved_files": protected, "reports": reports}
    (HERE / "acceptance.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k:v for k,v in report.items() if k != "preserved_files"}, indent=2))


if __name__ == "__main__":
    main()
