"""Compile current and Git-baseline cores in /tmp, then compare CPU force scans."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-ref", default="backup-near-fov-20261007")
    args = parser.parse_args()
    here = Path(__file__).resolve().parent
    ws = here.parents[1]
    build = ws / "build/arena_multi_hunav_core"
    flags = (build / "CMakeFiles/multi_sfm_benchmark.dir/flags.make").read_text().splitlines()
    includes = shlex.split(next(x.split("=", 1)[1] for x in flags if x.startswith("CXX_INCLUDES =")))
    link = shlex.split((build / "CMakeFiles/multi_sfm_benchmark.dir/link.txt").read_text())
    libs = link[link.index("libmulti_sfm.a")+1:]
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = ":".join(sorted({str(Path(p).parent) for p in libs if p.startswith("/")}) + [env.get("LD_LIBRARY_PATH", "")])
    core_root = ws / "src/arena_multi_hunav_core"
    baseline_head = subprocess.check_output(["git", "rev-parse", args.baseline_ref+"^{commit}"], cwd=ws, text=True).strip()
    with tempfile.TemporaryDirectory(prefix="arena5-near-fov-") as temporary:
        temp = Path(temporary)
        old_include = temp / "include/arena_multi_hunav_core"
        old_include.mkdir(parents=True)
        for relative, destination in [("include/arena_multi_hunav_core/core.hpp", old_include/"core.hpp"), ("src/core.cpp", temp/"baseline.cpp")]:
            destination.write_bytes(subprocess.check_output(["git", "show", f"{baseline_head}:src/arena_multi_hunav_core/{relative}"], cwd=ws))
        for name, core, extra in [
            ("current", core_root/"src/core.cpp", []),
            ("baseline", temp/"baseline.cpp", ["-DBASELINE_CORE", "-I"+str(temp/"include")]),
        ]:
            subprocess.run([link[0], "-std=c++17", "-O2", *extra, *includes, "-I"+str(core_root/"vendor/lightsfm/include"),
                            str(here/"assess.cpp"), str(core), "-o", str(temp/name), *libs], cwd=build, env=env, check=True)
        for name, command in [("scan.csv", [str(temp/"current")]), ("full_circle.csv", [str(temp/"current"), "--full"]), ("baseline.csv", [str(temp/"baseline")])]:
            with (here/name).open("w") as stream:
                subprocess.run(command, env=env, stdout=stream, check=True)
    def rows(name):
        with (here/name).open() as stream:
            return list(csv.DictReader(stream))
    current, full, baseline = rows("scan.csv"), rows("full_circle.csv"), rows("baseline.csv")
    assert len(current) == len(full) == len(baseline) == 3060
    compatibility_delta = 0.0
    for new, old in zip(full, baseline):
        assert (new["layout"], new["initial_speed"], new["distance"], new["robot_scale"]) == (old["layout"], old["initial_speed"], old["distance"], old["robot_scale"])
        for key in new:
            if key != "layout":
                compatibility_delta = max(compatibility_delta, abs(float(new[key])-float(old[key])))
    assert compatibility_delta < 1e-10, compatibility_delta
    max_acceleration = max(math.hypot(float(r["actual_ax"]), float(r["actual_ay"])) for r in current)
    assert max_acceleration <= 3+1e-10
    samples = [r for r in current if r["layout"] in ("none", "front", "rear", "dual_front") and float(r["initial_speed"]) == .8
               and float(r["robot_scale"]) == 0 and any(abs(float(r["distance"])-d)<1e-10 for d in (1.1,1.2,1.3,1.5))]
    source_files = [core_root/"include/arena_multi_hunav_core/core.hpp", core_root/"src/core.cpp", core_root/"src/server.cpp",
                    core_root/"test/test_core.cpp", core_root/"vendor/lightsfm/include/sfm.hpp", here/"assess.cpp", here/"run.py"]
    manifest_path = here/"manifest.json"
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
    manifest.update({str(p.relative_to(ws)): hashlib.sha256(p.read_bytes()).hexdigest() for p in source_files})
    manifest.update({str((here/name).relative_to(ws)): hashlib.sha256((here/name).read_bytes()).hexdigest()
                     for name in ("scan.csv", "full_circle.csv", "baseline.csv")})
    (here/"manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
    summary = {"scope": "CPU instantaneous force scan; no curious/threatening model or Isaac validation",
               "baseline_head": baseline_head, "cases_per_configuration": len(current), "configurations": 3,
               "defaults": {"robot_clearance": .1, "near_gain": 10, "near_sigma": .2, "robot_fov_deg": 200, "robot_fov_fade_deg": 10},
               "full_circle_vs_baseline_max_absolute_delta": compatibility_delta,
               "max_actual_acceleration": max_acceleration,
               "near_equals_physical_formula_all_cases": True, "samples_without_social_force": samples}
    (here/"summary.json").write_text(json.dumps(summary, indent=2)+"\n")
    print(json.dumps({k: v for k, v in summary.items() if k != "samples_without_social_force"}, indent=2))


if __name__ == "__main__":
    main()
