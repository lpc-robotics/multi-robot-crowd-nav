#!/usr/bin/env python3
"""Freeze effective V2 inputs before starting processes; never infer provenance later."""
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src/arena_multi_control'), str(ROOT/'src/arena_multi_hunav')]
from arena_multi_control.scenario import load_scenario
from arena_multi_hunav.psychology_config import load_psychology_config
from multigoal_evidence import provenance


def digest(path):
    with Path(path).open('rb') as f: return hashlib.file_digest(f, 'sha256').hexdigest()


def capture(scenario_path, run_dir):
    scenario = load_scenario(scenario_path)
    if scenario.psychology_model != 'sis_cf_v2': return
    run = Path(run_dir); inputs = run/'inputs'; inputs.mkdir(exist_ok=False)
    registry = ROOT/'config/psychology/parameter_registry_v2.yaml'
    files = [Path(scenario_path), Path(scenario.pedestrian_config), Path(scenario.psychology_config),
             registry, ROOT/'config/world/arena.yaml']
    if scenario.demo_tasks: files.append(Path(scenario.demo_tasks))
    hashes = {}
    for path in files:
        dest = inputs/path.name
        if dest.exists(): raise ValueError('ambiguous input basename')
        dest.write_bytes(path.read_bytes()); hashes[str(path.resolve())] = digest(dest)
    cfg = load_psychology_config(scenario.psychology_config)
    people = yaml.safe_load(Path(scenario.pedestrian_config).read_text())['hunav_loader']['ros__parameters']
    ids = [people[n]['id'] for n in people['agents']]
    if set(cfg.individuals)-set(ids): raise ValueError('unknown individual configuration')
    data = dict(schema_version=1, model='sis_cf_v2', model_config_hash=cfg.digest,
        provenance=provenance(scenario_path, ROOT/'config/world/arena.yaml'),
        source_dirty=bool(subprocess.check_output(['git','-C',str(ROOT),'status','--porcelain','--','src','scripts','config'],text=True)),
        input_sha256=hashes, registry_sha256=digest(registry), registry_file='inputs/'+registry.name,
        resolved_individuals={str(i):dict(profile=cfg.resolved(i)[0].profile,
            preferred_robot_id=cfg.resolved(i)[0].preferred_robot_id,parameters=asdict(cfg.resolved(i)[1])) for i in ids},
        participants={str(people[n]['id']):n for n in people['agents']}, scenario=asdict(scenario),
        clock=dict(kind='ROS simulation nanoseconds', frame_id='map', core_dt_s=.025,
                   actual_velocity='finite difference of Character Graph positions; not core velocity'),
        seed_semantics='fixed profiles and deterministic psychology; seed is not a population sample')
    with (run/'source_manifest.json').open('x') as f: json.dump(data,f,indent=2,allow_nan=False)


if __name__ == '__main__': capture(sys.argv[1], sys.argv[2])
