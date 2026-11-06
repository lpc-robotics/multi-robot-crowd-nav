"""Reproduce this single descriptive paired comparison from the archived bundles."""
import hashlib
import json
from pathlib import Path

root = Path(__file__).resolve().parent
bundles = {name: root/('actual_'+name+'_passed') for name in ('intrusive','considerate')}
reports = {k: json.loads((p/'acceptance.json').read_text()) for k,p in bundles.items()}
manifests = {k: json.loads((p/'source_manifest.json').read_text()) for k,p in bundles.items()}
a,b = (manifests[k] for k in ('intrusive','considerate'))
checks = dict(reports_passed=all(r['passed'] for r in reports.values()),
    runtime_source_identical=a['provenance']==b['provenance'],
    effective_psychology_identical=a['resolved_individuals']==b['resolved_individuals'],
    model_config_identical=a['model_config_hash']==b['model_config_hash'],
    same_seed=a['scenario']['random_seed']==b['scenario']['random_seed'],
    same_people=a['scenario']['pedestrian_config']==b['scenario']['pedestrian_config'],
    same_clock=a['clock']==b['clock'])
# Scenario hashes intentionally differ; source files, including both scenarios,
# are identical. Verify the source-specific fields rather than each run's inputs.
checks['runtime_source_identical'] = all(a['provenance'][key]==b['provenance'][key]
    for key in ('source_commit','source_sha256','source_files'))
keys = ('variant','wall_s','sim_s','peak_fear','final_fear','scared_sim_s','fear_triggered_scared_sim_s',
        'scared_reasons','actual_outward_s','actual_outward_displacement_m','task_resumed_samples',
        'minimum_robot_gap_m','minimum_human_gap_m','max_display_error_m','core_max_speed_mps','core_max_acceleration_mps2')
report = dict(passed=all(checks.values()),checks=checks,
    scope='one descriptive paired simulation with explicit sustained-attention assumption; not human-data validation or statistical causality',
    runtime_source_commit=a['provenance']['source_commit'],
    conditions={k:dict(metrics={key:r[key] for key in keys},stimulus=r['stimulus_execution']['stimulus']) for k,r in reports.items()},
    interpretation='Speed, approach gain and desired clearance vary together. No individual factor effect or real population prevalence is estimated.',
    input_sha256={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest()
        for directory in bundles.values() for p in (directory/'acceptance.json',directory/'source_manifest.json')},
    processor_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
with (root/'matched_comparison.json').open('x') as stream:
    json.dump(report,stream,indent=2,allow_nan=False)
assert report['passed'], checks
