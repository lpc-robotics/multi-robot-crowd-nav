"""Ablation controls allow natural non-completion; physical gates stay unchanged."""
import math


def assess_control(report, manifest):
    people = manifest.get('resolved_individuals', {})
    failures = []
    if manifest.get('model') != 'sis_cf_v2' or not people or any(
            p['parameters']['playful_enabled'] != 0 for p in people.values()):
        failures.append('control must disable blocking for every individual')
    provenance = manifest.get('provenance', {})
    if any(report.get('provenance', {}).get(k) != provenance.get(k) or not provenance.get(k)
           for k in ('source_commit', 'source_sha256')):
        failures.append('runtime provenance mismatch')
    if set(report.get('modes', {})) != set(people):
        failures.append('incomplete participant coverage')

    # The active demonstration requires completed observations for both playful
    # actors. An ablation may legitimately lose a target during observation.
    # Preserve that outcome; never waive motion, timing, identity or RPC failures.
    optional = {f'missing actual stopped aligned observation: {key}' for key in people}
    original_failures = report.get('failures', [])
    nonrequired = sorted(set(original_failures) & optional)
    failures.extend(reason for reason in original_failures if reason not in optional)

    bounds = {
        'minimum_robot_gap_m': (.10, math.inf),
        'minimum_human_gap_m': (0., math.inf),
        'max_display_error_m': (0., .25),
        'core_max_speed_mps': (0., 1.00001),
        'core_max_acceleration_mps2': (0., 3.00001),
        'matched_samples': (2, math.inf),
        'wall_s': (report.get('requested_wall_s', math.inf), math.inf),
    }
    for name, (low, high) in bounds.items():
        value = report.get(name)
        if not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
            failures.append('physical/duration gate failed: ' + name)
    status = report.get('last_status') or {}
    if not status.get('healthy') or status.get('fault') or status.get('paused'):
        failures.append('control did not finish healthy')
    demo = report.get('demo') or {}
    rounds = demo.get('rounds', [])
    if not demo.get('passed') or not rounds or len(rounds) != demo.get('rounds_requested') or any(
            item.get('status') != 4 for r in rounds for item in r['robots'].values()):
        failures.append('complete successful navigation demonstration required')
    if any('THREATENING' in modes for modes in report.get('modes', {}).values()) or any(
            key.endswith(':playful_started') and count for key, count in report.get('events', {}).items()):
        failures.append('disabled control contains blocking')
    return {
        'passed': not failures,
        'scope': 'ablation control: unchanged physical/transaction gates, complete navigation, no blocking; natural surprise exits allowed',
        'original_acceptance_passed': report.get('passed'),
        'nonrequired_active_demonstration_checks': nonrequired,
        'natural_surprise_exits': {k: v for k, v in report.get('events', {}).items()
                                  if k.endswith((':surprise_target_lost', ':surprise_timeout', ':surprise_risk'))},
        'metrics': {name: report.get(name) for name in (*bounds, 'sim_s')},
        'runtime_source_commit': provenance.get('source_commit'),
        'failures': sorted(set(failures)),
    }
