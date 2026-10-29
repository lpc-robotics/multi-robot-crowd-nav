import copy
import pytest
from arena_multi_hunav.control_validation_v2 import assess_control


def control():
    provenance = dict(source_commit='frozen', source_sha256='digest')
    meta = dict(model='sis_cf_v2', provenance=provenance,
                resolved_individuals={'13': {'parameters': {'playful_enabled': 0}}})
    report = dict(passed=False, provenance=provenance, modes={'13': ['SURPRISED', 'CURIOUS']},
        failures=['missing actual stopped aligned observation: 13'],
        minimum_robot_gap_m=.24, minimum_human_gap_m=.4, max_display_error_m=.04,
        core_max_speed_mps=1., core_max_acceleration_mps2=3., matched_samples=100,
        wall_s=1200., requested_wall_s=600., sim_s=650.,
        last_status=dict(healthy=True, paused=False, fault=''),
        demo=dict(passed=True, rounds_requested=1, rounds=[{'robots': {'robot_1': {'status': 4}}}]),
        events={'13:surprise_target_lost': 1})
    return report, meta


def test_control_preserves_natural_exit_and_original_failed_report():
    report, meta = control(); before = copy.deepcopy(report)
    result = assess_control(report, meta)
    assert result['passed'] and report == before
    assert not result['original_acceptance_passed']
    assert result['natural_surprise_exits'] == {'13:surprise_target_lost': 1}


@pytest.mark.parametrize('reason', ['health missing', 'unhealthy', 'clock_rewind',
                                  'temporary task advanced', 'actual human overlap'])
def test_control_does_not_waive_physical_or_transaction_failures(reason):
    report, meta = control(); report['failures'].append(reason)
    assert reason in assess_control(report, meta)['failures']


def test_active_configuration_and_accidental_blocking_cannot_pass_as_control():
    report, meta = control(); meta['resolved_individuals']['13']['parameters']['playful_enabled'] = 1
    assert not assess_control(report, meta)['passed']
    report, meta = control(); report['modes']['13'].append('THREATENING')
    assert not assess_control(report, meta)['passed']


def test_control_still_checks_raw_geometry_and_complete_navigation():
    report, meta = control(); report['minimum_robot_gap_m'] = .09
    assert not assess_control(report, meta)['passed']
    report, meta = control(); report['demo']['passed'] = False
    assert not assess_control(report, meta)['passed']
