"""The model-evaluation probe must retain close encounters and their aftermath."""
from collections import deque
import io
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'scripts'))
from interaction_outcomes import ClearanceObservations
from validate_multi_sfm_runtime import Probe


def test_low_clearance_and_overlap_are_outcomes_not_model_failures():
    m = ClearanceObservations()
    for t, gap in [(0,.2),(50_000_000,.05),(100_000_000,-.02),(150_000_000,-.03),(200_000_000,.3)]:
        m.observe('p','r',t,gap)
    row = m.report()['pairs'][0]
    assert not m.strict_failure()
    assert row['minimum_clearance_m'] == -.03
    assert row['below_reference_observed_s'] == pytest.approx(.1)
    assert row['envelope_overlap_observed_s'] == pytest.approx(.05)
    assert row['below_reference'][0]['end_reason'] == 'observed_exit'
    assert not row['below_reference'][0]['right_censored']


def test_missing_data_never_becomes_invented_exposure_duration():
    m = ClearanceObservations()
    for t in (0,50_000_000,50_000_000,1_000_000_000,1_050_000_000):m.observe('p','r',t,.01)
    row=m.report()['pairs'][0]
    assert row['samples']==4 and row['duplicate_samples']==1
    assert row['unobserved_interval_s']==pytest.approx(.95)
    assert row['below_reference_observed_s']==pytest.approx(.1)
    assert len(row['below_reference'])==2
    assert all(e['right_censored'] for e in row['below_reference'])
    assert row['below_reference'][1]['left_censored']


def test_invalid_data_and_legacy_strict_policy_still_fail():
    m=ClearanceObservations('strict');m.observe('p','r',1,.04)
    assert m.strict_failure()
    with pytest.raises(ValueError):m.observe('p','r',0,.1)
    with pytest.raises(ValueError):m.observe('p','r',2,float('nan'))
    with pytest.raises(ValueError):ClearanceObservations('unknown')


def test_duplicate_timestamp_cannot_hide_a_lower_observed_gap():
    m=ClearanceObservations('strict');m.observe('p','r',1,.2);m.observe('p','r',1,-.01)
    row=m.report()['pairs'][0]
    assert m.strict_failure() and row['minimum_clearance_m']==-.01
    assert row['duplicate_envelope_overlap_samples']==1
    assert row['below_reference_observed_s']==0.


def fixture(policy, positions):
    predicted=NS(x=0.,y=0.,vx=0.,vy=0.)
    history=NS(samples=[NS(stamp_ns=1_000_000_000)],at=lambda t:predicted)
    messages=[]
    for k,x in enumerate(positions):
        messages.append((time.monotonic(),NS(header=NS(stamp=NS(sec=0,nanosec=(k+1)*50_000_000)),
            pedestrians=[NS(name='p',pose=NS(position=NS(x=x,y=0.)))])))
    return NS(pending=deque(messages),robot_history={'r':history},person_history={'p':history},radii={'p':.4},
        min_clearance=float('inf'),max_display_error=0.,maximum_error_sample=None,status=[],
        clearance_observations=ClearanceObservations(policy),clearance_stream=io.StringIO(),
        checked_samples=0,skipped_samples=0,failures=set())


@pytest.mark.parametrize('policy,failed',[('record',False),('strict',True)])
def test_actual_probe_records_close_overlap_and_recovery_frames(policy,failed):
    p=fixture(policy,[.8,.7,1.])
    Probe.check_geometry(p)
    assert bool(p.failures)==failed
    assert p.checked_samples==3 and not p.pending
    samples=[json.loads(s) for s in p.clearance_stream.getvalue().splitlines()]
    assert [s['below_reference'] for s in samples]==[True,True,False]
    assert [s['envelope_overlap'] for s in samples]==[False,True,False]
    assert p.clearance_observations.report()['pairs'][0]['minimum_clearance_m']==pytest.approx(-.05)


def test_record_policy_does_not_accept_nonfinite_actual_positions():
    p=fixture('record',[float('nan')]);Probe.check_geometry(p)
    assert 'nonfinite actual position' in p.failures
    assert p.clearance_stream.getvalue()==''
