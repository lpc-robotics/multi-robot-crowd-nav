import csv
import json
from pathlib import Path
import pytest
import yaml
from arena_multi_hunav.data_v2 import OBSERVATIONS, EVENTS, sha256, validate_data, export_data

ROOT=Path(__file__).resolve().parents[3]


def test_original_eight_routes_and_cohort_are_preserved():
    old=yaml.safe_load((ROOT/'config/hunav/multi_regular_8_psychology_multigoal.yaml').read_text())['hunav_loader']['ros__parameters']
    new=yaml.safe_load((ROOT/'config/hunav/multi_14_psychology_v2_multigoal.yaml').read_text())['hunav_loader']['ros__parameters']
    for name in old['agents']: assert old[name]==new[name]
    assert len(new['agents'])==14
    cfg=yaml.safe_load((ROOT/'config/psychology/sis_cf_v2.yaml').read_text())
    from collections import Counter
    assert Counter(p['profile'] for p in cfg['individuals'].values())==dict(standard=8,immune_dynamic=2,impassive_static=2,playful=2)


def measured_dataset(tmp_path):
    with (tmp_path/'observations.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=OBSERVATIONS);w.writeheader()
        w.writerow(dict(experiment_id='real_trial',participant_id='pseudonym_1',entity_type='person',stamp_ns=1,
            frame_id='survey_map',x=1,y=2,vx='',vy='',body_yaw=.2,source='measured',quality='occlusion_partial',
            velocity_source='unavailable',interpolated='false',missing_fields='vx;vy'))
    with (tmp_path/'events.csv').open('w',newline='') as f:
        csv.DictWriter(f,fieldnames=EVENTS).writeheader()
    manifest=dict(schema_version=1,experiment_id='real_trial',provenance={'dataset_id':'future_measured_input'},
        output_sha256={name:sha256(tmp_path/name) for name in ('observations.csv','events.csv')})
    (tmp_path/'manifest.json').write_text(json.dumps(manifest))
    return manifest


def test_real_observations_do_not_require_emotional_latents(tmp_path):
    measured_dataset(tmp_path)
    assert validate_data(tmp_path)['passed']
    assert not (tmp_path/'latents.csv').exists()


def test_unmarked_missing_or_nonfinite_measurement_is_rejected(tmp_path):
    manifest=measured_dataset(tmp_path)
    path=tmp_path/'observations.csv';path.write_text(path.read_text().replace('1,2,,,0.2','nan,2,,,0.2'))
    manifest['output_sha256']['observations.csv']=sha256(path)
    (tmp_path/'manifest.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError):validate_data(tmp_path)


def test_export_refuses_unknown_runtime_provenance(tmp_path):
    with pytest.raises(FileNotFoundError):export_data(tmp_path,tmp_path/'out','unproven_trial')
