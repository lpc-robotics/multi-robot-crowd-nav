import csv
import json
from pathlib import Path
import pytest

from arena_multi_hunav import data_v2 as data


def test_robot_resampling_is_explicit_and_simulator_latents_stay_separate(tmp_path):
    run=tmp_path/'run';(run/'inputs').mkdir(parents=True)
    registry=run/'inputs/registry.yaml';registry.write_text('test registry\n')
    meta=dict(model='sis_cf_v2',registry_file='inputs/registry.yaml',registry_sha256=data.sha256(registry),
              input_sha256={'/fixture/registry.yaml':data.sha256(registry)},model_config_hash='fixed',
              resolved_individuals={'1':{'profile':'standard'}})
    (run/'source_manifest.json').write_text(json.dumps(meta))
    p=dict(name='person_1',x=1.,y=2.,vx=0.,vy=0.,yaw=0.)
    state=dict(behavior=dict(mode='REGULAR',target_robot_id=0,reason='regular'),variables={'curiosity':0.,'fear':0.},
               surprise=0.,phase='none',event='',playful_blockers=['ineligible_profile'])
    row=dict(schema_version=3,plugin={'config_hash':'fixed'},epoch=1,stamp_ns=25_000_000,input_stamp_ns=0,
             people={'1':p},states={'1':state},robots={'-1':dict(p,name='robot_1')})
    (run/'psychology_1.jsonl').write_text(json.dumps(row)+'\n')
    actual=tmp_path/'actual.jsonl';actual.write_text(json.dumps(dict(source='isaac_actual',frame_id='map',stamp_ns=25_000_000,
         people={'person_1':dict(p,velocity_source='finite_difference_actual_pose')}))+'\n')
    out=tmp_path/'out';result=data.export_data(run,out,'trial',actual)
    assert result['passed'] and result['observations']==3
    with (out/'observations.csv').open() as f:rows=list(csv.DictReader(f))
    robot=next(r for r in rows if r['entity_type']=='robot')
    assert robot['interpolated']=='unknown'
    assert robot['velocity_source']=='odometry_timeline_resampled_world_frame'
    assert robot['stamp_ns']=='0'  # Retain input time, not the human end-of-step time.
    assert all(r['interpolated']=='false' for r in rows if r['entity_type']=='person')
    assert not {'curiosity','fear','surprise'} & set(rows[0])
    manifest=json.loads((out/'manifest.json').read_text())
    assert manifest['processing']['exporter_version']=='1.1'
    assert 'bracketing' in manifest['robot_resampling']
    assert manifest['units']['body_yaw']=='radians'
    # A valid checksum alone does not make simulator activations valid data.
    latent=out/'latents.csv';original=latent.read_text()
    with latent.open() as f:latent_rows=list(csv.DictReader(f))
    latent_rows[0]['fear']='nan'
    with latent.open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=data.LATENTS);writer.writeheader();writer.writerows(latent_rows)
    manifest['output_sha256']['latents.csv']=data.sha256(latent)
    (out/'manifest.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError,match='latent activation'):data.validate_data(out)
    latent.write_text(original);manifest['output_sha256']['latents.csv']=data.sha256(latent)
    # Hash-valid but semantically invalid metadata must still be rejected.
    path=out/'observations.csv';path.write_text(path.read_text().replace(',unknown,',',guessed,'))
    manifest['output_sha256']['observations.csv']=data.sha256(path)
    (out/'manifest.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError,match='quality/derivation'):data.validate_data(out)
