"""Observable data exchange. Simulator latents are optional and explicitly separate."""
import csv
import hashlib
import json
import math
from pathlib import Path

OBSERVATIONS = ('experiment_id','participant_id','entity_type','stamp_ns','frame_id','x','y','vx','vy',
                'body_yaw','source','quality','velocity_source','interpolated','missing_fields')
EVENTS = ('experiment_id','participant_id','start_ns','end_ns','label','target_robot_id',
          'annotation_source','confidence','kind','reason')
LATENTS = ('experiment_id','participant_id','stamp_ns','curiosity','fear','surprise','profile','mode','phase')


def sha256(path):
    with Path(path).open('rb') as f: return hashlib.file_digest(f,'sha256').hexdigest()


def export_data(run_dir, output_dir, experiment_id, actual_path=None):
    run, out = Path(run_dir), Path(output_dir)
    if not experiment_id: raise ValueError('experiment_id is required')
    meta = json.loads((run/'source_manifest.json').read_text())
    if meta['model'] != 'sis_cf_v2': raise ValueError('V2 manifest required')
    registry = run/meta['registry_file']
    if sha256(registry) != meta['registry_sha256']: raise ValueError('registry snapshot changed')
    for path, digest in meta['input_sha256'].items():
        if sha256(run/'inputs'/Path(path).name)!=digest: raise ValueError('changed runtime input: '+path)
    traces = sorted(run.glob('psychology_*.jsonl'))
    if not traces: raise ValueError('no committed psychology trace')
    # Never overwrite a previous experiment or partially replace its files.
    out.mkdir(parents=True, exist_ok=False)
    seen_modes = {}; first = {}; last = {}; previous = {}; sources = set(); counts = {}
    streams = [(out/name).open('w', newline='') for name in ('observations.csv','events.csv','latents.csv')]
    ow, ew, lw = [csv.DictWriter(f,fieldnames=fields) for f,fields in zip(streams,(OBSERVATIONS,EVENTS,LATENTS))]
    for writer in (ow,ew,lw): writer.writeheader()
    def event(who, start, end, label, target='', kind='mode', reason=''):
        ew.writerow(dict(experiment_id=experiment_id,participant_id=who,start_ns=start,end_ns=end,label=label,
            target_robot_id=target,annotation_source='simulator_model',confidence='',kind=kind,reason=reason))
    def observe(who, kind, t, p, source, velocity_source, interpolated="false"):
        ow.writerow(dict(experiment_id=experiment_id,participant_id=who,entity_type=kind,stamp_ns=t,
            frame_id='map',x=p['x'],y=p['y'],vx=p['vx'],vy=p['vy'],body_yaw=p['yaw'],source=source,
            quality='simulated',velocity_source=velocity_source,interpolated=interpolated,missing_fields=''))
        sources.add(source); counts[source]=counts.get(source,0)+1
    epochs=set()
    try:
        for trace in traces:
            for line in trace.open():
                row=json.loads(line)
                if row['schema_version']!=3 or row['plugin']['config_hash']!=meta['model_config_hash']:
                    raise ValueError('trace/config mismatch')
                epochs.add(row['epoch'])
                if len(epochs)>1: raise ValueError('export epochs as separate experiments')
                t=row['stamp_ns']; input_t=row['input_stamp_ns']
                for key,p in row['people'].items():
                    name=p['name']; observe(name,'person',t,p,'core_committed','integrated_state')
                    state=row['states'][key]; b=state['behavior']; mode=b['mode']; target=b['target_robot_id'] or ''
                    profile=meta['resolved_individuals'][key]['profile']
                    lw.writerow(dict(experiment_id=experiment_id,participant_id=name,stamp_ns=t,
                        curiosity=state['variables']['curiosity'],fear=state['variables']['fear'],surprise=state['surprise'],
                        profile=profile,mode=mode,phase=state['phase']))
                    first.setdefault(name,t); last[name]=t; seen_modes.setdefault(name,set()).add(mode)
                    if name in previous and previous[name][:2] != (mode,target):
                        pm,pt,start=previous[name]; event(name,start,t,pm,pt,reason=b['reason'])
                        previous[name]=(mode,target,t)
                    previous.setdefault(name,(mode,target,t))
                    if state.get('event'):
                        event_target=state.get('event_target_robot_id',
                            state.get('interaction_target') if state['event'].startswith('playful') else target)
                        event(name,t,t,state['event'],event_target or '', 'transition',state.get('exit_reason',''))
                    blockers=tuple(state.get('playful_blockers',[]))
                    # Retain rejected opportunities; condense only consecutive identical gate outcomes.
                    gate_key=('gate',name)
                    if blockers != previous.get(gate_key):
                        if blockers: event(name,t,t,'not_triggered',target,'gate',';'.join(blockers))
                        previous[gate_key]=blockers
                # Timeline.at() resamples robot odometry before this log is written.
                # Old schema-3 traces do not retain brackets/exact-knot provenance.
                # Never label these rows as unmodified raw measurements.
                for p in row['robots'].values():
                    observe(p['name'],'robot',input_t,p,'robot_observation',
                            'odometry_timeline_resampled_world_frame',interpolated='unknown')
        for name in first:
            mode,target,start=previous[name]; event(name,start,last[name],mode,target,reason='recording_end_right_censored')
            for mode in ('SURPRISED','THREATENING'):
                if mode not in seen_modes[name]: event(name,first[name],last[name],mode,'','nontrigger','not_observed_in_recorded_window')
        if actual_path:
            for line in Path(actual_path).open():
                row=json.loads(line)
                if row['source']!='isaac_actual' or row['frame_id']!='map': raise ValueError('unexpected actual source/frame')
                for name,p in row['people'].items():
                    observe(name,'person',row['stamp_ns'],p,'isaac_actual',p['velocity_source'])
    finally:
        for f in streams: f.close()
    manifest=dict(schema_version=1,experiment_id=experiment_id,provenance=meta,
        units={'stamp_ns':'nanoseconds','position':'metres','velocity':'metres/second','body_yaw':'radians'},
        input_traces={str(p.resolve()):sha256(p) for p in traces},
        actual_input=None if not actual_path else {'path':str(Path(actual_path).resolve()),'sha256':sha256(actual_path)},
        output_sha256={name:sha256(out/name) for name in ('observations.csv','events.csv','latents.csv')},
        sources=sorted(sources),rows_by_source=counts,clock_alignment='same simulation clock; logged source timestamps retained; exporter performs no additional interpolation',
        robot_resampling='adapter timeline already resampled odometry; original bracketing stamps and exact-knot flags unavailable in these traces; interpolation flag is unknown',
        split_policy='group by participant AND entire experiment; never split neighboring frames',
        latent_semantics='optional simulator output; not observations of real human emotions',
        processing=dict(exporter_version='1.1',exporter_sha256=sha256(__file__)),
        labeling='simulator-derived labels, not human annotations or emotion ground truth')
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
    return validate_data(out)


def validate_data(directory):
    """Accept measured data without latents.csv; reject mixed/missing provenance and nonfinite rows."""
    root=Path(directory); manifest=json.loads((root/'manifest.json').read_text())
    if manifest.get('schema_version')!=1 or not manifest.get('experiment_id') or not manifest.get('provenance'):
        raise ValueError('missing experiment/provenance')
    for name in ('observations.csv','events.csv'):
        if manifest.get('output_sha256',{}).get(name)!=sha256(root/name): raise ValueError('changed file: '+name)
    times={}; observations=0; events=0
    with (root/'observations.csv').open(newline='') as f:
        reader=csv.DictReader(f)
        if tuple(reader.fieldnames or ())!=OBSERVATIONS: raise ValueError('observations columns mismatch')
        for row in reader:
            observations+=1
            if row['experiment_id']!=manifest['experiment_id'] or not row['participant_id'] or not row['frame_id']:
                raise ValueError('observation identity/frame missing')
            if row['source'] not in ('core_committed','isaac_actual','robot_observation','measured') or row['entity_type'] not in ('person','robot'):
                raise ValueError('unknown data source/entity')
            if not row['quality'] or not row['velocity_source'] or row['interpolated'] not in ('true','false','unknown'):
                raise ValueError('missing observation quality/derivation')
            missing=set(filter(None,row['missing_fields'].split(';')))
            if missing-set(('x','y','vx','vy','body_yaw')):raise ValueError('unknown missing field')
            for key in ('x','y','vx','vy','body_yaw'):
                if row[key]=='':
                    if key not in missing: raise ValueError('unmarked missing field')
                elif key in missing or not math.isfinite(float(row[key])): raise ValueError('invalid observation value')
            t=int(row['stamp_ns']); key=(row['participant_id'],row['entity_type'],row['source'])
            if t<0 or t<=times.get(key,-1): raise ValueError('duplicate/nonmonotonic source timestamp')
            times[key]=t
    with (root/'events.csv').open(newline='') as f:
        reader=csv.DictReader(f)
        if tuple(reader.fieldnames or ())!=EVENTS: raise ValueError('event columns mismatch')
        for row in reader:
            events+=1
            if row['experiment_id']!=manifest['experiment_id'] or not row['participant_id'] or not row['annotation_source'] or not row['label']:
                raise ValueError('event identity/source/label missing')
            if not 0<=int(row['start_ns'])<=int(row['end_ns']): raise ValueError('invalid event interval')
            if row['confidence'] and not 0<=float(row['confidence'])<=1: raise ValueError('invalid event confidence')
    if not observations: raise ValueError('empty observations')
    if (root/'latents.csv').exists():
        if manifest['output_sha256'].get('latents.csv')!=sha256(root/'latents.csv'):
            raise ValueError('changed latents file')
        latent_times={}
        with (root/'latents.csv').open(newline='') as f:
            reader=csv.DictReader(f)
            if tuple(reader.fieldnames or ())!=LATENTS: raise ValueError('latents columns mismatch')
            for row in reader:
                if row['experiment_id']!=manifest['experiment_id'] or not row['participant_id']:
                    raise ValueError('latent identity missing')
                for key in ('curiosity','fear','surprise'):
                    if not math.isfinite(float(row[key])) or not 0<=float(row[key])<=1:
                        raise ValueError('invalid latent activation')
                if row['profile'] not in ('standard','immune_dynamic','impassive_static','playful') or row['mode'] not in ('REGULAR','CURIOUS','SCARED','SURPRISED','THREATENING') or row['phase'] not in ('none','intercept','occupy','yield'):
                    raise ValueError('invalid latent category')
                t=int(row['stamp_ns']);key=row['participant_id']
                if t<0 or t<=latent_times.get(key,-1):raise ValueError('duplicate/nonmonotonic latent timestamp')
                latent_times[key]=t
    return dict(passed=True,observations=observations,events=events,sources=sorted({k[2] for k in times}))
