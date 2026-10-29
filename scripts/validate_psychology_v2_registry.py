#!/usr/bin/env python3
"""Check parameter coverage and effective scenario values without claiming identification."""
from dataclasses import asdict
import json
from pathlib import Path
import sys
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/arena_multi_hunav'))
from arena_multi_hunav.psychology_v2_config import ParametersV2
from arena_multi_hunav.psychology_config import load_psychology_config


def validate():
    doc=yaml.safe_load((ROOT/'config/psychology/parameter_registry_v2.yaml').read_text())
    entries={e['id']:e for e in doc['parameters']}
    if len(entries)!=len(doc['parameters']): raise ValueError('duplicate parameter id')
    required={'id','location','meaning','unit','current_value','scope','legal_range','category','fit_candidate',
              'calibration_status','observations','coupled_parameters','identifiability','fit_bounds','source','dataset','fit_result'}
    for e in entries.values():
        if required-set(e) or not e['unit'] or not e['meaning']: raise ValueError('incomplete parameter '+e['id'])
        if e['fit_candidate'] and e['calibration_status']!='unfitted': raise ValueError('unsupported fit claim')
        if e['category'] in ('engineering_experiment','measured_geometry') and e['fit_candidate']:
            raise ValueError('geometry/engineering must not enter automatic fit')
    cfg=load_psychology_config(ROOT/'config/psychology/sis_cf_v2.yaml')
    for key,value in asdict(ParametersV2()).items():
        e=entries['psychology.'+key]
        if e['current_value']!=value or getattr(cfg.parameters,key)!=value: raise ValueError('default drift: '+key)
        for i in cfg.individuals:
            expected=e.get('per_agent_overrides',{}).get(i,value)
            if getattr(cfg.resolved(i)[1],key)!=expected: raise ValueError('individual drift: '+str(i)+':'+key)
    return dict(passed=True,parameters=len(entries),psychology_parameters=len(asdict(ParametersV2())),
                fit_candidates=sum(e['fit_candidate'] for e in entries.values()),calibration='no fitted parameters')


if __name__=='__main__': print(json.dumps(validate(),indent=2))
