"""Finite robot-only exposure sequence. No crowd modes, latent values or human goals."""
from dataclasses import dataclass
import math
from pathlib import Path
from types import SimpleNamespace
import yaml
from .fear_stimulus import command


def load_sequence(path):
    d = yaml.safe_load(Path(path).read_text())
    if not isinstance(d, dict) or set(d) != {'schema_version','robots','startup_timeout_wall_s','experiment_timeout_wall_s','stages'} or type(d['schema_version']) is not int or d['schema_version'] != 1:
        raise ValueError('invalid crowd stimulus schema')
    names=d['robots']
    if not isinstance(names,list) or not names or any(not isinstance(n,str) or not n.startswith('robot_') or not n[6:].isdigit() for n in names) or len(set(names))!=len(names):
        raise ValueError('invalid stimulus robots')
    def positive(v):return type(v) in (int,float) and math.isfinite(v) and v>0
    if not all(positive(d[k]) for k in ('startup_timeout_wall_s','experiment_timeout_wall_s')):
        raise ValueError('invalid sequence wall timeout')
    if not isinstance(d['stages'],list) or not d['stages']:raise ValueError('empty sequence')
    seen=set()
    for s in d['stages']:
        if not isinstance(s,dict) or set(s)!={'name','duration_s','commands'} or not isinstance(s['name'],str) or not s['name'] or s['name'] in seen or not positive(s['duration_s']):
            raise ValueError('invalid stage')
        seen.add(s['name'])
        if not isinstance(s['commands'],dict) or set(s['commands'])!=set(names):raise ValueError('stage must cover every robot')
        for c in s['commands'].values():
            if not isinstance(c,dict) or c.get('kind') not in ('hold','point','follow'):raise ValueError('invalid command kind')
            common={'kind','speed','acceleration'}
            extra={'point'} if c['kind']=='point' else {'person','target_gap'} if c['kind']=='follow' else set()
            if set(c)!=common|extra or not positive(c['acceleration']) or c['acceleration']>2. or type(c['speed']) not in (int,float) or not math.isfinite(c['speed']) or not 0<=c['speed']<=1.:
                raise ValueError('invalid command fields or bounds')
            if (c['kind']=='hold') != (c['speed']==0):raise ValueError('hold must have zero speed; moving commands positive speed')
            if c['kind']=='point' and (not isinstance(c['point'],list) or len(c['point'])!=2 or not all(type(x) in (int,float) and math.isfinite(x) for x in c['point']) or not .5<=c['point'][0]<=29.5 or not .5<=c['point'][1]<=22.5):
                raise ValueError('point outside declared open world')
            if c['kind']=='follow' and (not isinstance(c['person'],str) or not c['person'] or type(c['target_gap']) not in (int,float) or not math.isfinite(c['target_gap']) or not -.5<=c['target_gap']<=5):
                raise ValueError('invalid follow command')
    if sum(s['duration_s'] for s in d['stages'])>d['experiment_timeout_wall_s']:
        raise ValueError('wall timeout shorter than nominal simulation protocol')
    return d


class Sequence:
    def __init__(self, cfg):
        self.cfg=cfg;self.index=0;self.stage_ns=None;self.last_ns=None;self.done=False
        self.last={n:(0.,0.) for n in cfg['robots']};self.transitions=[]

    def step(self, active_ns, robots, people, radii):
        if self.last_ns is not None and active_ns<self.last_ns:raise ValueError('sequence_clock_rewind')
        if self.last_ns==active_ns:return dict(self.last)
        dt=0. if self.last_ns is None else (active_ns-self.last_ns)/1e9
        if dt>.250001:raise ValueError('sequence_time_gap')
        self.last_ns=active_ns
        if self.stage_ns is None:
            self.stage_ns=active_ns;self.transitions.append(dict(index=0,name=self.cfg['stages'][0]['name'],active_ns=active_ns))
        if self.done:return {n:(0.,0.) for n in self.last}
        if (active_ns-self.stage_ns)/1e9>=self.cfg['stages'][self.index]['duration_s']:
            self.index+=1;self.stage_ns=active_ns
            if self.index==len(self.cfg['stages']):
                self.done=True;self.last={n:(0.,0.) for n in self.last};return dict(self.last)
            self.transitions.append(dict(index=self.index,name=self.cfg['stages'][self.index]['name'],active_ns=active_ns))
        for name,c in self.cfg['stages'][self.index]['commands'].items():
            if c['kind']=='hold':desired=(0.,0.)
            else:
                point=c['point'] if c['kind']=='point' else people[c['person']]
                stop=.2 if c['kind']=='point' else .35+radii[c['person']]+c['target_gap']
                desired=command(robots[name],point,stop,SimpleNamespace(max_linear=c['speed'],max_angular=1.,distance_gain=2.,yaw_gain=2.))
            previous=self.last[name]
            dv=c['acceleration']*dt
            self.last[name]=(max(previous[0]-dv,min(previous[0]+dv,desired[0])),desired[1])
        return dict(self.last)
