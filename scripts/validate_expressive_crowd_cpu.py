#!/usr/bin/env python3
"""Closed-loop actual C++ people + ideal unicycle robot sequence; no Isaac/Nav2 claim."""
import argparse,json,math,os,signal,subprocess,sys
from collections import Counter
from dataclasses import asdict
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from arena_multi_hunav_msgs.msg import BehaviorIntentV3,BehaviorModifiers
from arena_multi_hunav_msgs.srv import ComputeMultiAgentsV3
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src/arena_multi_hunav'),str(ROOT/'src/arena_multi_control')]
from arena_multi_control.scenario import load_scenario
from arena_multi_control.crowd_stimulus import load_sequence,Sequence
from arena_multi_hunav.psychology_config import load_psychology_config
from arena_multi_hunav.psychology_v2 import SocialPsychologyV2
from arena_multi_hunav.psychology import prepare_step,make_context
from arena_multi_hunav.behavior_v2 import MODES_V2
from arena_multi_hunav.multi_adapter import load_people,entity,commit_candidate,motion_json
from validate_psychology_cpu import robots,call
from multigoal_evidence import provenance


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output-dir',required=True);a=ap.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') in (None,'51','71'):raise SystemExit('Use private ROS domain')
    out=Path(a.output_dir);out.mkdir(parents=True,exist_ok=False)
    path=ROOT/'config/scenarios/two_robots_multi_sfm_14_multigoal_psychology_v2_nav2.yaml';sc=load_scenario(path)
    cfg=load_psychology_config(sc.psychology_config);seq=Sequence(load_sequence(sc.sequence_tasks))
    source=provenance(path,ROOT/'config/world/arena.yaml')
    for p in (path,Path(sc.psychology_config),Path(sc.pedestrian_config),Path(sc.sequence_tasks)):(out/p.name).write_bytes(p.read_bytes())
    agents,_=load_people(sc.pedestrian_config);model=SocialPsychologyV2();model.initialize(cfg.document(),[p.id for p in agents.agents],sc.random_seed);states=model.initial_states()
    positions={r.name:(r.x,r.y,r.yaw) for r in sc.robots};commands={r.name:(0.,0.) for r in sc.robots};radii={p.name:p.radius for p in agents.agents}
    events=[];modes={p.id:Counter() for p in agents.agents};phases={p.id:Counter() for p in agents.agents};peak_f={p.id:0. for p in agents.agents};peak_social={p.id:0. for p in agents.agents}
    minimum_gap=minimum_human=math.inf;max_accel=0.;immune_ok=True;stamp=0;core=None
    duration=sum(s['duration_s'] for s in seq.cfg['stages'])+10.
    rclpy.init();node=Node('expressive_cpu_probe');client=node.create_client(ComputeMultiAgentsV3,'/expressive_cpu/compute')
    try:
        with (out/'core.log').open('x') as log,(out/'trace.jsonl').open('x') as trace:
            core=subprocess.Popen(['ros2','run','arena_multi_hunav_core','multi_sfm_server','--ros-args','-r','__node:=expressive_cpu_core','-r','/multirobot/hunav/compute_agents_v3:=/expressive_cpu/compute'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            if not client.wait_for_service(timeout_sec=20):raise RuntimeError('core unavailable')
            for k in range(round(duration/.025)):
                if k%2==0:commands=seq.step(stamp,positions,{p.name:(p.position.position.x,p.position.position.y) for p in agents.agents},radii)
                rs=robots(stamp,[(positions[n][0],positions[n][1],commands[n][0]*math.cos(positions[n][2]),commands[n][0]*math.sin(positions[n][2])) for n in seq.cfg['robots']])
                agents.header.stamp=Time(nanoseconds=stamp).to_msg()
                proposal,snapshot=prepare_step(model,make_context(stamp,.025,[entity(p) for p in agents.agents],[entity(r) for r in rs.agents]),states)
                req=ComputeMultiAgentsV3.Request(epoch=1,step=k+1,dt=.025,current_agents=agents,robots=rs)
                for i,intent in proposal.intents.items():
                    v=asdict(intent);v['mode']=MODES_V2[intent.mode];req.intents.append(BehaviorIntentV3(agent_id=i,**v));req.modifiers.append(BehaviorModifiers(agent_id=i))
                response=call(node,client,req);agents,stamp,states=commit_candidate(model,snapshot,proposal,req,response)
                for i,s in states.items():
                    modes[i][s.behavior.mode]+=1;phases[i][s.phase]+=1;peak_f[i]=max(peak_f[i],s.variables['fear']);peak_social[i]=max(peak_social[i],proposal.diagnostics['agents'][i]['social_F'])
                    if s.event:events.append(dict(person=i,stamp_ns=stamp,event=s.event,reason=s.exit_reason))
                    if cfg.resolved(i)[0].profile in ('immune_dynamic','impassive_static'):immune_ok &= not any(s.variables.values()) and s.surprise==0 and s.behavior.mode=='REGULAR'
                for d in response.motion:max_accel=max(max_accel,math.hypot(d.acceleration.x,d.acceleration.y))
                pp=[entity(p) for p in agents.agents]
                for j,p in enumerate(pp):
                    for r in rs.agents:minimum_gap=min(minimum_gap,math.hypot(p.x-r.position.position.x,p.y-r.position.position.y)-p.radius-r.radius)
                    for q in pp[j+1:]:minimum_human=min(minimum_human,math.hypot(p.x-q.x,p.y-q.y)-p.radius-q.radius)
                if k%4==0 or any(s.event for s in states.values()):trace.write(json.dumps(dict(stamp_ns=stamp,stage='complete' if seq.done else seq.cfg['stages'][seq.index]['name'],states={str(i):asdict(s) for i,s in states.items()},agents=proposal.diagnostics['agents'],people={str(p.id):asdict(p) for p in pp},robots={str(r.id):asdict(entity(r)) for r in rs.agents},motion=[motion_json(d) for d in response.motion]),allow_nan=False)+'\n')
                for n,(x,y,yaw) in positions.items():
                    v,w=commands[n];positions[n]=(x+v*math.cos(yaw+w*.0125)*.025,y+v*math.sin(yaw+w*.0125)*.025,math.remainder(yaw+w*.025,2*math.pi))
        coverage=set().union(*(set(v) for v in modes.values()))
        checks=dict(complete=seq.done,all_five_modes=coverage=={'REGULAR','SURPRISED','CURIOUS','SCARED','THREATENING'},immune=immune_ok,human_geometry=minimum_human>=0,acceleration=max_accel<=3.00001,playful13=phases[13]['occupy']>=320,playful14=phases[14]['occupy']>=20,source_unchanged=source==provenance(path,ROOT/'config/world/arena.yaml'))
        report=dict(passed=all(checks.values()),scope=__doc__,checks=checks,sim_s=duration,minimum_robot_gap_m=minimum_gap,minimum_human_gap_m=minimum_human,max_acceleration_mps2=max_accel,modes={i:dict(c) for i,c in modes.items()},phase_s={i:{k:v*.025 for k,v in c.items()} for i,c in phases.items()},peak_fear=peak_f,peak_social_fear=peak_social,events=events,provenance=source)
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k!='provenance'},indent=2));return int(not report['passed'])
    finally:
        if core is not None:
            os.killpg(core.pid,signal.SIGINT);core.wait(timeout=10)
        node.destroy_node();rclpy.shutdown()
if __name__=='__main__':raise SystemExit(main())
