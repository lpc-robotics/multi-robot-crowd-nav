#!/usr/bin/env python3
"""Single-actor slice of showcase: C++ physical integration, prescribed robot; no Nav2/Isaac claim."""
import argparse
from collections import Counter
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import yaml
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from arena_multi_hunav_msgs.msg import BehaviorIntentV3, BehaviorModifiers
from arena_multi_hunav_msgs.srv import ComputeMultiAgentsV3
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src/arena_multi_hunav'),str(ROOT/'src/arena_multi_control')]
from arena_multi_hunav.psychology_v2 import SocialPsychologyV2
from arena_multi_hunav.psychology import prepare_step, make_context
from arena_multi_hunav.behavior_v2 import MODES_V2
from arena_multi_hunav.multi_adapter import load_people, entity, commit_candidate, motion_json
from validate_psychology_cpu import robots, call
from multigoal_evidence import provenance

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output-dir',required=True);ap.add_argument('--person-id',type=int,choices=(13,14),default=13);a=ap.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') in (None,'51','71'):raise SystemExit('Use an isolated ROS domain')
    out=Path(a.output_dir);out.mkdir(parents=True,exist_ok=False)
    cfg=yaml.safe_load((ROOT/'config/psychology/sis_cf_v2_playful_showcase.yaml').read_text())
    cfg['individuals']={a.person_id:cfg['individuals'][a.person_id]}
    scenario=ROOT/'config/scenarios/one_robot_multi_sfm_2_playful_v2_nav2.yaml'
    provenance_before=provenance(scenario,ROOT/'config/world/arena.yaml')
    speed=yaml.safe_load(scenario.read_text())['max_linear']
    agents,_=load_people(ROOT/'config/hunav/multi_2_playful_v2_showcase.yaml')
    agents.agents=[p for p in agents.agents if p.id==a.person_id]
    model=SocialPsychologyV2();model.initialize(cfg,(a.person_id,),11);states=model.initial_states()
    rclpy.init();node=Node('specialist_cpu');client=node.create_client(ComputeMultiAgentsV3,'/specialist_cpu/compute')
    core=None;events=[];phases=Counter();min_gap=math.inf;max_accel=0.;front_s=0.;stamp=0
    try:
        with (out/'core.log').open('x') as log, (out/'trace.jsonl').open('x') as trace:
            core=subprocess.Popen(['ros2','run','arena_multi_hunav_core','multi_sfm_server','--ros-args','-r','__node:=specialist_cpu_core','-r','/multirobot/hunav/compute_agents_v3:=/specialist_cpu/compute'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            if not client.wait_for_service(timeout_sec=20):raise RuntimeError('core unavailable')
            for k in range(4800):
                t=k*.025;y=5. if a.person_id==13 else 15.
                rs=robots(stamp,[(9.+speed*max(0.,min(t-8.,70.)),y,speed if 8<=t<78 else 0.,0.)])
                agents.header.stamp=Time(nanoseconds=stamp).to_msg()
                prop,snapshot=prepare_step(model,make_context(stamp,.025,[entity(p) for p in agents.agents],[entity(r) for r in rs.agents]),states)
                req=ComputeMultiAgentsV3.Request(epoch=1,step=k+1,dt=.025,current_agents=agents,robots=rs)
                for i,intent in prop.intents.items():
                    values=asdict(intent);values['mode']=MODES_V2[intent.mode]
                    req.intents.append(BehaviorIntentV3(agent_id=i,**values));req.modifiers.append(BehaviorModifiers(agent_id=i))
                old=entity(agents.agents[0]);res=call(node,client,req)
                agents,stamp,states=commit_candidate(model,snapshot,prop,req,res)
                s=states[a.person_id];phases[s.phase]+=1
                if s.event:events.append(dict(sim_s=t+.025,event=s.event,reason=s.exit_reason,outcome=s.playful_outcome))
                p=entity(agents.agents[0]);r=entity(rs.agents[0])
                min_gap=min(min_gap,math.hypot(p.x-r.x,p.y-r.y)-p.radius-r.radius)
                max_accel=max(max_accel,math.hypot(p.vx-old.vx,p.vy-old.vy)/.025)
                if s.phase=='occupy' and p.x-r.x>0. and abs(p.y-r.y)<p.radius+r.radius:front_s+=.025
                if k%4==0 or s.event:trace.write(json.dumps(dict(stamp_ns=stamp,states={str(a.person_id):asdict(s)},people={str(a.person_id):asdict(p)},robots={'-1':asdict(r)},motion=[motion_json(d) for d in res.motion]))+'\n')
        counts=Counter(e['event'] for e in events)
        checks=dict(finite_geometry=math.isfinite(min_gap),acceleration=max_accel<=3.00001,observation=counts['surprise_completed']>=1,long_actual_core_occupancy=front_s>=5.,completed=counts['playful_completed']>=1,task_return=counts['playful_task_resumed']>=1,source_unchanged=provenance_before==provenance(scenario,ROOT/'config/world/arena.yaml'))
        report=dict(passed=all(checks.values()),scope=__doc__,checks=checks,person_id=a.person_id,events=events,phase_s={k:v*.025 for k,v in phases.items()},front_occupancy_s=front_s,min_gap_m=min_gap,max_acceleration_mps2=max_accel,config=cfg,provenance=provenance_before)
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k not in ('provenance','config')},indent=2));return int(not report['passed'])
    finally:
        if core is not None:
            os.killpg(core.pid,signal.SIGINT);core.wait(timeout=10)
        node.destroy_node();rclpy.shutdown()
if __name__=='__main__':raise SystemExit(main())
