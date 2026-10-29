#!/usr/bin/env python3
"""V2 closed-loop mechanisms via the actual ROS/C++ integrator, without Isaac."""
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

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from arena_multi_hunav_msgs.msg import BehaviorIntentV3, BehaviorModifiers
from arena_multi_hunav_msgs.srv import ComputeMultiAgentsV3

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src/arena_multi_hunav'), str(ROOT/'src/arena_multi_control')]
from arena_multi_hunav.psychology_v2 import SocialPsychologyV2
from arena_multi_hunav.psychology_v2_config import PsychologyConfigV2, ParametersV2, Individual
from arena_multi_hunav.psychology import prepare_step, make_context
from arena_multi_hunav.behavior_v2 import MODES_V2
from arena_multi_hunav.multi_adapter import load_people, entity, commit_candidate, motion_json
from validate_psychology_cpu import robots, call
from multigoal_evidence import provenance


def run(node, client, output, disabled=False):
    source = provenance(ROOT/'config/scenarios/one_robot_multi_sfm_1_playful_v2_validation.yaml', ROOT/'config/world/arena.yaml')
    (output/'validator_source.py').write_text(Path(__file__).read_text())
    (output/'pedestrian_input.yaml').write_text((ROOT/'config/hunav/multi_playful_1_v2_validation.yaml').read_text())
    agents, _ = load_people(ROOT/'config/hunav/multi_playful_1_v2_validation.yaml')
    a = agents.agents[0]; a.position.position.x=12.; a.position.position.y=11.5; a.yaw=math.atan2(-1.5, -3.)
    a.position.orientation.z=math.sin(a.yaw/2); a.position.orientation.w=math.cos(a.yaw/2)
    model = SocialPsychologyV2()
    cfg = PsychologyConfigV2(ParametersV2(playful_enabled=float(not disabled)),
        {1: Individual('playful', -1, {'memory_tau': 2., 'memory_ttl': 4.})})
    model.initialize(cfg.document(), (1,), 11); states=model.initial_states()
    stamp=0; events=Counter(); modes=Counter(); phases=Counter(); blockers=Counter()
    min_gap=math.inf; max_acceleration=0.; front_s=0.
    trace=output/'closed_loop.jsonl'
    with trace.open('x') as stream:
        for k in range(2400):
            t=k*.025
            # Only robot trajectory is supplied. No mental state or pedestrian mode assignment.
            rs=robots(stamp, [(9.+.15*max(0.,min(t-8.,40.)), 10., .15 if 8.<=t<48. else 0., 0.)])
            agents.header.stamp=Time(nanoseconds=stamp).to_msg()
            proposal,snapshot=prepare_step(model,make_context(stamp,.025,[entity(p) for p in agents.agents],
                [entity(r) for r in rs.agents]),states)
            req=ComputeMultiAgentsV3.Request(epoch=1,step=k+1,dt=.025,current_agents=agents,robots=rs)
            for i, intent in proposal.intents.items():
                values=asdict(intent); values['mode']=MODES_V2[intent.mode]
                req.intents.append(BehaviorIntentV3(agent_id=i,**values))
                req.modifiers.append(BehaviorModifiers(agent_id=i))
            before=agents
            response=call(node,client,req)
            agents,stamp,states=commit_candidate(model,snapshot,proposal,req,response)
            s=states[1]; modes[s.behavior.mode]+=1; phases[s.phase]+=1; blockers.update(s.playful_blockers)
            if s.event: events[s.event]+=1
            p=entity(agents.agents[0]); r=entity(rs.agents[0]); old=entity(before.agents[0])
            min_gap=min(min_gap,math.hypot(p.x-r.x,p.y-r.y)-p.radius-r.radius)
            max_acceleration=max(max_acceleration,math.hypot(p.vx-old.vx,p.vy-old.vy)/.025)
            if s.behavior.mode=='THREATENING' and p.x-r.x>p.radius+r.radius+.1 and abs(p.y-r.y)<p.radius+r.radius:
                front_s+=.025
            if k%4==0 or s.event:
                stream.write(json.dumps(dict(schema_version=3,epoch=1,step=k+1,stamp_ns=stamp,
                    plugin=model.snapshot(),states={'1':asdict(s)},agents=proposal.diagnostics['agents'],
                    people={'1':asdict(p)},robots={'-1':asdict(r)},motion=[motion_json(d) for d in response.motion]),allow_nan=False)+'\n')
    checks=dict(geometry=min_gap>=.1, acceleration=max_acceleration<=3.0001,
        surprise=events['surprise_completed']>=1, playful=(events['playful_started']==0 if disabled else
        events['playful_started']==1 and events['playful_completed']==1 and events['playful_task_resumed']==1 and front_s>0))
    report=dict(passed=all(checks.values()),scope='C++ ROS closed loop; prescribed robot motion, no Isaac/Nav2 response',
        checks=checks,events=dict(events),modes=dict(modes),phases=dict(phases),blockers=dict(blockers),
        min_gap_m=min_gap,max_acceleration_mps2=max_acceleration,front_s=front_s,config=cfg.document(),provenance=source)
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('config','provenance')},indent=2),flush=True)
    return report['passed']


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output-dir',required=True);ap.add_argument('--disabled',action='store_true');args=ap.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') in (None,'51','71'):raise SystemExit('Use an isolated ROS domain')
    output=Path(args.output_dir);output.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=Node('psychology_v2_cpu_probe')
    client=node.create_client(ComputeMultiAgentsV3,'/v2_validation/compute')
    with (output/'core.log').open('x') as log:
        core=subprocess.Popen(['ros2','run','arena_multi_hunav_core','multi_sfm_server','--ros-args',
            '-r','__node:=v2_validation_core','-r','/multirobot/hunav/compute_agents_v3:=/v2_validation/compute'],
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            if not client.wait_for_service(timeout_sec=20):raise RuntimeError('core unavailable')
            return int(not run(node,client,output,args.disabled))
        finally:
            os.killpg(core.pid,signal.SIGINT);core.wait(timeout=10);node.destroy_node();rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
