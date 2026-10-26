#!/usr/bin/env python3
"""Screen a changed route layout against recorded robot inputs, without Isaac.

Robot motion is fixed to an earlier run, so this is not a replacement for Nav2
and actual Character Graph acceptance of the changed crowd.
"""
import argparse
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'src/arena_multi_hunav')]
from validate_psychology_cpu import (rclpy, Node, Time, ComputeMultiAgentsV2,
    BehaviorModifiers, BehaviorIntent, MODES, robots, load_people, entity,
    DualSISPsychology, make_context, prepare_step, commit_candidate, call)
from arena_multi_hunav.psychology_config import load_psychology_config
from multigoal_evidence import provenance


def replay(node, client, path, duration):
    agents, _ = load_people(ROOT/'config/hunav/multi_regular_8_psychology_multigoal.yaml', world_bounds=(30,23))
    model = DualSISPsychology()
    cfg = load_psychology_config(ROOT/'config/psychology/sis_cf_v1.yaml')
    model.initialize(cfg.document(), tuple(a.id for a in agents.agents), 11)
    state = model.initial_states(); minimum_human = math.inf; minimum_robot = math.inf
    minimum_sample = None; count = 0; modes = {a.id:set() for a in agents.agents}
    with path.open() as stream:
        for line in stream:
            data=json.loads(line)
            if count*.025 >= duration: break
            stamp=count*25_000_000;count+=1
            values=[data['robots'][str(-i)] for i in range(1,3)]
            rs=robots(stamp,[(r['x'],r['y'],r['vx'],r['vy']) for r in values])
            agents.header.stamp=Time(nanoseconds=stamp).to_msg()
            proposal,snapshot=prepare_step(model,make_context(stamp,.025,[entity(a) for a in agents.agents],
                                                            [entity(r) for r in rs.agents]),state)
            request=ComputeMultiAgentsV2.Request(epoch=1,step=count,dt=.025,current_agents=agents,robots=rs)
            for i,mod in proposal.modifiers.items(): request.modifiers.append(BehaviorModifiers(agent_id=i,**asdict(mod)))
            for i,intent in proposal.intents.items():
                value=asdict(intent);value['mode']=MODES[intent.mode]
                request.intents.append(BehaviorIntent(agent_id=i,**value))
            agents,_,state=commit_candidate(model,snapshot,proposal,request,call(node,client,request))
            for index,a in enumerate(agents.agents):
                modes[a.id].add(proposal.intents[a.id].mode)
                for b in agents.agents[index+1:]:
                    gap=math.dist((a.position.position.x,a.position.position.y),
                                  (b.position.position.x,b.position.position.y))-a.radius-b.radius
                    if gap<minimum_human:
                        minimum_human=gap;minimum_sample={'time_s':count*.025,'pair':[a.name,b.name],
                            'modes':[proposal.intents[a.id].mode,proposal.intents[b.id].mode]}
                for b in rs.agents:
                    minimum_robot=min(minimum_robot,math.dist((a.position.position.x,a.position.position.y),
                                           (b.position.position.x,b.position.position.y))-a.radius-b.radius)
    return {'passed':count>0 and minimum_human>=0 and minimum_robot>=.1,'robot_trace':str(path),
            'duration_sim_s':count*.025,'minimum_human_gap_m':minimum_human,'minimum_robot_gap_m':minimum_robot,
            'minimum_human_sample':minimum_sample,'modes':{i:sorted(v) for i,v in modes.items()}}


def main():
    ap=argparse.ArgumentParser();ap.add_argument('traces',nargs='+',type=Path)
    ap.add_argument('--duration',type=float,default=300);ap.add_argument('--output',required=True,type=Path)
    args=ap.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='91':raise SystemExit('Use isolated ROS_DOMAIN_ID=91')
    before=provenance(ROOT/'config/scenarios/two_robots_multi_sfm_8_multigoal_psychology_nav2.yaml',ROOT/'config/world/arena.yaml')
    rclpy.init();node=Node('psychology_route_replay');client=node.create_client(ComputeMultiAgentsV2,'/psychology_route_replay/compute_v2')
    report={'passed':False,'scope':'fixed recorded robot inputs; no actual Nav2 feedback or display','provenance':before}
    with args.output.with_suffix('.log').open('w') as log:
        child=subprocess.Popen(['ros2','run','arena_multi_hunav_core','multi_sfm_server','--ros-args',
            '-r','__node:=psychology_route_replay_core','-r','/multirobot/hunav/compute_agents_v2:=/psychology_route_replay/compute_v2'],
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            if not client.wait_for_service(timeout_sec=20):raise ValueError('core unavailable')
            report['cases']=[replay(node,client,p,args.duration) for p in args.traces]
            report['passed']=all(c['passed'] for c in report['cases'])
        except Exception as error: report['error']=str(error)
        finally:
            os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
            node.destroy_node();rclpy.shutdown()
    after=provenance(ROOT/'config/scenarios/two_robots_multi_sfm_8_multigoal_psychology_nav2.yaml',ROOT/'config/world/arena.yaml')
    report['source_unchanged']=before==after;report['passed'] &= before==after
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='provenance'},indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
