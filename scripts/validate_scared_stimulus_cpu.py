#!/usr/bin/env python3
"""Check a geometric fear stimulus through the C++ integrator; no Isaac evidence."""
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
from arena_multi_control.fear_stimulus import load_stimulus, Protocol
from arena_multi_hunav.psychology_v2 import SocialPsychologyV2
from arena_multi_hunav.psychology_config import load_psychology_config
from arena_multi_hunav.psychology import prepare_step, make_context
from arena_multi_hunav.behavior_v2 import MODES_V2
from arena_multi_hunav.multi_adapter import load_people, entity, commit_candidate, motion_json
from validate_psychology_cpu import robots, call
from multigoal_evidence import provenance


def run(node, client, output, variant, body_fov=False, post_complete=0.):
    condition = 'body_fov' if body_fov else variant
    scenario = ROOT/f'config/scenarios/one_robot_multi_sfm_6_scared_{condition}_external.yaml'
    source = provenance(scenario, ROOT/'config/world/arena.yaml')
    cfg = load_stimulus(ROOT/f'config/tasks/scared_stimulus_{variant}.yaml')
    psychology = ROOT/('config/psychology/sis_cf_v2_scared_body_fov.yaml' if body_fov else
                       'config/psychology/sis_cf_v2_scared_stimulus.yaml')
    for path in (scenario, ROOT/f'config/tasks/scared_stimulus_{variant}.yaml',
                 ROOT/'config/hunav/multi_6_scared_stimulus.yaml', psychology):
        (output/path.name).write_bytes(path.read_bytes())
    protocol = Protocol(cfg)
    agents, _ = load_people(ROOT/'config/hunav/multi_6_scared_stimulus.yaml')
    model = SocialPsychologyV2()
    model.initialize(load_psychology_config(psychology).document(),
                     tuple(p.id for p in agents.agents), 11)
    states = model.initial_states()
    robot = [cfg.home_x, cfg.home_y, 0.]
    command = (0., 0.)
    stamp = 0
    modes = {p.id: Counter() for p in agents.agents}
    reasons = {p.id: Counter() for p in agents.agents}
    peaks = {p.id: 0. for p in agents.agents}
    direct = {p.id: 0. for p in agents.agents}
    min_gap = math.inf
    max_acceleration = 0.
    min_human = math.inf
    walls_ok = True
    completed_ns = None
    with (output/'closed_loop.jsonl').open('x') as stream:
        for k in range(int((150+post_complete)/.025)):
            target = next(p for p in agents.agents if p.name == cfg.target_person)
            gap = min(math.hypot(p.position.position.x-robot[0], p.position.position.y-robot[1])-p.radius-cfg.robot_radius for p in agents.agents)
            min_gap = min(min_gap, gap)
            command = protocol.step(stamp, tuple(robot), (target.position.position.x, target.position.position.y), target.radius, gap)
            rs = robots(stamp, [(robot[0], robot[1], command[0]*math.cos(robot[2]), command[0]*math.sin(robot[2]))])
            rs.agents[0].yaw = robot[2]
            agents.header.stamp = Time(nanoseconds=stamp).to_msg()
            proposal, snapshot = prepare_step(model, make_context(stamp,.025,[entity(p) for p in agents.agents],
                                           [entity(r) for r in rs.agents]), states)
            req = ComputeMultiAgentsV3.Request(epoch=1, step=k+1, dt=.025, current_agents=agents, robots=rs)
            for i, intent in proposal.intents.items():
                values = asdict(intent); values['mode'] = MODES_V2[intent.mode]
                req.intents.append(BehaviorIntentV3(agent_id=i, **values))
                req.modifiers.append(BehaviorModifiers(agent_id=i))
            response = call(node, client, req)
            agents, stamp, states = commit_candidate(model, snapshot, proposal, req, response)
            for i, s in states.items():
                modes[i][s.behavior.mode] += 1
                reasons[i][s.behavior.reason] += 1
                peaks[i] = max(peaks[i], s.variables['fear'])
                direct[i] = max(direct[i], proposal.diagnostics['agents'][i]['direct_F'])
            for d in response.motion:
                max_acceleration = max(max_acceleration, math.hypot(d.acceleration.x, d.acceleration.y))
            for n, p in enumerate(agents.agents):
                x, y = p.position.position.x, p.position.position.y
                walls_ok &= p.radius <= x <= 30-p.radius and p.radius <= y <= 23-p.radius
                for other in agents.agents[n+1:]:
                    min_human = min(min_human, math.dist((x,y),(other.position.position.x,other.position.position.y))-p.radius-other.radius)
            if k % 4 == 0:
                stream.write(json.dumps(dict(schema_version=3,epoch=1,step=k+1,stamp_ns=stamp,input_stamp_ns=stamp-25_000_000,
                    plugin=model.snapshot(),states={str(i):asdict(s) for i,s in states.items()},agents=proposal.diagnostics['agents'],
                    people={str(p.id):asdict(entity(p)) for p in agents.agents},robots={'-1':asdict(entity(rs.agents[0]))},
                    stimulus_phase=protocol.phase,motion=[motion_json(d) for d in response.motion]),allow_nan=False)+'\n')
            robot[0] += command[0]*math.cos(robot[2])*.025
            robot[1] += command[0]*math.sin(robot[2])*.025
            robot[2] = math.remainder(robot[2]+command[1]*.025, 2*math.pi)
            if protocol.phase == 'complete' and completed_ns is None:
                completed_ns = stamp
            if protocol.phase == 'failed' or (completed_ns is not None and (stamp-completed_ns)/1e9 >= post_complete):
                break
    checks = dict(protocol=protocol.phase=='complete',geometry=min_gap>=.1 and min_human>=0 and walls_ok,acceleration=max_acceleration<=3.0001,
                  scared=(True if body_fov else (modes[1]['SCARED']>=20 if variant=='intrusive' else modes[1]['SCARED']==0)),
                  immune=all(peaks[i]==0 and set(modes[i])=={'REGULAR'} for i in (5,6)))
    report = dict(passed=all(checks.values()),scope='CPU C++ closed loop; geometric robot point follower, no Isaac',
                  checks=checks,variant=variant,body_heading_fov_condition=body_fov,model_config=model.config.document(),
                  stimulus=asdict(cfg),phase=protocol.phase,error=protocol.error,
                  post_complete_sim_s=post_complete,
                  transitions=protocol.transitions,sim_s=stamp/1e9,min_gap_m=min_gap,min_human_gap_m=min_human,maximum_acceleration=max_acceleration,
                  modes=modes,reasons=reasons,peak_fear=peaks,peak_direct_F=direct,final_fear={i:s.variables['fear'] for i,s in states.items()},
                  provenance=source)
    (output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='provenance'},indent=2),flush=True)
    return report['passed']


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--output-dir', required=True)
    ap.add_argument('--variant', choices=('intrusive','considerate'), default='intrusive')
    ap.add_argument('--post-complete-sim-s',type=float,default=0.)
    ap.add_argument('--body-fov',action='store_true'); args = ap.parse_args()
    if args.body_fov and args.variant != 'intrusive': ap.error('--body-fov uses the intrusive stimulus')
    if not math.isfinite(args.post_complete_sim_s) or args.post_complete_sim_s < 0: ap.error('invalid post-complete duration')
    if os.environ.get('ROS_DOMAIN_ID') in (None,'51','71'):
        raise SystemExit('Use an isolated ROS domain')
    output = Path(args.output_dir); output.mkdir(parents=True,exist_ok=False)
    rclpy.init(); node = Node('scared_stimulus_cpu_probe')
    client = node.create_client(ComputeMultiAgentsV3, '/fear_validation/compute')
    with (output/'core.log').open('x') as log:
        core = subprocess.Popen(['ros2','run','arena_multi_hunav_core','multi_sfm_server','--ros-args',
            '-r','__node:=fear_validation_core','-r','/multirobot/hunav/compute_agents_v3:=/fear_validation/compute'],
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            if not client.wait_for_service(timeout_sec=20): raise RuntimeError('core unavailable')
            return int(not run(node,client,output,args.variant,args.body_fov,args.post_complete_sim_s))
        finally:
            os.killpg(core.pid,signal.SIGINT); core.wait(timeout=10); node.destroy_node(); rclpy.shutdown()


if __name__=='__main__': raise SystemExit(main())
