#!/usr/bin/env python3
"""Frozen causal experiments and closed-loop integration through a real v2 ROS service.

Only the private core process is started. No Isaac or robot actuator is used.
"""
import argparse
import copy
from dataclasses import asdict, replace
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Pose
from hunav_msgs.msg import Agent, Agents
from arena_multi_hunav_msgs.msg import BehaviorIntent, BehaviorModifiers
from arena_multi_hunav_msgs.srv import ComputeMultiAgentsV2

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src/arena_multi_hunav'), str(ROOT/'src/arena_multi_control')]
from arena_multi_hunav.psychology import Entity, make_context, prepare_step
from arena_multi_hunav.psychology_sis import DualSISPsychology
from arena_multi_hunav.psychology_config import PsychologyConfig
from arena_multi_hunav.behavior import Intent, MODES
from arena_multi_hunav.multi_adapter import load_people, entity, commit_candidate, motion_json
from multigoal_evidence import provenance


def frozen_spread(enhanced=False):
    cfg=replace(PsychologyConfig(),robot_visual_range=1.5,
                beta_C=1.2 if enhanced else .06,beta_F=2.4 if enhanced else .12)
    m=DualSISPsychology();m.initialize(cfg.document(),(1,2,3),11);state=m.initial_states()
    people=(Entity(1,'A',10,10,0,0,.4,((20.,10.),),math.pi),
            Entity(2,'B',11,10,0,0,.4,((20.,10.),),7*math.pi/6),
            Entity(3,'C',11,11,0,0,.4,((20.,11.),),0))
    robot=Entity(-1,'robot_1',9,10,0,0,.35)
    peaks={i:0. for i in (1,2,3)};curve=[];sources={2:0.,3:0.};unknown_ok=True
    for k in range(6000):
        proposal=m.step(make_context(k*25_000_000,.025,people,(robot,) if k<2000 else ()),state)
        state=proposal.states
        for i in (1,2,3):peaks[i]=max(peaks[i],state[i].variables['fear'])
        for i in (2,3):
            sources[i]=max(sources[i],proposal.diagnostics['agents'][i]['direct_F'])
            unknown_ok &= proposal.intents[i].mode=='REGULAR' and not proposal.intents[i].has_target_robot
        if k%20==0:
            curve.append({'time_s':(k+1)*.025,'fear':[state[i].variables['fear'] for i in (1,2,3)],
                          'vigilant':[state[i].behavior.vigilant for i in (1,2,3)]})
    passed=(unknown_ok and max(sources.values())==0 and
            (min(peaks[2],peaks[3])>.65 if enhanced else peaks[2]>.08))
    if not enhanced:passed &= max(s.variables['fear'] for s in state.values())<1e-5
    return {'passed':bool(passed),'scope':'fixed geometric directed chain, robot stimulates only A; no physical motion',
            'config':cfg.document(),'peak_fear':peaks,'indirect_direct_input':sources,'unknown_threat_boundary':unknown_ok,
            'final_fear':{i:s.variables['fear'] for i,s in state.items()},'curve':curve}


def robots(stamp, specs):
    result=Agents();result.header.frame_id='map';result.header.stamp=Time(nanoseconds=stamp).to_msg()
    for index,(x,y,vx,vy) in enumerate(specs,1):
        a=Agent();a.id=-index;a.name=f'robot_{index}';a.type=Agent.ROBOT;a.group_id=-1;a.radius=.35
        a.position.position.x=float(x);a.position.position.y=float(y);a.position.orientation.w=1.
        a.velocity.linear.x=float(vx);a.velocity.linear.y=float(vy);result.agents.append(a)
    return result


def robot_input(case,t):
    if case=='observed_nav2':
        if not hasattr(robot_input,'replay'):
            robot_input.replay=json.loads((ROOT/'src/arena_multi_hunav/test/fixtures/observed_nav2_crossing.json').read_text())['samples']
        samples=robot_input.replay;index=min(int(t/.2),len(samples)-2)
        start,a=samples[index];end,b=samples[index+1];fraction=max(0.,min(1.,(t-start)/(end-start)))
        return [tuple(x+(y-x)*fraction for x,y in zip(left,right)) for left,right in zip(a,b)]
    if case=='threat':return [(11.,10.,0.,0.)]
    if case=='single':
        if t<12:return [(12.5,10,0,0)]
        if t<22:return [(12.5-.26*(t-12),10,-.26,0)]
        if t<40:return [(9.9+.26*(t-22),10,.26,0)]
        return [(14.58,10,0,0)]
    if t<5:return [(3,3,0,0),(3,7,0,0)]
    if t<30:return [(3+.26*(t-5),3,.26,0),(3+.26*(t-5),7,.26,0)]
    return [(9.5,3,0,0),(9.5,7,0,0)]


def call(node,client,req):
    future=client.call_async(req);rclpy.spin_until_future_complete(node,future,timeout_sec=3.)
    if not future.done():raise ValueError('private_core_timeout')
    result=future.result()
    if not result.success:raise ValueError('private_core_failed:'+result.error)
    return result


def closed_loop(node,client,case,outdir,noop=False,standoff=None):
    one=case in ('single','threat')
    config=ROOT/'config/hunav'/('multi_regular_1.yaml' if one else 'multi_regular_6_psychology.yaml')
    agents,_=load_people(config,world_bounds=(30,23));m=DualSISPsychology()
    from arena_multi_hunav.psychology_config import load_psychology_config
    cfg=load_psychology_config(ROOT/'config/psychology/sis_cf_v1.yaml')
    if standoff is not None:cfg=replace(cfg,curiosity_standoff_gap=standoff)
    m.initialize(cfg.document(),tuple(a.id for a in agents.agents),11);state=m.initial_states()
    warm_steps=320 if case=='threat' else 160
    if one:
        a=agents.agents[0];a.position.position.x=10.;a.position.position.y=10.;a.yaw=0.
        goal=Pose();goal.position.x=20.;goal.position.y=10.;goal.orientation.w=1.
        a.goals=[goal];a.cyclic_goals=False
        # Controlled frozen observation, model-generated activation/mode, never direct state assignment.
        if not noop:
            for k in range(warm_steps):
                proposal=m.step(make_context(k*25_000_000,.025,[entity(a)],
                    [entity(r) for r in robots(k*25_000_000,robot_input(case,0)).agents]),state)
                state=proposal.states
    stamp=warm_steps*25_000_000 if one else 0
    duration=60 if one else (120 if case=='observed_nav2' else 45)
    modes={a.id:set() for a in agents.agents};peaks={a.id:{'curiosity':0.,'fear':0.} for a in agents.agents}
    minimum_gap=math.inf;minimum_human_gap=math.inf;peak_speed=0.;peak_acceleration=0.;paused=0;switches=0
    last_modes={i:s.behavior.mode for i,s in state.items()};trace=outdir/f'{case}_{"noop" if noop else "sis"}.jsonl'
    standoff_samples=0
    with trace.open('w') as stream:
        for k in range(round(duration/.025)):
            agents.header.stamp=Time(nanoseconds=stamp).to_msg()
            rs=robots(stamp,robot_input(case,k*.025))
            if noop:
                from arena_multi_hunav.psychology import Proposal, Modifiers
                proposal=Proposal(copy.deepcopy(state),{i:Modifiers() for i in state},{i:Intent() for i in state})
                snapshot=m.snapshot()
            else:
                proposal,snapshot=prepare_step(m,make_context(stamp,.025,[entity(a) for a in agents.agents],
                                                        [entity(r) for r in rs.agents]),state)
            req=ComputeMultiAgentsV2.Request(epoch=1,step=k+1,dt=.025,current_agents=agents,robots=rs)
            for i,modifier in sorted(proposal.modifiers.items()):req.modifiers.append(BehaviorModifiers(agent_id=i,**asdict(modifier)))
            for i,intent in sorted(proposal.intents.items()):
                values=asdict(intent);values['mode']=MODES[intent.mode];req.intents.append(BehaviorIntent(agent_id=i,**values))
            response=call(node,client,req)
            before=agents
            agents,stamp,state=commit_candidate(m,snapshot,proposal,req,response)
            changed=False
            for a in agents.agents:
                mode=proposal.intents[a.id].mode;modes[a.id].add(mode)
                changed |= mode!=last_modes[a.id];switches+=int(mode!=last_modes[a.id]);last_modes[a.id]=mode
                paused+=int(not proposal.intents[a.id].allow_task_progress)
                speed=math.hypot(a.velocity.linear.x,a.velocity.linear.y);peak_speed=max(peak_speed,speed)
                old=next(b for b in before.agents if b.id==a.id)
                accel=math.hypot(a.velocity.linear.x-old.velocity.linear.x,a.velocity.linear.y-old.velocity.linear.y)/.025
                peak_acceleration=max(peak_acceleration,accel)
                for q in ('curiosity','fear'):peaks[a.id][q]=max(peaks[a.id][q],state[a.id].variables[q])
                for r in rs.agents:
                    minimum_gap=min(minimum_gap,math.hypot(a.position.position.x-r.position.position.x,
                        a.position.position.y-r.position.position.y)-a.radius-r.radius)
                    gap=math.hypot(a.position.position.x-r.position.position.x,a.position.position.y-r.position.position.y)-a.radius-r.radius
                    if case=='single' and k*.025<12 and mode=='CURIOUS' and abs(gap-cfg.curiosity_standoff_gap)<=cfg.curiosity_stop_band and speed<=.1:
                        standoff_samples+=1
                for b in agents.agents:
                    if b.id!=a.id:minimum_human_gap=min(minimum_human_gap,math.hypot(
                        a.position.position.x-b.position.position.x,a.position.position.y-b.position.position.y)-a.radius-b.radius)
            if k%4==0 or changed:
                stream.write(json.dumps({'stamp_ns':stamp,'time_s':(k+1)*.025,'step':k+1,
                    'states':{i:asdict(s) for i,s in state.items()},'diagnostics':proposal.diagnostics,
                    'motion':[motion_json(d) for d in response.motion],
                    'people':{a.id:{'position':[a.position.position.x,a.position.position.y],
                        'velocity':[a.velocity.linear.x,a.velocity.linear.y],
                        'goals':[[g.position.x,g.position.y] for g in a.goals]} for a in agents.agents},
                    'robots':[[r.position.position.x,r.position.position.y] for r in rs.agents]},allow_nan=False)+'\n')
    checks={'finite_bounded':peak_speed<=1.00001,'acceleration_bound':peak_acceleration<=3.0001,
            'human_robot_gap':minimum_gap>=.1,'no_human_overlap':minimum_human_gap>=0}
    if not noop:
        checks['psychology_changes_motion']=paused>0
    if case=='single' and not noop:
        checks['emergent_curiosity_and_task_resume']={'CURIOUS','REGULAR'}<=modes[1]
        checks['actual_standoff_observed']=standoff_samples>=80
    if case=='threat' and not noop:
        checks['emergent_scared_motion']='SCARED' in modes[1]
    return {'passed':all(checks.values()),'checks':checks,'case':case,'noop':noop,'duration_sim_s':duration,
            'initialization':f'{warm_steps*.025} s frozen observation' if one and not noop else 'zero activation',
            'model_config':cfg.document(),
            'modes':{i:sorted(values) for i,values in modes.items()},'peaks':peaks,
            'minimum_robot_gap_m':minimum_gap,'minimum_human_gap_m':None if math.isinf(minimum_human_gap) else minimum_human_gap,
            'peak_speed_mps':peak_speed,'peak_acceleration_mps2':peak_acceleration,'paused_agent_steps':paused,
            'switches':switches,'trace':trace.name,'trace_sha256':hashlib.file_digest(trace.open('rb'),'sha256').hexdigest()}


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output-dir',required=True)
    ap.add_argument('--calibration-standoff',type=float);args=ap.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') in (None,'51','71'):raise SystemExit('use an isolated ROS domain')
    out=Path(args.output_dir).resolve();out.mkdir(parents=True,exist_ok=True)
    before=provenance(ROOT/'config/scenarios/two_robots_multi_sfm_6_multigoal_psychology_nav2.yaml',ROOT/'config/world/arena.yaml')
    rclpy.init();node=Node('psychology_cpu_validation')
    client=node.create_client(ComputeMultiAgentsV2,'/psychology_validation/compute_v2')
    log=(out/'core.log').open('w');child=subprocess.Popen(['ros2','run','arena_multi_hunav_core','multi_sfm_server','--ros-args',
        '-r','__node:=psychology_validation_core','-r','/multirobot/hunav/compute_agents_v2:=/psychology_validation/compute_v2'],
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    report={'passed':False,'scope':'CPU/frozen mechanisms and real ROS v2 core, no Isaac acceptance','provenance':before}
    try:
        if not client.wait_for_service(timeout_sec=20.):raise ValueError('private service not ready')
        report['frozen_default']=frozen_spread()
        report['frozen_enhanced']=frozen_spread(True)
        report['closed_loop']=[closed_loop(node,client,'single',out,standoff=args.calibration_standoff),
                               closed_loop(node,client,'threat',out,standoff=args.calibration_standoff),
                               closed_loop(node,client,'six',out,standoff=args.calibration_standoff),
                               closed_loop(node,client,'observed_nav2',out,standoff=args.calibration_standoff),
                               closed_loop(node,client,'six',out,True,standoff=args.calibration_standoff)]
        after=provenance(ROOT/'config/scenarios/two_robots_multi_sfm_6_multigoal_psychology_nav2.yaml',ROOT/'config/world/arena.yaml')
        report['source_unchanged']=before==after
        report['passed']=before==after and report['frozen_default']['passed'] and report['frozen_enhanced']['passed'] and all(r['passed'] for r in report['closed_loop'])
    except Exception as error:
        report['error']=str(error)
    finally:
        node.destroy_node();rclpy.shutdown()
        if child.poll() is None:
            os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=5.)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        log.close()
    (out/'cpu.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('provenance','frozen_default','frozen_enhanced')},indent=2),flush=True)
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
