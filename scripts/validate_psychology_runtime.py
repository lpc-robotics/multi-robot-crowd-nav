#!/usr/bin/env python3
"""Read-only actual Isaac geometry plus psychological task/transaction monitoring."""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import time

import rclpy
from std_msgs.msg import String
from arena_multi_control.scenario import load_scenario
from validate_multi_sfm_runtime import Probe, stamp
from multigoal_evidence import provenance


class PsychologyProbe(Probe):
    def __init__(self,scenario):
        super().__init__(scenario)
        self.last_psych_step=None;self.last_psych_stamp=None;self.psych_count=0;self.psych_seen=0.
        self.epoch=None;self.last_goals={};self.peaks={};self.modes={};self.paused_samples=0
        self.minimum_human_gap=math.inf;self.maximum_speed=0.;self.pause_checks=0
        self.maximum_observed_dt=0.;self.live_inputs=set();self.seen_config_hash=None
        self.transitions=deque(maxlen=1000)
        self.minimum_human_sample=None
        self.create_subscription(String,'/multirobot/hunav/psychology',self.psychology,100)

    def psychology(self,msg):
        try:
            data=json.loads(msg.data)
            if data.get('schema_version')!=2 or data.get('plugin',{}).get('model')!='sis_cf_v1':
                self.failures.add('unexpected psychology schema/model');return
            epoch,step,t=data['epoch'],data['step'],data['stamp_ns']
            if self.epoch is not None and epoch!=self.epoch:self.failures.add('psychology epoch changed')
            if self.last_psych_step is not None and (step!=self.last_psych_step+1 or t-self.last_psych_stamp!=25_000_000):
                self.failures.add('missing/nonmonotonic committed psychology step')
            self.epoch=epoch;self.last_psych_step=step;self.last_psych_stamp=t
            self.psych_count+=1;self.psych_seen=time.monotonic()
            if set(data['agents'])!={str(i) for i in range(1,len(self.radii)+1)}:
                self.failures.add('psychology identity changed')
            config_hash=data['plugin']['config_hash']
            if self.seen_config_hash is not None and self.seen_config_hash!=config_hash:self.failures.add('model configuration changed')
            self.seen_config_hash=config_hash
            for i,agent in data['agents'].items():
                for q,v in agent['new'].items():
                    if not math.isfinite(v) or not 0<=v<=1:self.failures.add('activation out of bounds')
                    self.peaks.setdefault(i,{}).setdefault(q,0.)
                    self.peaks[i][q]=max(self.peaks[i][q],v)
                mode=agent['behavior']['mode'];self.modes.setdefault(i,set()).add(mode)
                intent=agent['intent'];goals=data['task_goals'][i]
                if not intent['allow_task_progress']:
                    self.paused_samples+=1
                    if i in self.last_goals:
                        self.pause_checks+=1
                        if goals!=self.last_goals[i]:self.failures.add('temporary behavior advanced original queue')
                self.last_goals[i]=goals
                if agent['target_knowledge']=='none' and intent['has_target_robot']:
                    self.failures.add('directional intent without target knowledge')
                for name in agent['visible_robots']:self.live_inputs.add(name)
                if agent['behavior']['entered_ns']==t:
                    self.transitions.append({'stamp_ns':t,'agent_id':i,'mode':mode,'reason':agent['behavior']['reason']})
        except (ValueError,KeyError,TypeError) as error:self.failures.add('invalid psychology record:'+str(error))

    def people(self,msg):
        super().people(msg)
        for a in msg.agents:
            speed=math.hypot(a.velocity.linear.x,a.velocity.linear.y);self.maximum_speed=max(self.maximum_speed,speed)
            if speed>1.00001:self.failures.add('pedestrian speed above 1 m/s')

    def actual(self,msg):
        super().actual(msg)
        if self.started is None:return
        for index,a in enumerate(msg.pedestrians):
            if not .4<=a.pose.position.x<=29.6 or not .4<=a.pose.position.y<=22.6:
                self.failures.add('actual wall envelope violation')
            for b in msg.pedestrians[index+1:]:
                gap=math.hypot(a.pose.position.x-b.pose.position.x,a.pose.position.y-b.pose.position.y)-self.radii[a.name]-self.radii[b.name]
                if gap < self.minimum_human_gap:
                    self.minimum_human_gap=gap
                    self.minimum_human_sample={'stamp_ns':stamp(msg),'gap_m':gap,'pair':[a.name,b.name],
                        'positions':[[p.pose.position.x,p.pose.position.y] for p in (a,b)]}
                if gap<0:self.failures.add('actual human overlap')


def main():
    ap=argparse.ArgumentParser();ap.add_argument('scenario');ap.add_argument('--duration-wall',type=float,default=1800)
    ap.add_argument('--ready-timeout',type=float,default=300);ap.add_argument('--output',required=True);args=ap.parse_args()
    scenario=load_scenario(args.scenario)
    if scenario.psychology_model!='sis_cf_v1':raise SystemExit('sis_cf_v1 scenario required')
    root=Path(__file__).resolve().parents[1];before=provenance(args.scenario,root/'config/world/arena.yaml')
    rclpy.init();node=PsychologyProbe(scenario);deadline=time.monotonic()+args.ready_timeout
    try:
        while time.monotonic()<deadline and not (node.ready() and node.psych_count>=40):
            rclpy.spin_once(node,timeout_sec=.05)
            if node.failures:break
        if not (node.ready() and node.psych_count>=40):node.failures.add('readiness timeout')
        else:
            node.started=time.monotonic();sim_start=node.last_psych_stamp
            while time.monotonic()-node.started<args.duration_wall:
                rclpy.spin_once(node,timeout_sec=.02);node.check_geometry()
                if time.monotonic()-node.health_seen>.6:node.failures.add('health heartbeat missing')
                if time.monotonic()-node.psych_seen>1:node.failures.add('psychology stream missing')
                if node.failures:break
        drain=time.monotonic()+1
        node.destroy_subscription(next(s for s in node.subscriptions if s.topic_name=='/multirobot/hunav/actual_people'))
        while node.pending and time.monotonic()<drain:
            rclpy.spin_once(node,timeout_sec=.02);node.check_geometry()
        if node.pending or node.checked_samples<2:node.failures.add('insufficient matched actual samples')
        if node.min_clearance<.1:node.failures.add('actual robot clearance below 0.10 m')
        if node.max_display_error>.25:node.failures.add('actual display error above 0.25 m')
        if node.paused_samples==0 or node.pause_checks==0:node.failures.add('no temporary behavior observed')
        if set(node.live_inputs)!={str(-i) for i in range(1,len(scenario.robots)+1)}:node.failures.add('not all robots perceived')
        graph=[name for name,ns in node.get_node_names_and_namespaces()]
        if graph.count('multi_sfm_server')!=1 or graph.count('multi_sfm_adapter')!=1 or 'hunav_agent_manager' in graph:
            node.failures.add('nonunique psychology motion backend')
        after=provenance(args.scenario,root/'config/world/arena.yaml')
        if before!=after:node.failures.add('source/configuration changed during acceptance')
        report={'passed':not node.failures,'scope':'actual Isaac Character Graph and committed psychology',
                'wall_duration_s':0 if node.started is None else time.monotonic()-node.started,
                'sim_duration_s':0 if node.started is None else (node.last_psych_stamp-sim_start)/1e9,
                'requested_wall_duration_s':args.duration_wall,'provenance':before,
                'psychology_samples':node.psych_count,'actual_samples':node.actual_samples,'matched_samples':node.checked_samples,
                'skipped_samples':node.skipped_samples,'minimum_robot_gap_m':None if math.isinf(node.min_clearance) else node.min_clearance,
                'minimum_human_gap_m':None if math.isinf(node.minimum_human_gap) else node.minimum_human_gap,
                'minimum_human_sample':node.minimum_human_sample,
                'max_display_error_m':node.max_display_error,'max_speed_mps':node.maximum_speed,
                'modes':{i:sorted(v) for i,v in node.modes.items()},'peaks':node.peaks,'pause_checks':node.pause_checks,
                'transitions':list(node.transitions),'failures':sorted(node.failures),'failure_events':node.failures.events}
        path=Path(args.output);path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k not in ('provenance','transitions')},indent=2),flush=True)
        return 0 if report['passed'] else 1
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
