#!/usr/bin/env python3
"""Actual Character Graph V2 probe. Core/mode labels alone cannot pass gaze checks."""
import argparse
from collections import deque, Counter
import json
import math
from pathlib import Path
import time

import rclpy
from std_msgs.msg import String
from arena_multi_control.scenario import load_scenario
from validate_multi_sfm_runtime import Probe, stamp
from multigoal_evidence import provenance


class V2Probe(Probe):
    def __init__(self, scenario, output):
        super().__init__(scenario)
        self.psych = deque(maxlen=600)
        self.modes = {}; self.events = Counter(); self.phase_counts = Counter()
        self.gaze_s = {}; self.front_s = {}; self.last_actual = None
        self.min_human = math.inf; self.max_speed = 0.; self.psych_seen = None
        self.last_goals = {}; self.pause_checks = 0; self.cycles = Counter()
        self.initial_goals = {}; self.last_commit = None; self.last_epoch = None
        self.first_stamp = None; self.last_stamp = None; self.gaze_current = {}
        self.occupy_s = {}; self.robot_response = {}; self.core_max_speed = 0.; self.core_max_acceleration = 0.
        self.actual_stream = output.with_suffix('.actual.jsonl').open('x')
        self.create_subscription(String, '/multirobot/hunav/psychology', self.mental, 100)

    def mental(self, msg):
        data = json.loads(msg.data)
        if data.get('schema_version') != 3 or data.get('plugin', {}).get('model') != 'sis_cf_v2':
            self.failures.add('wrong V2 schema/model'); return
        self.psych_seen = time.monotonic()
        t = data['stamp_ns']; self.last_stamp = t
        if self.first_stamp is None: self.first_stamp = t
        if self.last_commit is not None and (data['step'] != self.last_commit+1 or data['epoch'] != self.last_epoch):
            self.failures.add('missing or changed V2 transaction')
        self.last_commit = data['step']; self.last_epoch = data['epoch']
        self.psych.append(data)
        for p in data['people'].values():
            self.core_max_speed=max(self.core_max_speed,math.hypot(p['vx'],p['vy']))
        for d in data['motion']:
            self.core_max_acceleration=max(self.core_max_acceleration,math.hypot(*d['acceleration']))
        for key, state in data['states'].items():
            info = data['agents'][key]; mode = state['behavior']['mode']
            self.modes.setdefault(key, set()).add(mode)
            if state.get('event'): self.events[(key, state['event'])] += 1
            self.phase_counts[(key, state.get('phase', 'none'))] += 1
            if info['profile'] in ('immune_dynamic', 'impassive_static'):
                if any(state['variables'].values()) or state['surprise'] or mode != 'REGULAR':
                    self.failures.add('immune psychology changed')
            if mode == 'THREATENING' and info['profile'] != 'playful':
                self.failures.add('ineligible blocking')
            goals = data['task_goals'][key]
            self.initial_goals.setdefault(key, goals)
            if key in self.last_goals:
                if not info['intent']['allow_task_progress']:
                    self.pause_checks += 1
                    if goals != self.last_goals[key]: self.failures.add('temporary task advanced')
                elif goals != self.last_goals[key] and goals == self.initial_goals[key]:
                    self.cycles[key] += 1
            self.last_goals[key] = goals

    def actual(self, msg):
        super().actual(msg)
        t = stamp(msg)
        record = {'stamp_ns': t, 'frame_id': msg.header.frame_id, 'source': 'isaac_actual', 'people': {}}
        mental = next((r for r in reversed(self.psych) if r['stamp_ns'] <= t), None)
        dt = 0. if self.last_actual is None else (t-self.last_actual)/1e9
        self.last_actual = t
        for a in msg.pedestrians:
            q = a.pose.orientation; p = a.pose.position
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            vx, vy = a.twist.linear.x, a.twist.linear.y
            record['people'][a.name] = dict(x=p.x, y=p.y, yaw=yaw, vx=vx, vy=vy,
                                           velocity_source='finite_difference_actual_pose')
            self.max_speed = max(self.max_speed, math.hypot(vx, vy))
            radius = self.radii.get(a.name, .4)
            if not radius <= p.x <= 30-radius or not radius <= p.y <= 23-radius:
                self.failures.add('actual wall overlap')
            if mental is None or t-mental['stamp_ns'] > 250_000_000: continue
            key = next((k for k, person in mental['people'].items() if person['name'] == a.name), None)
            if key is None: continue
            state = mental['states'][key]; b = state['behavior']; target = str(b['target_robot_id'])
            robot = mental['robots'].get(target)
            aligned = False
            if robot and b['mode'] == 'SURPRISED':
                angle = abs(math.remainder(math.atan2(robot['y']-p.y, robot['x']-p.x)-yaw, 2*math.pi))
                aligned = math.hypot(vx, vy) <= .05 and angle <= math.radians(15)
            self.gaze_current[key] = self.gaze_current.get(key, 0.)+dt if aligned and 0 < dt <= .15 else 0.
            self.gaze_s[key] = max(self.gaze_s.get(key, 0.), self.gaze_current[key])
            if robot and b['mode'] == 'THREATENING':
                speed = math.hypot(robot['vx'], robot['vy'])
                heading=math.atan2(robot['vy'],robot['vx']) if speed>=.05 else state['robot_heading']
                dx, dy = p.x-robot['x'], p.y-robot['y']
                front = dx*math.cos(heading)+dy*math.sin(heading)
                lateral = abs(-dx*math.sin(heading)+dy*math.cos(heading))
                if front > radius+robot['radius']+.1 and lateral <= radius+robot['radius']:
                    self.front_s[key] = self.front_s.get(key, 0.)+max(0., min(.15, dt))
                    if state['phase']=='occupy' and state['target_point'] and math.dist((p.x,p.y),state['target_point'])<=.35:
                        self.occupy_s[key]=self.occupy_s.get(key,0.)+max(0.,min(.15,dt))
                response=self.robot_response.setdefault(key,dict(samples=0,speed_sum=0.,minimum_speed=math.inf,stopped_s=0.))
                response['samples']+=1;response['speed_sum']+=speed;response['minimum_speed']=min(response['minimum_speed'],speed)
                if speed<.05:response['stopped_s']+=max(0.,min(.15,dt))
        for i, a in enumerate(msg.pedestrians):
            for b in msg.pedestrians[i+1:]:
                gap = math.hypot(a.pose.position.x-b.pose.position.x, a.pose.position.y-b.pose.position.y)-self.radii[a.name]-self.radii[b.name]
                self.min_human = min(self.min_human, gap)
                if gap < 0: self.failures.add('actual human overlap')
        self.actual_stream.write(json.dumps(record, allow_nan=False)+'\n')


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('scenario'); ap.add_argument('--output', required=True)
    ap.add_argument('--duration-wall', type=float, default=60); ap.add_argument('--ready-timeout', type=float, default=300)
    ap.add_argument('--require-gaze', nargs='*', default=[]); ap.add_argument('--require-playful', nargs='*', default=[])
    ap.add_argument('--run-dir'); ap.add_argument('--require-demo',action='store_true')
    ap.add_argument('--until-demo-complete',action='store_true')
    args = ap.parse_args(); output = Path(args.output); output.parent.mkdir(parents=True, exist_ok=True)
    root = Path(__file__).resolve().parents[1]
    before = provenance(args.scenario, root/'config/world/arena.yaml')
    rclpy.init(); node = V2Probe(load_scenario(args.scenario), output)
    deadline = time.monotonic()+args.ready_timeout
    try:
        while time.monotonic() < deadline and not (node.ready() and node.psych):
            rclpy.spin_once(node, timeout_sec=.05)
            if node.failures: break
        if not node.ready(): node.failures.add('readiness timeout')
        else:
            node.started = time.monotonic()
            maximum=args.duration_wall+(1200 if args.until_demo_complete else 0)
            while time.monotonic()-node.started < maximum:
                rclpy.spin_once(node, timeout_sec=.02); node.check_geometry()
                if node.psych_seen and time.monotonic()-node.psych_seen > 1: node.failures.add('psychology missing')
                if time.monotonic()-node.health_seen > .6: node.failures.add('health missing')
                if node.failures: break
                if time.monotonic()-node.started>=args.duration_wall:
                    demo_path=Path(args.run_dir)/'psychology_demo.json' if args.run_dir else None
                    if not args.until_demo_complete or (demo_path and demo_path.exists()):break
        if node.checked_samples < 2: node.failures.add('insufficient actual geometry')
        if node.min_clearance < .1: node.failures.add('robot gap below .10m')
        if node.max_display_error > .25: node.failures.add('display error above .25m')
        if node.core_max_speed>1.00001 or node.core_max_acceleration>3.00001: node.failures.add('core motion bound exceeded')
        for key in args.require_gaze:
            if node.gaze_s.get(key, 0.) < 1 or not node.events[(key, 'surprise_completed')]:
                node.failures.add('missing actual stopped aligned observation: '+key)
        for key in args.require_playful:
            if node.occupy_s.get(key, 0.) < .25 or not node.events[(key, 'playful_completed')] or not node.events[(key, 'playful_task_resumed')]:
                node.failures.add('missing actual forward occupancy and completed yield: '+key)
        demo=None
        if args.run_dir and (Path(args.run_dir)/'psychology_demo.json').exists():
            demo=json.loads((Path(args.run_dir)/'psychology_demo.json').read_text())
        if args.require_demo and (not demo or not demo['passed']): node.failures.add('finite Nav2 demonstration incomplete')
        if provenance(args.scenario, root/'config/world/arena.yaml') != before:
            node.failures.add('source changed during validation')
        report = dict(passed=not node.failures, scope='actual Character Graph and committed V3 transactions',
            requested_wall_s=args.duration_wall, wall_s=0 if node.started is None else time.monotonic()-node.started,
            sim_s=0 if node.first_stamp is None else (node.last_stamp-node.first_stamp)/1e9,
            actual_samples=node.actual_samples, matched_samples=node.checked_samples,
            minimum_robot_gap_m=None if math.isinf(node.min_clearance) else node.min_clearance,
            minimum_human_gap_m=None if math.isinf(node.min_human) else node.min_human,
            max_display_error_m=node.max_display_error, max_actual_speed_mps=node.max_speed,
            core_max_speed_mps=node.core_max_speed, core_max_acceleration_mps2=node.core_max_acceleration,
            modes={k: sorted(v) for k,v in node.modes.items()}, actual_aligned_observation_s=node.gaze_s,
            actual_front_occupancy_s=node.front_s, events={f'{k[0]}:{k[1]}':v for k,v in node.events.items()},
            actual_occupy_at_target_s=node.occupy_s, robot_during_interaction=node.robot_response, demo=demo,
            phases={f'{k[0]}:{k[1]}':v for k,v in node.phase_counts.items()}, cycles=dict(node.cycles),
            pause_checks=node.pause_checks, last_status=node.status[-1] if node.status else None,
            failures=sorted(node.failures), provenance=before)
        output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k != 'provenance'}, indent=2), flush=True)
        return int(not report['passed'])
    finally:
        node.actual_stream.close(); node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': raise SystemExit(main())
