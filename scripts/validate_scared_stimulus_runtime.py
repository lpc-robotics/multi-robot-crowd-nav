#!/usr/bin/env python3
"""Actual fear/retreat evidence; record robot proximity separately from model execution."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import time

import rclpy
from arena_multi_control.scenario import load_scenario
from arena_multi_control.fear_stimulus import load_stimulus
from validate_psychology_v2_runtime import V2Probe
from validate_multi_sfm_runtime import stamp
from multigoal_evidence import provenance


class FearProbe(V2Probe):
    def __init__(self, scenario, output, target, robot_clearance_policy='record'):
        self.target_name = target
        self.target_id = None
        self.peaks = {}
        self.last_fear = {}
        self.reasons = Counter()
        self.scared_s = Counter()
        self.fear_triggered_s = Counter()
        self.social_peaks = {}
        self.direct_peaks = {}
        self.away_s = 0.
        self.away_distance = 0.
        self.last_retreat = None
        self.task_resumed_samples = 0
        super().__init__(scenario, output, robot_clearance_policy)

    def mental(self, msg):
        super().mental(msg)
        if not self.psych:
            return
        row = self.psych[-1]
        for key, state in row['states'].items():
            f = state['variables']['fear']
            self.peaks[key] = max(self.peaks.get(key,0.),f)
            self.last_fear[key] = f
            info = row['agents'][key]
            self.social_peaks[key] = max(self.social_peaks.get(key,0.),info['social_F'])
            self.direct_peaks[key] = max(self.direct_peaks.get(key,0.),info['direct_F'])
            if row['people'][key]['name'] == self.target_name:
                self.target_id = key
            if state['behavior']['mode'] == 'SCARED':
                self.scared_s[key] += .025
                self.reasons[(key,state['behavior']['reason'])] += 1
                if state['behavior']['reason'] == 'fear':
                    self.fear_triggered_s[key] += .025
            elif key == self.target_id and self.scared_s[key] > 0 and info['intent']['allow_task_progress']:
                self.task_resumed_samples += 1

    def actual(self, msg):
        super().actual(msg)
        t = stamp(msg)
        row = next((r for r in reversed(self.psych) if r['stamp_ns'] <= t),None)
        person = next((p for p in msg.pedestrians if p.name == self.target_name),None)
        if row is None or person is None or self.target_id is None or t-row['stamp_ns'] > 250_000_000:
            self.last_retreat = None
            return
        state = row['states'][self.target_id]
        robot = row['robots'].get(str(state['behavior']['target_robot_id']))
        p = person.pose.position
        if state['behavior']['mode'] != 'SCARED' or not robot:
            self.last_retreat = None
            return
        dx,dy = p.x-robot['x'],p.y-robot['y']
        norm = math.hypot(dx,dy)
        if self.last_retreat is not None and norm > 1e-9:
            old_t,x,y = self.last_retreat
            dt = (t-old_t)/1e9
            projected = ((p.x-x)*dx+(p.y-y)*dy)/norm
            if 0 < dt <= .15 and projected/dt >= .1:
                self.away_s += dt
                self.away_distance += projected
        self.last_retreat = (t,p.x,p.y)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('scenario'); ap.add_argument('--output',required=True)
    ap.add_argument('--run-dir',required=True); ap.add_argument('--duration-wall',type=float,default=60)
    ap.add_argument('--ready-timeout',type=float,default=300)
    ap.add_argument('--robot-clearance-policy',choices=('record','strict'),default='record'); args = ap.parse_args()
    scenario = load_scenario(args.scenario)
    cfg = load_stimulus(scenario.stimulus_tasks)
    root = Path(__file__).resolve().parents[1]
    before = provenance(args.scenario,root/'config/world/arena.yaml')
    output = Path(args.output); output.parent.mkdir(parents=True,exist_ok=True)
    rclpy.init(); node = FearProbe(scenario,output,cfg.target_person,args.robot_clearance_policy)
    execution = None
    try:
        deadline = time.monotonic()+args.ready_timeout
        while time.monotonic() < deadline and not (node.ready() and node.psych):
            rclpy.spin_once(node,timeout_sec=.05)
            if node.failures: break
        if not node.ready():
            node.failures.add('readiness timeout')
        else:
            node.started = time.monotonic()
            while time.monotonic()-node.started < cfg.experiment_timeout_wall_s+30:
                rclpy.spin_once(node,timeout_sec=.02); node.check_geometry()
                if node.psych_seen and time.monotonic()-node.psych_seen > 1: node.failures.add('psychology missing')
                if time.monotonic()-node.health_seen > .6: node.failures.add('health missing')
                if node.failures: break
                path = Path(args.run_dir)/'fear_stimulus.json'
                if path.exists():
                    execution = json.loads(path.read_text())
                    if not execution['passed'] or time.monotonic()-node.started >= args.duration_wall:
                        break
        if not execution or not execution['passed']: node.failures.add('finite stimulus incomplete')
        if node.checked_samples < 2: node.failures.add('insufficient actual geometry')
        if node.clearance_observations.strict_failure(): node.failures.add('robot gap below .10m')
        if node.max_display_error > .25: node.failures.add('display error above .25m')
        if node.core_max_speed>1.00001 or node.core_max_acceleration>3.00001: node.failures.add('core motion bound exceeded')
        key = node.target_id
        if key is None:
            node.failures.add('focal participant never observed')
        elif cfg.variant == 'intrusive':
            # The showcase must demonstrate accumulated SIS fear, not just an emergency label.
            if node.peaks.get(key,0.) < .65 or node.fear_triggered_s[key] < .5:
                node.failures.add('missing accumulated fear-triggered SCARED')
            if node.away_s < .5 or node.away_distance < .1:
                node.failures.add('missing actual outward pedestrian displacement')
            if node.task_resumed_samples < 20:
                node.failures.add('focal task did not resume after SCARED')
        elif node.scared_s[key] > 0:
            node.failures.add('considerate stimulus triggered focal SCARED')
        if key is not None and (node.last_fear.get(key,1.) >= .35 or (node.psych and node.psych[-1]['states'][key]['behavior']['mode']=='SCARED')):
            node.failures.add('fear did not recover')
        if provenance(args.scenario,root/'config/world/arena.yaml') != before:
            node.failures.add('source changed during validation')
        report = dict(passed=not node.failures,acceptance_version='crowd_response_v2.1',
            navigation_interaction=node.clearance_observations.report(),
            scope='actual Isaac poses and committed V3 psychology; robot clearance is an interaction outcome in record mode',
            variant=cfg.variant,requested_wall_s=args.duration_wall,
            wall_s=0 if node.started is None else time.monotonic()-node.started,
            sim_s=0 if node.first_stamp is None else (node.last_stamp-node.first_stamp)/1e9,
            matched_samples=node.checked_samples,minimum_robot_gap_m=None if math.isinf(node.min_clearance) else node.min_clearance,
            minimum_human_gap_m=None if math.isinf(node.min_human) else node.min_human,max_display_error_m=node.max_display_error,
            core_max_speed_mps=node.core_max_speed,core_max_acceleration_mps2=node.core_max_acceleration,
            target_id=key,peak_fear=node.peaks,final_fear=node.last_fear,peak_direct_F=node.direct_peaks,peak_social_F=node.social_peaks,
            scared_sim_s=node.scared_s,fear_triggered_scared_sim_s=node.fear_triggered_s,
            scared_reasons={f'{a}:{b}':v for (a,b),v in node.reasons.items()},
            actual_outward_s=node.away_s,actual_outward_displacement_m=node.away_distance,
            task_resumed_samples=node.task_resumed_samples,
            modes={k:sorted(v) for k,v in node.modes.items()},cycles=dict(node.cycles),pause_checks=node.pause_checks,
            stimulus_execution=execution,last_status=node.status[-1] if node.status else None,
            failures=sorted(node.failures),provenance=before)
        with output.open('x') as f: json.dump(report,f,indent=2,allow_nan=False)
        print(json.dumps({k:v for k,v in report.items() if k!='provenance'},indent=2),flush=True)
        return int(not report['passed'])
    finally:
        node.actual_stream.close(); node.destroy_node(); rclpy.shutdown()


if __name__=='__main__': raise SystemExit(main())
