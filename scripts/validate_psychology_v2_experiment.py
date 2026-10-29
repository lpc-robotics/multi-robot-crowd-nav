#!/usr/bin/env python3
"""Own one complete experiment, including watchdogs and unconditional process cleanup.

Run after sourcing scripts/env.sh, on a private ROS domain. A monitoring-tool
interruption must not leave an unattended Isaac experiment running overnight.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import yaml

ROOT=Path(__file__).resolve().parents[1]


def stop(process):
    if process is None or process.poll() is not None: return
    # These process groups were created by this driver, never discovered globally.
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGKILL):
        try: os.killpg(process.pid,sig)
        except ProcessLookupError: return
        try: process.wait(timeout=40 if sig==signal.SIGINT else 10); return
        except subprocess.TimeoutExpired: pass


def main():
    p=argparse.ArgumentParser();p.add_argument('scenario');p.add_argument('--output-dir',required=True)
    p.add_argument('--duration-wall',type=float,default=1800);p.add_argument('--seed',type=int)
    p.add_argument('--require-gaze',nargs='*',default=['13','14']);p.add_argument('--require-playful',nargs='*',default=['13','14'])
    p.add_argument('--no-demo',action='store_true');a=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') in (None,'51','71'):raise SystemExit('Use an isolated ROS domain')
    output=Path(a.output_dir).resolve();output.mkdir(parents=True,exist_ok=False)
    scenario=Path(a.scenario).resolve()
    if a.seed is not None:
        data=yaml.safe_load(scenario.read_text());data['random_seed']=a.seed
        for key in ('pedestrian_config','psychology_config','demo_tasks'):
            if data.get(key):data[key]=str((scenario.parent/data[key]).resolve())
        scenario=output/'scenario_override.yaml';scenario.write_text(yaml.safe_dump(data,sort_keys=False))
    env=os.environ.copy();env.update(LIVESTREAM='false',FOXGLOVE='false',ARENA_PEDESTRIAN_TELEMETRY='true',
                                    ARENA_MULTI_LOG_ROOT=str(output/'runs'))
    children=[];streams=[];report=dict(passed=False,scenario=str(scenario),requested_min_wall_s=a.duration_wall)
    start=time.monotonic()
    def launch(command,name):
        log=(output/name).open('x');streams.append(log)
        proc=subprocess.Popen(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        children.append(proc);return proc
    try:
        command=[str(ROOT/'scripts/run_multirobot.sh'),str(scenario)]
        if not a.no_demo:command.append('auto_demo:=true')
        launcher=launch(command,'launch.log')
        timing=launch([sys.executable,str(ROOT/'scripts/monitor_runtime_timing.py'),
            '--duration',str(a.duration_wall+1560),'--output',str(output/'timing.json')],'timing.log')
        deadline=time.monotonic()+60
        while time.monotonic()<deadline:
            manifests=list((output/'runs').glob('*/source_manifest.json'))
            if manifests:break
            if launcher.poll() is not None:raise RuntimeError('launcher failed before manifest')
            time.sleep(.2)
        else:raise RuntimeError('runtime manifest timeout')
        run=manifests[0].parent;report['run_dir']=str(run)
        command=[sys.executable,str(ROOT/'scripts/validate_psychology_v2_runtime.py'),str(scenario),
            '--duration-wall',str(a.duration_wall),'--output',str(output/'acceptance.json'),'--run-dir',str(run),
            '--require-gaze',*a.require_gaze,'--require-playful',*a.require_playful]
        if not a.no_demo:command.extend(['--require-demo','--until-demo-complete'])
        probe=launch(command,'probe.log')
        report['probe_exit']=probe.wait(timeout=a.duration_wall+1560)
        result=json.loads((output/'acceptance.json').read_text());report['passed']=result['passed']
    except (Exception,KeyboardInterrupt) as e:
        report['error']=str(e) or type(e).__name__
    finally:
        # Stop probe/timing first, then launcher; its existing trap owns ROS children.
        for proc in reversed(children):stop(proc)
        for stream in streams:stream.close()
        report['driver_wall_s']=time.monotonic()-start
        (output/'driver.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2),flush=True)
    return int(not report['passed'])


if __name__=='__main__':raise SystemExit(main())
