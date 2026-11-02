"""Sequential, owned-process performance trials; never changes source or gates."""
import os,sys,time,json,subprocess,signal,statistics
from pathlib import Path
import psutil
ROOT=Path('/home/lpc/workspace/arena5_multi_ws');base=ROOT/'evidence/crowd_visualization'
results={};active=None
try:
 for name,flags in [('off',[]),('foxglove',['--crowd-viz']),('isaac',['--isaac-overlay']),('both',['--crowd-viz','--isaac-overlay'])]:
  out=base/('perf_retry_'+name)
  if out.exists():raise RuntimeError('Refusing to overwrite '+str(out))
  log=(base/('perf_retry_'+name+'.log')).open('x')
  env=os.environ.copy();env.pop('ARENA_SCREENSHOT',None);env.pop('ARENA_CROWD_VIZ_METRICS',None)
  command=[sys.executable,str(ROOT/'scripts/validate_psychology_v2_experiment.py'),str(ROOT/'config/scenarios/two_robots_multi_sfm_14_multigoal_psychology_v2_nav2.yaml'),
   '--output-dir',str(out),'--duration-wall','600','--no-demo','--require-gaze','--require-playful','--livestream','--foxglove',*flags]
  active=subprocess.Popen(command,cwd=ROOT,env=env,stdout=log,stderr=log)
  cache={};samples=[];healthy_since=None;root_process=None;started=time.monotonic()
  while active.poll() is None:
   if time.monotonic()-started>1100:raise RuntimeError('performance trial watchdog')
   try:
    timing=json.loads((out/'timing.json').read_text())
    if timing.get('status',{}).get('healthy') and healthy_since is None:healthy_since=time.monotonic()
    paths=list((out/'runs').glob('*/launch.pid'))
    if paths and root_process is None:root_process=psutil.Process(int(paths[0].read_text()))
    cpu=0.;viz=0.
    for process in root_process.children(recursive=True) if root_process else []:
     try:
      if process.pid not in cache:
       cache[process.pid]=process;process.cpu_percent(None)
      else:process=cache[process.pid]
      value=process.cpu_percent(None);cpu+=value
      if any('crowd_visualizer' in a for a in process.cmdline()):viz+=value
     except psutil.Error:pass
    if healthy_since and time.monotonic()-healthy_since>60 and not (out/'acceptance.json').exists():samples.append({'cpu':cpu,'viz':viz})
   except (OSError,KeyError,json.JSONDecodeError,psutil.Error):pass
   time.sleep(1)
  code=active.returncode;active=None;log.close()
  acceptance=json.loads((out/'acceptance.json').read_text())
  results[name]={'passed':code==0 and acceptance['passed'],'samples':len(samples),
   'cpu_percent_one_core_mean':statistics.mean(s['cpu'] for s in samples) if samples else None,
   'viz_cpu_percent_one_core_mean':statistics.mean(s['viz'] for s in samples) if samples else None,
   'rtf':acceptance['sim_s']/acceptance['wall_s'] if acceptance['wall_s']>0 else None,'duration_wall_s':acceptance['wall_s']}
  (base/'performance_matrix.json').write_text(json.dumps({'scope':'same fourteen-person scenario, stationary robots, no desktop subscriber; 60s warmup excluded','cases':results},indent=2)+'\n')
  print(name,results[name],flush=True)
  # Preserve failed safety checks; later cases help distinguish baseline behavior.
 if results['off']['rtf'] is not None and results['off']['rtf']>0:
  for name,row in results.items():row['rtf_drop_fraction']=1-row['rtf']/results['off']['rtf'] if row['rtf'] is not None else None
 report={'scope':'same fourteen-person scenario, stationary robots, no desktop subscriber; 60s warmup excluded','cases':results,
  'passed':all(r['passed'] and r['samples']>0 and r['viz_cpu_percent_one_core_mean']<=10 and r.get('rtf_drop_fraction') is not None and r['rtf_drop_fraction']<=.05 for r in results.values())}
 (base/'performance_matrix.json').write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps(report),flush=True)
finally:
 if active is not None and active.poll() is None:
  active.send_signal(signal.SIGINT)
  try:active.wait(timeout=90)
  except subprocess.TimeoutExpired:active.terminate();active.wait(timeout=10)
