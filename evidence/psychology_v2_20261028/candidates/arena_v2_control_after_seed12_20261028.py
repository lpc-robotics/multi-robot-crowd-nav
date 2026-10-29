import os,subprocess,sys,time,json
from pathlib import Path
root=Path('/home/lpc/workspace/arena5_multi_ws');base=root/'evidence/runs/psychology_v2_20261028';start=time.monotonic()
while not (base/'seed12_night/driver.json').exists():
 if time.monotonic()-start>2400:raise SystemExit('predecessor deadline exceeded; no process started')
 time.sleep(2)
if subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip()!='7cffd0eddc4c55aa414338d609b5648c71d3d70d':raise SystemExit('source freeze changed')
env=os.environ.copy();env.update(ROS_DOMAIN_ID='83',GPU_ID='2')
cmd=[sys.executable,str(root/'scripts/validate_psychology_v2_experiment.py'),str(root/'config/scenarios/two_robots_multi_sfm_14_multigoal_psychology_v2_control_nav2.yaml'),'--duration-wall','600','--require-playful','--output-dir',str(base/'control_night')]
with (root/'evidence/psychology_v2_20261028/control_night_stdout.txt').open('x') as log:
 result=subprocess.run(cmd,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT)
print(json.dumps(dict(control_exit=result.returncode)),flush=True)
