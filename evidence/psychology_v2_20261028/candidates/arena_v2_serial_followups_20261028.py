import json,os,subprocess,sys,time
from pathlib import Path
root=Path('/home/lpc/workspace/arena5_multi_ws')
base=root/'evidence/runs/psychology_v2_20261028'
start=time.monotonic(); predecessor=base/'control_serial/driver.json'
while not predecessor.exists():
    if time.monotonic()-start>2400:raise SystemExit('predecessor exceeded bounded experiment deadline; no new process started')
    time.sleep(2)
report=[]
for seed,domain in [(12,84),(13,85)]:
    head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip()
    if head!='7cffd0eddc4c55aa414338d609b5648c71d3d70d':raise SystemExit('source freeze changed')
    name=f'seed{seed}_serial';env=os.environ.copy();env.update(ROS_DOMAIN_ID=str(domain),GPU_ID='2')
    command=[sys.executable,str(root/'scripts/validate_psychology_v2_experiment.py'),str(root/'config/scenarios/two_robots_multi_sfm_14_multigoal_psychology_v2_nav2.yaml'),'--seed',str(seed),'--duration-wall','600','--output-dir',str(base/name)]
    print('starting',name,flush=True)
    with (root/f'evidence/psychology_v2_20261028/{name}_stdout.txt').open('x') as log:
        result=subprocess.run(command,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT)
    report.append(dict(seed=seed,exit_code=result.returncode,driver=str(base/name/'driver.json')))
    print(report[-1],flush=True)
(root/'evidence/psychology_v2_20261028/serial_followups.json').write_text(json.dumps(report,indent=2)+'\n')
