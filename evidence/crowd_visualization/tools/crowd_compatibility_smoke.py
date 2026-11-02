import json,os,signal,subprocess,sys,time
from pathlib import Path
import rclpy
from arena_crowd_visualization_msgs.msg import PedestrianMotionArray,PedestrianPsychologyArray
from visualization_msgs.msg import MarkerArray
sys.path.insert(0,str(Path.cwd()/'scripts'))
from validate_psychology_v2_experiment import stop
root=Path.cwd();base=root/'evidence/crowd_visualization'
for name,scenario,count in [('noop_six_fixed','two_robots_multi_sfm_6_multigoal_nav2.yaml',6),('v1_eight','two_robots_multi_sfm_8_multigoal_psychology_nav2.yaml',8)]:
 out=base/name;out.mkdir(exist_ok=False)
 env=os.environ.copy();env.update(LIVESTREAM='false',FOXGLOVE='false',ARENA_MULTI_LOG_ROOT=str(out/'runs'),ARENA_PEDESTRIAN_TELEMETRY='true')
 children=[];streams=[];report={}
 rclpy.init();n=rclpy.create_node('compatibility_viz_observer');seen={}
 for key,typ in [('actual',PedestrianMotionArray),('psychology',PedestrianPsychologyArray),('markers',MarkerArray)]:
  n.create_subscription(typ,'/multirobot/viz/'+key,lambda m,k=key:seen.update({k:m}),2)
 try:
  for command,logname in [([str(root/'scripts/run_multirobot.sh'),str(root/'config/scenarios'/scenario),'crowd_viz:=true','isaac_overlay:=true'],'launch.log'),([sys.executable,str(root/'scripts/validate_multi_sfm_runtime.py'),str(root/'config/scenarios'/scenario),'--duration','60','--output',str(out/'acceptance.json')],'probe.log')]:
   log=(out/logname).open('x');streams.append(log);children.append(subprocess.Popen(command,env=env,stdout=log,stderr=log,start_new_session=True))
  until=time.monotonic()+450
  while children[-1].poll() is None and time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.1)
  actual=seen.get('actual');psych=seen.get('psychology');markers=seen.get('markers')
  report={'source_commit':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'actual_count':len(actual.people) if actual else 0,'psychology_count':len(psych.people) if psych else 0,'has_surprise':any(p.has_surprise for p in psych.people) if psych else None,'marker_count':len(markers.markers) if markers else 0,'runtime_exit':children[-1].poll()}
  report['passed']=report['runtime_exit']==0 and report['actual_count']==count and report['marker_count']>count and (name=='noop_six_fixed' or (report['psychology_count']==count and report['has_surprise'] is False))
 except Exception as exc:report.update(passed=False,error=str(exc))
 finally:
  for child in reversed(children):stop(child)
  for stream in streams:stream.close()
  n.destroy_node();rclpy.shutdown()
  (out/'visualization.json').write_text(json.dumps(report,indent=2)+'\n');print(name,report,flush=True)
 if not report['passed']:raise SystemExit(1)
