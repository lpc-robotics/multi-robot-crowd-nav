import json,subprocess,time
from pathlib import Path
import rclpy
from arena_crowd_visualization_msgs.msg import PedestrianMotionArray,PedestrianPsychologyArray
rclpy.init();node=rclpy.create_node('crowd_sim_time_replay_check');samples={'actual':[],'psychology':[]}
for name,kind in [('actual',PedestrianMotionArray),('psychology',PedestrianPsychologyArray)]:
 node.create_subscription(kind,'/multirobot/viz/'+name,lambda m,k=name:samples[k].append({'stamp':m.header.stamp.sec*10**9+m.header.stamp.nanosec,'people':len(m.people)}),10)
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data
clocks=[]
node.create_subscription(Clock,'/clock',lambda m:clocks.append(m.clock.sec*10**9+m.clock.nanosec),qos_profile_sensor_data)
log=open('evidence/crowd_visualization/replay_sim_time.log','w')
p=subprocess.Popen(['ros2','bag','play','evidence/crowd_visualization/recording_sim_time','--delay','1','--clock'],stdout=log,stderr=log)
try:
 end=time.monotonic()+30
 while time.monotonic()<end and p.poll() is None:rclpy.spin_once(node,timeout_sec=.05)
 for _ in range(10):rclpy.spin_once(node,timeout_sec=.05)
 report={'domain':90,'counts':{k:len(v) for k,v in samples.items()},'passed':all(v and all(r['people']==1 for r in v) and all(b['stamp']>a['stamp'] for a,b in zip(v,v[1:])) for v in samples.values())}
 stamps=[r['stamp'] for v in samples.values() for r in v]
 report['clock_messages']=len(clocks)
 report['clock_matches_header_domain']=bool(clocks and stamps and min(clocks)<=min(stamps)+500000000 and max(clocks)>=max(stamps)-500000000)
 report['passed']=report['passed'] and report['clock_matches_header_domain']
 Path('evidence/crowd_visualization/replay_sim_time.json').write_text(json.dumps(report,indent=2)+'\n');print(report)
finally:
 if p.poll() is None:p.terminate();p.wait(timeout=5)
 log.close();node.destroy_node();rclpy.shutdown()
