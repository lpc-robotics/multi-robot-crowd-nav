import json,os,subprocess,sys,time,signal
from pathlib import Path
root=Path.cwd();sys.path.insert(0,str(root/'scripts'))
from validate_psychology_v2_experiment import stop
base=root/'evidence/crowd_visualization';out=base/'sim_time_recording_smoke';children=[];streams=[]
try:
 log=(base/'sim_time_recording_smoke.log').open('x');streams.append(log)
 command=[sys.executable,str(root/'scripts/validate_psychology_v2_experiment.py'),str(root/'config/scenarios/one_robot_multi_sfm_1_playful_v2_validation.yaml'),'--output-dir',str(out),'--duration-wall','90','--no-demo','--require-gaze','--require-playful','--crowd-viz']
 driver=subprocess.Popen(command,stdout=log,stderr=log,start_new_session=True);children.append(driver)
 deadline=time.monotonic()+300
 while time.monotonic()<deadline:
  p=out/'timing.json'
  if p.exists():
   try:
    if json.loads(p.read_text()).get('status',{}).get('healthy'):break
   except json.JSONDecodeError:pass
  if driver.poll() is not None:raise RuntimeError('runtime failed before recording')
  time.sleep(.5)
 else:raise RuntimeError('runtime readiness timeout')
 log=(base/'sim_time_bag_record.log').open('x');streams.append(log)
 topics=['/multirobot/viz/'+k for k in ('actual','core','psychology','stimulus','markers','status')]+['/map','/tf','/tf_static','/robot_1/odom']
 recorder=subprocess.Popen(['ros2','bag','record','--use-sim-time','-o',str(base/'recording_sim_time'),*topics],stdout=log,stderr=log,start_new_session=True);children.append(recorder)
 time.sleep(15);stop(recorder)
 if driver.wait(timeout=400)!=0:raise RuntimeError('runtime acceptance failed')
 import rosbag2_py
 from rclpy.serialization import deserialize_message
 from arena_crowd_visualization_msgs.msg import PedestrianMotionArray
 reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(base/'recording_sim_time'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
 reader.set_filter(rosbag2_py.StorageFilter(topics=['/multirobot/viz/actual']))
 offsets=[];times=[]
 while reader.has_next():
  topic,data,t=reader.read_next();m=deserialize_message(data,PedestrianMotionArray);h=m.header.stamp.sec*10**9+m.header.stamp.nanosec
  offsets.append(abs(t-h)/1e9);times.append(t)
 report={'passed':len(offsets)>10 and max(offsets)<1 and all(a<=b for a,b in zip(times,times[1:])),'scope':'actual Isaac single-person recording with --use-sim-time; bag timestamps compared with actual headers','samples':len(offsets),'max_bag_header_offset_s':max(offsets) if offsets else None,'source_commit':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()}
 (base/'recording_sim_time.json').write_text(json.dumps(report,indent=2)+'\n');print(report,flush=True)
finally:
 for child in reversed(children):stop(child)
 for stream in streams:stream.close()
