#!/usr/bin/env python3
"""Real ROS actions and readiness gating with synthetic Nav2 servers, no robot motion."""
import argparse
import json
import os
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import SingleThreadedExecutor
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from arena_multi_hunav.demo import PsychologyDemo


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',required=True);args=ap.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') in (None,'51','71','85'):raise SystemExit('use an isolated ROS domain')
    root=Path(__file__).resolve().parents[1]
    output=Path(args.output).resolve();output.parent.mkdir(parents=True,exist_ok=True)
    os.environ['ARENA_MULTI_RUN_DIR']=str(output.parent/'demo_mock_logs')
    rclpy.init(args=['--ros-args','-p',f'tasks_file:={root}/config/tasks/psychology_crossings.yaml','-p','use_sim_time:=true'])
    world=Node('synthetic_nav2_for_psychology');demo=PsychologyDemo();executor=SingleThreadedExecutor()
    executor.add_node(world);executor.add_node(demo)
    received=[];servers=[]
    def execute(handle):
        received.append([handle.request.pose.pose.position.x,handle.request.pose.pose.position.y])
        handle.succeed();return NavigateToPose.Result()
    for name in ('robot_1','robot_2'):
        servers.append(ActionServer(world,NavigateToPose,f'/{name}/navigate_to_pose',execute))
    clock=world.create_publisher(Clock,'/clock',10);status=world.create_publisher(String,'/multirobot/hunav/status',10)
    started=time.monotonic();sim=10.;before_ready=True
    try:
        while time.monotonic()-started<15 and not demo.done:
            elapsed=time.monotonic()-started;sim+=.1
            msg=Clock();msg.clock.sec,msg.clock.nanosec=divmod(round(sim*1e9),10**9);clock.publish(msg)
            ready=elapsed>1
            status.publish(String(data=json.dumps({'healthy':ready,'updates':100 if ready else 0,'fault':''})))
            executor.spin_once(timeout_sec=.005)
            if not ready:before_ready &= not received
            time.sleep(.002)
        existing=json.loads((output.parent/'demo_mock_logs/psychology_demo.json').read_text()) if demo.done else {}
        rounds=existing.get('rounds',[])
        delayed=False
        if len(rounds)>1:
            r=rounds[1]['robots'];delayed=r['robot_2']['sent_ns']-r['robot_1']['sent_ns']>=10_000_000_000
        report={'passed':bool(demo.done and existing.get('passed') and before_ready and len(received)==6 and delayed),
                'scope':'real ROS action clients, synthetic Nav2 servers; no robot/Isaac motion',
                'goals_received':len(received),'no_goals_before_ready':before_ready,'configured_delay_respected':delayed,
                'completed_rounds':len(rounds),'wall_duration_s':time.monotonic()-started}
        output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
        return 0 if report['passed'] else 1
    finally:
        for server in servers:server.destroy()
        executor.shutdown();demo.destroy_node();world.destroy_node();rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
