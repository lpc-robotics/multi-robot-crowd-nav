#!/usr/bin/env python3
"""Read-only wall-arrival timing diagnostics; does not relax runtime guards."""
import argparse
import json
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, String


class Monitor(Node):
    def __init__(self):
        super().__init__('psychology_timing_monitor')
        self.started=time.monotonic(); self.last={}; self.maximum={}; self.count={}; self.events=[]; self.status={}
        self.create_subscription(Clock,'/clock',lambda m:self.seen('clock'),qos_profile_sensor_data)
        for name in ('robot_1','robot_2'):
            self.create_subscription(Odometry,f'/{name}/odom',lambda m,n=name:self.seen(n),qos_profile_sensor_data)
        self.create_subscription(Bool,'/multirobot/hunav/health',lambda m:self.seen('health'),10)
        self.create_subscription(String,'/multirobot/hunav/status',self.on_status,10)

    def seen(self,name):
        now=time.monotonic()
        if name in self.last:
            gap=now-self.last[name]; self.maximum[name]=max(self.maximum.get(name,0),gap)
            if gap>.25:
                event={'elapsed_wall_s':now-self.started,'topic':name,'arrival_gap_s':gap,'wall_time':time.time()}
                self.events.append(event); print(json.dumps(event),flush=True)
        self.last[name]=now; self.count[name]=self.count.get(name,0)+1

    def on_status(self,msg):
        self.status=json.loads(msg.data)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',required=True);ap.add_argument('--duration-wall',type=float,default=2100)
    args=ap.parse_args();rclpy.init();node=Monitor()
    try:
        while time.monotonic()-node.started<args.duration_wall:
            node.seen('monitor_loop');rclpy.spin_once(node,timeout_sec=.02)
            if node.status.get('fault'):break
    except KeyboardInterrupt:
        pass
    finally:
        report={'scope':'read-only arrival gaps, not a pass/fail acceptance test','maximum_arrival_gap_s':node.maximum,
                'sample_counts':node.count,'events':node.events,'last_status':node.status,
                'duration_wall_s':time.monotonic()-node.started}
        Path(args.output).write_text(json.dumps(report,indent=2)+'\n');node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
