"""Observed-pose multi-robot sequence, with simulation-clock and health gating."""
import json, math, os, time
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from arena_people_msgs.msg import Pedestrians
from std_msgs.msg import String, Bool
from .crowd_stimulus import load_sequence, Sequence
from .fear_stimulus_node import stamp


class CrowdStimulus(Node):
    def __init__(self):
        super().__init__('crowd_stimulus')
        self.declare_parameter('tasks_file','');self.declare_parameter('people_file','')
        self.cfg=load_sequence(self.get_parameter('tasks_file').value);self.sequence=Sequence(self.cfg)
        people=yaml.safe_load(Path(self.get_parameter('people_file').value).read_text())['hunav_loader']['ros__parameters']
        self.radii={n:float(people[n]['radius']) for n in people['agents']}
        for s in self.cfg['stages']:
            for c in s['commands'].values():
                if c['kind']=='follow' and c['person'] not in self.radii:raise ValueError('unknown sequence person')
        self.publishers_by_name={n:self.create_publisher(Twist,f'/{n}/cmd_vel_external',10) for n in self.cfg['robots']}
        self.status_pub=self.create_publisher(String,'/multirobot/crowd_stimulus/status',10)
        self.robots={};self.people={};self.seen={};self.stamps={};self.status={};self.healthy=False
        self.started=time.monotonic();self.active_wall=None;self.last_clock=None;self.active_ns=0;self.done=False
        self.minimum_gap=math.inf;self.samples=0
        for n in self.cfg['robots']:self.create_subscription(Odometry,f'/{n}/odom',lambda m,n=n:self.robot(n,m),qos_profile_sensor_data)
        self.create_subscription(Pedestrians,'/multirobot/hunav/actual_people',self.actual,10)
        self.create_subscription(String,'/multirobot/hunav/status',self.status_message,10)
        self.create_subscription(Bool,'/multirobot/hunav/health',self.health,10)
        self.run=Path(os.environ.get('ARENA_MULTI_RUN_DIR','/tmp/arena-crowd-stimulus'));self.run.mkdir(parents=True,exist_ok=True)
        self.stream=(self.run/'crowd_stimulus.jsonl').open('x')
        self.timer=self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def robot(self,n,m):
        if m.header.frame_id!=n+'/odom' or m.child_frame_id!=n+'/base_link':self.finish('invalid_robot_frames');return
        p,q=m.pose.pose.position,m.pose.pose.orientation
        self.robots[n]=(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        self.seen[n]=time.monotonic();self.stamps[n]=stamp(m)

    def actual(self,m):
        if m.header.frame_id!='map' or {p.name for p in m.pedestrians}!=set(self.radii):
            self.people={}
            if self.active_wall is not None:self.finish('people_identity_or_frame_changed')
            return
        self.people={p.name:(p.pose.position.x,p.pose.position.y) for p in m.pedestrians}
        self.seen['people']=time.monotonic();self.stamps['people']=stamp(m)

    def status_message(self,m):
        self.status=json.loads(m.data);self.seen['status']=time.monotonic()

    def health(self,m):
        self.healthy=m.data;self.seen['health']=time.monotonic()

    def send(self,commands=None):
        for n,pub in self.publishers_by_name.items():
            m=Twist();m.linear.x,m.angular.z=(0.,0.) if commands is None else commands[n];pub.publish(m)

    def finish(self,error=''):
        if self.done:return
        self.done=True;self.send();self.stream.close()
        report=dict(schema_version=1,passed=not error,error=error,config=self.cfg,active_sim_s=self.active_ns/1e9,
            wall_s=time.monotonic()-self.started,samples=self.samples,transitions=self.sequence.transitions,
            minimum_sampled_gap_m=None if math.isinf(self.minimum_gap) else self.minimum_gap,
            scope='robot stimulus execution only; no psychological mode is assigned',clearance_policy='record_only',
            input_knowledge='actual person positions and robot odometry only',last_status=self.status)
        with (self.run/'crowd_stimulus.json').open('x') as f:json.dump(report,f,indent=2,allow_nan=False)
        (self.get_logger().error if error else self.get_logger().info)('CROWD_STIMULUS_FAILED:'+error if error else 'CROWD_STIMULUS_COMPLETE')

    def tick(self):
        if self.done:self.send();return
        try:self.advance()
        except Exception as e:self.finish(str(e))

    def advance(self):
        wall=time.monotonic();now=self.get_clock().now().nanoseconds
        if self.status.get('fault'):self.finish('pedestrian_fault:'+self.status['fault']);return
        fresh_keys=[*self.cfg['robots'],'people','health']
        ready=(set(self.robots)==set(self.cfg['robots']) and bool(self.people) and self.healthy and self.status.get('healthy')
            and self.status.get('updates',0)>=40 and wall-self.seen.get('status',0)<=2.
            and all(wall-self.seen.get(k,0)<=.6 for k in fresh_keys)
            and all(-.050001<=(now-self.stamps.get(k,-10**18))/1e9<=.25 for k in [*self.cfg['robots'],'people']))
        if self.active_wall is None:
            if not ready:
                self.send()
                if wall-self.started>self.cfg['startup_timeout_wall_s']:self.finish('readiness_timeout')
                return
            self.active_wall=wall;self.last_clock=now
        if wall-self.active_wall>self.cfg['experiment_timeout_wall_s']:self.finish('experiment_wall_timeout');return
        if now<self.last_clock:self.finish('clock_rewind');return
        dt=now-self.last_clock;self.last_clock=now
        if self.status.get('paused') and wall-self.seen.get('status',0)<=2.:
            self.sequence.last={n:(0.,0.) for n in self.sequence.last};self.send();return
        if not ready:self.finish('observation_or_health_lost');return
        if dt>250_000_000:self.finish('simulation_time_gap');return
        if not all(math.isfinite(v) for p in [*self.robots.values(),*self.people.values()] for v in p):self.finish('nonfinite_pose');return
        if not dt:self.send(self.sequence.last);return
        self.active_ns+=dt
        commands=self.sequence.step(self.active_ns,self.robots,self.people,self.radii)
        gaps={n:min(math.dist(p[:2],q)-.35-self.radii[k] for k,q in self.people.items()) for n,p in self.robots.items()}
        self.minimum_gap=min(self.minimum_gap,*gaps.values());self.send(commands)
        stage='complete' if self.sequence.done else self.cfg['stages'][self.sequence.index]['name']
        row=dict(stamp_ns=now,active_ns=self.active_ns,stage=stage,commands=commands,robots=self.robots,people=self.people,minimum_robot_gaps_m=gaps,input_stamp_ns=self.stamps)
        self.stream.write(json.dumps(row,allow_nan=False)+'\n');self.samples+=1
        self.status_pub.publish(String(data=json.dumps(row,allow_nan=False)))
        if self.sequence.done:self.finish()


def main(args=None):
    rclpy.init(args=args);node=CrowdStimulus()
    try:rclpy.spin(node)
    except KeyboardInterrupt:node.finish('interrupted')
    finally:
        node.send();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
