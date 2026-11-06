"""Simulation-only robot stimulus using observed poses and the existing command guard."""
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from arena_people_msgs.msg import Pedestrians
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
import yaml

from .fear_stimulus import load_stimulus, Protocol


def stamp(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


class FearStimulus(Node):
    def __init__(self):
        super().__init__('fear_stimulus')
        self.declare_parameter('tasks_file', '')
        self.declare_parameter('people_file', '')
        self.tasks_file = Path(self.get_parameter('tasks_file').value)
        self.cfg = load_stimulus(self.tasks_file)
        self.protocol = Protocol(self.cfg)
        people = yaml.safe_load(Path(self.get_parameter('people_file').value).read_text())['hunav_loader']['ros__parameters']
        self.radii = {n: float(people[n]['radius']) for n in people['agents']}
        if self.cfg.target_person not in self.radii:
            raise ValueError('missing stimulus target')
        self.publisher = self.create_publisher(Twist, f'/{self.cfg.robot}/cmd_vel_external', 10)
        self.phase_publisher = self.create_publisher(String, '/multirobot/fear_stimulus/status', 10)
        self.create_subscription(Odometry, f'/{self.cfg.robot}/odom', self._robot, qos_profile_sensor_data)
        self.create_subscription(Pedestrians, '/multirobot/hunav/actual_people', self._actual, 10)
        self.create_subscription(String, '/multirobot/hunav/status', self._status, 10)
        self.create_subscription(Bool, '/multirobot/hunav/health', self._health, 10)
        self.robot = None
        self.people = {}
        self.status = {}
        self.healthy = False
        self.health_seen = self.status_seen = self.robot_seen = self.actual_seen = 0.
        self.robot_ns = self.actual_ns = self.last_clock_ns = None
        self.started_wall = time.monotonic()
        self.active_wall = None
        self.active_ns = 0
        self.last_command = (0., 0.)
        self.minimum_gap = math.inf
        self.done = False
        self.samples = 0
        self.stage_changes = []
        self.run = Path(os.environ.get('ARENA_MULTI_RUN_DIR', '/tmp/arena-fear-stimulus'))
        self.run.mkdir(parents=True, exist_ok=True)
        self.stream = (self.run/'fear_stimulus.jsonl').open('x')
        self.timer = self.create_timer(.05, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _robot(self, msg):
        if msg.header.frame_id != self.cfg.robot+'/odom' or msg.child_frame_id != self.cfg.robot+'/base_link':
            self._finish('unexpected_odometry_frames')
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.robot = (p.x, p.y, yaw)
        self.robot_ns = stamp(msg)
        self.robot_seen = time.monotonic()

    def _actual(self, msg):
        if msg.header.frame_id != 'map' or {p.name for p in msg.pedestrians} != set(self.radii):
            # Isaac publishes empty/partial population heartbeats while spawning.
            # They cannot establish readiness; identity becomes strict once active.
            if self.active_wall is None:
                self.people = {}
                self.actual_ns = None
                return
            self._finish('actual_people_identity_or_frame_changed')
            return
        self.people = {p.name: (p.pose.position.x, p.pose.position.y) for p in msg.pedestrians}
        self.actual_ns = stamp(msg)
        self.actual_seen = time.monotonic()

    def _status(self, msg):
        self.status = json.loads(msg.data)
        self.status_seen = time.monotonic()

    def _health(self, msg):
        self.healthy = msg.data
        self.health_seen = time.monotonic()

    def _send(self, pair):
        msg = Twist()
        msg.linear.x, msg.angular.z = pair
        self.publisher.publish(msg)

    def _finish(self, error=''):
        if self.done:
            return
        self.done = True
        self._send((0., 0.))
        if error and self.protocol.phase != 'failed':
            self.protocol.fail(error, self.active_ns)
        self.stream.close()
        report = dict(schema_version=1, passed=not error, error=error, variant=self.cfg.variant,
            stimulus=asdict(self.cfg), phase=self.protocol.phase, transitions=self.stage_changes,
            protocol_transitions=self.protocol.transitions, active_sim_s=self.active_ns/1e9,
            wall_s=time.monotonic()-self.started_wall, samples=self.samples,
            minimum_sampled_gap_m=None if math.isinf(self.minimum_gap) else self.minimum_gap,
            final_observation_status=dict(healthy=self.healthy,status=self.status,
                wall_ages_s={k:None if seen==0 else time.monotonic()-seen for k,seen in
                    [('health',self.health_seen),('status',self.status_seen),('robot',self.robot_seen),('actual',self.actual_seen)]},
                robot_stamp_ns=self.robot_ns,actual_stamp_ns=self.actual_ns,
                clock_ns=self.get_clock().now().nanoseconds),
            scope='geometric external robot stimulus execution; psychological and actual-motion evidence assessed independently',
            knowledge='Isaac actual person positions, robot odometry; no mental states or human goals read')
        with (self.run/'fear_stimulus.json').open('x') as f:
            json.dump(report, f, indent=2, allow_nan=False)
        (self.get_logger().error if error else self.get_logger().info)(
            'FEAR_STIMULUS_FAILED:'+error if error else 'FEAR_STIMULUS_COMPLETE')

    def _tick(self):
        if self.done:
            self._send((0., 0.))
            return
        try:
            self._advance()
        except Exception as error:
            self._finish(str(error))

    def _advance(self):
        wall = time.monotonic()
        now = self.get_clock().now().nanoseconds
        if self.status.get('fault'):
            self._finish('pedestrian_fault:'+self.status['fault'])
            return
        ready = (self.robot is not None and self.people and self.healthy and self.status.get('healthy')
                 and self.status.get('updates', 0) >= 40
                 # Bool health is 10 Hz; detailed adapter status is only ~1 Hz.
                 # Match the existing demonstration's 2 s status contract.
                 and wall-self.status_seen <= 2.
                 and all(wall-seen <= .6 for seen in (self.health_seen,self.robot_seen,self.actual_seen))
                 and all(-.050001 <= (now-s)/1e9 <= .25 for s in (self.robot_ns,self.actual_ns)))
        if self.active_wall is None:
            if not ready:
                self._send((0., 0.))
                if wall-self.started_wall > self.cfg.startup_timeout_wall_s:
                    self._finish('stimulus_readiness_timeout')
                return
            self.active_wall = wall
            self.last_clock_ns = now
        if wall-self.active_wall > self.cfg.experiment_timeout_wall_s:
            self._finish('stimulus_wall_timeout')
            return
        if now < self.last_clock_ns:
            self._finish('clock_rewind')
            return
        dt_ns = now-self.last_clock_ns
        self.last_clock_ns = now
        if self.status.get('paused') and wall-self.status_seen <= 2.:
            self._send((0.,0.))
            self.last_command = (0.,0.)
            return
        if not ready:
            self._finish('stimulus_observation_or_health_lost')
            return
        if not all(math.isfinite(v) for v in (*self.robot,*(v for p in self.people.values() for v in p))):
            self._finish('nonfinite_pose')
            return
        if dt_ns > 250_000_000:
            self._finish('stimulus_simulation_time_gap')
            return
        if dt_ns == 0:
            self._send(self.last_command)
            return
        self.active_ns += dt_ns
        gap = min(math.dist(self.robot[:2],p)-self.cfg.robot_radius-self.radii[n] for n,p in self.people.items())
        self.minimum_gap = min(self.minimum_gap,gap)
        previous_phase = self.protocol.phase
        self.last_command = self.protocol.step(self.active_ns,self.robot,self.people[self.cfg.target_person],
                                               self.radii[self.cfg.target_person],gap)
        if self.protocol.phase != previous_phase or not self.stage_changes:
            self.stage_changes.append(dict(phase=self.protocol.phase,stamp_ns=now,active_ns=self.active_ns))
        self._send(self.last_command)
        record = dict(stamp_ns=now,active_ns=self.active_ns,phase=self.protocol.phase,variant=self.cfg.variant,
                      robot=self.robot,target=self.people[self.cfg.target_person],minimum_gap_m=gap,
                      command_linear=self.last_command[0],command_angular=self.last_command[1],
                      robot_stamp_ns=self.robot_ns,actual_stamp_ns=self.actual_ns)
        self.stream.write(json.dumps(record,allow_nan=False)+'\n')
        self.samples += 1
        msg = String(); msg.data = json.dumps(record,allow_nan=False); self.phase_publisher.publish(msg)
        if self.protocol.phase in ('complete','failed'):
            self._finish(self.protocol.error)


def main(args=None):
    rclpy.init(args=args)
    node = FearStimulus()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._finish('interrupted')
    finally:
        node._send((0.,0.))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
