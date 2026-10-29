"""One authoritative pedestrian world with a stateless multi-robot SFM backend."""
from __future__ import annotations

import copy
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Bool, String
from rosgraph_msgs.msg import Clock as ClockMsg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from hunav_msgs.msg import Agent, Agents
from arena_people_msgs.msg import Pedestrian, SpawnPedestrian
from arena_people_msgs.srv import SpawnPedestrians, UpdatePedestrians
from arena_multi_hunav_msgs.msg import BehaviorModifiers, BehaviorIntent, BehaviorIntentV3, InteractionForces
from arena_multi_hunav_msgs.srv import ComputeMultiAgents, ComputeMultiAgentsV2, ComputeMultiAgentsV3
import yaml

from .psychology import Entity, MentalState, MODELS, make_context, prepare_step
from .timeline import Timeline, Sample
from .routes import compile_route
from .psychology_config import load_psychology_config
from .behavior import MODES
from .behavior_v2 import MODES_V2
from .clock_gate import ClockStallGate


def load_people(path, *, world_bounds=None):
    document = yaml.safe_load(Path(path).read_text())
    spec = document["hunav_loader"]["ros__parameters"]
    models = document["arena_multi_hunav"]["character_models"]
    if len(models) != len(spec["agents"]):
        raise ValueError("one character model required per pedestrian")
    result = Agents(); result.header.frame_id = "map"
    ids, names = set(), set()
    for name in spec["agents"]:
        item = spec[name]
        route = compile_route(item, person=name, world_bounds=world_bounds)
        a = Agent(); a.id = int(item["id"]); a.name = name; a.type = Agent.PERSON
        if a.id <= 0 or a.id in ids or name in names:
            raise ValueError("invalid pedestrian identity")
        ids.add(a.id); names.add(name)
        a.group_id = -1; a.skin = int(item.get("skin", 0)); a.radius = float(item["radius"])
        a.desired_velocity = float(item["max_vel"]); a.goal_radius = float(item["goal_radius"])
        a.cyclic_goals = route.cyclic
        if int(item["behavior"]["type"]) != 1 or int(item.get("group_id", -1)) != -1:
            raise ValueError("multi_sfm v1 supports only ungrouped regular people")
        a.behavior.type = 1
        for key in ("goal_force_factor", "obstacle_force_factor", "social_force_factor"):
            setattr(a.behavior, key, float(item["behavior"][key]))
        p = item["init_pose"]
        a.position.position.x = float(p["x"]); a.position.position.y = float(p["y"])
        a.yaw = float(p["h"])
        a.position.orientation.z = math.sin(a.yaw / 2); a.position.orientation.w = math.cos(a.yaw / 2)
        for x, y in route.points:
            goal = Pose(); goal.position.x = x; goal.position.y = y
            goal.orientation.w = 1.0; a.goals.append(goal)
        values = [a.position.position.x, a.position.position.y, a.yaw, a.radius,
                  a.desired_velocity, a.goal_radius] + [v for g in a.goals for v in (g.position.x, g.position.y)]
        if not all(math.isfinite(v) for v in values) or min(a.radius, a.goal_radius) <= 0 or not 0 < a.desired_velocity <= 1.0:
            raise ValueError("invalid pedestrian geometry, speed, or goals")
        result.agents.append(a)
    if not result.agents:
        raise ValueError("no pedestrians")
    return result, dict(zip(spec["agents"], models))


def entity(a):
    return Entity(a.id, a.name, a.position.position.x, a.position.position.y,
                  a.velocity.linear.x, a.velocity.linear.y, a.radius,
                  tuple((g.position.x, g.position.y) for g in a.goals), a.yaw, a.desired_velocity, a.goal_radius)


def validate_response(request, response):
    if not response or not response.success:
        raise ValueError("compute_failed:" + (response.error if response else "empty response"))
    start = request.current_agents.header.stamp
    expected = start.sec*1_000_000_000 + start.nanosec + round(request.dt*1e9)
    stamp = response.updated_agents.header.stamp
    if (response.epoch, response.step) != (request.epoch, request.step) or stamp.sec*1_000_000_000+stamp.nanosec != expected:
        raise ValueError("late_or_invalid_compute_response")
    if response.updated_agents.header.frame_id != "map":
        raise ValueError("compute_frame_changed")
    if sorted((a.id,a.name) for a in response.updated_agents.agents) != sorted((a.id,a.name) for a in request.current_agents.agents):
        raise ValueError("compute_identity_changed")
    for a in response.updated_agents.agents:
        values = (a.position.position.x,a.position.position.y,a.yaw,
                  a.velocity.linear.x,a.velocity.linear.y,a.velocity.angular.z)
        if not all(math.isfinite(v) for v in values) or math.hypot(a.velocity.linear.x,a.velocity.linear.y)>1.00001:
            raise ValueError("invalid_compute_motion")
        before = next(p for p in request.current_agents.agents if p.id == a.id)
        queue = [(g.position.x, g.position.y) for g in a.goals]
        old_queue = [(g.position.x, g.position.y) for g in before.goals]
        if hasattr(request, "intents"):
            intent = next(i for i in request.intents if i.agent_id == a.id)
            if not intent.allow_task_progress and queue != old_queue:
                raise ValueError("temporary_behavior_advanced_task")
            if intent.override_desired_velocity and math.hypot(
                    a.velocity.linear.x-before.velocity.linear.x, a.velocity.linear.y-before.velocity.linear.y) > 3*request.dt+1e-6:
                raise ValueError("invalid_compute_acceleration")
        if queue != old_queue:
            expected_queue = old_queue[1:] + (old_queue[:1] if before.cyclic_goals else [])
            if queue != expected_queue or not old_queue or math.dist(
                    (a.position.position.x, a.position.position.y), old_queue[0]) > before.goal_radius+1e-6:
                raise ValueError("invalid_compute_task_progress")
    expected_pairs={(a.id,r.name) for a in request.current_agents.agents for r in request.robots.agents}
    pairs={(v.agent_id,v.robot_name) for v in response.influences}
    if pairs!=expected_pairs or len(response.influences)!=len(expected_pairs):
        raise ValueError("invalid_compute_influences")
    if any(not all(math.isfinite(v) for v in (i.force.x, i.force.y, i.clearance)) for i in response.influences):
        raise ValueError("nonfinite_compute_influences")
    if hasattr(response, "motion"):
        if sorted(d.agent_id for d in response.motion) != sorted(a.id for a in request.current_agents.agents):
            raise ValueError("invalid_motion_diagnostics")
        for d in response.motion:
            for field in ("desired_force", "obstacle_force", "human_social_force", "robot_social_force", "robot_near_force", "total_before_limit", "acceleration"):
                v = getattr(d, field)
                if not all(math.isfinite(x) for x in (v.x, v.y, v.z)):
                    raise ValueError("nonfinite_motion_diagnostics")
            if hasattr(d, "robot_static_force"):
                if not all(math.isfinite(v) for v in (d.robot_static_force.x, d.robot_static_force.y,
                        d.robot_static_force.z, d.applied_yaw, d.applied_yaw_rate)):
                    raise ValueError("nonfinite_v3_motion_diagnostics")
                a = next(p for p in response.updated_agents.agents if p.id == d.agent_id)
                if abs(math.remainder(a.yaw-d.applied_yaw, 2*math.pi)) > 1e-6:
                    raise ValueError("inconsistent_applied_yaw")
                if d.yaw_applied:
                    i = next(i for i in request.intents if i.agent_id == d.agent_id)
                    if not i.override_yaw or abs(d.applied_yaw_rate) > i.max_yaw_rate+1e-9:
                        raise ValueError("invalid_applied_yaw_rate")
    return expected


def commit_candidate(model, snapshot, proposal, request, response):
    """Validate everything, including plugin restore, before caller swaps state."""
    stamp = validate_response(request, response)
    proposal.validate(a.id for a in request.current_agents.agents)
    before = copy.deepcopy(model.snapshot())
    try:
        model.restore(snapshot)
    except Exception:
        model.restore(before)
        raise
    return response.updated_agents, stamp, proposal.states


def motion_json(d):
    result = {"agent_id": d.agent_id, "acceleration_limited": d.acceleration_limited,
            "speed_limited": d.speed_limited, **{name: [getattr(d, name).x, getattr(d, name).y]
            for name in ("desired_force", "obstacle_force", "human_social_force", "robot_social_force", "robot_near_force", "total_before_limit", "acceleration")}}
    if hasattr(d, "robot_static_force"):
        result.update(robot_static_force=[d.robot_static_force.x, d.robot_static_force.y],
                      yaw_applied=d.yaw_applied, applied_yaw=d.applied_yaw, applied_yaw_rate=d.applied_yaw_rate)
    return result


class MultiSfmAdapter(Node):
    def __init__(self):
        super().__init__("multi_sfm_adapter")
        for name, value in {"robot_names": ["robot_1", "robot_2"], "agent_config_file": "",
                            "random_seed": 1, "psychology_model": "noop",
                            "psychology_config_file": "",
                            "observation_timeout": 0.6, "simulation_stamp_timeout": 1.0,
                            "clock_stall_timeout": 30.0,
                            "service_timeout": 1.0}.items():
            self.declare_parameter(name, value)
        self.names = sorted(self.get_parameter("robot_names").value)
        if not self.names or len(set(self.names)) != len(self.names):
            raise ValueError("robot_names must be nonempty and unique")
        self.robot_ids = {name: -i - 1 for i, name in enumerate(self.names)}
        self.history = {name: Timeline() for name in self.names}
        self.agents, self.models = load_people(self.get_parameter("agent_config_file").value)
        if set(self.models).intersection(self.names):
            raise ValueError("robot and pedestrian names overlap")
        self.epoch = uuid.uuid4().int & ((1 << 64) - 1); self.step = 0
        self.stamp_ns = None; self.clock_ns = None; self.scene_ready = False; self.spawned = False
        self.fault = ""; self.pending = None; self.update_pending = None
        self.fault_details = {}
        self.last_clock_wall = None
        self.last_tick_wall = None
        self.tick_gap_s = 0.0
        stall_timeout = float(self.get_parameter("clock_stall_timeout").value)
        if not math.isfinite(stall_timeout) or stall_timeout <= float(self.get_parameter("observation_timeout").value):
            raise ValueError("clock_stall_timeout must exceed observation_timeout")
        self.clock_gate = ClockStallGate(maximum_wall_s=stall_timeout)
        self.display_stamp = None; self.last_display_wall = 0.0
        self.compute_count = 0; self.update_count = 0; self.max_compute_wall = 0.0
        self.model = MODELS[str(self.get_parameter("psychology_model").value)]()
        self.use_v2 = self.model.name != "noop"
        self.use_v3 = self.model.name == "sis_cf_v2"
        config_path = str(self.get_parameter("psychology_config_file").value)
        config = load_psychology_config(config_path).document() if self.use_v2 else {}
        if not self.use_v2 and config_path:
            raise ValueError("noop accepts no psychology_config_file")
        self.model.initialize(config, tuple(a.id for a in self.agents.agents), int(self.get_parameter("random_seed").value))
        if self.use_v3 and any(p.preferred_robot_id and p.preferred_robot_id not in self.robot_ids.values()
                              for p, _ in self.model.resolved.values()):
            raise ValueError("preferred robot is absent from scenario")
        self.mental = self.model.initial_states() if self.use_v2 else {a.id: MentalState() for a in self.agents.agents}
        self.last_psychology = {}
        self.evidence_stream = None
        if self.use_v2:
            run_dir = Path(os.environ.get("ARENA_MULTI_RUN_DIR", str(Path(os.environ.get("ARENA_MULTI_WS", "/home/lpc/workspace/arena5_multi_ws")) / "runtime/psychology")))
            run_dir.mkdir(parents=True, exist_ok=True)
            self.evidence_stream = (run_dir / f"psychology_{self.epoch}.jsonl").open("x", buffering=1)
        self.agent_pub = self.create_publisher(Agents, "/multirobot/hunav/agents", 10)
        self.robot_pub = self.create_publisher(Agents, "/multirobot/hunav/robots", 10)
        self.force_pub = self.create_publisher(InteractionForces, "/multirobot/hunav/interactions", 10)
        self.health_pub = self.create_publisher(Bool, "/multirobot/hunav/health", 10)
        self.status_pub = self.create_publisher(String, "/multirobot/hunav/status", 10)
        self.psych_pub = self.create_publisher(String, "/multirobot/hunav/psychology", 10)
        self.create_subscription(String, "/multirobot/status", self._scene, 10)
        self.create_subscription(ClockMsg, "/clock", self._on_clock, qos_profile_sensor_data)
        for name in self.names:
            self.create_subscription(Odometry, f"/{name}/odom", lambda m, n=name: self._odom(n, m), qos_profile_sensor_data)
        self.compute_type = ComputeMultiAgentsV3 if self.use_v3 else (ComputeMultiAgentsV2 if self.use_v2 else ComputeMultiAgents)
        service_suffix = "_v3" if self.use_v3 else ("_v2" if self.use_v2 else "")
        self.compute = self.create_client(self.compute_type, "/multirobot/hunav/compute_agents" + service_suffix)
        self.spawn = self.create_client(SpawnPedestrians, "/isaac/SpawnPedestrians")
        self.update = self.create_client(UpdatePedestrians, "/isaac/UpdatePedestrians")
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.01, self._tick, clock=self.wall_clock)
        self.report_timer = self.create_timer(0.1, self._report, clock=self.wall_clock)
        self.last_report = 0.0
        self.spawn_future = None
        self.warm_display_ready = False

    def _fail(self, reason):
        if not self.fault:
            now = time.monotonic()
            self.fault_details = {
                "wall_monotonic_s": now,
                "clock_stamp_ns": self.clock_ns,
                "clock_receive_age_s": None if self.last_clock_wall is None else now-self.last_clock_wall,
                "timer_gap_s": self.tick_gap_s,
                "robot_receive_age_s": {name: None if h.wall_seen is None else now-h.wall_seen
                                        for name, h in self.history.items()},
                "robot_stamp_ns": {name: h.samples[-1].stamp_ns if h.samples else None
                                   for name, h in self.history.items()},
            }
            self.fault = reason; self.epoch = (self.epoch + 1) % (1 << 64)
            self.get_logger().error(reason)
            self.health_pub.publish(Bool(data=False))
            for a in self.agents.agents:
                a.velocity = Twist()
            # Do not cancel an outstanding Isaac write: serialize a stop AFTER
            # its completion so a late update cannot overwrite the stop command.

    def _scene(self, msg):
        if msg.data.startswith("MULTIROBOT_SCENE_READY"):
            self.scene_ready = True

    def _on_clock(self, msg):
        self.last_clock_wall = time.monotonic()
        stamp = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        if self.clock_ns is not None and stamp < self.clock_ns and self.spawned:
            self._fail("clock_rewind")
        self.clock_ns = stamp

    def _odom(self, name, msg):
        if self.fault:
            return
        try:
            if msg.header.frame_id != f"{name}/odom" or msg.child_frame_id != f"{name}/base_link":
                raise ValueError("unexpected odometry frames")
            q = msg.pose.pose.orientation
            if not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w)) or abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > 0.01:
                raise ValueError("invalid quaternion")
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
            v = msg.twist.twist
            sample = Sample(msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec,
                            msg.pose.pose.position.x, msg.pose.pose.position.y, yaw,
                            math.cos(yaw)*v.linear.x-math.sin(yaw)*v.linear.y,
                            math.sin(yaw)*v.linear.x+math.cos(yaw)*v.linear.y, v.angular.z)
            if not self.spawned and self.history[name].samples and sample.stamp_ns < self.history[name].samples[-1].stamp_ns:
                self.history[name] = Timeline()  # SpawnUrdf resets before scene readiness
            self.history[name].append(sample, time.monotonic())
        except Exception as error:
            self._fail(f"odom:{name}:{error}")

    def _robots(self, stamp_ns):
        result = Agents(); result.header.frame_id = "map"; result.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
        for name in self.names:
            s = self.history[name].at(stamp_ns)
            a = Agent(); a.id = self.robot_ids[name]; a.name = name; a.type = Agent.ROBOT; a.group_id = -1
            a.radius = 0.35; a.position.position.x = s.x; a.position.position.y = s.y; a.yaw = s.yaw
            a.position.orientation.z = math.sin(s.yaw/2); a.position.orientation.w = math.cos(s.yaw/2)
            a.velocity.linear.x = s.vx; a.velocity.linear.y = s.vy; a.velocity.angular.z = s.wz
            a.linear_vel = math.hypot(s.vx, s.vy); a.angular_vel = s.wz
            result.agents.append(a)
        return result

    def _fresh(self):
        now = time.monotonic()
        for name, history in self.history.items():
            if not history.samples:
                return False
            if now - history.wall_seen > self.get_parameter("observation_timeout").value:
                raise ValueError(f"robot_stale:{name}")
            age = (self.clock_ns - history.samples[-1].stamp_ns) / 1e9
            # Isaac float time can encode a 50 ms frame as 50,000,001 ns.
            # Allow only numerical roundoff around the existing one-frame lead.
            if age < -0.050001 or age > self.get_parameter("simulation_stamp_timeout").value:
                raise ValueError(f"robot_stamp_stale:{name}:age={age:.9f}")
        return True

    def _dispatch_display(self, stop=False):
        req = UpdatePedestrians.Request()
        req.stamp = Time(nanoseconds=(self.clock_ns or 0) if stop else self.stamp_ns).to_msg()
        for a in self.agents.agents:
            p = Pedestrian(); p.name = a.name; p.pose = copy.deepcopy(a.position)
            p.twist = Twist() if stop else copy.deepcopy(a.velocity); req.pedestrians.append(p)
        self.update_pending = (self.update.call_async(req), time.monotonic(), self.stamp_ns, stop)
        self.last_display_wall = time.monotonic()

    def _tick(self):
        now = time.monotonic()
        if self.last_tick_wall is not None:
            self.tick_gap_s = now-self.last_tick_wall
        self.last_tick_wall = now
        try:
            self._tick_impl()
        except Exception as error:
            self._fail(str(error))

    def _tick_impl(self):
        now = time.monotonic(); timeout = float(self.get_parameter("service_timeout").value)
        if not self.fault and self.stamp_ns is not None:
            was_held = self.clock_gate.active
            hold, held_seconds = self.clock_gate.update(
                now, self.clock_ns, self.last_clock_wall,
                {name: (h.samples[-1].stamp_ns, h.wall_seen) for name, h in self.history.items()},
                float(self.get_parameter("observation_timeout").value))
            if hold:
                if not was_held:
                    self.health_pub.publish(Bool(data=False))
                    self.get_logger().warn("SIMULATION_CLOCK_STALLED: holding motion and psychology")
                    self.last_report = now-1.0
                    self._report()
                return
            if held_seconds:
                # No commit, new request or display write was performed during
                # the coherent simulation stall. Retain serialized transactions.
                if self.pending:
                    future, started, *rest = self.pending
                    self.pending = (future, self.clock_gate.request_started_after_resume(started, now), *rest)
                if self.update_pending:
                    future, started, *rest = self.update_pending
                    self.update_pending = (future, self.clock_gate.request_started_after_resume(started, now), *rest)
                self.get_logger().info(
                    f"SIMULATION_CLOCK_RESUMED: gap={self.clock_gate.last_gap_s:.3f}s")
                self.last_report = now-1.0
                self._report()
        if self.update_pending:
            future, started, stamp, stopped = self.update_pending
            if future.done():
                self.update_pending = None
                response = future.result()
                if not response or list(response.results) != [0] * len(self.agents.agents):
                    self._fail("isaac_update_failed")
                elif not stopped:
                    self.display_stamp = stamp; self.update_count += 1
                elif self.stamp_ns is None:
                    self.warm_display_ready = True
                    self.warm_since = None
            elif not self.fault and now - started > (30.0 if self.stamp_ns is None else timeout):
                self.get_logger().error(
                    f"DISPLAY_REQUEST_TIMEOUT: effective_wait={now-started:.6f}s "
                    f"request_stamp_ns={stamp} clock_ns={self.clock_ns} "
                    f"recoveries={self.clock_gate.recoveries}")
                self._fail("isaac_update_timeout")
        if self.fault:
            if self.spawned and self.update_pending is None and self.update.service_is_ready() and now-self.last_display_wall >= 0.2:
                self._dispatch_display(stop=True)
            return
        if not self.scene_ready or not self.clock_ns or not all(h.samples for h in self.history.values()):
            return
        if not self.spawned:
            if self.spawn_future is None:
                if not (self.spawn.service_is_ready() and self.compute.service_is_ready() and self.update.service_is_ready()):
                    return
                req = SpawnPedestrians.Request()
                for a in self.agents.agents:
                    p = SpawnPedestrian(); p.model_ref = self.models[a.name]; p.pedestrian.name = a.name
                    p.pedestrian.pose = a.position; req.pedestrians.append(p)
                self.spawn_future = self.spawn.call_async(req); self.spawn_start = now
            elif self.spawn_future.done():
                res = self.spawn_future.result()
                if not res or list(res.results) != [0] * len(self.agents.agents):
                    raise ValueError("pedestrian_spawn_failed")
                self.spawned = True
                self.resume_wall = now
                self.warm_since = None
            elif now - self.spawn_start > 300:
                raise ValueError("pedestrian_spawn_timeout")
            return
        if self.stamp_ns is None:
            # Character creation blocks Isaac for seconds. Startup remains
            # unhealthy until every robot has a NEW sample after that operation.
            if now - self.resume_wall > 30:
                raise ValueError("post_spawn_odometry_timeout")
            if self.update_pending is None and now-self.last_display_wall >= 0.2:
                if not self.warm_display_ready:
                    self._dispatch_display(stop=True)
            if not self.warm_display_ready:
                return
            if not all(h.wall_seen >= self.resume_wall for h in self.history.values()):
                return
            try:
                fresh = self._fresh()
            except ValueError:
                fresh = False
            if not fresh:
                self.warm_since = None
                return
            if self.warm_since is None:
                self.warm_since = now
            if now-self.warm_since < 2.0:
                return
            self.stamp_ns = min(self.clock_ns, *(h.samples[-1].stamp_ns for h in self.history.values()))
            self.agents.header.stamp = Time(nanoseconds=self.stamp_ns).to_msg()
        if not self._fresh():
            return
        if self.pending:
            future, started, req, proposal, plugin_snapshot = self.pending
            if future.done():
                self.pending = None
                res = future.result()
                if (req.epoch, req.step) != (self.epoch, self.step):
                    raise ValueError("late_or_invalid_compute_response")
                self.agents, self.stamp_ns, self.mental = commit_candidate(self.model, plugin_snapshot, proposal, req, res)
                self.compute_count += 1; self.max_compute_wall = max(self.max_compute_wall, now-started)
                if self.use_v2:
                    self.last_psychology = {"schema_version": 3 if self.use_v3 else 2, "stamp_ns": self.stamp_ns,
                        "epoch": self.epoch, "step": req.step, "plugin": self.model.snapshot(),
                        "states": {str(i): asdict(s) for i, s in self.mental.items()},
                        **proposal.diagnostics, "motion": [motion_json(d) for d in res.motion],
                        "people": {str(a.id): asdict(entity(a)) for a in self.agents.agents},
                        "robots": {str(a.id): asdict(entity(a)) for a in req.robots.agents},
                        "task_goals": {str(a.id): [(g.position.x, g.position.y) for g in a.goals] for a in self.agents.agents}}
                    if self.use_v3:
                        self.last_psychology.update(input_stamp_ns=self.stamp_ns-round(req.dt*1e9), dt=req.dt)
                    encoded = json.dumps(self.last_psychology, sort_keys=True, allow_nan=False)
                    self.evidence_stream.write(encoded+"\n")
                    self.psych_pub.publish(String(data=encoded))
                self.agent_pub.publish(self.agents)
                self.force_pub.publish(InteractionForces(header=self.agents.header, epoch=self.epoch, step=self.step, influences=res.influences))
            elif now-started > timeout:
                raise ValueError("compute_timeout")
        if self.update_pending is None and now-self.last_display_wall >= 0.1:
            self._dispatch_display()
        if (self.clock_ns - self.stamp_ns) / 1e9 > 1.0:
            raise ValueError("integration_lag")
        if self.pending is not None:
            return
        available = min(self.clock_ns, *(h.samples[-1].stamp_ns for h in self.history.values()))
        if available-self.stamp_ns < 25_000_000:
            return
        robots = self._robots(self.stamp_ns)
        context = make_context(self.stamp_ns, 0.025, [entity(a) for a in self.agents.agents], [entity(a) for a in robots.agents])
        proposal, proposed_plugin = prepare_step(self.model, context, self.mental)
        self.step += 1
        req = self.compute_type.Request(epoch=self.epoch, step=self.step, dt=0.025,
                                        current_agents=copy.deepcopy(self.agents), robots=robots)
        for agent_id, modifier in sorted(proposal.modifiers.items()):
            req.modifiers.append(BehaviorModifiers(agent_id=agent_id, **asdict(modifier)))
        for agent_id, intent in sorted(proposal.intents.items()):
            fields = asdict(intent); fields["mode"] = (MODES_V2 if self.use_v3 else MODES)[intent.mode]
            req.intents.append((BehaviorIntentV3 if self.use_v3 else BehaviorIntent)(agent_id=agent_id, **fields))
        self.robot_pub.publish(robots)
        self.pending = (self.compute.call_async(req), now, req, proposal, proposed_plugin)

    def _report(self):
        now = time.monotonic()
        healthy = self.spawned and not self.fault and not self.clock_gate.active and self.update_count > 0
        self.health_pub.publish(Bool(data=healthy))
        if now-self.last_report < 1.0:
            return
        self.last_report = now
        payload = {"backend": "multi_sfm", "interaction_scope": "all_robots", "robots": self.names,
                   "healthy": healthy, "fault": self.fault, "epoch": self.epoch, "step": self.step,
                   "fault_details": self.fault_details,
                   "paused": self.clock_gate.active,
                   "clock_stall_recoveries": self.clock_gate.recoveries,
                   "last_clock_stall_wall_s": self.clock_gate.last_gap_s,
                   "compute": self.compute_count, "updates": self.update_count, "max_dt": 0.025,
                   "psychology_model": self.model.name,
                   "max_compute_wall_s": self.max_compute_wall,
                   "integration_lag_s": None if self.stamp_ns is None else (self.clock_ns-self.stamp_ns)/1e9,
                   "display_lag_s": None if self.display_stamp is None else (self.clock_ns-self.display_stamp)/1e9}
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))
        if not self.use_v2:
            self.psych_pub.publish(String(data=json.dumps({"schema_version": 1, "stamp_ns": self.stamp_ns,
                "plugin": self.model.snapshot(), "states": {str(i): asdict(s) for i, s in self.mental.items()}}, sort_keys=True)))

    def destroy_node(self):
        if self.evidence_stream is not None:
            self.evidence_stream.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MultiSfmAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.health_pub.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
