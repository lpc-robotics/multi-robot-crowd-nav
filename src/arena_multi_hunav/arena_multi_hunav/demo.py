"""Finite automatic robot experiment, gated by real scene and Nav2 readiness."""
import json
import math
import os
from pathlib import Path
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import String


def load_tasks(path):
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict) or set(data) != {"robots", "rounds", "startup_timeout_wall_s", "goal_timeout_wall_s", "hold_after_round_sim_s"}:
        raise ValueError("invalid psychology demo task schema")
    names = data["robots"]
    if not isinstance(names, list) or not names or len(set(names)) != len(names):
        raise ValueError("invalid demo robot list")
    for field in ("startup_timeout_wall_s", "goal_timeout_wall_s", "hold_after_round_sim_s"):
        if isinstance(data[field], bool) or not isinstance(data[field], (int, float)) or not math.isfinite(data[field]) or data[field] < 0:
            raise ValueError("invalid demo timing")
    if data["startup_timeout_wall_s"] == 0 or data["goal_timeout_wall_s"] == 0:
        raise ValueError("demo timeouts must be positive")
    if not isinstance(data["rounds"], list) or not data["rounds"]:
        raise ValueError("empty demo")
    for item in data["rounds"]:
        if not isinstance(item, dict) or set(item) - {"goals", "delays"} or set(item.get("goals", {})) != set(names):
            raise ValueError("each demo round must cover all robots")
        for goal in item["goals"].values():
            if not isinstance(goal, list) or len(goal) != 3 or not all(
                    isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in goal):
                raise ValueError("invalid demo goal")
        for name, delay in item.get("delays", {}).items():
            if name not in names or not isinstance(delay, (float, int)) or isinstance(delay, bool) or not math.isfinite(delay) or delay < 0:
                raise ValueError("invalid demo delay")
    return data


class PsychologyDemo(Node):
    def __init__(self):
        super().__init__("psychology_demo")
        self.declare_parameter("tasks_file", "")
        self.tasks_path = Path(self.get_parameter("tasks_file").value)
        self.tasks = load_tasks(self.tasks_path)
        self._action_clients = {n: ActionClient(self, NavigateToPose, f"/{n}/navigate_to_pose") for n in self.tasks["robots"]}
        self.status = {}; self.status_seen = None; self.started = time.monotonic()
        self.round_index = 0; self.active = {}; self.round_start = None; self.hold_start = None
        self.results = []; self.done = False
        self.create_subscription(String, "/multirobot/hunav/status", self._status, 10)
        self.timer = self.create_timer(.1, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _status(self, msg):
        self.status = json.loads(msg.data); self.status_seen = time.monotonic()

    def _finish(self, error=""):
        if self.done:
            return
        self.done = True
        for record in self.active.values():
            if record.get("handle") and rclpy.ok():
                record["handle"].cancel_goal_async()
        run = Path(os.environ.get("ARENA_MULTI_RUN_DIR", "/tmp/arena-psychology-demo")); run.mkdir(parents=True, exist_ok=True)
        report = {"schema_version": 1, "passed": not error, "error": error, "tasks": str(self.tasks_path),
                  "rounds_requested": len(self.tasks["rounds"]), "rounds": self.results,
                  "wall_duration_s": time.monotonic()-self.started,
                  "sim_stamp_ns": self.get_clock().now().nanoseconds,
                  "scope": "robot_action_execution; psychological mechanisms require separate evidence"}
        (run / "psychology_demo.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
        (self.get_logger().error if error else self.get_logger().info)("PSYCHOLOGY_DEMO_FAILED:"+error if error else "PSYCHOLOGY_DEMO_COMPLETE")

    def _tick(self):
        if self.done:
            return
        try:
            self._advance()
        except Exception as error:
            self._finish(str(error))

    def _advance(self):
        wall = time.monotonic(); stamp = self.get_clock().now().nanoseconds
        if self.status.get("fault"):
            raise ValueError("pedestrian_fault:"+self.status["fault"])
        if self.status.get("paused") and self.status_seen is not None and wall-self.status_seen < 2:
            # Simulation time has not advanced. Preserve action handles and
            # issue no new task until fresh clock and robot samples recover.
            return
        if self.round_start is None:
            ready = (self.status.get("healthy") and self.status.get("updates", 0) >= 10 and
                     self.status_seen is not None and wall-self.status_seen < 2 and
                     all(c.server_is_ready() for c in self._action_clients.values()))
            if not ready:
                if wall-self.started > self.tasks["startup_timeout_wall_s"]:
                    raise ValueError("demo_readiness_timeout")
                return
            self.round_start = stamp; self.round_wall = wall
        if not self.status.get("healthy") or wall-self.status_seen > 2:
            raise ValueError("pedestrian_unhealthy")
        if stamp < self.round_start:
            raise ValueError("clock_rewind")
        if self.hold_start is not None:
            if (stamp-self.hold_start)/1e9 < self.tasks["hold_after_round_sim_s"]:
                return
            self.round_index += 1
            if self.round_index == len(self.tasks["rounds"]):
                self._finish(); return
            self.active = {}; self.round_start = stamp; self.round_wall = wall; self.hold_start = None
        if wall-self.round_wall > self.tasks["goal_timeout_wall_s"]:
            raise ValueError("robot_goal_timeout")
        task = self.tasks["rounds"][self.round_index]
        for name, client in self._action_clients.items():
            if name not in self.active and (stamp-self.round_start)/1e9 >= task.get("delays", {}).get(name, 0):
                x, y, yaw = task["goals"][name]
                goal = NavigateToPose.Goal(); goal.pose.header.frame_id = "map"
                goal.pose.header.stamp = self.get_clock().now().to_msg()
                goal.pose.pose.position.x = float(x); goal.pose.pose.position.y = float(y)
                goal.pose.pose.orientation.z = math.sin(yaw/2); goal.pose.pose.orientation.w = math.cos(yaw/2)
                record = {"send": client.send_goal_async(goal), "sent_ns": stamp}
                self.active[name] = record
                record["send"].add_done_callback(lambda future, r=record: self._accepted(future, r))
            if name not in self.active:
                continue
            record = self.active[name]
            if record.get("result") and record["result"].done() and "status" not in record:
                record["status"] = record["result"].result().status
                record["completed_ns"] = stamp
                if record["status"] != GoalStatus.STATUS_SUCCEEDED:
                    raise ValueError(f"{name}_goal_failed:{record['status']}")
        if len(self.active) == len(self._action_clients) and all("status" in r for r in self.active.values()):
            self.results.append({"round": self.round_index+1, "robots": {n: {k: v for k, v in r.items()
                if k in ("sent_ns", "completed_ns", "status")} for n, r in self.active.items()}})
            self.hold_start = stamp

    def _accepted(self, future, record):
        try:
            handle = future.result()
            if handle is None or not handle.accepted:
                self._finish("robot_goal_rejected"); return
            record["handle"] = handle
            if self.done:
                handle.cancel_goal_async()
            else:
                record["result"] = handle.get_result_async()
        except Exception as error:
            self._finish(str(error))


def main(args=None):
    rclpy.init(args=args); node = PsychologyDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._finish("interrupted")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
