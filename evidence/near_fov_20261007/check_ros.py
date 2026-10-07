"""Verify declared startup parameters and real core service responses in an isolated ROS domain."""
import json
import math
import os
from pathlib import Path
import subprocess

import rclpy
from rclpy.serialization import serialize_message
from rcl_interfaces.srv import GetParameters
from arena_multi_hunav_msgs.msg import BehaviorModifiers
from arena_multi_hunav_msgs.srv import ComputeMultiAgents
from hunav_msgs.msg import Agent
from geometry_msgs.msg import Pose

root = Path(__file__).resolve().parent
workspace = root.parents[1]


def wait(node, future):
    rclpy.spin_until_future_complete(node, future, timeout_sec=10)
    if not future.done() or future.exception():
        raise RuntimeError("ROS request failed or timed out")
    return future.result()


def main():
    if int(os.environ.get("ROS_DOMAIN_ID", "0")) in (0, 71):
        raise RuntimeError("Set a dedicated ROS_DOMAIN_ID other than 0 or production domain 71")
    rclpy.init()
    node = rclpy.create_node("arena5_near_fov_parameter_test")
    reports = []
    try:
        for name, params, expected_social in [
            ("default_rear", {}, False),
            ("full_rear", {"robot_fov_deg": 360.0}, True),
            ("narrow_side", {"robot_fov_deg": 60.0, "robot_fov_fade_deg": 0.0}, False),
            ("physical_rear", {"near_gain": 8.0, "near_sigma": .1, "robot_clearance": .15}, False),
            ("invalid_fov", {"robot_fov_deg": 0.0}, None),
        ]:
            service = "/arena5_near_fov_test/" + name
            server = "near_fov_" + name
            command = [str(workspace/"build/arena_multi_hunav_core/multi_sfm_server"), "--ros-args", "-r", "__node:="+server,
                       "-r", "/multirobot/hunav/compute_agents:="+service]
            for key, value in params.items():
                command.extend(["-p", f"{key}:={value}"])
            with (root/"validation"/(name+".log")).open("w") as log:
                process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
                client = node.create_client(ComputeMultiAgents, service)
                parameters = node.create_client(GetParameters, "/"+server+"/get_parameters")
                try:
                    assert client.wait_for_service(timeout_sec=10)
                    assert parameters.wait_for_service(timeout_sec=10)
                    expected = {"robot_fov_deg": 200.0, "robot_fov_fade_deg": 10.0, "near_gain": 10.0, "near_sigma": .2, "robot_clearance": .1}
                    expected.update(params)
                    query = GetParameters.Request(); query.names = list(expected)
                    values = wait(node, parameters.call_async(query)).values
                    assert [v.double_value for v in values] == list(expected.values())
                    q = ComputeMultiAgents.Request(); q.dt = .025; q.epoch = 9; q.step = 1
                    q.current_agents.header.frame_id = "map"; q.robots.header.frame_id = "map"
                    person = Agent(id=1, name="person", type=Agent.PERSON, group_id=-1, radius=.4, desired_velocity=1.0, goal_radius=.3)
                    person.behavior.type = 1; person.behavior.goal_force_factor = 2.0
                    person.behavior.obstacle_force_factor = 10.0; person.behavior.social_force_factor = 5.0
                    person.position.position.x = 15.0; person.position.position.y = 11.5
                    goal = Pose(); goal.position.x = 25.0; goal.position.y = 11.5
                    person.goals = [goal]; q.current_agents.agents = [person]
                    robot = Agent(id=-1, name="robot", type=Agent.ROBOT, group_id=-1, radius=.35)
                    robot.position.position.x = 15.0 if name == "narrow_side" else 13.8
                    robot.position.position.y = 12.7 if name == "narrow_side" else 11.5
                    q.robots.agents = [robot]; q.modifiers = [BehaviorModifiers(agent_id=1)]
                    result = wait(node, client.call_async(q))
                    if expected_social is None:
                        assert not result.success and not result.influences, result.error
                        # ROS float32 fields round Python floats on the wire.
                        assert serialize_message(result.updated_agents) == serialize_message(q.current_agents)
                        assert "robot_fov_deg" in result.error
                    else:
                        assert result.success, result.error
                        f = result.influences[0]
                        near = expected["near_gain"] * math.exp((expected["robot_clearance"]-f.clearance)/expected["near_sigma"])
                        magnitude = math.hypot(f.force.x, f.force.y)
                        assert (abs(magnitude-near) > .01) if expected_social else math.isclose(magnitude, near, abs_tol=1e-10)
                    reports.append({"case": name, "declared_parameters": expected, "passed": True, "success": result.success})
                finally:
                    node.destroy_client(client); node.destroy_client(parameters)
                    process.terminate()
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill(); process.wait(timeout=5)
    finally:
        node.destroy_node(); rclpy.shutdown()
    report = {"scope": "isolated ROS parameter and service checks; no Isaac", "ros_domain_id": int(os.environ["ROS_DOMAIN_ID"]), "passed": True, "cases": reports}
    (root/"validation/ros_parameters.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
