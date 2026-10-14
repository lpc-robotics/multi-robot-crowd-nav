import copy
import importlib.util
from pathlib import Path
import pytest
import yaml
from launch import LaunchContext
from arena_multi_hunav.demo import load_tasks
from arena_multi_hunav.demo import PsychologyDemo

ROOT=Path(__file__).resolve().parents[3]


def test_real_node_initialization_and_no_goals_before_readiness(monkeypatch):
    import rclpy
    monkeypatch.setenv('ROS_DOMAIN_ID','91');monkeypatch.setenv('ROS_LOCALHOST_ONLY','1')
    rclpy.init(args=['--ros-args','-p',f'tasks_file:={ROOT}/config/tasks/psychology_crossings.yaml'])
    node=None
    try:
        node=PsychologyDemo()
        node._advance()
        assert len(node._action_clients)==2
        assert node.active=={} and node.round_start is None
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()


def test_task_file_covers_two_robots_and_three_rounds():
    task=load_tasks(ROOT/'config/tasks/psychology_crossings.yaml')
    assert task['robots']==['robot_1','robot_2'] and len(task['rounds'])==3
    assert task['rounds'][1]['delays']['robot_2']==10


@pytest.mark.parametrize('mutation',['missing_robot','nonfinite','negative_delay'])
def test_invalid_tasks_fail_before_goal_sending(tmp_path,mutation):
    task=yaml.safe_load((ROOT/'config/tasks/psychology_crossings.yaml').read_text())
    if mutation=='missing_robot':del task['rounds'][0]['goals']['robot_2']
    if mutation=='nonfinite':task['rounds'][0]['goals']['robot_1'][0]=float('nan')
    if mutation=='negative_delay':task['rounds'][0]['delays']={'robot_1':-1}
    path=tmp_path/'tasks.yaml';path.write_text(yaml.safe_dump(task))
    with pytest.raises(ValueError):load_tasks(path)


@pytest.mark.parametrize('auto',['false','true'])
def test_launch_constructs_demo_only_when_enabled(tmp_path,monkeypatch,auto):
    spec=importlib.util.spec_from_file_location('psych_launch',ROOT/'src/arena_multi_bringup/launch/multirobot.launch.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    monkeypatch.setenv('ARENA_MULTI_RUN_DIR',str(tmp_path))
    context=LaunchContext();context.launch_configurations.update({
        'scenario':str(ROOT/'config/scenarios/two_robots_multi_sfm_6_multigoal_psychology_nav2.yaml'),
        'urdf':str(ROOT/'config/urdf/jackal.urdf'),'map':str(ROOT/'config/maps/map.yaml'),
        'world':str(ROOT/'config/world/arena.yaml'),'auto_demo':auto,'foxglove':'false',
        'headless':'true','livestream':'false','webrtc_ip':'127.0.0.1','webrtc_signal_port':'49130','webrtc_media_port':'48030'})
    actions=module._launch(context)
    executables=[getattr(a,'node_executable',None) for a in actions]
    # Node substitutions preserve the executable literal until launch execution.
    count=sum('psychology_demo' in str(exe) for exe in executables)
    assert count==(1 if auto=='true' else 0)
