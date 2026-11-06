from dataclasses import replace
import importlib.util
import math
from pathlib import Path

from launch import LaunchContext
import pytest
import yaml

from arena_multi_control.fear_stimulus import Stimulus, Protocol, command, load_stimulus
from arena_multi_control.scenario import load_scenario

ROOT = Path(__file__).resolve().parents[3]
SCENARIO = ROOT/'config/scenarios/one_robot_multi_sfm_6_scared_intrusive_external.yaml'


def test_protocol_is_finite_and_requires_return_home_before_recovery():
    cfg = Stimulus(warmup_s=1, approach_s=2, recovery_s=3, withdraw_timeout_s=4)
    p = Protocol(cfg)
    args = ((7.,10.,0.), (12.,10.), .4, 4.)
    assert p.step(0,*args) == (0.,0.)
    assert p.step(1_000_000_000,*args)[0] > 0 and p.phase == 'approach'
    p.step(3_000_000_000,(9.,10.,math.pi),(12.,10.),.4,2.)
    assert p.phase == 'withdraw'
    p.step(4_000_000_000,*args)
    assert p.phase == 'recovery'
    assert p.step(7_000_000_000,*args) == (0.,0.) and p.phase == 'complete'
    assert p.step(8_000_000_000,*args) == (0.,0.)


@pytest.mark.parametrize('failure', ['clearance','rewind','withdraw_timeout'])
def test_abort_is_terminal_and_never_continues_toward_person(failure):
    p = Protocol(Stimulus(warmup_s=1,approach_s=1,withdraw_timeout_s=1))
    args = ((9.,10.,0.),(12.,10.),.4,2.)
    p.step(1_000_000_000,*args)
    p.step(2_000_000_000,*args)
    if failure == 'clearance':
        result = p.step(3_000_000_000,*args[:-1],.17)
    elif failure == 'rewind':
        result = p.step(0,*args)
    else:
        p.step(3_000_000_000,*args)
        result = p.step(4_000_000_000,*args)
    assert result == (0.,0.) and p.phase == 'failed'
    assert p.step(20_000_000_000,*args) == (0.,0.)


def test_follower_does_not_drive_sideways_or_into_target_envelope():
    cfg = Stimulus()
    assert command((7.,10.,math.pi),(12.,10.),1.,cfg)[0] == 0
    assert command((7.,10.,0.),(7.5,10.),1.,cfg)[0] == 0
    linear,angular = command((7.,10.,0.),(20.,11.),1.,cfg)
    assert 0 <= linear <= cfg.max_linear and abs(angular) <= cfg.max_angular


@pytest.mark.parametrize('override', [{'max_linear':1.01},{'abort_gap':.1},{'target_gap':.17},
                                      {'approach_s':float('nan')},{'max_linear':True}])
def test_invalid_stimulus_cannot_start(override):
    with pytest.raises(ValueError):
        replace(Stimulus(),**override).validate()


def test_high_speed_opt_in_cannot_enable_nav2_or_normal_multi_sfm(tmp_path):
    d = yaml.safe_load(SCENARIO.read_text())
    for key in ('pedestrian_config','psychology_config','stimulus_tasks'):
        d[key] = str((SCENARIO.parent/d[key]).resolve())
    path = tmp_path/'test.yaml'
    path.write_text(yaml.safe_dump(d))
    assert load_scenario(path).max_linear == .8
    for variant in ('missing_opt_in','nav2','no_stimulus','not_boolean'):
        changed = yaml.safe_load(yaml.safe_dump(d))
        if variant == 'missing_opt_in': changed.pop('allow_high_speed_external')
        if variant == 'nav2': changed['robots'][0]['control_mode'] = 'nav2'
        if variant == 'no_stimulus': changed.pop('stimulus_tasks')
        if variant == 'not_boolean': changed['allow_high_speed_external'] = 'true'
        path.write_text(yaml.safe_dump(changed))
        with pytest.raises(ValueError): load_scenario(path)


@pytest.mark.parametrize('auto',['false','true'])
def test_external_stimulus_launch_has_no_dwb_controller(tmp_path,monkeypatch,auto):
    spec = importlib.util.spec_from_file_location('fear_launch',ROOT/'src/arena_multi_bringup/launch/multirobot.launch.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    monkeypatch.setenv('ARENA_MULTI_RUN_DIR',str(tmp_path))
    context = LaunchContext(); context.launch_configurations.update(dict(
        scenario=str(SCENARIO),urdf=str(ROOT/'config/urdf/jackal.urdf'),map=str(ROOT/'config/maps/map.yaml'),
        world=str(ROOT/'config/world/arena.yaml'),auto_demo=auto,foxglove='false',headless='true',livestream='false',
        webrtc_ip='127.0.0.1',webrtc_signal_port='49130',webrtc_media_port='48030'))
    actions = module._launch(context)
    executables = [str(getattr(a,'node_executable',None)) for a in actions]
    assert sum('fear_stimulus' in e for e in executables) == (1 if auto=='true' else 0)
    assert not any('controller_server' in e or 'velocity_smoother' in e or 'psychology_demo' in e for e in executables)
    assert sum('command_guard' in e for e in executables) == 1


def test_sustained_attention_is_explicit_and_fear_dynamics_unchanged():
    original = yaml.safe_load((ROOT/'config/psychology/sis_cf_v2.yaml').read_text())
    experimental = yaml.safe_load((ROOT/'config/psychology/sis_cf_v2_scared_stimulus.yaml').read_text())
    assert original['parameters'] == experimental['parameters']
    assert experimental['individuals'][1]['parameters'] == {'robot_fov_deg':360.}
    for variant in ('intrusive','considerate'):
        s = load_scenario(ROOT/f'config/scenarios/one_robot_multi_sfm_6_scared_{variant}_external.yaml')
        assert s.psychology_config.endswith('sis_cf_v2_scared_stimulus.yaml')
        assert load_stimulus(s.stimulus_tasks).target_person == 'person_1'


@pytest.mark.parametrize('fault',['health','identity'])
def test_actual_node_waits_for_observations_freezes_pause_and_stops_on_fault(tmp_path,monkeypatch,fault):
    import json
    import time
    from types import SimpleNamespace
    import rclpy
    from arena_multi_control.fear_stimulus_node import FearStimulus
    monkeypatch.setenv('ROS_DOMAIN_ID','94')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY','1')
    monkeypatch.setenv('ARENA_MULTI_RUN_DIR',str(tmp_path))
    rclpy.init(args=['--ros-args','-p',f'tasks_file:={ROOT}/config/tasks/scared_stimulus_intrusive.yaml',
                    '-p',f'people_file:={ROOT}/config/hunav/multi_6_scared_stimulus.yaml'])
    node = None
    try:
        node = FearStimulus()
        sent = []
        monkeypatch.setattr(node,'_send',lambda pair:sent.append(pair))
        now = SimpleNamespace(nanoseconds=1_000_000_000)
        monkeypatch.setattr(node,'get_clock',lambda:SimpleNamespace(now=lambda:now))
        empty = SimpleNamespace(header=SimpleNamespace(frame_id='map'),pedestrians=[])
        node._actual(empty)
        assert not node.done and not node.people
        node._advance()
        assert sent[-1] == (0.,0.) and node.protocol.start_ns is None
        node.robot = (7.,10.,0.)
        node.people = {f'person_{i}':(16.+i,10.+i) for i in range(1,7)}
        node.healthy = True; node.status = dict(healthy=True,updates=40)
        def fresh():
            node.robot_ns = node.actual_ns = now.nanoseconds
            node.health_seen = node.status_seen = node.robot_seen = node.actual_seen = time.monotonic()
        fresh(); node._advance()
        now.nanoseconds += 50_000_000; fresh(); node._advance()
        active = node.active_ns
        assert node.protocol.start_ns is not None
        node.status['paused'] = True
        for _ in range(3):
            now.nanoseconds += 100_000_000; fresh(); node._advance()
        assert node.active_ns == active and sent[-1] == (0.,0.)
        node.status['paused'] = False
        now.nanoseconds += 50_000_000; fresh()
        # Detailed status legitimately arrives at 1 Hz; Bool health is fresh.
        node.status_seen -= 1.
        node._advance()
        assert node.active_ns == active+50_000_000
        if fault == 'health':
            node.healthy = False
            now.nanoseconds += 50_000_000; fresh(); node._advance()
        else:
            node._actual(empty)
        assert node.done and sent[-1] == (0.,0.)
        report = json.loads((tmp_path/'fear_stimulus.json').read_text())
        expected = 'stimulus_observation_or_health_lost' if fault == 'health' else 'actual_people_identity_or_frame_changed'
        assert not report['passed'] and report['error'] == expected
    finally:
        if node is not None:
            node._finish('test_end'); node.destroy_node()
        rclpy.shutdown()
