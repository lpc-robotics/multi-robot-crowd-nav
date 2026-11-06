import copy
from pathlib import Path
import pytest,yaml
from arena_multi_control.crowd_stimulus import load_sequence,Sequence
from arena_multi_control.scenario import load_scenario
ROOT=Path(__file__).resolve().parents[3]
PATH=ROOT/'config/tasks/psychology_v2_expressive_sequence.yaml'


def test_sequence_only_controls_external_robots_and_has_finite_exposure():
    cfg=load_sequence(PATH);sc=load_scenario(ROOT/'config/scenarios/two_robots_multi_sfm_14_multigoal_psychology_v2_nav2.yaml')
    assert all(r.control_mode=='external' for r in sc.robots)
    assert sc.sequence_tasks and not sc.demo_tasks and not sc.stimulus_tasks
    s=Sequence(cfg);poses={'robot_1':(9.,5.,0.),'robot_2':(12.,15.,0.)}
    people={'person_1':(10.,5.),'person_14':(13.,15.)};radii={n:.4 for n in people}
    prev={n:0. for n in poses}
    for k in range(4000):
        result=s.step(k*50_000_000,poses,people,radii)
        assert all(0<=v<=1. and abs(w)<=1. for v,w in result.values())
        # No clearance termination even when target and robot envelopes overlap.
        if s.done:break
        acceleration=max(c['acceleration'] for c in cfg['stages'][s.index]['commands'].values())
        for n,(v,w) in result.items():assert abs(v-prev[n])<=acceleration*.05+1e-9;prev[n]=v
    assert s.done and all(v==(0.,0.) for v in result.values())
    assert len(s.transitions)==len(cfg['stages'])


def test_repeated_simulation_time_freezes_sequence_and_rewind_rejected():
    cfg=load_sequence(PATH);s=Sequence(cfg);poses={n:(9.,5.,0.) for n in cfg['robots']}
    first=s.step(1,poses,{},{});snapshot=copy.deepcopy(s.__dict__)
    assert s.step(1,poses,{}, {})==first and s.__dict__==snapshot
    with pytest.raises(ValueError):s.step(0,poses,{}, {})


@pytest.mark.parametrize('edit',['psychology','unknown_person_field','speed','missing_robot'])
def test_invalid_or_hidden_state_commands_rejected(tmp_path,edit):
    d=yaml.safe_load(PATH.read_text());c=d['stages'][0]['commands']['robot_1']
    if edit=='psychology':c['fear']=.9
    elif edit=='unknown_person_field':c['set_mode']='THREATENING'
    elif edit=='speed':c['speed']=1.1
    else:d['stages'][0]['commands'].pop('robot_2')
    p=tmp_path/'bad.yaml';p.write_text(yaml.safe_dump(d))
    with pytest.raises(ValueError):load_sequence(p)


@pytest.mark.parametrize('fault',['health','identity','rewind'])
def test_node_waits_freezes_pause_and_stops_every_robot(tmp_path,monkeypatch,fault):
    import json,time,rclpy
    from types import SimpleNamespace
    from arena_multi_control.crowd_stimulus_node import CrowdStimulus
    monkeypatch.setenv('ROS_DOMAIN_ID','94');monkeypatch.setenv('ROS_LOCALHOST_ONLY','1')
    monkeypatch.setenv('ARENA_MULTI_RUN_DIR',str(tmp_path))
    rclpy.init(args=['--ros-args','-p',f'tasks_file:={ROOT}/config/tasks/psychology_v2_expressive_sequence.yaml',
        '-p',f'people_file:={ROOT}/config/hunav/multi_14_psychology_expressive.yaml'])
    node=None
    try:
        node=CrowdStimulus();sent={n:[] for n in node.cfg['robots']}
        for n in sent:monkeypatch.setattr(node.publishers_by_name[n],'publish',lambda msg,n=n:sent[n].append((msg.linear.x,msg.angular.z)))
        now=SimpleNamespace(nanoseconds=1_000_000_000)
        monkeypatch.setattr(node,'get_clock',lambda:SimpleNamespace(now=lambda:now))
        empty=SimpleNamespace(header=SimpleNamespace(frame_id='map'),pedestrians=[])
        node.actual(empty);node.advance()
        assert not node.done and node.active_wall is None
        node.robots={n:(9.,5.,0.) for n in sent}
        node.people={n:(15.,10.) for n in node.radii}
        node.healthy=True;node.status=dict(healthy=True,updates=40)
        def fresh():
            node.seen={k:time.monotonic() for k in [*sent,'people','health','status']}
            node.stamps={k:now.nanoseconds for k in [*sent,'people']}
        fresh();node.advance();now.nanoseconds+=50_000_000;fresh();node.advance()
        active=node.active_ns
        node.status['paused']=True
        for _ in range(3):
            now.nanoseconds+=100_000_000;fresh();node.advance()
        assert node.active_ns==active
        assert all(v[-1]==(0.,0.) for v in sent.values())
        node.status['paused']=False;now.nanoseconds+=50_000_000;fresh()
        node.seen['status']-=1.;node.advance()
        assert node.active_ns==active+50_000_000
        if fault=='health':node.healthy=False;node.advance()
        elif fault=='identity':node.actual(empty)
        else:now.nanoseconds-=100_000_000;node.advance()
        assert node.done and all(v[-1]==(0.,0.) for v in sent.values())
        report=json.loads((tmp_path/'crowd_stimulus.json').read_text())
        assert not report['passed']
    finally:
        if node is not None:node.finish('test_end');node.destroy_node()
        rclpy.shutdown()
