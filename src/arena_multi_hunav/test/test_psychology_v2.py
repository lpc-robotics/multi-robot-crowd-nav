import copy
from dataclasses import asdict, replace
import math
import pytest

from arena_multi_hunav.psychology import Entity, make_context, prepare_step
from arena_multi_hunav.psychology_v2 import SocialPsychologyV2
from arena_multi_hunav.psychology_v2_config import PsychologyConfigV2, ParametersV2, Individual


def model(individuals=None, **parameters):
    cfg = PsychologyConfigV2(replace(ParametersV2(), **parameters), individuals or {})
    m = SocialPsychologyV2()
    m.initialize(cfg.document(), tuple(sorted(individuals or {1: None})), 11)
    return m


def person(i=1, **kwargs):
    return replace(Entity(i, f'person_{i}', 10., 10., 0., 0., .4, ((20., 10.),)), **kwargs)


def test_immune_profiles_neither_receive_nor_relay_cues():
    m = model({1: Individual(), 2: Individual('immune_dynamic'), 3: Individual('impassive_static')})
    states = m.initial_states()
    people = [person(1), person(2, y=11.), person(3, y=12.)]
    robot = Entity(-1, 'robot_1', 11., 10., -.3, 0., .35)
    for k in range(200):
        p = m.step(make_context(k*25_000_000, .025, people, (robot,)), states)
        states = p.states
        for i in (2, 3):
            assert states[i].variables == {'curiosity': 0., 'fear': 0.}
            assert p.intents[i].mode == 'REGULAR'
            assert not states[i].memories
        assert not p.diagnostics['edges']
    assert states[1].variables['fear'] > 0
    assert p.intents[2].robot_response == 0 and p.intents[3].robot_response == 1


def test_individual_overrides_are_resolved_and_hashed():
    m = model({1: Individual(parameters={'alpha_C': .2}), 2: Individual(parameters={'alpha_C': .8})})
    assert m.resolved[1][1].alpha_C == .2
    assert m.resolved[2][1].alpha_C == .8
    assert m.snapshot()['config_hash'] == m.config.digest
    with pytest.raises(ValueError):
        model({1: Individual(parameters={'typo': 1})})
    with pytest.raises(ValueError):
        model({1: Individual(parameters={'world_width': 1})})


def test_preference_restricts_interest_not_emergency_fear():
    m = model({1: Individual('playful', -1)})
    states = m.initial_states()
    robots = (Entity(-1, 'robot_1', 15., 10., 0., 0., .35),
              Entity(-2, 'robot_2', 10.8, 10., -.3, 0., .35))
    p = m.step(make_context(0, .025, (person(),), robots), states)
    assert p.intents[1].mode == 'SCARED' and p.intents[1].target_robot_id == -2


def test_prepare_does_not_mutate_original_state():
    m = model()
    states = m.initial_states(); original = copy.deepcopy(states)
    context = make_context(0, .025, (person(),), (Entity(-1, 'r', 13., 10., 0., 0., .35),))
    proposed, snapshot = prepare_step(m, context, states)
    assert states == original
    assert proposed.states != original
    assert snapshot == m.snapshot()


def test_surprise_stops_turns_completes_once_and_retries_do_not_consume_event():
    m = model(); states = m.initial_states(); robot = Entity(-1, 'r', 14., 10., 0., 0., .35)
    saw = set(); complete = False
    for k in range(400):
        context = make_context(k*25_000_000, .025, (person(),), (robot,))
        p, _ = prepare_step(m, context, states)
        repeated, _ = prepare_step(m, context, states)
        assert p == repeated
        if p.states[1].event == 'surprise_started':
            assert not states[1].surprise_consumed
            assert p.states[1].event_target_robot_id == -1
        states = p.states; saw.add(p.intents[1].mode)
        if p.intents[1].mode == 'SURPRISED':
            assert p.intents[1].desired_velocity_x == p.intents[1].desired_velocity_y == 0
            assert not p.intents[1].allow_task_progress and p.intents[1].override_yaw
        complete |= states[1].event == 'surprise_completed'
    assert {'SURPRISED', 'CURIOUS'} <= saw and complete
    assert states[1].surprise_consumed == states[1].surprise_completed == [-1]


def test_surprise_timeout_does_not_qualify_as_completed_observation():
    m = model(); states = m.initial_states(); robot = Entity(-1, 'r', 10., 14., 0., 0., .35)
    events = []
    for k in range(220):
        p = m.step(make_context(k*25_000_000, .025, (person(),), (robot,)), states)
        states = p.states; events.append(states[1].event)
    assert 'surprise_timeout' in events
    assert states[1].surprise_completed == []


def test_visibility_chatter_does_not_accumulate_continuous_exposure():
    m = model(); states = m.initial_states(); robot = Entity(-1, 'r', 14., 10., 0., 0., .35)
    for k in range(100):
        p = m.step(make_context(k*25_000_000, .025, (person(),), (robot,) if k % 2 else ()), states)
        states = p.states
        assert p.intents[1].mode != 'SURPRISED'
    assert states[1].surprise_consumed == []


def qualified_playful(profile='playful', **parameters):
    """Fixed perception mechanism test, not a physical trajectory acceptance."""
    m = model({1: Individual(profile, -1)}, **parameters)
    states = m.initial_states()
    me = person(x=12., y=11.5, yaw=math.atan2(-1.5, -2.))
    robot = Entity(-1, 'robot_1', 10., 10., .2, 0., .35)
    for k in range(600):
        p = m.step(make_context(k*25_000_000, .025, (me,), (robot,)), states)
        states = p.states
        if p.intents[1].mode == 'THREATENING': return m, states, me, robot, (k+1)*25_000_000
    return m, states, me, robot, 600*25_000_000


def test_playful_requires_eligibility_and_complete_observation_then_curiosity():
    m, states, me, robot, t = qualified_playful()
    state = states[1]
    assert state.behavior.mode == 'THREATENING' and state.phase == 'intercept'
    assert state.surprise_completed == [-1]
    assert state.episode_counts == {-1: 1}
    assert state.variables['curiosity'] >= .7
    assert t/1e9 >= 4+1
    assert qualified_playful('standard')[1][1].behavior.mode != 'THREATENING'
    assert qualified_playful(playful_enabled=0.)[1][1].behavior.mode != 'THREATENING'


@pytest.mark.parametrize('exit_kind', ['stopped', 'turned', 'lost', 'neighbor', 'risk'])
def test_playful_abort_does_not_reset_episode_clock_or_budget(exit_kind):
    m, states, me, robot, t = qualified_playful()
    assert states[1].phase == 'intercept'
    if exit_kind == 'stopped': robot = replace(robot, vx=0.)
    if exit_kind == 'turned': robot = replace(robot, vx=0., vy=.2)
    if exit_kind == 'risk': robot = replace(robot, x=me.x+.8, y=me.y, vx=-.2); me = replace(me, yaw=0.)
    if exit_kind == 'neighbor':
        # A newly observed robot location makes the moving front point unreachable.
        robot=replace(robot,x=29.,y=22.); me=replace(me,x=27.,y=21.,yaw=0.)
    for j in range(200):
        p = m.step(make_context(t+j*25_000_000, .025, (me,), () if exit_kind == 'lost' else (robot,)), states)
        states = p.states
        if states[1].phase == 'none': break
    assert states[1].phase == 'none'
    assert states[1].episode_counts == {-1: 1}
    assert states[1].playful_returning
    assert states[1].cooldown_until_ns[-1] > t


def test_playful_candidate_counter_is_transactional_and_returns_original_task():
    m, states, me, robot, t = qualified_playful()
    before = copy.deepcopy(states)
    context = make_context(t, .025, (me,), (replace(robot, vx=0.),))
    p, _ = prepare_step(m, context, states)
    assert states == before and states[1].phase == 'intercept'
    assert p.states[1].phase == 'yield'
    for j in range(200):
        p = m.step(make_context(t+j*25_000_000, .025, (me,), (replace(robot, vx=0.),)), states)
        states = p.states
        if states[1].phase == 'none': break
    assert p.intents[1].mode == 'REGULAR' and p.intents[1].allow_task_progress
    assert states[1].behavior.return_goal == me.goals[0]


@pytest.mark.parametrize('stamp', [0, 50_000_000])
def test_v2_rejects_rewind_or_skipped_simulation_interval(stamp):
    m=model(); states=m.initial_states()
    states=m.step(make_context(0,.025,(person(),),()),states).states
    with pytest.raises(ValueError):m.step(make_context(stamp,.025,(person(),),()),states)


def test_interception_tracks_observed_front_but_occupancy_point_and_timers_are_fixed():
    m, states, me, robot, t=qualified_playful()
    old=copy.deepcopy(states[1]); moved=replace(robot,x=robot.x+.1)
    p=m.step(make_context(t,.025,(me,),(moved,)),states)
    new=p.states[1]
    assert new.phase=='intercept'
    assert new.target_point[0]==pytest.approx(old.target_point[0]+.1)
    assert new.phase_since_ns==old.phase_since_ns
    # Explicit state-machine boundary fixture, separate from physical acceptance.
    new.phase='occupy';new.phase_since_ns=t+25_000_000
    at_point=replace(me,x=new.target_point[0],y=new.target_point[1],yaw=math.pi)
    fixed=new.target_point
    p=m.step(make_context(t+25_000_000,.025,(at_point,),(replace(moved,x=moved.x+.1),)),p.states)
    assert p.states[1].phase=='occupy' and p.states[1].target_point==fixed


def test_cached_snapshot_isolated_and_reset_invalidates_old_configuration():
    m=model();snapshot=m.snapshot();snapshot['agent_ids'].append(999)
    assert m.snapshot()['agent_ids']==[1]
    original=m.snapshot()
    cfg=PsychologyConfigV2(ParametersV2(alpha_C=.3),{})
    m.initialize(cfg.document(),(1,),11)
    with pytest.raises(ValueError):m.restore(original)
