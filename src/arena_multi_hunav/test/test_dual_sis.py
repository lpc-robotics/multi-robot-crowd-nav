import copy
from dataclasses import replace
import math
import pytest
from arena_multi_hunav.psychology import Entity, make_context, prepare_step
from arena_multi_hunav.psychology_sis import DualSISPsychology, CrowdMentalState, exponential_step
from arena_multi_hunav.psychology_config import PsychologyConfig
from arena_multi_hunav.perception import risk, visibility, propagation_graph
from arena_multi_hunav.behavior import BehaviorState, choose_behavior


def model(cfg=None, ids=(1, 2)):
    result = DualSISPsychology(); result.initialize((cfg or PsychologyConfig()).document(), ids, 11)
    return result


def person(i=1, x=10., yaw=0., **kw):
    return Entity(i, f'person_{i}', x, 10., 0., 0., .4, ((20., 10.),), yaw, **kw)


def robot(x=12., vx=0., i=-1):
    return Entity(i, f'robot_{-i}', x, 10., vx, 0., .35)


def test_zero_and_analytic_update():
    assert exponential_step(0., 0., .2, .025) == 0
    assert exponential_step(.8, 0., .2, 5.) == pytest.approx(.8*math.exp(-1), abs=1e-12)
    assert exponential_step(.3, .7, .2, .025) == pytest.approx(.7/.9 + (.3-.7/.9)*math.exp(-.9*.025), abs=1e-12)
    for value in (0., .1, 1.):
        for rate in (0., 1e-12, 1e8):
            assert 0 <= exponential_step(value, rate, .25, .025) <= 1


def test_no_input_restores_and_no_neighbor_is_noop_motion():
    m=model(ids=(1,)); state=m.initial_states()
    p=m.step(make_context(0,.025,[person()],[]),state)
    assert p.states[1].variables == {'curiosity':0.,'fear':0.}
    assert not p.intents[1].override_desired_velocity
    state[1].variables['fear']=.8
    p=m.step(make_context(0,.025,[person()],[]),state)
    assert p.states[1].variables['fear']==pytest.approx(.8*math.exp(-.25*.025))
    assert p.intents[1].mode=='REGULAR' and p.intents[1].vigilant
    assert not p.intents[1].has_target_robot


def test_risk_relative_motion_and_fov():
    p=person()
    assert risk(p,robot(x=10.8),PsychologyConfig())>.8
    assert risk(p,robot(x=16.,vx=2.),PsychologyConfig())==0
    assert risk(p,robot(x=11.8,vx=-1.),PsychologyConfig())>0
    assert risk(p,robot(x=12.,vx=-1.),PsychologyConfig())==0  # prediction window endpoint
    assert visibility(p,robot(x=8.),8.,200.,10.)==0
    assert visibility(p,robot(),8.,200.,10.)==1
    assert risk(p,robot(x=10.),PsychologyConfig())==1


def test_synchronous_directional_propagation_and_permutation():
    m=model(); states=m.initial_states();states[1].variables['fear']=.8
    people=[person(1,x=10.,yaw=math.pi),person(2,x=11.,yaw=math.pi)]
    edges=propagation_graph(people,m.config)
    assert any(s==1 and t==2 for s,t,w in edges)
    assert not any(s==2 and t==1 for s,t,w in edges)
    p=m.step(make_context(0,.025,people,[]),states)
    q=m.step(make_context(0,.025,list(reversed(people)),[]),states)
    assert p==q
    assert p.diagnostics['agents'][2]['direct_F']==0
    assert p.states[2].variables['fear']>0
    assert states[2].variables['fear']==0
    assert not p.intents[2].has_target_robot


@pytest.mark.parametrize('field',['beta_F','expressivity_F'])
def test_ablation_eliminates_fear_propagation(field):
    m=model(replace(PsychologyConfig(),**{field:0.}));states=m.initial_states();states[1].variables['fear']=.8
    p=m.step(make_context(0,.025,[person(1),person(2,x=11.,yaw=math.pi)],[]),states)
    assert p.states[2].variables['fear']==0


def test_low_activation_still_emits_and_weights_bounded():
    m=model(ids=(1,2,3,4));states=m.initial_states();states[1].variables['fear']=.03
    people=[person(i,x=10.+.2*i,yaw=math.pi) for i in (1,2,3,4)]
    p=m.step(make_context(0,.025,people,[]),states)
    assert p.diagnostics['agents'][2]['social_F']>0
    for receiver in (1,2,3,4):
        assert sum(w for _,t,w in propagation_graph(people,m.config) if t==receiver)<=1+1e-12


def test_novelty_memory_expiry_and_hidden_truth_isolation():
    m=model(ids=(1,));states=m.initial_states()
    p=m.step(make_context(0,.025,[person()],[robot()]),states)
    assert p.states[1].memories[-1].novelty<1
    observed=copy.deepcopy(p.states);observed[1].variables['curiosity']=.8
    observed[1].behavior=BehaviorState('CURIOUS',target_robot_id=-1)
    # Robot behind observer cannot update last-seen position, risk, or intent.
    first=m.step(make_context(25_000_000,.025,[person(yaw=math.pi)],[robot(x=14.)]),observed)
    second=m.step(make_context(25_000_000,.025,[person(yaw=math.pi)],[robot(x=15.,vx=10.)]),observed)
    assert first==second
    assert first.diagnostics['agents'][1]['direct_C']==0
    assert first.states[1].memories[-1].last_seen_ns==0
    expired=m.step(make_context(3_000_000_000,.025,[person(yaw=math.pi)],[]),observed)
    assert expired.intents[1].mode=='REGULAR'
    again=m.step(make_context(4_000_000_000,.025,[person()],[robot()]),expired.states)
    assert again.diagnostics['agents'][1]['visible_robots'][-1]['novelty']<1


def test_inhibition_and_snapshot_transaction():
    normal=model(ids=(1,));inhibited=model(replace(PsychologyConfig(),kappa=1.),ids=(1,))
    state=normal.initial_states();state[1].variables={'curiosity':.5,'fear':.5}
    context=make_context(0,.025,[person()],[robot(x=14.)])
    p,snapshot=prepare_step(inhibited,context,state)
    assert p.states[1].variables['curiosity']<normal.step(context,state).states[1].variables['curiosity']
    assert snapshot==inhibited.snapshot()
    assert state[1].variables['curiosity']==.5
    with pytest.raises(ValueError):inhibited.restore(normal.snapshot())


def test_behavior_target_conflict_hysteresis_and_unknown_threat():
    cfg=PsychologyConfig();me=person();a=robot(x=13.);b=robot(x=10.8,i=-2)
    state,intent=choose_behavior(me,.8,0.,BehaviorState(),{-1:(a,0.,1.)},{-1:1.},0.,10**9,cfg)
    assert intent.mode=='CURIOUS' and intent.desired_velocity_x>0 and not intent.allow_task_progress
    next_state,intent=choose_behavior(me,.8,.01,state,{-1:(a,0.,1.),-2:(b,1.,1.)},{-1:1.},1.,10**9+25_000_000,cfg)
    assert intent.mode=='SCARED' and intent.target_robot_id==-2
    _,intent=choose_behavior(me,.8,.9,next_state,{}, {},0.,4*10**9,cfg)
    assert intent.mode=='REGULAR' and intent.vigilant and not intent.has_target_robot
    state=BehaviorState('CURIOUS',entered_ns=0,target_robot_id=-1)
    _,intent=choose_behavior(me,.5,0.,state,{-1:(a,0.,1.)},{-1:1.},0.,2*10**9,cfg)
    assert intent.mode=='CURIOUS'
    _,intent=choose_behavior(me,.3,0.,state,{-1:(a,0.,1.)},{-1:1.},0.,2*10**9,cfg)
    assert intent.mode=='REGULAR'


def test_enhanced_second_hop_and_default_recovery():
    # Frozen directed-chain mechanism test, independent of motion/FOV feedback.
    def chain(beta, mu):
        a=b=0.; peaks=[0.,0.]
        for k in range(4000):
            source=.8 if k<800 else 0.
            old_a=a
            a=exponential_step(a,beta*4/9*.8*source,mu,.025)
            b=exponential_step(b,beta*4/9*.8*old_a,mu,.025)
            peaks=[max(peaks[0],a),max(peaks[1],b)]
        return peaks,(a,b)
    weak,recovered=chain(.12,.25);strong,_=chain(2.4,.25)
    assert weak[0]>.08 and weak[1]<.08
    assert min(strong)>.65
    assert max(recovered)<1e-6


def test_time_refinement():
    def run(dt):
        x=[.1,.2]
        for _ in range(round(10/dt)):
            x=[exponential_step(x[i],.1+.12*.8*x[1-i],.25,dt) for i in (0,1)]
        return x
    values=[run(dt) for dt in (.025,.0125,.00625)]
    errors=[max(abs(a-b) for a,b in zip(values[i],values[i+1])) for i in (0,1)]
    assert 0<errors[1]<errors[0]*.6


def test_approach_feasibility_wait_and_retry_preserves_activation():
    m=model();state=m.initial_states();state[1].variables['curiosity']=.8
    people=[person(1),person(2,x=11.5,yaw=math.pi)]
    p=m.step(make_context(0,.025,people,[robot(x=14.)]),state)
    assert p.intents[1].mode=='REGULAR' and p.intents[1].override_desired_velocity
    assert p.intents[1].desired_velocity_x==0
    assert p.states[1].behavior.reason=='approach_blocked'
    assert p.states[1].variables['curiosity']>.7
    assert p.diagnostics['agents'][1]['approach_conflicts']==[2]
    clear=[person(1),person(2,x=25.,yaw=math.pi)]
    wait=m.step(make_context(25_000_000,.025,clear,[robot(x=14.)]),p.states)
    assert wait.states[1].behavior.reason=='approach_wait'
    assert wait.intents[1].desired_velocity_x==0
    retry=m.step(make_context(3_000_000_000,.025,clear,[robot(x=14.)]),wait.states)
    assert retry.intents[1].mode=='CURIOUS'


def test_crossing_risk_is_not_absolute_robot_speed():
    p=person();cfg=PsychologyConfig()
    crossing=Entity(-1,'robot_1',11.,8.8,0.,1.,.35)
    assert risk(p,crossing,cfg)>0
    away=replace(crossing,vy=-1.)
    assert risk(p,away,cfg)==0


def test_task_return_keeps_neighbor_avoidance_until_goal_reached():
    m=model();state=m.initial_states();state[1].behavior.returning_to_task=True
    people=[person(1),replace(person(2,x=11.5,yaw=math.pi),vx=-.5)]
    p=m.step(make_context(0,.025,people,[]),state)
    assert p.intents[1].mode=='REGULAR' and p.intents[1].allow_task_progress
    assert p.intents[1].override_desired_velocity and p.intents[1].desired_velocity_x<0
    assert p.states[1].variables=={'curiosity':0.,'fear':0.}
    assert p.states[1].behavior.returning_to_task
    assert p.diagnostics['agents'][1]['avoidance_conflicts']==[2]
    q=m.step(make_context(25_000_000,.025,[replace(people[0],x=20.),people[1]],[]),p.states)
    assert not q.states[1].behavior.returning_to_task
    # Core rotates the cyclic queue in the arrival tick. The saved anchor must
    # clear recovery even when the next psychology tick sees a new first goal.
    rotated=replace(people[0],goals=((20.,12.),(20.,10.)))
    q=m.step(make_context(25_000_000,.025,[rotated,people[1]],[]),p.states)
    assert not q.states[1].behavior.returning_to_task and q.states[1].behavior.return_goal is None


def test_neighbor_memory_hidden_truth_isolation_and_expiry():
    m=model();state=m.initial_states();state[1].behavior.returning_to_task=True
    people=[person(1),replace(person(2,x=11.5),vx=-.5)]
    first=m.step(make_context(0,.025,people,[]),state)
    blind=replace(people[0],yaw=math.pi)
    a=m.step(make_context(25_000_000,.025,[blind,people[1]],[]),first.states)
    b=m.step(make_context(25_000_000,.025,[blind,replace(people[1],x=50.,vx=9.)],[]),first.states)
    assert a.intents[1]==b.intents[1] and a.states[1]==b.states[1]
    assert a.states[1].neighbors[2].last_seen_ns==0
    expired=m.step(make_context(3_000_000_000,.025,[blind,people[1]],[]),a.states)
    assert not expired.states[1].neighbors
    assert not expired.intents[1].override_desired_velocity


def test_neighbor_avoidance_joint_constraints_and_permutation():
    from arena_multi_hunav.behavior import avoid_neighbors,Intent
    me=person();cfg=PsychologyConfig()
    others=[replace(person(2,x=11.2),vx=-.1),replace(person(3,x=10.),y=11.2,vy=-.1)]
    intent=Intent(override_desired_velocity=True,desired_velocity_x=.5,desired_velocity_y=.5)
    result,conflicts,feasible=avoid_neighbors(me,intent,others,cfg)
    assert feasible and conflicts==[2,3]
    assert result.desired_velocity_x<=-.075+1e-9 and result.desired_velocity_y<=-.075+1e-9
    assert (result,conflicts,feasible)==avoid_neighbors(me,intent,list(reversed(others)),cfg)


def test_neutral_regular_is_unmodified_even_with_neighbors():
    m=model();people=[person(1),person(2,x=11.5,yaw=math.pi)]
    proposal=m.step(make_context(0,.025,people,[]),m.initial_states())
    assert all(not i.override_desired_velocity for i in proposal.intents.values())


def test_target_hold_margin_and_expiry_bypass():
    from arena_multi_hunav.behavior import select_target
    cfg=PsychologyConfig();old=BehaviorState('CURIOUS',target_robot_id=-1,target_since_ns=0)
    assert select_target({-1:.8,-2:.9},old,2*10**9,cfg)==-1
    assert select_target({-1:.5,-2:.9},old,2*10**9,cfg)==-2
    assert select_target({-2:.1},old,10,cfg)==-2
    assert select_target({-1:.1,-2:1.},old,10,cfg,True)==-2


@pytest.mark.parametrize('update',[{'mu_F':0.},{'fear_on':.1,'fear_off':.2},{'robot_fov_deg':400.},{'gap_warn':.1},{'curiosity_speed':2.},{'beta_F':float('nan')},{'unexpected':1}])
def test_invalid_config(update):
    document=PsychologyConfig().document();document['parameters'].update(update)
    with pytest.raises(ValueError):PsychologyConfig.parse(document)
