import copy, math
from dataclasses import replace
import pytest
from arena_multi_hunav.psychology import Entity, make_context, prepare_step
from arena_multi_hunav.psychology_v2 import SocialPsychologyV2
from arena_multi_hunav.psychology_v2_config import PsychologyConfigV2, ParametersV2, Individual
from arena_multi_hunav.perception import risk, playful_motion_risk


def model(profile='playful', enabled=1.):
    m=SocialPsychologyV2()
    m.initialize(PsychologyConfigV2(ParametersV2(),{1:Individual(profile,-1,dict(playful_motion_fear=enabled,fear_on=.8))}).document(),(1,),11)
    return m


@pytest.mark.parametrize('distance',[.8,1.2])
def test_stationary_and_slow_robot_do_not_create_playful_emergency_by_proximity_alone(distance):
    me=Entity(1,'p',0.,0.,0.,0.,.4,((5.,0.),))
    robot=Entity(-1,'robot_1',distance,0.,-.1,0.,.35)
    modern=model();legacy=model(enabled=0);s=modern.initial_states();old=legacy.initial_states()
    for k in range(400):
        ctx=make_context(k*25_000_000,.025,(me,),(robot,))
        a=modern.step(ctx,s);b=legacy.step(ctx,old);s=a.states;old=b.states
        assert a.intents[1].mode!='SCARED'
        assert a.diagnostics['agents'][1]['direct_C']==b.diagnostics['agents'][1]['direct_C']
        assert s[1].variables['curiosity']==pytest.approx(old[1].variables['curiosity'])
    assert s[1].variables['fear']<.2
    if distance==.8: assert old[1].behavior.mode=='SCARED'
    else: assert s[1].variables['curiosity']>0.


@pytest.mark.parametrize('robot_velocity,human_velocity,expected', [((-.9,0.),(0.,0.),True),((.9,0.),(0.,0.),False),((0.,.9),(0.,0.),False),((0.,0.),(1.,0.),False)])
def test_collision_prediction_requires_robot_directed_approach(robot_velocity,human_velocity,expected):
    me=Entity(1,'p',0.,0.,*human_velocity,.4)
    robot=Entity(-1,'r',2.,0.,*robot_velocity,.35)
    cfg=ParametersV2();fr,d=playful_motion_risk(me,robot,risk(me,robot,cfg),cfg)
    assert (fr>=cfg.emergency_risk)==expected
    if expected:assert d['envelope_ttc_s']==pytest.approx((2.-.85)/.9)


def test_any_visible_charging_robot_overrides_playful_and_is_transactional():
    from test_psychology_v2 import qualified_playful
    m,states,me,robot,t=qualified_playful(playful_motion_fear=1.,fear_on=.8)
    assert states[1].phase=='intercept'
    charging=Entity(-2,'robot_2',me.x-2.,me.y,.9,0.,.35)
    before=copy.deepcopy(states)
    p,_=prepare_step(m,make_context(t,.025,(me,),(robot,charging)),states)
    assert states==before
    assert p.intents[1].mode=='SCARED' and p.intents[1].target_robot_id==-2
    assert p.states[1].phase=='none' and p.states[1].exit_reason=='playful_emergency'
    assert p.states[1].episode_counts==before[1].episode_counts


def test_profile_eligibility_and_default_preserve_standard_response():
    me=Entity(1,'p',0.,0.,0.,0.,.4)
    robot=Entity(-1,'r',.8,0.,0.,0.,.35)
    m=model('standard')
    p=m.step(make_context(0,.025,(me,),(robot,)),m.initial_states())
    assert p.intents[1].mode=='SCARED'
    assert p.diagnostics['agents'][1]['motion_fear_appraisal']=={}


@pytest.mark.parametrize('values',[{'playful_motion_fear':.5},{'playful_imminent_ttc_s':3.},{'playful_collision_margin':-.1},{'playful_danger_approach_speed':0.}])
def test_motion_appraisal_configuration_rejects_ambiguous_values(values):
    with pytest.raises(ValueError):ParametersV2.from_values(values)


def test_attention_is_bounded_transactional_and_does_not_discover_hidden_robots():
    m=SocialPsychologyV2()
    m.initialize(PsychologyConfigV2(ParametersV2(threat_attention_s=.1),{}).document(),(1,),11)
    me=Entity(1,'p',0.,0.,0.,0.,.4,((5.,0.),))
    robot=Entity(-1,'r',1.,0.,-.4,0.,.35)
    states=m.initial_states()
    from arena_multi_hunav.behavior import RobotMemory
    states[1].memories[-1]=RobotMemory(-1,'r',1.,0.,-.4,0.,.35,0,1.)
    states[1].behavior=replace(states[1].behavior,mode='CURIOUS',target_robot_id=-1)
    before=copy.deepcopy(states)
    p,_=prepare_step(m,make_context(0,.025,(me,),(robot,)),states)
    assert states==before and p.states[1].attention_until_ns[-1]==125_000_000
    states=p.states
    behind=replace(me,yaw=math.pi)
    unseen=Entity(-2,'unseen',1.,0.,-.9,0.,.35)
    for k in range(1,5):
        p=m.step(make_context(k*25_000_000,.025,(behind,),(robot,unseen)),states)
        d=p.diagnostics['agents'][1]
        assert d['visible_robots'][-1]['knowledge']=='tracked_attention'
        assert d['direct_F']>0 and d['direct_C']==0
        assert -2 not in p.states[1].memories
        assert p.states[1].attention_until_ns[-1]==125_000_000
        states=p.states
    p=m.step(make_context(125_000_000,.025,(behind,),(robot,unseen)),states)
    assert not p.states[1].attention_until_ns
    assert p.diagnostics['agents'][1]['direct_F']==0
    assert p.diagnostics['agents'][1]['visible_robots']=={}


@pytest.mark.parametrize('value',[-.1,5.1])
def test_attention_bounds(value):
    with pytest.raises(ValueError):ParametersV2.from_values({'threat_attention_s':value})
