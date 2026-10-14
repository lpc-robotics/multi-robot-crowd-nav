import copy
from pathlib import Path
from types import SimpleNamespace
import pytest
from arena_multi_hunav.psychology import NoOpPsychology, MentalState, Proposal, Modifiers
from arena_multi_hunav.multi_adapter import load_people, commit_candidate
from arena_multi_hunav_msgs.srv import ComputeMultiAgents

ROOT=Path(__file__).resolve().parents[3]


def fixture():
    req=ComputeMultiAgents.Request(epoch=1,step=2,dt=.025)
    req.current_agents,_=load_people(ROOT/'config/hunav/multi_regular_1.yaml')
    res=ComputeMultiAgents.Response(epoch=1,step=2,success=True,updated_agents=copy.deepcopy(req.current_agents))
    res.updated_agents.header.stamp.nanosec=25_000_000
    proposal=Proposal({1:MentalState({'fear':.5})},{1:Modifiers()})
    return req,res,proposal


def test_bad_snapshot_cannot_commit_motion_or_mental_state():
    req,res,proposal=fixture();m=NoOpPsychology();m.initialize({},(1,),11)
    before=m.snapshot();bad=copy.deepcopy(before);bad['version']='invalid'
    committed=SimpleNamespace(agents=copy.deepcopy(req.current_agents),mental={1:MentalState()})
    with pytest.raises(ValueError):
        committed.agents,stamp,committed.mental=commit_candidate(m,bad,proposal,req,res)
    assert committed.agents==req.current_agents
    assert committed.mental=={1:MentalState()}
    assert m.snapshot()==before


@pytest.mark.parametrize('failure',['late','failed','task'])
def test_invalid_response_does_not_restore_candidate(failure):
    req,res,proposal=fixture();m=NoOpPsychology();m.initialize({},(1,),11);before=m.snapshot()
    if failure=='late':res.step=1
    if failure=='failed':res.success=False
    if failure=='task':res.updated_agents.agents[0].goals=[]
    with pytest.raises(ValueError):commit_candidate(m,before,proposal,req,res)
    assert m.snapshot()==before
