import pytest
from arena_crowd_visualization.model import arrows, project

def test_weighted_direction_and_limits():
 d={'agents':{'13':{'visible_robots':{'-1':{'visibility':.5,'novelty':.8,'risk':.2}}}},
    'edges':[{'source_id':i,'target_id':13,'weight':.1,'cue_C':i/10,'cue_F':0} for i in range(1,9)]}
 edges=arrows(d,13)
 assert edges[0]==(.4,-1,.4,.1,'robot')
 assert [e[1] for e in edges[1:]]==[8,7,6]
 assert arrows(d,14)==[]

def test_zero_cue_is_not_a_stimulus():
 assert arrows({'agents':{},'edges':[dict(source_id=1,target_id=2,weight=1,cue_C=0,cue_F=0)]},2)==[]

def test_unknown_schema_rejected():
 with pytest.raises(ValueError):project({'schema_version':99})
