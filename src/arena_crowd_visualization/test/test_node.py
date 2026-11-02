"""Exercise generated messages, source timestamps, expiry and bounded projection."""
import json
from pathlib import Path
from types import SimpleNamespace
import pytest
import rclpy
from std_msgs.msg import String
from arena_people_msgs.msg import Pedestrian, Pedestrians
from arena_crowd_visualization.node import Visualizer, stamp, ns

ROOT=Path(__file__).resolve().parents[3]

@pytest.fixture
def node():
    config=ROOT/'config/hunav/multi_14_psychology_v2_multigoal.yaml'
    rclpy.init(args=['--ros-args','-p',f'people_config:={config}'])
    n=Visualizer()
    n.sent={}
    for key in ('actual','core','psych','stimulus','marker','status'):
        n.sent[key]=[]
        setattr(n,key+'_pub',SimpleNamespace(publish=n.sent[key].append))
    yield n
    n.destroy_node();rclpy.shutdown()


def frame(node,epoch=1,step=1):
    name,i=next(iter(node.ids.items()))
    actual=Pedestrians();actual.header.frame_id='map';actual.header.stamp=stamp(1000000000)
    p=Pedestrian(name=name);p.pose.orientation.w=1.;p.pose.position.x=2.;p.twist.linear.x=.3
    actual.pedestrians=[p];node.receive('actual',actual)
    d=dict(schema_version=3,epoch=epoch,step=step,stamp_ns=1025000000,input_stamp_ns=1000000000,
        agents={str(i):dict(profile='immune_dynamic',new=dict(curiosity=0.,fear=0.),behavior=dict(mode='REGULAR'),
            direct_C=0.,direct_F=0.,social_C=0.,social_F=0.)},states={str(i):dict(surprise=0.)},
        people={str(i):dict(name=name,x=2.,y=0.,yaw=0.,vx=.3,vy=0.)},robots={},edges=[])
    node.receive('psych',String(data=json.dumps(d)));node.next_psych=0.;node.update()
    return d


def test_truth_and_stimulus_use_distinct_source_times(node):
    frame(node)
    assert ns(node.sent['actual'][0].header.stamp)==1000000000
    assert ns(node.sent['psych'][0].header.stamp)==1025000000
    assert ns(node.sent['stimulus'][0].header.stamp)==1000000000
    assert node.sent['actual'][0].people[0].speed==.3
    assert node.sent['psych'][0].people[0].has_surprise
    node.next_psych=0.;node.update()
    assert len(node.sent['psych'])==len(node.sent['actual'])==1


def test_expired_data_and_epoch_clear_positions(node):
    frame(node)
    node.progress={k:-1e30 for k in node.progress}
    node.next_psych=0.;node.update()
    assert len(node.sent['marker'][-1].markers)==1
    assert json.loads(node.sent['status'][-1].data)['paused_or_stale']
    frame(node,epoch=2)
    assert len(node.sent['marker'][-1].markers)==1


def test_invalid_sample_does_not_raise_into_executor(node):
    node.receive('psych',String(data='not json'));node.tick()
    assert node.sent['marker'][-1].markers[0].action==3


def test_ros_transport_only_emits_visualization_topics(node):
    import time
    from rclpy.executors import SingleThreadedExecutor
    from arena_crowd_visualization_msgs.msg import PedestrianPsychologyArray
    observer=rclpy.create_node('crowd_visualization_test_observer')
    received=[]
    publisher=observer.create_publisher(String,'/multirobot/hunav/psychology',2)
    node.psych_pub=node.create_publisher(PedestrianPsychologyArray,'/multirobot/viz/psychology',2)
    subscription=observer.create_subscription(PedestrianPsychologyArray,'/multirobot/viz/psychology',received.append,2)
    d=frame(node);d['step']=2
    executor=SingleThreadedExecutor();executor.add_node(node);executor.add_node(observer)
    try:
        until=time.monotonic()+3
        while time.monotonic()<until and not any(m.step==2 for m in received):
            publisher.publish(String(data=json.dumps(d)))
            executor.spin_once(timeout_sec=.05)
        assert any(m.step==2 for m in received)
    finally:
        executor.remove_node(observer);executor.remove_node(node);executor.shutdown();observer.destroy_node()


def test_marker_objects_are_reused_and_cache_is_bounded(node):
    frame(node)
    original={key:id(value) for key,value in node.marker_cache.items()}
    for step in range(2,20):frame(node,step=step)
    assert {key:id(value) for key,value in node.marker_cache.items()}==original
    assert len(original)==2  # one person and one label; no zero-strength arrows


def test_serialized_truth_is_decoded_only_when_replaced(node):
    from rclpy.serialization import serialize_message
    frame(node)
    actual=node.cache['actual'];actual.header.stamp=stamp(2000000000)
    encoded=serialize_message(actual);node.receive('actual',encoded);node.next_psych=0.;node.update()
    decoded=node.actual_decoded
    assert ns(node.sent['actual'][-1].header.stamp)==2000000000
    node.next_psych=0.;node.update()
    assert node.actual_decoded is decoded


def test_noop_heartbeat_keeps_actual_labels_without_invented_psychology(node):
    frame(node)
    published=len(node.sent['psych'])
    node.receive('psych',String(data=json.dumps(dict(schema_version=1,plugin=dict(model='noop'),states={}))))
    node.next_psych=0.;node.update()
    markers=node.sent['marker'][-1].markers
    assert any(m.ns=='labels' and 'N/A' in m.text for m in markers)
    assert len(node.sent['psych'])==published
    status=json.loads(node.sent['status'][-1].data)
    assert status['actual_fresh'] and not status['psychology_fresh']
    assert node.doc is None and not node.states
