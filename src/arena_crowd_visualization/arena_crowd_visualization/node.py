"""Latest-only display adapter; failures never propagate to simulation control."""
import json
import math
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, ColorRGBA
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from arena_people_msgs.msg import Pedestrians
from visualization_msgs.msg import Marker, MarkerArray
from arena_crowd_visualization_msgs.msg import (PedestrianMotion, PedestrianMotionArray,
    PedestrianPsychology, PedestrianPsychologyArray, PedestrianStimulus, PedestrianStimulusArray)
from .model import project, arrows, COLORS, number

PREFIX='/multirobot/viz/'

def stamp(ns):
    from builtin_interfaces.msg import Time
    return Time(sec=int(ns)//1000000000,nanosec=int(ns)%1000000000)

def ns(t): return t.sec*1000000000+t.nanosec

def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

class Visualizer(Node):
    def __init__(self):
        super().__init__('crowd_visualizer')
        self.declare_parameter('people_config','');self.declare_parameter('focus_id',13)
        self.declare_parameter('robot_names',['robot_1','robot_2'])
        self.ids={}
        path=self.get_parameter('people_config').value
        if path:
            with open(path) as f: spec=yaml.safe_load(f)['hunav_loader']['ros__parameters']
            self.ids={name:int(spec[name]['id']) for name in spec['agents']}
        self.cache={};self.progress={};self.keys={};self.robot_positions={}
        self.doc=None;self.states=[];self.last_psych=None;self.last_actual=None;self.last_error=0.
        self.actual_raw=None;self.actual_decoded=None
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(String,'/multirobot/hunav/psychology',lambda m:self.receive('psych',m),qos,raw=True)
        self.create_subscription(Pedestrians,'/multirobot/hunav/actual_people',lambda m:self.receive('actual',m),qos,raw=True)
        for name in self.get_parameter('robot_names').value:
            self.create_subscription(Odometry,'/'+name+'/odom',lambda m,n=name:self.receive(n,m),qos)
        self.actual_pub=self.create_publisher(PedestrianMotionArray,PREFIX+'actual',2)
        self.core_pub=self.create_publisher(PedestrianMotionArray,PREFIX+'core',2)
        self.psych_pub=self.create_publisher(PedestrianPsychologyArray,PREFIX+'psychology',2)
        self.stimulus_pub=self.create_publisher(PedestrianStimulusArray,PREFIX+'stimulus',2)
        self.marker_pub=self.create_publisher(MarkerArray,PREFIX+'markers',2)
        self.status_pub=self.create_publisher(String,PREFIX+'status',2)
        self.marker_cache={}
        self.clear_marker=Marker(action=Marker.DELETEALL)
        self.next_psych=0.
        self.display_timer=self.create_timer(.1,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive(self,key,msg): self.cache[key]=msg

    def advancing(self,key,value):
        old=self.keys.get(key)
        if old!=value:
            self.keys[key]=value;self.progress[key]=time.monotonic()
        return old!=value

    def fresh(self,key): return time.monotonic()-self.progress.get(key,-1e30)<1.

    def tick(self):
        try: self.update()
        except Exception as e:
            if time.monotonic()-self.last_error>5:
                self.get_logger().warning('Visualization sample dropped: '+str(e));self.last_error=time.monotonic()
            self.marker_pub.publish(MarkerArray(markers=[self.clear_marker]))

    def update(self):
        now=time.monotonic()
        raw_actual=self.cache.get('actual')
        if raw_actual is not self.actual_raw:
            self.actual_raw=raw_actual
            self.actual_decoded=(deserialize_message(raw_actual,Pedestrians)
                if isinstance(raw_actual,bytes) else raw_actual)
        actual=self.actual_decoded
        positions={}
        if actual is not None:
            t=ns(actual.header.stamp)
            if self.last_actual is not None and t<self.last_actual:
                self.doc=None;self.states=[];self.progress.clear();self.keys.clear();self.last_psych=None
                self.cache.pop('psych',None)
            self.last_actual=t
            changed=self.advancing('actual',t)
            if self.fresh('actual'):
                output=PedestrianMotionArray(header=actual.header,source='isaac_actual_finite_difference')
                if actual.header.frame_id!='map': raise ValueError('actual frame must be map')
                for p in actual.pedestrians:
                    if p.name not in self.ids: continue
                    pos=p.pose.position;v=p.twist.linear;i=self.ids[p.name]
                    row=PedestrianMotion(id=i,name=p.name,x=number(pos.x),y=number(pos.y),yaw=number(yaw(p.pose.orientation)),
                        vx=number(v.x),vy=number(v.y),speed=math.hypot(v.x,v.y))
                    output.people.append(row);positions[i]=(row.x,row.y)
                if changed:self.actual_pub.publish(output)
        for name in self.get_parameter('robot_names').value:
            m=self.cache.get(name)
            if m is not None:
                self.advancing(name,ns(m.header.stamp))
                if self.fresh(name):
                    p=m.pose.pose.position;self.robot_positions[name]=(number(p.x),number(p.y))
                else:self.robot_positions.pop(name,None)
        if now<self.next_psych:return
        self.next_psych=now+.2
        raw=self.cache.get('psych')
        if raw is not None and raw is not self.last_psych:
            self.last_psych=raw
            text=deserialize_message(raw,String).data if isinstance(raw,bytes) else raw.data
            d=json.loads(text)
            if d.get('schema_version')==1 and d.get('plugin',{}).get('model')=='noop':
                # Legacy heartbeat has no psychology transaction or activations.
                self.doc=None;self.states=[]
                self.keys.pop('psych',None);self.progress.pop('psych',None)
            else:
                rows=project(d);key=(int(d['epoch']),int(d['step']))
                old=self.keys.get('psych')
                if old and (old[0]!=key[0] or key[1]<old[1]):
                    self.doc=None;self.states=[];positions={};self.cache.pop('actual',None)
                    self.last_actual=None;self.keys.pop('actual',None);self.progress.pop('actual',None)
                if self.advancing('psych',key):
                    self.doc=d;self.states=rows
                    out=PedestrianPsychologyArray(epoch=key[0],step=key[1])
                    out.header.frame_id='map';out.header.stamp=stamp(d['stamp_ns'])
                    out.input_stamp=stamp(d.get('input_stamp_ns',d['stamp_ns']-25000000))
                    out.people=[PedestrianPsychology(**row) for row in rows];self.psych_pub.publish(out)
                    stimulus=PedestrianStimulusArray(epoch=out.epoch,step=out.step)
                    stimulus.header.frame_id='map';stimulus.header.stamp=out.input_stamp
                    stimulus.people=[PedestrianStimulus(**{k:r[k] for k in ('id','name','direct_c','direct_f','social_c','social_f')}) for r in rows]
                    self.stimulus_pub.publish(stimulus)
                    core=PedestrianMotionArray(header=out.header,source='core_committed')
                    for key,p in d['people'].items():
                        core.people.append(PedestrianMotion(id=int(key),name=p['name'],x=number(p['x']),y=number(p['y']),
                            yaw=number(p['yaw']),vx=number(p['vx']),vy=number(p['vy']),speed=math.hypot(p['vx'],p['vy'])))
                    self.core_pub.publish(core)
        focus=int(self.get_parameter('focus_id').value)
        if focus not in self.ids.values():focus=min(self.ids.values(),default=0)
        healthy=self.fresh('psych') and self.doc is not None
        message=MarkerArray(markers=[self.clear_marker])
        timestamp=actual.header.stamp if actual else stamp(0)
        def marker(kind,ident,namespace):
            key=(namespace,ident)
            m=self.marker_cache.get(key)
            if m is None:
                m=Marker(type=kind,id=ident,ns=namespace)
                m.header.frame_id='map';m.pose.orientation.w=1.;m.color.a=1.;m.lifetime.sec=1
                self.marker_cache[key]=m
            m.header.stamp=timestamp
            return m
        dots=None
        if positions:
            dots=marker(Marker.SPHERE_LIST,0,'people')
            dots.scale.x=dots.scale.y=dots.scale.z=.25
            dots.points=[];dots.colors=[];message.markers.append(dots)
        rows={r['id']:r for r in self.states}
        for i,(x,y) in positions.items():
            r=rows.get(i) if healthy else None;mode=r['mode'] if r else 'N/A'
            color=COLORS.get(mode,(.6,.6,.6))
            dots.points.append(Point(x=x,y=y,z=.2))
            dots.colors.append(ColorRGBA(r=color[0],g=color[1],b=color[2],a=1.))
            m=marker(Marker.TEXT_VIEW_FACING,i,'labels');m.pose.position=Point(x=x,y=y,z=2.1)
            m.scale.z=.22;m.color.r,m.color.g,m.color.b=color
            m.text=f'P{i} {mode}'
            if r:
                s=f"{r['surprise']:.2f}" if r['has_surprise'] else 'N/A'
                m.text+=f"\nC:{r['curiosity']:.2f} F:{r['fear']:.2f} S:{s}"
                if r['phase']!='none':m.text+=' '+r['phase']
            else:m.text+=' (paused / unavailable)'
            message.markers.append(m)
        if healthy and focus in positions:
            for idx,(strength,source,c,f,kind) in enumerate(arrows(self.doc,focus)):
                if kind=='human':start=positions.get(source)
                else:
                    robot=self.doc['robots'].get(str(source),{})
                    start=self.robot_positions.get(robot.get('name',''))
                if start is None:continue
                end=positions[focus];m=marker(Marker.ARROW,idx,'stimuli')
                m.points=[Point(x=start[0],y=start[1],z=1.5),Point(x=end[0],y=end[1],z=1.5)]
                m.scale.x=.015+.04*min(strength,1.);m.scale.y=.09;m.scale.z=.14
                m.color.r,m.color.g,m.color.b=COLORS['CURIOUS' if c>=f else 'SCARED'];message.markers.append(m)
        self.marker_pub.publish(message)
        self.status_pub.publish(String(data=json.dumps(dict(focus_id=focus,actual_fresh=self.fresh('actual'),
            psychology_fresh=healthy,paused_or_stale=not healthy))))

def main():
    from rclpy.executors import SingleThreadedExecutor
    rclpy.init();node=Visualizer()
    # Four latest-only inputs in the two-robot scene. Bound dispatch as well as
    # decoding; DDS reception must not wake Python at every simulation step.
    node.display_timer.cancel()
    executor=SingleThreadedExecutor();executor.add_node(node)
    try:
        while rclpy.ok():
            started=time.monotonic()
            for _ in range(4):
                executor.spin_once(timeout_sec=0.)
            node.tick()
            time.sleep(max(0.,.1-(time.monotonic()-started)))
    except KeyboardInterrupt:pass
    finally:
        executor.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
