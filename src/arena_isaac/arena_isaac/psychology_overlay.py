"""Optional viewport-only layer. No physics or control writes."""
import math
import time
import os
import json
from pathlib import Path

class PsychologyOverlay:
    def __init__(self,node):
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension('omni.ui.scene')
        from omni.ui import scene as sc
        from omni.kit.viewport.utility import get_active_viewport_window
        from visualization_msgs.msg import MarkerArray
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        self.sc=sc;self.node=node;self.items={};self.pending=None;self.seen=0.;self.closed=False
        self.metrics_path=os.environ.get('ARENA_CROWD_VIZ_METRICS','')
        self.metrics_bins=[0]*102;self.metrics_count=0;self.metrics_sum=0.;self.metrics_max=0.
        self.window=get_active_viewport_window()
        if self.window is None:raise RuntimeError('No viewport window for crowd overlay')
        with self.window.get_frame('arena_crowd_visualization'):
            self.view=sc.SceneView()
        self.window.viewport_api.add_scene_view(self.view)
        self.node.get_logger().info("Crowd SceneView overlay ready")
        self.subscription=node.create_subscription(MarkerArray,'/multirobot/viz/markers',self.receive,
            QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))

    def receive(self,message):self.pending=message;self.seen=time.monotonic()

    def update(self):
        if self.closed:return
        try:
            if time.monotonic()-self.seen>1.:
                for transform,_ in self.items.values():transform.visible=False
            if self.pending is None:return
            message=self.pending;self.pending=None;sc=self.sc
            started=time.perf_counter() if self.metrics_path else None
            for transform,_ in self.items.values():transform.visible=False
            for m in message.markers:
                if m.action!=0 or m.type not in (9,0):continue
                key=(m.ns,m.id);color=(m.color.r,m.color.g,m.color.b,m.color.a)
                if key not in self.items:
                    with self.view.scene:
                        with sc.Transform() as transform:
                            if m.type==9:
                                shape=sc.Label('',size=14)
                            else:
                                shape=[sc.Line((0,0,0),(0,0,0),thickness=2) for _ in range(3)]
                    self.items[key]=(transform,shape)
                transform,shape=self.items[key];transform.visible=True
                if m.type==9:
                    p=m.pose.position;transform.transform=sc.Matrix44.get_translation_matrix(p.x,p.y,p.z)
                    shape.text=m.text;shape.color=color
                elif len(m.points)==2:
                    a,b=m.points;dx=b.x-a.x;dy=b.y-a.y;n=max(math.hypot(dx,dy),1e-9)
                    ends=[((a.x,a.y,a.z),(b.x,b.y,b.z))]
                    for sign in (-1,1):
                        ends.append(((b.x,b.y,b.z),(b.x-.2*dx/n+sign*.1*dy/n,b.y-.2*dy/n-sign*.1*dx/n,b.z)))
                    for line,(start,end) in zip(shape,ends):
                        line.start=start;line.end=end;line.color=color
            if started is not None:
                ms=(time.perf_counter()-started)*1000
                self.metrics_bins[min(101,math.ceil(ms*10))]+=1
                self.metrics_count+=1;self.metrics_sum+=ms;self.metrics_max=max(self.metrics_max,ms)
                if self.metrics_count%100==0:self.flush_metrics()
        except Exception as e:
            self.node.get_logger().warning('Crowd overlay disabled: '+str(e));self.close()

    def flush_metrics(self):
        if not self.metrics_path or not self.metrics_count:return
        count=0;upper=None
        for index,value in enumerate(self.metrics_bins):
            count+=value
            if count>=math.ceil(.95*self.metrics_count):
                upper=index/10 if index<=100 else None
                break
        report=dict(scope='SceneView updates, histogram upper bound 0.1ms bins',
            samples=self.metrics_count,mean_ms=self.metrics_sum/self.metrics_count,
            max_ms=self.metrics_max,p95_upper_bound_ms=upper,
            passed=upper is not None and upper<=2.)
        try:
            path=Path(self.metrics_path);path.parent.mkdir(parents=True,exist_ok=True)
            temporary=path.with_suffix('.tmp');temporary.write_text(json.dumps(report,indent=2)+'\n');temporary.replace(path)
        except OSError as exc:
            self.node.get_logger().warning('Crowd metrics unavailable: '+str(exc));self.metrics_path=''

    def close(self):
        if self.closed:return
        self.closed=True
        self.flush_metrics()
        try:
            self.node.destroy_subscription(self.subscription)
            self.window.viewport_api.remove_scene_view(self.view)
            self.view.scene.clear()
        except Exception as exc:
            self.node.get_logger().warning('Crowd overlay cleanup: '+str(exc))
        self.items.clear();self.pending=None
