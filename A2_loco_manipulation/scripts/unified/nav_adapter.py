"""ROS1 project object-navigation adapter; never forwards velocity to the base.
A separate verified authority must connect/gate outputs before real execution.
"""
import json, math, sys, threading, time
from pathlib import Path
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from a2_target_manager.msg import TargetStatus
from a2_target_manager.srv import SelectTarget
from a2_perception.msg import TrackedObjectArray
from common import serve, rpc

class Navigation:
    def __init__(self, config, directory):
        self.directory=directory; self.target=None; self.objects_at=0.; self.target_at=0.
        self.mode_request='';self.mode_at=0.
        self.config = config; self.lock = threading.RLock()
        self.task = None; self.result = 'IDLE'; self.reason = 'No task submitted'
        self.deadline = 0; self.status = ''; self.status_at = 0; self.running_seen = False
        self.odom_at = 0; self.cmd_at = 0; self.still_since = None; self.cmd_zero = False; self.velocity=[0.,0.,0.]
        self.odom_stamp = 0; self.last_stamp = 0; self.base_master = None
        rospy.Subscriber('/a2/object_navigation_status',String,self.status_cb,queue_size=10)
        rospy.Subscriber('/a2/navigation_mode_request',String,self.mode_cb,queue_size=1)
        rospy.Subscriber('/a2/navigation_velocity',Twist,self.cmd_cb,queue_size=1)
        # Both project adapter and real feedback live on 11311 in this new entrance.
        rospy.Subscriber('/target_manager/status',TargetStatus,self.target_cb,queue_size=1)
        rospy.Subscriber('/perception/objects',TrackedObjectArray,self.objects_cb,queue_size=1)
        threading.Thread(target=self.poll_base,daemon=True).start()

    def status_cb(self, m):
        with self.lock:
            self.status=m.data; self.status_at=time.monotonic()
            if not self.task or self.result not in ('RUNNING','STARTING'): return
            if m.data=='running': self.running_seen=True;self.result='RUNNING'
            if self.running_seen and m.data in ('reached','failed','cancelled'):
                self.result={'reached':'ARRIVED','failed':'FAILED','cancelled':'CANCELLED'}[m.data]
                self.reason='object_navigation_agent: '+m.data

    def mode_cb(self,m):
        self.mode_request=m.data;self.mode_at=time.monotonic()

    def cmd_cb(self,m):
        values=[m.linear.x,m.linear.y,m.linear.z,m.angular.x,m.angular.y,m.angular.z]
        with self.lock:
            self.velocity=[m.linear.x,m.linear.y,m.angular.z]
            self.cmd_at=time.monotonic(); self.cmd_zero=all(math.isfinite(v) and abs(v)<1e-6 for v in values)
            if not self.cmd_zero:self.still_since=None

    def target_cb(self,m):
        if 0 <= (rospy.Time.now()-m.header.stamp).to_sec() <= .5:
            self.target=m;self.target_at=time.monotonic()
    def objects_cb(self,m):
        if 0 <= (rospy.Time.now()-m.header.stamp).to_sec() <= .5:self.objects_at=time.monotonic()
    def poll_base(self):
        while not rospy.is_shutdown():
            try:
                data=rpc(self.directory/'base.sock','feedback',timeout=.3)
                now=time.monotonic()
                with self.lock:
                    stamp=data['stamp']; xyz=data['linear']; angular=data['angular']
                    fresh=now-data['received']<.5 and 0<=data['age']+now-data['received']<=.5
                    good=(fresh and stamp>self.last_stamp and all(math.isfinite(x) for x in xyz+angular) and
                          math.sqrt(sum(x*x for x in xyz))<self.config['linear_stop_mps'] and
                          math.sqrt(sum(x*x for x in angular))<self.config['angular_stop_rps'] and
                          self.cmd_zero and now-self.cmd_at<.3 and self.result!='RUNNING')
                    if now-self.odom_at>.5:self.still_since=None
                    if stamp!=self.last_stamp:
                        self.last_stamp=stamp;self.odom_at=data['received']
                        self.still_since=(self.still_since or now) if good else None
                    elif not fresh:self.still_since=None
            except Exception:
                self.still_since=None
            time.sleep(.05)

    def stopped(self):
        now=time.monotonic()
        return bool(self.still_since and now-self.still_since>=self.config['settle_seconds'] and
                    now-self.odom_at<self.config['feedback_max_age'] and now-self.cmd_at<.3 and self.cmd_zero)

    def trigger(self,name):
        rospy.wait_for_service(name,timeout=2)
        r=rospy.ServiceProxy(name,Trigger)()
        if not r.success:raise RuntimeError(name+': '+r.message)
        return r.message

    def identity(self):
        t=self.target
        if t is None or time.monotonic()-self.target_at>.5 or not t.locked or not t.position_valid:return None
        return {'tracker_session':t.tracker_session,'track_id':t.track_id,'class_id':t.class_id}
    def select_target(self):
        if self.target is None or time.monotonic()-self.target_at>.5:raise RuntimeError('Target manager unavailable')
        if self.target.task_active:raise RuntimeError('Other navigation task active')
        if not self.target.locked:
            rospy.wait_for_service('/target_manager/select',timeout=2)
            result=rospy.ServiceProxy('/target_manager/select',SelectTarget)(-1)
            if not result.success:raise RuntimeError(result.message)
        end=time.monotonic()+2
        while time.monotonic()<end:
            identity=self.identity()
            if identity:return identity
            time.sleep(.05)
        raise RuntimeError('No fresh stable locked bottle')

    def dispatch(self,q):
        method=q['method']
        if method=='health':
            errors=[]
            services=rospy.get_master().getSystemState()[2][2]
            names={name for name,_ in services}
            for name in ('/object_navigation_agent/start','/object_navigation_agent/cancel'):
                if name not in names:errors.append(name+' unavailable')
            if time.monotonic()-self.odom_at>.5: errors.append('Measured /a2/odom missing/stale')
            if time.monotonic()-self.objects_at>.5:errors.append('Project RGB-D tracking missing/stale')
            if time.monotonic()-self.target_at>.5:errors.append('Target manager missing/stale')
            if time.monotonic()-self.cmd_at>.3: errors.append('Project navigation command stream missing/stale')
            return {'errors':errors,'status':self.status,'stopped':self.stopped(),
                    'base_output_connected':False,'interface':'project object_navigation_agent'}
        if method=='navigate':
            with self.lock:
                if self.task:raise RuntimeError('Navigation adapter is one-shot; restart after reconciliation')
                if q['goal']!={'target_class':'bottle','stop_distance':0.52}:
                    raise ValueError('Supported goal: target_class=bottle, stop_distance=0.52 (trigger 0.54m); launch configuration must match')
                if self.status not in ('idle','cancelled','reached','failed'):raise RuntimeError('Navigation state not idle')
                self.task=q['task_id'];self.result='STARTING';self.running_seen=False
                self.deadline=time.monotonic()+q['duration']
            try:
                self.select_target()
                self.trigger('/object_navigation_agent/start')
                if self.result=='STARTING':self.result='RUNNING'
            except Exception as exc:
                self.result='FAILED';self.reason=str(exc);raise
            return {'accepted':True,'task_id':self.task}
        if method=='select_target':return self.select_target()
        if method=='get_status':
            if self.task and q.get('task_id') not in (None,self.task):raise RuntimeError('task_id mismatch')
            if self.result=='RUNNING' and time.monotonic()>self.deadline:
                try:self.trigger('/object_navigation_agent/cancel')
                finally:self.result='TIMEOUT';self.reason='Navigation deadline exceeded'
            return {'task_id':self.task,'result':self.result,'reason':self.reason,'stopped':self.stopped(), 'identity':self.identity(),'velocity':list(self.velocity),'command_age':time.monotonic()-self.cmd_at,'mode_request':self.mode_request,'mode_age':time.monotonic()-self.mode_at}
        if method=='abort':
            if not self.task or q.get('task_id')!=self.task:return {'requested':False,'confirmed':False,'reason':'No matching owned navigation task'}
            self.trigger('/object_navigation_agent/cancel')
            end=time.monotonic()+3
            while time.monotonic()<end:
                if self.status=='cancelled' and self.stopped():return {'requested':True,'confirmed':True}
                time.sleep(.05)
            return {'requested':True,'confirmed':False,'reason':'Cancellation/physical stop feedback not confirmed'}
        raise ValueError('Unknown method')

if __name__=='__main__':
    config=json.loads(Path(sys.argv[1]).read_text())
    rospy.init_node('a2_unified_navigation',disable_signals=True)
    adapter=Navigation(config,Path(sys.argv[2]).parent)
    serve(sys.argv[2],adapter.dispatch).serve_forever()
