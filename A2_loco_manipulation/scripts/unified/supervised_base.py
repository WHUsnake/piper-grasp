"""Operator-supervised bridge to the existing base driver, not physical exclusivity.
No output on startup. Only a claimed navigation stage can forward velocity.
"""
import math,time,threading,sys
from pathlib import Path
from common import rpc,serve

MOTION_TOPICS=('/cmd_vel','/mode_switch','/upDown','/temStatus','/a2/sport_cmd','/a2/body_height','/a2/speed_level','/a2/euler','/a2/body_pose')
class Bridge:
    def __init__(self,transport,directory,clock=time.monotonic):
        self.t=transport;self.directory=Path(directory);self.clock=clock;self.lock=threading.RLock()
        self.owner=None;self.stage='stationary';self.renewed=0.;self.fault='';self.sent=False;self.mode_sent=False
    def health(self):
        conflicts=self.t.conflicts()
        if conflicts:raise RuntimeError('Other base command source: '+str(conflicts))
        if not self.t.driver_ready():raise RuntimeError('Original base driver unavailable')
    def dispatch(self,q):
        with self.lock:
            if q['method']=='health':
                self.health();return {'available':True,'exclusive':False,'supervised':True,'fault':self.fault}
            tid=q.get('task_id')
            if q['method']=='claim':
                if self.owner is not None:raise RuntimeError('Bridge already claimed; reconciliation required')
                self.health();self.owner=tid;self.renewed=self.clock()
            elif tid!=self.owner or not tid:raise RuntimeError('Task owner mismatch')
            elif q['method']=='abort':
                self.stop('operator_abort');return {'task_id':tid,'requested':True,'physical_stop_confirmed':False}
            elif q['method']=='renew':
                if self.fault:raise RuntimeError(self.fault)
                stage=q['stage']
                if stage not in ('navigation','stationary','grasp'):raise ValueError('Invalid phase')
                if self.stage=='grasp' and stage!='grasp':raise RuntimeError('Grasp phase latched')
                if self.stage=='navigation' and stage!='navigation':self.t.publish((0.,0.,0.))
                self.stage=stage;self.renewed=self.clock();self.health()
            else:raise ValueError('Unknown operation')
            return {'task_id':tid,'stage':self.stage,'supervised':True,'exclusive':False}
    def stop(self,reason):
        self.fault=reason
        if self.sent:self.t.publish((0.,0.,0.))
    def tick(self):
        with self.lock:
            if not self.owner:return
            if self.fault:
                # Until acknowledgement/recovery, keep our last command zero.
                if self.sent:self.t.publish((0.,0.,0.))
                return
            try:
                if self.clock()-self.renewed>.6:raise RuntimeError('Supervisor heartbeat lost')
                self.health()
                arm=rpc(self.directory/'arm.sock','feedback',timeout=.2)
                if arm['errors']:raise RuntimeError('Arm feedback lost')
                base=rpc(self.directory/'base.sock','feedback',timeout=.2)
                if self.clock()-base.get('received',-100)>.3 or not 0<=base.get('age',100)<=.3:raise RuntimeError('Measured base feedback lost')
                nav=rpc(self.directory/'nav.sock','get_status',timeout=.2,task_id=self.owner)
                if self.stage=='navigation' and nav['result']=='RUNNING':
                    if nav.get('task_id')!=self.owner or nav.get('command_age',100)>.2:raise RuntimeError('Navigation stale/wrong task')
                    if not self.mode_sent:
                        if nav.get('mode_request')!='lingdong' or nav.get('mode_age',100)>.5:raise RuntimeError('Fresh original navigation mode request required')
                        self.t.mode('lingdong');self.mode_sent=True
                    v=nav['velocity']
                    if len(v)!=3 or not all(math.isfinite(x) for x in v):raise RuntimeError('Invalid velocity')
                    self.t.publish(tuple(max(-cap,min(cap,x)) for x,cap in zip(v,(.10,.05,.15))));self.sent=True
                else:
                    if self.sent:self.t.publish((0.,0.,0.))
                    if self.stage=='grasp' and not nav.get('stopped'):raise RuntimeError('Base is not stopped during grasp')
            except Exception as exc:
                self.stop(str(exc))
                for side in ('nav','arm'):
                    try:rpc(self.directory/(side+'.sock'),'abort',timeout=.2,task_id=self.owner)
                    except Exception:pass
class RosTransport:
    def __init__(self):
        import rospy
        from geometry_msgs.msg import Twist
        from std_msgs.msg import String
        self.r=rospy;self.Twist=Twist;self.String=String
        rospy.init_node('a2_supervised_base',disable_signals=True)
        self.pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        self.mode_pub=rospy.Publisher('/mode_switch',String,queue_size=1)
    def graph(self):return self.r.get_master().getSystemState()[2]
    def conflicts(self):
        return [(t,n) for t,names in self.graph()[0] if t in MOTION_TOPICS for n in names if n!='/a2_supervised_base']
    def driver_ready(self):
        return any(t=='/cmd_vel' and names==['/unitree_a2_ros_control'] for t,names in self.graph()[1])
    def mode(self,value):self.mode_pub.publish(self.String(data=value))
    def publish(self,v):
        m=self.Twist();m.linear.x,m.linear.y,m.angular.z=v;self.pub.publish(m)
if __name__=='__main__':
    a=Bridge(RosTransport(),Path(sys.argv[1]))
    def loop():
        while True:a.tick();time.sleep(.05)
    threading.Thread(target=loop,daemon=True).start()
    serve(Path(sys.argv[1])/'supervised.sock',a.dispatch).serve_forever()
