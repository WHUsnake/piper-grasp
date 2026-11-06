"""ROS1 transport to the guarded driver; no SDK or direct /cmd_vel publisher.
Each command requires an owned lease, fresh local inputs and driver acknowledgement.
"""
import json, math, threading, time, sys
from pathlib import Path
from common import rpc, serve


def truth(value):
    return value is True or value == 'true'

class Authority:
    def __init__(self, transport, directory, clock=time.monotonic):
        self.t=transport;self.directory=Path(directory);self.clock=clock
        self.lock=threading.RLock();self.owner=None;self.token=None;self.boot=None
        self.seq=0;self.renewed=-1.;self.stage='stationary';self.fault='';self.sent=0

    def status(self):
        d=self.t.status()
        if not d or self.clock()-d.get('_received',-100)>0.25:
            raise RuntimeError('Guarded driver acknowledgement missing/stale')
        if not truth(d.get('legacy_inputs_disabled')) or not truth(d.get('enabled')):
            raise RuntimeError('Guarded driver disabled or old motion inputs still enabled')
        if not truth(d.get('base_feedback_fresh')):raise RuntimeError('Guarded driver base feedback stale')
        return d

    def dispatch(self,q):
        with self.lock:
            method=q['method']
            if method=='health':
                d=self.status();return {'available':True,'driver':d,'fault':self.fault}
            tid=q.get('task_id')
            if not isinstance(tid,str) or not tid:raise ValueError('task_id required')
            if method=='claim':
                if self.owner is not None:raise RuntimeError('Authority already owned; explicit recovery required')
                d=self.status()
                if d.get('stage')!='idle':raise RuntimeError('Driver is not idle')
                result=self.t.claim()
                self.owner=tid;self.token=result['token'];self.boot=result['boot_id']
                self.renewed=self.clock();self.stage='stationary'
                self.send((0.,0.,0.))
            elif tid!=self.owner:raise RuntimeError('Authority task_id mismatch')
            elif method=='abort':
                self.fault='operator_abort';self.t.abort()
                return {'task_id':tid,'requested':True,'exclusive':False,'physical_stop_confirmed':False}
            elif method=='renew':
                if self.fault:raise RuntimeError(self.fault)
                stage=q['stage']
                if stage not in ('stationary','navigation','grasp'):raise ValueError('Unknown authority stage')
                if self.stage=='grasp' and stage!='grasp':raise RuntimeError('Grasp stage latched')
                self.stage=stage;self.renewed=self.clock()
            else:raise ValueError('Unknown method')
            # Permission is confirmed only after an acknowledged command in this stage.
            self.send((0.,0.,0.))
            d=self.t.await_ack(self.boot,self.owner,self.seq,self.stage,.25)
            if not d:raise RuntimeError('Driver did not acknowledge stage; no execution permission')
            return {'task_id':tid,'exclusive':True,'scope':'guarded ROS driver only','stage':self.stage}

    def send(self,velocity):
        self.seq+=1
        self.t.send(dict(boot_id=self.boot,token=self.token,task_id=self.owner,sequence=self.seq,
                        stage=self.stage,stamp=self.t.ros_time(),vx=velocity[0],vy=velocity[1],wz=velocity[2]))

    def tick(self):
        with self.lock:
            if self.owner is None or self.fault:return
            try:
                if self.clock()-self.renewed>.6:raise RuntimeError('Supervisor lease expired')
                d=self.status()
                if d.get('boot_id')!=self.boot or d.get('stage')=='recovery':raise RuntimeError('Driver restarted/revoked lease')
                arm=rpc(self.directory/'arm.sock','feedback',timeout=.2)
                if arm['errors']:raise RuntimeError('Arm feedback missing/stale')
                nav=rpc(self.directory/'nav.sock','get_status',timeout=.2,task_id=self.owner)
                v=(0.,0.,0.)
                if self.stage=='navigation':
                    if nav['result']=='RUNNING':
                        if nav.get('task_id')!=self.owner:raise RuntimeError('Navigation owner mismatch')
                        if nav.get('command_age',100)>.2:raise RuntimeError('Navigation command stale')
                        raw=nav['velocity']
                        if len(raw)!=3 or not all(math.isfinite(x) for x in raw):raise RuntimeError('Invalid velocity')
                        # Deliberate first-validation speed caps; planner stop geometry unchanged.
                        v=tuple(max(-cap,min(cap,x)) for x,cap in zip(raw,(.10,.05,.15)))
                    elif nav['result'] not in ('IDLE','ARRIVED'):raise RuntimeError('Navigation failed/cancelled')
                elif self.stage=='grasp' and (nav['result']=='RUNNING' or not nav.get('stopped',False)):raise RuntimeError('Base motion/stop feedback violation during grasp')
                self.send(v)
            except Exception as exc:
                self.fault=str(exc)
                try:self.t.abort()
                except Exception:pass  # Driver's independent steady-clock watchdog revokes.
                try:rpc(self.directory/'arm.sock','abort',timeout=.2,task_id=self.owner)
                except Exception:pass

class RosTransport:
    def __init__(self):
        import rospy
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        self.rospy=rospy;self.String=String;self.Trigger=Trigger;self.latest={}
        rospy.init_node('a2_unified_authority',disable_signals=True)
        self.pub=rospy.Publisher('/a2_unified/base_command',String,queue_size=1)
        rospy.Subscriber('/a2_unified/base_status',String,self.receive,queue_size=1)
    def receive(self,m):
        try:
            d=json.loads(m.data)
            if not 0<=self.ros_time()-float(d['stamp'])<=.25:return
            d['_received']=time.monotonic();self.latest=d
        except (ValueError,KeyError,TypeError):pass
    def status(self):return dict(self.latest)
    def ros_time(self):return self.rospy.Time.now().to_sec()
    def call(self,name):
        self.rospy.wait_for_service(name,timeout=.3)
        r=self.rospy.ServiceProxy(name,self.Trigger)()
        if not r.success:raise RuntimeError(r.message)
        return r.message
    def claim(self):return json.loads(self.call('/a2_unified/base_claim'))
    def abort(self):return self.call('/a2_unified/base_abort')
    def send(self,data):self.pub.publish(self.String(data=json.dumps(data,allow_nan=False)))
    def await_ack(self,boot,task,seq,stage,timeout):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            d=self.status()
            if (d.get('boot_id')==boot and d.get('task_id')==task and int(d.get('sequence',-1))>=seq and
                d.get('stage')==stage and time.monotonic()-d.get('_received',-100)<.25 and truth(d.get('lease_valid'))):return d
            time.sleep(.005)
        return None

if __name__=='__main__':
    directory=Path(sys.argv[1]);a=Authority(RosTransport(),directory)
    def loop():
        while True:a.tick();time.sleep(.05)
    threading.Thread(target=loop,daemon=True).start()
    serve(directory/'authority.sock',a.dispatch).serve_forever()
