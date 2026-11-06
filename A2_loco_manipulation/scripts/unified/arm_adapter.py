"""ROS2 original single_view adapter. Runs beside, never duplicates, the pipeline."""
import json, math, sys, threading, time
from pathlib import Path
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_srvs.srv import Trigger
from piper_msgs.srv import Enable
from sensor_msgs.msg import Image, JointState, PointCloud2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from grasp_msgs.msg import GraspCandidateArray, BoundTargetCloud
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener
from common import serve

class Arm:
    def __init__(self, config):
        self.config=config;self.n=rclpy.create_node('a2_unified_arm'); self.group=ReentrantCallbackGroup()
        self.seen={};self.state='';self.state_at=0.; self.cutoff=0.;self.preview_future=None
        self.preview_at=0.;self.started_at=0.;self.started=False
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self.n)
        for topic,typ in [('/joint_states_feedback',JointState),('/end_pose_stamped',PoseStamped),
                          ('/camera/color/image_raw',Image),('/camera/depth/image_raw',Image),
                          ('/grasp/candidates',GraspCandidateArray),('/planner/selected_pregrasp',PoseStamped)]:
            self.n.create_subscription(typ,topic,lambda m,t=topic:self.seen.update({t:(time.monotonic(),m)}),qos_profile_sensor_data if typ==Image else 2,callback_group=self.group)
        self.n.create_subscription(String,'/task/status',self.status_cb,10,callback_group=self.group)
        self.clients={};self.identity=None;self.bound_stamp=0.;self.bound_at=0.
        self.cloud=self.n.create_publisher(PointCloud2,'/perception/target_cloud',1)
        self.n.create_subscription(BoundTargetCloud,'/grasp_perception/target',self.bound_cb,2,callback_group=self.group)
        self.mutation=threading.Lock()

    def bound_cb(self,m):
        if self.identity is None or not m.valid:return
        if any(getattr(m,key)!=value for key,value in self.identity.items()):return
        stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        cloud_stamp=m.cloud.header.stamp.sec+m.cloud.header.stamp.nanosec/1e9
        now=self.n.get_clock().now().nanoseconds/1e9
        if not self.cutoff<stamp or not 0<=now-stamp<=.5 or stamp!=cloud_stamp or m.cloud.header.frame_id!=m.header.frame_id:return
        self.bound_stamp=stamp;self.bound_at=time.monotonic()
        self.cloud.publish(m.cloud)

    def status_cb(self,m):self.state=m.data;self.state_at=time.monotonic()
    def client(self,typ,name):
        if name not in self.clients:self.clients[name]=self.n.create_client(typ,name,callback_group=self.group)
        return self.clients[name]
    def call(self,typ,name,req,timeout=4):
        c=self.client(typ,name)
        if not c.wait_for_service(timeout_sec=1):raise RuntimeError(name+' unavailable')
        f=c.call_async(req);end=time.monotonic()+timeout
        while not f.done():
            if time.monotonic()>end:raise TimeoutError(name+' response timeout; outcome unknown')
            time.sleep(.01)
        return f.result()
    def params(self,node,names):return self.call(GetParameters,node+'/get_parameters',GetParameters.Request(names=names)).values
    def errors(self,topics):
        errors=[];now=time.monotonic();rosnow=self.n.get_clock().now().nanoseconds/1e9
        for topic in topics:
            item=self.seen.get(topic)
            if item is None or now-item[0]>.5:errors.append(topic+' missing/stale');continue
            m=item[1];stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
            if not 0<=rosnow-stamp<=.5:errors.append(topic+' acquisition stamp stale/future')
        return errors
    def feedback(self):
        errors=self.errors(['/joint_states_feedback','/end_pose_stamped'])
        item=self.seen.get('/joint_states_feedback')
        if item:
            m=item[1];q=dict(zip(m.name,m.position))
            if any(name not in q or not math.isfinite(q[name]) for name in ['joint'+str(i) for i in range(1,7)]):errors.append('Invalid arm joint feedback')
        return {'errors':errors}
    def health(self):
        errors=self.feedback()['errors']+self.errors(['/camera/color/image_raw','/camera/depth/image_raw'])
        names=[name for name,_ in self.n.get_node_names_and_namespaces()]
        for name in ('perception_node','grasp_generator','robot_planner','task_executor','piper_ctrl_single_node'):
            # Actual generator node name is checked through service/topic graph below.
            if name=='grasp_generator':continue
            if names.count(name)!=1:errors.append('Expected one '+name+', got '+str(names.count(name)))
        for topic in ('/joint_states_feedback','/camera/color/image_raw','/camera/depth/image_raw','/grasp/candidates'):
            endpoints=self.n.get_publishers_info_by_topic(topic)
            physical=[e for e in endpoints if e.node_name!='ros_bridge']
            bridges=[e for e in endpoints if e.node_name=='ros_bridge']
            if len(physical)!=1 or len(bridges)>1:errors.append('Conflicting/missing publisher: '+topic)
        for name,typ in [('/enable_srv',Enable),('/planner/preview',Trigger),('/task/start',Trigger),('/task/abort',Trigger)]:
            if not self.client(typ,name).service_is_ready():errors.append(name+' not ready')
        tcp=None
        try:
            tcp=list(self.params('/robot_planner',['tool_translation'])[0].double_array_value)
            if tcp!=[0.,0.,.105]:errors.append('Effective TCP differs from [0,0,0.105]')
            values=self.params('/piper_ctrl_single_node',['auto_enable','cartesian_speed_percent','cartesian_move_mode'])
            if values[0].bool_value or values[1].integer_value!=10 or values[2].integer_value!=0:errors.append('Driver configuration mismatch')
            if not self.started and self.params('/task_executor',['execute'])[0].bool_value:errors.append('execute must be false before task')
        except Exception as exc:errors.append(str(exc))
        try:self.buffer.lookup_transform('base_link','camera_color_optical_frame',rclpy.time.Time())
        except Exception as exc:errors.append('TF unavailable: '+str(exc))
        if self.state not in ('WAITING_FOR_PREGRASP','PREGRASP_READY'):errors.append('Executor not idle: '+self.state)
        return {'errors':errors,'effective_tcp':tcp,'executor':self.state}
    def require_execution_candidate(self):
        item=self.seen.get('/grasp/candidates')
        if getattr(self, 'preview_graspnet', False) or (item and any(c.generator == 'graspnet_baseline_preview' for c in item[1].candidates)):
            raise RuntimeError('GraspNet preview only: PiPER TCP/width mapping not physically validated')

    def dispatch(self,q):
        method=q['method']
        if method=='health':return self.health()
        if method=='feedback':return self.feedback()
        if method=='pose':
            errors=self.feedback()['errors'];item=self.seen.get('/joint_states_feedback')
            if errors:return {'valid':False,'reason':'; '.join(errors)}
            positions=dict(zip(item[1].name,item[1].position));target=q['target']
            valid=len(target)==6 and all(math.isfinite(t) and abs(math.degrees(positions['joint'+str(i+1)])-t)<=self.config['joint_tolerance_deg'] for i,t in enumerate(target))
            return {'valid':valid,'joint_deg':[math.degrees(positions['joint'+str(i)]) for i in range(1,7)],'target_joint_deg':target,'reason':'' if valid else 'Arm is not in the recorded observation posture; prepare with the existing observe-pose tool while unified driver is stopped'}
        if method=='fresh_reset':
            self.cutoff=self.n.get_clock().now().nanoseconds/1e9;self.preview_future=None
            self.identity=q['identity']; self.bound_stamp=0.; self.bound_at=0.
            r=self.call(Trigger,'/grasp_perception/start',Trigger.Request())
            if not r.success:raise RuntimeError('Bound perception refused: '+r.message)
            return {'cutoff':self.cutoff}
        if method=='fresh':
            errors=self.errors(['/camera/color/image_raw','/camera/depth/image_raw','/grasp/candidates'])
            for topic in ('/camera/color/image_raw','/camera/depth/image_raw','/grasp/candidates'):
                item=self.seen.get(topic)
                if item:
                    m=item[1];stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
                    if stamp<=self.cutoff:errors.append('Observation predates verified stop')
                    if topic=='/grasp/candidates' and not m.candidates:errors.append('No candidates')
            if time.monotonic()-self.bound_at>.5:errors.append('Identity-bound cloud missing/stale')
            return {'valid':not errors,'errors':errors}
        if method=='preview':
            if not self.dispatch({'method':'fresh'})['valid']:raise RuntimeError('Fresh RGB-D/candidates required')
            if self.preview_future is not None:raise RuntimeError('Preview already requested')
            self.preview_graspnet=any(c.generator == 'graspnet_baseline_preview' for c in self.seen['/grasp/candidates'][1].candidates)
            self.preview_at=time.monotonic()
            self.preview_future=self.client(Trigger,'/planner/preview').call_async(Trigger.Request())
            return {'accepted':True}
        if method=='preview_result':
            f=self.preview_future
            if f is None:raise RuntimeError('Preview not requested')
            if not f.done():return {'done':False}
            r=f.result();item=self.seen.get('/planner/selected_pregrasp')
            valid=bool(item and item[0]>=self.preview_at and item[1].header.stamp.sec+item[1].header.stamp.nanosec/1e9>self.cutoff)
            return {'done':True,'success':bool(r.success and valid),'reason':r.message if valid else 'New selected preview not received'}
        if method=='enable':
            self.require_execution_candidate()
            if not self.dispatch({'method':'preview_result'}).get('success'):raise RuntimeError('No new valid preview')
            if self.feedback()['errors']:raise RuntimeError('Arm feedback lost')
            r=self.call(Enable,'/enable_srv',Enable.Request(enable_request=True))
            if not r.enable_response:raise RuntimeError('Driver enable refused')
            return {'enabled':True}
        if method=='start':
            self.require_execution_candidate()
            with self.mutation:
                if self.started:raise RuntimeError('One shot already attempted')
                p=self.dispatch({'method':'preview_result'})
                if not p.get('success'):raise RuntimeError('Preview invalid')
                item=self.seen['/planner/selected_pregrasp'];m=item[1]
                age=self.n.get_clock().now().nanoseconds/1e9-m.header.stamp.sec-m.header.stamp.nanosec/1e9
                if not 0<=age<=1:raise RuntimeError('Selected preview observation expired; no automatic retry')
                if self.state not in ('WAITING_FOR_PREGRASP','PREGRASP_READY'):raise RuntimeError('Executor not idle')
                if time.monotonic()-self.bound_at>.5:raise RuntimeError('Identity-bound observation lost')
                self.started=True;self.started_at=time.monotonic()
                r=self.call(SetParameters,'/task_executor/set_parameters',SetParameters.Request(parameters=[Parameter(name='execute',value=ParameterValue(type=ParameterType.PARAMETER_BOOL,bool_value=True))]))
                if not r.results or not all(x.successful for x in r.results):raise RuntimeError('execute parameter rejected')
                r=self.call(Trigger,'/task/start',Trigger.Request())
                if not r.success:raise RuntimeError('Task refused: '+r.message)
                return {'accepted':True}
        if method=='get_status':
            if not self.started:return {'result':'FAILED','reason':'Task not started'}
            if time.monotonic()-self.state_at>2.5:raise RuntimeError('Task status communication lost')
            state=self.state if self.state_at>=self.started_at else ''
            if state=='HOLDING':
                m=self.seen['/joint_states_feedback'][1]
                threshold=self.params('/task_executor',['close_effort_threshold'])[0].double_value
                effort=m.effort[6] if len(m.effort)>6 else None
                contact=bool(effort is not None and math.isfinite(effort) and threshold>=.30 and abs(effort)>=threshold and not self.feedback()['errors'])
                return {'result':'HOLDING','reason':'执行器报告持物保持','contact_evidence':contact,'effort':effort,'threshold':threshold,'object_grasp_confirmed':False}
            if state.startswith('RECOVERY_REQUIRED'):return {'result':'RECOVERY_REQUIRED','reason':state}
            return {'result':'RUNNING','executor':state}
        if method=='abort':
            if not self.started:return {'requested':False,'confirmed':False,'reason':'No owned grasp attempt'}
            r=self.call(Trigger,'/task/abort',Trigger.Request())
            end=time.monotonic()+2.5
            while time.monotonic()<end:
                if self.state.startswith('RECOVERY_REQUIRED'):break
                time.sleep(.05)
            return {'requested':r.success,'confirmed':self.state.startswith('RECOVERY_REQUIRED'),'physical_stop_confirmed':False,'reason':self.state}
        raise ValueError('Unknown method')

if __name__=='__main__':
    rclpy.init();a=Arm(json.loads(Path(sys.argv[1]).read_text()))
    e=MultiThreadedExecutor(num_threads=4);e.add_node(a.n)
    threading.Thread(target=e.spin,daemon=True).start()
    serve(sys.argv[2],a.dispatch).serve_forever()
