"""Full MoveIt pregrasp/Cartesian approach/lift planning for the maintained node."""
import copy,json,threading,time
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState,PlanningScene,CollisionObject,AttachedCollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionIK,GetCartesianPath,GetStateValidity,GetPositionFK
from grasp_msgs.msg import SelectedGrasp
from tf2_geometry_msgs import do_transform_pose

class FullPlanning:
    def initialize_full_planning(self,group):
        for k,v in dict(plan_full_grasp=True,session_id='',planning_time_sec=2.,max_gripper_width_m=.075,require_target_collision=False,max_candidate_checks=32,lift_retreat_options_m=[0.0],search_budget_sec=90.0,pregrasp_distance_options_m=[0.0]).items():self.declare_parameter(k,v)
        self.ik_client=self.create_client(GetPositionIK,'/compute_ik',callback_group=group)
        self.cartesian_client=self.create_client(GetCartesianPath,'/compute_cartesian_path',callback_group=group)
        self.validity_client=self.create_client(GetStateValidity,'/check_state_validity',callback_group=group)
        self.fk_client=self.create_client(GetPositionFK,'/compute_fk',callback_group=group)
        self.selection=self.create_publisher(SelectedGrasp,'/planner/selected_grasp',2)
        self.diagnostics=self.create_publisher(String,'/planner/diagnostics',10)
        self.scene_client=self.create_client(ApplyPlanningScene,'/apply_planning_scene',callback_group=group)
        self.geometry_records={};self.target_attached=False
        self.create_subscription(String,'/grasp/diagnostics',self.geometry_callback,10,callback_group=group)
        self.measured_joints=None
        self.create_subscription(JointState,'/joint_states',lambda m:setattr(self,'measured_joints',m),10,callback_group=group)

    def geometry_callback(self,msg):
        try:
            data=json.loads(msg.data)
            if data.get('geometry_fit'):
                self.geometry_records[tuple(data['acquisition_stamp'])]=data
                while len(self.geometry_records)>100:self.geometry_records.pop(next(iter(self.geometry_records)))
        except (ValueError,KeyError):pass

    def target_geometry(self,msg):
        key=(msg.header.stamp.sec,msg.header.stamp.nanosec);end=time.monotonic()+1.
        while key not in self.geometry_records and time.monotonic()<end:time.sleep(.01)
        data=self.geometry_records.get(key)
        if not data:
            if self.get_parameter('require_target_collision').value:raise ValueError('Exact target collision geometry unavailable')
            return None
        if data['geometry_frame']!=self.get_parameter('base_frame').value:raise ValueError('Target collision frame mismatch')
        fit=data['geometry_fit'];obj=CollisionObject();obj.id='selected_target';obj.header.frame_id=data['geometry_frame'];obj.operation=CollisionObject.ADD
        primitive=SolidPrimitive();pose=Pose();pose.orientation.w=1.
        if fit.get('shape')=='sphere':
            primitive.type=SolidPrimitive.SPHERE;primitive.dimensions=[fit['radius_m']+.001]
            pose.position.x,pose.position.y,pose.position.z=map(float,fit['center'])
        elif fit.get('shape','cylinder')=='cylinder':
            primitive.type=SolidPrimitive.CYLINDER;primitive.dimensions=[fit['observed_height_m'],fit['radius_m']+.001]
            pose.position.x,pose.position.y=map(float,fit['axis_xy']);pose.position.z=(fit['bottom_observed_z_m']+fit['top_observed_z_m'])/2
        elif fit.get('shape')=='box':
            primitive.type=SolidPrimitive.BOX;primitive.dimensions=[float(v)+.001 for v in fit['size_m']]
            pose.position.x,pose.position.y,pose.position.z=map(float,fit['center'])
        else:raise ValueError('Unsupported target collision shape')
        obj.primitives=[primitive];obj.primitive_poses=[pose]
        self.apply_target(obj);return obj

    def apply_target(self,obj,attached=None):
        scene=PlanningScene();scene.is_diff=True;scene.robot_state.is_diff=True
        if attached is not None:
            scene.robot_state.attached_collision_objects=[attached]
        else:
            removal=AttachedCollisionObject();removal.link_name=self.get_parameter('end_effector_link').value;removal.object.id=obj.id;removal.object.operation=CollisionObject.REMOVE
            if self.target_attached:scene.robot_state.attached_collision_objects=[removal]
            scene.world.collision_objects=[obj]
        result=self.rpc(self.scene_client,ApplyPlanningScene.Request(scene=scene))
        if not result.success:raise RuntimeError('Target collision scene update rejected')
        self.target_attached=attached is not None

    def attach_target(self,obj,grasp):
        link=self.link_target(grasp);q=link.orientation;r=Rotation.from_quat([q.x,q.y,q.z,q.w]);p=link.position
        original=obj.primitive_poses[0].position;v=r.inv().apply(np.array([original.x-p.x,original.y-p.y,original.z-p.z]))
        attached=AttachedCollisionObject();attached.link_name=self.get_parameter('end_effector_link').value;attached.touch_links=['p_link7','p_link8'];attached.object=copy.deepcopy(obj);attached.object.header.frame_id=attached.link_name
        pose=attached.object.primitive_poses[0];pose.position.x,pose.position.y,pose.position.z=map(float,v);pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=map(float,r.inv().as_quat())
        self.apply_target(obj,attached)

    def rpc(self,client,req):
        if not client.wait_for_service(timeout_sec=3):raise RuntimeError('MoveIt service unavailable: '+client.srv_name)
        return self.wait(client.call_async(req),10.)

    def link_target(self,tcp_pose):
        result=copy.deepcopy(tcp_pose)
        r=Rotation.from_quat([result.orientation.x,result.orientation.y,result.orientation.z,result.orientation.w])
        tool=Rotation.from_quat(self.get_parameter('tool_quaternion').value);link=r*tool.inv()
        end=np.array([result.position.x,result.position.y,result.position.z])-link.apply(self.get_parameter('tool_translation').value)
        result.position.x,result.position.y,result.position.z=map(float,end)
        result.orientation.x,result.orientation.y,result.orientation.z,result.orientation.w=map(float,link.as_quat())
        return result

    def pregrasp_ik(self,request,record):
        # Different elbow seeds address local KDL convergence without relaxing constraints.
        seeds=[None,[0.,1.4,-.5,0.,-1.,0.],[0.,.8,-1.3,0.,.4,0.],
               [0.,2.,-1.5,0.,-1.1,0.]]
        attempts=[]
        for seed in seeds:
            trial=copy.deepcopy(request)
            if seed is not None:
                for n,value in enumerate(seed,1):
                    name=f'joint{n}'
                    if name in trial.ik_request.robot_state.joint_state.name:
                        trial.ik_request.robot_state.joint_state.position[trial.ik_request.robot_state.joint_state.name.index(name)]=value
            result=self.rpc(self.ik_client,trial);attempts.append(int(result.error_code.val))
            if result.error_code.val==1:
                record['ik_attempts']=attempts
                return result
        # Diagnostic query only: its result is NEVER returned for planning/execution.
        probe=copy.deepcopy(request);probe.ik_request.avoid_collisions=False
        diagnostic=self.rpc(self.ik_client,probe)
        record.update(ik_attempts=attempts,ik_without_collision_code=int(diagnostic.error_code.val))
        kind='collision constraints rejected pose' if diagnostic.error_code.val==1 else 'no kinematic solution found within limits/search budget'
        raise ValueError(f'Pregrasp IK failed: {kind}; codes={attempts}')

    def validate_cartesian_geometry(self,start,trajectory,goal):
        """Verify returned/resampled joint path, not just Cartesian service fraction."""
        def fk(state):
            req=GetPositionFK.Request();req.header.frame_id=self.get_parameter('base_frame').value
            req.fk_link_names=[self.get_parameter('end_effector_link').value];req.robot_state=state
            result=self.rpc(self.fk_client,req)
            if result.error_code.val!=1 or not result.pose_stamped:raise ValueError('FK path verification failed')
            return result.pose_stamped[0].pose
        if not trajectory.points:raise ValueError('Empty Cartesian trajectory')
        initial=fk(start);a=np.array([initial.position.x,initial.position.y,initial.position.z]);b=np.array([goal.position.x,goal.position.y,goal.position.z]);line=b-a
        q=goal.orientation;target=Rotation.from_quat([q.x,q.y,q.z,q.w])
        for point in trajectory.points:
            state=copy.deepcopy(start);mapping=dict(zip(state.joint_state.name,state.joint_state.position));mapping.update(zip(trajectory.joint_names,point.positions))
            state.joint_state.name=list(mapping);state.joint_state.position=list(mapping.values());pose=fk(state)
            pos=np.array([pose.position.x,pose.position.y,pose.position.z]);progress=float(np.clip((pos-a)@line/max(line@line,1e-12),0,1))
            q=pose.orientation;angle=(target.inv()*Rotation.from_quat([q.x,q.y,q.z,q.w])).magnitude()
            if angle>.03 or np.linalg.norm(pos-(a+progress*line))>.003:
                raise ValueError(f'Resampled Cartesian geometry invalid: orientation_error={angle:.4f} rad, line_error={np.linalg.norm(pos-(a+progress*line)):.4f} m')
            validity=self.rpc(self.validity_client,GetStateValidity.Request(robot_state=state,group_name=self.get_parameter('planning_group').value))
            if not validity.valid:raise ValueError('Resampled Cartesian path contains invalid collision state')
        if np.linalg.norm(pos-b)>.003:raise ValueError('Cartesian endpoint does not reach requested pose')

    def cartesian(self,start,pose):
        req=GetCartesianPath.Request();req.header.frame_id=self.get_parameter('base_frame').value
        req.start_state=start;req.group_name=self.get_parameter('planning_group').value
        req.link_name=self.get_parameter('end_effector_link').value;req.waypoints=[self.link_target(pose)]
        req.max_step=.005;req.jump_threshold=0.;req.revolute_jump_threshold=.25;req.avoid_collisions=True
        result=self.rpc(self.cartesian_client,req)
        if result.error_code.val!=1 or result.fraction<.999:
            diagnostic={}
            if hasattr(self,'validity_client'):
                probe=copy.deepcopy(req);probe.avoid_collisions=False
                unfiltered=self.rpc(self.cartesian_client,probe)
                diagnostic['without_collision_fraction']=float(unfiltered.fraction)
                # Diagnostic path is never returned or executed. Inspect it for exact colliding bodies.
                for point in unfiltered.solution.joint_trajectory.points:
                    state=copy.deepcopy(start);mapping=dict(zip(state.joint_state.name,state.joint_state.position))
                    mapping.update(zip(unfiltered.solution.joint_trajectory.joint_names,point.positions))
                    state.joint_state.name=list(mapping);state.joint_state.position=list(mapping.values())
                    validity=self.rpc(self.validity_client,GetStateValidity.Request(robot_state=state,group_name=req.group_name))
                    if not validity.valid:
                        diagnostic['first_invalid_contacts']=[dict(body1=c.contact_body_1,body2=c.contact_body_2,depth=float(c.depth)) for c in validity.contacts[:8]]
                        break
            raise ValueError(f'Cartesian path fraction={result.fraction:.3f}, code={result.error_code.val}; diagnostic={json.dumps(diagnostic)}')
        trajectory=result.solution.joint_trajectory
        # Independently enforce the absolute joint-step bound even if a service ignores it.
        if len(trajectory.points)>1:
            steps=np.diff(np.array([p.positions for p in trajectory.points]),axis=0)
            if not np.isfinite(steps).all() or np.max(np.abs(steps))>.25+1e-6:
                raise ValueError('Cartesian path exceeds absolute 0.25 rad joint-step limit')
        if hasattr(self,'fk_client'):self.validate_cartesian_geometry(start,trajectory,self.link_target(pose))
        # Conservative retiming of the collision-checked Cartesian joint path.
        elapsed=0.;previous=None
        for point in trajectory.points:
            if previous is not None:elapsed+=max(.04,float(np.max(np.abs(np.array(point.positions)-previous)))/.35)
            point.time_from_start.sec=int(elapsed);point.time_from_start.nanosec=int((elapsed-int(elapsed))*1e9)
            point.velocities=[];point.accelerations=[];previous=np.array(point.positions)
        return trajectory

    def after(self,state,trajectory):
        if not trajectory.points:raise ValueError('Empty MoveIt trajectory')
        result=copy.deepcopy(state);mapping=dict(zip(result.joint_state.name,result.joint_state.position))
        mapping.update(zip(trajectory.joint_names,trajectory.points[-1].positions))
        result.joint_state.name=list(mapping);result.joint_state.position=list(mapping.values());result.joint_state.velocity=[];result.joint_state.effort=[]
        return result

    @staticmethod
    def candidate_attempts(parents,distances):
        # Small batches preserve orientation coverage without starving shorter approaches.
        return [(i,c,d) for offset in range(0,len(parents),4) for d in distances for i,c in parents[offset:offset+4]]

    def full_preview(self,request,response):
        if not self.lock.acquire(False):response.message='Planning already active';return response
        reports=[];target_object=None;search_started=time.monotonic()
        try:
            if not self.get_parameter('tcp_calibrated').value:raise ValueError('Tool transform has not been validated')
            msg=copy.deepcopy(self.fresh_observation());joint=copy.deepcopy(self.measured_joints)
            if joint is None:raise ValueError('Joint feedback unavailable')
            age=(self.get_clock().now()-rclpy.time.Time.from_msg(joint.header.stamp)).nanoseconds/1e9
            if not 0<=age<=.75:raise ValueError('Joint feedback stale')
            start=RobotState();start.joint_state=joint;start.is_diff=True
            base=self.get_parameter('base_frame').value
            target_object=self.target_geometry(msg)
            tf=self.buffer.lookup_transform(base,msg.header.frame_id,rclpy.time.Time.from_msg(msg.header.stamp))
            parents=list(enumerate(msg.candidates[:self.get_parameter('max_candidate_checks').value]))
            endpoint_cache={};unreachable_endpoints=set()
            attempts=self.candidate_attempts(parents,self.get_parameter('pregrasp_distance_options_m').value)
            for i,original,distance in attempts:
                if i in unreachable_endpoints:continue
                if time.monotonic()-search_started>self.get_parameter('search_budget_sec').value:raise ValueError('Planning search budget exhausted; no complete grasp found')
                c=copy.deepcopy(original)
                if distance>0:c.pregrasp_distance_m=float(distance)
                record=dict(index=i,score=float(c.score),stage='grasp_endpoint',pregrasp_distance_m=float(c.pregrasp_distance_m))
                try:
                    opening=float(c.gripper_width_m)
                    if not 0<opening<=self.get_parameter('max_gripper_width_m').value:raise ValueError('Required opening exceeds gripper capacity')
                    for name,value in [('joint7',opening/2),('joint8',-opening/2)]:
                        if name in start.joint_state.name:start.joint_state.position[start.joint_state.name.index(name)]=value
                    grasp=do_transform_pose(c.pose,tf);r=Rotation.from_quat([grasp.orientation.x,grasp.orientation.y,grasp.orientation.z,grasp.orientation.w])
                    pre=copy.deepcopy(grasp);v=np.array([grasp.position.x,grasp.position.y,grasp.position.z])-r.apply([0,0,c.pregrasp_distance_m])
                    pre.position.x,pre.position.y,pre.position.z=map(float,v)
                    lift=copy.deepcopy(grasp);lift.position.z+=c.lift_distance_m
                    ik=GetPositionIK.Request();ik.ik_request.group_name=self.get_parameter('planning_group').value
                    ik.ik_request.robot_state=start;ik.ik_request.ik_link_name=self.get_parameter('end_effector_link').value
                    ik.ik_request.pose_stamped=PoseStamped();ik.ik_request.pose_stamped.header.frame_id=base;ik.ik_request.pose_stamped.pose=self.link_target(pre)
                    ik.ik_request.avoid_collisions=True;ik.ik_request.timeout.sec=1
                    # Solve the actual grasp first, then seed the pregrasp from that branch.
                    endpoint_request=copy.deepcopy(ik);endpoint_request.ik_request.pose_stamped.pose=self.link_target(grasp)
                    endpoint_request.ik_request.timeout.sec=0;endpoint_request.ik_request.timeout.nanosec=250000000
                    if i in endpoint_cache:
                        endpoint,endpoint_record=endpoint_cache[i]
                        record['endpoint_cached']=True
                    else:
                        endpoint_record={}
                        try:endpoint=self.pregrasp_ik(endpoint_request,endpoint_record)
                        except ValueError as exc:
                            record['grasp_ik']=endpoint_record;unreachable_endpoints.add(i)
                            raise ValueError('Grasp endpoint IK failed: '+str(exc))
                        endpoint_cache[i]=(endpoint,endpoint_record)
                    record['grasp_ik']=endpoint_record;record['stage']='pregrasp'
                    ik.ik_request.robot_state=endpoint.solution
                    ikresult=self.pregrasp_ik(ik,record)
                    # Use the project's MoveGroup request constraints, now with an explicit start state.
                    from moveit_msgs.action import MoveGroup
                    from moveit_msgs.msg import Constraints,JointConstraint
                    goal=MoveGroup.Goal();goal.request.group_name=self.get_parameter('planning_group').value
                    goal.request.start_state=start;goal.request.allowed_planning_time=self.get_parameter('planning_time_sec').value
                    goal.request.max_velocity_scaling_factor=.25;goal.request.max_acceleration_scaling_factor=.25
                    values=dict(zip(ikresult.solution.joint_state.name,ikresult.solution.joint_state.position))
                    constraints=Constraints()
                    for name in [f'joint{n}' for n in range(1,7)]:
                        constraints.joint_constraints.append(JointConstraint(joint_name=name,position=values[name],tolerance_above=.002,tolerance_below=.002,weight=1.))
                    goal.request.goal_constraints=[constraints];goal.planning_options.plan_only=True
                    if not self.action.wait_for_server(timeout_sec=3):raise RuntimeError('MoveGroup unavailable')
                    handle=self.wait(self.action.send_goal_async(goal),5.)
                    if not handle.accepted:raise ValueError('MoveGroup rejected request')
                    try:result=self.wait(handle.get_result_async(),12.)
                    except TimeoutError:
                        handle.cancel_goal_async();raise
                    if result.result.error_code.val!=1:raise ValueError(f'Pregrasp plan code={result.result.error_code.val}')
                    pre_traj=result.result.planned_trajectory.joint_trajectory
                    record['stage']='approach'
                    grasp_traj=self.cartesian(self.after(start,pre_traj),grasp)
                    if target_object is not None:self.attach_target(target_object,grasp)
                    try:
                        record['stage']='lift';record['lift_attempts']=[]
                        for retreat in self.get_parameter('lift_retreat_options_m').value:
                            trial_lift=copy.deepcopy(lift)
                            horizontal=r.as_matrix()[:,2].copy();horizontal[2]=0.;horizontal/=max(np.linalg.norm(horizontal),1e-9)
                            trial_lift.position.x-=float(retreat*horizontal[0]);trial_lift.position.y-=float(retreat*horizontal[1])
                            try:
                                lift_traj=self.cartesian(self.after(start,grasp_traj),trial_lift)
                                lift=trial_lift;record['lift_retreat_m']=float(retreat);break
                            except ValueError as exc:
                                record['lift_attempts'].append(dict(retreat_m=float(retreat),reason=str(exc)))
                        else:raise ValueError('No complete collision-free lift: '+json.dumps(record['lift_attempts']))
                    finally:
                        if target_object is not None:self.apply_target(target_object)
                    selected=SelectedGrasp();selected.header=msg.header;selected.header.frame_id=base
                    selected.session_id=self.get_parameter('session_id').value;selected.candidate_index=i
                    selected.candidate=copy.deepcopy(c);selected.candidate.pose=grasp
                    selected.candidate.gripper_width_m=min(c.gripper_width_m,self.get_parameter('max_gripper_width_m').value)
                    if target_object is not None:
                        selected.observed_object_center=copy.deepcopy(target_object.primitive_poses[0].position)
                        primitive=target_object.primitives[0]
                        selected.observed_object_radius_m=float(primitive.dimensions[0 if primitive.type==SolidPrimitive.SPHERE else 1]-.001) if primitive.type!=SolidPrimitive.BOX else float((primitive.dimensions[1]-.001)/2)
                        selected.observed_object_height_m=float(primitive.dimensions[0] if primitive.type==SolidPrimitive.CYLINDER else 2*(primitive.dimensions[0]-.001)) if primitive.type!=SolidPrimitive.BOX else float(primitive.dimensions[2]-.001)
                    selected.pregrasp_pose=pre;selected.lift_pose=lift
                    selected.pregrasp_trajectory=pre_traj;selected.approach_trajectory=grasp_traj;selected.lift_trajectory=lift_traj
                    self.selection.publish(selected)
                    legacy=PoseStamped();legacy.header=selected.header;legacy.pose=self.link_target(pre);self.selected_pub.publish(legacy)
                    record.update(status='selected',pre_points=len(pre_traj.points),approach_points=len(grasp_traj.points),lift_points=len(lift_traj.points));reports.append(record)
                    response.success=True;response.message='Full pregrasp/approach/lift planned; no execution';return response
                except Exception as exc:
                    record.update(status='rejected',reason=str(exc));reports.append(record)
                    if not isinstance(exc,ValueError):raise
            raise ValueError('No candidate passed complete IK/MoveIt planning')
        except Exception as exc:response.message=str(exc)
        finally:
            self.diagnostics.publish(String(data=json.dumps(dict(candidates=reports,result=response.message))))
            self.lock.release()
        return response
