"""Explicit simulation transport for the existing pick-and-hold state machine.

Physical /task/start remains execute=false; this path only calls /sim actions.
"""
import copy,threading,time,json
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Float64,String
from std_srvs.srv import Trigger
from grasp_msgs.msg import SelectedGrasp,GraspCandidateArray
import rclpy

class SimulationExecution:
    def initialize_simulation(self):
        self.declare_parameter('simulation_mode',False);self.declare_parameter('session_id','')
        self.declare_parameter('place_after_grasp',False)
        self.declare_parameter('require_selected_grasp',True)
        self.declare_parameter('tool_translation',[0.,0.,.105])
        self.declare_parameter('tool_quaternion',[0.,0.,0.,1.])
        self.selected_grasp=None;self.live_candidates=None;self.live_geometry=None;self.active_selection=None;self.active_open_width=None
        self.create_subscription(SelectedGrasp,'/planner/selected_grasp',self.receive_selection,2)
        if not self.get_parameter('simulation_mode').value:return
        self.sim_action=ActionClient(self,FollowJointTrajectory,'/sim/arm_controller/follow_joint_trajectory')
        self.sim_gripper=self.create_publisher(Float64,'/sim/gripper/command',1)
        self.sim_ready=self.create_client(Trigger,'/sim/ready')
        self.sim_near_support=self.create_client(Trigger,'/sim/target_near_support')
        self.sim_placed=self.create_client(Trigger,'/sim/target_placed')
        self.create_subscription(String,'/grasp/diagnostics',self.receive_geometry,10)
        self.create_subscription(GraspCandidateArray,'/grasp/candidates',lambda m:setattr(self,'live_candidates',m),2)
        self.create_service(Trigger,'/sim/task/start',self.start_simulation)

    def receive_geometry(self,msg):
        try:
            data=json.loads(msg.data)
            if data.get('geometry_fit'):self.live_geometry=data
        except ValueError:pass

    def validate_target_geometry(self,chosen):
        data=self.live_geometry
        if not data or data['geometry_frame']!=chosen.header.frame_id:raise ValueError('Fresh target geometry/frame unavailable')
        stamp=data['acquisition_stamp'];age=self.get_clock().now().nanoseconds/1e9-(stamp[0]+stamp[1]/1e9)
        if not 0<=age<=.5:raise ValueError('Target geometry observation expired')
        f=data['geometry_fit'];p=chosen.observed_object_center
        shape=f.get('shape','cylinder');sphere=shape=='sphere'
        centre=f['center'] if shape in ('sphere','box') else [*f['axis_xy'],(f['bottom_observed_z_m']+f['top_observed_z_m'])/2]
        height=2*f['radius_m'] if sphere else f['observed_height_m']
        if np.linalg.norm(np.array(centre)-[p.x,p.y,p.z])>.005 or abs(f['radius_m']-chosen.observed_object_radius_m)>.002 or abs(height-chosen.observed_object_height_m)>.015:
            raise ValueError('Observed object moved or changed shape since planning')

    def receive_selection(self,msg):
        if not self.running and not self.completion_latched:self.selected_grasp=copy.deepcopy(msg)

    def physical_link_pose(self,tcp_pose):
        result=copy.deepcopy(tcp_pose);q=result.orientation
        rotation=Rotation.from_quat([q.x,q.y,q.z,q.w])*Rotation.from_quat(self.get_parameter('tool_quaternion').value).inv()
        offset=rotation.apply(self.get_parameter('tool_translation').value)
        result.position.x-=offset[0];result.position.y-=offset[1];result.position.z-=offset[2]
        result.orientation.x,result.orientation.y,result.orientation.z,result.orientation.w=map(float,rotation.as_quat())
        return result

    def validate_physical_selection(self):
        if not self.get_parameter('require_selected_grasp').value:return
        selected=self.selected_grasp
        if selected is None:raise ValueError('Complete SelectedGrasp is required; legacy pregrasp-only execution disabled')
        if selected.candidate.generator.endswith('_preview'):raise ValueError('Candidate TCP geometry is preview-only; hardware execution refused')
        age=(self.get_clock().now()-rclpy.time.Time.from_msg(selected.header.stamp)).nanoseconds/1e9
        if not 0<=age<=1.:raise ValueError('Selected acquisition expired; hardware re-observation required')
        self.current_arm_joints();self.require_fresh_pose()
        width=selected.candidate.gripper_width_m
        if not .005<=width<=.075:raise ValueError('Invalid selected gripper opening')
        from geometry_msgs.msg import PoseStamped
        self.active_selection=copy.deepcopy(selected);self.active_open_width=float(width)
        self.pregrasp=PoseStamped();self.pregrasp.header=selected.header;self.pregrasp.pose=self.physical_link_pose(selected.pregrasp_pose)

    def start_simulation(self,request,response):
        try:
            if not self.get_parameter('simulation_mode').value:raise ValueError('Simulation transport disabled')
            if self.running or self.completion_latched:raise ValueError('One-shot task latched; manual simulation reset required')
            chosen=copy.deepcopy(self.selected_grasp)
            if chosen is None or chosen.session_id!=self.get_parameter('session_id').value:raise ValueError('Selected grasp/session mismatch')
            # The scene observation can outlive planning only after new matching evidence.
            recent=self.live_candidates
            if recent is None:raise ValueError('Fresh revalidation candidates unavailable')
            age=(self.get_clock().now()-rclpy.time.Time.from_msg(recent.header.stamp)).nanoseconds/1e9
            if not 0<=age<=.5:raise ValueError('Revalidation observation expired')
            # GraspNet can emit an empty candidate batch for one RGB-D frame
            # even while the selected bottle remains visible. The independent
            # geometry check below is the execution-time target evidence.
            self.validate_target_geometry(chosen)
            self.current_arm_joints();self.require_fresh_pose()
            first=chosen.pregrasp_trajectory.points[0]
            mapping=dict(zip(chosen.pregrasp_trajectory.joint_names,first.positions))
            if max(abs(float(self.current_arm_joints()[i])-mapping[f'joint{i+1}']) for i in range(6))>.03:raise ValueError('Robot start state changed since planning')
            if not self.sim_ready.service_is_ready():raise ValueError('Simulation controller absent')
            self.running=True;self.completion_latched=True;self.abort_flag=False
            threading.Thread(target=self.run_simulation,args=(chosen,),daemon=True).start()
            response.success=True;response.message='One-shot simulated pick-and-hold started'
        except Exception as exc:response.message=str(exc)
        return response

    def simulation_path(self,trajectory):
        self.check_abort()
        if not self.sim_action.wait_for_server(timeout_sec=3):raise RuntimeError('Simulation FollowJointTrajectory unavailable')
        goal=FollowJointTrajectory.Goal();goal.trajectory=trajectory
        handle=self.wait_future(self.sim_action.send_goal_async(goal),5)
        if not handle.accepted:raise RuntimeError('Simulation rejected planned trajectory')
        try:
            result=self.wait_future(handle.get_result_async(),120)
        except Exception:
            handle.cancel_goal_async();raise
        if result.result.error_code!=0:raise RuntimeError(result.result.error_string or 'Trajectory tracking failed')

    def reverse_simulation_path(self,trajectory):
        """Return along a previously validated Cartesian/joint trajectory."""
        result=copy.deepcopy(trajectory)
        times=[p.time_from_start.sec+p.time_from_start.nanosec/1e9 for p in trajectory.points]
        result.points=list(reversed(result.points))
        for point,seconds in zip(result.points,times):
            point.time_from_start.sec=int(seconds)
            point.time_from_start.nanosec=int(round((seconds-int(seconds))*1e9))
            point.velocities=[];point.accelerations=[];point.effort=[]
        return result

    def place_back_on_table(self,chosen):
        self.state='PLACE_DESCEND'
        self.simulation_path(self.reverse_simulation_path(chosen.lift_trajectory))
        near=self.wait_future(self.sim_near_support.call_async(Trigger.Request()),3.)
        if not near.success:raise RuntimeError('Placement target is not above the original table position; gripper remains closed')
        self.state='PLACE_RELEASE';self.sim_gripper.publish(Float64(data=.07))
        for _ in range(30):self.check_abort();time.sleep(.05)
        placed=self.wait_future(self.sim_placed.call_async(Trigger.Request()),3.)
        if not placed.success:raise RuntimeError('Object is not supported after release; return-home cancelled')
        self.state='PLACE_RETREAT'
        self.simulation_path(self.reverse_simulation_path(chosen.approach_trajectory))
        placed=self.wait_future(self.sim_placed.call_async(Trigger.Request()),3.)
        if not placed.success:raise RuntimeError('Object moved during gripper retreat; return-home cancelled')
        self.state='RETURN_HOME'
        self.simulation_path(self.reverse_simulation_path(chosen.pregrasp_trajectory))
        placed=self.wait_future(self.sim_placed.call_async(Trigger.Request()),3.)
        if not placed.success:raise RuntimeError('Object did not remain supported on table after release')
        self.state='PLACED'

    def run_simulation(self,chosen):
        try:
            ready=self.wait_future(self.sim_ready.call_async(Trigger.Request()),3)
            if not ready.success or ready.message!=chosen.session_id:raise RuntimeError('Simulation session was replaced')
            self.state='APPROACH';self.sim_gripper.publish(Float64(data=chosen.candidate.gripper_width_m))
            self.simulation_path(chosen.pregrasp_trajectory)
            self.state='DESCEND';self.simulation_path(chosen.approach_trajectory)
            self.state='CLOSE';self.sim_gripper.publish(Float64(data=0.))
            # MuJoCo contact force may read zero while the object is still in the jaws.
            # Let the lift finish; the integration checks sustained object motion.
            for _ in range(20):self.check_abort();time.sleep(.05)
            self.state='LIFT';self.simulation_path(chosen.lift_trajectory)
            self.state='HOLDING'
            if self.get_parameter('place_after_grasp').value:
                self.place_back_on_table(chosen)
        except Exception as exc:self.state='RECOVERY_REQUIRED: '+str(exc)
        finally:self.running=False;self.selected_grasp=None
