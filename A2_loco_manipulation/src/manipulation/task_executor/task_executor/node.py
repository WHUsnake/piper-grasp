"""Single low-speed pick-and-hold executor for the Piper PosCmd interface.

Default is preview-only. Physical execution requires execute:=true and a
driver which is already deliberately enabled by the operator.
"""
import math
import threading
import time
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from piper_msgs.msg import PosCmd
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import Constraints, JointConstraint
from scipy.spatial.transform import Rotation

JOINT_NAMES = tuple(f'joint{i}' for i in range(1, 7))

from .simulation_execution import SimulationExecution

class TaskExecutor(SimulationExecution,Node):
    def __init__(self):
        super().__init__('task_executor')
        self.declare_parameter('execute',False)
        self.declare_parameter('motion_timeout_sec',30.)
        self.declare_parameter('control_hz',10.)
        self.declare_parameter('position_tolerance_m',.008)
        self.declare_parameter('angle_tolerance_deg',4.)
        self.declare_parameter('gripper_open_m',.075)
        self.declare_parameter('gripper_close_m',.050)
        self.declare_parameter('close_effort_threshold',.30)
        self.declare_parameter('place_base_angle_deg',90.)
        self.declare_parameter('pregrasp_distance_m',.10)
        self.declare_parameter('lift_distance_m',.12)
        self.declare_parameter('lift_delta_base_m',[0.,0.,.12])
        self.declare_parameter('home_joint_positions_rad',[0.]*6)
        self.declare_parameter('home_joint_speed_percent',5.)
        self.declare_parameter('home_joint_tolerance_rad',.012)
        self.declare_parameter('home_joint_deviation_limit_rad',.12)
        self.pregrasp=None; self.pose=None; self.joints=None; self.last_joints_monotonic=None; self.abort_flag=False; self.running=False
        self.completion_latched=False
        self.last_pose_monotonic=None
        self.last_command_width=None
        self.state='WAITING_FOR_PREGRASP'
        self.pub=self.create_publisher(String,'/task/status',2)
        self.command=self.create_publisher(PosCmd,'/pos_cmd',1)
        self.joint_command=self.create_publisher(JointState,'/joint_ctrl_single',1)
        self.home_planner=ActionClient(self,MoveGroup,'/move_action')
        self.state_validity=self.create_client(GetStateValidity,'/check_state_validity')
        self.create_subscription(PoseStamped,'/planner/selected_pregrasp',self.pregrasp_cb,2)
        self.create_subscription(PoseStamped,'/end_pose_stamped',self.pose_cb,2)
        self.create_subscription(JointState,'/joint_states_feedback',self.joint_cb,2)
        self.create_service(Trigger,'/task/abort',self.abort)
        self.create_service(Trigger,'/task/start',self.start)
        self.create_timer(.1,self.tick)
        self.initialize_simulation()
    def pregrasp_cb(self,msg):
        if self.get_parameter('require_selected_grasp').value:return
        if self.running or self.completion_latched: return
        self.pregrasp=msg
        if self.state=='WAITING_FOR_PREGRASP': self.state='PREGRASP_READY'
    def start(self,request,response):
        if self.completion_latched:
            response.success=False; response.message='manual recovery required after pick-and-hold attempt'; return response
        if not bool(self.get_parameter('execute').value):
            response.success=False; response.message='execute=false: preview only; set execute:=true and restart node'; return response
        try:self.validate_physical_selection()
        except Exception as exc:
            response.success=False;response.message=str(exc);return response
        if self.pregrasp is None:
            response.success=False; response.message='no selected pregrasp'; return response
        if self.running:
            response.success=False; response.message='task already running'; return response
        self.abort_flag=False; self.running=True; self.state='STARTING'
        threading.Thread(target=self.run,daemon=True).start()
        response.success=True; response.message='one-shot pick-and-hold started'; return response
    def pose_cb(self,msg): self.pose=msg; self.last_pose_monotonic=time.monotonic()
    def joint_cb(self,msg): self.joints=msg; self.last_joints_monotonic=time.monotonic()
    def abort(self,request,response):
        self.abort_flag=True
        if self.completion_latched: self.state='RECOVERY_REQUIRED: operator abort requested'
        response.success=True; response.message='abort requested; no automatic release or return home'; return response
    def tick(self): self.pub.publish(String(data=self.state))
    def wait_pose(self,target,rotation,opened=False):
        start=time.monotonic(); timeout=float(self.get_parameter('motion_timeout_sec').value)
        while time.monotonic()-start<timeout:
            if self.abort_flag: raise RuntimeError('aborted')
            self.require_fresh_pose()
            if self.pose is not None:
                # Piper accepts repeated Cartesian commands; keep sending at a
                # low rate while waiting for fresh feedback.
                self.send(target,rotation,float((self.active_open_width if self.active_open_width is not None else self.get_parameter('gripper_open_m').value) if opened else self.get_parameter('gripper_close_m').value))
                p=self.pose.pose.position; q=self.pose.pose.orientation
                xyz=np.array([p.x,p.y,p.z]); r=Rotation.from_quat([q.x,q.y,q.z,q.w])
                ok=np.linalg.norm(xyz-target)<=float(self.get_parameter('position_tolerance_m').value)
                ok=ok and (r.inv()*rotation).magnitude()<=math.radians(float(self.get_parameter('angle_tolerance_deg').value))
                if ok: return
            time.sleep(1.0/max(1.0,float(self.get_parameter('control_hz').value)))
        raise RuntimeError('motion feedback timeout')
    def require_fresh_pose(self):
        if self.pose is None or self.last_pose_monotonic is None or time.monotonic()-self.last_pose_monotonic > .75:
            raise RuntimeError('Piper pose feedback missing or stale (>0.75 s)')
    def require_contact(self):
        self.current_arm_joints()
        if len(self.joints.effort)<7:
            raise RuntimeError('gripper contact feedback missing')
        effort=float(self.joints.effort[6])
        threshold=float(self.get_parameter('close_effort_threshold').value)
        if not math.isfinite(effort) or not math.isfinite(threshold) or threshold<=0 or abs(effort)<threshold:
            raise RuntimeError('gripper contact feedback not confirmed')
    def send(self,xyz,rot,width):
        self.last_command_width=float(width)
        m=PosCmd(); m.x,m.y,m.z=map(float,xyz); m.roll,m.pitch,m.yaw=map(float,rot.as_euler('xyz')); m.gripper=float(width); m.mode1=0; m.mode2=0; self.command.publish(m)
    def hold(self):
        # Best-effort current-pose command, not proof of a physical safety stop.
        # Never fall back to an open gripper when feedback is unavailable.
        self.require_fresh_pose()
        if self.last_command_width is None:
            raise RuntimeError('no prior gripper command to preserve')
        p=self.pose.pose.position; q=self.pose.pose.orientation
        xyz=np.array([p.x,p.y,p.z]); quat=np.array([q.x,q.y,q.z,q.w])
        if not np.isfinite(xyz).all() or not np.isfinite(quat).all():
            raise RuntimeError('invalid hold pose')
        self.send(xyz,Rotation.from_quat(quat),self.last_command_width)
    def current_gripper_width(self):
        if self.joints is not None:
            mapping=dict(zip(self.joints.name,self.joints.position))
            width=float(mapping.get('gripper',float('nan')))
            if math.isfinite(width) and 0. <= width <= .08: return width
        return float(self.get_parameter('gripper_open_m').value)
    def current_arm_joints(self):
        if self.joints is None or self.last_joints_monotonic is None or time.monotonic()-self.last_joints_monotonic > .75:
            raise RuntimeError('Piper joint feedback missing or stale (>0.75 s)')
        mapping=dict(zip(self.joints.name,self.joints.position))
        try: q=np.asarray([mapping[name] for name in JOINT_NAMES],dtype=float)
        except KeyError as exc: raise RuntimeError('Missing joint feedback: '+str(exc)) from exc
        if not np.isfinite(q).all(): raise RuntimeError('Invalid joint feedback')
        return q
    def wait_future(self,future,timeout):
        deadline=time.monotonic()+timeout
        while not future.done():
            if self.abort_flag: raise RuntimeError('operator abort requested')
            if time.monotonic()>=deadline: raise RuntimeError('MoveIt home planning timeout')
            time.sleep(.02)
        return future.result()
    def plan_home(self,start,target):
        if not self.home_planner.wait_for_server(timeout_sec=3.):
            raise RuntimeError('MoveIt /move_action unavailable for return home')
        goal=MoveGroup.Goal()
        goal.request.group_name='arm'
        goal.request.allowed_planning_time=5.
        goal.request.max_velocity_scaling_factor=.05
        goal.request.max_acceleration_scaling_factor=.05
        goal.request.start_state.joint_state.name=list(JOINT_NAMES)
        goal.request.start_state.joint_state.position=list(map(float,start))
        goal.request.start_state.is_diff=True
        goal.request.goal_constraints=[Constraints(joint_constraints=[
            JointConstraint(joint_name=name,position=float(value),
                            tolerance_above=.01,tolerance_below=.01,weight=1.)
            for name,value in zip(JOINT_NAMES,target)])]
        goal.planning_options.plan_only=True
        handle=self.wait_future(self.home_planner.send_goal_async(goal),5.)
        if handle is None or not handle.accepted: raise RuntimeError('MoveIt rejected return-home plan')
        result=self.wait_future(handle.get_result_async(),12.)
        if result is None or result.status!=4 or result.result.error_code.val!=1:
            raise RuntimeError('MoveIt could not plan return home')
        trajectory=result.result.planned_trajectory.joint_trajectory
        if set(trajectory.joint_names)!=set(JOINT_NAMES) or not trajectory.points:
            raise RuntimeError('MoveIt returned an invalid return-home path')
        order=[trajectory.joint_names.index(name) for name in JOINT_NAMES]
        path=[np.asarray([point.positions[index] for index in order],dtype=float)
              for point in trajectory.points]
        if any(q.shape!=(6,) or not np.isfinite(q).all() for q in path):
            raise RuntimeError('MoveIt returned a nonfinite return-home path')
        if np.max(np.abs(path[0]-start))>.04 or np.max(np.abs(path[-1]-target))>.03:
            raise RuntimeError('MoveIt return-home path does not match current/zero joints')
        if any(q[0] < -.012 for q in path):
            raise RuntimeError('Return-home path enters right side (joint1 < 0)')
        return path
    def send_home_joint_target(self,q,width):
        speed=float(self.get_parameter('home_joint_speed_percent').value)
        if not 1.<=speed<=10.: raise RuntimeError('home_joint_speed_percent must be in [1, 10]')
        command=JointState()
        command.header.stamp=self.get_clock().now().to_msg()
        command.name=list(JOINT_NAMES)+['gripper']
        command.position=list(map(float,q))+[float(width)]
        command.velocity=[0.]*6+[speed]
        command.effort=[0.]*7
        self.joint_command.publish(command)
    def return_home(self,width):
        if self.abort_flag: raise RuntimeError('operator abort requested')
        target=np.asarray(self.get_parameter('home_joint_positions_rad').value,dtype=float)
        if target.shape!=(6,) or not np.isfinite(target).all():
            raise RuntimeError('Invalid six-joint home position')
        start=self.current_arm_joints()
        path=self.plan_home(start,target)
        self.execute_joint_path(path,width,'RETURN_HOME')
    def execute_joint_path(self,path,width,state):
        start=self.current_arm_joints()
        target=np.asarray(path[-1],dtype=float)
        if np.max(np.abs(start-path[0]))>.04:
            raise RuntimeError('Joint path start differs from current feedback')
        if self.joint_command.get_subscription_count()!=1:
            raise RuntimeError('Expected exactly one Piper /joint_ctrl_single subscriber')
        self.state=state
        rate=max(1.,float(self.get_parameter('control_hz').value))
        tolerance=float(self.get_parameter('home_joint_tolerance_rad').value)
        deviation=float(self.get_parameter('home_joint_deviation_limit_rad').value)
        deadline=time.monotonic()+float(self.get_parameter('motion_timeout_sec').value)
        current=start
        for endpoint in path:
            steps=max(1,int(np.ceil(np.max(np.abs(endpoint-current))/.01)))
            for q in np.linspace(current,endpoint,steps+1)[1:]:
                if self.abort_flag: raise RuntimeError('operator abort requested')
                actual=self.current_arm_joints()
                if np.max(np.abs(actual-q))>deviation:
                    raise RuntimeError('Return-home joint deviation exceeded limit')
                if time.monotonic()>=deadline: raise RuntimeError('Return-home motion timeout')
                self.send_home_joint_target(q,width)
                time.sleep(1./rate)
            current=endpoint
        stable=0
        while time.monotonic()<deadline:
            if self.abort_flag: raise RuntimeError('operator abort requested')
            actual=self.current_arm_joints()
            if np.max(np.abs(actual-target))>deviation:
                raise RuntimeError('Return-home final deviation exceeded limit')
            self.send_home_joint_target(target,width)
            stable=stable+1 if np.max(np.abs(actual-target))<=tolerance else 0
            if stable>=2: return
            time.sleep(1./rate)
        raise RuntimeError('Return-home feedback timeout')
    def placement_path(self):
        home=np.asarray(self.get_parameter('home_joint_positions_rad').value,dtype=float)
        angle=float(self.get_parameter('place_base_angle_deg').value)
        if home.shape!=(6,) or not np.isfinite(home).all() or np.max(np.abs(home))>1e-6:
            raise RuntimeError('Side placement requires all six home joints at zero')
        if not math.isfinite(angle) or not 0. < angle <= 145.:
            raise RuntimeError('place_base_angle_deg must be in (0, 145]')
        target=home.copy(); target[0]=math.radians(angle)
        # Explicit positive joint1 samples, not an unconstrained planner detour.
        return list(np.linspace(home,target,int(math.ceil(target[0]/.01))+1))
    def validate_joint_path(self,path,width):
        if not self.state_validity.wait_for_service(timeout_sec=3.):
            raise RuntimeError('MoveIt /check_state_validity unavailable')
        for q in path:
            if self.abort_flag: raise RuntimeError('operator abort requested')
            req=GetStateValidity.Request()
            req.group_name='arm'
            req.robot_state.is_diff=True
            req.robot_state.joint_state.name=list(JOINT_NAMES)+['joint7','joint8']
            half=max(0.,min(.035,width/2.))
            req.robot_state.joint_state.position=list(map(float,q))+[half,-half]
            response=self.wait_future(self.state_validity.call_async(req),3.)
            if response is None or not response.valid:
                raise RuntimeError('Left-side placement path invalid in MoveIt scene')
    def release_at_joint_target(self,target):
        width=float(self.get_parameter('gripper_open_m').value)
        deadline=time.monotonic()+3.
        while time.monotonic()<deadline:
            if self.abort_flag: raise RuntimeError('operator abort requested')
            actual=self.current_arm_joints()
            if np.max(np.abs(actual-target))>float(self.get_parameter('home_joint_tolerance_rad').value):
                raise RuntimeError('Arm moved away from placement position before release')
            self.send_home_joint_target(target,width)
            if abs(self.current_gripper_width()-width)<=.005: return
            time.sleep(.1)
        raise RuntimeError('Gripper release feedback timeout')
    def place_left_and_return(self):
        path=self.placement_path()
        close=float(self.get_parameter('gripper_close_m').value)
        opened=float(self.get_parameter('gripper_open_m').value)
        self.state='CHECK_LEFT_PLACE_PATH'
        self.validate_joint_path(path,close)
        self.validate_joint_path(list(reversed(path)),opened)
        # Keep the bottle gripped while first moving to the zero-arm posture.
        self.return_home(close)
        self.execute_joint_path(path,close,'ROTATE_LEFT_TO_PLACE')
        self.state='RELEASE_AT_LEFT_90'
        self.release_at_joint_target(path[-1])
        reverse=list(reversed(path))
        self.validate_joint_path(reverse,opened)
        self.execute_joint_path(reverse,opened,'RETURN_FROM_LEFT')
    def close_gripper_at_home(self):
        width=float(self.get_parameter('gripper_close_m').value)
        deadline=time.monotonic()+3.
        rate=max(1.,float(self.get_parameter('control_hz').value))
        while time.monotonic()<deadline:
            if self.abort_flag: raise RuntimeError('operator abort requested')
            self.send_home_joint_target(self.current_arm_joints(),width)
            if self.current_gripper_width()<=width+.005: return
            time.sleep(1./rate)
        raise RuntimeError('Gripper did not close at home')
    def check_abort(self):
        if self.abort_flag: raise RuntimeError('operator abort requested')
    def run(self):
        # Latched for both success and failure: a new target must not reopen a
        # possibly loaded gripper. No automatic release, home, or second pick.
        self.completion_latched=True
        try:
            self.check_abort()
            self.state='APPROACH'
            p=self.pregrasp.pose.position; pre=np.array([p.x,p.y,p.z])
            r=Rotation.from_quat([self.pregrasp.pose.orientation.x,self.pregrasp.pose.orientation.y,self.pregrasp.pose.orientation.z,self.pregrasp.pose.orientation.w])
            self.send(pre,r,float(self.active_open_width if self.active_open_width is not None else self.get_parameter('gripper_open_m').value)); self.wait_pose(pre,r,True)
            self.check_abort()
            self.state='DESCEND'
            if self.active_selection is not None:
                target=self.physical_link_pose(self.active_selection.candidate.pose);grasp=np.array([target.position.x,target.position.y,target.position.z])
            else:grasp=pre+r.apply([0.,0.,float(self.get_parameter('pregrasp_distance_m').value)])
            self.send(grasp,r,float(self.active_open_width if self.active_open_width is not None else self.get_parameter('gripper_open_m').value)); self.wait_pose(grasp,r,True)
            self.check_abort()
            self.state='CLOSE'; close=float(self.get_parameter('gripper_close_m').value); self.send(grasp,r,close)
            for _ in range(20):
                self.check_abort()
                time.sleep(.05)
            self.require_contact()
            self.check_abort()
            self.state='LIFT'
            if self.active_selection is not None:
                target=self.physical_link_pose(self.active_selection.lift_pose);lift=np.array([target.position.x,target.position.y,target.position.z])
            else:lift=grasp+np.asarray(self.get_parameter('lift_delta_base_m').value,dtype=float)
            self.send(lift,r,close); self.wait_pose(lift,r)
            self.require_contact()
            self.check_abort()
            self.state='HOLDING'
        except Exception as exc:
            self.state='RECOVERY_REQUIRED: '+str(exc)
            try:
                self.hold()
            except Exception as hold_exc:
                self.state+='; HOLD_UNCONFIRMED: '+str(hold_exc)
        finally:
            self.pregrasp=None
            self.running=False

def main(args=None):
    rclpy.init(args=args)
    node=TaskExecutor()
    try: rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
