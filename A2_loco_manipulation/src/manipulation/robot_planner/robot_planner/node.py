"""MoveIt plan-only service: acquisition-time TF, explicit grasp-to-link transform."""
import copy
import threading
import time
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from grasp_msgs.msg import GraspCandidateArray
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from scipy.spatial.transform import Rotation

from .full_planning import FullPlanning

class RobotPlanner(FullPlanning,Node):
    def __init__(self):
        super().__init__('robot_planner')
        self.declare_parameter('base_frame','base_link')
        self.declare_parameter('end_effector_link','link6')
        self.declare_parameter('planning_group','arm')
        self.declare_parameter('tcp_calibrated',False)
        self.declare_parameter('tool_translation',[0.,0.,0.])
        self.declare_parameter('tool_quaternion',[0.,0.,0.,1.])
        self.latest=None
        self.perception_status='No perception diagnostics received'
        self.perception_status_at=0.0
        self.lock=threading.Lock()
        group=ReentrantCallbackGroup()
        self.initialize_full_planning(group)
        self.buffer=Buffer()
        self.listener=TransformListener(self.buffer,self)
        self.action=ActionClient(self,MoveGroup,'/move_action',callback_group=group)
        self.create_subscription(GraspCandidateArray,'/grasp/candidates',self.receive,2,callback_group=group)
        self.create_subscription(String, '/perception/status', self.receive_status, 10, callback_group=group)
        self.selected_pub=self.create_publisher(PoseStamped,'/planner/selected_pregrasp',2)
        self.create_service(Trigger,'/planner/preview',self.preview,callback_group=group)
    def receive(self,msg): self.latest=msg
    def receive_status(self, msg):
        self.perception_status = msg.data
        self.perception_status_at = time.monotonic()
    def fresh_observation(self):
        # Wait for new evidence, never increase the accepted observation age.
        deadline = time.monotonic() + 5.0
        while True:
            msg = self.latest
            if msg is not None and msg.candidates:
                stamp = rclpy.time.Time.from_msg(msg.header.stamp)
                age = (self.get_clock().now()-stamp).nanoseconds/1e9
                if 0 <= age <= 1.0:
                    return msg
            if time.monotonic() >= deadline:
                status = self.perception_status
                if time.monotonic()-self.perception_status_at > 3.0:
                    status = 'Perception diagnostics missing/stale; check camera and perception node'
                raise ValueError('No fresh grasp candidates within 5s; ' + status)
            time.sleep(0.02)
    def wait(self,future,timeout):
        end=time.monotonic()+timeout
        while not future.done():
            if time.monotonic()>end: raise TimeoutError('MoveIt response timeout')
            time.sleep(0.01)
        return future.result()
    def preview(self,request,response):
        if self.get_parameter('plan_full_grasp').value:return self.full_preview(request,response)
        if not self.lock.acquire(False):
            response.message='Planning already active'; return response
        try:
            if not self.get_parameter('tcp_calibrated').value: raise ValueError('Measured link-to-grasp transform is required')
            msg=self.fresh_observation()
            stamp=rclpy.time.Time.from_msg(msg.header.stamp)
            if not 0 <= (self.get_clock().now()-stamp).nanoseconds/1e9 <= 1.: raise ValueError('Observation expired')
            transform=self.buffer.lookup_transform(self.get_parameter('base_frame').value,msg.header.frame_id,stamp)
            if not self.action.wait_for_server(timeout_sec=2.): raise ValueError('MoveIt /move_action unavailable')
            for candidate in msg.candidates[:12]:
                target=do_transform_pose(candidate.pose,transform)
                r=Rotation.from_quat([target.orientation.x,target.orientation.y,target.orientation.z,target.orientation.w])
                tool=Rotation.from_quat(self.get_parameter('tool_quaternion').value)
                link=r*tool.inv()
                center=np.array([target.position.x,target.position.y,target.position.z])
                end=center-r.apply([0,0,candidate.pregrasp_distance_m])-link.apply(self.get_parameter('tool_translation').value)
                target.position.x,target.position.y,target.position.z=map(float,end)
                target.orientation.x,target.orientation.y,target.orientation.z,target.orientation.w=map(float,link.as_quat())
                goal=MoveGroup.Goal()
                goal.request.group_name=self.get_parameter('planning_group').value
                goal.request.allowed_planning_time=2.
                goal.request.max_velocity_scaling_factor=0.1
                goal.request.max_acceleration_scaling_factor=0.1
                goal.request.start_state.is_diff=True
                pc=PositionConstraint()
                pc.header.frame_id=self.get_parameter('base_frame').value
                pc.link_name=self.get_parameter('end_effector_link').value
                pc.weight=1.
                pc.constraint_region.primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE,dimensions=[0.003])]
                pc.constraint_region.primitive_poses=[target]
                oc=OrientationConstraint()
                oc.header=pc.header
                oc.link_name=pc.link_name
                oc.orientation=target.orientation
                oc.absolute_x_axis_tolerance=oc.absolute_y_axis_tolerance=oc.absolute_z_axis_tolerance=0.03
                oc.weight=1.
                goal.request.goal_constraints=[Constraints(position_constraints=[pc],orientation_constraints=[oc])]
                goal.planning_options.plan_only=True
                handle=self.wait(self.action.send_goal_async(goal),5.)
                if not handle.accepted: continue
                try: result=self.wait(handle.get_result_async(),10.)
                except TimeoutError:
                    handle.cancel_goal_async()
                    raise
                if result.status==4 and result.result.error_code.val==1:
                    selected=PoseStamped()
                    selected.header=msg.header
                    selected.header.frame_id=self.get_parameter('base_frame').value
                    selected.pose=target
                    self.selected_pub.publish(selected)
                    response.success=True
                    response.message='Pregrasp plan valid; grasp/contact/lift/place not executed'
                    return response
            raise ValueError('No reachable pregrasp candidate')
        except Exception as exc: response.message=str(exc)
        finally: self.lock.release()
        return response

def main(args=None):
    rclpy.init(args=args)
    node=RobotPlanner()
    executor=MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try: executor.spin()
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
