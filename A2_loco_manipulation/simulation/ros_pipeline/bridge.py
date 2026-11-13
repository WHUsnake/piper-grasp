"""MuJoCo camera/joint/action adapter; controller has no physical transport."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,json,sys,threading,time
from pathlib import Path
import numpy as np
# Keep ROS's NumPy 1 ABI; load only missing simulation libraries afterwards.
ROOT=Path(__file__).resolve().parent;PROJECT=ROOT.parents[1];SHE=PROJECT.parent
sys.path.append(str(SHE/'supermimic训练机械臂抓取/.venv/lib/python3.10/site-packages'))
import mujoco
import imageio.v2 as imageio
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer,CancelResponse,GoalResponse
from sensor_msgs.msg import Image,CameraInfo,JointState
from geometry_msgs.msg import PoseStamped,TransformStamped
from std_msgs.msg import Float64,String
from std_srvs.srv import Trigger
from control_msgs.action import FollowJointTrajectory
from tf2_ros import StaticTransformBroadcaster
from scipy.spatial.transform import Rotation

from trajectory_timing import eased_path_time

class Bridge(Node):
 def __init__(self,folder,execution_speed_scale=1.0):
  super().__init__('mujoco_bridge');self.folder=folder;self.parameters=json.loads((folder/'parameters.json').read_text());self.session=folder.name
  self.execution_speed_scale=float(execution_speed_scale)
  if not 1.0<=self.execution_speed_scale<=1.3:raise ValueError('Simulation execution speed scale must be in [1.0, 1.3]')
  self.m=mujoco.MjModel.from_xml_path(str(folder/'scene.xml'));self.d=mujoco.MjData(self.m)
  self.qids=np.array([self.m.jnt_qposadr[self.m.joint('p_joint'+str(i)).id] for i in range(1,7)])
  self.d.qpos[6:8]=[.035,-.035];self.d.ctrl[6]=.035;mujoco.mj_forward(self.m,self.d)
  self.target_name='bottle' if self.parameters.get('target_kind','bottle')=='bottle' else 'round_target'
  self.target_body=self.d.body(self.target_name)
  self.lock=threading.RLock();self.phase='zero';self.finished=False;self.frames=[];self.trace=[];self.hold=[];self.initial_target=self.target_body.xpos.copy();self.last_stats={};self.action_start=None
  self.joints=self.create_publisher(JointState,'/joint_states',10);self.feedback=self.create_publisher(JointState,'/joint_states_feedback',10);self.pose=self.create_publisher(PoseStamped,'/end_pose_stamped',10)
  self.rgb=self.create_publisher(Image,'/camera/color/image_raw',2);self.depth=self.create_publisher(Image,'/camera/depth/image_raw',2);self.info=self.create_publisher(CameraInfo,'/camera/color/camera_info',2)
  self.create_subscription(Float64,'/sim/gripper/command',self.gripper,2);self.create_subscription(String,'/task/status',self.status,10)
  self.create_service(Trigger,'/sim/ready',self.ready);self.create_service(Trigger,'/sim/finish',self.finish)
  self.create_service(Trigger,'/sim/target_near_support',self.target_near_support)
  self.create_service(Trigger,'/sim/target_placed',self.target_placed)
  self.create_service(Trigger,'/sim/dc1_visible',self.dc1_visible)
  self.action=ActionServer(self,FollowJointTrajectory,'/sim/arm_controller/follow_joint_trajectory',execute_callback=self.execute,goal_callback=lambda r:GoalResponse.ACCEPT,cancel_callback=lambda h:CancelResponse.ACCEPT)
  self.create_timer(.02,self.publish_feedback)
  self.broadcast=StaticTransformBroadcaster(self)
  cam=self.m.camera('dc1_rgbd').id;rot=self.d.cam_xmat[cam].reshape(3,3)@np.diag([1,-1,-1]);self.camera_rotation=rot;self.camera_translation=self.d.cam_xpos[cam].copy()
  tf=TransformStamped();tf.header.frame_id='world';tf.child_frame_id='camera_color_optical_frame';tf.header.stamp=self.get_clock().now().to_msg()
  tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=map(float,self.camera_translation);q=Rotation.from_matrix(rot).as_quat();tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=map(float,q);self.broadcast.sendTransform(tf)
  self.renderer=mujoco.Renderer(self.m,height=480,width=640);self.overview=mujoco.MjvCamera();self.overview.lookat[:]=[.3,0,.5];self.overview.distance=2.45;self.overview.azimuth=135;self.overview.elevation=-20
  self.writer=imageio.get_writer(folder/'overview.mp4',fps=25);self.close_writer=imageio.get_writer(folder/'closeup.mp4',fps=25)
  self.close_cam=mujoco.MjvCamera();p=self.parameters;self.close_cam.lookat[:]=[p.get('target_x',p['bottle_x'])-.04,p.get('target_y',p['bottle_y']),p['table_z']+(.13 if self.target_name=='bottle' else .05)];self.close_cam.distance=.82;self.close_cam.azimuth=115;self.close_cam.elevation=-18
  self.n=0;self.target_geoms={i for i in range(self.m.ngeom) if self.m.geom_bodyid[i]==self.target_body.id and self.m.geom_contype[i]};self.finger_bodies={self.m.body(n).id for n in ['p_link7','p_link8']}
  self.visible_geoms={i for i in range(self.m.ngeom) if self.m.geom_bodyid[i]==self.target_body.id}
  self.dc1_visible_count,self.dc1_target_clipped=self.target_visibility()
  np.save(folder/'initial_qpos.npy',self.d.qpos.copy())
 def ready(self,q,r):r.success=True;r.message=self.session;return r
 def finish(self,q,r):self.finished=True;r.success=True;return r
 def near_support(self):
  p=self.parameters;half=p.get('target_half_height_m',.105)
  return bool(abs(float(self.target_body.xpos[2]-half-p['table_z']))<.025 and
              np.linalg.norm(self.target_body.xpos[:2]-self.initial_target[:2])<.04)
 def placed(self):
  if not self.near_support():return False
  table=self.m.geom('table_top').id
  supported=any((c.geom1==table and c.geom2 in self.target_geoms) or (c.geom2==table and c.geom1 in self.target_geoms) for c in self.d.contact)
  return bool(supported and self.d.qpos[6]-self.d.qpos[7]>.045)
 def target_near_support(self,q,r):
  with self.lock:r.success=self.near_support()
  r.message='target is near original table support' if r.success else 'target is not near table support';return r
 def target_placed(self,q,r):
  with self.lock:r.success=self.placed()
  r.message='target rests on table with open gripper' if r.success else 'target is not stably supported after release';return r
 def target_visibility(self):
  self.renderer.enable_segmentation_rendering();self.renderer.update_scene(self.d,camera='dc1_rgbd')
  seg=self.renderer.render().copy();self.renderer.disable_segmentation_rendering()
  mask=np.isin(seg[:,:,0],list(self.visible_geoms))&(seg[:,:,1]==int(mujoco.mjtObj.mjOBJ_GEOM))
  clipped=bool(np.any(mask[:5]) or np.any(mask[-5:]) or np.any(mask[:,:5]) or np.any(mask[:,-5:]))
  return int(np.count_nonzero(mask)),clipped
 def dc1_visible(self,q,r):
  with self.lock:count,clipped=self.dc1_visible_count,self.dc1_target_clipped
  minimum=1000 if clipped else 100
  r.success=count>=minimum;r.message=f'DC1 target visible pixels: {count} (minimum {minimum}); clipped at image border: {clipped}';return r
 def status(self,m):self.phase={'APPROACH':'pregrasp','DESCEND':'approach','CLOSE':'close','LIFT':'lift','HOLDING':'hold','PLACE_DESCEND':'place_lower','PLACE_RELEASE':'place_release','PLACE_RETREAT':'place_retreat','RETURN_HOME':'return_home','PLACED':'placed'}.get(m.data,self.phase)
 def gripper(self,m):
  if not np.isfinite(m.data) or not -1e-6<=m.data<=.070001:return
  with self.lock:self.d.ctrl[6]=min(.07,max(0.,m.data))/2
 def publish_feedback(self):
  with self.lock:
   stamp=self.get_clock().now().to_msg();j=JointState();j.header.stamp=stamp;j.name=[f'joint{i}' for i in range(1,9)]+['gripper'];j.position=self.d.qpos[:8].tolist()+[float(self.d.qpos[6]-self.d.qpos[7])];j.effort=self.d.qfrc_actuator[:6].tolist()+[float(self.last_stats.get('contact_force_n',0)),0.,0.]
   self.feedback.publish(j)
   j.name=j.name[:8];j.position=j.position[:8];j.effort=j.effort[:8];self.joints.publish(j)
   p=PoseStamped();p.header.stamp=stamp;p.header.frame_id='world';b=self.d.body('p_link6');p.pose.position.x,p.pose.position.y,p.pose.position.z=map(float,b.xpos);q=Rotation.from_matrix(b.xmat.reshape(3,3)).as_quat();p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w=map(float,q);self.pose.publish(p)
 def execute(self,handle):
  result=FollowJointTrajectory.Result();t=handle.request.trajectory
  names=[f'joint{i}' for i in range(1,7)]
  if set(t.joint_names)!=set(names) or not t.points:
   result.error_code=-2;result.error_string='Expected six planned arm joints';handle.abort();return result
  order=[t.joint_names.index(n) for n in names];times=np.array([p.time_from_start.sec+p.time_from_start.nanosec/1e9 for p in t.points]);positions=np.array([p.positions for p in t.points])[:,order]
  if not np.isfinite(positions).all() or np.any(np.diff(times)<0) or times[-1]<=0:
   result.error_code=-1;result.error_string='Invalid planned trajectory';handle.abort();return result
  with self.lock:start=float(self.d.time);self.action_start=start if self.action_start is None else self.action_start
  while rclpy.ok():
   if handle.is_cancel_requested:handle.canceled();result.error_code=-4;return result
   with self.lock:
    elapsed=self.d.time-start;path_time=eased_path_time(elapsed*self.execution_speed_scale,float(times[-1]));self.d.ctrl[:6]=[np.interp(path_time,times,positions[:,i]) for i in range(6)]
    error=float(np.max(np.abs(self.d.qpos[self.qids]-positions[-1])))
   if path_time>=times[-1] and error<.025:break
   if elapsed>(times[-1]+.3)/self.execution_speed_scale+8:
    result.error_code=-5;result.error_string=f'Tracking error {error:.4f} rad';handle.abort();return result
   time.sleep(.005)
  handle.succeed();result.error_code=0;return result
 def capture(self):
  if self.action_start is not None:return
  self.dc1_visible_count,self.dc1_target_clipped=self.target_visibility()
  if self.dc1_visible_count<(1000 if self.dc1_target_clipped else 100):return
  self.renderer.update_scene(self.d,camera='dc1_rgbd');rgb=self.renderer.render().copy();self.renderer.enable_depth_rendering();self.renderer.update_scene(self.d,camera='dc1_rgbd');depth=self.renderer.render().copy();self.renderer.disable_depth_rendering()
  stamp=self.get_clock().now().to_msg()
  for arr,encoding,pub in [(rgb,'rgb8',self.rgb),(depth.astype('<f4'),'32FC1',self.depth)]:
   msg=Image();msg.header.frame_id='camera_color_optical_frame';msg.header.stamp=stamp;msg.height=480;msg.width=640;msg.encoding=encoding;msg.step=640*(3 if encoding=='rgb8' else 4);msg.data=arr.tobytes();pub.publish(msg)
  f=240/np.tan(np.deg2rad(60)/2);info=CameraInfo();info.header=msg.header;info.height=480;info.width=640;info.distortion_model='plumb_bob';info.d=[0.]*5;info.k=[f,0.,319.5,0.,f,239.5,0.,0.,1.];self.info.publish(info)
  if not (self.folder/'camera_rgb.png').exists():imageio.imwrite(self.folder/'camera_rgb.png',rgb)
 def record(self):
  contacts=set();force=0.;insets=[];heights=[]
  for i,c in enumerate(self.d.contact):
   a,b=int(c.geom1),int(c.geom2);other=b if a in self.target_geoms else a if b in self.target_geoms else -1
   if other>=0 and self.m.geom_bodyid[other] in self.finger_bodies:
    contacts.add(int(self.m.geom_bodyid[other]));f=np.zeros(6);mujoco.mj_contactForce(self.m,self.d,i,f);force+=abs(f[0])
    link=self.d.body('p_link6');v=link.xmat.reshape(3,3).T@(c.pos-link.xpos);insets.append(float(.13503-v[2]));v=self.target_body.xmat.reshape(3,3).T@(c.pos-self.target_body.xpos);heights.append(float(v[2]+self.parameters.get('target_half_height_m',.105 if self.target_name=='bottle' else .028)))
  lift=float(self.target_body.xpos[2]-self.initial_target[2]);self.last_stats=dict(contact_force_n=force)
  row=dict(t=round(float(self.d.time),3),phase=self.phase,bottle_lift_m=lift,contacts=len(contacts),q=self.d.qpos[:6].tolist(),gripper_width_m=float(self.d.qpos[6]-self.d.qpos[7]),contact_force_n=force,contact_inset_m=float(np.median(insets)) if insets else None,contact_height_from_bottom_m=float(np.median(heights)) if heights else None)
  self.trace.append(row);self.frames.append(dict(t=row['t'],phase=self.phase,qpos=self.d.qpos.tolist(),lift_m=lift,contacts=len(contacts)))
  if self.phase=='hold':self.hold.append(row)
  self.renderer.update_scene(self.d,self.overview);frame=self.renderer.render().copy();self.writer.append_data(frame)
  if not (self.folder/'initial.png').exists():imageio.imwrite(self.folder/'initial.png',frame)
  if self.phase=='hold':imageio.imwrite(self.folder/'result.png',frame)
  self.renderer.update_scene(self.d,self.close_cam);self.close_writer.append_data(self.renderer.render().copy())
 def run(self):
  try:
   while rclpy.ok() and not self.finished:
    start=time.monotonic()
    with self.lock:
     mujoco.mj_step(self.m,self.d);self.n+=1
     if self.n%20==0:self.record()
     if self.n%100==0:self.capture()
    time.sleep(max(0,.002-(time.monotonic()-start)))
  finally:
   self.writer.close();self.close_writer.close();self.renderer.close()
   (self.folder/'trace.json').write_text(json.dumps(self.trace));(self.folder/'frames.json').write_text(json.dumps(self.frames))
   (self.folder/'physical.json').write_text(json.dumps(dict(hold=self.hold,placement_verified=self.placed(),initial_target=self.initial_target.tolist(),camera_rotation=self.camera_rotation.tolist(),camera_translation=self.camera_translation.tolist(),action_start=self.action_start,execution_speed_scale=self.execution_speed_scale)))

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--execution-speed-scale',type=float,default=1.0);a=p.parse_args();rclpy.init();b=Bridge(a.output.resolve(),a.execution_speed_scale);ex=MultiThreadedExecutor(num_threads=4);ex.add_node(b);threading.Thread(target=ex.spin,daemon=True).start()
 try:b.run()
 finally:ex.shutdown();b.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
