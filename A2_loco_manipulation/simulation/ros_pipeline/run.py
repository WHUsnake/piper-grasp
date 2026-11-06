"""Own one isolated ROS2/MoveIt/MuJoCo experiment and collect its evidence."""
import argparse,copy,json,os,signal,subprocess,sys,threading,time,fcntl,hashlib
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from grasp_msgs.msg import SelectedGrasp,GraspCandidateArray
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene,CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parent;PROJECT=ROOT.parents[1];SHE=PROJECT.parent

class Monitor(Node):
 def __init__(self,out):
  super().__init__('pipeline_experiment');self.out=out;self.selected=None;self.state='';self.cloud=None;self.candidates=None;self.diagnostic='';self.clouds={};self.grasp_diagnostic={};self.grasp_history={}
  self.create_subscription(SelectedGrasp,'/planner/selected_grasp',lambda m:setattr(self,'selected',m),2)
  self.create_subscription(String,'/task/status',lambda m:setattr(self,'state',m.data),10)
  self.create_subscription(PointCloud2,'/perception/target_cloud',self.receive_cloud,2)
  self.create_subscription(String,'/grasp/diagnostics',self.receive_grasp_diagnostic,10)
  self.create_subscription(GraspCandidateArray,'/grasp/candidates',lambda m:setattr(self,'candidates',m),2)
  self.create_subscription(String,'/planner/diagnostics',lambda m:setattr(self,'diagnostic',m.data),10)
 def receive_cloud(self,m):
  self.cloud=m;key=(m.header.stamp.sec,m.header.stamp.nanosec);self.clouds[key]=m
  while len(self.clouds)>100:self.clouds.pop(next(iter(self.clouds)))
 def receive_grasp_diagnostic(self,m):
  try:
   value=json.loads(m.data)
   if value.get('status')=='ready':
    self.grasp_diagnostic=value
    if value.get('acquisition_stamp'):
     self.grasp_history[tuple(value['acquisition_stamp'])]=value
     while len(self.grasp_history)>100:self.grasp_history.pop(next(iter(self.grasp_history)))
  except ValueError:pass
 def call(self,name,request=None,service=Trigger,timeout=180):
  client=self.create_client(service,name)
  if not client.wait_for_service(timeout_sec=30):raise RuntimeError(name+' unavailable')
  future=client.call_async(request or service.Request());end=time.monotonic()+timeout
  while not future.done():
   if time.monotonic()>end:raise TimeoutError(name+' timeout')
   time.sleep(.02)
  result=future.result();self.destroy_client(client);return result
 def wait_until(self,predicate,timeout,reason):
  end=time.monotonic()+timeout
  while not predicate():
   if time.monotonic()>end:raise TimeoutError(reason)
   time.sleep(.05)


def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--bottle-x',type=float,default=.72);p.add_argument('--bottle-y',type=float,default=0);p.add_argument('--table-z',type=float,default=.76);p.add_argument('--tcp-m',type=float,default=.095);a=p.parse_args();out=a.output.resolve();out.mkdir(parents=True,exist_ok=True)
 lock=(ROOT/"session.lock").open("a")
 try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 except BlockingIOError:raise SystemExit("Another isolated ROS pipeline experiment is running")
 procs=[];logs=[]
 def spawn(name,cmd,env=None):
  log=(out/(name+'.log')).open('w');logs.append(log)
  process=subprocess.Popen(list(map(str,cmd)),env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);process.experiment_name=name;procs.append(process);return process
 def progress(stage):(out/'progress.json').write_text(json.dumps({'stage':stage}))
 sim_python=SHE/'supermimic训练机械臂抓取/.venv/bin/python'
 subprocess.run([str(sim_python),str(ROOT/'prepare.py'),'--output',str(out),'--bottle-x',str(a.bottle_x),'--bottle-y',str(a.bottle_y),'--table-z',str(a.table_z),'--tcp-m',str(a.tcp_m)],check=True)
 config=json.loads((out/'moveit.json').read_text());(out/'moveit.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':config}}))
 session=out.name
 rclpy.init()
 def interrupted(signum,frame):raise InterruptedError('Simulation session cancelled')
 signal.signal(signal.SIGTERM,interrupted)
 n=Monitor(out);executor=MultiThreadedExecutor(num_threads=4);executor.add_node(n);threading.Thread(target=executor.spin,daemon=True).start()
 failure=None;execution_started=False
 try:
  progress('building')
  spawn('robot_state_publisher',['/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher','--ros-args','--params-file',out/'moveit.yaml'])
  spawn('move_group',['/opt/ros/humble/lib/moveit_ros_move_group/move_group','--ros-args','--params-file',out/'moveit.yaml'])
  spawn('mujoco_bridge',[sys.executable,ROOT/'bridge.py','--output',out])
  ready=n.call('/sim/ready');assert ready.success
  scene=PlanningScene();scene.is_diff=True;obj=CollisionObject();obj.id='table';obj.header.frame_id='world';obj.operation=CollisionObject.ADD
  boxes=[([a.bottle_x+.2,0,a.table_z-.025],[.56,.8,.05])]
  for x in [a.bottle_x-.01,a.bottle_x+.41]:
   for y in [-.32,.32]:boxes.append(([x,y,(a.table_z-.05)/2],[.044,.044,a.table_z-.05]))
  for xyz,size in boxes:
   obj.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX,dimensions=size));pose=Pose();pose.position.x,pose.position.y,pose.position.z=map(float,xyz);pose.orientation.w=1.;obj.primitive_poses.append(pose)
  scene.world.collision_objects=[obj];result=n.call('/apply_planning_scene',ApplyPlanningScene.Request(scene=scene),ApplyPlanningScene);assert result.success
  spawn('grasp_generator',[sys.executable,'-c','from grasp_generation.node import main; main()','--ros-args','-p',f'graspnet_config:={out}/graspnet.json'])
  spawn('perception',[SHE/'search-grasp2/.venv/bin/python','-c','from perception_node.node import main; main()','--ros-args','-p',f'weights:={SHE}/search-grasp2/models/yolo11s-seg.pt','-p','simulation_mode:=true','-p',f'target_session:={session}','-p',f'capture_directory:={out}'])
  spawn('planner',[sys.executable,'-c','from robot_planner.node import main; main()','--ros-args','-p','base_frame:=world','-p','end_effector_link:=p_link6','-p','tcp_calibrated:=true','-p',f'tool_translation:=[0.0,0.0,{a.tcp_m}]','-p',f'session_id:={session}','-p','max_gripper_width_m:=0.07','-p','require_target_collision:=true','-p','lift_retreat_options_m:=[0.0,0.04,0.08,0.12]','-p','pregrasp_distance_options_m:=[0.0,0.06,0.14]'])
  spawn('executor',[sys.executable,'-c','from task_executor.node import main; main()','--ros-args','-p','execute:=false','-p','simulation_mode:=true','-p',f'session_id:={session}'])
  progress('inference');n.wait_until(lambda:n.candidates is not None and len(n.candidates.candidates)>0,60,'YOLO/GraspNet did not produce valid candidates')
  progress('planning');result=n.call('/planner/preview');(out/'planner_diagnostics.json').write_text(n.diagnostic)
  if not result.success:raise RuntimeError(result.message+' '+n.diagnostic)
  n.wait_until(lambda:n.selected is not None,5,'Selected grasp not received')
  from rosidl_runtime_py.convert import message_to_ordereddict
  (out/'selected_grasp.json').write_text(json.dumps(message_to_ordereddict(n.selected),default=list,indent=2))
  # Save the exact camera cloud consumed during the observation chain.
  selected_stamp=n.selected.header.stamp;cloud=n.clouds.get((selected_stamp.sec,selected_stamp.nanosec))
  if cloud is None:raise RuntimeError('Exact selected observation is no longer available for audit')
  (out/'selected_observation.json').write_text(json.dumps(dict(sec=cloud.header.stamp.sec,nanosec=cloud.header.stamp.nanosec,frame_id=cloud.header.frame_id)))
  selected_diagnostic=n.grasp_history.get((selected_stamp.sec,selected_stamp.nanosec),{})
  (out/'selected_grasp_diagnostics.json').write_text(json.dumps(selected_diagnostic,indent=2))
  points=np.array(list(point_cloud2.read_points(cloud,field_names=('x','y','z'),skip_nans=True)))
  if points.dtype.names:points=np.column_stack([points[k] for k in ('x','y','z')])
  n.wait_until(lambda:n.candidates is not None and len(n.candidates.candidates)>0 and 0<=(n.get_clock().now()-rclpy.time.Time.from_msg(n.candidates.header.stamp)).nanoseconds/1e9<.4,15,'No fresh matching observation after planning')
  result=n.call('/sim/task/start',timeout=10)
  if not result.success:raise RuntimeError(result.message)
  execution_started=True
  n.wait_until(lambda:n.state=='HOLDING' or n.state.startswith('RECOVERY_REQUIRED'),150,'Executor timed out')
  if n.state!='HOLDING':raise RuntimeError(n.state)
  progress('hold');time.sleep(4)
 except Exception as exc:failure=str(exc);print('PIPELINE FAILURE:',failure,flush=True)
 finally:
  try:n.call('/sim/finish',timeout=5)
  except Exception:pass
  time.sleep(1)
  for proc in reversed(procs):
   if proc.poll() is None:
    os.killpg(proc.pid,signal.SIGTERM)
    try:proc.wait(timeout=4)
    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
  executor.shutdown();n.destroy_node();rclpy.shutdown()
  for log in logs:log.close()
  (out/'process_exit_codes.json').write_text(json.dumps([{'name':p.experiment_name,'pid':p.pid,'returncode':p.returncode} for p in procs],indent=2))
 physical=json.loads((out/'physical.json').read_text()) if (out/'physical.json').is_file() else {}
 hold=physical.get('hold',[])
 sel=n.selected
 if failure:
  (out/'pipeline_failure.json').write_text(json.dumps({'failure':failure,'planner':n.diagnostic,'execution_started':execution_started},indent=2));raise SystemExit(failure)
 c=sel.candidate;q=c.pose.orientation;r=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();v=c.pose.position
 selected=dict(position=[v.x,v.y,v.z],rotation_world=r.tolist(),quaternion=[q.x,q.y,q.z,q.w],width=c.gripper_width_m,score=c.score,raw_index=sel.candidate_index,geometry_refined=True,pad_center_from_link6_m=.105,pregrasp_distance=c.pregrasp_distance_m)
 selected_source=selected_diagnostic.get('candidates',[])[sel.candidate_index]
 for key in ['orientation_adaptation','adapted_elevation_deg','network_rotation_world','parallel_gripper_symmetry']:
  if key in selected_source:selected[key]=selected_source[key]
 selected['score_semantics']='parent_network_proposal_score; adapted orientation requires independent validation'
 report=dict(success=bool(hold and min(x['bottle_lift_m'] for x in hold)>.08 and np.mean([x['contacts']==2 for x in hold])>.9),status='success',parameters=json.loads((out/'parameters.json').read_text()),tcp_m=a.tcp_m,video_available=True,selected=selected,grasp_strategy='body_middle_pad_center_v2',pipeline='ROS2_YOLO_GraspNet_MoveIt_TaskExecutor',oracle_segmentation=False,
  point_count=len(points),graspnet_inference_sec=selected_diagnostic.get('inference_sec'),min_hold_lift_m=min(x['bottle_lift_m'] for x in hold),bilateral_hold_fraction=float(np.mean([x['contacts']==2 for x in hold])),hold_contact_inset_median_m=float(np.median([x['contact_inset_m'] for x in hold if x['contact_inset_m'] is not None])),close_contact_height_from_bottom_m=float(np.median([x['contact_height_from_bottom_m'] for x in json.loads((out/'trace.json').read_text()) if x['phase']=='close' and x['contact_height_from_bottom_m'] is not None])),pad_support_verified=True,physical_attachment=False,object_pose_written_during_rollout=False,execute_robot=False)
 report['pad_support_verified']=report['hold_contact_inset_median_m']>=.008
 report['success']=bool(report['success'] and report['pad_support_verified'])
 report['status']='success' if report['success'] else 'grasp_failed'
 report['collision_scene']='robot_self_table_and_depth_fitted_bottle_with_attached_lift'
 report['source_paths']={'perception':'src/perception/perception_node','generator':'src/manipulation/grasp_generation','planner':'src/manipulation/robot_planner','executor':'src/manipulation/task_executor'}
 (out/'metrics.json').write_text(json.dumps(report,indent=2));(out/'proposals.json').write_text(n.diagnostic or '{}')
 world=points@np.array(physical['camera_rotation']).T+physical['camera_translation'];(out/'pointcloud.json').write_text(json.dumps(world[::max(1,len(world)//2200)].tolist()))
 print(json.dumps(report),flush=True)
if __name__=='__main__':main()
