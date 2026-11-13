"""Own one isolated ROS2/MoveIt/MuJoCo experiment and collect its evidence."""
import argparse,copy,json,os,signal,subprocess,sys,threading,time,fcntl,hashlib
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2,Image
from sensor_msgs_py import point_cloud2
from grasp_msgs.msg import SelectedGrasp,GraspCandidateArray
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene,CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parent;PROJECT=ROOT.parents[1];SHE=PROJECT.parent

class Monitor(Node):
 def __init__(self,out,mark):
  super().__init__('pipeline_experiment');self.out=out;self.mark=mark;self.selected=None;self.state='';self.report_execution=False;self.cloud=None;self.candidates=None;self.diagnostic='';self.clouds={};self.grasp_diagnostic={};self.grasp_history={}
  self.create_subscription(Image,'/camera/color/image_raw',lambda m:self.mark('dc1_rgb_received'),2)
  self.create_subscription(Image,'/camera/depth/image_raw',lambda m:self.mark('dc1_depth_received'),2)
  self.create_subscription(SelectedGrasp,'/planner/selected_grasp',self.receive_selected,2)
  self.create_subscription(String,'/task/status',self.receive_state,10)
  self.create_subscription(PointCloud2,'/perception/target_cloud',self.receive_cloud,2)
  self.perception_status=''
  self.create_subscription(String,'/perception/status',lambda m:setattr(self,'perception_status',m.data),10)
  self.create_subscription(String,'/grasp/diagnostics',self.receive_grasp_diagnostic,10)
  self.create_subscription(GraspCandidateArray,'/grasp/candidates',self.receive_candidates,2)
  self.create_subscription(String,'/planner/diagnostics',lambda m:setattr(self,'diagnostic',m.data),10)
 def receive_cloud(self,m):
  self.mark('target_cloud_received')
  self.cloud=m;key=(m.header.stamp.sec,m.header.stamp.nanosec);self.clouds[key]=m
  while len(self.clouds)>100:self.clouds.pop(next(iter(self.clouds)))
 def receive_state(self,m):
  changed=m.data!=self.state;self.state=m.data
  if changed:self.mark('task_'+m.data.lower())
  if self.report_execution and changed:
   stage={'APPROACH':'pregrasp','DESCEND':'approach','CLOSE':'close','LIFT':'lift','HOLDING':'hold','PLACE_DESCEND':'place_lower','PLACE_RELEASE':'place_release','PLACE_RETREAT':'place_retreat','RETURN_HOME':'return_home','PLACED':'placed'}.get(m.data)
   if stage:self.out.joinpath('progress.json').write_text(json.dumps({'stage':stage}))
 def receive_grasp_diagnostic(self,m):
  try:
   value=json.loads(m.data)
   if value.get('status')=='ready':
    self.mark('graspnet_ready')
    self.grasp_diagnostic=value
    if value.get('acquisition_stamp'):
     self.grasp_history[tuple(value['acquisition_stamp'])]=value
     while len(self.grasp_history)>100:self.grasp_history.pop(next(iter(self.grasp_history)))
  except ValueError:pass
 def receive_candidates(self,m):
  if m.candidates:self.mark('grasp_candidates_received')
  self.candidates=m
 def receive_selected(self,m):
  self.mark('selected_grasp_received');self.selected=m
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
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--bottle-x',type=float,default=.72);p.add_argument('--bottle-y',type=float,default=0);p.add_argument('--table-z',type=float,default=.76);p.add_argument('--tcp-m',type=float,default=.095);p.add_argument('--target-kind',choices=['bottle','round_proxy','apple','orange','egg','can','cup','small_carton','tissue_box','cup_inverted'],default='bottle');p.add_argument('--selected-pixel-u',type=int,default=-1);p.add_argument('--selected-pixel-v',type=int,default=-1);p.add_argument('--multi-objects',action='store_true');p.add_argument('--place-after-grasp',action='store_true');p.add_argument('--execution-speed-scale',type=float,default=1.25);a=p.parse_args();out=a.output.resolve();out.mkdir(parents=True,exist_ok=True)
 if (a.selected_pixel_u<0)!=(a.selected_pixel_v<0) or a.selected_pixel_u>=640 or a.selected_pixel_v>=480:raise ValueError('Selected DC1 pixel must be within 640x480')
 if a.multi_objects and a.target_kind not in ('bottle','apple','small_carton','can','cup'):raise ValueError('Multi-object experiment currently supports bottle, apple, small_carton, can or cup as selected target')
 if a.multi_objects and a.selected_pixel_u<0:raise ValueError('Multi-object experiment requires a selected DC1 pixel')
 lock=(ROOT/"session.lock").open("a")
 try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 except BlockingIOError:raise SystemExit("Another isolated ROS pipeline experiment is running")
 procs=[];logs=[];timeline={};timeline_lock=threading.Lock();origin_ns=time.monotonic_ns()
 def mark(name):
  with timeline_lock:
   if name not in timeline:
    timeline[name]={'elapsed_sec':round((time.monotonic_ns()-origin_ns)/1e9,6),'wall_time_unix_sec':round(time.time(),6)}
 def save_timeline(outcome):
  with timeline_lock:events=dict(sorted(timeline.items(),key=lambda item:item[1]['elapsed_sec']))
  seen=events.get('dc1_depth_received');grasp=events.get('task_holding')
  report={'clock':'monotonic_ns on pipeline host; wall_time_unix_sec for display only','camera_source':'dc1_rgbd','outcome':outcome,'events':events,'dc1_depth_to_holding_sec':round(grasp['elapsed_sec']-seen['elapsed_sec'],6) if seen and grasp else None}
  (out/'timeline.json').write_text(json.dumps(report,indent=2))
 mark('run_started')
 def spawn(name,cmd,env=None):
  log=(out/(name+'.log')).open('w');logs.append(log)
  process=subprocess.Popen(list(map(str,cmd)),env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);process.experiment_name=name;procs.append(process);return process
 def progress(stage):(out/'progress.json').write_text(json.dumps({'stage':stage}))
 sim_python=SHE/'supermimic训练机械臂抓取/.venv/bin/python'
 subprocess.run([str(sim_python),str(ROOT/'prepare.py'),'--output',str(out),'--bottle-x',str(a.bottle_x),'--bottle-y',str(a.bottle_y),'--table-z',str(a.table_z),'--tcp-m',str(a.tcp_m),'--target-kind',a.target_kind,*(['--multi-objects'] if a.multi_objects else [])],check=True)
 mark('scene_prepared')
 parameters=json.loads((out/'parameters.json').read_text());parameters['place_after_grasp']=a.place_after_grasp;(out/'parameters.json').write_text(json.dumps(parameters,indent=2))
 config=json.loads((out/'moveit.json').read_text());(out/'moveit.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':config}}))
 session=out.name
 rclpy.init()
 def interrupted(signum,frame):raise InterruptedError('Simulation session cancelled')
 signal.signal(signal.SIGTERM,interrupted)
 n=Monitor(out,mark);executor=MultiThreadedExecutor(num_threads=4);executor.add_node(n);threading.Thread(target=executor.spin,daemon=True).start()
 failure=None;execution_started=False;placement_failure=None
 try:
  progress('building')
  spawn('robot_state_publisher',['/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher','--ros-args','--params-file',out/'moveit.yaml'])
  spawn('move_group',['/opt/ros/humble/lib/moveit_ros_move_group/move_group','--ros-args','--params-file',out/'moveit.yaml'])
  spawn('mujoco_bridge',[sys.executable,ROOT/'bridge.py','--output',out,'--execution-speed-scale',a.execution_speed_scale])
  ready=n.call('/sim/ready');assert ready.success
  mark('bridge_ready')
  visible=n.call('/sim/dc1_visible')
  mark('dc1_visibility_checked')
  (out/'dc1_visibility.json').write_text(json.dumps({'visible':visible.success,'message':visible.message,'camera_source':'dc1_rgbd'},indent=2))
  if not visible.success:raise RuntimeError('DC1 cannot see the target at the observation pose; no grasp candidates may be generated. '+visible.message)
  scene=PlanningScene();scene.is_diff=True;obj=CollisionObject();obj.id='table';obj.header.frame_id='world';obj.operation=CollisionObject.ADD
  boxes=[([a.bottle_x+.2,0,a.table_z-.025],[.76 if a.multi_objects else .56,.8,.05])]
  for x in [a.bottle_x-.01,a.bottle_x+.41]:
   for y in [-.32,.32]:boxes.append(([x,y,(a.table_z-.05)/2],[.044,.044,a.table_z-.05]))
  for xyz,size in boxes:
   obj.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX,dimensions=size));pose=Pose();pose.position.x,pose.position.y,pose.position.z=map(float,xyz);pose.orientation.w=1.;obj.primitive_poses.append(pose)
  collision_objects=[obj]
  if a.multi_objects:
   root=ET.parse(out/'scene.xml').getroot()
   for body in root.findall(".//worldbody/body"):
    if not body.get('name','').startswith('demo_'):continue
    geom=body.find('geom');shape=geom.get('type');sizes=list(map(float,geom.get('size').split()))
    obstacle=CollisionObject();obstacle.id=body.get('name');obstacle.header.frame_id='world';obstacle.operation=CollisionObject.ADD
    primitive=SolidPrimitive()
    if shape=='sphere':primitive.type=SolidPrimitive.SPHERE;primitive.dimensions=[sizes[0]]
    elif shape=='box':primitive.type=SolidPrimitive.BOX;primitive.dimensions=[2*v for v in sizes]
    elif shape=='cylinder':primitive.type=SolidPrimitive.CYLINDER;primitive.dimensions=[2*sizes[1],sizes[0]]
    else:raise ValueError('Unsupported obstacle shape '+shape)
    pose=Pose();pose.position.x,pose.position.y,pose.position.z=map(float,body.get('pos').split());pose.orientation.w=1.
    obstacle.primitives=[primitive];obstacle.primitive_poses=[pose];collision_objects.append(obstacle)
  scene.world.collision_objects=collision_objects;result=n.call('/apply_planning_scene',ApplyPlanningScene.Request(scene=scene),ApplyPlanningScene);assert result.success
  mark('planning_scene_ready')
  spawn('grasp_generator',[sys.executable,'-c','from grasp_generation.node import main; main()','--ros-args','-p',f'graspnet_config:={out}/graspnet.json'])
  parameters=json.loads((out/'parameters.json').read_text())
  perception_args=['-p','segmentation_backend:=tabletop_depth_components','-p','simulation_mode:=true',
   '-p',f'target_session:={session}','-p',f'capture_directory:={out}',
   '-p',f'selected_pixel_u:={a.selected_pixel_u}','-p',f'selected_pixel_v:={a.selected_pixel_v}',
   '-p',f'enforce_selected_target_position:={str(a.multi_objects).lower()}',
   '-p',f"target_x_world:={parameters['target_x']}",'-p',f"target_y_world:={parameters['target_y']}",
   '-p',f'table_height_world:={a.table_z}',
   '-p','camera_rotation_world:='+json.dumps(parameters['camera_rotation_world']),
   '-p','camera_translation_world:='+json.dumps(parameters['camera_translation_world'])]
  spawn('perception',[SHE/'search-grasp2/.venv/bin/python','-c','from perception_node.node import main; main()','--ros-args',*perception_args])
  spawn('planner',[sys.executable,'-c','from robot_planner.node import main; main()','--ros-args','-p','base_frame:=world','-p','end_effector_link:=p_link6','-p','tcp_calibrated:=true','-p',f'tool_translation:=[0.0,0.0,{a.tcp_m}]','-p',f'session_id:={session}','-p','max_gripper_width_m:=0.07','-p','require_target_collision:=true','-p','lift_retreat_options_m:=[0.0,0.04,0.08,0.12]','-p','pregrasp_distance_options_m:=[0.0,0.06,0.14]'])
  spawn('executor',[sys.executable,'-c','from task_executor.node import main; main()','--ros-args','-p','execute:=false','-p','simulation_mode:=true','-p',f'place_after_grasp:={str(a.place_after_grasp).lower()}','-p',f'session_id:={session}'])
  progress('inference');mark('perception_and_inference_started')
  n.wait_until(lambda:(n.candidates is not None and len(n.candidates.candidates)>0) or n.perception_status in ('multiple_objects_select_one','clicked_pixel_has_no_object','selected_other_object'),120,'DC1 depth segmentation / GraspNet did not produce valid candidates')
  if n.perception_status=='multiple_objects_select_one':raise RuntimeError('DC1 detected multiple objects; select one in the DC1 image before grasping')
  if n.perception_status=='clicked_pixel_has_no_object':raise RuntimeError('Selected DC1 pixel is not on a tabletop object; no grasp executed')
  if n.perception_status=='selected_other_object':raise RuntimeError('Clicked object differs from the chosen grasp target; no grasp executed')
  progress('planning');mark('moveit_planning_started');planning_started=time.monotonic();result=n.call('/planner/preview');planning_sec=round(time.monotonic()-planning_started,3);mark('moveit_planning_finished')
  (out/'planning_timing.json').write_text(json.dumps({'planning_sec':planning_sec,'finished_at':time.time(),'success':result.success},indent=2))
  (out/'planner_diagnostics.json').write_text(n.diagnostic)
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
  progress('revalidating')
  n.wait_until(lambda:n.candidates is not None and len(n.candidates.candidates)>0 and 0<=(n.get_clock().now()-rclpy.time.Time.from_msg(n.candidates.header.stamp)).nanoseconds/1e9<.4,15,'No fresh matching observation after planning')
  result=n.call('/sim/task/start',timeout=10)
  if not result.success:raise RuntimeError(result.message)
  mark('task_start_accepted')
  execution_started=True
  n.report_execution=True;progress('pregrasp')
  n.wait_until(lambda:n.state=='HOLDING' or n.state.startswith('RECOVERY_REQUIRED'),150,'Executor timed out')
  if n.state!='HOLDING':raise RuntimeError(n.state)
  mark('grasp_confirmed')
  progress('hold')
  if a.place_after_grasp:
   n.wait_until(lambda:n.state=='PLACED' or n.state.startswith('RECOVERY_REQUIRED'),100,'Placement timed out')
   if n.state!='PLACED':placement_failure=n.state;progress('placement_failed')
   else:progress('placed');time.sleep(1)
  else:time.sleep(4)
 except Exception as exc:failure=str(exc);print('PIPELINE FAILURE:',failure,flush=True)
 finally:
  mark('pipeline_finished');save_timeline('failed' if failure else 'holding_reached')
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
 report=dict(success=bool(hold and min(x['bottle_lift_m'] for x in hold)>.08 and np.mean([x['contacts']==2 for x in hold])>.9),status='success',parameters=json.loads((out/'parameters.json').read_text()),tcp_m=a.tcp_m,video_available=True,selected=selected,grasp_strategy='box_network_direction' if a.target_kind in ('small_carton','tissue_box') else 'upright_cylinder' if a.target_kind in ('bottle','can','cup','cup_inverted') else 'near_sphere_network_direction',pipeline='ROS2_DC1_DepthComponents_GraspNet_MoveIt_TaskExecutor',camera_source='dc1_rgbd',simulation_execution_speed_scale=a.execution_speed_scale,oracle_segmentation=False,target_kind=a.target_kind,place_after_grasp=a.place_after_grasp,placement_success=physical.get('placement_verified') if a.place_after_grasp else None,
  point_count=len(points),graspnet_inference_sec=selected_diagnostic.get('inference_sec'),min_hold_lift_m=min(x['bottle_lift_m'] for x in hold),bilateral_hold_fraction=float(np.mean([x['contacts']==2 for x in hold])),hold_contact_inset_median_m=float(np.median([x['contact_inset_m'] for x in hold if x['contact_inset_m'] is not None])),close_contact_height_from_bottom_m=float(np.median([x['contact_height_from_bottom_m'] for x in json.loads((out/'trace.json').read_text()) if x['phase']=='close' and x['contact_height_from_bottom_m'] is not None])),pad_support_verified=True,physical_attachment=False,object_pose_written_during_rollout=False,execute_robot=False)
 report['pad_support_verified']=report['hold_contact_inset_median_m']>=.008
 report['success']=bool(report['success'] and report['pad_support_verified'])
 report['grasp_success']=bool(report['success'])
 report['success']=bool(report['grasp_success'] and (not a.place_after_grasp or report['placement_success']))
 report['status']='success' if report['success'] else 'placement_failed' if report['grasp_success'] and a.place_after_grasp else 'grasp_failed'
 if report['status']=='placement_failed':report['failure_reason']=placement_failure or '抓取保持成功，但物体未通过松爪后的桌面放置检查'
 report['collision_scene']='robot_self_table_other_objects_and_depth_fitted_target_with_attached_lift' if a.multi_objects else 'robot_self_table_and_depth_fitted_target_with_attached_lift'
 report['multi_objects']=a.multi_objects
 report['selected_dc1_pixel']=[a.selected_pixel_u,a.selected_pixel_v] if a.selected_pixel_u>=0 else None
 if (out/'detected_objects.json').is_file():report['depth_objects']=json.loads((out/'detected_objects.json').read_text())
 report['source_paths']={'perception':'src/perception/perception_node','generator':'src/manipulation/grasp_generation','planner':'src/manipulation/robot_planner','executor':'src/manipulation/task_executor'}
 (out/'metrics.json').write_text(json.dumps(report,indent=2));(out/'proposals.json').write_text(n.diagnostic or '{}')
 world=points@np.array(physical['camera_rotation']).T+physical['camera_translation'];(out/'pointcloud.json').write_text(json.dumps(world[::max(1,len(world)//2200)].tolist()))
 print(json.dumps(report),flush=True)
if __name__=='__main__':main()
