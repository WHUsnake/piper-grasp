"""Produce a MoveIt model from the very same source MJCF used by MuJoCo."""
import argparse,json,sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parent;PROJECT=ROOT.parents[1];SHE=PROJECT.parent
sys.path.insert(0,str(PROJECT/'simulation/a2_table_grasp'))
from simulate import build
import mujoco

def text(v):return ' '.join(map(str,v))
def mesh_stl(m,i,path):
    v=m.mesh_vert[m.mesh_vertadr[i]:m.mesh_vertadr[i]+m.mesh_vertnum[i]]
    f=m.mesh_face[m.mesh_faceadr[i]:m.mesh_faceadr[i]+m.mesh_facenum[i]]
    triangles=v[f];normals=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]);normals/=np.maximum(np.linalg.norm(normals,axis=1)[:,None],1e-20)
    dt=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attr','<u2')]);rows=np.zeros(len(f),dtype=dt);rows['normal']=normals;rows['vertices']=triangles
    path.write_bytes(b'A2 MuJoCo to URDF collision mesh'.ljust(80,b'\0')+np.array([len(f)],dtype='<u4').tobytes()+rows.tobytes())

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--bottle-x',type=float,default=.72);p.add_argument('--bottle-y',type=float,default=0);p.add_argument('--table-z',type=float,default=.76);p.add_argument('--tcp-m',type=float,default=.095);p.add_argument('--target-kind',choices=['bottle','round_proxy','apple','orange','egg','can','cup','small_carton','tissue_box','cup_inverted'],default='bottle');p.add_argument('--multi-objects',action='store_true');a=p.parse_args();out=a.output.resolve();out.mkdir(parents=True,exist_ok=True)
 m=build(out,a.bottle_x,a.table_z,a.tcp_m,a.bottle_y,a.target_kind,a.multi_objects);d=mujoco.MjData(m);d.qpos[6:8]=[.035,-.035];mujoco.mj_forward(m,d)
 robot=ET.Element('robot',name='a2_piper_sim');ET.SubElement(robot,'link',name='world');links={0:'world'};parents={};meshes=out/'meshes';meshes.mkdir(exist_ok=True)
 for i in range(1,m.nbody):
  if m.body(i).name in ('bottle','round_target') or m.body(i).name.startswith('demo_'):continue
  name=m.body(i).name;links[i]=name;link=ET.SubElement(robot,'link',name=name)
  for gi in range(m.ngeom):
   if m.geom_bodyid[gi]!=i or m.geom_group[gi]!=3:continue
   c=ET.SubElement(link,'collision');q=m.geom_quat[gi][[1,2,3,0]]
   ET.SubElement(c,'origin',xyz=text(m.geom_pos[gi]),rpy=text(Rotation.from_quat(q).as_euler('xyz')));g=ET.SubElement(c,'geometry');typ=int(m.geom_type[gi]);size=m.geom_size[gi]
   if typ==7:
    mid=int(m.geom_dataid[gi]);path=meshes/f'{mid}.stl'
    if not path.exists():mesh_stl(m,mid,path)
    ET.SubElement(g,'mesh',filename='file://'+str(path))
   elif typ==6:ET.SubElement(g,'box',size=text(2*size))
   elif typ==2:ET.SubElement(g,'sphere',radius=str(size[0]))
   elif typ==5:ET.SubElement(g,'cylinder',radius=str(size[0]),length=str(2*size[1]))
   else:link.remove(c)
  parent=links[int(m.body_parentid[i])];parents[name]=parent
  nj=int(m.body_jntnum[i]);jid=int(m.body_jntadr[i])
  active=nj and m.jnt_type[jid]!=int(mujoco.mjtJoint.mjJNT_FREE)
  jname=m.joint(jid).name.replace('p_joint','joint') if active else name+'_fixed'
  j=ET.SubElement(robot,'joint',name=jname,type=('prismatic' if m.jnt_type[jid]==int(mujoco.mjtJoint.mjJNT_SLIDE) else 'revolute') if active else 'fixed')
  ET.SubElement(j,'parent',link=parent);ET.SubElement(j,'child',link=name)
  ET.SubElement(j,'origin',xyz=text(m.body_pos[i]),rpy=text(Rotation.from_quat(m.body_quat[i][[1,2,3,0]]).as_euler('xyz')))
  if active:
   assert np.linalg.norm(m.jnt_pos[jid])<1e-8
   ET.SubElement(j,'axis',xyz=text(m.jnt_axis[jid]));ET.SubElement(j,'limit',lower=str(m.jnt_range[jid,0]),upper=str(m.jnt_range[jid,1]),effort='100',velocity='1.2')
 srdf=ET.Element('robot',name='a2_piper_sim');group=ET.SubElement(srdf,'group',name='arm');ET.SubElement(group,'chain',base_link='p_base_link',tip_link='p_link6')
 for child,parent in parents.items():ET.SubElement(srdf,'disable_collisions',link1=parent,link2=child,reason='Adjacent')
 # Rigid body pairs connected only through fixed links cannot move relative to each other.
 def active_ancestor(n):
  while n!='world':
   b=m.body(n).id
   if m.body_jntnum[b]:return n
   n=parents[n]
  return 'world'
 names=list(parents)
 for i,n in enumerate(names):
  for o in names[i+1:]:
   if active_ancestor(n)==active_ancestor(o):ET.SubElement(srdf,'disable_collisions',link1=n,link2=o,reason='RigidAssembly')
 urdf=ET.tostring(robot,encoding='unicode');semantic=ET.tostring(srdf,encoding='unicode')
 (out/'robot.urdf').write_text(urdf);(out/'robot.srdf').write_text(semantic)
 config=dict(robot_description=urdf,robot_description_semantic=semantic,
  robot_description_kinematics={'arm':{'kinematics_solver':'kdl_kinematics_plugin/KDLKinematicsPlugin','kinematics_solver_timeout':.2,'kinematics_solver_search_resolution':.005}},
  planning_pipelines=['ompl'],default_planning_pipeline='ompl',
  ompl={'planning_plugin':'ompl_interface/OMPLPlanner','request_adapters':'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints',
        'start_state_max_bounds_error':.1,'planner_configs':{'RRTConnectkConfigDefault':{'type':'geometric::RRTConnect','range':0.}},'arm':{'planner_configs':['RRTConnectkConfigDefault'],'longest_valid_segment_fraction':.01}},
  allow_trajectory_execution=False,publish_robot_description_semantic=True,
  moveit_controller_manager='moveit_simple_controller_manager/MoveItSimpleControllerManager',
  moveit_simple_controller_manager={'controller_names':['sim_arm'],'sim_arm':{'type':'FollowJointTrajectory','action_ns':'follow_joint_trajectory','default':True,'joints':[f'joint{i}' for i in range(1,7)]}},
  robot_description_planning={'joint_limits':{f'joint{i}':{'has_velocity_limits':True,'max_velocity':1.2,'has_acceleration_limits':True,'max_acceleration':1.0} for i in range(1,7)}},
  publish_planning_scene=True,publish_geometry_updates=True,publish_state_updates=True,publish_transforms_updates=True)
 (out/'moveit.json').write_text(json.dumps(config))
 base=json.loads((PROJECT/'src/manipulation/grasp_generation/config/graspnet.json').read_text());base['adapter']['tcp_from_tip_m']=.135-a.tcp_m;base['adapter']['max_candidates']=100
 if a.target_kind in ('bottle','can','cup','cup_inverted'):base['refinement']={'enabled':True,'shape':'upright_cylinder','tcp_m':a.tcp_m,'orientation_search':True,'contact_height_fractions':[.5,.4,.6],'approach_origin_world':d.body('p_base_link').xpos.tolist()}
 elif a.target_kind in ('small_carton','tissue_box'):
  base['refinement']={'enabled':True,'shape':'box','tcp_m':a.tcp_m,'size_m':{'small_carton':[.044,.044,.044],'tissue_box':[.084,.044,.064]}[a.target_kind],'orientation_search':True,'approach_origin_world':d.body('p_base_link').xpos.tolist(),'support_surface_z':a.table_z}
  base['adapter']['horizontal_tolerance_deg']=90.;base['adapter']['closing_tolerance_deg']=90.
 else:
  base['refinement']={'enabled':True,'shape':'near_sphere','tcp_m':a.tcp_m,'orientation_search':True,'approach_origin_world':d.body('p_base_link').xpos.tolist(),'support_surface_z':a.table_z}
  base['adapter']['horizontal_tolerance_deg']=90.;base['adapter']['closing_tolerance_deg']=90.
 base['base_frame']='world';(out/'graspnet.json').write_text(json.dumps(base,indent=2))
 cam=m.camera('dc1_rgbd').id;rotation=d.cam_xmat[cam].reshape(3,3)@np.diag([1,-1,-1])
 from simulate import target_xy
 target_x,target_y=target_xy(a.bottle_x,a.bottle_y,a.target_kind,a.multi_objects)
 (out/'parameters.json').write_text(json.dumps(dict(bottle_x=a.bottle_x,bottle_y=a.bottle_y,target_x=target_x,target_y=target_y,table_z=a.table_z,tcp_m=a.tcp_m,target_kind=a.target_kind,multi_objects=a.multi_objects,
  target_half_height_m={'bottle':.105,'round_proxy':.028,'apple':.029,'orange':.027,'egg':.020,'can':.055,'cup':.045,'cup_inverted':.045,'small_carton':.022,'tissue_box':.032}[a.target_kind],
  camera_rotation_world=rotation.reshape(-1).tolist(),camera_translation_world=d.cam_xpos[cam].tolist(),camera_source='dc1_rgbd')))
 print(json.dumps({'model':str(out/'robot.urdf'),'nq':m.nq,'robot_bodies':len(links),'session':out.name}))
if __name__=='__main__':main()
