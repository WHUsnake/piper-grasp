"""No ROS nodes or robot commands: maintained planner/executor contracts."""
import copy,sys,unittest
from pathlib import Path
from types import SimpleNamespace as S
import numpy as np
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from moveit_msgs.msg import RobotState
from grasp_msgs.msg import SelectedGrasp
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/manipulation/robot_planner'));sys.path.insert(0,str(ROOT/'src/manipulation/task_executor'))
from robot_planner.full_planning import FullPlanning
from task_executor.simulation_execution import SimulationExecution

class PlannerTests(unittest.TestCase):
 def make(self):
  p=FullPlanning();values=dict(tool_translation=[0.,0.,.095],tool_quaternion=[0.,0.,0.,1.],base_frame='world',planning_group='arm',end_effector_link='p_link6')
  p.get_parameter=lambda k:S(value=values[k]);return p
 def test_tcp_removed_once_and_input_unchanged(self):
  p=self.make();pose=Pose();pose.position.z=.8;pose.orientation.w=1.;out=p.link_target(pose)
  self.assertAlmostEqual(out.position.z,.705);self.assertEqual(pose.position.z,.8)
 def test_trajectory_state_keeps_gripper(self):
  p=self.make();state=RobotState();state.joint_state=JointState(name=['joint1','joint7'],position=[0.,.035]);trajectory=JointTrajectory(joint_names=['joint1'],points=[JointTrajectoryPoint(positions=[.5])]);out=p.after(state,trajectory)
  self.assertEqual(list(out.joint_state.position),[.5,.035]);self.assertEqual(list(state.joint_state.position),[0.,.035])
 def test_partial_cartesian_path_refused(self):
  p=self.make();p.cartesian_client=None;p.rpc=lambda c,r:S(error_code=S(val=1),fraction=.98)
  with self.assertRaisesRegex(ValueError,'fraction'):p.cartesian(RobotState(),Pose())
 def test_empty_trajectory_refused(self):
  with self.assertRaises(ValueError):self.make().after(RobotState(),JointTrajectory())

class SymmetryTests(unittest.TestCase):
 def test_parallel_jaws_preserve_approach_and_tcp(self):
  sys.path.insert(0,str(ROOT/'src/manipulation/grasp_generation'))
  from grasp_generation.refinement import parallel_gripper_symmetries
  from scipy.spatial.transform import Rotation
  c={'position':[.4,0,.8],'quaternion':[0.,0.,0.,1.],'rotation_world':np.eye(3).tolist()}
  items=parallel_gripper_symmetries([c]);r=Rotation.from_quat(items[1]['quaternion']).as_matrix()
  np.testing.assert_allclose(r[:,2],[0,0,1],atol=1e-7)
  np.testing.assert_allclose(r[:,1],[0,-1,0],atol=1e-7)
  self.assertEqual(items[1]['position'],c['position']);self.assertAlmostEqual(np.linalg.det(r),1.)

class ExecutorTests(unittest.TestCase):
 def test_fresh_object_geometry_accepts_network_orientation_jitter(self):
  e=SimulationExecution();e.get_clock=lambda:S(now=lambda:S(nanoseconds=100_000_000_000))
  e.live_geometry={'geometry_frame':'world','acquisition_stamp':[99,800000000],'geometry_fit':{'axis_xy':[.72,0.],'bottom_observed_z_m':.76,'top_observed_z_m':.98,'observed_height_m':.22,'radius_m':.026}}
  c=SelectedGrasp();c.header.frame_id='world';c.observed_object_center.x=.72;c.observed_object_center.z=.87;c.observed_object_radius_m=.026;c.observed_object_height_m=.22
  # Revalidation depends on the object, rather than random proposal angle classes.
  e.validate_target_geometry(c)
  c.observed_object_center.x+=.01
  with self.assertRaisesRegex(ValueError,'moved'):e.validate_target_geometry(c)
 def test_stale_object_geometry_refused(self):
  e=SimulationExecution();e.get_clock=lambda:S(now=lambda:S(nanoseconds=100_000_000_000));e.live_geometry={'geometry_frame':'world','acquisition_stamp':[98,0]}
  c=SelectedGrasp();c.header.frame_id='world'
  with self.assertRaisesRegex(ValueError,'expired'):e.validate_target_geometry(c)
 def test_selected_grasp_not_replaced_during_attempt(self):
  e=SimulationExecution();e.running=True;e.completion_latched=False;e.selected_grasp='original';e.receive_selection(SelectedGrasp());self.assertEqual(e.selected_grasp,'original')
 def test_selected_grasp_not_replaced_after_latch(self):
  e=SimulationExecution();e.running=False;e.completion_latched=True;e.selected_grasp='original';e.receive_selection(SelectedGrasp());self.assertEqual(e.selected_grasp,'original')
 def test_hardware_preview_candidate_is_refused(self):
  e=SimulationExecution();e.get_parameter=lambda k:S(value=True);e.selected_grasp=SelectedGrasp();e.selected_grasp.candidate.generator='graspnet_baseline_preview'
  with self.assertRaisesRegex(ValueError,'preview-only'):e.validate_physical_selection()
 def test_legacy_only_hardware_selection_refused(self):
  e=SimulationExecution();e.get_parameter=lambda k:S(value=True);e.selected_grasp=None
  with self.assertRaisesRegex(ValueError,'SelectedGrasp'):e.validate_physical_selection()

if __name__=='__main__':unittest.main()

class IKSearchTests(unittest.TestCase):
 def request(self):
  from moveit_msgs.srv import GetPositionIK
  r=GetPositionIK.Request();r.ik_request.avoid_collisions=True
  r.ik_request.robot_state.joint_state=JointState(name=[f'joint{i}' for i in range(1,7)],position=[0.]*6)
  return r
 def test_retry_keeps_collision_check_and_original_seed(self):
  p=FullPlanning();p.ik_client=None;seen=[]
  def rpc(client,r):
   seen.append(copy.deepcopy(r));return S(error_code=S(val=1 if len(seen)==2 else -31))
  p.rpc=rpc;r=self.request();record={};p.pregrasp_ik(r,record)
  self.assertTrue(all(x.ik_request.avoid_collisions for x in seen));self.assertEqual(list(r.ik_request.robot_state.joint_state.position),[0.]*6)
  self.assertEqual(record['ik_attempts'],[-31,1])
 def test_collision_free_diagnostic_never_accepted(self):
  p=FullPlanning();p.ik_client=None
  p.rpc=lambda c,r:S(error_code=S(val=-31 if r.ik_request.avoid_collisions else 1))
  record={}
  with self.assertRaisesRegex(ValueError,'collision constraints'):p.pregrasp_ik(self.request(),record)
  self.assertEqual(record['ik_without_collision_code'],1)

class TimingTests(unittest.TestCase):
 def test_time_scaling_preserves_path_and_limits_speed(self):
  sys.path.insert(0,str(ROOT/'simulation/ros_pipeline'))
  from trajectory_timing import eased_path_time
  t=np.linspace(0,2.3,2301);v=np.array([eased_path_time(x,2.) for x in t]);speed=np.diff(v)/.001
  self.assertEqual(v[0],0.);self.assertEqual(v[-1],2.)
  self.assertTrue(np.all(speed>=-1e-10));self.assertLessEqual(max(speed),1.+1e-9)
  self.assertLess(speed[0],.001);self.assertLess(speed[-1],.001)

class UprightSearchTests(unittest.TestCase):
 def test_adapted_hypotheses_keep_contact_center_and_network_evidence(self):
  sys.path.insert(0,str(ROOT/'src/manipulation/grasp_generation'))
  from grasp_generation.refinement import upright_orientation_search
  from scipy.spatial.transform import Rotation
  r=Rotation.from_euler('y',65,degrees=True).as_matrix();centre=np.array([.72,0.,.70])
  original=dict(rotation_world=r.tolist(),quaternion=Rotation.from_matrix(r).as_quat().tolist(),contact_center_world=centre.tolist(),position=(centre-.01*r[:,2]).tolist(),score=.5)
  variants=upright_orientation_search([original],.095)
  self.assertIs(variants[-1],original);self.assertEqual(len(variants),6)
  for c in variants[:-1]:
   rotation=np.array(c['rotation_world']);np.testing.assert_allclose(rotation.T@rotation,np.eye(3),atol=1e-12)
   self.assertAlmostEqual(np.linalg.det(rotation),1.)
   np.testing.assert_allclose(np.array(c['position'])+.01*rotation[:,2],centre)
   self.assertAlmostEqual(rotation[2,1],0.)
   self.assertEqual(c['orientation_adaptation'],'upright_cylinder_network_and_radial_search')
   np.testing.assert_allclose(c['network_rotation_world'],r)

class DiversityTests(unittest.TestCase):
 def test_direct_poses_precede_equivalent_flips(self):
  from grasp_generation.refinement import parallel_gripper_symmetries
  c=dict(position=[0.,0.,0.],quaternion=[0.,0.,0.,1.],rotation_world=np.eye(3).tolist())
  d=dict(c,position=[.1,0.,0.]);items=parallel_gripper_symmetries([c,d])
  self.assertIs(items[0],c);self.assertIs(items[1],d)
  self.assertTrue(all(x.get('parallel_gripper_symmetry') for x in items[2:]))
 def test_duplicate_parents_do_not_consume_direction_budget(self):
  from grasp_generation.refinement import upright_orientation_search
  from scipy.spatial.transform import Rotation
  r=Rotation.from_euler('y',65,degrees=True).as_matrix();centre=np.array([.72,.10,.70])
  c=dict(rotation_world=r.tolist(),quaternion=Rotation.from_matrix(r).as_quat().tolist(),contact_center_world=centre.tolist(),position=(centre-.01*r[:,2]).tolist(),score=.5)
  one=upright_orientation_search([c],.095,[.09,0.,.55]);many=upright_orientation_search([c]*5,.095,[.09,0.,.55])
  self.assertEqual(len(one),len(many));self.assertGreater(len(one),6)
  np.testing.assert_allclose(np.array(one[0]['rotation_world'])[:2,2],np.array([.63,.10])/np.linalg.norm([.63,.10]))

class CartesianGuardTests(unittest.TestCase):
 def test_full_fraction_does_not_override_absolute_jump_guard(self):
  p=PlannerTests().make();p.cartesian_client=None
  trajectory=JointTrajectory(joint_names=['joint1'],points=[JointTrajectoryPoint(positions=[0.]),JointTrajectoryPoint(positions=[.4])])
  p.rpc=lambda c,r:S(error_code=S(val=1),fraction=1.,solution=S(joint_trajectory=trajectory))
  with self.assertRaisesRegex(ValueError,'absolute'):p.cartesian(RobotState(),Pose())

class CartesianFKTests(unittest.TestCase):
 def test_dense_joint_path_with_wrong_tool_orientation_is_rejected(self):
  p=PlannerTests().make();p.fk_client=object();p.validity_client=object();calls=[]
  def rpc(client,request):
   pose=Pose();pose.orientation.w=1.
   if calls:pose.orientation.w=0.;pose.orientation.z=1.
   calls.append(request)
   return S(error_code=S(val=1),pose_stamped=[S(pose=pose)])
  p.rpc=rpc;goal=Pose();goal.orientation.w=1.
  trajectory=JointTrajectory(joint_names=['joint1'],points=[JointTrajectoryPoint(positions=[0.])])
  with self.assertRaisesRegex(ValueError,'geometry invalid'):p.validate_cartesian_geometry(RobotState(),trajectory,goal)

class HeightSearchTests(unittest.TestCase):
 def test_height_variants_recompute_tcp_and_keep_original(self):
  from grasp_generation.refinement import upright_orientation_search
  from scipy.spatial.transform import Rotation
  r=Rotation.from_euler('y',65,degrees=True).as_matrix();centre=np.array([.72,0.,.71])
  c=dict(rotation_world=r.tolist(),quaternion=Rotation.from_matrix(r).as_quat().tolist(),contact_center_world=centre.tolist(),position=(centre-.01*r[:,2]).tolist(),score=.5,body_bottom_z_m=.6,body_span_m=.22,geometry_opening_m=.064)
  items=upright_orientation_search([c],.095,[.09,0.,.55],[.5,.4,.6]);fractions=set()
  for item in items:
   if not item.get('orientation_adaptation'):continue
   fractions.add(item['height_fraction']);rot=np.array(item['rotation_world'])
   contact=np.array(item['position'])+.01*rot[:,2]
   self.assertAlmostEqual(contact[2],.6+.22*item['height_fraction'])
   self.assertEqual(item['width'],.064)
  self.assertEqual(fractions,{.4,.5,.6});self.assertEqual(c['contact_center_world'],centre.tolist())

class SearchBudgetTests(unittest.TestCase):
 def test_short_approach_is_not_starved_by_many_primary_candidates(self):
  parents=list(enumerate(range(32)));attempts=FullPlanning.candidate_attempts(parents,[0.,.06,.14])
  before_fifth=attempts[:next(i for i,x in enumerate(attempts) if x[0]==4)]
  self.assertTrue(any(i==0 and distance==.06 for i,c,distance in before_fifth))
  self.assertTrue(any(i==0 and distance==.14 for i,c,distance in before_fifth))
  self.assertEqual(len(attempts),96)
