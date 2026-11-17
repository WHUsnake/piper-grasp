"""Run the user's maintained ROS grasp nodes against the live Go2 world only."""
import base64
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation

from integration.rpc import call


def executor_command(command, setup=None):
    if not setup:
        return command
    setup = Path(setup)
    if not setup.is_file():
        raise RuntimeError('Grasp message overlay missing; run bin/lab grasp prepare')
    return ['bash', '-c', 'set -e; source "$1"; shift; exec "$@"',
            'go2-grasp-executor', str(setup), *command]


def camera_info(frame, header):
    from sensor_msgs.msg import CameraInfo
    intrinsics = np.asarray(frame['intrinsics'], dtype=np.float64)
    if intrinsics.shape != (9,) or not np.isfinite(intrinsics).all():
        raise ValueError('Camera intrinsics require nine finite numbers')
    if intrinsics[0] <= 0 or intrinsics[4] <= 0 or not np.allclose(intrinsics[6:], [0., 0., 1.]):
        raise ValueError('Invalid camera focal lengths or homogeneous row')
    info = CameraInfo()
    info.header = header
    info.width, info.height = frame['width'], frame['height']
    info.k = [float(value) for value in intrinsics]
    info.distortion_model, info.d = 'plumb_bob', [0.]*5
    return info


class KinematicSeed:
    def __init__(self, description):
        import xml.etree.ElementTree as ET
        robot = ET.fromstring(description)
        joints = {joint.find('child').get('link'): joint for joint in robot.findall('joint')}
        chain, child = [], 'link6'
        while child in joints:
            joint = joints[child]
            chain.append(joint)
            child = joint.find('parent').get('link')
        self.chain, bounds = [], {}
        for joint in reversed(chain):
            origin = joint.find('origin')
            transform = np.eye(4)
            if origin is not None:
                transform[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
                transform[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
            name = joint.get('name')
            index, axis = None, None
            if joint.get('type') != 'fixed':
                if name not in ['joint'+str(number) for number in range(1, 7)] or joint.get('type') != 'revolute':
                    raise ValueError('Unsupported planning chain')
                index = int(name[5:])-1
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                limit = joint.find('limit')
                bounds[index] = [float(limit.get('lower')), float(limit.get('upper'))]
            self.chain.append((transform, index, axis))
        if set(bounds) != set(range(6)):
            raise ValueError('Planning chain must contain the six Piper joints')
        self.bounds = np.array([bounds[index] for index in range(6)])

    def forward(self, positions):
        result = np.eye(4)
        for transform, index, axis in self.chain:
            result = result@transform
            if index is not None:
                motion = np.eye(4)
                motion[:3, :3] = Rotation.from_rotvec(axis*positions[index]).as_matrix()
                result = result@motion
        return result

    def solve(self, target, measured):
        from scipy.optimize import least_squares
        def residual(positions):
            actual = self.forward(positions)
            return np.r_[actual[:3, 3]-target[:3, 3],
                         .2*Rotation.from_matrix(target[:3, :3].T@actual[:3, :3]).as_rotvec()]
        starts = [measured, self.bounds.mean(axis=1)]
        starts.extend(np.random.default_rng(7).uniform(self.bounds[:, 0], self.bounds[:, 1], size=(6, 6)))
        accepted = []
        for start in starts:
            solution = least_squares(residual, np.clip(start, self.bounds[:, 0]+1e-7, self.bounds[:, 1]-1e-7),
                                     bounds=(self.bounds[:, 0], self.bounds[:, 1]), max_nfev=100)
            error = residual(solution.x)
            if np.linalg.norm(error[:3]) > .0005 or np.linalg.norm(error[3:])/.2 > .003:
                continue
            if any(np.linalg.norm(solution.x-previous) < .05 for previous in accepted):
                continue
            accepted.append(solution.x)
            yield solution.x


def run_planner(description, arguments):
    import copy
    import rclpy
    from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
    from robot_planner.node import RobotPlanner
    seed_solver = KinematicSeed(Path(description).read_text())

    class Planner(RobotPlanner):
        def wait(self, future, timeout):
            return super().wait(future, max(timeout, 45.))

        def pregrasp_ik(self, request, record):
            if request.ik_request.pose_stamped.header.frame_id != 'world':
                raise ValueError('Seed solver requires the exported world frame')
            pose = request.ik_request.pose_stamped.pose
            target = np.eye(4)
            target[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
            target[:3, :3] = Rotation.from_quat([pose.orientation.x, pose.orientation.y,
                                                pose.orientation.z, pose.orientation.w]).as_matrix()
            joints = request.ik_request.robot_state.joint_state
            indices = [joints.name.index('joint'+str(index)) for index in range(1, 7)]
            attempts = []
            for seed in seed_solver.solve(target, np.asarray(joints.position)[indices]):
                trial = copy.deepcopy(request)
                trial.ik_request.avoid_collisions = True
                for index, value in zip(indices, seed):
                    trial.ik_request.robot_state.joint_state.position[index] = float(value)
                result = self.rpc(self.ik_client, trial)
                attempts.append(int(result.error_code.val))
                record['model_seeded_ik_attempts'] = attempts
                if result.error_code.val == 1:
                    record['model_seed'] = seed.tolist()
                    return result
            return super().pregrasp_ik(request, record)

    rclpy.init(args=arguments)
    node = Planner()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    request = json.loads(Path(sys.argv[1]).read_text())
    directory = Path(request['directory'])
    owner = directory.name
    session = request['world_session']
    socket = request['socket']
    processes, logs = [], []
    reserved = False
    execution_started = False
    successful_hold = False
    node = executor = None

    def rpc(method, **values):
        return call(socket, method, owner=owner, session=session, **values)

    def write(name, value):
        (directory/name).write_text(json.dumps(value, indent=2, allow_nan=False))

    def interrupted(*unused):
        raise InterruptedError('Grasp interrupted; no automatic gripper release')

    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)

    def spawn(name, command):
        output = (directory/(name+'.log')).open('w')
        logs.append(output)
        process = subprocess.Popen(list(map(str, command)), stdout=output, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        processes.append((name, process))

    try:
        if request['operation'] in ('preview', 'run'):
            check = ['/usr/bin/python3', '-c',
                     'from task_executor.node import TaskExecutor; from piper_msgs.msg import PosCmd; '
                     'from rclpy.serialization import serialize_message; serialize_message(PosCmd())']
            with (directory/'executor_preflight.log').open('w') as output:
                checked = subprocess.run(executor_command(check, request.get('interfaces_setup')),
                    stdout=output, stderr=subprocess.STDOUT, timeout=20)
            if checked.returncode:
                raise RuntimeError('Executor dependency check failed; run bin/lab grasp prepare; see '+
                                   str(directory/'executor_preflight.log'))
        # A just-finished observation pose can leave a brief settling motion.
        # Wait only for this pre-motion condition; never retry executed motion.
        reserve_deadline = time.monotonic()+5.
        while True:
            try:
                state = rpc('reserve', stabilize=True)
                break
            except RuntimeError as error:
                if 'wait for measured stability' not in str(error) or time.monotonic() >= reserve_deadline:
                    raise
                time.sleep(.1)
        reserved = True
        if state['session'] != session or not state['simulated']:
            raise RuntimeError('World identity changed; no operation permitted')
        deadline = time.monotonic()+20
        while True:
            state = rpc('planning_snapshot')
            if state['snapshot_ready']:
                break
            if time.monotonic() >= deadline:
                raise RuntimeError('Base did not settle within 20 seconds; no planning snapshot accepted')
            time.sleep(.1)
        write('planning_snapshot.json', state)
        frame = rpc('capture', camera=request['camera'])
        expected = request.get('expected_camera')
        if expected:
            moved = np.linalg.norm(np.asarray(frame['translation'])-expected['translation'])
            rotated = Rotation.from_matrix(np.asarray(frame['rotation'])@np.asarray(expected['rotation']).T).magnitude()
            if expected['session'] != frame['session'] or time.time()-expected['stamp'] > 60 or moved > .02 or rotated > .04:
                raise RuntimeError('Selected RGB-D image expired or camera moved; capture and select the target again')
        write('capture.json', {key: value for key, value in frame.items() if key not in ('rgb', 'depth', 'self_mask')})
        from PIL import Image as PillowImage
        rgb = np.frombuffer(base64.b64decode(frame['rgb']), dtype=np.uint8).reshape(480, 640, 3)
        depth = np.frombuffer(base64.b64decode(frame['depth']), dtype='<f4').reshape(480, 640)
        self_mask = np.frombuffer(base64.b64decode(frame['self_mask']), dtype=np.uint8).reshape(480, 640).astype(bool)
        if request['pixel'] and self_mask[request['pixel'][1], request['pixel'][0]]:
            raise RuntimeError('Selected pixel belongs to the robot; move to observation pose and select visible target')
        if expected:
            selected_depth = float(depth[request['pixel'][1], request['pixel'][0]])
            if not np.isfinite(selected_depth) or abs(selected_depth-expected['depth_m']) > .03:
                raise RuntimeError('Target depth changed since selection; capture and select the target again')
        PillowImage.fromarray(rgb).save(directory/'camera_rgb.png')
        PillowImage.fromarray((self_mask*255).astype('uint8')).save(directory/'robot-self-mask.png')
        depth = depth.copy()
        depth[self_mask] = np.nan
        np.save(directory/'depth.npy', depth)
        if request['operation'] == 'capture':
            write('result.json', {'passed': True, 'operation': 'capture', 'motion_executed': False})
            print('Live RGB-D captured: '+str(directory/'camera_rgb.png'))
            return 0

        from perception_node.depth_target import tabletop_component
        from integration.paths import REPOSITORY
        profile = json.loads((REPOSITORY/'profiles/hssd-go2-piper.json').read_text())
        from integration.go2_grasp import grasp_support
        support_context = (grasp_support(frame, request['pixel'], profile) if request['pixel'] else
                           profile['placed_bottle'])
        request['table_height'] = support_context['tabletop_height_m']
        table_bounds = support_context['table_bounds_world']
        write('support-context.json', support_context)
        mask, segmentation = tabletop_component(depth, frame['intrinsics'], frame['rotation'], frame['translation'],
                                                 request['table_height'], selected_pixel=request['pixel'],
                                                 table_bounds_world=table_bounds)
        segmentation['table_bounds_world'] = table_bounds
        PillowImage.fromarray((mask*255).astype('uint8')).save(directory/'mask.png')
        write('segmentation.json', segmentation)
        if segmentation.get('selected_pixels', 0) < 100:
            raise RuntimeError('No unambiguous depth target in live camera. Move within reach and select a target pixel; '+json.dumps(segmentation))
        ys, xs = np.nonzero(mask & np.isfinite(depth))
        values = depth[ys, xs]
        fx, _, cx, _, fy, cy, *_ = frame['intrinsics']
        cloud = np.stack(((xs-cx)*values/fx, (ys-cy)*values/fy, values), axis=1)
        observed = cloud @ np.asarray(frame['rotation']).T + np.asarray(frame['translation'])
        observed_height = float(np.percentile(observed[:, 2], 99.5)-np.percentile(observed[:, 2], .5))
        if observed_height < .06:
            raise RuntimeError('Selected RGB-D target shows only %.1f cm of height; bottle grasp requires at least 6 cm. Select a more visible bottle body or change the observation pose' % (observed_height*100))
        object_bottom = [float(segmentation['centroid_world_m'][0]),
                         float(segmentation['centroid_world_m'][1]),
                         float(np.percentile(observed[:, 2], 2))]
        # The visible cylinder component ends ABOVE its actual base because
        # segmentation removes points close to the table. Infer the base from
        # the adjacent observed support, not that truncated percentile or the
        # simulated object's true position.
        yy, xx = np.mgrid[0:480:3, 0:640:3]
        zz = depth[yy, xx]
        valid = np.isfinite(zz) & (zz > .1) & (zz < 2.5)
        surrounding = np.stack(((xx[valid]-cx)*zz[valid]/fx,
                                (yy[valid]-cy)*zz[valid]/fy, zz[valid]), axis=1)
        surrounding = surrounding @ np.asarray(frame['rotation']).T+np.asarray(frame['translation'])
        distance = np.linalg.norm(surrounding[:, :2]-np.asarray(object_bottom[:2]), axis=1)
        support = surrounding[(distance > .05) & (distance < .15) &
            (surrounding[:, 2] > .25) & (surrounding[:, 2] < object_bottom[2]+.003)]
        if len(support) < 100:
            raise RuntimeError('Object base is occluded and adjacent wrist depth cannot measure its support plane')
        # The lower bottle may itself be outside the image. Find the dominant
        # horizontal height, rather than assuming its visible bottom is within
        # 5 cm of the table or taking the median of nearby bottle surfaces.
        counts, edges = np.histogram(support[:, 2], bins=np.arange(.25, .855, .003))
        peak = int(np.argmax(counts))
        height = (edges[peak]+edges[peak+1])/2
        support = support[abs(support[:, 2]-height) < .004]
        if len(support) < 100:
            raise RuntimeError('Insufficient observed support for held-object base geometry')
        if np.min(np.ptp(support[:, :2], axis=0)) < .05:
            raise RuntimeError('Observed support is too narrow to estimate the object base reliably')
        center = np.median(support, axis=0)
        _, _, axes = np.linalg.svd(support-center, full_matrices=False)
        normal = axes[-1] if axes[-1, 2] > 0 else -axes[-1]
        residual = float(np.percentile(abs((support-center) @ normal), 90))
        if normal[2] < .985 or residual > .004:
            raise RuntimeError('Observed object support is not a reliable horizontal plane')
        object_bottom[2] = float(center[2]-np.dot(normal[:2], np.asarray(object_bottom[:2])-center[:2])/normal[2])
        exported = rpc('export', directory=str(directory), selected_world=segmentation['centroid_world_m'])
        write('target_identity.json', {'body': exported['target_body'],
                                       'centroid_world_m': segmentation['centroid_world_m']})
        config = {'robot_description': exported['robot_description'],
                  'robot_description_semantic': exported['robot_description_semantic'],
                  'robot_description_kinematics': {'arm': {'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
                      'kinematics_solver_timeout': .2, 'kinematics_solver_search_resolution': .005}},
                  'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl',
                  'ompl': {'planning_plugin': 'ompl_interface/OMPLPlanner',
                      'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints',
                      'start_state_max_bounds_error': .1,
                      'planner_configs': {'RRTConnectkConfigDefault': {'type': 'geometric::RRTConnect', 'range': 0.}},
                      'arm': {'planner_configs': ['RRTConnectkConfigDefault'], 'longest_valid_segment_fraction': .01}},
                  'allow_trajectory_execution': False, 'publish_robot_description_semantic': True,
                  'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
                  'moveit_simple_controller_manager': {
                      'controller_names': ['sim/arm_controller'],
                      'sim/arm_controller': {'type': 'FollowJointTrajectory',
                          'action_ns': 'follow_joint_trajectory', 'default': True,
                          'joints': ['joint'+str(index) for index in range(1, 7)]}},
                  'robot_description_planning': {'joint_limits': {'joint'+str(index): {
                      'has_velocity_limits': True, 'max_velocity': 1.2,
                      'has_acceleration_limits': True, 'max_acceleration': 1.} for index in range(1, 7)}}}
        import yaml
        (directory/'moveit.yaml').write_text(yaml.safe_dump({'/**': {'ros__parameters': config}}))
        grasp = json.loads((Path(request['source'])/'src/manipulation/grasp_generation/config/graspnet.json').read_text())
        grasp['worker_python'] = request['worker_python']
        grasp['model'].update(repository=request['baseline'], checkpoint=request['checkpoint'])
        grasp['base_frame'] = 'world'
        grasp['adapter'].update(tcp_from_tip_m=.135-request['tcp_m'], max_candidates=100)
        grasp['refinement'] = {'enabled': True, 'shape': 'upright_cylinder', 'tcp_m': request['tcp_m'],
                               'orientation_search': True, 'approach_origin_world': exported['arm_origin'],
                               'contact_height_fractions': [.5, .4, .6]}
        write('graspnet.json', grasp)

        import rclpy
        from rclpy.action import ActionServer, CancelResponse, GoalResponse
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from control_msgs.action import FollowJointTrajectory
        from geometry_msgs.msg import PoseStamped, TransformStamped
        from grasp_msgs.msg import GraspCandidateArray, SelectedGrasp
        from sensor_msgs.msg import CameraInfo, Image, JointState
        from std_msgs.msg import Float64, String
        from std_srvs.srv import Trigger
        from tf2_ros import TransformBroadcaster
        from rosidl_runtime_py.convert import message_to_ordereddict

        class Bridge(Node):
            def __init__(self):
                super().__init__('go2_piper_grasp_bridge')
                self.selected = None
                self.state = ''
                self.candidates = None
                self.had_candidates = False
                self.error = None
                self.closed = threading.Event()
                self.group = ReentrantCallbackGroup()
                self.joints = self.create_publisher(JointState, '/joint_states', 10)
                self.feedback = self.create_publisher(JointState, '/joint_states_feedback', 10)
                self.pose = self.create_publisher(PoseStamped, '/end_pose_stamped', 10)
                self.rgb = self.create_publisher(Image, '/camera/color/image_raw', 2)
                self.depth = self.create_publisher(Image, '/camera/depth/image_raw', 2)
                self.info = self.create_publisher(CameraInfo, '/camera/color/camera_info', 2)
                self.transforms = TransformBroadcaster(self)
                self.create_subscription(SelectedGrasp, '/planner/selected_grasp', self.selection, 2)
                self.create_subscription(GraspCandidateArray, '/grasp/candidates', self.candidate, 2)
                self.create_subscription(String, '/grasp/diagnostics', self.grasp_diagnostic, 10)
                self.create_subscription(String, '/task/status', lambda message: setattr(self, 'state', message.data), 10)
                self.motion_lock = threading.RLock()
                self.pending_gripper = None
                self.grasp_anchor = None
                self.create_subscription(Float64, '/sim/gripper/command', self.gripper, 2)
                self.create_subscription(String, '/planner/diagnostics', lambda message: (directory/'planner_diagnostics.json').write_text(message.data), 2)
                self.create_service(Trigger, '/sim/ready', self.ready)
                self.action = ActionServer(self, FollowJointTrajectory, '/sim/arm_controller/follow_joint_trajectory',
                    execute_callback=self.execute, callback_group=self.group,
                    goal_callback=lambda goal: GoalResponse.ACCEPT if request['operation'] == 'run' and self.selected else GoalResponse.REJECT,
                    cancel_callback=lambda goal: CancelResponse.ACCEPT)
                self.create_timer(.03, self.publish_feedback)
                self.camera_thread = threading.Thread(target=self.camera, daemon=True)
                self.camera_thread.start()

            def ready(self, unused, response):
                current = rpc('feedback')
                response.success = current['session'] == session and current['owner'] == owner and not current['fault']
                response.message = owner if response.success else 'World identity/fault mismatch'
                return response

            def candidate(self, message):
                self.candidates = message
                self.had_candidates = self.had_candidates or bool(message.candidates)

            def grasp_diagnostic(self, message):
                try:
                    diagnostic = json.loads(message.data)
                    write('grasp_diagnostics.json', diagnostic)
                    status = diagnostic.get('status')
                    # TF delivery is asynchronous across DDS topics. A failed
                    # lookup invalidates that cloud, not the entire task; the
                    # bounded candidate wait below will accept a fresh frame.
                    if isinstance(status, str) and ('extrapolation' in status.lower() or
                            'does not exist' in status.lower()):
                        return
                    if (status not in ('ready', 'target_invalidated', 'published_candidates_expired') and
                            self.selected is None and not self.had_candidates):
                        self.error = 'GraspNet candidate generation failed: '+str(status)
                except (ValueError, TypeError) as error:
                    self.error = 'Invalid GraspNet diagnostics: '+str(error)

            def selection(self, message):
                if message.session_id != owner:
                    self.error = 'Selected grasp session mismatch'
                    return
                self.selected = message
                write('selected_grasp.json', message_to_ordereddict(message))

            def publish_feedback(self):
                try:
                    current = rpc('feedback')
                    if current['fault']:
                        raise RuntimeError(current['fault'])
                    stamp = self.get_clock().now().to_msg()
                    message = JointState()
                    message.header.stamp = stamp
                    message.name = ['joint'+str(index) for index in range(1, 9)]+['gripper']
                    message.position = current['joints']+current['jaw_positions']+[current['gripper_width']]
                    message.effort = [0.]*6+[current['contact_force'], 0., 0.]
                    self.feedback.publish(message)
                    message.name, message.position, message.effort = message.name[:8], message.position[:8], message.effort[:8]
                    self.joints.publish(message)
                    pose = PoseStamped()
                    pose.header.frame_id, pose.header.stamp = 'world', stamp
                    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = current['link6_position']
                    quaternion = Rotation.from_matrix(current['link6_rotation']).as_quat()
                    pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = map(float, quaternion)
                    self.pose.publish(pose)
                except Exception as error:
                    self.error = str(error)

            def camera(self):
                while not self.closed.is_set():
                    try:
                        current = rpc('capture', camera=request['camera'])
                        stamp = rclpy.time.Time(seconds=current['stamp']).to_msg()
                        transform = TransformStamped()
                        transform.header.frame_id, transform.header.stamp = 'world', stamp
                        transform.child_frame_id = 'camera_color_optical_frame'
                        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = current['translation']
                        quaternion = Rotation.from_matrix(current['rotation']).as_quat()
                        transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = map(float, quaternion)
                        self.transforms.sendTransform(transform)
                        # Give the TF subscriber a turn before sending this
                        # frame. Preserve acquisition time, reject a cloud if
                        # its exact transform is still unavailable, and retry
                        # with the next measured frame (see diagnostics above).
                        if self.closed.wait(.025):
                            return
                        for key, encoding, channels, publisher in [('rgb', 'rgb8', 3, self.rgb), ('depth', '32FC1', 4, self.depth)]:
                            message = Image()
                            message.header.frame_id, message.header.stamp = 'camera_color_optical_frame', stamp
                            message.width, message.height, message.step = 640, 480, 640*channels
                            message.encoding, message.data = encoding, base64.b64decode(current[key])
                            publisher.publish(message)
                        self.info.publish(camera_info(current, message.header))
                    except Exception as error:
                        self.error = str(error)
                        return
                    self.closed.wait(.05)

            def gripper(self, message):
                if request['operation'] != 'run' or self.selected is None:
                    self.error = 'Unplanned gripper command rejected'
                    return
                try:
                    width = float(message.data)
                    if width == 0.:
                        radius = float(self.selected.observed_object_radius_m)
                        if not np.isfinite(radius) or not .008 <= radius <= .04:
                            raise RuntimeError('Observed bottle radius is invalid for controlled closure')
                        width = max(0., 2*radius-.005)
                    with self.motion_lock:
                        current = rpc('feedback')
                        if (current.get('trajectory') or {}).get('state') == 'running':
                            self.pending_gripper = width
                        else:
                            rpc('gripper', width=width)
                except Exception as error:
                    self.error = str(error)

            def flush_gripper(self):
                with self.motion_lock:
                    if self.pending_gripper is not None:
                        rpc('gripper', width=self.pending_gripper)
                        self.pending_gripper = None

            def execute(self, handle):
                result = FollowJointTrajectory.Result()
                trajectory = handle.request.trajectory
                try:
                    paths = [self.selected.pregrasp_trajectory, self.selected.approach_trajectory, self.selected.lift_trajectory]
                    def positions(path):
                        return [list(point.positions) for point in path.points]
                    selected_index = next((index for index, path in enumerate(paths)
                        if trajectory.joint_names == path.joint_names and positions(trajectory) == positions(path)), None)
                    if selected_index is None:
                        raise RuntimeError('Trajectory differs from the selected MoveIt paths')
                    # Retiming stays inside the simulation joint-speed guard;
                    # the loaded lift keeps its planned timing for grip stability.
                    speedup = (.65, .9, 1.)[selected_index]
                    points = []
                    for point in trajectory.points:
                        seconds = (point.time_from_start.sec + point.time_from_start.nanosec/1e9)*speedup
                        whole = int(seconds)
                        points.append({'positions': list(point.positions), 'sec': whole,
                                       'nanosec': int((seconds-whole)*1e9)})
                    with self.motion_lock:
                        self.flush_gripper()
                        if selected_index == 2:
                            # Bind pre-grasp RGB-D geometry to the measured
                            # end-effector before lift, not to its lifted pose.
                            self.grasp_anchor = rpc('feedback')
                        rpc('trajectory', joint_names=list(trajectory.joint_names), points=points,
                            phase=('pregrasp', 'approach', 'lift')[selected_index])
                    deadline = time.monotonic()+75
                    while time.monotonic() < deadline:
                        if handle.is_cancel_requested or self.closed.is_set() or self.error:
                            rpc('abort')
                            raise RuntimeError(self.error or 'Trajectory canceled')
                        current = rpc('feedback')
                        state = current['trajectory'] or {}
                        if state.get('state') == 'succeeded':
                            self.flush_gripper()
                            handle.succeed()
                            result.error_code = 0
                            return result
                        if state.get('state') in ('failed', 'aborted'):
                            raise RuntimeError(state.get('error', 'Trajectory aborted'))
                        time.sleep(.02)
                    rpc('abort')
                    raise RuntimeError('Trajectory timed out')
                except Exception as error:
                    print('SIM_ACTION_ERROR: '+str(error), file=sys.stderr, flush=True)
                    handle.abort()
                    result.error_code, result.error_string = -1, str(error)
                    return result

        rclpy.init()
        probe = rclpy.create_node('go2_grasp_domain_probe')
        try:
            deadline = time.monotonic()+1.
            while time.monotonic() < deadline:
                rclpy.spin_once(probe, timeout_sec=.1)
            foreign = [name for name, namespace in probe.get_node_names_and_namespaces()
                       if name != probe.get_name()]
            if foreign:
                raise RuntimeError('ROS domain 183 already contains nodes; no grasp nodes started: '+str(foreign))
        finally:
            probe.destroy_node()
        node = Bridge()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        spin = threading.Thread(target=executor.spin, daemon=True)
        spin.start()

        def wait(predicate, timeout, message):
            deadline = time.monotonic()+timeout
            while time.monotonic() < deadline:
                if node.error:
                    raise RuntimeError(node.error)
                for name, process in processes:
                    if process.poll() is not None:
                        raise RuntimeError(name+' exited; see '+str(directory/(name+'.log')))
                if predicate():
                    return
                time.sleep(.03)
            raise RuntimeError(message)

        def service(name, timeout=90):
            client = node.create_client(Trigger, name, callback_group=node.group)
            if not client.wait_for_service(timeout_sec=10):
                raise RuntimeError('Service unavailable: '+name)
            future = client.call_async(Trigger.Request())
            wait(future.done, timeout, 'Service timeout: '+name)
            response = future.result()
            if not response.success:
                raise RuntimeError(name+': '+response.message)
            return response

        def ros_node(name, module, parameters):
            command = (['/usr/bin/python3', '-m', 'integration.go2_grasp_ros', 'planner', str(directory/'robot.urdf')]
                       if name == 'planner' else ['/usr/bin/python3', '-c', 'from '+module+' import main; main()'])
            command += ['--ros-args']
            for key, value in parameters.items():
                command += ['-p', key+':='+json.dumps(value)]
            if name == 'executor':
                command = executor_command(command, request.get('interfaces_setup'))
            spawn(name, command)

        for name, binary in [('robot_state_publisher', '/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher'),
                             ('move_group', '/opt/ros/humble/lib/moveit_ros_move_group/move_group')]:
            spawn(name, [binary, '--ros-args', '--params-file', directory/'moveit.yaml'])
        ros_node('grasp_generator', 'grasp_generation.node', {'graspnet_config': str(directory/'graspnet.json')})
        ros_node('perception', 'perception_node.node', {'segmentation_backend': 'tabletop_depth_components',
            'simulation_mode': True, 'target_session': owner, 'capture_directory': str(directory),
            'selected_pixel_u': request['pixel'][0] if request['pixel'] else -1,
            'selected_pixel_v': request['pixel'][1] if request['pixel'] else -1,
            'table_height_world': request['table_height'],
            'table_bounds_world': table_bounds,
            'camera_rotation_world': np.asarray(frame['rotation']).reshape(-1).tolist(),
            'camera_translation_world': frame['translation']})
        ros_node('planner', 'robot_planner.node', {'base_frame': 'world', 'end_effector_link': 'link6',
            'tcp_calibrated': True, 'tool_translation': [0., 0., request['tcp_m']],
            'session_id': owner, 'max_gripper_width_m': .075, 'require_target_collision': True,
            'table_height_world': request['table_height'], 'min_grasp_above_table_m': .135,
            'lift_retreat_options_m': [0., .04, .08, .12], 'pregrasp_distance_options_m': [0., .06, .14]})
        ros_node('executor', 'task_executor.node', {'execute': False, 'simulation_mode': True,
            'place_after_grasp': False, 'session_id': owner,
            'tool_translation': [0., 0., request['tcp_m']]})
        wait(lambda: node.had_candidates,
             150, 'No fresh GraspNet candidates; inspect perception/grasp_generator logs')
        service('/planner/preview', 150)
        wait(lambda: node.selected is not None, 5, 'Planner did not publish the selected path')
        if request['operation'] == 'preview':
            write('result.json', {'passed': True, 'operation': 'preview', 'motion_executed': False,
                                 'pipeline': 'local_perception_GraspNet_MoveIt'})
            print('Live-world MoveIt preview passed; no arm/gripper commands sent.')
            return 0
        wait(lambda: node.candidates is not None and
             0 <= node.get_clock().now().nanoseconds/1e9-(node.candidates.header.stamp.sec+
                 node.candidates.header.stamp.nanosec/1e9) < .4,
             15, 'No fresh execution-time target observation')
        execution_started = True
        service('/sim/task/start', 10)
        wait(lambda: node.state == 'HOLDING' or node.state.startswith('RECOVERY_REQUIRED'),
             180, 'Executor did not reach HOLDING')
        if node.state != 'HOLDING':
            raise RuntimeError(node.state)
        held_feedback = rpc('hold_confirmed')
        if node.grasp_anchor is None:
            raise RuntimeError('Missing measured grasp anchor before lift')
        anchor = node.grasp_anchor
        held_rotation = np.asarray(anchor['link6_rotation']).reshape(3, 3)
        bottom_offset = held_rotation.T @ (np.asarray(object_bottom)-np.asarray(anchor['link6_position']))
        write('held-object-geometry.json', {'source': 'selected RGB-D object component',
              'observed_bottom_world_m': object_bottom, 'bottom_from_link6_local_m': bottom_offset.tolist(),
              'bottom_source': 'adjacent wrist RGB-D support plane; upright bottle assumption',
              'support_plane_residual_m': residual,
              'upright_axis_link6_local': (held_rotation.T @ np.array([0., 0., 1.])).tolist(),
              'grasp_link6_position_m': anchor['link6_position'], 'anchor_stage': 'before_lift'})
        evidence = []
        for unused in range(30):
            evidence.append(rpc('evidence'))
            time.sleep(.1)
        write('physical-evidence.json', evidence)
        offsets = []
        for row in evidence:
            rotation = np.asarray(row['link6_rotation']).reshape(3, 3)
            offsets.append(rotation.T @ (np.asarray(row['target_position_world_m'])-
                                          np.asarray(row['link6_position'])))
        offset_center = np.median(offsets, axis=0)
        offset_spread = max(float(np.linalg.norm(offset-offset_center)) for offset in offsets)
        write('lift-evidence-summary.json', {'min_lift_m': min(row['bottle_lift'] for row in evidence),
              'max_object_offset_spread_m': offset_spread,
              'contact_samples': sum(row['contact_force'] >= .1 for row in evidence)})
        if min(row['bottle_lift'] for row in evidence) < .05 or offset_spread > .025:
            raise RuntimeError('HOLDING lacks sustained object lift and relative-pose evidence')
        zero_motion = rpc('preset', positions=[0.]*6, held_return=True)
        write('held-return-zero.json', zero_motion)
        deadline = time.monotonic()+60
        while time.monotonic() < deadline:
            feedback = rpc('feedback')
            if feedback['fault']:
                raise RuntimeError('Held return stopped: '+feedback['fault'])
            state = (feedback.get('trajectory') or {}).get('state')
            if state == 'succeeded':
                break
            if state != 'running':
                raise RuntimeError('Held return did not complete: '+str(state))
            time.sleep(.1)
        else:
            raise RuntimeError('Held return to zero timed out')
        returned_evidence = []
        for unused in range(30):
            returned_evidence.append(rpc('evidence'))
            time.sleep(.1)
        write('held-return-evidence.json', returned_evidence)
        offsets = []
        for row in returned_evidence:
            rotation = np.asarray(row['link6_rotation']).reshape(3, 3)
            offset = rotation.T @ (np.asarray(row['target_position_world_m'])-
                                   np.asarray(row['link6_position']))
            offsets.append(offset)
        offset_center = np.median(offsets, axis=0)
        offset_spread = max(float(np.linalg.norm(offset-offset_center)) for offset in offsets)
        write('held-return-contact-summary.json', {'contact_samples': sum(row['contact_force'] >= .1 for row in returned_evidence),
              'max_object_offset_spread_m': offset_spread, 'min_lift_m': min(row['bottle_lift'] for row in returned_evidence)})
        # Returning to reference zero legitimately lowers the hand. Absolute
        # height over the original tabletop is no longer a grasp validity test.
        if not all(row.get('hold_valid', False) for row in returned_evidence) or offset_spread > .025:
            raise RuntimeError('Return to zero lost the held object')
        deadline = time.monotonic()+2.
        while True:
            try:
                drive_state = rpc('enable_drive')
                break
            except RuntimeError as error:
                if 'Held object evidence is missing' not in str(error) or time.monotonic() >= deadline:
                    raise
                time.sleep(.1)
        write('drive-enabled.json', drive_state)
        successful_hold = True
        write('result.json', {'passed': True, 'operation': 'run', 'motion_executed': True,
                             'holding': True, 'returned_to_zero': True, 'drive_enabled': True,
                             'pipeline': 'local_perception_GraspNet_MoveIt_TaskExecutor'})
        print('Grasp and held return to zero physically verified; ownership retained. No automatic release/reset.')
        return 0
    except Exception as error:
        if reserved:
            try:
                write('failure_feedback.json', rpc('feedback'))
            except Exception:
                pass
        write('result.json', {'passed': False, 'error': str(error), 'execution_requested': execution_started})
        print('ERROR: '+str(error), file=sys.stderr)
        return 1
    finally:
        if node:
            node.closed.set()
        for name, process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
        for name, process in reversed(processes):
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        if reserved:
            try:
                if not successful_hold:
                    rpc('abort')
                if not execution_started:
                    rpc('release')
            except Exception as error:
                print('Recovery ownership retained: '+str(error), file=sys.stderr)
        if executor:
            executor.shutdown(timeout_sec=3)
        if node:
            node.destroy_node()
            rclpy.shutdown()
        for output in logs:
            output.close()


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'planner':
        run_planner(sys.argv[2], sys.argv[3:])
    else:
        raise SystemExit(main())
