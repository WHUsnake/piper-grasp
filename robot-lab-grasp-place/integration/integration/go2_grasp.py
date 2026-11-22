"""Device-free grasp transport for the authoritative Go2/Piper world."""
import base64
import json
import math
import socketserver
import threading
import time
import uuid
from pathlib import Path


def launch_pipeline(settings, options, directory, log=None):
    import hashlib
    import os
    import subprocess
    from integration.paths import ROOT
    source = Path(settings['source'])
    directory.mkdir(parents=True, exist_ok=False)
    hashes = {}
    for base in ('src/manipulation', 'src/perception', 'src/interfaces/ros2/grasp_msgs'):
        for path in (source/base).rglob('*'):
            if path.is_file() and '__pycache__' not in path.parts and path.suffix in ('.py', '.msg', '.json'):
                hashes[str(path.relative_to(source))] = hashlib.sha256(path.read_bytes()).hexdigest()
    (directory/'source-fingerprints.json').write_text(json.dumps({'source': str(source), 'sha256': hashes}, indent=2))
    if os.environ.get('LAB_GRASP_BACKEND'):
        options = dict(options, backend=os.environ['LAB_GRASP_BACKEND'])
    (directory/'request.json').write_text(json.dumps(dict(settings, **options, directory=str(directory)), indent=2))
    env = dict(os.environ, GRASPNET_PYTHON=settings['worker_python'],
               GRASPNET_BASELINE=settings['baseline'], GRASPNET_CHECKPOINT=settings['checkpoint'],
               ROS_DOMAIN_ID='183', ROS_LOCALHOST_ONLY='1', PYTHONNOUSERSITE='1')
    python_paths = [ROOT, source/'src/manipulation/grasp_generation',
                    source/'src/manipulation/robot_planner', source/'src/manipulation/task_executor',
                    source/'src/perception/perception_node']
    script = ('set -e; source /opt/ros/humble/setup.bash; source "$1"; '
              'export PYTHONPATH="$2${PYTHONPATH:+:$PYTHONPATH}"; '
              'exec /usr/bin/python3 -m integration.go2_grasp_ros "$3"')
    output = log.open('w') if log else None
    try:
        return subprocess.Popen(['bash', '-c', script, 'go2-grasp',
                                 str(source/'workspaces/pipeline/install/setup.bash'),
                                 ':'.join(map(str, python_paths)), str(directory/'request.json')],
                                env=env, stdout=output, stderr=None if output is None else subprocess.STDOUT,
                                start_new_session=True)
    finally:
        if output:
            output.close()


def grasp_support(frame, pixel, profile):
    """Choose a configured tabletop from the measured click, never object truth.

    This selects the segmentation prior only. RGB-D must still segment and
    measure the target/support, and MoveIt must check arm reach and collisions.
    """
    import struct
    u, v = pixel
    if frame.get('camera') != 'wrist_rgbd' or not frame.get('depth'):
        raise ValueError('Grasp support selection requires wrist RGB-D')
    if type(u) is not int or type(v) is not int or not 0 <= u < 640 or not 0 <= v < 480:
        raise ValueError('Invalid grasp pixel')
    depth = struct.unpack_from('<f', base64.b64decode(frame['depth']), 4*(v*640+u))[0]
    if not math.isfinite(depth) or not .1 <= depth <= 2.5:
        raise ValueError('Grasp pixel has no valid metric depth')
    fx, _, cx, _, fy, cy, *_ = frame['intrinsics']
    optical = [(u-cx)*depth/fx, (v-cy)*depth/fy, depth]
    point = [sum(row[i]*optical[i] for i in range(3))+origin
             for row, origin in zip(frame['rotation'], frame['translation'])]
    surfaces = profile.get('grasp_surfaces') or [profile['placed_bottle']]
    candidates = []
    for surface in surfaces:
        xmin, xmax, ymin, ymax = surface['table_bounds_world']
        gap = point[2]-surface['tabletop_height_m']
        if xmin <= point[0] <= xmax and ymin <= point[1] <= ymax and .015 <= gap <= .4:
            candidates.append((gap, surface))
    if not candidates:
        raise ValueError('Selected object is outside configured test tabletops; move closer and select a visible bottle')
    return dict(min(candidates, key=lambda item: item[0])[1], selected_world_m=point)


class PlacementNotVisible(ValueError):
    """The measured target lies outside the current wrist view."""


def placement_surface(frame, pixel, radius=.045):
    """Measure a support plane and a free footprint, using wrist depth only."""
    import numpy as np
    if frame.get('camera', 'wrist_rgbd') != 'wrist_rgbd' or not frame.get('depth'):
        raise ValueError('Placement requires link6 wrist RGB-D; body RGB is observation only')
    u, v = pixel
    if not (4 <= u < 636 and 4 <= v < 476):
        raise PlacementNotVisible('Select a placement point away from the image border')
    depth = np.frombuffer(base64.b64decode(frame['depth']), dtype='<f4').reshape(480, 640)
    robot = np.frombuffer(base64.b64decode(frame['self_mask']), dtype=np.uint8).reshape(480, 640)
    selected = float(depth[v, u])
    if robot[v, u] or not math.isfinite(selected) or not .15 <= selected <= 2.5:
        raise ValueError('Select a visible support surface with valid depth')
    # A metric footprint replaces the fixed 65-pixel window. At oblique wrist
    # views the old window rejected usable surfaces and accepted table edges.
    yy, xx = np.mgrid[0:480:3, 0:640:3]
    z = depth[yy, xx]
    fx, _, cx, _, fy, cy, *_ = frame['intrinsics']
    rotation = np.asarray(frame['rotation'], dtype=float)
    origin = np.asarray(frame['translation'], dtype=float)
    selected_world = rotation @ np.array([(u-cx)*selected/fx, (v-cy)*selected/fy, selected]) + origin
    valid = np.isfinite(z) & (z >= .15) & (z <= 2.5) & (robot[yy, xx] == 0)
    optical = np.stack(((xx[valid]-cx)*z[valid]/fx,
                        (yy[valid]-cy)*z[valid]/fy, z[valid]), axis=1)
    points = optical @ rotation.T + origin
    nearby = points[np.linalg.norm(points[:, :2]-selected_world[:2], axis=1) < .10]
    points = nearby[abs(nearby[:, 2]-selected_world[2]) < .015]
    if len(points) < 100:
        raise ValueError('Selected placement surface lacks a continuous RGB-D patch')
    # Isolate the dominant height before SVD. Near the bottle, a 15-mm height
    # slab also contains its lower curved wall, which used to tilt the fitted
    # plane and make an unchanged tabletop alternate between pass and fail.
    counts, edges = np.histogram(points[:, 2], bins=np.arange(selected_world[2]-.016,
                                                           selected_world[2]+.018, .002))
    peak = int(np.argmax(counts))
    height = (edges[peak]+edges[peak+1])/2
    points = points[abs(points[:, 2]-height) < .003]
    if len(points) < 100:
        raise ValueError('Not enough reliable support-plane depth samples')
    center = np.median(points, axis=0)
    centered = points-center
    _, _, axes = np.linalg.svd(centered, full_matrices=False)
    normal = axes[-1]
    if normal[2] < 0:
        normal = -normal
    residual = float(np.percentile(abs(centered @ normal), 90))
    if normal[2] < .985 or residual > .004:
        raise ValueError('Select a flat, horizontal support surface')
    spread = np.percentile(points[:, :2], 90, axis=0)-np.percentile(points[:, :2], 10, axis=0)
    if min(spread) < .025:
        raise ValueError('Selected support patch is too narrow for placement')
    if abs(float((selected_world-center) @ normal)) > .008:
        raise ValueError('Selected pixel does not lie on the measured support plane')
    if not .25 <= center[2] <= .85:
        raise ValueError('Select a tabletop support at arm placement height')
    selected_world[2] = center[2]-np.dot(normal[:2], selected_world[:2]-center[:2])/normal[2]
    # Every footprint sample must be observed at the plane. Missing depth,
    # self occlusion, an obstacle and an edge are all unknown/unsafe, not free.
    samples = [selected_world.copy()]
    for r in (radius*.5, radius):
        for angle in np.linspace(0., 2*np.pi, 24, endpoint=False):
            p = selected_world+np.array([r*np.cos(angle), r*np.sin(angle), 0.])
            p[2] -= np.dot(normal[:2], p[:2]-selected_world[:2])/normal[2]
            samples.append(p)
    optical = (np.asarray(samples)-origin) @ rotation
    if np.any(optical[:, 2] <= .1):
        raise ValueError('Placement footprint is outside the wrist view')
    sx = np.rint(fx*optical[:, 0]/optical[:, 2]+cx).astype(int)
    sy = np.rint(fy*optical[:, 1]/optical[:, 2]+cy).astype(int)
    if np.any((sx < 1) | (sx >= 639) | (sy < 1) | (sy >= 479)):
        raise PlacementNotVisible('Placement footprint crosses the image border; select farther inside the table')
    measured = depth[sy, sx]
    visible = np.isfinite(measured) & (robot[sy, sx] == 0) & (abs(measured-optical[:, 2]) < .008)
    if not visible.all():
        raise ValueError('Placement footprint is occluded, obstructed or crosses a table edge')
    heights = (nearby-selected_world) @ normal
    if np.any((np.linalg.norm(nearby[:, :2]-selected_world[:2], axis=1) < radius) &
              (heights > .012) & (heights < .25)):
        raise ValueError('An obstacle occupies the selected placement footprint')
    return {'point_world': selected_world.tolist(), 'normal_world': normal.tolist(),
            'patch_points': len(points), 'height_m': float(selected_world[2]),
            'footprint_radius_m': radius, 'plane_residual_m': residual,
            'source': 'wrist_rgbd', 'stamp': frame.get('stamp'),
            'uncertainty_m': max(.002, residual)}


def reobserve_placement(frame, surface):
    """Reproject a world target into a NEW wrist image, never reuse its old pixel."""
    import numpy as np
    p = np.asarray(surface['point_world'])
    optical = np.asarray(frame['rotation']).T @ (p-np.asarray(frame['translation']))
    fx, _, cx, _, fy, cy, *_ = frame['intrinsics']
    if optical[2] <= .1:
        raise PlacementNotVisible('Placement target is behind the wrist camera; select a visible surface')
    pixel = [int(round(fx*optical[0]/optical[2]+cx)), int(round(fy*optical[1]/optical[2]+cy))]
    if 0 <= pixel[0] < 640 and 0 <= pixel[1] < 480:
        mask = base64.b64decode(frame['self_mask'])
        if mask[pixel[1]*640+pixel[0]]:
            raise PlacementNotVisible('Robot occludes the measured placement target in the wrist view')
    fresh = placement_surface(frame, pixel, surface.get('footprint_radius_m', .045))
    if abs(fresh['height_m']-p[2]) > .008 or math.dist(fresh['point_world'][:2], p[:2]) > .012:
        raise ValueError('Selected support changed in fresh wrist RGB-D; select again')
    fresh['point_world'][:2] = p[:2].tolist()
    fresh['pixel'] = pixel
    return fresh


def placement_visual_result(frame, surface):
    """Check visible above-plane geometry; never claim force-confirmed support."""
    import numpy as np
    if frame.get('camera') != 'wrist_rgbd' or not frame.get('depth'):
        raise ValueError('Placement verification requires wrist RGB-D')
    depth = np.frombuffer(base64.b64decode(frame['depth']), dtype='<f4').reshape(480, 640)
    robot = np.frombuffer(base64.b64decode(frame['self_mask']), dtype=np.uint8).reshape(480, 640)
    yy, xx = np.mgrid[0:480:2, 0:640:2]
    z = depth[yy, xx]
    valid = np.isfinite(z) & (z >= .1) & (z <= 2.5) & (robot[yy, xx] == 0)
    fx, _, cx, _, fy, cy, *_ = frame['intrinsics']
    cloud = np.stack(((xx[valid]-cx)*z[valid]/fx, (yy[valid]-cy)*z[valid]/fy, z[valid]), axis=1)
    cloud = cloud @ np.asarray(frame['rotation']).T+np.asarray(frame['translation'])
    point = np.asarray(surface['point_world'])
    height = (cloud-point) @ np.asarray(surface['normal_world'])
    cloud = cloud[(np.linalg.norm(cloud[:, :2]-point[:2], axis=1) < .065) & (height > .025) & (height < .30)]
    if len(cloud) < 40 or np.ptp(cloud[:, 2]) < .05:
        raise ValueError('Released object is not sufficiently visible in wrist depth; outcome unverified')
    centroid = np.median(cloud, axis=0)
    _, _, axes = np.linalg.svd(cloud-centroid, full_matrices=False)
    if abs(axes[0, 2]) < .94 or math.dist(centroid[:2], point[:2]) > .04:
        raise ValueError('Observed released object is tilted or outside the placement tolerance')
    return {'source': 'wrist_rgbd', 'stamp': frame['stamp'], 'points': len(cloud),
            'visible_centroid_world_m': centroid.tolist(), 'visible_axis_vertical_cosine': abs(float(axes[0, 2])),
            'support_confirmed': False}


def placement_joint_targets(service, point_world, bottom_offset, axis_local=None):
    """Solve and collision-check the approach above the selected support without moving the world."""
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation
    world = service.world
    np, mj, model, data = world.np, world.mj, world.model, world.data
    point = np.asarray(point_world, dtype=float)
    offset = np.asarray(bottom_offset, dtype=float)
    if point.shape != (3,) or offset.shape != (3,) or not np.isfinite(point).all() or not np.isfinite(offset).all():
        raise ValueError('Invalid RGB-D placement geometry')
    if not .02 <= np.linalg.norm(offset) <= .4:
        raise ValueError('Held-object RGB-D grasp offset is implausible')
    link = model.body('link6').id
    start = data.qpos[world.arm_qi].copy()
    orientation = data.body('link6').xmat.reshape(3, 3).copy()
    if axis_local is not None:
        axis = np.asarray(axis_local, dtype=float)
        if axis.shape != (3,) or not np.isfinite(axis).all() or abs(np.linalg.norm(axis)-1.) > .01:
            raise ValueError('Invalid observed held-object axis')
        correction, _ = Rotation.align_vectors([[0., 0., 1.]], [(orientation @ axis).tolist()])
        orientation = correction.as_matrix() @ orientation
    limits = model.jnt_range[[model.joint('joint'+str(i)).id for i in range(1, 7)]]
    scratch = mj.MjData(model)
    mj.mj_copyData(scratch, model, data)
    def pose(joints):
        scratch.qpos[world.arm_qi] = joints
        mj.mj_forward(model, scratch)
        body = scratch.body('link6')
        return body.xpos.copy(), body.xmat.reshape(3, 3).copy()
    def solve(target, seed):
        def residual(joints):
            position, rotation = pose(joints)
            return np.r_[position-target, .15*Rotation.from_matrix(orientation.T @ rotation).as_rotvec()]
        solution = least_squares(residual, np.clip(seed, limits[:, 0]+1e-5, limits[:, 1]-1e-5),
                                 bounds=(limits[:, 0], limits[:, 1]), max_nfev=160)
        error = residual(solution.x)
        if np.linalg.norm(error[:3]) > .004 or np.linalg.norm(error[3:])/.15 > .03:
            raise RuntimeError('Clicked placement point is outside collision-free arm reach')
        return solution.x
    bottom = point-orientation @ offset+np.array([0., 0., .006])
    above = bottom+np.array([0., 0., .06])
    first = solve(above, start)
    for begin, end in ((start, first),):
        steps = max(2, int(np.max(np.abs(end-begin))/.025)+1)
        for fraction in np.linspace(0, 1, steps):
            pose(begin+(end-begin)*fraction)
            if service.arm_collision(scratch, ignore_held_target=True):
                raise RuntimeError('Placement path collides; gripper will stay closed')
    return {'above': first.tolist(), 'retreat': first.tolist(),
            'link6_rotation_world': orientation.tolist(),
            'link6_above_world_m': above.tolist(), 'link6_release_world_m': bottom.tolist()}


def placement_pose_acceptable(measured_bottom, support_point):
    """Allow a resting bottle, while rejecting a meaningful tabletop penetration."""
    horizontal = math.dist(measured_bottom[:2], support_point[:2])
    vertical = measured_bottom[2]-support_point[2]
    return horizontal <= .025 and -.0015 <= vertical <= .025


class GraspPanel:
    def __init__(self, workspace, instance):
        self.workspace = Path(workspace)
        self.instance = instance
        self.lock = threading.RLock()
        self.frames = {}
        self.process = None
        self.directory = None
        self.operation = None
        self.held_owner = None

    def identity(self):
        return json.loads((self.workspace/'state/go2-grasp-world.json').read_text())

    def settings(self):
        from integration.paths import REPOSITORY
        return json.loads((REPOSITORY/'config.local.json').read_text()).get('go2_grasp')

    def status(self):
        from integration.rpc import call
        from integration.paths import REPOSITORY
        with self.lock:
            identity = self.identity()
            world = call(identity['socket'], 'feedback')
            if not (self.process and self.process.poll() is None) and \
                    (world.get('placement_control') or {}).get('state') == 'running':
                world = call(identity['socket'], 'abort', session=identity['session'], owner=world['owner'])
            recovery_available = bool(world['owner'] and not (self.process and self.process.poll() is None)
                                      and world['contact_force'] < .1 and
                                      not (world.get('trajectory') or {}).get('state') == 'running')
            if recovery_available and world['holding']:
                evidence = call(identity['socket'], 'evidence', session=identity['session'], owner=world['owner'])
                recovery_available = evidence['bottle_lift'] <= .02
            value = {'configured': bool(self.settings()), 'instance': self.instance, 'world': world,
                     'supports_restore_initial': True,
                     'supports_rgbd_place': True,
                     'placement_strategy': 'wrist_rgbd_geometry',
                     'sensors': {'wrist_rgbd': ['rgb', 'depth'], 'body_rgbd': ['rgb']},
                     'supports_place_retry': True,
                     'recovery_available': recovery_available,
                     'observation_pose_deg': json.loads((REPOSITORY/'profiles/hssd-go2-piper.json').read_text()).get('observation_pose_deg'),
                     'running': self.process is not None and self.process.poll() is None,
                     'operation': self.operation}
            if self.directory:
                value['run_directory'] = str(self.directory)
                result = self.directory/'result.json'
                if result.is_file():
                    value['result'] = json.loads(result.read_text())
                if self.process.poll() is not None:
                    value['exit_code'] = self.process.returncode
                log = self.directory/'web.log'
                if log.is_file():
                    with log.open('rb') as stream:
                        stream.seek(max(0, log.stat().st_size-3000))
                        value['log'] = stream.read().decode(errors='replace')
            # This is the sole web operation contract. Browser selection and
            # network state may further disable buttons, never enable them.
            active = value['running']
            failed = bool(world['fault'])
            report = value.get('result') or {}
            retryable = not active and (report.get('passed') is False or
                        (world.get('placement_control') or {}).get('state') in ('failed', 'aborted'))
            state = ('recovery_required' if failed else
                     (world.get('task_phase') or 'planning') if active else
                     'retryable_failure' if retryable else
                     'holding' if world['holding'] else
                     'retryable_failure' if world['owner'] else 'idle')
            idle = not active and not world['owner'] and not failed
            actions = {'zero': idle and value['configured'], 'observation': idle and value['configured'],
                       'preview': idle and value['configured'], 'run': idle and value['configured'],
                       'place': not active and world.get('hold_valid', False) and not failed,
                       'abort': active and bool(world['owner']),
                       'release': not active and bool(world['owner']) and not world['holding'] and world['contact_force'] < .1,
                       'recover_fault': recovery_available and not world.get('physics_fault', False),
                       'restore_initial': not active, 'observe': not active}
            value['task'] = {'state': state, 'allowed_actions': actions,
                             'error': world['fault'] or (report.get('error') if retryable else None),
                             'owner': world['owner']}
            return value

    def frame(self, camera):
        from integration.rpc import call
        with self.lock:
            identity = self.identity()
            frame = call(identity['socket'], 'observe', session=identity['session'], camera=camera)
            token = uuid.uuid4().hex
            self.frames = {key: value for key, value in self.frames.items() if time.monotonic()-value[0] < 60}
            if len(self.frames) >= 128:
                del self.frames[next(iter(self.frames))]
            self.frames[token] = (time.monotonic(), {key: value for key, value in frame.items() if key != 'rgb'})
            return dict(frame, frame_id=token, instance=self.instance)

    def submit(self, request):
        import struct
        from integration.rpc import call
        with self.lock:
            if request.get('instance') != self.instance:
                raise RuntimeError('Preview instance changed; reload this page')
            operation = request.get('operation')
            if operation not in ('preview', 'run', 'place', 'abort', 'release', 'recover_fault', 'zero', 'observation', 'restore_initial'):
                raise ValueError('Invalid grasp operation')
            identity = self.identity()
            world = call(identity['socket'], 'feedback')
            running = self.process is not None and self.process.poll() is None
            if operation == 'restore_initial':
                if request.get('confirm_restore') is not True:
                    raise ValueError('Restoring the initial state requires explicit confirmation')
                if running:
                    raise RuntimeError('Stop the running grasp task before restoring the initial state')
                result = call(identity['socket'], 'restore_initial', session=identity['session'],
                              confirm_restore=True)
                self.frames.clear()
                self.directory = self.operation = self.held_owner = None
                return {'restored': True, 'archive': result['archive'], 'world': result['world']}
            if operation == 'recover_fault':
                if running or world['contact_force'] > .1:
                    raise RuntimeError('Task recovery requires a stopped task without a held object or gripper contact')
                if not self.directory or world['owner'] != (self.held_owner or self.directory.name):
                    raise RuntimeError('This panel does not own the failed grasp')
                result = call(identity['socket'], 'recover_fault', session=identity['session'], owner=world['owner'])
                self.frames.clear()
                self.directory = self.operation = self.held_owner = None
                return {'recovered': True, 'world': result}
            if operation in ('abort', 'release'):
                if not self.directory or world['owner'] != (self.held_owner or self.directory.name):
                    raise RuntimeError('This panel does not own the active task')
                if operation == 'release' and running:
                    raise RuntimeError('Stop the active task before returning control')
                if operation == 'abort' and running:
                    self.process.terminate()
                return call(identity['socket'], operation, session=identity['session'], owner=world['owner'])
            if operation == 'place':
                if request.get('execute') is not True:
                    raise ValueError('Placement requires explicit confirmation')
                if running or not world['holding'] or not world['owner'] or world['fault']:
                    raise RuntimeError('Placement requires a completed healthy held grasp')
                if not world['owner'].startswith('go2-grasp-') or '/' in world['owner']:
                    raise RuntimeError('This panel lacks the original held grasp context')
                grasp_directory = self.workspace/'runs'/world['owner']
                geometry_path = grasp_directory/'held-object-geometry.json'
                if not world.get('hold_valid') or not geometry_path.is_file():
                    raise RuntimeError('RGB-D grasp geometry or current verified hold evidence is missing')
                held_geometry = json.loads(geometry_path.read_text())
                if not held_geometry.get('upright_axis_link6_local'):
                    raise RuntimeError('Held grasp predates wrist RGB-D placement geometry; start a new grasp')
                pixel = [request.get('u'), request.get('v')]
                if any(type(value) is not int for value in pixel) or not 0 <= pixel[0] < 640 or not 0 <= pixel[1] < 480:
                    raise ValueError('Select a placement pixel in the RGB-D image')
                saved = self.frames.get(request.get('frame_id'))
                if not saved or time.monotonic()-saved[0] > 30:
                    raise ValueError('Placement RGB-D frame expired; capture again')
                frame = saved[1]
                if frame['session'] != world['session']:
                    raise RuntimeError('World changed; refresh placement RGB-D')
                surface = placement_surface(frame, pixel)
                fresh = call(identity['socket'], 'observe', session=identity['session'], camera=frame['camera'])
                fresh_surface = reobserve_placement(fresh, surface)
                directory = self.workspace/'runs'/('go2-place-'+str(time.time_ns()))
                directory.mkdir(parents=True, exist_ok=False)
                options = {'operation': 'place', 'socket': identity['socket'], 'session': world['session'],
                           'owner': world['owner'], 'directory': str(directory), 'surface': fresh_surface,
                           'bottom_offset_local': held_geometry['bottom_from_link6_local_m'],
                           'axis_local': held_geometry['upright_axis_link6_local']}
                request_path = directory/'request.json'
                request_path.write_text(json.dumps(options, indent=2))
                import subprocess
                import sys
                from integration.paths import ROOT
                with (directory/'web.log').open('w') as log:
                    self.process = subprocess.Popen([sys.executable, '-m', 'integration.go2_grasp', str(request_path)],
                                                    cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                self.held_owner, self.directory, self.operation = world['owner'], directory, 'place'
                self.frames.clear()
                return {'started': True, 'operation': 'place', 'run_directory': str(directory), 'surface': fresh_surface}
            if running or world['owner']:
                raise RuntimeError('An active grasp/recovery owns the robot; inspect status first')
            self.held_owner = None
            if operation in ('zero', 'observation'):
                import subprocess
                import sys
                from integration.paths import ROOT
                if request.get('execute') is not True:
                    raise ValueError('Pose motion requires explicit confirmation')
                settings = self.settings()
                if not settings:
                    raise RuntimeError('Configure the local grasp source first')
                directory = self.workspace/'runs'/('go2-pose-'+str(time.time_ns()))
                directory.mkdir(parents=True)
                options = dict(socket=identity['socket'], session=world['session'],
                               owner=directory.name, operation=operation, directory=str(directory),
                               source=settings['source'], workspace=str(self.workspace))
                request_path = directory/'request.json'
                request_path.write_text(json.dumps(options, indent=2))
                with (directory/'web.log').open('w') as log:
                    self.process = subprocess.Popen([sys.executable, '-m', 'integration.go2_grasp', str(request_path)],
                                                    cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                                    start_new_session=True)
                self.directory, self.operation = directory, operation
                self.frames.clear()
                return {'started': True, 'operation': operation, 'run_directory': str(directory)}
            if operation == 'run' and request.get('execute') is not True:
                raise ValueError('Execution requires explicit confirmation')
            pixel = [request.get('u'), request.get('v')]
            if any(type(value) is not int for value in pixel) or not 0 <= pixel[0] < 640 or not 0 <= pixel[1] < 480:
                raise ValueError('Select a pixel inside the 640x480 image')
            saved = self.frames.get(request.get('frame_id'))
            if not saved or time.monotonic()-saved[0] > 60:
                raise ValueError('Image expired; capture RGB-D and select the target again')
            frame = saved[1]
            if frame['session'] != world['session']:
                raise RuntimeError('World changed; capture RGB-D again')
            if frame['camera'] != 'wrist_rgbd' or not frame.get('depth'):
                raise ValueError('Grasp selection requires wrist RGB-D; body RGB is observation only')
            depth = struct.unpack_from('<f', base64.b64decode(frame['depth']), 4*(pixel[1]*640+pixel[0]))[0]
            if not math.isfinite(depth) or not .1 <= depth <= 2.5:
                raise ValueError('Selected depth is invalid or outside the original grasp range (0.1–2.5m)')
            if base64.b64decode(frame['self_mask'])[pixel[1]*640+pixel[0]]:
                raise ValueError('Selected pixel belongs to the robot; move to observation pose and select visible target')
            settings = self.settings()
            if not settings:
                raise RuntimeError('Local grasp source is not configured; run lab grasp configure')
            expected = {key: frame[key] for key in ('session', 'stamp', 'rotation', 'translation')}
            expected['depth_m'] = depth
            # Reject a stale click before starting the costly ROS/MoveIt worker.
            # The worker repeats this check after the base has settled.
            fresh = call(identity['socket'], 'observe', session=identity['session'], camera=frame['camera'])
            import numpy as np
            from scipy.spatial.transform import Rotation
            moved = np.linalg.norm(np.asarray(fresh['translation'])-frame['translation'])
            rotated = Rotation.from_matrix(np.asarray(fresh['rotation']) @ np.asarray(frame['rotation']).T).magnitude()
            fresh_depth = struct.unpack_from('<f', base64.b64decode(fresh['depth']),
                                             4*(pixel[1]*640+pixel[0]))[0]
            if (time.monotonic()-saved[0] > 25 or fresh['session'] != frame['session'] or
                    moved > .02 or rotated > .04 or not math.isfinite(fresh_depth) or
                    abs(fresh_depth-depth) > .03):
                raise ValueError('Selected RGB-D image expired or camera moved; capture and select the target again')
            directory = self.workspace/'runs'/('go2-grasp-'+str(time.time_ns()))
            from integration.paths import REPOSITORY
            profile = json.loads((REPOSITORY/'profiles/hssd-go2-piper.json').read_text())
            support = grasp_support(fresh, pixel, profile)
            options = dict(operation=operation, socket=identity['socket'], world_session=world['session'],
                           camera=frame['camera'], table_height=support['tabletop_height_m'],
                           tcp_m=float(settings.get('tcp_m', .11)), pixel=pixel,
                           table_bounds_world=support['table_bounds_world'],
                           support_table=support.get('support_table'), expected_camera=expected)
            self.process = launch_pipeline(settings, options, directory, log=directory/'web.log')
            self.directory, self.operation = directory, operation
            return {'started': True, 'operation': operation, 'run_directory': str(directory)}

    def close(self):
        import subprocess
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()


class GraspWorld:
    def __init__(self, world, workspace):
        self.world = world
        # RGB and depth must see the same support geometry as the physics.
        # Only promote existing static table-top meshes; never add overlapping
        # collision proxies or change their contact, mass or friction settings.
        import numpy as np
        model, data = world.model, world.data
        manifest = json.loads((world.assets/'manifest.json').read_text())
        surfaces = manifest.get('manipulation_surfaces', [])
        self.support_render_geoms = {}
        for surface in surfaces:
            xmin, xmax, ymin, ymax = surface['table_bounds_world']
            height = surface['tabletop_height_m']
            selected = []
            for geom in np.flatnonzero((model.geom_group == 4) & (model.geom_bodyid == 0)):
                mesh = int(model.geom_dataid[geom])
                if model.geom_type[geom] != world.mj.mjtGeom.mjGEOM_MESH or mesh < 0:
                    continue
                start, count = int(model.mesh_vertadr[mesh]), int(model.mesh_vertnum[mesh])
                vertices = (model.mesh_vert[start:start+count] @
                            data.geom_xmat[geom].reshape(3, 3).T + data.geom_xpos[geom])
                lo, hi = vertices.min(axis=0), vertices.max(axis=0)
                if (abs(hi[2]-height) <= .015 and
                        lo[0] >= xmin-.025 and hi[0] <= xmax+.025 and
                        lo[1] >= ymin-.025 and hi[1] <= ymax+.025 and
                        min(hi[0]-lo[0], hi[1]-lo[1]) >= .06):
                    selected.append(int(geom))
            self.support_render_geoms[surface['id']] = selected
        # Backward-compatible original scene bundles have no surface metadata.
        if not surfaces:
            self.support_render_geoms['living-coffee'] = [
                model.geom(name).id for name in ('c4412', 'c4413')]
        for geoms in self.support_render_geoms.values():
            for geom in geoms:
                model.geom_group[geom] = 2
                model.geom_rgba[geom] = [.30, .22, .16, 1.]
        self.workspace = Path(workspace)
        self.lock = threading.RLock()
        self.render_lock = threading.RLock()
        self.session = uuid.uuid4().hex
        self.owner = None
        self.trajectory = None
        self.trajectory_phase = None
        self.preset_active = False
        self.result = None
        self.renderer = None
        self.render_thread = None
        self.fault = None
        self.fault_details = None
        self.settling = False
        self.stance_history = []
        self.stable_since = None
        self.target_body = 'bottle'
        self.holding = False
        self.drive_enabled = False
        self.held_return_active = False
        self.place_release_active = False
        self.drop_since = None
        self.held_offset_local = None
        self.jaw_goal_width = None
        self.task_phase = None
        self.place_control = None
        self.physics_fault = False
        self.physics_substeps = 5
        # Record rather than tune the restored physical parameters.
        bottle_names = [world.model.body(index).name for index in range(1, world.model.nbody)
                        if world.model.body(index).name == 'bottle' or
                        (world.model.body(index).name.startswith('tabletop_') and
                         world.model.body(index).name.endswith('_bottle'))]
        self.physics_parameters = {
            'bottle_mass_kg': {name: float(world.model.body_mass[world.model.body(name).id])
                               for name in bottle_names},
            'bottle_inertia': {name: world.model.body_inertia[world.model.body(name).id].tolist()
                               for name in bottle_names},
            'finger_friction': {name: world.model.geom_friction[world.model.geom(name).id].tolist()
                                for name in ('link7_collision', 'link8_collision')}}
        # The HSSD bundle has many mesh contacts. Its original impratio=100 and
        # 4 ms bottle contacts can make a free bottle's angular DOF diverge when
        # the arm approaches the table. Keep a compliant, finite contact patch.
        world.model.opt.impratio = 10
        for name in ('link7_collision', 'link8_collision'):
            geom_id = world.model.geom(name).id
            world.model.geom_condim[geom_id] = 6
            world.model.geom_priority[geom_id] = 1
            world.model.geom_solref[geom_id] = [.02, 1.]
        for name in bottle_names:
            body_id = world.model.body(name).id
            for geom_id in range(world.model.ngeom):
                if world.model.geom_bodyid[geom_id] != body_id or not world.model.geom_contype[geom_id]:
                    continue
                world.model.geom_condim[geom_id] = 4
                world.model.geom_solref[geom_id] = [.02, 1.]
        world.mj.mj_forward(world.model, world.data)
        import hashlib
        self.physics_parameters.update(timestep_s=float(world.model.opt.timestep),
                                       integration_substeps=self.physics_substeps,
                                       integration_timestep_s=float(world.model.opt.timestep)/self.physics_substeps,
                                       impratio=float(world.model.opt.impratio))
        self.physics_parameters['sha256'] = hashlib.sha256(json.dumps(
            self.physics_parameters, sort_keys=True).encode()).hexdigest()
        self.jaw_force_limits = world.model.actuator_forcerange[world.jaw_ai].copy()
        self.last_client = time.monotonic()
        self.record = self.workspace/'state/go2-grasp-world.json'
        self.record.parent.mkdir(parents=True, exist_ok=True)
        # AF_UNIX paths are limited to about 108 bytes; long workspace names exceed that.
        self.socket = Path('/tmp')/('go2-grasp-'+self.session+'.sock')
        service = self

        class Handler(socketserver.StreamRequestHandler):
            def handle(self):
                try:
                    raw = self.rfile.readline(1024*1024+1)
                    if len(raw) > 1024*1024:
                        raise ValueError('Oversized request')
                    request = json.loads(raw)
                    if request.get('method') in ('capture', 'observe'):
                        value = service.capture(request)
                    else:
                        with service.lock:
                            value = service.request(request)
                except Exception as error:
                    value = {'error': str(error)}
                self.wfile.write((json.dumps(value, allow_nan=False)+'\n').encode())

        self.server = socketserver.UnixStreamServer(str(self.socket), Handler)
        self.socket.chmod(0o600)
        self.thread = threading.Thread(target=self.serve, daemon=True)
        self.thread.start()
        self.record.write_text(json.dumps({'socket': str(self.socket), 'session': self.session}))

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.socket.unlink(missing_ok=True)

    def serve(self):
        try:
            self.server.serve_forever()
        finally:
            if self.renderer is not None:
                self.renderer.close()

    def capture(self, request):
        world = self.world
        np, model = world.np, world.model
        camera = request.get('camera', 'wrist_rgbd')
        if camera not in ('body_rgbd', 'wrist_rgbd', 'overview'):
            raise ValueError('Unsupported acquisition camera')
        if request.get('method') == 'capture' and camera != 'wrist_rgbd':
            raise ValueError('Grasp capture requires wrist RGB-D; body RGB is observation only')
        with self.lock:
            if request.get('session') != self.session:
                raise RuntimeError('Grasp session/owner mismatch')
            if request['method'] == 'capture':
                if not self.owner or request.get('owner') != self.owner:
                    raise RuntimeError('Grasp session/owner mismatch')
                if self.fault:
                    raise RuntimeError(self.fault)
                if self.settling:
                    raise RuntimeError('Wait for the stable planning snapshot before capture')
                self.last_client = time.monotonic()
            data = world.mj.MjData(model)
            world.mj.mj_copyData(data, model, world.data)
            stamp = time.time()
        with self.render_lock:
            if self.renderer is None:
                self.render_thread = threading.get_ident()
                self.renderer = world.mj.Renderer(model, height=480, width=640)
                self.render_option = world.mj.MjvOption()
                self.render_option.geomgroup[3] = 1
                self.robot_render_option = world.mj.MjvOption()
                self.robot_render_option.geomgroup[:] = 0
                self.robot_render_option.geomgroup[3] = 1
            elif self.render_thread != threading.get_ident():
                raise RuntimeError('RGB-D renderer must be used from its service thread')
            self.renderer.disable_depth_rendering()
            self.renderer.update_scene(data, camera=camera, scene_option=self.render_option)
            rgb = self.renderer.render().copy()
            self.renderer.enable_depth_rendering()
            try:
                self.renderer.update_scene(data, camera=camera, scene_option=self.render_option)
                depth = self.renderer.render().astype('<f4')
                self.renderer.update_scene(data, camera=camera, scene_option=self.robot_render_option)
                robot_depth = self.renderer.render().astype('<f4')
            finally:
                self.renderer.disable_depth_rendering()
        self_mask = (robot_depth < depth+.003) & (robot_depth < 10.)
        camera_id = model.camera(camera).id
        focal = 240/math.tan(math.radians(float(model.cam_fovy[camera_id]))/2)
        rotation = data.cam_xmat[camera_id].reshape(3, 3)@np.diag([1, -1, -1])
        return {'session': self.session, 'stamp': stamp, 'width': 640, 'height': 480,
                'camera': camera, 'rgb': base64.b64encode(rgb.tobytes()).decode(),
                'modality': 'rgbd' if camera == 'wrist_rgbd' else 'rgb',
                'depth': base64.b64encode(depth.tobytes()).decode() if camera == 'wrist_rgbd' else None,
                'self_mask': base64.b64encode(self_mask.astype('uint8').tobytes()).decode(),
                'intrinsics': [focal, 0., 319.5, 0., focal, 239.5, 0., 0., 1.],
                'rotation': rotation.tolist(), 'translation': data.cam_xpos[camera_id].tolist()}

    def feedback(self):
        world = self.world
        np, model, data = world.np, world.model, world.data
        finger_bodies = {model.body('link7').id, model.body('link8').id}
        target_body = model.body(self.target_body).id
        forces = {body: 0.0 for body in finger_bodies}
        for index in range(data.ncon):
            contact = data.contact[index]
            bodies = {int(model.geom_bodyid[contact.geom1]), int(model.geom_bodyid[contact.geom2])}
            # Only bottle-to-finger contact confirms a grasp. A finger resting
            # on the tabletop must not keep the logical grasp state alive.
            if target_body in bodies and bodies & finger_bodies:
                force = np.zeros(6)
                world.mj.mj_contactForce(model, data, index, force)
                for body in bodies & finger_bodies:
                    forces[body] += abs(float(force[0]))
        position = data.body('link6').xpos.copy()
        rotation = data.body('link6').xmat.reshape(3, 3).copy()
        offset_error = (float(np.linalg.norm(rotation.T @ (data.body(self.target_body).xpos-position)-
                                            self.held_offset_local))
                        if self.holding and self.held_offset_local is not None else None)
        jaw_ids = [model.joint('joint'+str(index)).id for index in (7, 8)]
        jaws = data.qpos[model.jnt_qposadr[jaw_ids]]
        return {'simulated': True, 'session': self.session, 'stamp': time.time(),
                'owner': self.owner, 'fault': self.fault, 'fault_details': self.fault_details,
                'holding': self.holding, 'drive_enabled': self.drive_enabled,
                'held_pose_error_m': offset_error,
                'hold_valid': bool(self.holding and not self.fault and offset_error is not None and offset_error <= .04),
                'snapshot_ready': not self.settling, 'placement_control_version': 4,
                'task_phase': self.task_phase, 'placement_control': self.place_control,
                'physics_fault': self.physics_fault, 'physics_parameters': self.physics_parameters,
                'joints': data.qpos[world.arm_qi].tolist(),
                'velocities': data.qvel[world.arm_vi].tolist(),
                'jaw_positions': jaws.tolist(), 'gripper_width': float(jaws[0]-jaws[1]),
                'contact_force': min(forces.values()),
                'finger_friction': [float(model.geom_friction[model.geom(name).id, 0])
                                    for name in ('link7_collision', 'link8_collision')],
                'grasp_state_consistent': self.holding == (self.held_offset_local is not None),
                'link6_position': position.tolist(), 'link6_rotation': rotation.tolist(),
                'base_position': data.qpos[:3].tolist(), 'base_quaternion': data.qpos[3:7].tolist(),
                'linear_speed': float(np.linalg.norm(data.qvel[:3])),
                'angular_speed': float(np.linalg.norm(data.qvel[3:6])),
                'trajectory': self.result}

    def stop_place_control(self, error=None):
        control = self.place_control
        if control and control['state'] == 'running':
            control.update(state='failed' if error else 'aborted', error=error)
            self.world.arm_target = self.world.data.qpos[self.world.arm_qi].copy()
            if control['mode'] == 'release':
                width = max(0., min(.075, self.feedback()['gripper_width']))
                self.jaw_goal_width = width
                self.world.jaw_target = [width/2, -width/2]
        if error:
            self.result = {'state': 'failed', 'error': error}
            self.task_phase = 'retryable_failure'

    def tick_place_control(self):
        """Contact decisions run at physics rate, independently of web/RPC polling."""
        control = self.place_control
        if not control or control['state'] != 'running':
            return
        if control.get('strategy') == 'wrist_rgbd_geometry':
            self.tick_visual_place_control()
            return
        world, data = self.world, self.world.data
        np = world.np
        def internal_request(method, **values):
            heartbeat = self.last_client
            try:
                return self.request(dict(method=method, session=self.session, owner=self.owner, **values))
            finally:
                self.last_client = heartbeat
        try:
            if self.fault or not self.holding or time.monotonic()-self.last_client > 2.:
                raise RuntimeError(self.fault or 'Placement client disconnected or held object lost')
            if data.time > control['deadline']:
                raise RuntimeError('Placement contact controller timed out; grip maintained')
            if self.arm_collision(data, ignore_held_target=True):
                raise RuntimeError('Placement arm collision; grip maintained')
            sample = internal_request('place_evidence', point_world=control['point_world'])
            control['evidence'] = sample
            good = (sample['upward_force_n'] >= max(.05, .3*sample['weight_n']) and
                    sample['linear_speed_mps'] < .03)
            control['support_since'] = (control['support_since'] if control['support_since'] is not None
                                        else float(data.time)) if good else None
            stable = control['support_since'] is not None and data.time-control['support_since'] >= .3
            if control['mode'] == 'release':
                # Stop opening on the first unsupported sample. A short dropout
                # can recover, but cannot command extra opening while absent.
                if not good:
                    width = max(0., min(.075, self.feedback()['gripper_width']))
                    self.jaw_goal_width = width
                    world.jaw_target = [width/2, -width/2]
                    control['lost_since'] = control.get('lost_since', float(data.time))
                    if data.time-control['lost_since'] > .2:
                        raise RuntimeError('Support lost during release; opening paused')
                    return
                control.pop('lost_since', None)
                if stable:
                    self.jaw_goal_width = .075
                feedback = self.feedback()
                # The simulated jaw joint stops permit 70 mm total travel;
                # the 75 mm command saturates there. Match the release RPC's
                # measured-open tolerance instead of waiting for 73 mm forever.
                if stable and feedback['gripper_width'] >= .0695 and feedback['contact_force'] <= .1:
                    self.holding = False
                    self.held_offset_local = None
                    self.drive_enabled = False
                    self.place_release_active = True
                    control['state'] = 'succeeded'
                    self.result = {'state': 'succeeded'}
                return
            if sample['support_contact'] and control.get('contact_at') is None:
                control['contact_at'] = float(data.time)
                world.arm_target = data.qpos[world.arm_qi].copy()
                control.pop('step', None)
            if control.get('contact_at') is not None:
                if stable:
                    control['state'] = 'succeeded'
                    self.result = {'state': 'succeeded'}
                elif data.time-control['contact_at'] > 1.5:
                    raise RuntimeError('Support contact unstable; descent stopped, grip maintained')
                return
            step = control.get('step')
            if step:
                fraction = min(1., (data.time-step['started'])/.4)
                world.arm_target = np.asarray(step['start'])+fraction*(np.asarray(step['target'])-step['start'])
                if fraction >= 1. and np.max(abs(data.qpos[world.arm_qi]-world.arm_target)) < .003 and np.max(abs(data.qvel[world.arm_vi])) < .02:
                    if step['height']-float(data.body('link6').xpos[2]) < .0005:
                        raise RuntimeError('Descent stalled without support; grip maintained')
                    control.pop('step')
                elif data.time-step['started'] > 3.:
                    raise RuntimeError('Descent tracking stalled; grip maintained')
                return
            if control['steps'] >= 45 or control['start_z']-data.body('link6').xpos[2] >= .13:
                raise RuntimeError('No support within bounded descent; grip maintained')
            step = internal_request('place_step', distance=.003)
            step.update(started=float(data.time), height=float(data.body('link6').xpos[2]))
            control['step'] = step
            control['steps'] += 1
        except Exception as error:
            self.stop_place_control(str(error))

    def tick_visual_place_control(self):
        """Bounded geometric servo using encoder FK and previously observed depth.

        No target body position, contact force, mass or simulator support oracle
        is read here. A reached pose is NOT labelled confirmed contact.
        """
        c, world = self.place_control, self.world
        np, data = world.np, world.data
        try:
            if self.fault or not self.holding or time.monotonic()-self.last_client > 2.:
                raise RuntimeError(self.fault or 'Placement client disconnected or held object lost')
            if data.time > c['deadline']:
                raise RuntimeError('Geometric placement timed out; grip maintained')
            if self.arm_collision(data, ignore_held_target=True):
                raise RuntimeError('Placement arm collision; grip maintained')
            link = data.body('link6')
            bottom = link.xpos+link.xmat.reshape(3, 3) @ np.asarray(c['bottom_offset_local'])
            point = np.asarray(c['point_world'])
            normal = np.asarray(c['normal_world'])
            gap = float((bottom-point) @ normal)
            lateral = float(np.linalg.norm((bottom-point)-gap*normal))
            axis = np.asarray(c.get('axis_local', [0., 0., 1.]))
            upright = link.xmat.reshape(3, 3) @ axis
            tilt = math.acos(max(-1., min(1., float(upright[2]))))
            c['evidence'] = {'source': 'wrist_depth_plane_and_encoder_fk',
                'stamp': float(data.time), 'estimated_bottom_world_m': bottom.tolist(),
                'estimated_gap_m': gap, 'lateral_error_m': lateral,
                'estimated_tilt_rad': tilt,
                'support_confirmed': False}
            if c['mode'] != 'release' and (gap < -.003 or lateral > .045):
                raise RuntimeError('Estimated object pose left the bounded placement corridor; grip maintained')
            if c['mode'] == 'release':
                # Once the fingers open the object may settle independently.
                # Its old rigid hand offset is no longer a release invariant.
                # Hold the measured hand pose instead, including payload-unload
                # correction, without interpreting settling as penetration.
                anchor = np.asarray(c['release_anchor_world'])
                drift = float(np.linalg.norm(link.xpos-anchor))
                c['evidence']['hand_drift_m'] = drift
                if drift > .015 or tilt > .06:
                    raise RuntimeError('Release pose drifted; opening paused')
                step = c.get('hold_step')
                if step:
                    elapsed = data.time-step['started']
                    fraction = min(1., elapsed/.15)
                    world.arm_target = np.asarray(step['start'])+fraction*(np.asarray(step['target'])-step['start'])
                    if elapsed >= .2:
                        c.pop('hold_step')
                elif drift > .001:
                    delta = anchor-link.xpos
                    delta *= min(1., .003/max(1e-9, drift))
                    heartbeat = self.last_client
                    try:
                        step = self.request(dict(method='place_step', session=self.session, owner=self.owner,
                            delta_world=delta.tolist(), orientation_world=c['release_orientation_world'],
                            compensate_tracking=True))
                    finally:
                        self.last_client = heartbeat
                    step['started'] = float(data.time)
                    c['hold_step'] = step
                self.jaw_goal_width = .075
                jaw_ids = [world.model.joint('joint'+str(i)).id for i in (7, 8)]
                jaws = data.qpos[world.model.jnt_qposadr[jaw_ids]]
                width = float(jaws[0]-jaws[1])
                if width >= .0695:
                    self.holding = False
                    self.held_offset_local = None
                    self.drive_enabled = False
                    self.place_release_active = True
                    c['state'] = 'succeeded'
                    self.result = {'state': 'succeeded'}
                return
            # Cartesian task error is the completion condition; a blanket
            # 0.003-rad joint threshold is not meaningful for a 3-mm step.
            good = .002 <= gap <= .010 and lateral <= .012 and tilt <= .04 and np.max(abs(data.qvel[world.arm_vi])) < .08
            c['pose_since'] = (c.get('pose_since') if c.get('pose_since') is not None else float(data.time)) if good else None
            if c['pose_since'] is not None and data.time-c['pose_since'] >= .25:
                c['state'] = 'succeeded'
                self.result = {'state': 'succeeded'}
                return
            step = c.get('step')
            if step:
                elapsed = data.time-step['started']
                fraction = min(1., elapsed/.25)
                world.arm_target = np.asarray(step['start'])+fraction*(np.asarray(step['target'])-step['start'])
                if elapsed >= .6:
                    c.pop('step')
                return
            if good:
                return
            if c['steps'] >= 70 or c['start_z']-link.xpos[2] >= .10:
                raise RuntimeError('Placement pose did not converge within bounded steps; grip maintained')
            delta = point+.006*normal-bottom
            if tilt > .04:
                delta[2] = 0. # Correct upright attitude before descending farther.
            delta *= min(1., .003/max(1e-9, float(np.linalg.norm(delta))))
            heartbeat = self.last_client
            try:
                step = self.request(dict(method='place_step', session=self.session, owner=self.owner,
                                         delta_world=delta.tolist(), orientation_world=c['orientation_world'],
                                         compensate_tracking=True))
            finally:
                self.last_client = heartbeat
            step['started'] = float(data.time)
            c['step'] = step
            c['steps'] += 1
        except Exception as error:
            self.stop_place_control(str(error))

    def before_physics_step(self):
        self.tick_place_control()
        data = self.world.data
        return (data.qpos.copy(), data.qvel.copy(), data.act.copy(), float(data.time),
                [int(w.number) for w in data.warning])

    def after_physics_step(self, checkpoint):
        """Quarantine a divergent step before it reaches renderers or workers."""
        np, model, data = self.world.np, self.world.model, self.world.data
        positions, velocities, activation, stamp, warnings = checkpoint
        bad_warnings = [int(self.world.mj.mjtWarning.mjWARN_INERTIA),
                        int(self.world.mj.mjtWarning.mjWARN_BADQPOS),
                        int(self.world.mj.mjtWarning.mjWARN_BADQVEL),
                        int(self.world.mj.mjtWarning.mjWARN_BADQACC)]
        advanced = [index for index in bad_warnings if int(data.warning[index].number) > warnings[index]]
        invalid = (not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all() or
                   data.time <= stamp or np.max(np.abs(data.qvel)) > 100. or
                   np.max(np.abs(data.qpos-positions)) > .1)
        if not advanced and not invalid:
            return True
        details = {'stage': self.task_phase, 'simulation_time': stamp,
                   'warnings': advanced, 'max_velocity': float(np.max(np.abs(data.qvel)))}
        data.qpos[:] = positions
        data.qvel[:] = velocities
        data.act[:] = activation
        data.time = stamp
        self.world.mj.mj_forward(model, data)
        self.stop_place_control('Physics numerical instability; scene frozen for recovery')
        self.trajectory = None
        self.world.arm_target = data.qpos[self.world.arm_qi].copy()
        self.physics_fault = True
        self.fault = 'Physics numerical instability; reset the scene to continue'
        self.fault_details = details
        self.result = {'state': 'failed', 'error': self.fault}
        try:
            archive = self.workspace/'runs'/('go2-physics-fault-'+str(time.time_ns()))
            archive.mkdir(parents=True)
            (archive/'checkpoint.json').write_text(json.dumps(dict(
                details=details, qpos=positions.tolist(), qvel=velocities.tolist(),
                simulation_time=stamp, physics_parameters=self.physics_parameters), indent=2))
            details['archive'] = str(archive)
        except (OSError, AttributeError) as error:
            details['archive_error'] = str(error)
        return False

    def tick(self):
        world = self.world
        np, data = world.np, world.data
        if self.physics_fault:
            return
        if self.owner and self.jaw_goal_width is not None:
            width = float(world.jaw_target[0]-world.jaw_target[1])
            delta = .02*world.model.opt.timestep
            width += max(-delta, min(delta, self.jaw_goal_width-width))
            world.jaw_target = [width/2, -width/2]
        if self.owner and self.holding and not self.place_release_active and self.initial_bottle is not None:
            target_height = float(data.body(self.target_body).xpos[2])
            link = data.body('link6')
            offset = link.xmat.reshape(3, 3).T @ (data.body(self.target_body).xpos-link.xpos)
            slipped = self.held_offset_local is not None and np.linalg.norm(offset-self.held_offset_local) > .06
            dropped = target_height < min(.34, float(self.initial_bottle[2])-.15) or slipped
            self.drop_since = (data.time if self.drop_since is None else self.drop_since) if dropped else None
            if self.drop_since is not None and data.time-self.drop_since > .5:
                self.holding = False
                self.held_offset_local = None
                self.drive_enabled = False
                self.fault = 'Held object moved away from the gripper'
                self.fault_details = {'target_body': self.target_body, 'target_height_m': target_height,
                                      'initial_height_m': float(self.initial_bottle[2])}
                self.drop_since = None
        still = np.linalg.norm(data.qvel[:3]) < .02 and np.linalg.norm(data.qvel[3:6]) < .03
        self.stable_since = (self.stable_since or time.monotonic()) if still else None
        if self.owner and self.trajectory and time.monotonic()-self.last_client > 2. and not self.fault:
            self.fault = 'Task client heartbeat lost; arm stopped, gripper maintained'
            self.trajectory = None
            self.result = {'state': 'failed', 'error': self.fault}
            world.arm_target = data.qpos[world.arm_qi].copy()
        if self.settling:
            self.stance_history.append((data.time, data.qpos[:7].copy()))
            self.stance_history = [entry for entry in self.stance_history if data.time-entry[0] <= 2.1]
        if self.owner and not self.holding and not self.place_release_active:
            displacement = np.linalg.norm(data.qpos[:3]-self.base_position)
            angle = 2*math.acos(min(1., abs(float(np.dot(data.qpos[3:7], self.base_quaternion)))))
            unsafe_stance = self.preset_active and (data.body('base').xmat[8] < .97 or
                np.linalg.norm(data.qvel[:3]) > .15 or np.linalg.norm(data.qvel[3:6]) > .3)
            if not self.fault and unsafe_stance:
                self.fault = 'Unsafe base stance during preset motion'
                self.fault_details = dict(displacement_m=float(displacement), rotation_rad=float(angle),
                    reference_position=self.base_position.tolist(), measured_position=data.qpos[:3].tolist(),
                    linear_speed=float(np.linalg.norm(data.qvel[:3])), angular_speed=float(np.linalg.norm(data.qvel[3:6])),
                    simulation_time=float(data.time), stage='settling' if self.settling else 'planning_or_motion')
                self.trajectory = None
                self.result = {'state': 'failed', 'error': self.fault}
                world.arm_target = data.qpos[world.arm_qi].copy()
        if self.trajectory:
            if (self.preset_active or self.trajectory_phase == 'place') and self.arm_collision(data):
                self.fault = 'Preset motion stopped: arm collision'
                self.trajectory = None
                self.result = {'state': 'failed', 'error': self.fault}
                world.arm_target = data.qpos[world.arm_qi].copy()
                return
            path, started = self.trajectory
            elapsed = data.time-started
            world.arm_target = np.array(path.sample(elapsed))
            error = float(np.max(np.abs(data.qpos[world.arm_qi]-world.arm_target)))
            tolerance = (.03 if self.preset_active else
                         .008 if self.trajectory_phase == 'approach' else
                         .003 if self.trajectory_phase == 'place' else .025)
            settled = (not self.preset_active and self.trajectory_phase not in ('approach', 'place')) or \
                      float(np.max(np.abs(data.qvel[world.arm_vi]))) < .02
            if elapsed >= path.duration and error < tolerance and settled:
                self.trajectory = None
                self.result = {'state': 'succeeded'}
            elif elapsed > path.duration+(12 if self.preset_active else 8):
                self.trajectory = None
                self.fault = 'Arm tracking tolerance exceeded'
                self.fault_details = {'stage': 'held_return' if self.held_return_active else self.trajectory_phase,
                                      'max_joint_error_rad': error, 'joint_tolerance_rad': tolerance,
                                      'max_joint_speed_rad_s': float(np.max(np.abs(data.qvel[world.arm_vi])))}
                self.result = {'state': 'failed', 'error': self.fault}
                world.arm_target = data.qpos[world.arm_qi].copy()

    def request(self, request):
        from scipy.spatial.transform import Rotation
        method = request['method']
        world = self.world
        np, model, data = world.np, world.model, world.data
        if self.owner and request.get('owner') == self.owner and request.get('session') == self.session:
            self.last_client = time.monotonic()
        if method == 'feedback':
            return self.feedback()
        if method == 'restore_initial':
            if request.get('session') != self.session or request.get('confirm_restore') is not True:
                raise RuntimeError('Restoring the initial state requires explicit confirmation')
            archive = self.workspace/'runs'/('go2-initial-restore-'+str(time.time_ns()))
            archive.mkdir(parents=True, exist_ok=False)
            (archive/'before.json').write_text(json.dumps({
                'session': self.session, 'owner': self.owner, 'fault': self.fault,
                'holding': self.holding, 'qpos': data.qpos.tolist(),
                'qvel': data.qvel.tolist(), 'simulation_time': float(data.time)}, indent=2))
            self.trajectory = None
            self.place_control = None
            self.task_phase = None
            self.physics_fault = False
            self.owner = None
            self.holding = False
            self.drive_enabled = False
            self.preset_active = False
            self.held_return_active = False
            self.place_release_active = False
            self.drop_since = None
            self.held_offset_local = None
            self.settling = False
            self.jaw_goal_width = None
            model.actuator_forcerange[world.jaw_ai] = self.jaw_force_limits
            world.reset()
            self.fault = self.fault_details = self.result = None
            self.stance_history = []
            self.stable_since = None
            self.target_body = 'bottle'
            return {'archive': str(archive), 'world': self.feedback()}
        if method == 'reserve':
            if self.physics_fault:
                raise RuntimeError('Reset the scene after a physics fault')
            if request.get('session') != self.session or not isinstance(request.get('owner'), str) or not request['owner']:
                raise RuntimeError('Grasp session/owner mismatch')
            if self.owner:
                raise RuntimeError('Another grasp owns this world')
            if not self.stable_since or time.monotonic()-self.stable_since < .5:
                raise RuntimeError('Stop the base and wait for measured stability')
            self.owner = request['owner']
            self.task_phase = 'planning'
            self.place_control = None
            self.holding = False
            self.drive_enabled = False
            self.held_return_active = False
            self.place_release_active = False
            self.drop_since = None
            self.held_offset_local = None
            self.jaw_goal_width = float(self.feedback()['gripper_width'])
            model.actuator_forcerange[world.jaw_ai] = [-32., 32.]
            self.last_client = time.monotonic()
            self.base_position, self.base_quaternion = data.qpos[:3].copy(), data.qpos[3:7].copy()
            self.fault = None
            self.fault_details = None
            self.settling = request.get('stabilize') is True
            self.stance_history = []
            if self.settling:
                world.begin_stance_hold()
                self.stance_started = data.time
            self.initial_bottle = data.body('bottle').xpos.copy()
            self.target_body = 'bottle'
            return self.feedback()
        if request.get('session') != self.session or request.get('owner') != self.owner or not self.owner:
            raise RuntimeError('Grasp session/owner mismatch')
        self.last_client = time.monotonic()
        if method in ('trajectory', 'gripper', 'preset', 'place_plan') and self.place_control and self.place_control['state'] == 'running':
            raise RuntimeError('Simulation contact controller owns arm and gripper; abort it first')
        if method == 'recover_fault':
            if self.physics_fault:
                raise RuntimeError('Physics fault requires scene reset')
            if (self.trajectory or self.feedback()['contact_force'] > .1 or
                    (self.holding and (self.initial_bottle is None or
                     data.body(self.target_body).xpos[2] > self.initial_bottle[2]+.02))):
                raise RuntimeError('Task recovery requires no held-object contact and a stopped trajectory')
            archive = self.workspace/'runs'/('go2-fault-recovery-'+str(time.time_ns()))
            archive.mkdir(parents=True, exist_ok=False)
            (archive/'before.json').write_text(json.dumps({'session': self.session, 'owner': self.owner,
                'fault': self.fault, 'fault_details': self.fault_details,
                'qpos': data.qpos.tolist(), 'simulation_time': float(data.time)}, indent=2))
            self.owner = None
            self.place_control = None
            self.task_phase = None
            self.holding = False
            self.fault = self.fault_details = self.result = None
            self.drive_enabled = self.held_return_active = self.place_release_active = False
            self.preset_active = self.settling = False
            self.drop_since = self.jaw_goal_width = self.held_offset_local = None
            model.actuator_forcerange[world.jaw_ai] = self.jaw_force_limits
            world.jaw_target = [.0375, -.0375]
            world.end_stance_hold()
            return self.feedback()
        if method in ('abort', 'release'):
            if method == 'release' and self.feedback()['contact_force'] > .1:
                raise RuntimeError('Object may be held; keep ownership and gripper state')
            self.stop_place_control()
            self.task_phase = 'holding' if self.holding else 'retryable_failure'
            self.trajectory = None
            self.preset_active = False
            self.held_return_active = False
            world.arm_target = data.qpos[world.arm_qi].copy()
            self.result = {'state': 'aborted'}
            if method == 'release':
                self.owner = None
                self.task_phase = None
                self.holding = False
                self.held_offset_local = None
                self.drive_enabled = False
                self.held_return_active = False
                self.place_release_active = False
                self.settling = False
                self.jaw_goal_width = None
                model.actuator_forcerange[world.jaw_ai] = self.jaw_force_limits
                world.end_stance_hold()
            return self.feedback()
        if method == 'task_phase':
            if request.get('phase') not in ('planning', 'approaching', 'closing', 'lifting',
                    'holding', 'place_descending', 'releasing', 'retreating', 'returning_zero'):
                raise ValueError('Unknown task phase')
            self.task_phase = request['phase']
            return self.feedback()
        if method in ('place_descend', 'place_release'):
            if self.fault or not self.holding or self.trajectory or self.settling or \
                    (self.place_control and self.place_control['state'] == 'running'):
                raise RuntimeError('Contact control requires a healthy stationary held arm')
            point = request.get('point_world')
            if point is None or len(point) != 3 or not np.isfinite(point).all():
                raise ValueError('A finite RGB-D support point is required')
            if method == 'place_release' and (not self.place_control or
                    self.place_control['mode'] != 'descend' or self.place_control['state'] != 'succeeded' or
                    point != self.place_control['point_world']):
                raise RuntimeError('Release requires completed support confirmation at the same point')
            previous = self.place_control or {}
            visual = request.get('strategy') == 'wrist_rgbd_geometry' or (method == 'place_release' and
                        previous.get('strategy') == 'wrist_rgbd_geometry')
            geometry = {}
            if visual:
                offset = request.get('bottom_offset_local', previous.get('bottom_offset_local'))
                normal = request.get('normal_world', previous.get('normal_world'))
                if (np.asarray(offset).shape != (3,) or np.asarray(normal).shape != (3,) or
                        not np.isfinite(offset).all() or not np.isfinite(normal).all() or
                        not .02 <= np.linalg.norm(offset) <= .4 or abs(np.linalg.norm(normal)-1.) > .01 or normal[2] < .985):
                    raise ValueError('Valid wrist RGB-D geometry is required for visual placement')
                geometry = dict(strategy='wrist_rgbd_geometry', bottom_offset_local=list(offset),
                                normal_world=list(normal),
                                orientation_world=request.get('orientation_world', previous.get('orientation_world')),
                                axis_local=request.get('axis_local', previous.get('axis_local')))
                orientation = np.asarray(geometry['orientation_world'], dtype=float)
                axis = np.asarray(geometry['axis_local'], dtype=float)
                if (orientation.shape != (3, 3) or not np.isfinite(orientation).all() or
                        not np.allclose(orientation.T @ orientation, np.eye(3), atol=1e-4) or np.linalg.det(orientation) < .99 or
                        axis.shape != (3,) or not np.isfinite(axis).all() or abs(np.linalg.norm(axis)-1.) > .01):
                    raise ValueError('Observed upright object axis and checked placement orientation are required')
                if method == 'place_release':
                    link = data.body('link6')
                    bottom = link.xpos+link.xmat.reshape(3, 3) @ np.asarray(offset)
                    gap = float((bottom-np.asarray(point)) @ np.asarray(normal))
                    lateral = float(np.linalg.norm((bottom-np.asarray(point))-gap*np.asarray(normal)))
                    tilt = math.acos(max(-1., min(1., float((link.xmat.reshape(3, 3) @ axis)[2]))))
                    if not -.003 <= gap <= .018 or lateral > .02 or tilt > .06:
                        raise RuntimeError('Release preflight pose changed; fingers remain closed')
                    # The last descent step may still target a lower joint pose
                    # after its Cartesian completion condition has succeeded.
                    # Freeze at measured joints before unloading the gripper;
                    # release-mode Cartesian correction then holds this anchor.
                    world.arm_target = data.qpos[world.arm_qi].copy()
                    geometry.update(release_anchor_world=link.xpos.tolist(),
                                    release_orientation_world=link.xmat.reshape(3, 3).tolist())
            self.task_phase = 'place_descending' if method == 'place_descend' else 'releasing'
            self.place_control = {'mode': 'descend' if method == 'place_descend' else 'release',
                'state': 'running', 'point_world': list(point), 'support_since': None,
                'deadline': float(data.time)+(45. if method == 'place_descend' else 12.),
                'steps': 0, 'start_z': float(data.body('link6').xpos[2]), **geometry}
            self.result = {'state': 'running'}
            return self.feedback()
        if method == 'evidence':
            feedback = self.feedback()
            feedback['target_body'] = self.target_body
            feedback['bottle_lift'] = float(data.body(self.target_body).xpos[2]-self.initial_bottle[2])
            feedback['target_position_world_m'] = data.body(self.target_body).xpos.tolist()
            return feedback
        if method == 'place_evidence':
            target = model.body(self.target_body).id
            point = request.get('point_world')
            if point is not None and (len(point) != 3 or not np.isfinite(point).all()):
                raise ValueError('Invalid placement support point')
            upward = 0.
            contacts = []
            robot_root = model.body_rootid[model.body('link6').id]
            for index, contact in enumerate(data.contact[:data.ncon]):
                first, second = (int(model.geom_bodyid[g]) for g in (contact.geom1, contact.geom2))
                if target not in (first, second):
                    continue
                other = second if first == target else first
                if model.body_rootid[other] == robot_root:
                    continue
                if point is not None and (abs(contact.pos[2]-point[2]) > .015 or
                        np.linalg.norm(contact.pos[:2]-np.asarray(point[:2])) > .08):
                    continue
                force = np.zeros(6)
                world.mj.mj_contactForce(model, data, index, force)
                sign = 1. if second == target else -1.
                normal = sign*contact.frame.reshape(3, 3)[0]
                force_world = sign*contact.frame.reshape(3, 3).T @ force[:3]
                if normal[2] < .8 or force_world[2] <= 0:
                    continue
                upward += float(force_world[2])
                contacts.append(contact.pos.tolist())
            weight = float(model.body_mass[target]*np.linalg.norm(model.opt.gravity))
            return {'stamp': float(data.time), 'body': self.target_body,
                    'position_world_m': data.body(self.target_body).xpos.tolist(),
                    'linear_speed_mps': float(np.linalg.norm(data.cvel[target, 3:])),
                    'support_contact': upward > 0., 'upward_force_n': upward,
                    'weight_n': weight, 'support_points_world_m': contacts,
                    'gripper_contact_n': self.feedback()['contact_force']}
        if method == 'place_step':
            if not self.holding or self.trajectory or self.fault or self.settling:
                raise RuntimeError('Placement step requires a healthy stationary held arm')
            from scipy.optimize import least_squares
            from scipy.spatial.transform import Rotation
            distance = float(request.get('distance', .003))
            if not 0 < distance <= .003:
                raise ValueError('Placement descent step must be at most 3 mm')
            scratch = world.mj.MjData(model)
            world.mj.mj_copyData(scratch, model, data)
            start = data.qpos[world.arm_qi].copy()
            link = data.body('link6')
            position = link.xpos.copy()-np.array([0., 0., distance])
            orientation = link.xmat.reshape(3, 3).copy()
            if 'delta_world' in request:
                delta = np.asarray(request['delta_world'], dtype=float)
                rotation = np.asarray(request.get('orientation_world'), dtype=float)
                if (delta.shape != (3,) or not np.isfinite(delta).all() or np.linalg.norm(delta) > .003001 or
                        rotation.shape != (3, 3) or not np.isfinite(rotation).all() or
                        not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-4) or np.linalg.det(rotation) < .99):
                    raise ValueError('Invalid bounded visual placement step')
                position = link.xpos.copy()+delta
                orientation = rotation
            limits = model.jnt_range[[model.joint('joint'+str(i)).id for i in range(1, 7)]]
            def residual(joints):
                scratch.qpos[world.arm_qi] = joints
                world.mj.mj_forward(model, scratch)
                current = scratch.body('link6')
                return np.r_[current.xpos-position,
                    .15*Rotation.from_matrix(orientation.T @ current.xmat.reshape(3, 3)).as_rotvec()]
            solution = least_squares(residual, np.clip(start, limits[:, 0]+1e-6, limits[:, 1]-1e-6),
                bounds=(limits[:, 0], limits[:, 1]), max_nfev=80)
            error = residual(solution.x)
            if np.linalg.norm(error[:3]) > .0008 or np.linalg.norm(error[3:]) > .003:
                raise RuntimeError('Small placement step is not reachable; grip maintained')
            target = solution.x
            if request.get('compensate_tracking'):
                # Bounded integral correction for payload tracking error. Use
                # encoder error, never object mass/contact truth, and check the
                # actual compensated command against limits and collisions.
                correction = np.clip(np.asarray(world.arm_target)-start, -.04, .04)
                target = np.clip(target+correction, limits[:, 0], limits[:, 1])
                target = start+np.clip(target-start, -.05, .05)
            for fraction in np.linspace(0., 1., max(3, int(np.max(abs(target-start))/.01)+2)):
                residual(start+fraction*(target-start))
                if self.arm_collision(scratch, ignore_held_target=True):
                    raise RuntimeError('Small placement step collides; grip maintained')
            return {'start': start.tolist(), 'target': target.tolist()}
        if method == 'place_pause_opening':
            # This stop remains available during a fault; never finish opening
            # merely because the placement worker has exited.
            width = max(0., min(.075, self.feedback()['gripper_width']))
            self.jaw_goal_width = width
            world.jaw_target = [width/2, -width/2]
            return self.feedback()
        if method == 'mark_place_released':
            feedback = self.feedback()
            if not self.holding or feedback['gripper_width'] < .0695 or feedback['contact_force'] > .1:
                raise RuntimeError('Placement release lacks open-gripper feedback')
            self.holding = False
            self.held_offset_local = None
            self.drive_enabled = False
            self.place_release_active = True
            return feedback
        if method == 'hold_confirmed':
            if self.fault or (self.result or {}).get('state') != 'succeeded':
                raise RuntimeError('Cannot confirm holding without completed lift and healthy world')
            evidence = self.request(dict(request, method='evidence'))
            if evidence['bottle_lift'] < .05:
                raise RuntimeError('Cannot confirm holding without measured object lift')
            self.held_offset_local = (np.asarray(evidence['link6_rotation']).reshape(3, 3).T @
                                      (np.asarray(evidence['target_position_world_m'])-
                                       np.asarray(evidence['link6_position'])))
            self.holding = True
            self.task_phase = 'holding'
            return evidence
        if method == 'enable_drive':
            if (not self.holding or not self.held_return_active or self.fault or self.trajectory or
                    (self.result or {}).get('state') != 'succeeded' or self.settling):
                raise RuntimeError('Driving requires a completed healthy held grasp')
            evidence = self.request(dict(request, method='evidence'))
            if not evidence['hold_valid']:
                raise RuntimeError('Held object evidence is missing; driving remains disabled')
            world.end_stance_hold()
            self.drive_enabled = True
            return self.feedback()
        if method == 'place_plan':
            if not self.holding or self.trajectory or self.fault or self.settling:
                raise RuntimeError('Placement requires a stable, confirmed held object')
            if not self.feedback()['hold_valid']:
                raise RuntimeError('Held object relative-pose evidence is missing; placement is refused')
            if np.linalg.norm(data.qvel[:3]) > .015 or np.linalg.norm(data.qvel[3:6]) > .025:
                raise RuntimeError('Stop the base before placement planning')
            return placement_joint_targets(self, request['point_world'], request['bottom_offset_local'], request.get('axis_local'))
        if self.fault:
            raise RuntimeError(self.fault)
        if method == 'planning_snapshot':
            if self.settling and data.time-self.stance_started >= 3. and len(self.stance_history) >= 2:
                history = self.stance_history
                recent = [pose for stamp, pose in history if data.time-stamp <= 2.]
                moved = max(np.linalg.norm(pose[:3]-data.qpos[:3]) for pose in recent)
                rotated = max(2*math.acos(min(1., abs(float(np.dot(pose[3:7], data.qpos[3:7]))))) for pose in recent)
                if (data.time-history[0][0] >= 2. and moved < .002 and rotated < .005 and
                        np.linalg.norm(data.qvel[:3]) < .005 and np.linalg.norm(data.qvel[3:6]) < .01):
                    self.base_position, self.base_quaternion = data.qpos[:3].copy(), data.qpos[3:7].copy()
                    self.settling = False
            return self.feedback()
        if self.settling:
            raise RuntimeError('Wait for the stable planning snapshot before commanding the arm')
        if method == 'preset':
            if self.trajectory:
                raise RuntimeError('Arm trajectory already active')
            held_return = request.get('held_return') is True
            if self.feedback()['contact_force'] > .1 and not (held_return and self.holding):
                raise RuntimeError('Object may be held; preset motion is refused')
            target = np.asarray(request['positions'], dtype=float)
            if target.shape != (6,) or not np.isfinite(target).all():
                raise ValueError('Invalid preset target')
            if held_return and (not self.holding or not np.allclose(target, 0., atol=1e-8)):
                raise RuntimeError('Held return is allowed only for confirmed grasp to reference zero')
            reference = target.copy()
            import xml.etree.ElementTree as ET
            urdf = ET.parse(self.workspace/'sources/A2_loco_manipulation/third_party/piper_ros/src/piper_description/urdf/piper_description.urdf').getroot()
            for index in range(6):
                joint = urdf.find("joint[@name='joint%d']" % (index+1))
                origin = joint.find('origin')
                body = model.body('link'+str(index+1)).id
                position = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
                desired = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' '))
                actual = Rotation.from_quat(np.roll(model.body_quat[body], -1))
                delta = (actual.inv()*desired).as_rotvec()
                joint_id = model.joint('joint'+str(index+1)).id
                if (not np.allclose(model.body_pos[body], position, atol=1e-6) or
                        not np.allclose(model.jnt_axis[joint_id], [0, 0, 1], atol=1e-6) or
                        not np.allclose(np.fromstring(joint.find('axis').get('xyz'), sep=' '), [0, 0, 1]) or
                        np.linalg.norm(delta[:2]) > 1e-4):
                    raise RuntimeError('Reference zero cannot be mapped to the current arm model')
                target[index] += delta[2]
            start = data.qpos[world.arm_qi].copy()
            joint_ids = [model.joint('joint'+str(index)).id for index in range(1, 7)]
            limits = model.jnt_range[joint_ids]
            if np.any(target < limits[:, 0]) or np.any(target > limits[:, 1]):
                raise ValueError('Preset exceeds joint limits')
            snapshot = world.mj.MjData(model)
            world.mj.mj_copyData(snapshot, model, data)
            for fraction in np.linspace(0, 1, max(2, int(np.max(np.abs(target-start))/.005)+2)):
                snapshot.qpos[world.arm_qi] = start+(target-start)*fraction
                world.mj.mj_forward(model, snapshot)
                if self.arm_collision(snapshot):
                    raise RuntimeError('Preset path collides; no motion was started')
            duration = (max(4., float(np.max(np.abs(target-start)))/.12) if held_return else
                        max(2.5, float(np.max(np.abs(target-start)))/.24))
            hold_delay = .5 if held_return else 0.
            if held_return:
                # Preserve the original closed-jaw target. Recomputing it from the
                # measured opening would loosen the grip under load.
                if self.jaw_goal_width is None:
                    raise RuntimeError('Held return requires an active gripper target')
                model.actuator_forcerange[world.jaw_ai] = [-24., 24.]
            end_time = duration+hold_delay
            waypoints = [dict(positions=start.tolist(), sec=0, nanosec=0)]
            if held_return:
                waypoints.append(dict(positions=start.tolist(), sec=0, nanosec=int(hold_delay*1e9)))
            waypoints.append(dict(positions=target.tolist(), sec=int(end_time),
                                  nanosec=int(end_time % 1*1e9)))
            result = self.request(dict(request, method='trajectory',
                joint_names=['joint'+str(index) for index in range(1, 7)],
                points=waypoints))
            self.preset_active = True
            self.task_phase = 'returning_zero' if held_return or np.allclose(reference, 0.) else 'approaching'
            self.held_return_active = held_return
            return dict(result, reference_positions=reference.tolist(), model_positions=target.tolist(),
                        gripper_policy='preserve closed-jaw target during held return' if held_return else 'preserve current opening; no automatic release')
        if method == 'export':
            result = export_model(world, Path(request['directory']), self.workspace,
                                  selected_world=request.get('selected_world'))
            self.target_body = result['target_body']
            self.initial_bottle = data.body(self.target_body).xpos.copy()
            return result
        if method == 'export_place_model':
            # MTC runs twice: before release with a held object and after
            # release with the object resting in the world. Keep the same
            # target identity for retreat planning after the fingers open.
            if not self.owner or not self.target_body or not (self.holding or self.place_release_active):
                raise RuntimeError('MTC placement model requires an active placement task')
            return export_model(world, Path(request['directory']), self.workspace,
                                selected_body=self.target_body)
        if method == 'gripper':
            width = request['width']
            if type(width) not in (int, float) or not math.isfinite(width) or not 0 <= width <= .075:
                raise ValueError('Gripper opening outside simulated limits')
            if self.trajectory:
                raise RuntimeError('Do not change jaw geometry during an arm trajectory')
            self.jaw_goal_width = float(width)
            self.task_phase = 'closing' if width < self.feedback()['gripper_width'] else 'approaching'
            return self.feedback()
        if method == 'trajectory':
            from types import SimpleNamespace
            from integration.arm_trajectory import JointPath
            if self.trajectory:
                raise RuntimeError('Arm trajectory already active')
            if len(request['joint_names']) != 6 or set(request['joint_names']) != {'joint'+str(index) for index in range(1, 7)}:
                raise ValueError('Only the six Piper arm joints may be commanded')
            points = [SimpleNamespace(positions=point['positions'],
                      time_from_start=SimpleNamespace(sec=point['sec'], nanosec=point['nanosec']))
                      for point in request['points']]
            if len(points) > 10000:
                raise ValueError('Oversized trajectory')
            trajectory = SimpleNamespace(joint_names=request['joint_names'], points=points)
            joint_ids = [model.joint('joint'+str(index)).id for index in range(1, 7)]
            path = JointPath(trajectory, data.qpos[world.arm_qi], model.jnt_range[joint_ids], max_duration=60)
            phase = request.get('phase')
            if phase not in (None, 'pregrasp', 'approach', 'lift', 'place'):
                raise ValueError('Unknown trajectory phase')
            self.trajectory = (path, data.time)
            self.trajectory_phase = phase
            if phase in ('pregrasp', 'approach', 'lift'):
                self.task_phase = {'pregrasp': 'approaching', 'approach': 'approaching', 'lift': 'lifting'}[phase]
            self.preset_active = False
            self.held_return_active = False
            self.result = {'state': 'running'}
            return self.feedback()
        raise ValueError('Unknown grasp operation')

    def arm_collision(self, data, ignore_held_target=False):
        model = self.world.model
        arm = {model.body('link'+str(index)).id for index in range(1, 9)}
        fingers = {model.body('link7').id, model.body('link8').id}
        held_target = model.body(self.target_body).id if self.holding else None
        for contact in data.contact[:data.ncon]:
            first, second = (int(model.geom_bodyid[index]) for index in (contact.geom1, contact.geom2))
            if not ({first, second} & arm) or contact.dist > -.001:
                continue
            if ignore_held_target and held_target in (first, second):
                continue
            if held_target in (first, second) and ({first, second}-{held_target}) & fingers:
                continue
            if first == second or model.body_parentid[first] == second or model.body_parentid[second] == first:
                continue
            return True
        return False


def export_model(world, directory, workspace, selected_world=None, selected_body=None):
    import struct
    import xml.etree.ElementTree as ET
    from scipy.spatial.transform import Rotation
    np, model, data = world.np, world.model, world.data
    target_body = 'bottle'
    if selected_body is not None:
        if selected_body != 'bottle' and not selected_body.startswith('tabletop_'):
            raise ValueError('Unsupported held placement object')
        model.body(selected_body)
        target_body = selected_body
    if selected_world is not None:
        point = np.asarray(selected_world, dtype=float)
        if point.shape != (3,) or not np.isfinite(point).all():
            raise ValueError('Selected target requires a finite world XYZ point')
        matches = []
        for body_id in range(1, model.nbody):
            name = model.body(body_id).name
            if name != 'bottle' and not name.startswith('tabletop_'):
                continue
            center = data.xpos[body_id]
            horizontal = float(np.linalg.norm(point[:2]-center[:2]))
            vertical = abs(float(point[2]-center[2]))
            if horizontal <= .06 and vertical <= .12:
                matches.append((horizontal, name))
        if len(matches) != 1:
            raise RuntimeError('Selected depth target does not match exactly one simulated tabletop object; '
                               'refresh RGB-D and select the bottle again')
        target_body = matches[0][1]
    directory = directory.resolve()
    if (workspace/'runs').resolve() not in directory.parents:
        raise ValueError('Planning artifacts must remain in this workspace runs directory')
    directory.mkdir(parents=True, exist_ok=True)
    meshes = directory/'meshes'
    meshes.mkdir(exist_ok=True)
    robot = ET.Element('robot', name='go2_piper_live')
    ET.SubElement(robot, 'link', name='world')
    links, parents = {0: 'world'}, {}
    text = lambda values: ' '.join(map(str, values))

    def collision(link, index, position, rotation):
        element = ET.SubElement(link, 'collision')
        ET.SubElement(element, 'origin', xyz=text(position), rpy=text(Rotation.from_matrix(rotation).as_euler('xyz')))
        geometry = ET.SubElement(element, 'geometry')
        kind, size = int(model.geom_type[index]), model.geom_size[index]
        if kind == 7:
            mesh_id = int(model.geom_dataid[index])
            path = meshes/(str(mesh_id)+'.stl')
            if not path.exists():
                vertices = model.mesh_vert[model.mesh_vertadr[mesh_id]:model.mesh_vertadr[mesh_id]+model.mesh_vertnum[mesh_id]]
                faces = model.mesh_face[model.mesh_faceadr[mesh_id]:model.mesh_faceadr[mesh_id]+model.mesh_facenum[mesh_id]]
                with path.open('wb') as output:
                    output.write(b'Live Go2 collision mesh'.ljust(80, b'\0')+struct.pack('<I', len(faces)))
                    for face in faces:
                        triangle = vertices[face]
                        normal = np.cross(triangle[1]-triangle[0], triangle[2]-triangle[0])
                        normal /= max(float(np.linalg.norm(normal)), 1e-20)
                        output.write(struct.pack('<12fH', *normal, *triangle.flatten(), 0))
            ET.SubElement(geometry, 'mesh', filename=path.as_uri())
        elif kind == 6:
            ET.SubElement(geometry, 'box', size=text(2*size))
        elif kind == 2:
            ET.SubElement(geometry, 'sphere', radius=str(size[0]))
        elif kind == 4:
            ET.SubElement(geometry, 'sphere', radius=str(float(np.max(size))))
        elif kind in (3, 5):
            ET.SubElement(geometry, 'cylinder', radius=str(size[0]), length=str(2*(size[1]+(size[0] if kind == 3 else 0))))
        elif kind == 0:
            ET.SubElement(geometry, 'box', size='100 100 .02')
            element.find('origin').set('xyz', text(np.asarray(position)-np.array([0, 0, .01])))
        else:
            raise ValueError('Unsupported collision geometry: '+str(kind))

    arm_names = {'joint'+str(index) for index in range(1, 9)}
    for body_id in range(1, model.nbody):
        name = model.body(body_id).name
        if name == target_body:
            continue
        parent_id = int(model.body_parentid[body_id])
        if parent_id not in links:
            raise ValueError('Unsupported free object in robot collision tree')
        links[body_id] = name
        parent = links[parent_id]
        parents[name] = parent
        link = ET.SubElement(robot, 'link', name=name)
        for index in range(model.ngeom):
            if model.geom_bodyid[index] == body_id and (model.geom_contype[index] or model.geom_conaffinity[index]):
                collision(link, index, model.geom_pos[index], Rotation.from_quat(model.geom_quat[index][[1, 2, 3, 0]]).as_matrix())
        joint_id = int(model.body_jntadr[body_id])
        active = bool(model.body_jntnum[body_id]) and model.joint(joint_id).name in arm_names
        joint = ET.SubElement(robot, 'joint', name=model.joint(joint_id).name if active else name+'_fixed',
                              type=('prismatic' if model.jnt_type[joint_id] == 2 else 'revolute') if active else 'fixed')
        ET.SubElement(joint, 'parent', link=parent)
        ET.SubElement(joint, 'child', link=name)
        if active:
            if np.linalg.norm(model.jnt_pos[joint_id]) > 1e-8:
                raise ValueError('Unsupported nonzero arm joint origin')
            position, rotation = model.body_pos[body_id], Rotation.from_quat(model.body_quat[body_id][[1, 2, 3, 0]]).as_matrix()
            ET.SubElement(joint, 'axis', xyz=text(model.jnt_axis[joint_id]))
            ET.SubElement(joint, 'limit', lower=str(model.jnt_range[joint_id, 0]), upper=str(model.jnt_range[joint_id, 1]), effort='100', velocity='1.2')
        else:
            parent_rotation = np.eye(3) if parent_id == 0 else data.xmat[parent_id].reshape(3, 3)
            parent_position = np.zeros(3) if parent_id == 0 else data.xpos[parent_id]
            position = parent_rotation.T@(data.xpos[body_id]-parent_position)
            rotation = parent_rotation.T@data.xmat[body_id].reshape(3, 3)
        ET.SubElement(joint, 'origin', xyz=text(position), rpy=text(Rotation.from_matrix(rotation).as_euler('xyz')))
    environment = ET.SubElement(robot, 'link', name='hssd_collision_world')
    fixed = ET.SubElement(robot, 'joint', name='hssd_collision_fixed', type='fixed')
    ET.SubElement(fixed, 'parent', link='world')
    ET.SubElement(fixed, 'child', link='hssd_collision_world')
    for index in range(model.ngeom):
        if model.geom_bodyid[index] == 0 and (model.geom_contype[index] or model.geom_conaffinity[index]):
            # Conservative broad-phase: a rigid world geom whose bounding sphere
            # is beyond the Piper reach plus margin cannot contact the arm.
            separation = float(np.linalg.norm(data.geom_xpos[index]-data.body('arm_base').xpos)
                               - model.geom_rbound[index])
            if separation > 1.1:
                continue
            collision(environment, index, data.geom_xpos[index], data.geom_xmat[index].reshape(3, 3))
    semantic = ET.Element('robot', name='go2_piper_live')
    group = ET.SubElement(semantic, 'group', name='arm')
    ET.SubElement(group, 'chain', base_link='arm_base', tip_link='link6')
    for child, parent in parents.items():
        ET.SubElement(semantic, 'disable_collisions', link1=child, link2=parent, reason='Adjacent')
    def assembly(name):
        while name != 'world':
            body_id = model.body(name).id
            joint_id = int(model.body_jntadr[body_id])
            if model.body_jntnum[body_id] and model.joint(joint_id).name in arm_names:
                return name
            name = parents[name]
        return 'world'
    names = list(parents)
    for name in names:
        if assembly(name) == 'world':
            ET.SubElement(semantic, 'disable_collisions', link1=name, link2='hssd_collision_world', reason='RigidAssembly')
    for index, name in enumerate(names):
        for other in names[index+1:]:
            if assembly(name) == assembly(other):
                ET.SubElement(semantic, 'disable_collisions', link1=name, link2=other, reason='RigidAssembly')
    aliases = {'link7': 'p_link7', 'link8': 'p_link8'}
    for link in robot.findall('link'):
        link.set('name', aliases.get(link.get('name'), link.get('name')))
    for joint in robot.findall('joint'):
        for tag in ('parent', 'child'):
            element = joint.find(tag)
            element.set('link', aliases.get(element.get('link'), element.get('link')))
    for exclusion in semantic.findall('disable_collisions'):
        for key in ('link1', 'link2'):
            exclusion.set(key, aliases.get(exclusion.get(key), exclusion.get(key)))
    description = ET.tostring(robot, encoding='unicode')
    srdf = ET.tostring(semantic, encoding='unicode')
    (directory/'robot.urdf').write_text(description)
    (directory/'robot.srdf').write_text(srdf)
    return {'robot_description': description, 'robot_description_semantic': srdf,
            'arm_origin': data.body('arm_base').xpos.tolist(), 'target_body': target_body}


def run_pose(request_path):
    import hashlib
    from integration.rpc import call
    options = json.loads(Path(request_path).read_text())
    directory = Path(options['directory'])
    report = {'passed': False, 'operation': options['operation'], 'simulated': True}
    reserved = False
    def rpc(method, **values):
        return call(options['socket'], method, session=options['session'], owner=options['owner'], **values)
    try:
        if options['operation'] == 'zero':
            angles = [0.]*6
        elif options['operation'] == 'observation':
            source = Path(options['source'])/'config/robots/a2_piper/observation.json'
            raw = source.read_bytes()
            source_angles = json.loads(raw)['joint_deg']
            from integration.paths import REPOSITORY
            profile = REPOSITORY/'profiles/hssd-go2-piper.json'
            profile_raw = profile.read_bytes()
            angles = json.loads(profile_raw).get('observation_pose_deg', source_angles)
            report.update(source=str(source), source_sha256=hashlib.sha256(raw).hexdigest(),
                          source_joint_deg=source_angles, profile=str(profile),
                          profile_sha256=hashlib.sha256(profile_raw).hexdigest())
        else:
            raise ValueError('Unknown pose')
        if len(angles) != 6 or any(type(value) not in (int, float) or not math.isfinite(value) for value in angles):
            raise ValueError('Invalid source joint angles')
        reference = Path(options['workspace'])/'sources/A2_loco_manipulation/third_party/piper_ros/src/piper_description/urdf/piper_description.urdf'
        report.update(reference_urdf=str(reference), reference_sha256=hashlib.sha256(reference.read_bytes()).hexdigest(),
                      joint_deg=angles, gripper_policy='unchanged')
        # The restored simulator needs consecutive measured base samples before
        # it can reserve arm control. A request made immediately after startup
        # should wait for that short settling period instead of failing a task.
        reserve_deadline = time.monotonic()+12
        while True:
            try:
                rpc('reserve')
                break
            except RuntimeError as error:
                if (str(error) != 'Stop the base and wait for measured stability'
                        or time.monotonic() >= reserve_deadline):
                    raise
                time.sleep(.15)
        reserved = True
        report['motion'] = rpc('preset', positions=[math.radians(value) for value in angles])
        deadline = time.monotonic()+75
        while time.monotonic() < deadline:
            feedback = rpc('feedback')
            if feedback['fault']:
                raise RuntimeError(feedback['fault'])
            state = (feedback.get('trajectory') or {}).get('state')
            if state == 'succeeded':
                report['final_feedback'] = feedback
                rpc('release')
                reserved = False
                report['passed'] = True
                break
            if state != 'running':
                raise RuntimeError('Preset motion did not complete: '+str(state))
            time.sleep(.15)
        else:
            raise RuntimeError('Preset motion timed out')
    except Exception as error:
        report['error'] = str(error)
        if reserved:
            try:
                rpc('abort')
            except Exception:
                pass
    finally:
        (directory/'result.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report), flush=True)
def run_place(request_path):
    from integration.rpc import call
    import os
    import subprocess
    import sys
    options = json.loads(Path(request_path).read_text())
    directory = Path(options['directory'])
    report = {'passed': False, 'operation': 'place', 'simulated': True,
              'target_source': 'link6 wrist RGB-D and calibrated encoder FK',
              'strategy': 'wrist_rgbd_geometry', 'force_sensor_required': False}
    phase = 'preflight'
    motion_samples = []
    def rpc(method, **values):
        return call(options['socket'], method, session=options['session'], owner=options['owner'], **values)
    def wait_motion(timeout=60):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            feedback = rpc('feedback')
            motion_samples.append({'phase': phase, 'stamp': feedback['stamp'],
                                   'contact_force_n': feedback['contact_force'],
                                   'gripper_width_m': feedback['gripper_width'],
                                   'joints': feedback['joints'],
                                   'trajectory': feedback.get('trajectory')})
            if feedback['fault']:
                raise RuntimeError('Placement motion stopped: '+feedback['fault'])
            state = (feedback.get('trajectory') or {}).get('state')
            if state == 'succeeded':
                return feedback
            if state != 'running':
                raise RuntimeError('Placement trajectory failed: '+str(state))
            time.sleep(.08)
        raise TimeoutError('Placement trajectory timed out')
    def mtc_paths(phase, feedback, above=None):
        from integration.paths import ROOT
        import hashlib
        source = ROOT/'mtc_place'
        fingerprint = hashlib.sha256(b''.join(path.read_bytes() for path in
            (source/'CMakeLists.txt', source/'package.xml', source/'src'/'place.cpp'))).hexdigest()[:16]
        build_root = Path(options['directory']).parents[1]/'builds'
        build = build_root/('mtc-place-'+fingerprint)
        binary = build/'install'/'go2_place_mtc'/'lib'/'go2_place_mtc'/'go2_place_mtc'
        if not binary.is_file():
            build.mkdir(parents=True, exist_ok=True)
            command = ['bash', '-c',
                'set -e; source /opt/ros/humble/setup.bash; '
                'colcon --log-base "$1/log" build --base-paths "$2" '
                '--build-base "$1/build" --install-base "$1/install" '
                '--packages-select go2_place_mtc',
                'go2-place-build', str(build), str(source)]
            with (directory/'mtc-build.log').open('w') as log:
                result = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                        timeout=180, check=False)
            if result.returncode or not binary.is_file():
                raise RuntimeError('MoveIt Task Constructor build failed; see '+str(directory/'mtc-build.log'))
        exported = rpc('export_place_model', directory=str(directory))
        (directory/'mtc-model.json').write_text(json.dumps(exported))
        config = {'robot_description': exported['robot_description'],
                  'robot_description_semantic': exported['robot_description_semantic'],
                  'robot_description_kinematics': {'arm': {
                      'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
                      'kinematics_solver_timeout': .2,
                      'kinematics_solver_search_resolution': .005}},
                  'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl',
                  'ompl': {'planning_plugin': 'ompl_interface/OMPLPlanner',
                           'planner_configs': {'RRTConnectkConfigDefault': {'type': 'geometric::RRTConnect', 'range': 0.}},
                           'arm': {'planner_configs': ['RRTConnectkConfigDefault'],
                                   'longest_valid_segment_fraction': .01}}}
        parameter_path = directory/'mtc-parameters.yaml'
        parameter_path.write_text(json.dumps({'/**': {'ros__parameters': config}}))
        request = {'phase': phase, 'joints': feedback['joints'],
                   'jaw_positions': feedback['jaw_positions']}
        if above is not None:
            request['above'] = above
        path = directory/('mtc-'+phase+'.json')
        path.write_text(json.dumps(request))
        env = dict(os.environ, ROS_DOMAIN_ID='184', ROS_LOCALHOST_ONLY='1',
                   PYTHONPATH=str(ROOT)+os.pathsep+os.environ.get('PYTHONPATH', ''))
        command = ['bash', '-c', 'set -e; source /opt/ros/humble/setup.bash; exec "$1" "$2" --ros-args --params-file "$3"',
                   'go2-place-mtc', str(binary), str(path), str(parameter_path)]
        with (directory/('mtc-'+phase+'.log')).open('w') as log:
            result = subprocess.run(command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT,
                                    timeout=90, check=False)
        if result.returncode:
            raise RuntimeError('MoveIt Task Constructor '+phase+' failed; see '+str(directory/('mtc-'+phase+'.log')))
        return json.loads(path.with_name(path.stem+'-result.json').read_text())
    def execute_mtc(path, minimum_duration):
        # A completed stage may still be settling within its 0.03 rad tracking
        # tolerance. The next MTC stage starts at the exact planned waypoint,
        # while JointPath accepts only 0.02 rad from the measured start.
        start = path['points'][0]['positions']
        deadline = time.monotonic()+5
        while True:
            current = rpc('feedback')
            if max(abs(a-b) for a, b in zip(current['joints'], start)) <= .019:
                break
            if current['fault'] or time.monotonic() >= deadline:
                raise RuntimeError('Placement stage start did not settle to the checked MTC path')
            time.sleep(.08)
        duration = path['points'][-1]['sec']+path['points'][-1]['nanosec']/1e9
        if duration <= 0:
            raise RuntimeError('MTC returned an untimed placement stage')
        factor = max(1., minimum_duration/duration)
        timed = []
        for point in path['points']:
            seconds = (point['sec']+point['nanosec']/1e9)*factor
            whole = int(seconds)
            timed.append({'positions': point['positions'], 'sec': whole,
                          'nanosec': int((seconds-whole)*1e9)})
        rpc('trajectory', joint_names=path['joint_names'], points=timed)
        return wait_motion(90)
    support_samples = []
    def verify_visual_result(timeout=3.):
        deadline = time.monotonic()+timeout
        since = None
        reference = None
        last_error = None
        while time.monotonic() < deadline:
            if rpc('feedback')['fault']:
                raise RuntimeError('Placement stopped by simulator fault during visual verification')
            try:
                sample = placement_visual_result(rpc('observe', camera='wrist_rgbd'), options['surface'])
                support_samples.append(dict(sample, phase=phase))
                centroid = sample['visible_centroid_world_m']
                if reference is None or math.dist(centroid, reference) > .008:
                    since, reference = sample['stamp'], centroid
                elif sample['stamp']-since >= .4:
                    return sample
            except ValueError as error:
                last_error = str(error)
                since = None
                reference = None
            time.sleep(.12)
        raise RuntimeError(last_error or 'Released object did not settle in observed wrist depth')
    released = False
    placed = False
    def wait_control(timeout=90):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            feedback = rpc('feedback')
            control = feedback.get('placement_control') or {}
            if control.get('evidence'):
                support_samples.append(dict(control['evidence'], phase=phase))
            if feedback['fault']:
                raise RuntimeError(feedback['fault'])
            if control.get('state') == 'succeeded':
                return feedback
            if control.get('state') != 'running':
                raise RuntimeError(control.get('error') or 'Contact controller stopped')
            time.sleep(.08)
        rpc('abort')
        raise TimeoutError('Contact controller response timed out')
    try:
        feedback = rpc('feedback')
        if not feedback['holding'] or feedback['fault']:
            raise RuntimeError('No confirmed held object; place operation refused')
        if feedback.get('placement_control_version') != 4:
            raise RuntimeError('Restart preview service to enable wrist RGB-D placement')
        fresh = rpc('observe', camera='wrist_rgbd')
        options['surface'] = reobserve_placement(fresh, options['surface'])
        phase = 'plan'
        rpc('task_phase', phase='planning')
        plan = rpc('place_plan', point_world=options['surface']['point_world'],
                   bottom_offset_local=options['bottom_offset_local'], axis_local=options.get('axis_local'))
        (directory/'place-plan.json').write_text(json.dumps(plan, indent=2))
        paths = mtc_paths('before_release', feedback, plan['above'])
        phase = 'move_above'
        rpc('task_phase', phase='approaching')
        at_above = execute_mtc(paths['approach'], 5)
        # The wrist may no longer see the target underneath the held bottle.
        # Keep the measured world plane instead of reusing an old pixel or
        # replacing unseen geometry with the simulator's target position.
        try:
            observed = rpc('observe', camera='wrist_rgbd')
            options['surface'] = reobserve_placement(observed, options['surface'])
            report['approach_reobservation'] = 'fresh_wrist_depth'
        except PlacementNotVisible as error:
            report['approach_reobservation'] = 'last_measured_static_plane'
            report['occlusion_note'] = str(error)
        phase = 'descend'
        rpc('place_descend', point_world=options['surface']['point_world'],
            strategy='wrist_rgbd_geometry', bottom_offset_local=options['bottom_offset_local'],
            axis_local=options['axis_local'], orientation_world=plan['link6_rotation_world'],
            normal_world=options['surface']['normal_world'])
        at_bottom = wait_control()
        phase = 'open_gripper'
        rpc('place_release', point_world=options['surface']['point_world'])
        after_release = wait_control(30)
        released = True
        # Opening the gripper can shift the arm under contact. Plan the
        # retreat from the measured post-release joints, not the pre-release
        # pose at the end of the contact descent.
        phase = 'plan_retreat_and_zero'
        rpc('task_phase', phase='planning')
        post_paths = mtc_paths('after_release', after_release)
        phase = 'retreat'
        rpc('task_phase', phase='retreating')
        retreat = execute_mtc(post_paths['retreat'], 4)
        time.sleep(1.5)
        phase = 'return_to_zero'
        rpc('task_phase', phase='returning_zero')
        at_zero = execute_mtc(post_paths['zero'], 5)
        report['returned_to_zero'] = True
        phase = 'verify_visual_result'
        report['visual_result'] = verify_visual_result()
        placed = True
        phase = 'release_ownership'
        rpc('release')
        report.update(passed=True, released=True, target_world=options['surface']['point_world'],
                      returned_to_zero=True, final_joints=at_zero['joints'])
    except Exception as error:
        report['error'] = str(error)
        report['failed_phase'] = phase
        report['released'] = released
        report['placed'] = placed
        if phase == 'open_gripper':
            try:
                rpc('place_pause_opening')
            except Exception as pause_error:
                report['opening_pause_error'] = str(pause_error)
        try:
            feedback = rpc('feedback')
            report['failure_feedback'] = {key: feedback.get(key) for key in
                                          ('stamp', 'contact_force', 'gripper_width', 'holding',
                                           'drive_enabled', 'fault', 'joints', 'trajectory')}
        except Exception:
            pass
        try:
            rpc('abort')
        except Exception:
            pass
        if released:
            try:
                rpc('release')
                report['ownership_released'] = True
            except Exception as release_error:
                report['release_error'] = str(release_error)
    finally:
        (directory/'place-support.json').write_text(json.dumps(support_samples, indent=2))
        (directory/'place-motion.json').write_text(json.dumps(motion_samples, indent=2))
        (directory/'result.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    import sys
    request = json.loads(Path(sys.argv[1]).read_text())
    if request['operation'] == 'place':
        sys.exit(run_place(sys.argv[1]))
    else:
        run_pose(sys.argv[1])
