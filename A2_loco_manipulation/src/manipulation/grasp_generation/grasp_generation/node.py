"""Target-only GraspNet proposals; acquisition headers and execution gates retained."""
import json
import os
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from grasp_msgs.msg import GraspCandidate, GraspCandidateArray, BoundTargetCloud
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener
from ament_index_python.packages import get_package_share_directory
from .geometry import candidates
from .refinement import refine_candidates,refine_near_sphere_candidates,refine_box_candidates,parallel_gripper_symmetries,upright_orientation_search
from .graspnet_backend import adapt
from .worker_client import Worker

class GraspGenerator(Node):
    def __init__(self):
        super().__init__('grasp_generator')
        self.declare_parameter('backend', 'graspnet')
        self.declare_parameter('graspnet_config', str(Path(get_package_share_directory('grasp_generation'))/'config/graspnet.json'))
        self.declare_parameter('worker_python', '')
        self.declare_parameter('max_age_sec', .5)
        self.backend = self.get_parameter('backend').value
        if self.backend not in ('graspnet', 'pca'): raise ValueError('Unknown grasp backend')
        self.pub = self.create_publisher(GraspCandidateArray, '/grasp/candidates', 2)
        self.status = self.create_publisher(String, '/grasp/diagnostics', 10)
        self.buffer = Buffer(); self.listener = TransformListener(self.buffer, self)
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.future = None; self.pending = None; self.worker = None
        self.identity = None; self.epoch = 0; self.published_header = None
        self.cfg = {}
        if self.backend == 'graspnet':
            config = self.get_parameter('graspnet_config').value
            self.cfg = json.loads(Path(config).read_text())
            python = self.get_parameter('worker_python').value or os.path.expandvars(self.cfg.get('worker_python', ''))
            if not python or python.startswith('${'):
                python = os.environ.get('GRASPNET_PYTHON', sys.executable)
            # Load once, outside ROS callbacks. No candidate until ready.
            self.worker = Worker(python, Path(__file__).with_name('worker.py'), config)
        self.create_subscription(BoundTargetCloud, '/grasp_perception/target', self.bound, 2)
        self.create_subscription(PointCloud2, '/perception/target_cloud', self.callback, 1)
        self.create_timer(.02, self.poll)

    def diagnostic(self, **data):
        self.status.publish(String(data=json.dumps(data, allow_nan=False, default=lambda v: v.tolist() if isinstance(v, np.ndarray) else float(v))))

    def clear(self, header, reason):
        self.published_header = None
        msg = GraspCandidateArray(); msg.header = header; self.pub.publish(msg)
        self.diagnostic(status=reason, backend=self.backend)

    def bound(self, msg):
        identity = (msg.exporter_session, msg.tracker_session, msg.track_id, msg.class_id)
        if not msg.valid or identity != self.identity:
            self.epoch += 1; self.pending = None
            self.clear(msg.header, 'target_invalidated')
        self.identity = identity

    def age(self, header):
        return (self.get_clock().now()-rclpy.time.Time.from_msg(header.stamp)).nanoseconds/1e9

    def callback(self, cloud):
        try:
            if not cloud.header.frame_id or not 0 <= self.age(cloud.header) <= self.get_parameter('max_age_sec').value:
                self.pending = None; self.epoch += 1; self.clear(cloud.header, 'stale_or_invalid_input'); return
            points = np.array(list(point_cloud2.read_points(cloud, field_names=('x','y','z'), skip_nans=True)))
            if points.dtype.names: points = np.column_stack([points[n] for n in ('x','y','z')])
            points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
            points = points[np.isfinite(points).all(axis=1)]
            if len(points) < 30: raise ValueError('Insufficient finite target points')
            rb = np.eye(3)
            if self.backend == 'graspnet':
                tf = self.buffer.lookup_transform(self.cfg['base_frame'], cloud.header.frame_id,
                                                  rclpy.time.Time.from_msg(cloud.header.stamp))
                q = tf.transform.rotation
                rb = Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
            translation=np.zeros(3)
            if self.backend=='graspnet':
                t=tf.transform.translation;translation=np.array([t.x,t.y,t.z])
            self.pending = (cloud.header, points, rb, self.epoch, translation)
        except Exception as exc:
            self.pending = None; self.epoch += 1
            self.clear(cloud.header, str(exc))

    def poll(self):
        if self.published_header is not None and not 0 <= self.age(self.published_header) <= self.get_parameter('max_age_sec').value:
            self.clear(self.published_header, 'published_candidates_expired')
        if self.future is not None and self.future.done():
            header, rb, epoch = self.active
            try:
                reply = self.future.result()
                if epoch != self.epoch:
                    self.clear(header, 'superseded_target')
                elif not 0 <= self.age(header) <= self.get_parameter('max_age_sec').value:
                    self.clear(header, 'inference_output_expired')
                else:
                    fit=None
                    if self.backend == 'graspnet':
                        proposals, counts = adapt(reply['rows'], rb, self.cfg['adapter'])
                        refinement=self.cfg.get('refinement',{})
                        if refinement.get('enabled',False):
                            points,translation=self.active_geometry
                            world=points@rb.T+translation
                            for c in proposals:
                                c['position']=(rb@c['position']+translation).tolist()
                                c['rotation_world']=(rb@Rotation.from_quat(c['quaternion']).as_matrix()).tolist()
                                c['quaternion']=Rotation.from_matrix(c['rotation_world']).as_quat().tolist()
                            shape=refinement.get('shape','upright_cylinder')
                            if shape=='upright_cylinder':
                                proposals,fit=refine_candidates(proposals,world,refinement['tcp_m'])
                                if refinement.get('orientation_search',False):proposals=upright_orientation_search(proposals,refinement['tcp_m'],refinement.get('approach_origin_world'),refinement.get('contact_height_fractions',[.5]))
                            elif shape=='near_sphere':
                                proposals,fit=refine_near_sphere_candidates(proposals,world,refinement['tcp_m'],self.cfg['adapter']['maximum_width_m'],refinement.get('support_surface_z'))
                                if refinement.get('orientation_search',False):
                                    proposals=upright_orientation_search(proposals,refinement['tcp_m'],refinement.get('approach_origin_world'))
                                    for c in proposals:
                                        if c.get('orientation_adaptation')=='upright_cylinder_network_and_radial_search':
                                            c['orientation_adaptation']='near_sphere_network_and_radial_search'
                            elif shape=='box':
                                proposals,fit=refine_box_candidates(proposals,world,refinement['tcp_m'],refinement['size_m'],refinement['support_surface_z'],self.cfg['adapter']['maximum_width_m'])
                                if refinement.get('orientation_search',False):
                                    proposals=upright_orientation_search(proposals,refinement['tcp_m'],refinement.get('approach_origin_world'))
                                    for c in proposals:
                                        if c.get('orientation_adaptation')=='upright_cylinder_network_and_radial_search':c['orientation_adaptation']='box_network_and_radial_search'
                            else:raise ValueError('Unsupported target geometry: '+str(shape))
                            proposals=parallel_gripper_symmetries(proposals)[:self.cfg['adapter']['max_candidates']]
                            for c in proposals:
                                c['position']=(rb.T@(np.array(c['position'])-translation)).tolist()
                                c['quaternion']=Rotation.from_matrix(rb.T@np.asarray(c['rotation_world'])).as_quat().tolist()
                            counts['refinement_applied']=len(proposals)

                    else: proposals, counts = reply['proposals'], {}
                    result = GraspCandidateArray(); result.header = header
                    for c in proposals:
                        m = GraspCandidate()
                        m.pose.position.x,m.pose.position.y,m.pose.position.z = map(float,c['position'])
                        m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w = map(float,c['quaternion'])
                        m.gripper_width_m = float(c['width']); m.score = float(c['score'])
                        m.generator = 'graspnet_baseline_preview' if self.backend == 'graspnet' else 'visible_cloud_pca_obb'
                        m.pregrasp_distance_m = float(c.get('pregrasp_distance', .10)); m.lift_distance_m = .12
                        result.candidates.append(m)
                    self.pub.publish(result)
                    self.published_header = header if proposals else None
                    self.diagnostic(status='ready', backend=self.backend, count=len(proposals), rejected=counts,
                                    inference_sec=reply.get('inference_sec'), candidates=proposals,
                                    tcp_geometry_verified=self.cfg.get('tcp_geometry_verified', False),geometry_fit=fit,geometry_frame=self.cfg.get('base_frame'),acquisition_stamp=[header.stamp.sec,header.stamp.nanosec])
            except Exception as exc: self.clear(header, str(exc))
            self.future = None
        if self.future is None and self.pending is not None:
            header, points, rb, epoch, translation = self.pending; self.pending = None
            if not 0 <= self.age(header) <= self.get_parameter('max_age_sec').value:
                self.clear(header, 'queued_input_expired'); return
            self.active = (header, rb, epoch)
            self.active_geometry=(points,translation)
            self.future = self.pool.submit(self.worker.predict, points) if self.worker else self.pool.submit(lambda: dict(proposals=candidates(points)))

    def close(self):
        self.pool.shutdown(wait=True, cancel_futures=True)
        if self.worker: self.worker.close()


def main(args=None):
    rclpy.init(args=args); node = None
    try:
        node = GraspGenerator(); rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        if node: node.close(); node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
