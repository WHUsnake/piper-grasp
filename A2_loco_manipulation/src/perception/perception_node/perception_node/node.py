"""Registered RGB-D to target cloud. No robot commands."""
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_msgs.msg import String
from grasp_msgs.msg import BoundTargetCloud
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from .geometry import masked_cloud
from .depth_target import tabletop_component
MODULE_LOADED_NS = time.monotonic_ns()

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        defaults = dict(weights='', device='cpu', confidence=0.55, target_classes=['bottle'],
                        segmentation_backend='yolo',table_height_world=0.0,target_x_world=0.0,target_y_world=0.0,
                        camera_rotation_world=[1.,0.,0.,0.,1.,0.,0.,0.,1.],camera_translation_world=[0.,0.,0.],
                        rgb_topic='/camera/color/image_raw', depth_topic='/camera/depth/image_raw',
                        camera_info_topic='/camera/color/camera_info',
                        simulation_mode=False,target_session='',capture_directory='',selected_pixel_u=-1,selected_pixel_v=-1,
                        enforce_selected_target_position=False,
                        min_valid_depth_ratio=0.15, min_depth_m=0.10, max_depth_m=2.50)
        for k,v in defaults.items(): self.declare_parameter(k,v)
        self.cfg = {k:self.get_parameter(k).value for k in defaults}
        self._timing_path = Path(self.cfg['capture_directory'])/'perception_timing.json' if self.cfg['capture_directory'] else None
        self._timing_events = {}
        self._first_latency_logged = False
        self.trace_once('module_loaded',MODULE_LOADED_NS)
        self.trace_once('node_initialized')
        if self.cfg['target_session'] and not self.cfg['simulation_mode']:raise ValueError('Synthetic single-target session is simulation-only')
        if self.cfg['segmentation_backend'] not in ('yolo','tabletop_depth_roi','tabletop_depth_components'):raise ValueError('Unsupported target segmentation backend')
        if self.cfg['segmentation_backend'] in ('tabletop_depth_roi','tabletop_depth_components') and not self.cfg['simulation_mode']:
            raise ValueError('Configured-table tabletop ROI is simulation-only')
        self.model=None
        if self.cfg['segmentation_backend']=='yolo':
            if not Path(self.cfg['weights']).is_file(): raise ValueError('Set local YOLO11s-seg weights')
            from ultralytics import YOLO
            self.model = YOLO(self.cfg['weights'])
            if self.model.task != 'segment': raise ValueError('Segmentation checkpoint required')
            if not set(self.cfg['target_classes']) <= set(self.model.names.values()): raise ValueError('Missing target class')
            self.model.predict(np.zeros((480,640,3),dtype=np.uint8),device=self.cfg['device'],verbose=False)
        self.mask_pub=self.create_publisher(Image,'/perception/target_mask',2)
        self.status_pub=self.create_publisher(String,'/perception/status',10)
        self.bound_pub=self.create_publisher(BoundTargetCloud,'/grasp_perception/target',2)
        self.sequence=0
        self.bridge = CvBridge()
        self.info = None
        self.cloud = self.create_publisher(PointCloud2,'/perception/target_cloud' if self.cfg['simulation_mode'] else '/perception/unbound_target_cloud',2)
        image_qos=QoSProfile(depth=4,reliability=ReliabilityPolicy.RELIABLE) if self.cfg['simulation_mode'] else qos_profile_sensor_data
        self.create_subscription(CameraInfo,self.cfg['camera_info_topic'],self.on_info,image_qos)
        self.rgb = Subscriber(self,Image,self.cfg['rgb_topic'],qos_profile=image_qos)
        self.depth = Subscriber(self,Image,self.cfg['depth_topic'],qos_profile=image_qos)
        self.rgb.registerCallback(lambda msg:self.trace_once('first_rgb_received'))
        self.depth.registerCallback(lambda msg:self.trace_once('first_depth_received'))
        self.sync = ApproximateTimeSynchronizer([self.rgb,self.depth],10,0.04)
        self.sync.registerCallback(self.callback)
        self.trace_once('subscriptions_ready')
    def trace_once(self,name,when_ns=None):
        if self._timing_path is None or name in self._timing_events:return
        self._timing_events[name]={'monotonic_ns':when_ns or time.monotonic_ns(),'wall_time_unix_sec':time.time()}
        self._timing_path.write_text(json.dumps(self._timing_events,indent=2))
    def on_info(self,msg):
        self.trace_once('first_camera_info_received');self.info=msg
    def invalidate(self,header,reason):
        self.trace_once('first_invalid_'+reason)
        self.cloud.publish(point_cloud2.create_cloud_xyz32(header,[]))
        self.status_pub.publish(String(data=reason))
        if self.cfg['target_session']:
            bound=BoundTargetCloud();bound.header=header;bound.valid=False;bound.reason=reason;bound.exporter_session=self.cfg['target_session'];self.bound_pub.publish(bound)
    def callback(self,rgb,depth):
        callback_started_ns=time.monotonic_ns()
        self.trace_once('first_rgbd_callback')
        try:
            info=self.info
            if info is None:
                self.trace_once('first_rgbd_without_camera_info');return
            stamp=rclpy.time.Time.from_msg(rgb.header.stamp)
            if not 0 <= (self.get_clock().now()-stamp).nanoseconds/1e9 <= 0.5:
                self.invalidate(rgb.header,'input_expired');return
            if not rgb.header.frame_id or rgb.header.frame_id != depth.header.frame_id or info.header.frame_id != rgb.header.frame_id: raise ValueError('Registered frames differ')
            if (rgb.width,rgb.height)!=(depth.width,depth.height) or (rgb.width,rgb.height)!=(info.width,info.height): raise ValueError('Registered sizes differ')
            image=self.bridge.imgmsg_to_cv2(rgb,'bgr8');depth_image=self.bridge.imgmsg_to_cv2(depth,'passthrough')
            if self.cfg['segmentation_backend']=='yolo':
                result=self.model.predict(image,conf=self.cfg['confidence'],device=self.cfg['device'],retina_masks=True,verbose=False)[0]
                if result.masks is None:
                    self.invalidate(rgb.header,'no_segmentation');return
                options=[(float(b.conf.item()),i) for i,b in enumerate(result.boxes) if result.names[int(b.cls.item())] in self.cfg['target_classes']]
                if not options:self.invalidate(rgb.header,'no_target');return
                _,i=max(options);mask=result.masks.data[i].cpu().numpy()>0.5
                class_id=int(result.boxes[i].cls.item());confidence=float(result.boxes[i].conf.item())
            elif self.cfg['segmentation_backend']=='tabletop_depth_components':
                if depth.encoding!='32FC1':raise ValueError('Tabletop depth components require metric 32FC1 depth')
                chosen=(self.cfg['selected_pixel_u'],self.cfg['selected_pixel_v']) if self.cfg['selected_pixel_u']>=0 and self.cfg['selected_pixel_v']>=0 else None
                mask,selection=tabletop_component(depth_image,info.k,self.cfg['camera_rotation_world'],
                                                   self.cfg['camera_translation_world'],self.cfg['table_height_world'],selected_pixel=chosen)
                if self.cfg['capture_directory']:
                    (Path(self.cfg['capture_directory'])/'detected_objects.json').write_text(json.dumps(selection,indent=2))
                if selection['selected_pixels']<100:
                    self.invalidate(rgb.header,selection.get('selection_error','no_tabletop_component'));return
                if self.cfg['enforce_selected_target_position']:
                    xy=np.asarray(selection['centroid_world_m'][:2])
                    requested=np.array([self.cfg['target_x_world'],self.cfg['target_y_world']])
                    if np.linalg.norm(xy-requested)>.085:
                        self.invalidate(rgb.header,'selected_other_object');return
                class_id=-1;confidence=None
            else:
                if depth.encoding!='32FC1':raise ValueError('Tabletop depth ROI requires metric 32FC1 depth')
                u,v=np.meshgrid(np.arange(depth.width),np.arange(depth.height));z=np.asarray(depth_image,float)
                local=np.stack([(u-info.k[2])*z/info.k[0],(v-info.k[5])*z/info.k[4],z],axis=-1)
                rotation=np.asarray(self.cfg['camera_rotation_world'],float).reshape(3,3)
                translation=np.asarray(self.cfg['camera_translation_world'],float)
                world=local@rotation.T+translation
                mask=(np.isfinite(z)&(z>.1)&(z<2.5)&
                      (world[:,:,2]>self.cfg['table_height_world']+.006)&
                      (world[:,:,2]<self.cfg['table_height_world']+.14)&
                      (np.abs(world[:,:,0]-self.cfg['target_x_world'])<.09)&
                      (np.abs(world[:,:,1]-self.cfg['target_y_world'])<.09))
                if np.count_nonzero(mask)<100:self.invalidate(rgb.header,'no_tabletop_target');return
                class_id=-1;confidence=None
            points=masked_cloud(depth_image,depth.encoding,mask,info.k,
                                minimum=self.cfg['min_depth_m'], maximum=self.cfg['max_depth_m'],
                                distortion=info.d, distortion_model=info.distortion_model,
                                min_valid_ratio=self.cfg['min_valid_depth_ratio'])
            if (self.get_clock().now()-stamp).nanoseconds/1e9 > 0.5:
                self.get_logger().warning('Target expired after inference (>0.5 s)', throttle_duration_sec=3.)
                self.invalidate(rgb.header,'inference_expired');return
            cloud=point_cloud2.create_cloud_xyz32(rgb.header,points)
            mask_msg=self.bridge.cv2_to_imgmsg(mask.astype(np.uint8)*255,encoding='mono8');mask_msg.header=rgb.header
            self.mask_pub.publish(mask_msg)
            if self.cfg['target_session']:
                self.sequence+=1;bound=BoundTargetCloud();bound.header=rgb.header;bound.cloud=cloud;bound.mask=mask_msg;bound.valid=True
                bound.exporter_session=self.cfg['target_session'];bound.tracker_session=self.cfg['target_session'];bound.track_id=1;bound.class_id=class_id;bound.sequence=self.sequence
                self.bound_pub.publish(bound)
            self.cloud.publish(cloud)
            self.trace_once('first_valid_cloud_published')
            if not self._first_latency_logged:
                acquisition_to_cloud=(self.get_clock().now()-stamp).nanoseconds/1e9
                processing=(time.monotonic_ns()-callback_started_ns)/1e9
                self.get_logger().info(f'First target cloud: acquisition_to_cloud={acquisition_to_cloud:.3f}s callback_processing={processing:.3f}s points={len(points)}')
                self._first_latency_logged=True
            self.status_pub.publish(String(data=self.cfg['segmentation_backend']+'_to_metric_cloud'))
            folder=Path(self.cfg['capture_directory']) if self.cfg['capture_directory'] else None
            if folder and not (folder/'perception_capture.npz').exists():
                folder.mkdir(parents=True,exist_ok=True)
                np.savez_compressed(folder/'perception_capture.npz',rgb=image,depth=depth_image,mask=mask,points_camera_m=points)
                import cv2
                cv2.imwrite(str(folder/'mask.png'),mask.astype(np.uint8)*255)
                (folder/'perception_provenance.json').write_text(json.dumps(dict(method=self.cfg['segmentation_backend'],weights=self.cfg['weights'] if self.model is not None else None,confidence=confidence,oracle_segmentation=False,uses_configured_target_location=self.cfg['segmentation_backend']=='tabletop_depth_roi',selection=selection if self.cfg['segmentation_backend']=='tabletop_depth_components' else None,point_count=len(points),frame=rgb.header.frame_id,stamp=dict(sec=rgb.header.stamp.sec,nanosec=rgb.header.stamp.nanosec))))
        except ValueError as exc:
            self.invalidate(rgb.header,str(exc));self.get_logger().warning(str(exc),throttle_duration_sec=2.)

def main(args=None):
    rclpy.init(args=args)
    node=PerceptionNode()
    try: rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
