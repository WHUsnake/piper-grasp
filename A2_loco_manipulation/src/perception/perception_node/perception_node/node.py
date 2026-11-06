"""Registered RGB-D to target cloud. No robot commands."""
from pathlib import Path
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_msgs.msg import String
from grasp_msgs.msg import BoundTargetCloud
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from .geometry import masked_cloud

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        defaults = dict(weights='', device='cpu', confidence=0.55, target_classes=['bottle'],
                        rgb_topic='/camera/color/image_raw', depth_topic='/camera/depth/image_raw',
                        camera_info_topic='/camera/color/camera_info',
                        simulation_mode=False,target_session='',capture_directory='',min_valid_depth_ratio=0.15, min_depth_m=0.10, max_depth_m=2.50)
        for k,v in defaults.items(): self.declare_parameter(k,v)
        self.cfg = {k:self.get_parameter(k).value for k in defaults}
        if self.cfg['target_session'] and not self.cfg['simulation_mode']:raise ValueError('Synthetic single-target session is simulation-only')
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
        self.create_subscription(CameraInfo,self.cfg['camera_info_topic'],self.on_info,qos_profile_sensor_data)
        self.rgb = Subscriber(self,Image,self.cfg['rgb_topic'],qos_profile=qos_profile_sensor_data)
        self.depth = Subscriber(self,Image,self.cfg['depth_topic'],qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer([self.rgb,self.depth],10,0.04)
        self.sync.registerCallback(self.callback)
    def on_info(self,msg): self.info=msg
    def invalidate(self,header,reason):
        self.cloud.publish(point_cloud2.create_cloud_xyz32(header,[]))
        self.status_pub.publish(String(data=reason))
        if self.cfg['target_session']:
            bound=BoundTargetCloud();bound.header=header;bound.valid=False;bound.reason=reason;bound.exporter_session=self.cfg['target_session'];self.bound_pub.publish(bound)
    def callback(self,rgb,depth):
        try:
            info=self.info
            if info is None: return
            stamp=rclpy.time.Time.from_msg(rgb.header.stamp)
            if not 0 <= (self.get_clock().now()-stamp).nanoseconds/1e9 <= 0.5:
                self.invalidate(rgb.header,'input_expired');return
            if not rgb.header.frame_id or rgb.header.frame_id != depth.header.frame_id or info.header.frame_id != rgb.header.frame_id: raise ValueError('Registered frames differ')
            if (rgb.width,rgb.height)!=(depth.width,depth.height) or (rgb.width,rgb.height)!=(info.width,info.height): raise ValueError('Registered sizes differ')
            image=self.bridge.imgmsg_to_cv2(rgb,'bgr8')
            result=self.model.predict(image,conf=self.cfg['confidence'],device=self.cfg['device'],retina_masks=True,verbose=False)[0]
            if result.masks is None:
                self.get_logger().info('No segmented object detected', throttle_duration_sec=3.)
                self.invalidate(rgb.header,'no_segmentation');return
            options=[(float(b.conf.item()),i) for i,b in enumerate(result.boxes) if result.names[int(b.cls.item())] in self.cfg['target_classes']]
            if not options:
                self.get_logger().info('No target bottle detected', throttle_duration_sec=3.)
                self.invalidate(rgb.header,'no_target');return
            _,i=max(options)
            mask=result.masks.data[i].cpu().numpy()>0.5
            points=masked_cloud(self.bridge.imgmsg_to_cv2(depth,'passthrough'),depth.encoding,mask,info.k,
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
                bound.exporter_session=self.cfg['target_session'];bound.tracker_session=self.cfg['target_session'];bound.track_id=1;bound.class_id=39;bound.sequence=self.sequence
                self.bound_pub.publish(bound)
            self.cloud.publish(cloud)
            self.status_pub.publish(String(data='YOLO_mask_to_metric_cloud'))
            folder=Path(self.cfg['capture_directory']) if self.cfg['capture_directory'] else None
            if folder and not (folder/'perception_capture.npz').exists():
                folder.mkdir(parents=True,exist_ok=True)
                np.savez_compressed(folder/'perception_capture.npz',rgb=image,depth=self.bridge.imgmsg_to_cv2(depth,'passthrough'),mask=mask,points_camera_m=points)
                import cv2,json
                cv2.imwrite(str(folder/'mask.png'),mask.astype(np.uint8)*255)
                (folder/'perception_provenance.json').write_text(json.dumps(dict(method='YOLO11s-seg',weights=self.cfg['weights'],confidence=float(result.boxes[i].conf.item()),oracle_segmentation=False,point_count=len(points),frame=rgb.header.frame_id,stamp=dict(sec=rgb.header.stamp.sec,nanosec=rgb.header.stamp.nanosec))))
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
