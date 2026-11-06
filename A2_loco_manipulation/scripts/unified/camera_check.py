"""Read-only compatibility probe for an externally owned Orbbec RGB-D camera."""
import json,time
import rclpy
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import Image,CameraInfo
from rclpy.qos import qos_profile_sensor_data

def main():
    rclpy.init();n=rclpy.create_node('a2_unified_camera_check');seen={};counts={}
    topics={'/camera/color/image_raw':Image,'/camera/depth/image_raw':Image,'/camera/color/camera_info':CameraInfo}
    def receive(t,m):
        stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        age=n.get_clock().now().nanoseconds/1e9-stamp
        if 0<=age<=.5 and (t not in seen or stamp>seen[t][2]):
            seen[t]=(time.monotonic(),m,stamp);counts[t]=counts.get(t,0)+1
    subs=[n.create_subscription(typ,t,lambda m,t=t:receive(t,m),qos_profile_sensor_data) for t,typ in topics.items()]
    try:
        c=n.create_client(GetParameters,'/camera/camera/get_parameters')
        if not c.wait_for_service(timeout_sec=5):raise RuntimeError('Camera parameters unavailable')
        names=['depth_registration','enable_color','enable_depth','publish_tf','enable_depth_scale']
        f=c.call_async(GetParameters.Request(names=names));rclpy.spin_until_future_complete(n,f,timeout_sec=5)
        if not f.done() or f.result() is None:raise RuntimeError('Camera parameter timeout')
        params={name:v.bool_value if v.type==1 else None for name,v in zip(names,f.result().values)}
        if any(params.get(name) is not True for name in names):raise RuntimeError('Required camera parameters differ: '+str(params))
        end=time.monotonic()+8
        while time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.1)
            if all(counts.get(t,0)>=5 and time.monotonic()-seen[t][0]<.5 for t in topics):break
        else:raise RuntimeError('Fresh RGB/depth/calibration stream unavailable')
        rgb,depth,info=[seen[t][1] for t in topics]
        if (rgb.width,rgb.height)!=(640,480) or (depth.width,depth.height)!=(rgb.width,rgb.height) or (info.width,info.height)!=(rgb.width,rgb.height):raise RuntimeError('Expected aligned 640x480 RGB-D/calibration')
        if depth.encoding not in ('16UC1','32FC1'):raise RuntimeError('Unsupported depth encoding')
        if len({m.header.frame_id for m in (rgb,depth,info)})!=1 or rgb.header.frame_id!='camera_color_optical_frame':raise RuntimeError('RGB-D optical frames differ')
        if info.k[0]<=0 or info.k[4]<=0:raise RuntimeError('Invalid camera intrinsics')
        for topic in topics:
            publishers=n.get_publishers_info_by_topic(topic)
            if len(publishers)!=1 or publishers[0].node_name!='camera' or publishers[0].node_namespace!='/camera':raise RuntimeError('Ambiguous camera publishers: '+topic)
        if ('local_camera_view_stream','/') in n.get_node_names_and_namespaces():
            pubs=n.get_publisher_names_and_types_by_node('local_camera_view_stream','/')
            if any(t not in ('/rosout','/parameter_events') for t,_ in pubs):raise RuntimeError('Preview node has unexpected publishers')
        print(json.dumps({'compatible':True,'ownership':'external','parameters':params,'size':[rgb.width,rgb.height],'depth_encoding':depth.encoding,'frame':rgb.header.frame_id}))
    except Exception as exc:
        print(json.dumps({'compatible':False,'reason':str(exc)}));return 2
    finally:n.destroy_node();rclpy.shutdown()
    return 0
if __name__=='__main__':raise SystemExit(main())
