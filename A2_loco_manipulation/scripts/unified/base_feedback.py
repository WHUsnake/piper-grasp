"""Read-only ROS1 11311 feedback adapter. No publishers or motion services."""
import math, sys, time, threading
import rospy
from nav_msgs.msg import Odometry
from common import serve
sample={};lock=threading.Lock()
def receive(m):
    now=time.monotonic();stamp=m.header.stamp.to_sec()
    v=m.twist.twist
    with lock:sample.update(received=now,stamp=stamp,age=rospy.Time.now().to_sec()-stamp,
        linear=[v.linear.x,v.linear.y,v.linear.z],angular=[v.angular.x,v.angular.y,v.angular.z])
def dispatch(q):
    if q['method']!='feedback':raise ValueError('Read-only feedback endpoint')
    with lock:return dict(sample)
if __name__=='__main__':
    rospy.init_node('a2_unified_base_feedback',disable_signals=True)
    rospy.Subscriber('/a2/odom',Odometry,receive,queue_size=1)
    serve(sys.argv[1],dispatch).serve_forever()
