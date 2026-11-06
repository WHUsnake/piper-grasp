#!/usr/bin/env python3
"""Isolated-domain transport smoke; synthetic TF is NOT calibration evidence."""
import json
import os
import time
from pathlib import Path
os.environ['ROS_DOMAIN_ID']='177'
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from grasp_msgs.msg import GraspCandidateArray
from grasp_generation.node import GraspGenerator
from scipy.spatial.transform import Rotation

root=Path(__file__).resolve().parents[2]
workspace=root.parent
# Parse archived PCD (not RGB-derived pseudo depth).
path=workspace/'search-grasp2/runs/2026-10-14/c6295667e186448eaf14d59d7dd2cd93/observations/1791978191345435904/target_cloud.pcd'
lines=path.read_text().splitlines()
i=next(i for i,l in enumerate(lines) if l.startswith('DATA ascii'))
points=np.loadtxt(lines[i+1:])[:,:3].astype('float32')
report=json.loads((root/'data/graspnet/replay.json').read_text())
c=report['records'][0]['candidates'][0]
# Choose a synthetic camera rotation making one recorded proposal horizontal.
r=Rotation.from_quat(c['quaternion']).as_matrix()
axes=np.array([[0,0,1],[0,1,0],[-1,0,0]])
rb=axes@r.T
rclpy.init()
n=GraspGenerator(); probe=rclpy.create_node('graspnet_transport_test')
messages=[]
sub=probe.create_subscription(GraspCandidateArray,'/grasp/candidates',messages.append,10)
try:
    tf=TransformStamped(); tf.header.frame_id='base_link'; tf.child_frame_id='camera_test'
    q=Rotation.from_matrix(rb).as_quat()
    tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=map(float,q)
    n.buffer.set_transform_static(tf,'smoke_synthetic_only')
    deadline=time.monotonic()+.3
    while time.monotonic()<deadline:rclpy.spin_once(probe,timeout_sec=.02)
    header=Header(); header.frame_id='camera_test'; header.stamp=n.get_clock().now().to_msg()
    n.callback(create_cloud_xyz32(header,points))
    deadline=time.monotonic()+2
    while time.monotonic()<deadline:
        rclpy.spin_once(n,timeout_sec=.005); rclpy.spin_once(probe,timeout_sec=.005)
    nonempty=[m for m in messages if m.candidates]
    assert nonempty, 'No nonempty candidate message received'
    m=nonempty[0]
    assert m.header==header,'Acquisition header changed'
    assert messages[-1].candidates==[], 'Cached candidate did not expire'
    assert all(c.generator=='graspnet_baseline_preview' and abs(c.pregrasp_distance_m-.1)<1e-6 for c in m.candidates)
    out=dict(status='PASS',domain=177,synthetic_tf=True,physical_horizontal_verified=False,
             input_points=len(points),candidate_count=len(m.candidates),header_preserved=True,
             expiration_clears_candidates=True,execute=False)
    (root/'data/graspnet/ros_smoke.json').write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps(out))
finally:
    n.close(); n.destroy_node(); probe.destroy_node(); rclpy.shutdown()
