#!/usr/bin/python3
"""Read A2 ROS camera over SSH and display locally. No web or motion commands."""
import shlex
import struct
import subprocess
import threading
import time
import cv2
import numpy as np

REMOTE = '''import sys,struct,time
import rclpy
from sensor_msgs.msg import CompressedImage
from rclpy.qos import qos_profile_sensor_data
rclpy.init()
n=rclpy.create_node('local_camera_view_stream')
last=[0.]
def frame(m):
 now=time.monotonic()
 if now-last[0]<.1:return
 last[0]=now
 b=bytes(m.data)
 try:
  sys.stdout.buffer.write(struct.pack('!I',len(b))+b)
  sys.stdout.buffer.flush()
 except BrokenPipeError:
  raise SystemExit(0)
n.create_subscription(CompressedImage,'/camera/color/image_raw/compressed',frame,qos_profile_sensor_data)
try:rclpy.spin(n)
finally:
 n.destroy_node()
 if rclpy.ok():rclpy.shutdown()
'''
command = 'source /home/unitree/A2_loco_manipulation/activate-ros2.sh; exec python -u -c ' + shlex.quote(REMOTE)
proc = subprocess.Popen(['ssh','-T','-o','BatchMode=yes','-o','ConnectTimeout=5','-o','ServerAliveInterval=5','unitree@192.168.31.22','bash -c '+shlex.quote(command)],stdout=subprocess.PIPE)
latest=[None,0.]; error=['']
def read_exact(n):
 data=bytearray()
 while len(data)<n:
  chunk=proc.stdout.read(n-len(data))
  if not chunk:raise EOFError('Camera connection closed')
  data.extend(chunk)
 return bytes(data)
def receive():
 try:
  while True:
   size=struct.unpack('!I',read_exact(4))[0]
   if not 0<size<10000000:raise ValueError('Invalid image length')
   im=cv2.imdecode(np.frombuffer(read_exact(size),np.uint8),cv2.IMREAD_COLOR)
   if im is not None:latest[:]=[im,time.monotonic()]
 except Exception as exc:error[0]=str(exc)
threading.Thread(target=receive,daemon=True).start()
name='A2 Camera - current live view (Q / Esc to close)'
cv2.namedWindow(name,cv2.WINDOW_NORMAL)
cv2.resizeWindow(name,960,720)
reported=False
opened=time.monotonic()
try:
 while True:
  frame,at=latest
  canvas=np.zeros((480,640,3),np.uint8) if frame is None else frame.copy()
  age=time.monotonic()-at if at else None
  label=error[0] or ('Connecting to A2...' if age is None else ('LIVE' if age<2 else 'STALE - no fresh camera frames'))
  cv2.putText(canvas,label,(12,28),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,220,0) if age is not None and age<2 else (0,160,255),2)
  cv2.imshow(name,canvas)
  if frame is not None and not reported:
   print('LIVE_FRAME_RECEIVED '+str(frame.shape),flush=True);reported=True
  key=cv2.waitKey(30)&255
  if key in (27,ord('q')) or (time.monotonic()-opened>2 and cv2.getWindowProperty(name,cv2.WND_PROP_VISIBLE)==0):break
finally:
 proc.terminate()
 try:proc.wait(timeout=3)
 except subprocess.TimeoutExpired:proc.kill();proc.wait()
 cv2.destroyAllWindows()
